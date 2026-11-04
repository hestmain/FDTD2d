/*
 * source.c - spatial discretisation of sources onto the staggered grid.
 *
 * See source.h for the contracts. Everything here is pure "shape -> cell
 * weights" bookkeeping; the time dependence stays in the Signal.
 */
#include "source.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Bilinear weight below which a point-source contribution is dropped. */
#define SOURCE_MIN_WEIGHT 1e-12

/* Position of a physical coordinate along one axis in units of cell centres
 * (cell i has position i), clamped to the PHYSICAL cell centres
 * [off, off + n_phys - 1]. Sources therefore never put weight into PML
 * cells, so their strength does not depend on the PML thickness. */
static double axis_position(double x, double x0, double h, int off, int n_phys)
{
    const double lo = (double)off;
    const double hi = (double)(off + n_phys - 1);
    const double f = (x - x0) / h - 0.5 + (double)off;
    if (!(f > lo)) {
        return lo; /* also maps NaN to lo */
    }
    if (f > hi) {
        return hi;
    }
    return f;
}

/* Lower neighbour index and interpolation fraction along one axis. With
 * n_phys >= 2 the index is limited to off + n_phys - 2 so that i+1 stays a
 * physical cell; the resulting fraction is always in [0, 1]. */
static int axis_bracket(double f, int off, int n_phys, double *t)
{
    int i0 = (int)floor(f);
    if (n_phys >= 2 && i0 > off + n_phys - 2) {
        i0 = off + n_phys - 2;
    }
    if (i0 < off) {
        i0 = off;
    }
    *t = f - (double)i0;
    return i0;
}

/* Clamp a floored cell coordinate into [lo, hi]. */
static int clamp_cell_index(double f, int lo, int hi)
{
    if (!(f >= (double)lo)) {
        return lo; /* also maps NaN to lo */
    }
    if (f > (double)hi) {
        return hi;
    }
    return (int)f;
}

/* Growable (cell, length) list used by the segment discretisation. */
typedef struct CellList {
    size_t *cells;
    double *lens;
    size_t n, cap;
} CellList;

static int cell_list_push(CellList *l, size_t cell, double len, FdtdError *err)
{
    if (l->n == l->cap) {
        const size_t cap = l->cap ? 2 * l->cap : 64;
        size_t *c = realloc(l->cells, cap * sizeof *c);
        if (!c) {
            fdtd_error_set(err, "out of memory discretising segment source");
            return -1;
        }
        l->cells = c;
        double *d = realloc(l->lens, cap * sizeof *d);
        if (!d) {
            fdtd_error_set(err, "out of memory discretising segment source");
            return -1;
        }
        l->lens = d;
        l->cap = cap;
    }
    l->cells[l->n] = cell;
    l->lens[l->n] = len;
    l->n++;
    return 0;
}

static void cell_list_free(CellList *l)
{
    free(l->cells);
    free(l->lens);
    memset(l, 0, sizeof *l);
}

/* Copy the signal and shape type into the result (shared by all shapes). */
static void set_metadata(ModelSource *out, const Shape *shape, const Signal *signal,
                         size_t n_cells)
{
    out->shape_type = shape->type;
    out->signal = *signal;
    out->n_cells = n_cells;
}

static int discretize_point(const Shape *shape, const Signal *signal, const Grid *g,
                            const uint8_t *solid, ModelSource *out, FdtdError *err)
{
    const Vec2 p = shape->u.point.position;
    const double x1 = grid_x1(g);
    const double y1 = grid_y1(g);
    if (!(p.x >= g->x0 && p.x <= x1 && p.y >= g->y0 && p.y <= y1)) {
        fdtd_error_set(err, "point (%g, %g) is outside the physical domain "
                            "[%g, %g] x [%g, %g]", p.x, p.y, g->x0, x1, g->y0, y1);
        return -1;
    }

    /* A source buried in a rigid obstacle cannot radiate: reject it instead
     * of silently moving the weight onto the surrounding fluid cells. */
    if (solid) {
        /* Containing physical cell (a point on the upper domain edge belongs
         * to the last physical cell, not to the PML beyond it). */
        const int ci = clamp_cell_index(floor((p.x - g->x0) / g->dx) + (double)g->off_x,
                                        g->off_x, g->off_x + g->nx_phys - 1);
        const int cj = clamp_cell_index(floor((p.y - g->y0) / g->dy) + (double)g->off_y,
                                        g->off_y, g->off_y + g->ny_phys - 1);
        if (solid[grid_cell_index(g, ci, cj)]) {
            fdtd_error_set(err, "point (%g, %g) lies inside a rigid obstacle", p.x, p.y);
            return -1;
        }
    }

    const double fx = axis_position(p.x, g->x0, g->dx, g->off_x, g->nx_phys);
    const double fy = axis_position(p.y, g->y0, g->dy, g->off_y, g->ny_phys);
    double tx, ty;
    const int i0 = axis_bracket(fx, g->off_x, g->nx_phys, &tx);
    const int j0 = axis_bracket(fy, g->off_y, g->ny_phys, &ty);

    const int ni = (i0 + 1 < g->off_x + g->nx_phys) ? 2 : 1;
    const int nj = (j0 + 1 < g->off_y + g->ny_phys) ? 2 : 1;
    const double wx[2] = {1.0 - tx, tx};
    const double wy[2] = {1.0 - ty, ty};

    size_t cell[4];
    double weight[4];
    size_t n = 0;
    for (int b = 0; b < nj; b++) {
        for (int a = 0; a < ni; a++) {
            const double w = wx[a] * wy[b];
            if (!(w >= SOURCE_MIN_WEIGHT)) {
                continue;
            }
            const size_t c = grid_cell_index(g, i0 + a, j0 + b);
            if (solid && solid[c]) {
                continue;
            }
            cell[n] = c;
            weight[n] = w;
            n++;
        }
    }
    if (n == 0) {
        fdtd_error_set(err, "point (%g, %g) touches rigid cells only", p.x, p.y);
        return -1;
    }

    /* Renormalise the surviving weights to sum 1, then convert to a
     * density [1/m^2] (sum_c w_c dx dy = 1). */
    double sum = 0.0;
    for (size_t k = 0; k < n; k++) {
        sum += weight[k];
    }
    const double scale = 1.0 / (sum * g->dx * g->dy);

    out->cells = fdtd_calloc(n, sizeof *out->cells, err);
    if (!out->cells) {
        return -1;
    }
    out->weights = fdtd_calloc(n, sizeof *out->weights, err);
    if (!out->weights) {
        return -1;
    }
    for (size_t k = 0; k < n; k++) {
        out->cells[k] = cell[k];
        out->weights[k] = weight[k] * scale;
    }
    set_metadata(out, shape, signal, n);
    return 0;
}

static int discretize_segment(const Shape *shape, const Signal *signal, const Grid *g,
                              const uint8_t *solid, ModelSource *out, FdtdError *err)
{
    const Vec2 a = shape->u.segment.start;
    const Vec2 b = shape->u.segment.end;
    const double orig = hypot(b.x - a.x, b.y - a.y);
    if (!(orig > 0.0)) {
        fdtd_error_set(err, "segment has zero length");
        return -1;
    }

    const Vec2 bmin = {g->x0, g->y0};
    const Vec2 bmax = {grid_x1(g), grid_y1(g)};
    Vec2 ca = a;
    Vec2 cb = b;
    if (!geom_clip_segment(&ca, &cb, bmin, bmax)) {
        fdtd_error_set(err, "segment (%g, %g)-(%g, %g) does not cross the physical "
                            "domain [%g, %g] x [%g, %g]",
                       a.x, a.y, b.x, b.y, bmin.x, bmax.x, bmin.y, bmax.y);
        return -1;
    }
    const double clipped = hypot(cb.x - ca.x, cb.y - ca.y);
    if (!(clipped > 0.0)) {
        fdtd_error_set(err, "segment (%g, %g)-(%g, %g) touches the physical domain "
                            "in a single point only", a.x, a.y, b.x, b.y);
        return -1;
    }
    if (clipped < orig * (1.0 - 1e-9)) {
        fdtd_warn("segment source clipped to the physical domain: length %g m -> %g m",
                  orig, clipped);
    }

    /* Walk the PHYSICAL cell columns covered by the clipped segment, expanded
     * by one so that segments on a cell boundary are captured on both sides
     * (PML cells are never used). In each column only the rows the segment
     * passes through are tested, so the cost is O(length / cell size), not
     * O(bounding-box area). */
    const int pi_lo = g->off_x, pi_hi = g->off_x + g->nx_phys - 1;
    const int pj_lo = g->off_y, pj_hi = g->off_y + g->ny_phys - 1;
    const int i_lo = clamp_cell_index(floor((fmin(ca.x, cb.x) - g->x0) / g->dx) +
                                      (double)g->off_x - 1.0, pi_lo, pi_hi);
    const int i_hi = clamp_cell_index(floor((fmax(ca.x, cb.x) - g->x0) / g->dx) +
                                      (double)g->off_x + 1.0, pi_lo, pi_hi);

    CellList list = {0};
    double total = 0.0;
    for (int i = i_lo; i <= i_hi; i++) {
        /* Part of the segment inside this column's closed x-slab. */
        Vec2 sa = ca, sb = cb;
        const Vec2 slab_min = {grid_face_x(g, i), bmin.y};
        const Vec2 slab_max = {grid_face_x(g, i + 1), bmax.y};
        if (!geom_clip_segment(&sa, &sb, slab_min, slab_max)) {
            continue;
        }
        const int j_lo = clamp_cell_index(floor((fmin(sa.y, sb.y) - g->y0) / g->dy) +
                                          (double)g->off_y - 1.0, pj_lo, pj_hi);
        const int j_hi = clamp_cell_index(floor((fmax(sa.y, sb.y) - g->y0) / g->dy) +
                                          (double)g->off_y + 1.0, pj_lo, pj_hi);
        for (int j = j_lo; j <= j_hi; j++) {
            const Vec2 cmin = {grid_face_x(g, i), grid_face_y(g, j)};
            const Vec2 cmax = {grid_face_x(g, i + 1), grid_face_y(g, j + 1)};
            const double len = geom_segment_length_in_box(ca, cb, cmin, cmax);
            if (!(len > 0.0)) {
                continue;
            }
            if (cell_list_push(&list, grid_cell_index(g, i, j), len, err) != 0) {
                cell_list_free(&list);
                return -1;
            }
            total += len;
        }
    }
    const size_t n = list.n;
    size_t *cells = list.cells;
    double *lens = list.lens;
    if (n == 0 || !(total > 0.0)) {
        free(cells);
        free(lens);
        fdtd_error_set(err, "segment (%g, %g)-(%g, %g) covers no grid cell",
                       a.x, a.y, b.x, b.y);
        return -1;
    }

    /* A segment lying on a cell edge is counted in both neighbours; scaling
     * by clipped/total makes the weights sum to the clipped length exactly. */
    const double scale = clipped / total;
    size_t kept = 0;
    double dropped = 0.0;
    for (size_t k = 0; k < n; k++) {
        const double len = lens[k] * scale;
        if (solid && solid[cells[k]]) {
            dropped += len;
            continue;
        }
        cells[kept] = cells[k];
        lens[kept] = len;
        kept++;
    }
    if (kept == 0) {
        free(cells);
        free(lens);
        fdtd_error_set(err, "segment (%g, %g)-(%g, %g) lies on rigid cells only",
                       a.x, a.y, b.x, b.y);
        return -1;
    }
    if (dropped > 0.0) {
        fdtd_warn("segment source: %g%% of the segment length lies on rigid cells",
                  100.0 * dropped / clipped);
    }

    out->cells = fdtd_calloc(kept, sizeof *out->cells, err);
    if (!out->cells) {
        free(cells);
        free(lens);
        return -1;
    }
    out->weights = fdtd_calloc(kept, sizeof *out->weights, err);
    if (!out->weights) {
        free(cells);
        free(lens);
        return -1;
    }
    const double area = g->dx * g->dy;
    for (size_t k = 0; k < kept; k++) {
        out->cells[k] = cells[k];
        out->weights[k] = lens[k] / area;
    }
    free(cells);
    free(lens);
    set_metadata(out, shape, signal, kept);
    return 0;
}

int source_discretize(const Shape *shape, const Signal *signal, const Grid *g,
                      const uint8_t *solid, ModelSource *out, FdtdError *err)
{
    memset(out, 0, sizeof *out);
    if (!shape || !signal || !g) {
        fdtd_error_set(err, "source_discretize: missing shape, signal or grid");
        return -1;
    }
    if (!(g->dx > 0.0) || !(g->dy > 0.0) || g->nx < 1 || g->ny < 1) {
        fdtd_error_set(err, "source_discretize: invalid grid (%d x %d, dx = %g, dy = %g)",
                       g->nx, g->ny, g->dx, g->dy);
        return -1;
    }

    int rc;
    switch (shape->type) {
    case SHAPE_POINT:
        rc = discretize_point(shape, signal, g, solid, out, err);
        break;
    case SHAPE_SEGMENT:
        rc = discretize_segment(shape, signal, g, solid, out, err);
        break;
    case SHAPE_RECT:
    case SHAPE_CIRCLE:
    case SHAPE_POLYGON:
        fdtd_error_set(err, "%s source is not supported in this version",
                       shape_type_name(shape->type));
        rc = -1;
        break;
    case SHAPE_TYPE_COUNT:
    default:
        fdtd_error_set(err, "unknown source shape type %d", (int)shape->type);
        rc = -1;
        break;
    }
    if (rc != 0) {
        model_source_free(out);
    }
    return rc;
}

void model_source_free(ModelSource *s)
{
    if (!s) {
        return;
    }
    free(s->cells);
    free(s->weights);
    memset(s, 0, sizeof *s);
}
