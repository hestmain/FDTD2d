/*
 * test_source.c - unit tests for the spatial discretisation of sources.
 *
 * Grids are built by hand (the TOML parser is a separate task). Every
 * successful discretisation is checked against the normalisation rules in
 * source.h: sum_c w_c dx dy == 1 for a point and == the radiating segment
 * length for a line source.
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "common.h"
#include "geometry.h"
#include "grid.h"
#include "signal.h"
#include "source.h"

#include "test_util.h"

/* ------------------------------------------------------------- utilities */

/* Grid with no PML: off = 0, n = n_phys. */
static Grid plain_grid(int nx, int ny, double dx, double dy)
{
    Grid g;
    memset(&g, 0, sizeof g);
    g.nx = nx;
    g.ny = ny;
    g.nx_phys = nx;
    g.ny_phys = ny;
    g.off_x = 0;
    g.off_y = 0;
    g.dx = dx;
    g.dy = dy;
    g.x0 = 0.0;
    g.y0 = 0.0;
    return g;
}

/* Physical domain with lo_x/hi_x and lo_y/hi_y PML cells outside it. */
static Grid pml_grid(int nx_phys, int ny_phys, int lo_x, int hi_x, int lo_y, int hi_y,
                     double dx, double dy, double x0, double y0)
{
    Grid g;
    memset(&g, 0, sizeof g);
    g.nx_phys = nx_phys;
    g.ny_phys = ny_phys;
    g.off_x = lo_x;
    g.off_y = lo_y;
    g.nx = lo_x + nx_phys + hi_x;
    g.ny = lo_y + ny_phys + hi_y;
    g.dx = dx;
    g.dy = dy;
    g.x0 = x0;
    g.y0 = y0;
    return g;
}

static Signal harmonic_signal(double f, double amp)
{
    Signal s;
    memset(&s, 0, sizeof s);
    s.type = SIGNAL_HARMONIC;
    s.u.harmonic.frequency = f;
    s.u.harmonic.amplitude = amp;
    s.u.harmonic.phase = 0.0;
    s.u.harmonic.ramp_cycles = 0.0;
    return s;
}

static Shape point_at(double x, double y)
{
    Shape s;
    memset(&s, 0, sizeof s);
    s.type = SHAPE_POINT;
    s.u.point.position.x = x;
    s.u.point.position.y = y;
    return s;
}

static Shape segment_from(double x0, double y0, double x1, double y1)
{
    Shape s;
    memset(&s, 0, sizeof s);
    s.type = SHAPE_SEGMENT;
    s.u.segment.start.x = x0;
    s.u.segment.start.y = y0;
    s.u.segment.end.x = x1;
    s.u.segment.end.y = y1;
    return s;
}

static Shape rect_shape(double x0, double y0, double x1, double y1)
{
    Shape s;
    memset(&s, 0, sizeof s);
    s.type = SHAPE_RECT;
    s.u.rect.min.x = x0;
    s.u.rect.min.y = y0;
    s.u.rect.max.x = x1;
    s.u.rect.max.y = y1;
    return s;
}

static Shape circle_shape(double cx, double cy, double radius)
{
    Shape s;
    memset(&s, 0, sizeof s);
    s.type = SHAPE_CIRCLE;
    s.u.circle.center.x = cx;
    s.u.circle.center.y = cy;
    s.u.circle.radius = radius;
    return s;
}

/* Weighted "mass" of the distribution: sum_c w_c dx dy. */
static double source_mass(const Grid *g, const ModelSource *s)
{
    double sum = 0.0;
    for (size_t k = 0; k < s->n_cells; k++) {
        sum += s->weights[k];
    }
    return sum * g->dx * g->dy;
}

static double weight_of(const ModelSource *s, size_t cell)
{
    for (size_t k = 0; k < s->n_cells; k++) {
        if (s->cells[k] == cell) {
            return s->weights[k];
        }
    }
    return -1.0;
}

static int has_cell(const ModelSource *s, size_t cell)
{
    for (size_t k = 0; k < s->n_cells; k++) {
        if (s->cells[k] == cell) {
            return 1;
        }
    }
    return 0;
}

/* Structural invariants every successful result must satisfy. */
static void check_source_sane(const Grid *g, const ModelSource *s, const char *what)
{
    const size_t n_cells = grid_cell_count(g);
    CHECK_MSG(s->n_cells > 0, "%s: no cells", what);
    CHECK_MSG(s->cells != NULL, "%s: cells is NULL", what);
    CHECK_MSG(s->weights != NULL, "%s: weights is NULL", what);
    if (!s->cells || !s->weights) {
        return;
    }
    for (size_t k = 0; k < s->n_cells; k++) {
        CHECK_MSG(s->cells[k] < n_cells, "%s: cell %zu out of range", what, s->cells[k]);
        CHECK_MSG(isfinite(s->weights[k]), "%s: weight %zu not finite", what, k);
        CHECK_MSG(s->weights[k] > 0.0, "%s: weight %zu = %g not > 0", what, k,
                  s->weights[k]);
        for (size_t q = k + 1; q < s->n_cells; q++) {
            CHECK_MSG(s->cells[q] != s->cells[k], "%s: cell %zu duplicated", what,
                      s->cells[k]);
        }
    }
}

/* *out must be zeroed after a failed call. */
static void check_source_zeroed(const ModelSource *s)
{
    CHECK(s->cells == NULL);
    CHECK(s->weights == NULL);
    CHECK(s->n_cells == 0);
    CHECK((int)s->signal.type == 0);
    CHECK((int)s->shape_type == 0);
}

/* ------------------------------------------------------------- point source */

static void test_point_at_cell_centre(void)
{
    const Grid g = plain_grid(10, 10, 0.1, 0.1);
    const Signal sig = harmonic_signal(1000.0, 1.0);
    const Shape sh = point_at(0.35, 0.65); /* centre of cell (3, 6) */
    ModelSource s;
    FdtdError err = {{0}};

    CHECK(source_discretize(&sh, &sig, &g, NULL, &s, &err) == 0);
    check_source_sane(&g, &s, "centre");
    CHECK(s.n_cells == 1);
    CHECK(s.cells[0] == grid_cell_index(&g, 3, 6));
    CHECK_NEAR(s.weights[0], 1.0 / (g.dx * g.dy), 1e-9);
    CHECK_NEAR(source_mass(&g, &s), 1.0, 1e-12);
    model_source_free(&s);
}

static void test_point_bilinear(void)
{
    const Grid g = plain_grid(10, 10, 0.1, 0.1);
    const Signal sig = harmonic_signal(1000.0, 1.0);
    const Shape sh = point_at(0.37, 0.62); /* 0.2 along x, 0.7 along y */
    ModelSource s;
    FdtdError err = {{0}};

    CHECK(source_discretize(&sh, &sig, &g, NULL, &s, &err) == 0);
    check_source_sane(&g, &s, "bilinear");
    CHECK(s.n_cells == 4);
    CHECK_NEAR(source_mass(&g, &s), 1.0, 1e-12);

    /* bilinear weights (1-tx)(1-ty), tx(1-ty), (1-tx)ty, tx ty */
    const double area = g.dx * g.dy;
    const double w00 = 0.8 * 0.3;
    const double w10 = 0.2 * 0.3;
    const double w01 = 0.8 * 0.7;
    const double w11 = 0.2 * 0.7;
    CHECK_NEAR(weight_of(&s, grid_cell_index(&g, 3, 5)), w00 / area, 1e-9 * w00 / area);
    CHECK_NEAR(weight_of(&s, grid_cell_index(&g, 4, 5)), w10 / area, 1e-9 * w10 / area);
    CHECK_NEAR(weight_of(&s, grid_cell_index(&g, 3, 6)), w01 / area, 1e-9 * w01 / area);
    CHECK_NEAR(weight_of(&s, grid_cell_index(&g, 4, 6)), w11 / area, 1e-9 * w11 / area);
    model_source_free(&s);
}

/* True if every source cell belongs to the physical domain (none in PML). */
static bool all_cells_physical(const Grid *g, const ModelSource *s)
{
    for (size_t k = 0; k < s->n_cells; k++) {
        const int i = (int)(s->cells[k] % (size_t)g->nx);
        const int j = (int)(s->cells[k] / (size_t)g->nx);
        if (!grid_cell_is_physical(g, i, j)) {
            return false;
        }
    }
    return true;
}

static void test_point_with_pml_offset(void)
{
    /* Physical domain [0, 1]^2, 3 PML cells on the low and high x side,
     * 2 on the low and high y side. */
    const Grid g = pml_grid(10, 10, 3, 3, 2, 2, 0.1, 0.1, 0.0, 0.0);
    const Signal sig = harmonic_signal(1000.0, 1.0);
    FdtdError err = {{0}};

    CHECK(g.nx == 16 && g.ny == 14);

    /* The four corners and the middle of every edge must work. */
    const double xs[6] = {0.0, 1.0, 0.0, 1.0, 0.5, 0.5};
    const double ys[6] = {0.0, 0.0, 1.0, 1.0, 0.0, 1.0};
    for (int k = 0; k < 6; k++) {
        const Shape sh = point_at(xs[k], ys[k]);
        ModelSource s;
        err.msg[0] = '\0';
        CHECK_MSG(source_discretize(&sh, &sig, &g, NULL, &s, &err) == 0,
                  "corner/edge %d: %s", k, err.msg);
        check_source_sane(&g, &s, "corner/edge");
        CHECK_NEAR(source_mass(&g, &s), 1.0, 1e-12);
        /* weights stay in the physical domain, independent of the PML */
        CHECK_MSG(all_cells_physical(&g, &s), "corner/edge %d puts weight into PML", k);
        if (k < 4) {
            CHECK_MSG(s.n_cells == 1, "corner %d: %zu cells", k, s.n_cells);
        } else {
            CHECK_MSG(s.n_cells == 2, "edge %d: %zu cells", k, s.n_cells);
        }
        model_source_free(&s);
    }

    /* A point on the upper physical edge x = x1 belongs to the last
     * physical column (off_x + 9), not to the PML column after it: making
     * that cell solid must reject the source. */
    {
        uint8_t solid[16 * 14];
        memset(solid, 0, sizeof solid);
        solid[grid_cell_index(&g, 3 + 9, 2 + 5)] = 1;
        const Shape edge = point_at(1.0, 0.55);
        ModelSource se;
        err.msg[0] = '\0';
        CHECK(source_discretize(&edge, &sig, &g, solid, &se, &err) == -1);
        CHECK_CONTAINS(err.msg, "rigid obstacle");
        check_source_zeroed(&se);
    }

    /* A point in the middle of the physical domain: cell (off+5, off+5). */
    const Shape sh = point_at(0.55, 0.55);
    ModelSource s;
    CHECK(source_discretize(&sh, &sig, &g, NULL, &s, &err) == 0);
    CHECK(s.n_cells == 1);
    CHECK(s.cells[0] == grid_cell_index(&g, 3 + 5, 2 + 5));
    CHECK_NEAR(source_mass(&g, &s), 1.0, 1e-12);
    model_source_free(&s);
}

static void test_point_uses_grid_origin(void)
{
    /* x0/y0 are not zero: the bilinear bracket must use *x0*, not 0. */
    Grid g = plain_grid(10, 10, 0.1, 0.1);
    g.x0 = 1.0;
    g.y0 = -2.0;
    const Signal sig = harmonic_signal(1000.0, 1.0);
    const Shape sh = point_at(1.35, -1.35); /* centre of cell (3, 6) */
    ModelSource s;
    FdtdError err = {{0}};

    CHECK(source_discretize(&sh, &sig, &g, NULL, &s, &err) == 0);
    check_source_sane(&g, &s, "origin");
    CHECK(s.n_cells == 1);
    CHECK(s.cells[0] == grid_cell_index(&g, 3, 6));
    CHECK_NEAR(source_mass(&g, &s), 1.0, 1e-12);
    model_source_free(&s);
}

static void test_point_dx_ne_dy(void)
{
    const Grid g = plain_grid(10, 12, 0.1, 0.2);
    const Signal sig = harmonic_signal(1000.0, 1.0);
    const Shape sh = point_at(0.37, 0.62);
    ModelSource s;
    FdtdError err = {{0}};

    CHECK(source_discretize(&sh, &sig, &g, NULL, &s, &err) == 0);
    check_source_sane(&g, &s, "dx != dy");
    CHECK_NEAR(source_mass(&g, &s), 1.0, 1e-12);
    model_source_free(&s);
}

static void test_point_drops_solid_neighbour(void)
{
    const Grid g = plain_grid(10, 10, 0.1, 0.1);
    const Signal sig = harmonic_signal(1000.0, 1.0);
    /* (0.38, 0.39) lies in cell (3, 3); its bilinear neighbours are
     * (3, 3), (4, 3), (3, 4) and (4, 4). */
    const Shape sh = point_at(0.38, 0.39);
    uint8_t solid[100];
    ModelSource s;
    FdtdError err = {{0}};

    memset(solid, 0, sizeof solid);
    solid[grid_cell_index(&g, 4, 4)] = 1;

    CHECK(source_discretize(&sh, &sig, &g, solid, &s, &err) == 0);
    check_source_sane(&g, &s, "solid neighbour");
    CHECK(s.n_cells == 3);
    CHECK(!has_cell(&s, grid_cell_index(&g, 4, 4)));
    CHECK(has_cell(&s, grid_cell_index(&g, 3, 3)));
    CHECK(has_cell(&s, grid_cell_index(&g, 4, 3)));
    CHECK(has_cell(&s, grid_cell_index(&g, 3, 4)));
    CHECK_NEAR(source_mass(&g, &s), 1.0, 1e-12);
    /* the surviving weights keep their ratios: 0.42 : 0.18 : 0.28 */
    const double w33 = weight_of(&s, grid_cell_index(&g, 3, 3));
    const double w43 = weight_of(&s, grid_cell_index(&g, 4, 3));
    CHECK(w33 > w43);
    CHECK_NEAR(w33 / w43, 0.42 / 0.18, 1e-9);
    model_source_free(&s);
}

static void test_point_inside_obstacle_is_error(void)
{
    const Grid g = plain_grid(10, 10, 0.1, 0.1);
    const Signal sig = harmonic_signal(1000.0, 1.0);
    uint8_t solid[100];
    ModelSource s;
    FdtdError err = {{0}};

    memset(solid, 0, sizeof solid);
    solid[grid_cell_index(&g, 3, 4)] = 1;

    /* exactly at the cell centre of the solid cell */
    const Shape at_centre = point_at(0.35, 0.45);
    err.msg[0] = '\0';
    CHECK(source_discretize(&at_centre, &sig, &g, solid, &s, &err) == -1);
    CHECK_CONTAINS(err.msg, "lies inside a rigid obstacle");
    check_source_zeroed(&s);

    /* off centre, but still inside the solid cell (its nearest cell centre
     * and all neighbours except (3,4) are fluid): must still be an error. */
    const Shape off_centre = point_at(0.31, 0.41);
    err.msg[0] = '\0';
    CHECK(source_discretize(&off_centre, &sig, &g, solid, &s, &err) == -1);
    CHECK_CONTAINS(err.msg, "lies inside a rigid obstacle");
    check_source_zeroed(&s);

    /* the same geometry without a solid mask works fine */
    CHECK(source_discretize(&at_centre, &sig, &g, NULL, &s, &err) == 0);
    model_source_free(&s);
}

static void test_point_outside_domain(void)
{
    const Grid g = plain_grid(10, 10, 0.1, 0.1);
    const Signal sig = harmonic_signal(1000.0, 1.0);
    ModelSource s;
    FdtdError err = {{0}};

    const double bad[6][2] = {{1.5, 0.5},  {-0.01, 0.5}, {0.5, 1.01},
                              {0.5, -0.01}, {5.0, 5.0},   {-1.0, -1.0}};
    for (int k = 0; k < 6; k++) {
        const Shape sh = point_at(bad[k][0], bad[k][1]);
        err.msg[0] = '\0';
        CHECK_MSG(source_discretize(&sh, &sig, &g, NULL, &s, &err) == -1,
                  "point %d accepted", k);
        CHECK_CONTAINS(err.msg, "outside the physical domain");
        check_source_zeroed(&s);
    }

    /* a point on the closed boundary is inside; the bilinear bracket is
     * clamped, so the whole weight stays on the nearest cell */
    const Shape edge = point_at(1.0, 1.0);
    CHECK(source_discretize(&edge, &sig, &g, NULL, &s, &err) == 0);
    check_source_sane(&g, &s, "corner");
    CHECK(s.n_cells == 1);
    CHECK(s.cells[0] == grid_cell_index(&g, 9, 9));
    CHECK_NEAR(source_mass(&g, &s), 1.0, 1e-12);
    model_source_free(&s);

    /* upper edge, x on a cell boundary: split between the two cells of the
     * last row */
    const Shape edge_mid = point_at(0.5, 1.0);
    CHECK(source_discretize(&edge_mid, &sig, &g, NULL, &s, &err) == 0);
    check_source_sane(&g, &s, "edge midpoint");
    CHECK(s.n_cells == 2);
    CHECK_NEAR(source_mass(&g, &s), 1.0, 1e-12);
    model_source_free(&s);
}

/* ----------------------------------------------------------- line sources */

static void expect_segment_mass(const Grid *g, const Shape *sh, const uint8_t *solid,
                                double expected_len, int expected_warnings)
{
    const Signal sig = harmonic_signal(1000.0, 1.0);
    ModelSource s;
    FdtdError err = {{0}};

    fdtd_reset_warning_count();
    CHECK_MSG(source_discretize(sh, &sig, g, solid, &s, &err) == 0, "%s", err.msg);
    check_source_sane(g, &s, "segment");
    CHECK_NEAR(source_mass(g, &s), expected_len, 1e-12 * (1.0 + fabs(expected_len)));
    CHECK_MSG(fdtd_warning_count() == expected_warnings, "%d warnings, expected %d",
              fdtd_warning_count(), expected_warnings);
    model_source_free(&s);
}

static void test_segment_lengths(void)
{
    const Grid g = plain_grid(10, 10, 0.1, 0.1);

    /* horizontal, off the grid lines */
    Shape h = segment_from(0.05, 0.35, 0.95, 0.35);
    expect_segment_mass(&g, &h, NULL, 0.9, 0);

    /* vertical */
    Shape v = segment_from(0.35, 0.05, 0.35, 0.95);
    expect_segment_mass(&g, &v, NULL, 0.9, 0);

    /* diagonal through cell corners */
    Shape d = segment_from(0.05, 0.05, 0.95, 0.95);
    expect_segment_mass(&g, &d, NULL, 0.9 * sqrt(2.0), 0);

    /* diagonal with an arbitrary slope */
    Shape d2 = segment_from(0.05, 0.15, 0.95, 0.65);
    expect_segment_mass(&g, &d2, NULL, hypot(0.9, 0.5), 0);

    /* on a horizontal grid line: must not be counted in both neighbours */
    Shape gl = segment_from(0.05, 0.5, 0.95, 0.5);
    expect_segment_mass(&g, &gl, NULL, 0.9, 0);

    /* on a vertical grid line */
    Shape gv = segment_from(0.5, 0.05, 0.5, 0.95);
    expect_segment_mass(&g, &gv, NULL, 0.9, 0);

    /* on a grid line, spanning the whole domain */
    Shape full = segment_from(0.0, 0.5, 1.0, 0.5);
    expect_segment_mass(&g, &full, NULL, 1.0, 0);

    /* reversed end points give the same result */
    Shape rev = segment_from(0.95, 0.35, 0.05, 0.35);
    expect_segment_mass(&g, &rev, NULL, 0.9, 0);
}

static void test_segment_dx_ne_dy(void)
{
    const Grid g = plain_grid(10, 10, 0.05, 0.2);

    Shape h = segment_from(0.025, 0.3, 0.475, 0.3);
    expect_segment_mass(&g, &h, NULL, 0.45, 0);

    Shape v = segment_from(0.125, 0.1, 0.125, 1.9);
    expect_segment_mass(&g, &v, NULL, 1.8, 0);
}

static void test_segment_with_pml_offset(void)
{
    const Grid g = pml_grid(10, 10, 3, 3, 2, 2, 0.1, 0.1, 0.0, 0.0);

    /* segment along the physical domain edge: its full length must be
     * distributed, and only onto physical cells (never the PML cell that
     * shares the face) */
    Shape sh = segment_from(0.0, 0.2, 0.0, 0.8);
    expect_segment_mass(&g, &sh, NULL, 0.6, 0);
    {
        const Signal sig = harmonic_signal(1000.0, 1.0);
        ModelSource s;
        FdtdError err = {{0}};
        CHECK(source_discretize(&sh, &sig, &g, NULL, &s, &err) == 0);
        CHECK(all_cells_physical(&g, &s));
        CHECK(s.n_cells == 6);
        model_source_free(&s);
    }

    /* the same on the upper physical edges x = x1 and y = y1 */
    const Shape upper[2] = {segment_from(1.0, 0.2, 1.0, 0.8),
                            segment_from(0.2, 1.0, 0.8, 1.0)};
    for (int k = 0; k < 2; k++) {
        expect_segment_mass(&g, &upper[k], NULL, 0.6, 0);
        const Signal sig = harmonic_signal(1000.0, 1.0);
        ModelSource s;
        FdtdError err = {{0}};
        CHECK(source_discretize(&upper[k], &sig, &g, NULL, &s, &err) == 0);
        CHECK_MSG(all_cells_physical(&g, &s), "upper edge %d puts weight into PML", k);
        CHECK_MSG(s.n_cells == 6, "upper edge %d: %zu cells", k, s.n_cells);
        model_source_free(&s);
    }

    /* long diagonal across a large grid: cost/memory must scale with the
     * length (regression test for the former bounding-box allocation) */
    const Grid big = pml_grid(4000, 4000, 20, 20, 20, 20, 1e-3, 1e-3, 0.0, 0.0);
    Shape diag = segment_from(0.0, 0.0, 4.0, 4.0);
    expect_segment_mass(&big, &diag, NULL, 4.0 * sqrt(2.0), 0);

    /* interior segment */
    Shape in = segment_from(0.05, 0.35, 0.95, 0.35);
    expect_segment_mass(&g, &in, NULL, 0.9, 0);
}

static void test_segment_clipped(void)
{
    const Grid g = plain_grid(10, 10, 0.1, 0.1);

    Shape lo = segment_from(-0.5, 0.35, 0.5, 0.35);
    expect_segment_mass(&g, &lo, NULL, 0.5, 1);

    Shape hi = segment_from(0.5, 0.35, 2.5, 0.35);
    expect_segment_mass(&g, &hi, NULL, 0.5, 1);

    Shape both = segment_from(-1.0, 0.35, 2.0, 0.35);
    expect_segment_mass(&g, &both, NULL, 1.0, 1);

    /* diagonal crossing two edges */
    Shape diag = segment_from(-0.5, -0.5, 0.5, 0.5);
    expect_segment_mass(&g, &diag, NULL, 0.5 * sqrt(2.0), 1);
}

static void test_segment_outside_domain(void)
{
    const Grid g = plain_grid(10, 10, 0.1, 0.1);
    const Signal sig = harmonic_signal(1000.0, 1.0);
    ModelSource s;
    FdtdError err = {{0}};

    const Shape beside = segment_from(2.0, 0.35, 3.0, 0.35);
    err.msg[0] = '\0';
    CHECK(source_discretize(&beside, &sig, &g, NULL, &s, &err) == -1);
    CHECK_CONTAINS(err.msg, "does not cross the physical domain");
    check_source_zeroed(&s);

    const Shape below = segment_from(0.5, -1.0, 0.5, -0.5);
    err.msg[0] = '\0';
    CHECK(source_discretize(&below, &sig, &g, NULL, &s, &err) == -1);
    CHECK_CONTAINS(err.msg, "does not cross the physical domain");
    check_source_zeroed(&s);
}

static void test_segment_zero_length(void)
{
    const Grid g = plain_grid(10, 10, 0.1, 0.1);
    const Signal sig = harmonic_signal(1000.0, 1.0);
    const Shape sh = segment_from(0.5, 0.5, 0.5, 0.5);
    ModelSource s;
    FdtdError err = {{0}};

    CHECK(source_discretize(&sh, &sig, &g, NULL, &s, &err) == -1);
    CHECK_CONTAINS(err.msg, "zero length");
    check_source_zeroed(&s);
}

static void test_segment_drops_solid_cells(void)
{
    const Grid g = plain_grid(10, 10, 0.1, 0.1);
    const Signal sig = harmonic_signal(1000.0, 1.0);
    /* The segment runs through cells (0..9, 3); marking (4, 3) and (5, 3)
     * solid removes exactly 0.2 m of its 0.9 m length. */
    const Shape sh = segment_from(0.05, 0.35, 0.95, 0.35);
    uint8_t solid[100];
    ModelSource s;
    FdtdError err = {{0}};

    memset(solid, 0, sizeof solid);
    solid[grid_cell_index(&g, 4, 3)] = 1;
    solid[grid_cell_index(&g, 5, 3)] = 1;

    fdtd_reset_warning_count();
    CHECK(source_discretize(&sh, &sig, &g, solid, &s, &err) == 0);
    check_source_sane(&g, &s, "segment solid");
    CHECK(fdtd_warning_count() == 1);
    CHECK(s.n_cells == 8);
    CHECK(!has_cell(&s, grid_cell_index(&g, 4, 3)));
    CHECK(!has_cell(&s, grid_cell_index(&g, 5, 3)));
    CHECK_NEAR(source_mass(&g, &s), 0.7, 1e-12);
    model_source_free(&s);

    /* A vertical segment exactly on the grid line between a fluid column
     * (i = 4) and a solid column (i = 5): its length is shared equally by
     * both columns, so the solid half is dropped with one warning. */
    memset(solid, 0, sizeof solid);
    for (int j = 0; j < 10; j++) {
        solid[grid_cell_index(&g, 5, j)] = 1;
    }
    const Shape line = segment_from(0.5, 0.05, 0.5, 0.95);
    expect_segment_mass(&g, &line, solid, 0.45, 1);
}

static void test_segment_all_solid(void)
{
    const Grid g = plain_grid(10, 10, 0.1, 0.1);
    const Signal sig = harmonic_signal(1000.0, 1.0);
    const Shape sh = segment_from(0.05, 0.35, 0.95, 0.35);
    uint8_t solid[100];
    ModelSource s;
    FdtdError err = {{0}};

    memset(solid, 0, sizeof solid);
    for (int i = 0; i < g.nx; i++) {
        solid[grid_cell_index(&g, i, 3)] = 1;
    }

    CHECK(source_discretize(&sh, &sig, &g, solid, &s, &err) == -1);
    CHECK_CONTAINS(err.msg, "lies on rigid cells only");
    check_source_zeroed(&s);
}

/* ---------------------------------------------------------- unsupported */

static void test_area_shapes_rejected(void)
{
    const Grid g = plain_grid(10, 10, 0.1, 0.1);
    const Signal sig = harmonic_signal(1000.0, 1.0);
    ModelSource s;
    FdtdError err = {{0}};

    Vec2 tri[3] = {{0.1, 0.1}, {0.5, 0.1}, {0.3, 0.5}};
    Shape poly;
    memset(&poly, 0, sizeof poly);
    poly.type = SHAPE_POLYGON;
    poly.u.polygon.vertices = tri;
    poly.u.polygon.n = 3;

    const Shape rect = rect_shape(0.1, 0.1, 0.4, 0.4);
    const Shape circ = circle_shape(0.3, 0.3, 0.1);

    err.msg[0] = '\0';
    CHECK(source_discretize(&rect, &sig, &g, NULL, &s, &err) == -1);
    CHECK_CONTAINS(err.msg, "not supported in this version");
    CHECK_CONTAINS(err.msg, "rect");
    check_source_zeroed(&s);

    err.msg[0] = '\0';
    CHECK(source_discretize(&circ, &sig, &g, NULL, &s, &err) == -1);
    CHECK_CONTAINS(err.msg, "not supported in this version");
    CHECK_CONTAINS(err.msg, "circle");
    check_source_zeroed(&s);

    err.msg[0] = '\0';
    CHECK(source_discretize(&poly, &sig, &g, NULL, &s, &err) == -1);
    CHECK_CONTAINS(err.msg, "not supported in this version");
    CHECK_CONTAINS(err.msg, "polygon");
    check_source_zeroed(&s);
}

/* -------------------------------------------------------- invalid input */

static void test_invalid_arguments(void)
{
    const Grid g = plain_grid(10, 10, 0.1, 0.1);
    const Signal sig = harmonic_signal(1000.0, 1.0);
    const Shape sh = point_at(0.35, 0.35);
    ModelSource s;
    FdtdError err = {{0}};

    CHECK(source_discretize(NULL, &sig, &g, NULL, &s, &err) == -1);
    check_source_zeroed(&s);

    CHECK(source_discretize(&sh, NULL, &g, NULL, &s, &err) == -1);
    check_source_zeroed(&s);

    CHECK(source_discretize(&sh, &sig, NULL, NULL, &s, &err) == -1);
    check_source_zeroed(&s);

    Grid bad = plain_grid(10, 10, 0.0, 0.1); /* dx <= 0 */
    CHECK(source_discretize(&sh, &sig, &bad, NULL, &s, &err) == -1);
    check_source_zeroed(&s);

    bad = plain_grid(0, 10, 0.1, 0.1);
    CHECK(source_discretize(&sh, &sig, &bad, NULL, &s, &err) == -1);
    check_source_zeroed(&s);
}

/* ------------------------------------------------------- metadata / free */

static void test_metadata_and_free(void)
{
    const Grid g = plain_grid(10, 10, 0.1, 0.1);
    Signal sig = harmonic_signal(1234.0, 0.25);
    sig.u.harmonic.phase = 0.5;
    sig.u.harmonic.ramp_cycles = 2.0;
    const Shape sh = point_at(0.35, 0.35);
    ModelSource s;
    FdtdError err = {{0}};

    CHECK(source_discretize(&sh, &sig, &g, NULL, &s, &err) == 0);
    CHECK(s.shape_type == SHAPE_POINT);
    CHECK(s.signal.type == SIGNAL_HARMONIC);
    CHECK(s.signal.u.harmonic.frequency == 1234.0);
    CHECK(s.signal.u.harmonic.amplitude == 0.25);
    CHECK(s.signal.u.harmonic.phase == 0.5);
    CHECK(s.signal.u.harmonic.ramp_cycles == 2.0);

    /* segment metadata */
    ModelSource seg;
    const Shape shs = segment_from(0.05, 0.35, 0.95, 0.35);
    CHECK(source_discretize(&shs, &sig, &g, NULL, &seg, &err) == 0);
    CHECK(seg.shape_type == SHAPE_SEGMENT);
    model_source_free(&seg);
    model_source_free(&seg); /* twice */
    CHECK(seg.cells == NULL && seg.weights == NULL && seg.n_cells == 0);

    model_source_free(&s);
    CHECK(s.cells == NULL && s.weights == NULL && s.n_cells == 0);
    model_source_free(&s); /* twice */

    ModelSource zeroed;
    memset(&zeroed, 0, sizeof zeroed);
    model_source_free(&zeroed);
    model_source_free(NULL); /* must not crash */
}

static void test_zeroed_mask_matches_null(void)
{
    const Grid g = plain_grid(10, 10, 0.1, 0.1);
    const Signal sig = harmonic_signal(1000.0, 1.0);
    const Shape sh = point_at(0.37, 0.62);
    uint8_t solid[100];
    ModelSource a;
    ModelSource b;
    FdtdError err = {{0}};

    memset(solid, 0, sizeof solid);
    CHECK(source_discretize(&sh, &sig, &g, NULL, &a, &err) == 0);
    CHECK(source_discretize(&sh, &sig, &g, solid, &b, &err) == 0);
    CHECK(a.n_cells == b.n_cells);
    if (a.n_cells == b.n_cells) {
        for (size_t k = 0; k < a.n_cells; k++) {
            CHECK(a.cells[k] == b.cells[k]);
            CHECK(a.weights[k] == b.weights[k]);
        }
    }
    model_source_free(&a);
    model_source_free(&b);
}

int main(void)
{
    fdtd_set_warning_stream(NULL);

    RUN_TEST(test_point_at_cell_centre);
    RUN_TEST(test_point_bilinear);
    RUN_TEST(test_point_with_pml_offset);
    RUN_TEST(test_point_uses_grid_origin);
    RUN_TEST(test_point_dx_ne_dy);
    RUN_TEST(test_point_drops_solid_neighbour);
    RUN_TEST(test_point_inside_obstacle_is_error);
    RUN_TEST(test_point_outside_domain);
    RUN_TEST(test_segment_lengths);
    RUN_TEST(test_segment_dx_ne_dy);
    RUN_TEST(test_segment_with_pml_offset);
    RUN_TEST(test_segment_clipped);
    RUN_TEST(test_segment_outside_domain);
    RUN_TEST(test_segment_zero_length);
    RUN_TEST(test_segment_drops_solid_cells);
    RUN_TEST(test_segment_all_solid);
    RUN_TEST(test_area_shapes_rejected);
    RUN_TEST(test_invalid_arguments);
    RUN_TEST(test_metadata_and_free);
    RUN_TEST(test_zeroed_mask_matches_null);

    return TEST_REPORT();
}
