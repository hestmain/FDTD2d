/*
 * model.c - parser layer 2: SimConfig -> Model (grid, timestep, media map,
 * rigid mask, PML damping profiles and discretised sources).
 *
 * Everything the solver needs is precomputed here so that solver_step()
 * only reads the Model. See model.h for the contract.
 */
#include "model.h"

#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Clamp a floored cell coordinate into [0, n-1]. */
static int clamp_cell_index(double f, int n)
{
    if (!(f >= 0.0)) {
        return 0; /* also maps NaN to 0 */
    }
    if (f > (double)(n - 1)) {
        return n - 1;
    }
    return (int)f;
}

/* Number of cells of size h that fit into [lo, hi]. */
static int axis_cell_count(double lo, double hi, double h, char axis, int *out,
                           FdtdError *err)
{
    char hname[3] = {'d', axis, '\0'};
    const double len = hi - lo;
    const double q = len / h;
    if (!isfinite(q)) {
        fdtd_error_set(err, "grid: %c_range length %g cannot be resolved with %s = %g",
                       axis, len, hname, h);
        return -1;
    }
    if (q > (double)INT_MAX) {
        fdtd_error_set(err, "grid: %c_range length %g needs %.0f cells, more than the "
                            "supported maximum %d", axis, len, q, INT_MAX);
        return -1;
    }
    const double n = floor(q + 0.5); /* llround without the integer overflow risk */
    if (n < 1.0) {
        fdtd_error_set(err, "grid: %c_range length %g is smaller than one cell of %s = %g",
                       axis, len, hname, h);
        return -1;
    }
    if (fabs(q - n) > 1e-6 * fmax(1.0, q)) {
        fdtd_error_set(err, "grid: %c_range length %g is not a multiple of %s = %g "
                            "(nearest valid %s: %g)", axis, len, hname, h, hname, len / n);
        return -1;
    }
    *out = (int)n;
    return 0;
}

/* PML damping profile sigma(d) = sigma_max (d/L)^m. */
typedef struct PmlProfile {
    double L;         /* thickness [m] */
    double sigma_max; /* [1/s] */
} PmlProfile;

static PmlProfile make_profile(const PmlParams *p, double h, double c_ref)
{
    PmlProfile prof;
    prof.L = (double)p->thickness * h;
    prof.sigma_max = -(p->order + 1.0) * c_ref * log(p->reflection) / (2.0 * prof.L);
    return prof;
}

/* Damping at depth d [m] into the PML (0 at or outside the interface).
 * fill_sigma_axis() only passes d = k * h (0 <= k <= T) or (k + 0.5) * h
 * (0 <= k < T), T = PML thickness in cells, with exactly representable
 * multipliers; rounding is monotonic, so d <= fl(T * h) = L and
 * d / L <= 1 without clamping. */
static double profile_sigma(const PmlProfile *prof, double order, double d)
{
    if (!(d > 0.0)) {
        return 0.0; /* also keeps pow(0, 0) from returning 1 */
    }
    return prof->sigma_max * pow(d / prof->L, order);
}

/* Fill one axis' damping profiles. `off` is the index of the first physical
 * cell, so the lower interface sits at face `off` and the upper one at face
 * off + n_phys. `cell` [n] and `face` [n+1] must be zero-initialised;
 * non-PML sides keep sigma = 0 everywhere. */
static void fill_sigma_axis(double *cell, double *face, int n, int off, int n_phys,
                            int pml_lo, int pml_hi, double h, const PmlParams *p,
                            double c_ref)
{
    if (pml_lo) {
        const PmlProfile prof = make_profile(p, h, c_ref);
        for (int i = 0; i <= off; i++) {
            face[i] = profile_sigma(&prof, p->order, ((double)off - (double)i) * h);
        }
        for (int i = 0; i < off; i++) {
            cell[i] = profile_sigma(&prof, p->order,
                                    ((double)off - ((double)i + 0.5)) * h);
        }
    }
    if (pml_hi) {
        const int iface = off + n_phys;
        const PmlProfile prof = make_profile(p, h, c_ref);
        for (int i = iface; i <= n; i++) {
            face[i] = profile_sigma(&prof, p->order, ((double)i - (double)iface) * h);
        }
        for (int i = iface; i < n; i++) {
            cell[i] = profile_sigma(&prof, p->order,
                                    ((double)i + 0.5 - (double)iface) * h);
        }
    }
}

static void build_pml(Model *m, const SimConfig *cfg)
{
    m->pml = cfg->pml;
    for (int s = 0; s < SIDE_COUNT; s++) {
        m->boundary[s] = cfg->boundary[s];
        m->pml_cells[s] = cfg->boundary[s] == BC_PML ? cfg->pml.thickness : 0;
    }
}

/* Mark every cell whose centre lies inside an obstacle shape. m->solid and
 * m->n_solid start zeroed. */
static void build_solid_mask(Model *m, const SimConfig *cfg, size_t n_cells)
{
    const Grid *g = &m->grid;
    for (int k = 0; k < cfg->n_obstacles; k++) {
        const Shape *shape = &cfg->obstacles[k].shape;
        Vec2 bmin, bmax;
        shape_bbox(shape, &bmin, &bmax);
        const int i_lo = clamp_cell_index(floor((bmin.x - g->x0) / g->dx) +
                                          (double)g->off_x - 1.0, g->nx);
        const int i_hi = clamp_cell_index(floor((bmax.x - g->x0) / g->dx) +
                                          (double)g->off_x + 1.0, g->nx);
        const int j_lo = clamp_cell_index(floor((bmin.y - g->y0) / g->dy) +
                                          (double)g->off_y - 1.0, g->ny);
        const int j_hi = clamp_cell_index(floor((bmax.y - g->y0) / g->dy) +
                                          (double)g->off_y + 1.0, g->ny);
        size_t contained = 0;
        size_t in_pml = 0;
        for (int j = j_lo; j <= j_hi; j++) {
            for (int i = i_lo; i <= i_hi; i++) {
                const Vec2 centre = {grid_cell_x(g, i), grid_cell_y(g, j)};
                if (!shape_contains(shape, centre)) {
                    continue;
                }
                contained++;
                if (!grid_cell_is_physical(g, i, j)) {
                    in_pml++;
                }
                m->solid[grid_cell_index(g, i, j)] = 1;
            }
        }
        if (contained == 0) {
            fdtd_warn("obstacle[%d] (%s) contains no cell centre and has no effect",
                      k, shape_type_name(shape->type));
        }
        if (in_pml > 0) {
            fdtd_warn("obstacle[%d] (%s) extends into the PML (%zu cells); rigid cells "
                      "inside the PML reflect waves and reduce its absorption",
                      k, shape_type_name(shape->type), in_pml);
        }
    }
    for (size_t c = 0; c < n_cells; c++) {
        if (m->solid[c]) {
            m->n_solid++;
        }
    }
}

/* Warn when the shortest wavelength is resolved by too few cells. */
static void check_resolution(const SimConfig *cfg, const Grid *g, double c_min)
{
    double f_max = 0.0;
    for (int k = 0; k < cfg->n_sources; k++) {
        const double f = signal_max_frequency(&cfg->sources[k].signal);
        if (f > f_max) {
            f_max = f;
        }
    }
    if (!(f_max > 0.0)) {
        return;
    }
    const double ppw = c_min / (f_max * fmax(g->dx, g->dy));
    if (ppw < MODEL_MIN_POINTS_PER_WAVELENGTH) {
        fdtd_warn("only %.2f points per wavelength at %g Hz (recommended >= %g); "
                  "reduce dx/dy or the source frequency",
                  ppw, f_max, MODEL_MIN_POINTS_PER_WAVELENGTH);
    }
}

static int build_sources(Model *m, const SimConfig *cfg, FdtdError *err)
{
    if (cfg->n_sources == 0) {
        fdtd_warn("no sources defined; nothing will excite the domain");
        return 0;
    }
    m->sources = fdtd_calloc((size_t)cfg->n_sources, sizeof *m->sources, err);
    if (!m->sources) {
        return -1;
    }
    m->n_sources = cfg->n_sources; /* set first so a partial build is freed */
    for (int k = 0; k < cfg->n_sources; k++) {
        if (source_discretize(&cfg->sources[k].shape, &cfg->sources[k].signal,
                              &m->grid, m->solid, &m->sources[k], err) != 0) {
            fdtd_error_prefix(err, "source[%d]", k);
            return -1;
        }
    }
    return 0;
}

static int model_build_impl(const SimConfig *cfg, Model *m, FdtdError *err)
{
    Grid *g = &m->grid;

    m->duration = cfg->duration;

    /* ---- grid geometry ---------------------------------------------------- */
    int nx_phys = 0;
    int ny_phys = 0;
    if (axis_cell_count(cfg->x_range[0], cfg->x_range[1], cfg->dx, 'x', &nx_phys, err) != 0) {
        return -1;
    }
    if (axis_cell_count(cfg->y_range[0], cfg->y_range[1], cfg->dy, 'y', &ny_phys, err) != 0) {
        return -1;
    }
    build_pml(m, cfg);
    const long long ll_nx = (long long)m->pml_cells[SIDE_X_MIN] + (long long)nx_phys +
                            (long long)m->pml_cells[SIDE_X_MAX];
    const long long ll_ny = (long long)m->pml_cells[SIDE_Y_MIN] + (long long)ny_phys +
                            (long long)m->pml_cells[SIDE_Y_MAX];
    if (ll_nx + 1 > (long long)INT_MAX || ll_ny + 1 > (long long)INT_MAX) {
        fdtd_error_set(err, "grid: total cell count %lld x %lld is too large",
                       ll_nx, ll_ny);
        return -1;
    }
    g->nx = (int)ll_nx;
    g->ny = (int)ll_ny;
    g->nx_phys = nx_phys;
    g->ny_phys = ny_phys;
    g->off_x = m->pml_cells[SIDE_X_MIN];
    g->off_y = m->pml_cells[SIDE_Y_MIN];
    g->dx = cfg->dx;
    g->dy = cfg->dy;
    g->x0 = cfg->x_range[0];
    g->y0 = cfg->y_range[0];
    if ((size_t)g->nx > SIZE_MAX / (size_t)g->ny) {
        fdtd_error_set(err, "grid: %d x %d cells do not fit in a size_t",
                       g->nx, g->ny);
        return -1;
    }
    const size_t n_cells = grid_cell_count(g);

    /* ---- media ------------------------------------------------------------ */
    if (medium_table_init(&m->media, &cfg->medium, err) != 0) {
        return -1;
    }
    const double c_max = medium_table_c_max(&m->media);
    const double c_min = medium_table_c_min(&m->media);
    /* Version 1: background only. Medium regions will append table entries
     * and paint their index into this map. */
    m->medium_id = fdtd_calloc(n_cells, sizeof *m->medium_id, err);
    if (!m->medium_id) {
        return -1;
    }

    /* ---- time step -------------------------------------------------------- */
    const double inv2 = 1.0 / (g->dx * g->dx) + 1.0 / (g->dy * g->dy);
    const double denom = c_max * sqrt(inv2);
    if (!isfinite(denom) || !(denom > 0.0)) {
        fdtd_error_set(err, "grid: cell size dx = %g, dy = %g is not usable",
                       g->dx, g->dy);
        return -1;
    }
    if (cfg->dt_set) {
        m->dt = cfg->dt;
        m->cfl = c_max * m->dt * sqrt(inv2);
        if (!(m->cfl <= 1.0)) {
            fdtd_error_set(err, "simulation: dt = %g s is above the CFL limit %g s "
                                "(cfl = %.6g > 1)", m->dt, 1.0 / denom, m->cfl);
            return -1;
        }
    } else {
        m->cfl = cfg->cfl;
        m->dt = cfg->cfl / denom;
        /* denom can be finite but tiny (very slow medium and huge cells),
         * then dt overflows. */
        if (!isfinite(m->dt)) {
            fdtd_error_set(err, "simulation: the derived time step cfl / (c_max * "
                                "sqrt(1/dx^2 + 1/dy^2)) overflows (c_max = %g m/s, "
                                "dx = %g m, dy = %g m)", c_max, g->dx, g->dy);
            return -1;
        }
    }
    double steps = ceil(m->duration / m->dt - 1e-9);
    if (!(steps >= 1.0)) {
        steps = 1.0;
    }
    if (!(steps < (double)LONG_MAX)) { /* strict: (double)LONG_MAX may round up */
        fdtd_error_set(err, "simulation: duration/dt = %g needs too many steps",
                       m->duration / m->dt);
        return -1;
    }
    m->n_steps = (long)steps;

    /* ---- rigid obstacles -------------------------------------------------- */
    m->solid = fdtd_calloc(n_cells, sizeof *m->solid, err);
    if (!m->solid) {
        return -1;
    }
    build_solid_mask(m, cfg, n_cells);

    /* ---- PML damping profiles --------------------------------------------- */
    m->sigma_x_cell = fdtd_calloc((size_t)g->nx, sizeof *m->sigma_x_cell, err);
    m->sigma_x_face = fdtd_calloc((size_t)g->nx + 1, sizeof *m->sigma_x_face, err);
    m->sigma_y_cell = fdtd_calloc((size_t)g->ny, sizeof *m->sigma_y_cell, err);
    m->sigma_y_face = fdtd_calloc((size_t)g->ny + 1, sizeof *m->sigma_y_face, err);
    if (!m->sigma_x_cell || !m->sigma_x_face || !m->sigma_y_cell || !m->sigma_y_face) {
        return -1;
    }
    fill_sigma_axis(m->sigma_x_cell, m->sigma_x_face, g->nx, g->off_x, g->nx_phys,
                    m->boundary[SIDE_X_MIN] == BC_PML,
                    m->boundary[SIDE_X_MAX] == BC_PML, g->dx, &m->pml, c_max);
    fill_sigma_axis(m->sigma_y_cell, m->sigma_y_face, g->ny, g->off_y, g->ny_phys,
                    m->boundary[SIDE_Y_MIN] == BC_PML,
                    m->boundary[SIDE_Y_MAX] == BC_PML, g->dy, &m->pml, c_max);

    /* ---- sources ---------------------------------------------------------- */
    check_resolution(cfg, g, c_min);
    if (build_sources(m, cfg, err) != 0) {
        return -1;
    }
    return 0;
}

int model_build(const SimConfig *cfg, Model *m, FdtdError *err)
{
    memset(m, 0, sizeof *m);
    if (model_build_impl(cfg, m, err) != 0) {
        model_free(m); /* also leaves *m zeroed */
        return -1;
    }
    return 0;
}

void model_free(Model *m)
{
    if (!m) {
        return;
    }
    for (int k = 0; k < m->n_sources; k++) {
        model_source_free(&m->sources[k]);
    }
    free(m->sources);
    free(m->sigma_x_cell);
    free(m->sigma_x_face);
    free(m->sigma_y_cell);
    free(m->sigma_y_face);
    free(m->solid);
    free(m->medium_id);
    medium_table_free(&m->media);
    memset(m, 0, sizeof *m);
}

/* Highest source frequency, 0 if there is none. */
static double model_f_max(const Model *m)
{
    double f_max = 0.0;
    for (int k = 0; k < m->n_sources; k++) {
        const double f = signal_max_frequency(&m->sources[k].signal);
        if (f > f_max) {
            f_max = f;
        }
    }
    return f_max;
}

void model_print_summary(FILE *out, const Model *m)
{
    const Grid *g = &m->grid;

    fprintf(out, "grid:  %d x %d cells physical, %d x %d cells total (incl. PML)\n",
            g->nx_phys, g->ny_phys, g->nx, g->ny);
    fprintf(out, "       dx = %g m, dy = %g m\n", g->dx, g->dy);
    fprintf(out, "       domain x = [%g, %g] m, y = [%g, %g] m\n",
            g->x0, grid_x1(g), g->y0, grid_y1(g));
    fprintf(out, "       pml cells: x_min %d, x_max %d, y_min %d, y_max %d\n",
            m->pml_cells[SIDE_X_MIN], m->pml_cells[SIDE_X_MAX],
            m->pml_cells[SIDE_Y_MIN], m->pml_cells[SIDE_Y_MAX]);
    fprintf(out, "time:  dt = %g s, n_steps = %ld, requested duration = %g s, "
                 "simulated %g s\n",
            m->dt, m->n_steps, m->duration, (double)m->n_steps * m->dt);
    fprintf(out, "       cfl = %g\n", m->cfl);
    const double f_max = model_f_max(m);
    if (f_max > 0.0) {
        const double c_min = medium_table_c_min(&m->media);
        fprintf(out, "       points per wavelength = %.2f at %g Hz\n",
                c_min / (f_max * fmax(g->dx, g->dy)), f_max);
    } else {
        fputs("       points per wavelength = n/a (no periodic source)\n", out);
    }
    for (int k = 0; k < m->media.count; k++) {
        const Medium *med = &m->media.items[k];
        fprintf(out, "medium: [%d] rho = %g kg/m^3, c = %g m/s, K = %g Pa\n",
                k, med->density, med->c, medium_bulk_modulus(med));
    }
    fprintf(out, "boundary: x_min %s, x_max %s, y_min %s, y_max %s\n",
            boundary_type_name(m->boundary[SIDE_X_MIN]),
            boundary_type_name(m->boundary[SIDE_X_MAX]),
            boundary_type_name(m->boundary[SIDE_Y_MIN]),
            boundary_type_name(m->boundary[SIDE_Y_MAX]));
    fprintf(out, "obstacles: %zu solid cells of %zu\n", m->n_solid,
            grid_cell_count(g));
    fprintf(out, "sources: %d\n", m->n_sources);
    for (int k = 0; k < m->n_sources; k++) {
        const ModelSource *src = &m->sources[k];
        fprintf(out, "  [%d] %s source, %zu cells, %s signal", k,
                shape_type_name(src->shape_type), src->n_cells,
                signal_type_name(src->signal.type));
        if (src->signal.type == SIGNAL_HARMONIC) {
            fprintf(out, ", f = %g Hz, amplitude = %g",
                    src->signal.u.harmonic.frequency,
                    src->signal.u.harmonic.amplitude);
        }
        fputc('\n', out);
    }
}
