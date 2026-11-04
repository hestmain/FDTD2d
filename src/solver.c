/*
 * solver.c - 2D linear-acoustics FDTD, staggered grid, leapfrog, split-field
 * PML. See solver.h for the equations and the boundary rules.
 *
 * Memory layout (see grid.h): x varies fastest. Cell (i, j) -> j*nx + i,
 * x-face (i, j) -> j*(nx+1) + i, y-face (i, j) -> j*nx + i.
 *
 * Hot loops run j outer / i inner over contiguous rows with restrict-
 * qualified row pointers. The outermost faces are handled outside of them:
 * rigid and PML sides keep v = 0 there (and are never touched), only
 * pressure_release sides get a ghost-cell update.
 */
#include "solver.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* ----------------------------------------------------------------- setup */

/* Precondition of this file: *m comes from model_build() and outlives the
 * solver, so grid, dt, medium map, damping tables, boundary enums and source
 * cells are valid and are not re-checked. Checked here: allocation sizes, and
 * K and sigma, which valid user input can still make unusable. */

static bool cell_is_solid(const Model *m, size_t c)
{
    return m->solid[c] != 0;
}

/* Damping coefficients a, c for one axis from a sigma profile. Extreme but
 * valid PML parameters (e.g. a tiny cell size) can overflow sigma to inf. */
static int fill_damping(double *a, double *c, const double *sigma, int n, double dt,
                        const char *what, FdtdError *err)
{
    for (int i = 0; i < n; i++) {
        const double sg = sigma[i];
        if (!isfinite(sg)) {
            fdtd_error_set(err, "solver_init: %s damping %g at index %d is not usable",
                           what, sg, i);
            return -1;
        }
        const double h = 0.5 * sg * dt;
        a[i] = (1.0 - h) / (1.0 + h);
        c[i] = 1.0 / (1.0 + h);
    }
    return 0;
}

/* kdt, bx, by and the damping tables. Returns -1 and sets err on bad data. */
static int compute_coefficients(Solver *s, const Model *m, FdtdError *err)
{
    const int nx = s->nx;
    const int ny = s->ny;
    const double dt = m->dt;
    const Medium *med = m->media.items;
    const MediumId *mid = m->medium_id;

    /* K dt per cell (0 for solid cells). */
    for (size_t c = 0; c < (size_t)nx * (size_t)ny; c++) {
        if (cell_is_solid(m, c)) {
            s->kdt[c] = 0.0;
            continue;
        }
        const double rho = med[mid[c]].density;
        const double K = medium_bulk_modulus(&med[mid[c]]);
        if (!(isfinite(rho) && rho > 0.0 && isfinite(K) && K > 0.0)) {
            fdtd_error_set(err, "solver_init: medium %d has unusable density %g or "
                                "bulk modulus %g", (int)mid[c], rho, K);
            return -1;
        }
        s->kdt[c] = K * dt;
    }

    /* dt / rho_face on x-faces. Interior faces between two fluid cells;
     * outer faces only on pressure_release sides (ghost cell, rho of the
     * inside cell). Everything else stays 0, i.e. v = 0. */
    for (int j = 0; j < ny; j++) {
        const size_t crow = (size_t)j * (size_t)nx;
        const size_t frow = (size_t)j * (size_t)(nx + 1);
        for (int i = 1; i < nx; i++) {
            const size_t cl = crow + (size_t)(i - 1);
            const size_t cr = crow + (size_t)i;
            if (s->kdt[cl] > 0.0 && s->kdt[cr] > 0.0) {
                const double rho = 0.5 * (med[mid[cl]].density + med[mid[cr]].density);
                s->bx[frow + (size_t)i] = dt / rho;
            }
        }
        if (m->boundary[SIDE_X_MIN] == BC_PRESSURE_RELEASE && s->kdt[crow] > 0.0) {
            s->bx[frow] = dt / med[mid[crow]].density;
        }
        if (m->boundary[SIDE_X_MAX] == BC_PRESSURE_RELEASE &&
            s->kdt[crow + (size_t)(nx - 1)] > 0.0) {
            s->bx[frow + (size_t)nx] = dt / med[mid[crow + (size_t)(nx - 1)]].density;
        }
    }

    /* Same for the y-faces. */
    for (int j = 1; j < ny; j++) {
        const size_t lo = (size_t)(j - 1) * (size_t)nx;
        const size_t hi = (size_t)j * (size_t)nx;
        for (int i = 0; i < nx; i++) {
            const size_t cl = lo + (size_t)i;
            const size_t cr = hi + (size_t)i;
            if (s->kdt[cl] > 0.0 && s->kdt[cr] > 0.0) {
                const double rho = 0.5 * (med[mid[cl]].density + med[mid[cr]].density);
                s->by[hi + (size_t)i] = dt / rho;
            }
        }
    }
    for (int i = 0; i < nx; i++) {
        if (m->boundary[SIDE_Y_MIN] == BC_PRESSURE_RELEASE && s->kdt[i] > 0.0) {
            s->by[i] = dt / med[mid[i]].density;
        }
        const size_t top = (size_t)(ny - 1) * (size_t)nx + (size_t)i;
        if (m->boundary[SIDE_Y_MAX] == BC_PRESSURE_RELEASE && s->kdt[top] > 0.0) {
            s->by[(size_t)ny * (size_t)nx + (size_t)i] = dt / med[mid[top]].density;
        }
    }

    /* Damping. */
    if (fill_damping(s->ax_c, s->cx_c, m->sigma_x_cell, nx, dt, "x cell", err) != 0 ||
        fill_damping(s->ax_f, s->cx_f, m->sigma_x_face, nx + 1, dt, "x face", err) != 0 ||
        fill_damping(s->ay_c, s->cy_c, m->sigma_y_cell, ny, dt, "y cell", err) != 0 ||
        fill_damping(s->ay_f, s->cy_f, m->sigma_y_face, ny + 1, dt, "y face", err) != 0) {
        return -1;
    }
    /* Pressure-release sides have no PML (sigma = 0 on their outer faces),
     * so a = c = 1 there and the plain update applies. */
    return 0;
}

void solver_free(Solver *s)
{
    if (!s) {
        return;
    }
    free(s->px);
    free(s->py);
    free(s->vx);
    free(s->vy);
    free(s->kdt);
    free(s->bx);
    free(s->by);
    free(s->ax_c);
    free(s->cx_c);
    free(s->ax_f);
    free(s->cx_f);
    free(s->ay_c);
    free(s->cy_c);
    free(s->ay_f);
    free(s->cy_f);
    memset(s, 0, sizeof *s);
}

int solver_init(Solver *s, const Model *m, FdtdError *err)
{
    memset(s, 0, sizeof *s);
    const size_t nx = (size_t)m->grid.nx;
    const size_t ny = (size_t)m->grid.ny;
    if (nx > SIZE_MAX / (ny + 1) || ny > SIZE_MAX / (nx + 1)) {
        fdtd_error_set(err, "solver_init: grid %d x %d is too large", m->grid.nx,
                       m->grid.ny);
        return -1;
    }
    s->model = m;
    s->nx = m->grid.nx;
    s->ny = m->grid.ny;

    const struct {
        double **ptr;
        size_t count;
    } tables[] = {
        {&s->px, nx * ny},        {&s->py, nx * ny},
        {&s->vx, (nx + 1) * ny},  {&s->vy, nx * (ny + 1)},
        {&s->kdt, nx * ny},
        {&s->bx, (nx + 1) * ny},  {&s->by, nx * (ny + 1)},
        {&s->ax_c, nx},           {&s->cx_c, nx},
        {&s->ax_f, nx + 1},       {&s->cx_f, nx + 1},
        {&s->ay_c, ny},           {&s->cy_c, ny},
        {&s->ay_f, ny + 1},       {&s->cy_f, ny + 1},
    };
    for (size_t t = 0; t < sizeof tables / sizeof tables[0]; t++) {
        *tables[t].ptr = fdtd_calloc(tables[t].count, sizeof(double), err);
        if (!*tables[t].ptr) {
            solver_free(s);
            return -1;
        }
    }
    if (compute_coefficients(s, m, err) != 0) {
        solver_free(s);
        return -1;
    }
    return 0;
}

/* ------------------------------------------------------------ time step */

/* vx on interior faces i = 1..nx-1 of one row. All pointers are row bases. */
static void update_vx_row(double *restrict vx, const double *restrict bx,
                          const double *restrict px, const double *restrict py,
                          const double *restrict ax, const double *restrict cx,
                          int nx, double inv_dx)
{
    for (int i = 1; i < nx; i++) {
        const double dp = (px[i] + py[i]) - (px[i - 1] + py[i - 1]);
        vx[i] = ax[i] * vx[i] - (cx[i] * bx[i] * inv_dx) * dp;
    }
}

/* vy on the interior y-face row j >= 1 (a, c are the face's scalars). */
static void update_vy_row(double *restrict vy, const double *restrict by,
                          const double *restrict px, const double *restrict py,
                          const double *restrict pxm, const double *restrict pym,
                          double a, double c_dy, int nx)
{
    for (int i = 0; i < nx; i++) {
        const double dp = (px[i] + py[i]) - (pxm[i] + pym[i]);
        vy[i] = a * vy[i] - (c_dy * by[i]) * dp;
    }
}

/* px, py of one row from the velocity divergence. */
static void update_p_row(double *restrict px, double *restrict py,
                         const double *restrict kdt, const double *restrict vx,
                         const double *restrict vy_lo, const double *restrict vy_hi,
                         const double *restrict ax, const double *restrict cx,
                         double ay, double cy_dy, int nx, double inv_dx)
{
    for (int i = 0; i < nx; i++) {
        px[i] = ax[i] * px[i] - (cx[i] * kdt[i] * inv_dx) * (vx[i + 1] - vx[i]);
        py[i] = ay * py[i] - (cy_dy * kdt[i]) * (vy_hi[i] - vy_lo[i]);
    }
}

/* Outer-face velocities on pressure_release sides (ghost p = -p_inside). */
static void update_release_faces(Solver *s)
{
    const Model *m = s->model;
    const int nx = s->nx;
    const int ny = s->ny;
    const double inv_dx = 1.0 / m->grid.dx;
    const double inv_dy = 1.0 / m->grid.dy;
    const size_t fx = (size_t)nx + 1;

    if (m->boundary[SIDE_X_MIN] == BC_PRESSURE_RELEASE) {
        const double a = s->ax_f[0];
        const double c = s->cx_f[0];
        for (int j = 0; j < ny; j++) {
            const size_t f = (size_t)j * fx;
            const size_t cell = (size_t)j * (size_t)nx;
            const double dp = 2.0 * (s->px[cell] + s->py[cell]);
            s->vx[f] = a * s->vx[f] - (c * s->bx[f] * inv_dx) * dp;
        }
    }
    if (m->boundary[SIDE_X_MAX] == BC_PRESSURE_RELEASE) {
        const double a = s->ax_f[nx];
        const double c = s->cx_f[nx];
        for (int j = 0; j < ny; j++) {
            const size_t f = (size_t)j * fx + (size_t)nx;
            const size_t cell = (size_t)j * (size_t)nx + (size_t)(nx - 1);
            const double dp = -2.0 * (s->px[cell] + s->py[cell]);
            s->vx[f] = a * s->vx[f] - (c * s->bx[f] * inv_dx) * dp;
        }
    }
    if (m->boundary[SIDE_Y_MIN] == BC_PRESSURE_RELEASE) {
        const double a = s->ay_f[0];
        const double c = s->cy_f[0];
        for (int i = 0; i < nx; i++) {
            const double dp = 2.0 * (s->px[i] + s->py[i]);
            s->vy[i] = a * s->vy[i] - (c * s->by[i] * inv_dy) * dp;
        }
    }
    if (m->boundary[SIDE_Y_MAX] == BC_PRESSURE_RELEASE) {
        const double a = s->ay_f[ny];
        const double c = s->cy_f[ny];
        const size_t row = (size_t)ny * (size_t)nx;
        const size_t last = (size_t)(ny - 1) * (size_t)nx;
        for (int i = 0; i < nx; i++) {
            const double dp = -2.0 * (s->px[last + (size_t)i] + s->py[last + (size_t)i]);
            s->vy[row + (size_t)i] = a * s->vy[row + (size_t)i] -
                                     (c * s->by[row + (size_t)i] * inv_dy) * dp;
        }
    }
}

void solver_step(Solver *s)
{
    const Model *m = s->model;
    const int nx = s->nx;
    const int ny = s->ny;
    const double dt = m->dt;
    const double inv_dx = 1.0 / m->grid.dx;
    const double inv_dy = 1.0 / m->grid.dy;
    const size_t cw = (size_t)nx;
    const size_t fw = (size_t)nx + 1;

    /* 1. velocities: interior faces, then pressure_release outer faces. */
    for (int j = 0; j < ny; j++) {
        const size_t crow = (size_t)j * cw;
        update_vx_row(s->vx + (size_t)j * fw, s->bx + (size_t)j * fw, s->px + crow,
                      s->py + crow, s->ax_f, s->cx_f, nx, inv_dx);
        if (j >= 1) {
            update_vy_row(s->vy + crow, s->by + crow, s->px + crow, s->py + crow,
                          s->px + crow - cw, s->py + crow - cw, s->ay_f[j],
                          s->cy_f[j] * inv_dy, nx);
        }
    }
    update_release_faces(s);

    /* 2. pressure. */
    for (int j = 0; j < ny; j++) {
        const size_t crow = (size_t)j * cw;
        update_p_row(s->px + crow, s->py + crow, s->kdt + crow, s->vx + (size_t)j * fw,
                     s->vy + crow, s->vy + crow + cw, s->ax_c, s->cx_c, s->ay_c[j],
                     s->cy_c[j] * inv_dy, nx, inv_dx);
    }

    /* 3. sources at t_{n+1/2}; total increment K dt w s split evenly. */
    const double t_half = ((double)s->step + 0.5) * dt;
    for (int k = 0; k < m->n_sources; k++) {
        const ModelSource *src = &m->sources[k];
        const double val = signal_eval(&src->signal, t_half);
        for (size_t n = 0; n < src->n_cells; n++) {
            const size_t c = src->cells[n];
            const double inc = 0.5 * s->kdt[c] * src->weights[n] * val;
            s->px[c] += inc;
            s->py[c] += inc;
        }
    }

    /* 4. clock. */
    s->step++;
    s->time = (double)s->step * dt;
}

/* ---------------------------------------------------------- observation */

void solver_get_pressure(const Solver *s, double *out)
{
    const size_t n = (size_t)s->nx * (size_t)s->ny;
    for (size_t c = 0; c < n; c++) {
        out[c] = s->px[c] + s->py[c];
    }
}

double solver_pressure_at(const Solver *s, size_t cell)
{
    return s->px[cell] + s->py[cell];
}

double solver_acoustic_energy(const Solver *s)
{
    const Model *m = s->model;
    const size_t n_cells = (size_t)s->nx * (size_t)s->ny;
    const MediumId *mid = m->medium_id;
    const Medium *med = m->media.items;

    double potential = 0.0; /* sum p^2 / (2K) */
    for (size_t c = 0; c < n_cells; c++) {
        if (s->kdt[c] > 0.0) {
            const double p = s->px[c] + s->py[c];
            potential += p * p / (2.0 * medium_bulk_modulus(&med[mid[c]]));
        }
    }

    /* sum w rho_face v^2 / 2 over active faces. The outer faces of
     * pressure_release sides have half of their control volume outside the
     * grid: weight 1/2 (the p0*v0 cross terms of the leapfrog energy balance
     * cancel only with this weight). All other active faces have weight 1. */
    const double w_xmin = m->boundary[SIDE_X_MIN] == BC_PRESSURE_RELEASE ? 0.5 : 1.0;
    const double w_xmax = m->boundary[SIDE_X_MAX] == BC_PRESSURE_RELEASE ? 0.5 : 1.0;
    const double w_ymin = m->boundary[SIDE_Y_MIN] == BC_PRESSURE_RELEASE ? 0.5 : 1.0;
    const double w_ymax = m->boundary[SIDE_Y_MAX] == BC_PRESSURE_RELEASE ? 0.5 : 1.0;
    double kinetic = 0.0;
    const size_t fw = (size_t)s->nx + 1;
    for (int j = 0; j < s->ny; j++) {
        const size_t row = (size_t)j * fw;
        for (size_t i = 0; i <= (size_t)s->nx; i++) {
            const size_t f = row + i;
            if (s->bx[f] > 0.0) {
                const double w = i == 0 ? w_xmin : (i == (size_t)s->nx ? w_xmax : 1.0);
                kinetic += w * (m->dt / s->bx[f]) * s->vx[f] * s->vx[f];
            }
        }
    }
    for (int j = 0; j <= s->ny; j++) {
        const size_t row = (size_t)j * (size_t)s->nx;
        const double w = j == 0 ? w_ymin : (j == s->ny ? w_ymax : 1.0);
        for (size_t i = 0; i < (size_t)s->nx; i++) {
            if (s->by[row + i] > 0.0) {
                kinetic += w * (m->dt / s->by[row + i]) * s->vy[row + i] * s->vy[row + i];
            }
        }
    }
    return (potential + 0.5 * kinetic) * m->grid.dx * m->grid.dy;
}
