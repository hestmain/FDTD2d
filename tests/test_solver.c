/*
 * test_solver.c - unit tests for the FDTD time stepper.
 *
 * Models are built from TOML text (config_load_string + model_build). Grids
 * are small (dx = 1 cm, air) so the whole file runs in a few seconds.
 * Physics checks print the numbers they measured on lines starting with
 * "  [measure]".
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "boundary.h"
#include "common.h"
#include "config.h"
#include "grid.h"
#include "model.h"
#include "solver.h"

#include "test_util.h"

#define RHO 1.21
#define CSOUND 343.0
#define DX 0.01

/* ------------------------------------------------------------- fixtures */

typedef struct CaseSpec {
    int nx, ny;                    /* physical cells */
    BoundaryType bc[SIDE_COUNT];
    int pml_thickness;
    int has_source;
    double src_x, src_y;           /* [m] */
    double freq, amp, ramp_cycles;
    const char *extra;             /* appended TOML (obstacles) */
} CaseSpec;

static CaseSpec default_spec(int nx, int ny)
{
    CaseSpec c;
    memset(&c, 0, sizeof c);
    c.nx = nx;
    c.ny = ny;
    for (int s = 0; s < SIDE_COUNT; s++) {
        c.bc[s] = BC_RIGID;
    }
    c.pml_thickness = 20;
    c.has_source = 1;
    c.src_x = 0.5 * nx * DX;
    c.src_y = 0.5 * ny * DX;
    c.freq = 1500.0;
    c.amp = 1e-3;
    c.ramp_cycles = 1.0;
    c.extra = "";
    return c;
}

static void spec_all(CaseSpec *c, BoundaryType bc)
{
    for (int s = 0; s < SIDE_COUNT; s++) {
        c->bc[s] = bc;
    }
}

static void spec_to_toml(const CaseSpec *c, char *buf, size_t n)
{
    int w = snprintf(buf, n,
        "[simulation]\nduration = 0.1\ncfl = 0.7\n"
        "[grid]\nx_range = [0, %.17g]\ny_range = [0, %.17g]\ndx = %.17g\n"
        "[medium]\ndensity = %.17g\nc = %.17g\n"
        "[boundary]\nx_min = \"%s\"\nx_max = \"%s\"\ny_min = \"%s\"\ny_max = \"%s\"\n"
        "[boundary.pml]\nthickness = %d\nreflection = 1e-6\norder = 2\n",
        c->nx * DX, c->ny * DX, DX, RHO, CSOUND,
        boundary_type_name(c->bc[SIDE_X_MIN]), boundary_type_name(c->bc[SIDE_X_MAX]),
        boundary_type_name(c->bc[SIDE_Y_MIN]), boundary_type_name(c->bc[SIDE_Y_MAX]),
        c->pml_thickness);
    if (c->has_source && w > 0 && (size_t)w < n) {
        w += snprintf(buf + w, n - (size_t)w,
            "[[source]]\nshape = { type = \"point\", position = [%.17g, %.17g] }\n"
            "signal = { type = \"harmonic\", frequency = %.17g, amplitude = %.17g, "
            "phase = 0, ramp_cycles = %.17g }\n",
            c->src_x, c->src_y, c->freq, c->amp, c->ramp_cycles);
    }
    if (w > 0 && (size_t)w < n) {
        snprintf(buf + w, n - (size_t)w, "%s", c->extra);
    }
}

typedef struct Fixture {
    Model model;
    Solver solver;
    int n_sources_built; /* restored before model_free */
    int open;
} Fixture;

/* Returns 0 on success; on failure err holds the message and nothing is open. */
static int fixture_try(Fixture *f, const CaseSpec *c, FdtdError *err)
{
    char text[4096];
    SimConfig cfg;
    memset(f, 0, sizeof *f);
    spec_to_toml(c, text, sizeof text);
    if (config_load_string(text, "test_solver", &cfg, err) != 0) {
        return -1;
    }
    int rc = model_build(&cfg, &f->model, err);
    config_free(&cfg);
    if (rc != 0) {
        return -1;
    }
    f->n_sources_built = f->model.n_sources;
    if (solver_init(&f->solver, &f->model, err) != 0) {
        model_free(&f->model);
        return -1;
    }
    f->open = 1;
    return 0;
}

static int fixture_open(Fixture *f, const CaseSpec *c)
{
    FdtdError err;
    memset(&err, 0, sizeof err);
    if (fixture_try(f, c, &err) != 0) {
        fprintf(stderr, "fixture: %s\n", err.msg);
        CHECK_MSG(0, "fixture could not be built");
        return -1;
    }
    return 0;
}

static void fixture_close(Fixture *f)
{
    if (!f->open) {
        return;
    }
    solver_free(&f->solver);
    f->model.n_sources = f->n_sources_built;
    model_free(&f->model);
    f->open = 0;
}

static void run(Solver *s, long steps)
{
    for (long k = 0; k < steps; k++) {
        solver_step(s);
    }
}

static double max_abs_p(const Solver *s)
{
    double m = 0.0;
    const size_t n = (size_t)s->nx * (size_t)s->ny;
    for (size_t c = 0; c < n; c++) {
        const double p = fabs(solver_pressure_at(s, c));
        if (p > m) {
            m = p;
        }
    }
    return m;
}

static int all_finite(const Solver *s)
{
    const size_t n = (size_t)s->nx * (size_t)s->ny;
    for (size_t c = 0; c < n; c++) {
        if (!isfinite(s->px[c]) || !isfinite(s->py[c])) {
            return 0;
        }
    }
    for (size_t f = 0; f < (size_t)(s->nx + 1) * (size_t)s->ny; f++) {
        if (!isfinite(s->vx[f])) {
            return 0;
        }
    }
    for (size_t f = 0; f < (size_t)s->nx * (size_t)(s->ny + 1); f++) {
        if (!isfinite(s->vy[f])) {
            return 0;
        }
    }
    return 1;
}

/* Exact conserved quantity of the leapfrog scheme in a lossless closed
 * domain with rigid walls: sum p^n p^(n+1) / (2K) + sum rho_face v^2 / 2,
 * with v at n+1/2 (p_prev = p^n, solver holds p^(n+1)). */
static double leapfrog_invariant(const Solver *s, const double *p_prev)
{
    const double dt = s->model->dt;
    const size_t n = (size_t)s->nx * (size_t)s->ny;
    double pot = 0.0, kin = 0.0;
    for (size_t c = 0; c < n; c++) {
        if (s->kdt[c] > 0.0) {
            pot += p_prev[c] * solver_pressure_at(s, c) / (2.0 * (s->kdt[c] / dt));
        }
    }
    for (size_t f = 0; f < (size_t)(s->nx + 1) * (size_t)s->ny; f++) {
        if (s->bx[f] > 0.0) {
            kin += 0.5 * (dt / s->bx[f]) * s->vx[f] * s->vx[f];
        }
    }
    for (size_t f = 0; f < (size_t)s->nx * (size_t)(s->ny + 1); f++) {
        if (s->by[f] > 0.0) {
            kin += 0.5 * (dt / s->by[f]) * s->vy[f] * s->vy[f];
        }
    }
    return (pot + kin) * s->model->grid.dx * s->model->grid.dy;
}

/* ----------------------------------------------------------------- tests */

static void test_no_sources_stays_zero(void)
{
    CaseSpec c = default_spec(40, 30);
    c.has_source = 0;
    spec_all(&c, BC_PML);
    c.pml_thickness = 5;
    c.bc[SIDE_Y_MIN] = BC_PRESSURE_RELEASE; /* exercise every boundary kind */
    c.bc[SIDE_X_MAX] = BC_RIGID;
    Fixture f;
    if (fixture_open(&f, &c) != 0) {
        return;
    }
    run(&f.solver, 100);
    CHECK(f.solver.step == 100);
    CHECK(f.solver.time == 100.0 * f.model.dt);
    int nonzero = 0;
    for (size_t k = 0; k < (size_t)f.solver.nx * (size_t)f.solver.ny; k++) {
        nonzero += f.solver.px[k] != 0.0 || f.solver.py[k] != 0.0;
    }
    for (size_t k = 0; k < (size_t)(f.solver.nx + 1) * (size_t)f.solver.ny; k++) {
        nonzero += f.solver.vx[k] != 0.0;
    }
    for (size_t k = 0; k < (size_t)f.solver.nx * (size_t)(f.solver.ny + 1); k++) {
        nonzero += f.solver.vy[k] != 0.0;
    }
    CHECK(nonzero == 0);
    CHECK(solver_acoustic_energy(&f.solver) == 0.0);
    fixture_close(&f);
}

static void test_source_amplitude_first_step(void)
{
    /* One step from rest: p = K dt w s(dt/2); sum(p) dx dy = K dt s(dt/2). */
    CaseSpec c = default_spec(21, 21);
    c.ramp_cycles = 0.0;
    c.src_x = 0.0537; /* off-centre: several cells carry weight */
    c.src_y = 0.1181;
    Fixture f;
    if (fixture_open(&f, &c) != 0) {
        return;
    }
    solver_step(&f.solver);
    const double dt = f.model.dt;
    const double K = RHO * CSOUND * CSOUND;
    const double s_val = c.amp * sin(2.0 * M_PI * c.freq * 0.5 * dt);
    double sum = 0.0;
    const size_t n = (size_t)f.solver.nx * (size_t)f.solver.ny;
    for (size_t k = 0; k < n; k++) {
        sum += solver_pressure_at(&f.solver, k);
    }
    sum *= DX * DX;
    CHECK_NEAR(sum, K * dt * s_val, 1e-12 * fabs(K * dt * s_val));
    /* px and py receive equal shares. */
    double diff = 0.0;
    for (size_t k = 0; k < n; k++) {
        diff = fmax(diff, fabs(f.solver.px[k] - f.solver.py[k]));
    }
    CHECK(diff == 0.0);
    fixture_close(&f);
}

static void test_symmetry(void)
{
    const int n = 61;
    CaseSpec c = default_spec(n, n);
    c.src_x = (n / 2 + 0.5) * DX; /* centre of the middle cell */
    c.src_y = (n / 2 + 0.5) * DX;
    Fixture f;
    if (fixture_open(&f, &c) != 0) {
        return;
    }
    CHECK(f.solver.nx == n && f.solver.ny == n);
    run(&f.solver, 300); /* several wall reflections */
    const double pmax = max_abs_p(&f.solver);
    CHECK(pmax > 0.0);
    double worst = 0.0;
    for (int j = 0; j < n; j++) {
        for (int i = 0; i < n; i++) {
            const double p = solver_pressure_at(&f.solver, (size_t)j * n + (size_t)i);
            const double px = solver_pressure_at(&f.solver, (size_t)j * n + (size_t)(n - 1 - i));
            const double py = solver_pressure_at(&f.solver, (size_t)(n - 1 - j) * n + (size_t)i);
            const double pt = solver_pressure_at(&f.solver, (size_t)i * n + (size_t)j);
            worst = fmax(worst, fmax(fabs(p - px), fmax(fabs(p - py), fabs(p - pt))));
        }
    }
    printf("  [measure] symmetry: max asymmetry / max|p| = %.3g (max|p| = %.3g Pa)\n",
           worst / pmax, pmax);
    CHECK(worst <= 1e-12 * pmax);
    fixture_close(&f);
}

static void test_energy_conservation_rigid_box(void)
{
    CaseSpec c = default_spec(51, 41);
    c.src_x = 0.1737;
    c.src_y = 0.1263;
    Fixture f;
    if (fixture_open(&f, &c) != 0) {
        return;
    }
    const double period_steps = 1.0 / (c.freq * f.model.dt);
    const long drive = (long)(4.0 * period_steps);
    run(&f.solver, drive);
    const size_t n_cells = (size_t)f.solver.nx * (size_t)f.solver.ny;
    double *p_prev = malloc(n_cells * sizeof *p_prev);
    const long n = 4000;
    double *e = malloc((size_t)n * sizeof *e);
    CHECK(p_prev != NULL && e != NULL);
    if (!p_prev || !e) {
        free(p_prev);
        free(e);
        fixture_close(&f);
        return;
    }
    const int saved = f.model.n_sources;
    f.model.n_sources = 0;
    solver_get_pressure(&f.solver, p_prev);
    solver_step(&f.solver);
    const double e_on = solver_acoustic_energy(&f.solver);
    const double inv0 = leapfrog_invariant(&f.solver, p_prev);
    CHECK(e_on > 0.0);

    double emin = 1e300, emax = 0.0, inv_dev = 0.0;
    for (long k = 0; k < n; k++) {
        solver_get_pressure(&f.solver, p_prev);
        solver_step(&f.solver);
        e[k] = solver_acoustic_energy(&f.solver);
        emin = fmin(emin, e[k]);
        emax = fmax(emax, e[k]);
        inv_dev = fmax(inv_dev, fabs(leapfrog_invariant(&f.solver, p_prev) / inv0 - 1.0));
    }
    double head = 0.0, tail = 0.0;
    const long w = 500;
    for (long k = 0; k < w; k++) {
        head += e[k];
        tail += e[n - w + k];
    }
    head /= (double)w;
    tail /= (double)w;
    printf("  [measure] rigid box: E(switch-off) = %.4g J/m, min/E0 = %.5f, "
           "max/E0 = %.5f, mean drift (last 500 vs first 500 steps) = %.3g, "
           "exact leapfrog invariant deviation = %.3g\n",
           e_on, emin / e_on, emax / e_on, tail / head - 1.0, inv_dev);
    /* solver_acoustic_energy mixes p^n with v^(n+1/2), so it oscillates by a
     * few percent (more with broadband content from the abrupt switch-off),
     * but has no drift. The staggered invariant is conserved to rounding. */
    CHECK(inv_dev < 1e-11);
    CHECK(emin >= 0.88 * e_on);
    CHECK(emax <= 1.12 * e_on);
    CHECK(fabs(tail / head - 1.0) < 1e-3);
    CHECK(all_finite(&f.solver));
    free(e);
    free(p_prev);
    f.model.n_sources = saved;
    fixture_close(&f);
}

static void test_pressure_release(void)
{
    /* Release at x_min, rigid at x_max, source in the middle (equal distance
     * to both). PML in y keeps the field one-dimensional enough. */
    const int nxp = 161, nyp = 41;
    CaseSpec c = default_spec(nxp, nyp);
    c.bc[SIDE_X_MIN] = BC_PRESSURE_RELEASE;
    c.bc[SIDE_X_MAX] = BC_RIGID;
    c.bc[SIDE_Y_MIN] = BC_PML;
    c.bc[SIDE_Y_MAX] = BC_PML;
    c.pml_thickness = 10;
    Fixture f;
    if (fixture_open(&f, &c) != 0) {
        return;
    }
    const Grid *g = &f.model.grid;
    CHECK(g->nx == nxp);
    const int jc = g->off_y + nyp / 2;
    const size_t cell_release = grid_cell_index(g, 0, jc);
    const size_t cell_rigid = grid_cell_index(g, g->nx - 1, jc);
    /* Stop before the wave reflected by the far wall returns to either probe. */
    const long steps = 240;
    double pmax_release = 0.0, pmax_rigid = 0.0;
    for (long k = 0; k < steps; k++) {
        solver_step(&f.solver);
        pmax_release = fmax(pmax_release, fabs(solver_pressure_at(&f.solver, cell_release)));
        pmax_rigid = fmax(pmax_rigid, fabs(solver_pressure_at(&f.solver, cell_rigid)));
    }
    printf("  [measure] pressure release: max|p| next to release side = %.4g Pa, "
           "next to rigid side = %.4g Pa, ratio = %.3f\n",
           pmax_release, pmax_rigid, pmax_release / pmax_rigid);
    CHECK(pmax_rigid > 0.0);
    CHECK(pmax_release < 0.25 * pmax_rigid);
    CHECK(all_finite(&f.solver));
    fixture_close(&f);
}

static void test_pressure_release_all_sides(void)
{
    /* Release on one side, rigid elsewhere, off-centre source. The problem
     * for x_max, y_min and y_max is the mirror / transpose of the x_min one,
     * so the fields must agree exactly (to rounding). */
    const int n = 31;
    const double xs = 0.0737, ys = 0.1531; /* source offset from x_min / y_min */
    const double len = n * DX;
    const Side sides[4] = {SIDE_X_MIN, SIDE_X_MAX, SIDE_Y_MIN, SIDE_Y_MAX};
    double *field[4] = {NULL, NULL, NULL, NULL};
    int ok = 1;
    for (int k = 0; k < 4 && ok; k++) {
        CaseSpec c = default_spec(n, n);
        c.bc[sides[k]] = BC_PRESSURE_RELEASE;
        switch (sides[k]) {
        case SIDE_X_MIN: c.src_x = xs;       c.src_y = ys;       break;
        case SIDE_X_MAX: c.src_x = len - xs; c.src_y = ys;       break;
        case SIDE_Y_MIN: c.src_x = ys;       c.src_y = xs;       break;
        default:         c.src_x = ys;       c.src_y = len - xs; break;
        }
        Fixture f;
        if (fixture_open(&f, &c) != 0) {
            ok = 0;
            break;
        }
        run(&f.solver, 250);
        field[k] = malloc((size_t)n * (size_t)n * sizeof(double));
        if (!field[k]) {
            ok = 0;
        } else {
            solver_get_pressure(&f.solver, field[k]);
        }
        fixture_close(&f);
    }
    CHECK(ok);
    if (ok) {
        double pmax = 0.0;
        for (int q = 0; q < n * n; q++) {
            pmax = fmax(pmax, fabs(field[0][q]));
        }
        CHECK(pmax > 0.0);
        double worst[3] = {0.0, 0.0, 0.0};
        for (int j = 0; j < n; j++) {
            for (int i = 0; i < n; i++) {
                const double ref = field[0][j * n + i];
                worst[0] = fmax(worst[0], fabs(ref - field[1][j * n + (n - 1 - i)]));
                worst[1] = fmax(worst[1], fabs(ref - field[2][i * n + j]));
                worst[2] = fmax(worst[2], fabs(ref - field[3][(n - 1 - i) * n + j]));
            }
        }
        printf("  [measure] release on each side vs x_min run (rel. diff): x_max %.2g, "
               "y_min %.2g, y_max %.2g\n", worst[0] / pmax, worst[1] / pmax, worst[2] / pmax);
        for (int k = 0; k < 3; k++) {
            CHECK(worst[k] <= 1e-12 * pmax);
        }
    }
    for (int k = 0; k < 4; k++) {
        free(field[k]);
    }
}

static void test_pressure_release_energy_bounded(void)
{
    CaseSpec c = default_spec(51, 41);
    c.bc[SIDE_X_MIN] = BC_PRESSURE_RELEASE;
    c.bc[SIDE_Y_MAX] = BC_PRESSURE_RELEASE;
    c.src_x = 0.2137;
    c.src_y = 0.1663;
    Fixture f;
    if (fixture_open(&f, &c) != 0) {
        return;
    }
    const long drive = (long)(4.0 / (c.freq * f.model.dt));
    run(&f.solver, drive);
    const double e_on = solver_acoustic_energy(&f.solver);
    CHECK(e_on > 0.0);
    const int saved = f.model.n_sources;
    f.model.n_sources = 0;
    double emin = 1e300, emax = 0.0;
    for (long k = 0; k < 4000; k++) {
        solver_step(&f.solver);
        const double e = solver_acoustic_energy(&f.solver);
        emin = fmin(emin, e);
        emax = fmax(emax, e);
    }
    printf("  [measure] release box: E(switch-off) = %.4g J/m, min/E0 = %.4f, "
           "max/E0 = %.4f\n", e_on, emin / e_on, emax / e_on);
    CHECK(emin >= 0.9 * e_on);
    CHECK(emax <= 1.1 * e_on);
    CHECK(all_finite(&f.solver));
    f.model.n_sources = saved;
    fixture_close(&f);
}

static void test_pml_absorption(void)
{
    CaseSpec c = default_spec(60, 60);
    spec_all(&c, BC_PML);
    c.pml_thickness = 20;
    Fixture f;
    if (fixture_open(&f, &c) != 0) {
        return;
    }
    CHECK(f.solver.nx == 100 && f.solver.ny == 100);
    const long drive = (long)(3.0 / (c.freq * f.model.dt));
    const long total = 1500;
    double e_peak = 0.0;
    int switched = 0;
    const int saved = f.model.n_sources;
    for (long k = 0; k < total; k++) {
        if (k == drive) {
            f.model.n_sources = 0;
            switched = 1;
        }
        solver_step(&f.solver);
        e_peak = fmax(e_peak, solver_acoustic_energy(&f.solver));
    }
    CHECK(switched);
    const double e_end = solver_acoustic_energy(&f.solver);
    printf("  [measure] PML: peak energy = %.4g J/m, energy after %ld steps = %.4g J/m, "
           "ratio = %.3g\n", e_peak, total, e_end, e_end / e_peak);
    CHECK(e_peak > 0.0);
    CHECK(e_end < 1e-3 * e_peak);
    CHECK(all_finite(&f.solver));
    f.model.n_sources = saved;
    fixture_close(&f);
}

static void test_obstacles_stay_rigid(void)
{
    CaseSpec c = default_spec(80, 60);
    c.bc[SIDE_X_MIN] = BC_PML;
    c.bc[SIDE_X_MAX] = BC_PML;
    c.bc[SIDE_Y_MIN] = BC_PRESSURE_RELEASE;
    c.bc[SIDE_Y_MAX] = BC_RIGID;
    c.pml_thickness = 10;
    c.src_x = 0.20;
    c.src_y = 0.30;
    c.extra =
        "[[obstacle]]\nshape = { type = \"rect\", min = [0.30, 0.0], max = [0.40, 0.20] }\n"
        "[[obstacle]]\nshape = { type = \"circle\", center = [0.55, 0.35], radius = 0.10 }\n"
        "[[obstacle]]\nshape = { type = \"rect\", min = [0.10, 0.50], max = [0.20, 0.60] }\n";
    Fixture f;
    if (fixture_open(&f, &c) != 0) {
        return;
    }
    const Model *m = &f.model;
    const Solver *s = &f.solver;
    CHECK(m->n_solid > 0);
    run(&f.solver, 400);
    CHECK(max_abs_p(s) > 0.0);

    const int nx = s->nx, ny = s->ny;
    size_t bad_p = 0, bad_vx = 0, bad_vy = 0, n_checked = 0;
    for (int j = 0; j < ny; j++) {
        for (int i = 0; i < nx; i++) {
            const size_t cell = (size_t)j * (size_t)nx + (size_t)i;
            if (!m->solid[cell]) {
                continue;
            }
            n_checked++;
            bad_p += s->px[cell] != 0.0 || s->py[cell] != 0.0;
            bad_vx += s->vx[(size_t)j * (size_t)(nx + 1) + (size_t)i] != 0.0 ||
                      s->vx[(size_t)j * (size_t)(nx + 1) + (size_t)i + 1] != 0.0;
            bad_vy += s->vy[(size_t)j * (size_t)nx + (size_t)i] != 0.0 ||
                      s->vy[(size_t)(j + 1) * (size_t)nx + (size_t)i] != 0.0;
        }
    }
    CHECK(n_checked == m->n_solid);
    CHECK(bad_p == 0);
    CHECK(bad_vx == 0);
    CHECK(bad_vy == 0);
    CHECK(all_finite(s));
    fixture_close(&f);
}

/* Arrival analysis of a pressure series sampled after steps 1..n: time (in
 * steps, linearly interpolated) at which the series first reaches `frac` of
 * its first local maximum, and the (parabolically interpolated) time of that
 * maximum. Returns -1 if there is no maximum. */
static int analyse_front(const double *series, long n, double frac, double *t_front,
                         double *t_peak)
{
    long k = 1;
    while (k + 1 < n && !(series[k] > series[k - 1] && series[k] >= series[k + 1])) {
        k++;
    }
    if (k + 1 >= n || !(series[k] > 0.0)) {
        return -1;
    }
    const double a = series[k - 1], b = series[k], d = series[k + 1];
    const double den = a - 2.0 * b + d;
    *t_peak = (double)(k + 1) + (den != 0.0 ? 0.5 * (a - d) / den : 0.0);
    const double level = frac * b;
    long m = 0;
    while (m < k && series[m] < level) {
        m++;
    }
    if (m == 0) {
        *t_front = 1.0;
    } else {
        /* sample m-1 below the level, sample m at or above; step = index + 1 */
        const double u = (level - series[m - 1]) / (series[m] - series[m - 1]);
        *t_front = (double)m + u;
    }
    return 0;
}

static void test_propagation_speed(void)
{
    const int nphys = 201;
    CaseSpec c = default_spec(nphys, nphys);
    spec_all(&c, BC_PML);
    c.pml_thickness = 10;
    c.ramp_cycles = 0.0; /* sharp front */
    c.src_x = (nphys / 2 + 0.5) * DX;
    c.src_y = (nphys / 2 + 0.5) * DX;
    Fixture f;
    if (fixture_open(&f, &c) != 0) {
        return;
    }
    const Grid *g = &f.model.grid;
    const int ic = g->off_x + nphys / 2;
    const int jc = g->off_y + nphys / 2;
    const double dt = f.model.dt;

    enum { NPROBE = 3 };
    const int off[NPROBE][2] = {{30, 0}, {90, 0}, {60, 60}}; /* near, far, diagonal */
    const char *const name[NPROBE] = {"axis 30 cells", "axis 90 cells", "diagonal 85 cells"};
    size_t cells[NPROBE];
    double expect[NPROBE]; /* r / (c dt) in steps */
    for (int k = 0; k < NPROBE; k++) {
        cells[k] = grid_cell_index(g, ic + off[k][0], jc + off[k][1]);
        expect[k] = DX * sqrt((double)(off[k][0] * off[k][0] + off[k][1] * off[k][1])) /
                    (CSOUND * dt);
    }
    const long total = 250; /* far probe: arrival ~182, first peak ~190 */
    double *series[NPROBE];
    int ok = 1;
    for (int k = 0; k < NPROBE; k++) {
        series[k] = calloc((size_t)total, sizeof(double));
        ok = ok && series[k] != NULL;
    }
    CHECK(ok);
    if (ok) {
        for (long n = 0; n < total; n++) {
            solver_step(&f.solver); /* now at step n + 1 */
            for (int k = 0; k < NPROBE; k++) {
                series[k][n] = solver_pressure_at(&f.solver, cells[k]);
            }
        }
        double t_front[NPROBE] = {0}, t_peak[NPROBE] = {0};
        for (int k = 0; k < NPROBE; k++) {
            const int rc = analyse_front(series[k], total, 0.1, &t_front[k], &t_peak[k]);
            CHECK(rc == 0);
            if (rc != 0) {
                continue;
            }
            printf("  [measure] %-17s: r/(c dt) = %7.2f steps, 10%% front at %7.2f "
                   "(error %+.2f), first peak at %7.2f (offset %+.2f)\n", name[k], expect[k],
                   t_front[k], t_front[k] - expect[k], t_peak[k], t_peak[k] - expect[k]);
            /* The front of a switched-on 2D source is smooth (finite source
             * cell, pulse spreading), so it is only located to a few steps
             * (<= 4 of 60..180). The phase-delay test below is the tight one. */
            CHECK(fabs(t_front[k] - expect[k]) <= 4.0);
        }
    }
    for (int k = 0; k < NPROBE; k++) {
        free(series[k]);
    }
    fixture_close(&f);
}

/* Time (in steps, linearly interpolated) of the first upward zero crossing of
 * series[] (sample k belongs to step k + 1) at or after sample `from`. */
static double upward_zero_crossing(const double *series, long from, long n)
{
    for (long k = from; k + 1 < n; k++) {
        if (series[k] < 0.0 && series[k + 1] >= 0.0) {
            return (double)(k + 1) + (-series[k]) / (series[k + 1] - series[k]);
        }
    }
    return -1.0;
}

static void test_phase_velocity(void)
{
    /* Steady state of a ramped harmonic source: the phase delay between two
     * probes on the same ray gives the numerical phase speed. */
    const int nphys = 201;
    CaseSpec c = default_spec(nphys, nphys);
    spec_all(&c, BC_PML);
    c.pml_thickness = 10;
    c.ramp_cycles = 2.0;
    c.src_x = (nphys / 2 + 0.5) * DX;
    c.src_y = (nphys / 2 + 0.5) * DX;
    Fixture f;
    if (fixture_open(&f, &c) != 0) {
        return;
    }
    const Grid *g = &f.model.grid;
    const int ic = g->off_x + nphys / 2;
    const int jc = g->off_y + nphys / 2;
    const double dt = f.model.dt;
    const double period = 1.0 / (c.freq * dt); /* steps */

    enum { NR = 4 };
    /* axis near/far and the same radii along the diagonal (30,30)->(60,60) */
    const int off[NR][2] = {{30, 0}, {90, 0}, {21, 21}, {63, 63}};
    size_t cells[NR];
    double r[NR];
    for (int k = 0; k < NR; k++) {
        cells[k] = grid_cell_index(g, ic + off[k][0], jc + off[k][1]);
        r[k] = DX * sqrt((double)(off[k][0] * off[k][0] + off[k][1] * off[k][1]));
    }
    const long total = 480;
    double *series[NR];
    int ok = 1;
    for (int k = 0; k < NR; k++) {
        series[k] = calloc((size_t)total, sizeof(double));
        ok = ok && series[k] != NULL;
    }
    CHECK(ok);
    if (ok) {
        for (long n = 0; n < total; n++) {
            solver_step(&f.solver);
            for (int k = 0; k < NR; k++) {
                series[k][n] = solver_pressure_at(&f.solver, cells[k]);
            }
        }
        for (int pair = 0; pair < 2; pair++) {
            const int a = 2 * pair, b = 2 * pair + 1;
            const double d_exp = (r[b] - r[a]) / (CSOUND * dt); /* steps */
            /* Steady state at the far probe starts after arrival + ramp + a
             * period; look at the crossings in the last stretch. */
            const long start_b = (long)(r[b] / (CSOUND * dt) + 2.0 * period + period + 5.0);
            const double tb = upward_zero_crossing(series[b], start_b, total);
            CHECK(tb > 0.0);
            if (!(tb > 0.0)) {
                continue;
            }
            /* The near crossing closest to tb - d_exp (mod one period). */
            const double target = tb - d_exp;
            const long from = (long)(target - period) - 1;
            double ta = upward_zero_crossing(series[a], from > 1 ? from : 1, total);
            double best = ta;
            for (int tries = 0; tries < 3 && ta > 0.0; tries++) {
                if (fabs(ta - target) < fabs(best - target)) {
                    best = ta;
                }
                ta = upward_zero_crossing(series[a], (long)ta + 2, total);
            }
            CHECK(best > 0.0);
            const double fold = fmod((tb - best) - d_exp, period); /* error, steps */
            const double err = fabs(fold) > 0.5 * period ? fold - copysign(period, fold) : fold;
            printf("  [measure] phase delay %s ray, r = %.3f -> %.3f m: expected %.2f steps, "
                   "error %+.3f steps (%+.3f%% of the delay)\n", pair == 0 ? "axis" : "diagonal",
                   r[a], r[b], d_exp, err, 100.0 * err / d_exp);
            CHECK(fabs(err) <= 1.0);
        }
    }
    for (int k = 0; k < NR; k++) {
        free(series[k]);
    }
    fixture_close(&f);
}

static void test_coefficients(void)
{
    /* Damping tables follow a = (1 - h)/(1 + h), c = 1/(1 + h), h = sigma dt/2;
     * outer faces of pressure_release sides are undamped; v = 0 outer faces
     * of rigid/pml sides have no coefficient (bx = by = 0). */
    CaseSpec c = default_spec(30, 24);
    c.bc[SIDE_X_MIN] = BC_PML;
    c.bc[SIDE_X_MAX] = BC_PRESSURE_RELEASE;
    c.bc[SIDE_Y_MIN] = BC_PML;
    c.bc[SIDE_Y_MAX] = BC_RIGID;
    c.pml_thickness = 8;
    Fixture f;
    if (fixture_open(&f, &c) != 0) {
        return;
    }
    const Solver *s = &f.solver;
    const Model *m = &f.model;
    const double dt = m->dt;
    double worst = 0.0, sigma_max = 0.0;
    for (int i = 0; i < s->nx; i++) {
        const double h = 0.5 * m->sigma_x_cell[i] * dt;
        worst = fmax(worst, fabs(s->ax_c[i] - (1.0 - h) / (1.0 + h)));
        worst = fmax(worst, fabs(s->cx_c[i] - 1.0 / (1.0 + h)));
        sigma_max = fmax(sigma_max, m->sigma_x_cell[i]);
    }
    for (int j = 0; j < s->ny; j++) {
        const double h = 0.5 * m->sigma_y_cell[j] * dt;
        worst = fmax(worst, fabs(s->ay_c[j] - (1.0 - h) / (1.0 + h)));
        worst = fmax(worst, fabs(s->cy_c[j] - 1.0 / (1.0 + h)));
    }
    for (int i = 1; i < s->nx; i++) {
        const double h = 0.5 * m->sigma_x_face[i] * dt;
        worst = fmax(worst, fabs(s->ax_f[i] - (1.0 - h) / (1.0 + h)));
        worst = fmax(worst, fabs(s->cx_f[i] - 1.0 / (1.0 + h)));
    }
    for (int j = 1; j < s->ny; j++) {
        const double h = 0.5 * m->sigma_y_face[j] * dt;
        worst = fmax(worst, fabs(s->ay_f[j] - (1.0 - h) / (1.0 + h)));
        worst = fmax(worst, fabs(s->cy_f[j] - 1.0 / (1.0 + h)));
    }
    CHECK(sigma_max > 0.0);
    CHECK(worst < 1e-15);
    CHECK(s->ax_f[s->nx] == 1.0 && s->cx_f[s->nx] == 1.0);
    /* Outer faces: only the release side (x_max) is active. */
    const size_t fw = (size_t)s->nx + 1;
    int active_xmin = 0, active_xmax = 0, active_ymin = 0, active_ymax = 0;
    for (int j = 0; j < s->ny; j++) {
        active_xmin += s->bx[(size_t)j * fw] != 0.0;
        active_xmax += s->bx[(size_t)j * fw + (size_t)s->nx] != 0.0;
    }
    for (int i = 0; i < s->nx; i++) {
        active_ymin += s->by[i] != 0.0;
        active_ymax += s->by[(size_t)s->ny * (size_t)s->nx + (size_t)i] != 0.0;
    }
    CHECK(active_xmin == 0 && active_ymin == 0 && active_ymax == 0);
    CHECK(active_xmax == s->ny);
    fixture_close(&f);
}

static void test_two_media(void)
{
    /* Heavier, slower medium in the right half: rho_face is the arithmetic
     * mean at the interface, K per cell, and a closed box still conserves
     * the staggered invariant to rounding. */
    CaseSpec c = default_spec(40, 30);
    Fixture f;
    if (fixture_open(&f, &c) != 0) {
        return;
    }
    solver_free(&f.solver);
    const Medium heavy = {1000.0, 300.0}; /* c <= c_max: the model's dt stays stable */
    FdtdError err;
    memset(&err, 0, sizeof err);
    const int id = medium_table_add(&f.model.media, &heavy, &err);
    CHECK(id == 1);
    if (id != 1) {
        f.open = 0;
        model_free(&f.model);
        return;
    }
    const Grid *g = &f.model.grid;
    const int i_if = g->nx / 2; /* first cell of the heavy medium */
    for (int j = 0; j < g->ny; j++) {
        for (int i = i_if; i < g->nx; i++) {
            f.model.medium_id[grid_cell_index(g, i, j)] = (MediumId)id;
        }
    }
    CHECK(solver_init(&f.solver, &f.model, &err) == 0);
    const Solver *s = &f.solver;
    const double dt = f.model.dt;
    const int j0 = 7;
    const size_t fw = (size_t)g->nx + 1;
    CHECK_NEAR(s->bx[(size_t)j0 * fw + 5] * RHO / dt, 1.0, 1e-14);
    CHECK_NEAR(s->bx[(size_t)j0 * fw + (size_t)(g->nx - 3)] * 1000.0 / dt, 1.0, 1e-14);
    CHECK_NEAR(s->bx[(size_t)j0 * fw + (size_t)i_if] * 0.5 * (RHO + 1000.0) / dt, 1.0, 1e-14);
    CHECK_NEAR(s->by[(size_t)j0 * (size_t)g->nx + (size_t)(i_if - 1)] * RHO / dt, 1.0, 1e-14);
    CHECK_NEAR(s->by[(size_t)j0 * (size_t)g->nx + (size_t)i_if] * 1000.0 / dt, 1.0, 1e-14);
    CHECK_NEAR(s->kdt[grid_cell_index(g, 3, 3)], RHO * CSOUND * CSOUND * dt, 1e-9);
    CHECK_NEAR(s->kdt[grid_cell_index(g, g->nx - 2, 3)], 1000.0 * 300.0 * 300.0 * dt, 1e-6);

    const size_t n_cells = (size_t)s->nx * (size_t)s->ny;
    double *p_prev = malloc(n_cells * sizeof *p_prev);
    CHECK(p_prev != NULL);
    if (p_prev) {
        run(&f.solver, (long)(3.0 / (c.freq * dt)));
        const int saved = f.model.n_sources;
        f.model.n_sources = 0;
        solver_get_pressure(&f.solver, p_prev);
        solver_step(&f.solver);
        const double inv0 = leapfrog_invariant(&f.solver, p_prev);
        CHECK(inv0 > 0.0);
        double dev = 0.0;
        for (int k = 0; k < 1500; k++) {
            solver_get_pressure(&f.solver, p_prev);
            solver_step(&f.solver);
            dev = fmax(dev, fabs(leapfrog_invariant(&f.solver, p_prev) / inv0 - 1.0));
        }
        printf("  [measure] two media: invariant deviation = %.3g\n", dev);
        CHECK(dev < 1e-11);
        f.model.n_sources = saved;
        free(p_prev);
    }
    fixture_close(&f);
}

static void test_degenerate_sizes(void)
{
    /* nx_phys = 1 or ny_phys = 1: must work or fail with a message. */
    const BoundaryType kinds[3] = {BC_RIGID, BC_PRESSURE_RELEASE, BC_PML};
    const int shapes[3][2] = {{1, 1}, {1, 9}, {9, 1}};
    int ran = 0, failed = 0;
    for (int sh = 0; sh < 3; sh++) {
        for (int kd = 0; kd < 3; kd++) {
            CaseSpec c = default_spec(shapes[sh][0], shapes[sh][1]);
            spec_all(&c, kinds[kd]);
            c.pml_thickness = 3;
            c.freq = 400.0; /* resolve the wavelength; keeps warnings out */
            Fixture f;
            FdtdError err;
            memset(&err, 0, sizeof err);
            if (fixture_try(&f, &c, &err) != 0) {
                CHECK_MSG(err.msg[0] != '\0', "failure without message (%d x %d, bc %d)",
                          shapes[sh][0], shapes[sh][1], kd);
                failed++;
                continue;
            }
            ran++;
            run(&f.solver, 60);
            CHECK_MSG(all_finite(&f.solver), "non-finite field (%d x %d, bc %d)",
                      shapes[sh][0], shapes[sh][1], kd);
            const double e = solver_acoustic_energy(&f.solver);
            CHECK(isfinite(e) && e >= 0.0);
            fixture_close(&f);
        }
    }
    printf("  [measure] degenerate sizes: %d of 9 cases ran, %d failed cleanly\n", ran, failed);
}

static void test_init_errors_and_free(void)
{
    FdtdError err;
    Solver s;
    Model zero;
    memset(&err, 0, sizeof err);
    memset(&zero, 0, sizeof zero);

    memset(&s, 0xAB, sizeof s);
    CHECK(solver_init(&s, NULL, &err) == -1);
    CHECK(err.msg[0] != '\0');
    CHECK(s.px == NULL && s.model == NULL && s.step == 0);

    memset(&s, 0xAB, sizeof s);
    memset(&err, 0, sizeof err);
    CHECK(solver_init(&s, &zero, &err) == -1);
    CHECK(err.msg[0] != '\0');
    CHECK(s.px == NULL && s.vx == NULL && s.kdt == NULL);

    /* medium index outside the table */
    CaseSpec c = default_spec(10, 10);
    Fixture f;
    if (fixture_open(&f, &c) == 0) {
        const MediumId saved = f.model.medium_id[7];
        f.model.medium_id[7] = 9;
        Solver bad;
        memset(&bad, 0xAB, sizeof bad);
        memset(&err, 0, sizeof err);
        CHECK(solver_init(&bad, &f.model, &err) == -1);
        CHECK_CONTAINS(err.msg, "medium");
        CHECK(bad.px == NULL && bad.model == NULL);
        f.model.medium_id[7] = saved;
        fixture_close(&f);
    }

    /* free: zeroed struct, NULL, and twice on a live one */
    Solver z;
    memset(&z, 0, sizeof z);
    solver_free(&z);
    solver_free(&z);
    solver_free(NULL);
    if (fixture_open(&f, &c) == 0) {
        solver_step(&f.solver);
        solver_free(&f.solver);
        CHECK(f.solver.px == NULL && f.solver.model == NULL);
        solver_free(&f.solver);
        solver_step(&f.solver); /* a freed solver must be inert */
        CHECK(solver_pressure_at(&f.solver, 0) == 0.0);
        CHECK(solver_acoustic_energy(&f.solver) == 0.0);
        f.open = 0;
        f.model.n_sources = f.n_sources_built;
        model_free(&f.model);
    }
}

int main(void)
{
    fdtd_set_warning_stream(NULL);
    RUN_TEST(test_no_sources_stays_zero);
    RUN_TEST(test_source_amplitude_first_step);
    RUN_TEST(test_symmetry);
    RUN_TEST(test_energy_conservation_rigid_box);
    RUN_TEST(test_pressure_release);
    RUN_TEST(test_pressure_release_all_sides);
    RUN_TEST(test_pressure_release_energy_bounded);
    RUN_TEST(test_pml_absorption);
    RUN_TEST(test_obstacles_stay_rigid);
    RUN_TEST(test_propagation_speed);
    RUN_TEST(test_phase_velocity);
    RUN_TEST(test_coefficients);
    RUN_TEST(test_two_media);
    RUN_TEST(test_degenerate_sizes);
    RUN_TEST(test_init_errors_and_free);
    return TEST_REPORT();
}
