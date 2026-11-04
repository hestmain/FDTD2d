/*
 * test_solver.c - unit tests for the FDTD time stepper.
 *
 * Models are built from TOML text (config_load_string + model_build). Grids
 * are small (dx = 1 cm, air) so the whole file runs in about a second
 * (Release build). Physics checks print the numbers they measured on lines starting with
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
    double dy;                     /* cell height [m]; 0 = same as DX */
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

/* Cell height of the case. */
static double spec_dy(const CaseSpec *c)
{
    return c->dy > 0.0 ? c->dy : DX;
}

/* Use cell height dy (!= DX gives non-square cells) and re-centre the
 * default source in y. */
static void spec_set_dy(CaseSpec *c, double dy)
{
    c->dy = dy;
    c->src_y = 0.5 * c->ny * dy;
}

/* Writes the TOML text for c into buf. Returns 0, or -1 (after a failed
 * CHECK) if buf is too small and the text would be truncated. */
static int spec_to_toml(const CaseSpec *c, char *buf, size_t n)
{
    int w = snprintf(buf, n,
        "[simulation]\nduration = 0.1\ncfl = 0.7\n"
        "[grid]\nx_range = [0, %.17g]\ny_range = [0, %.17g]\ndx = %.17g\ndy = %.17g\n"
        "[medium]\ndensity = %.17g\nc = %.17g\n"
        "[boundary]\nx_min = \"%s\"\nx_max = \"%s\"\ny_min = \"%s\"\ny_max = \"%s\"\n"
        "[boundary.pml]\nthickness = %d\nreflection = 1e-6\norder = 2\n",
        c->nx * DX, c->ny * spec_dy(c), DX, spec_dy(c), RHO, CSOUND,
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
        w += snprintf(buf + w, n - (size_t)w, "%s", c->extra);
    }
    CHECK_MSG(w > 0 && (size_t)w < n, "TOML buffer of %zu bytes is too small", n);
    return (w > 0 && (size_t)w < n) ? 0 : -1;
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
    if (spec_to_toml(c, text, sizeof text) != 0) {
        fdtd_error_set(err, "test TOML text does not fit the buffer");
        return -1;
    }
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

/* Weight of a velocity face in the discrete energy: the outer faces of a
 * pressure_release side get 1/2 (half of the control volume is outside the
 * grid), every other face 1. Derived here from the model's boundary types
 * only, independently of solver.c. `index` is the face index along the
 * normal axis (0..n), `n` the cell count along it. */
static double face_weight(const Model *m, Side lo, Side hi, int index, int n)
{
    if (index == 0 && m->boundary[lo] == BC_PRESSURE_RELEASE) {
        return 0.5;
    }
    if (index == n && m->boundary[hi] == BC_PRESSURE_RELEASE) {
        return 0.5;
    }
    return 1.0;
}

/* sum_faces w rho_face v^2 / 2 (no cell area factor). */
static double weighted_kinetic(const Solver *s)
{
    const Model *m = s->model;
    const double dt = m->dt;
    double kin = 0.0;
    for (int j = 0; j < s->ny; j++) {
        for (int i = 0; i <= s->nx; i++) {
            const size_t f = (size_t)j * (size_t)(s->nx + 1) + (size_t)i;
            if (s->bx[f] > 0.0) {
                kin += face_weight(m, SIDE_X_MIN, SIDE_X_MAX, i, s->nx) * 0.5 *
                       (dt / s->bx[f]) * s->vx[f] * s->vx[f];
            }
        }
    }
    for (int j = 0; j <= s->ny; j++) {
        for (int i = 0; i < s->nx; i++) {
            const size_t f = (size_t)j * (size_t)s->nx + (size_t)i;
            if (s->by[f] > 0.0) {
                kin += face_weight(m, SIDE_Y_MIN, SIDE_Y_MAX, j, s->ny) * 0.5 *
                       (dt / s->by[f]) * s->vy[f] * s->vy[f];
            }
        }
    }
    return kin;
}

/* Exact conserved quantity of the leapfrog scheme in a lossless closed
 * domain (rigid and/or pressure_release sides, obstacles, several media):
 *   sum p^n p^(n+1) / (2K) + sum_faces w rho_face v^2 / 2,
 * with v at n+1/2 (p_prev = p^n, solver holds p^(n+1)). w = 1/2 on the
 * outer faces of pressure_release sides, 1 elsewhere: the ghost cell
 * p = -p_inside makes the p0*v0 cross terms of the two half steps cancel
 * only with that weight. */
static double leapfrog_invariant(const Solver *s, const double *p_prev)
{
    const double dt = s->model->dt;
    const size_t n = (size_t)s->nx * (size_t)s->ny;
    double pot = 0.0;
    for (size_t c = 0; c < n; c++) {
        if (s->kdt[c] > 0.0) {
            pot += p_prev[c] * solver_pressure_at(s, c) / (2.0 * (s->kdt[c] / dt));
        }
    }
    return (pot + weighted_kinetic(s)) * s->model->grid.dx * s->model->grid.dy;
}

/* Independent evaluation of the documented solver_acoustic_energy():
 * sum p^2/(2K) + weighted kinetic energy, times the cell area. */
static double reference_energy(const Solver *s)
{
    const Model *m = s->model;
    double pot = 0.0;
    for (size_t c = 0; c < (size_t)s->nx * (size_t)s->ny; c++) {
        if (s->kdt[c] > 0.0) {
            const double p = solver_pressure_at(s, c);
            pot += p * p / (2.0 * (s->kdt[c] / m->dt));
        }
    }
    return (pot + weighted_kinetic(s)) * m->grid.dx * m->grid.dy;
}

typedef struct InvariantResult {
    double inv_dev;      /* max |I_n / I_0 - 1| of the leapfrog invariant */
    double ref_dev;      /* max |solver_acoustic_energy - reference_energy| / E0 */
    double e0;           /* solver_acoustic_energy at the switch-off [J/m] */
    double e_min, e_max; /* range of E(t) / E0 */
    size_t n_solid;      /* solid cells of the model */
} InvariantResult;

/* Drives the case with its source for `drive_cycles` periods, switches the
 * source off and follows `n` further steps, evaluating the exact invariant
 * and the documented energy at every step. Returns 0, or -1 if the case
 * could not be set up (a CHECK has failed then). */
static int invariant_run(const CaseSpec *c, double drive_cycles, long n, InvariantResult *r)
{
    memset(r, 0, sizeof *r);
    Fixture f;
    if (fixture_open(&f, c) != 0) {
        return -1;
    }
    run(&f.solver, (long)(drive_cycles / (c->freq * f.model.dt)));
    const size_t n_cells = (size_t)f.solver.nx * (size_t)f.solver.ny;
    double *p_prev = malloc(n_cells * sizeof *p_prev);
    CHECK(p_prev != NULL);
    if (!p_prev) {
        fixture_close(&f);
        return -1;
    }
    r->n_solid = f.model.n_solid;
    const int saved = f.model.n_sources;
    f.model.n_sources = 0;
    solver_get_pressure(&f.solver, p_prev);
    solver_step(&f.solver);
    const double inv0 = leapfrog_invariant(&f.solver, p_prev);
    r->e0 = solver_acoustic_energy(&f.solver);
    CHECK(inv0 > 0.0 && r->e0 > 0.0);
    r->e_min = 1e300;
    for (long k = 0; k < n; k++) {
        solver_get_pressure(&f.solver, p_prev);
        solver_step(&f.solver);
        r->inv_dev = fmax(r->inv_dev, fabs(leapfrog_invariant(&f.solver, p_prev) / inv0 - 1.0));
        const double e = solver_acoustic_energy(&f.solver);
        r->ref_dev = fmax(r->ref_dev, fabs(e - reference_energy(&f.solver)) / r->e0);
        r->e_min = fmin(r->e_min, e / r->e0);
        r->e_max = fmax(r->e_max, e / r->e0);
    }
    CHECK(all_finite(&f.solver));
    free(p_prev);
    f.model.n_sources = saved;
    fixture_close(&f);
    return 0;
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

static void report_invariant(const char *what, const InvariantResult *r)
{
    printf("  [measure] %s: E(switch-off) = %.4g J/m, E/E0 in [%.4f, %.4f], "
           "invariant deviation = %.3g, |E - E_ref|/E0 = %.3g\n",
           what, r->e0, r->e_min, r->e_max, r->inv_dev, r->ref_dev);
}

static void test_energy_conservation_anisotropic_box(void)
{
    /* dy = 2 dx and a non-square cell count: an inv_dx/inv_dy or sigma_x/
     * sigma_y mix-up breaks the symmetry of the discrete operators, so the
     * exact invariant no longer holds. */
    CaseSpec c = default_spec(61, 29);
    spec_set_dy(&c, 2.0 * DX);
    c.src_x = 0.2137;
    c.src_y = 0.1963;
    InvariantResult r;
    if (invariant_run(&c, 4.0, 3000, &r) != 0) {
        return;
    }
    report_invariant("rigid box dx=1cm dy=2cm 61x29", &r);
    CHECK(r.inv_dev < 1e-11);
    CHECK(r.ref_dev < 1e-12);
    CHECK(r.e_min >= 0.85 && r.e_max <= 1.15);
}

static void test_energy_conservation_release_box(void)
{
    /* Closed boxes with pressure_release sides. The exact invariant needs
     * the outer release faces weighted by 1/2 and the ghost cell p = -p_in
     * (mutating either breaks it); the documented solver_acoustic_energy
     * must use the same weights (compared with an independent evaluation). */
    struct {
        const char *name;
        BoundaryType bc[SIDE_COUNT];
        double dy;
    } cases[3] = {
        {"x_min + y_max release, square cells",
         {BC_PRESSURE_RELEASE, BC_RIGID, BC_RIGID, BC_PRESSURE_RELEASE}, DX},
        {"all four sides release, dy = 2 dx",
         {BC_PRESSURE_RELEASE, BC_PRESSURE_RELEASE, BC_PRESSURE_RELEASE, BC_PRESSURE_RELEASE},
         2.0 * DX},
        {"x_max + y_min release, dy = 2 dx",
         {BC_RIGID, BC_PRESSURE_RELEASE, BC_PRESSURE_RELEASE, BC_RIGID}, 2.0 * DX},
    };
    for (int k = 0; k < 3; k++) {
        CaseSpec c = default_spec(51, 41);
        for (int s = 0; s < SIDE_COUNT; s++) {
            c.bc[s] = cases[k].bc[s];
        }
        spec_set_dy(&c, cases[k].dy);
        c.src_x = 0.2137;
        c.src_y = 0.37 * c.ny * cases[k].dy;
        InvariantResult r;
        if (invariant_run(&c, 4.0, 3000, &r) != 0) {
            continue;
        }
        report_invariant(cases[k].name, &r);
        CHECK_MSG(r.inv_dev < 1e-11, "%s", cases[k].name);
        CHECK_MSG(r.ref_dev < 1e-12, "%s", cases[k].name);
        CHECK_MSG(r.e_min >= 0.85 && r.e_max <= 1.15, "%s", cases[k].name);
    }
}

static void test_energy_conservation_obstacles(void)
{
    /* Closed boxes with solid cells (circle, rect), one of them also with
     * release sides and obstacles touching the release walls, and dy != dx. */
    CaseSpec c = default_spec(51, 41);
    c.src_x = 0.1737;
    c.src_y = 0.1263;
    c.extra =
        "[[obstacle]]\nshape = { type = \"circle\", center = [0.35, 0.25], radius = 0.08 }\n"
        "[[obstacle]]\nshape = { type = \"rect\", min = [0.22, 0.30], max = [0.30, 0.36] }\n";
    InvariantResult r;
    if (invariant_run(&c, 4.0, 3000, &r) == 0) {
        report_invariant("rigid box + circle + rect", &r);
        CHECK(r.n_solid > 100);
        CHECK(r.inv_dev < 1e-11);
        CHECK(r.ref_dev < 1e-12);
        CHECK(r.e_min >= 0.85 && r.e_max <= 1.15);
    }

    CaseSpec d = default_spec(51, 41);
    d.bc[SIDE_X_MIN] = BC_PRESSURE_RELEASE;
    d.bc[SIDE_Y_MAX] = BC_PRESSURE_RELEASE;
    spec_set_dy(&d, 2.0 * DX);
    d.src_x = 0.2137;
    d.src_y = 0.30;
    d.extra =
        "[[obstacle]]\nshape = { type = \"rect\", min = [0.0, 0.30], max = [0.06, 0.50] }\n"
        "[[obstacle]]\nshape = { type = \"rect\", min = [0.30, 0.70], max = [0.40, 0.82] }\n"
        "[[obstacle]]\nshape = { type = \"circle\", center = [0.38, 0.30], radius = 0.07 }\n";
    if (invariant_run(&d, 4.0, 3000, &r) == 0) {
        report_invariant("release x_min/y_max + obstacles at the release walls, dy = 2 dx", &r);
        CHECK(r.n_solid > 20);
        CHECK(r.inv_dev < 1e-11);
        CHECK(r.ref_dev < 1e-12);
        CHECK(r.e_min >= 0.85 && r.e_max <= 1.15);
    }
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
    /* The first cell centre is dx/2 from the p = 0 plane (ghost cell
     * p = -p_inside). Incident + reflected plane wave of amplitude 1:
     * p = 2 sin(k s) at distance s from the release plane (s = dx/2 at the
     * probe), and p = 2 cos(k s) at distance s from the rigid wall (s = dx/2
     * there too). Ratio of the peaks = tan(k dx/2). A ghost cell a full cell
     * away would give about twice this value. */
    const double ratio = pmax_release / pmax_rigid;
    const double k_dx_half = M_PI * c.freq * DX / CSOUND; /* k dx / 2 */
    const double analytic = tan(k_dx_half);
    printf("  [measure] pressure release: max|p| next to release side = %.4g Pa, "
           "next to rigid side = %.4g Pa, ratio = %.4f (tan(k dx/2) = %.4f)\n",
           pmax_release, pmax_rigid, ratio, analytic);
    CHECK(pmax_rigid > 0.0);
    /* Measured 0.1402 vs 0.1383 analytic (+1.4%: numerical dispersion and
     * the finite ramp). The run is deterministic, so +-5% around the
     * analytic value leaves 3.5% margin on the high side and still excludes
     * ghost-cell errors (a cell-distance ghost gives ~0.28, i.e. +100%). The
     * exact release-box invariant tests are the primary guard. */
    CHECK(ratio > 0.95 * analytic && ratio < 1.05 * analytic);
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

static void test_pml_absorption_anisotropic(void)
{
    /* 80 x 40 physical cells, dx = 1 cm, dy = 2 cm, PML of 20 cells on all
     * four sides (0.2 m in x, 0.4 m in y; 120 x 80 cells in total). */
    CaseSpec c = default_spec(80, 40);
    spec_set_dy(&c, 2.0 * DX);
    spec_all(&c, BC_PML);
    c.pml_thickness = 20;
    Fixture f;
    if (fixture_open(&f, &c) != 0) {
        return;
    }
    CHECK(f.solver.nx == 120 && f.solver.ny == 80);
    const long drive = (long)(3.0 / (c.freq * f.model.dt));
    const long total = 1500;
    double e_peak = 0.0;
    const int saved = f.model.n_sources;
    for (long k = 0; k < total; k++) {
        if (k == drive) {
            f.model.n_sources = 0;
        }
        solver_step(&f.solver);
        e_peak = fmax(e_peak, solver_acoustic_energy(&f.solver));
    }
    const double e_end = solver_acoustic_energy(&f.solver);
    printf("  [measure] PML 80x40, dy = 2 dx: peak energy = %.4g J/m, energy after %ld steps "
           "= %.4g J/m, ratio = %.3g\n", e_peak, total, e_end, e_end / e_peak);
    CHECK(e_peak > 0.0);
    CHECK(e_end < 1e-3 * e_peak);
    CHECK(all_finite(&f.solver));
    f.model.n_sources = saved;
    fixture_close(&f);
}

/* Gaussian pressure pulse that varies along one axis only (uniform along the
 * other), stored in the split field that belongs to that axis; vx = vy = 0. */
static void seed_axis_pulse(Solver *s, int along_x, double centre, double width)
{
    for (int j = 0; j < s->ny; j++) {
        for (int i = 0; i < s->nx; i++) {
            const double u = ((along_x ? i : j) - centre) / width;
            (along_x ? s->px : s->py)[(size_t)j * (size_t)s->nx + (size_t)i] =
                exp(-0.5 * u * u);
        }
    }
}

typedef struct ChannelResult {
    double e_ratio;   /* E(end) / E(start) */
    double inv_dev;   /* max deviation of the leapfrog invariant */
    double cross_max; /* max |px| (y pulse) or |py| (x pulse), + |v| across */
} ChannelResult;

/* A quasi-1D pulse (along x if pulse_x, else along y) in a channel that has
 * PML on the x sides and rigid y sides (pml_x) or the other way round. The
 * pulse starts in the middle; the energy and the leapfrog invariant are
 * tracked for `steps` steps. Returns 0, or -1 if the setup failed. */
static int channel_run(int pml_x, int pulse_x, long steps, ChannelResult *out)
{
    CaseSpec c = default_spec(pml_x ? 80 : 40, pml_x ? 40 : 80);
    spec_set_dy(&c, 2.0 * DX);
    c.has_source = 0;
    c.pml_thickness = 10;
    c.bc[SIDE_X_MIN] = c.bc[SIDE_X_MAX] = pml_x ? BC_PML : BC_RIGID;
    c.bc[SIDE_Y_MIN] = c.bc[SIDE_Y_MAX] = pml_x ? BC_RIGID : BC_PML;
    Fixture f;
    if (fixture_open(&f, &c) != 0) {
        return -1;
    }
    Solver *s = &f.solver;
    const size_t n_cells = (size_t)s->nx * (size_t)s->ny;
    double *p_prev = malloc(n_cells * sizeof *p_prev);
    CHECK(p_prev != NULL);
    if (!p_prev) {
        fixture_close(&f);
        return -1;
    }
    /* pulse between the middle and the PML-side end, width 5 cells */
    const double centre = pulse_x ? 0.5 * s->nx : 0.5 * s->ny;
    seed_axis_pulse(s, pulse_x, centre, 5.0);
    solver_get_pressure(s, p_prev);
    solver_step(s);
    const double inv0 = leapfrog_invariant(s, p_prev);
    const double e0 = solver_acoustic_energy(s);
    CHECK(inv0 > 0.0 && e0 > 0.0);
    out->inv_dev = 0.0;
    out->cross_max = 0.0;
    for (long k = 1; k < steps; k++) {
        solver_get_pressure(s, p_prev);
        solver_step(s);
        out->inv_dev = fmax(out->inv_dev, fabs(leapfrog_invariant(s, p_prev) / inv0 - 1.0));
    }
    /* A pulse along x only has px != 0, vx != 0; py and vy must stay zero
     * (and the other way round). */
    for (size_t q = 0; q < n_cells; q++) {
        out->cross_max = fmax(out->cross_max, fabs(pulse_x ? s->py[q] : s->px[q]));
    }
    for (size_t q = 0; q < (size_t)(s->nx + 1) * (size_t)s->ny; q++) {
        out->cross_max = fmax(out->cross_max, pulse_x ? 0.0 : fabs(s->vx[q]));
    }
    for (size_t q = 0; q < (size_t)s->nx * (size_t)(s->ny + 1); q++) {
        out->cross_max = fmax(out->cross_max, pulse_x ? fabs(s->vy[q]) : 0.0);
    }
    out->e_ratio = solver_acoustic_energy(s) / e0;
    CHECK(all_finite(s));
    free(p_prev);
    fixture_close(&f);
    return 0;
}

static void test_pml_axis_assignment(void)
{
    /* Discriminates sigma_x from sigma_y (and inv_dx from inv_dy), which an
     * isotropic all-PML test cannot do. dy = 2 dx.
     *
     * A pulse that varies along x only (px, vx) feels only sigma_x; a pulse
     * along y only (py, vy) feels only sigma_y. In a channel with PML on the
     * x sides and rigid y sides:
     *   - the x pulse runs into the PML and is absorbed (energy collapses);
     *   - the y pulse never sees any damping: it bounces between the rigid
     *     y walls, the energy stays constant and the exact leapfrog
     *     invariant is conserved, and px / vx stay exactly zero.
     * The channel with PML on the y sides is the mirror image. With sigma_x
     * and sigma_y swapped the "lossless" pulse would be damped (and the
     * "absorbed" one reflected), and with inv_dx/inv_dy swapped the
     * invariant of the mixed-up operators breaks. */
    static const struct {
        const char *name;
        int pml_x, pulse_x, absorbed;
        long steps;
    } cases[4] = {
        {"PML x sides, pulse along x", 1, 1, 1, 700},
        {"PML x sides, pulse along y", 1, 0, 0, 700},
        {"PML y sides, pulse along y", 0, 0, 1, 900},
        {"PML y sides, pulse along x", 0, 1, 0, 900},
    };
    for (int k = 0; k < 4; k++) {
        ChannelResult r;
        if (channel_run(cases[k].pml_x, cases[k].pulse_x, cases[k].steps, &r) != 0) {
            continue;
        }
        if (cases[k].absorbed) {
            printf("  [measure] %s: E_end/E_start = %.3g (invariant not conserved: "
                   "absorbing), cross-axis field = %.3g\n", cases[k].name, r.e_ratio,
                   r.cross_max);
        } else {
            printf("  [measure] %s: E_end/E_start = %.3g, invariant deviation = %.3g, "
                   "cross-axis field = %.3g\n", cases[k].name, r.e_ratio, r.inv_dev,
                   r.cross_max);
        }
        CHECK_MSG(r.cross_max == 0.0, "%s", cases[k].name);
        if (cases[k].absorbed) {
            CHECK_MSG(r.e_ratio < 1e-3, "%s", cases[k].name);
        } else {
            CHECK_MSG(r.inv_dev < 1e-11, "%s", cases[k].name);
            CHECK_MSG(fabs(r.e_ratio - 1.0) < 0.1, "%s", cases[k].name);
        }
    }
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

/* Error (in steps) of the measured phase delay between the steady-state
 * series of a near probe (sa) and a far probe (sb) on the same ray, against
 * the expected delay d_exp [steps], folded into +-period/2. The far
 * crossing is the first upward zero crossing at or after sample start_b; the
 * near one is the crossing closest to (far - d_exp). NaN if not found. */
static double phase_delay_error(const double *sa, const double *sb, long total, double period,
                                double d_exp, long start_b)
{
    const double tb = upward_zero_crossing(sb, start_b, total);
    if (!(tb > 0.0)) {
        return NAN;
    }
    const double target = tb - d_exp;
    const long from = (long)(target - period) - 1;
    double ta = upward_zero_crossing(sa, from > 1 ? from : 1, total);
    double best = ta;
    for (int tries = 0; tries < 3 && ta > 0.0; tries++) {
        if (fabs(ta - target) < fabs(best - target)) {
            best = ta;
        }
        ta = upward_zero_crossing(sa, (long)ta + 2, total);
    }
    if (!(best > 0.0)) {
        return NAN;
    }
    const double fold = fmod((tb - best) - d_exp, period);
    return fabs(fold) > 0.5 * period ? fold - copysign(period, fold) : fold;
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
            const double err = phase_delay_error(series[a], series[b], total, period, d_exp,
                                                 start_b);
            CHECK(!isnan(err));
            if (isnan(err)) {
                continue;
            }
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

/* Exact numerical phase delay [steps] over the distance r [m] along a grid
 * axis with cell size h, from the 1D dispersion relation of the leapfrog
 * scheme, sin(w dt/2) = (c dt/h) sin(k h/2). */
static double numerical_delay_steps(double r, double h, double dt, double freq)
{
    const double w = 2.0 * M_PI * freq;
    const double nu = CSOUND * dt / h;
    const double k = 2.0 / h * asin(sin(0.5 * w * dt) / nu);
    return (r * k / w) / dt;
}

static void test_phase_velocity_anisotropic(void)
{
    /* dy = 2 dx. Phase delay between probes at 0.3 m and 0.9 m from the
     * source along +x and along +y. A swapped inv_dx/inv_dy changes the
     * speed along an axis by a factor 2, a swapped sigma changes nothing
     * here but is covered by the PML tests. Tolerance: 1 step (~1% of the
     * ~96-step delay), as in test_phase_velocity. The expected value is
     * r/c; the exact numerical dispersion of the scheme is printed too (at
     * 1 kHz it is +0.5 step along y and +0.1 step along x, so r/c is tight
     * enough). */
    const int nxp = 201, nyp = 101;
    CaseSpec c = default_spec(nxp, nyp);
    spec_set_dy(&c, 2.0 * DX);
    spec_all(&c, BC_PML);
    c.pml_thickness = 10;
    c.freq = 1000.0;
    c.ramp_cycles = 2.0;
    c.src_x = (nxp / 2 + 0.5) * DX;
    c.src_y = (nyp / 2 + 0.5) * spec_dy(&c);
    Fixture f;
    if (fixture_open(&f, &c) != 0) {
        return;
    }
    const Grid *g = &f.model.grid;
    CHECK_NEAR(g->dy, 2.0 * g->dx, 1e-15);
    const int ic = g->off_x + nxp / 2;
    const int jc = g->off_y + nyp / 2;
    const double dt = f.model.dt;
    const double period = 1.0 / (c.freq * dt);

    /* probes at 0.3 m / 0.9 m: x offsets 30 / 90 cells, y offsets 15 / 45 */
    enum { NR = 4 };
    const int off[NR][2] = {{30, 0}, {90, 0}, {0, 15}, {0, 45}};
    size_t cells[NR];
    double r[NR];
    for (int k = 0; k < NR; k++) {
        cells[k] = grid_cell_index(g, ic + off[k][0], jc + off[k][1]);
        r[k] = hypot(off[k][0] * g->dx, off[k][1] * g->dy);
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
            const double h = pair == 0 ? g->dx : g->dy;
            const double d_exp = (r[b] - r[a]) / (CSOUND * dt);
            const double d_num = numerical_delay_steps(r[b] - r[a], h, dt, c.freq);
            const long start_b = (long)(r[b] / (CSOUND * dt) + 3.0 * period + 5.0);
            const double err = phase_delay_error(series[a], series[b], total, period, d_exp,
                                                 start_b);
            CHECK(!isnan(err));
            if (isnan(err)) {
                continue;
            }
            printf("  [measure] phase delay along %s (h = %.0f mm), r = %.3f -> %.3f m: "
                   "r/c = %.2f steps, error %+.3f steps (%+.3f%%); scheme's own dispersion "
                   "predicts %+.3f, residual %+.3f\n", pair == 0 ? "x" : "y", 1e3 * h, r[a],
                   r[b], d_exp, err, 100.0 * err / d_exp, d_num - d_exp,
                   err - (d_num - d_exp));
            CHECK(fabs(err) <= 1.0);
            CHECK(fabs(err - (d_num - d_exp)) <= 0.5);
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

    /* The solver does not force a = c = 1 on release faces; it follows from
     * sigma = 0 there (model_build). Check all four sides. */
    CaseSpec r = default_spec(30, 24);
    spec_all(&r, BC_PRESSURE_RELEASE);
    if (fixture_open(&f, &r) != 0) {
        return;
    }
    s = &f.solver;
    CHECK(s->ax_f[0] == 1.0 && s->cx_f[0] == 1.0);
    CHECK(s->ax_f[s->nx] == 1.0 && s->cx_f[s->nx] == 1.0);
    CHECK(s->ay_f[0] == 1.0 && s->cy_f[0] == 1.0);
    CHECK(s->ay_f[s->ny] == 1.0 && s->cy_f[s->ny] == 1.0);
    fixture_close(&f);
}

/* Heavier, slower medium in the right half (vertical interface: media differ
 * between columns i, the bx averaging) or in the upper half (horizontal
 * interface: media differ between rows j, the by averaging; dy = 2 dx there).
 * rho_face is the arithmetic mean at the interface, K per cell, and a closed
 * box still conserves the staggered invariant to rounding. */
static void two_media_case(int horizontal)
{
    CaseSpec c = default_spec(40, 30);
    if (horizontal) {
        spec_set_dy(&c, 2.0 * DX);
        c.src_y = 0.25 * c.ny * spec_dy(&c); /* in the light medium */
    }
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
    const double rho_mean = 0.5 * (RHO + 1000.0);
    const size_t fw = (size_t)g->nx + 1;
    const size_t cw = (size_t)g->nx;
    if (!horizontal) {
        const int i_if = g->nx / 2; /* first column of the heavy medium */
        for (int j = 0; j < g->ny; j++) {
            for (int i = i_if; i < g->nx; i++) {
                f.model.medium_id[grid_cell_index(g, i, j)] = (MediumId)id;
            }
        }
    } else {
        const int j_if = g->ny / 2; /* first row of the heavy medium */
        for (int j = j_if; j < g->ny; j++) {
            for (int i = 0; i < g->nx; i++) {
                f.model.medium_id[grid_cell_index(g, i, j)] = (MediumId)id;
            }
        }
    }
    CHECK(solver_init(&f.solver, &f.model, &err) == 0);
    const Solver *s = &f.solver;
    const double dt = f.model.dt;
    if (!horizontal) {
        const int i_if = g->nx / 2;
        const int j0 = 7;
        CHECK_NEAR(s->bx[(size_t)j0 * fw + 5] * RHO / dt, 1.0, 1e-14);
        CHECK_NEAR(s->bx[(size_t)j0 * fw + (size_t)(g->nx - 3)] * 1000.0 / dt, 1.0, 1e-14);
        CHECK_NEAR(s->bx[(size_t)j0 * fw + (size_t)i_if] * rho_mean / dt, 1.0, 1e-14);
        CHECK_NEAR(s->by[(size_t)j0 * cw + (size_t)(i_if - 1)] * RHO / dt, 1.0, 1e-14);
        CHECK_NEAR(s->by[(size_t)j0 * cw + (size_t)i_if] * 1000.0 / dt, 1.0, 1e-14);
        CHECK_NEAR(s->kdt[grid_cell_index(g, 3, 3)], RHO * CSOUND * CSOUND * dt, 1e-9);
        CHECK_NEAR(s->kdt[grid_cell_index(g, g->nx - 2, 3)], 1000.0 * 300.0 * 300.0 * dt, 1e-6);
    } else {
        const int j_if = g->ny / 2;
        const int i0 = 9;
        /* y-faces: j_if - 1 and below lie between light cells, j_if between
         * light (row j_if - 1) and heavy (row j_if), j_if + 1 and above
         * between heavy cells. */
        CHECK_NEAR(s->by[(size_t)(j_if - 1) * cw + (size_t)i0] * RHO / dt, 1.0, 1e-14);
        CHECK_NEAR(s->by[(size_t)j_if * cw + (size_t)i0] * rho_mean / dt, 1.0, 1e-14);
        CHECK_NEAR(s->by[(size_t)(j_if + 1) * cw + (size_t)i0] * 1000.0 / dt, 1.0, 1e-14);
        /* x-faces follow the medium of their row */
        CHECK_NEAR(s->bx[(size_t)(j_if - 1) * fw + (size_t)i0] * RHO / dt, 1.0, 1e-14);
        CHECK_NEAR(s->bx[(size_t)j_if * fw + (size_t)i0] * 1000.0 / dt, 1.0, 1e-14);
        CHECK_NEAR(s->kdt[grid_cell_index(g, 3, 3)], RHO * CSOUND * CSOUND * dt, 1e-9);
        CHECK_NEAR(s->kdt[grid_cell_index(g, 3, g->ny - 2)], 1000.0 * 300.0 * 300.0 * dt, 1e-6);
    }

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
        printf("  [measure] two media, %s interface: invariant deviation = %.3g\n",
               horizontal ? "horizontal" : "vertical", dev);
        CHECK(dev < 1e-11);
        f.model.n_sources = saved;
        free(p_prev);
    }
    fixture_close(&f);
}

static void test_two_media(void)
{
    two_media_case(0);
}

static void test_two_media_horizontal(void)
{
    two_media_case(1);
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

/* Builds a model from `toml` (must succeed) and expects solver_init to fail
 * with a message containing `needle`; *s must be zeroed afterwards. */
static void expect_init_error(const char *toml, const char *needle)
{
    SimConfig cfg;
    Model m;
    Solver s;
    FdtdError err = {{0}};
    if (config_load_string(toml, "test_solver", &cfg, &err) != 0) {
        CHECK_MSG(0, "config: %s", err.msg);
        return;
    }
    const int rc = model_build(&cfg, &m, &err);
    config_free(&cfg);
    if (rc != 0) {
        CHECK_MSG(0, "model: %s", err.msg);
        return;
    }
    CHECK(solver_init(&s, &m, &err) == -1);
    CHECK_CONTAINS(err.msg, needle);
    CHECK(s.px == NULL && s.model == NULL);
    solver_free(&s);
    model_free(&m);
}

/* The two checks solver_init keeps: values that pass config and model
 * validation individually but are unusable for the update coefficients. */
static void test_init_rejects_unusable_coefficients(void)
{
    /* c = 1e-200 is a valid speed, but K = rho c^2 underflows to 0. */
    expect_init_error("[simulation]\nduration = 0.1\n"
                      "[grid]\nx_range = [0, 0.1]\ny_range = [0, 0.1]\ndx = 0.01\n"
                      "[medium]\ndensity = 1.21\nc = 1e-200\n"
                      "[boundary]\nx_min = \"rigid\"\nx_max = \"rigid\"\n"
                      "y_min = \"rigid\"\ny_max = \"rigid\"\n",
                      "bulk modulus");

    /* A valid but huge PML order overflows sigma_max to inf. */
    expect_init_error("[simulation]\nduration = 0.001\n"
                      "[grid]\nx_range = [0, 0.1]\ny_range = [0, 0.1]\ndx = 0.01\n"
                      "[medium]\ndensity = 1.21\nc = 343\n"
                      "[boundary]\nx_min = \"pml\"\nx_max = \"rigid\"\n"
                      "y_min = \"rigid\"\ny_max = \"rigid\"\n"
                      "[boundary.pml]\nthickness = 4\nreflection = 1e-6\norder = 1e306\n",
                      "damping");
}

/* solver_free accepts a zeroed struct, NULL, and repeated calls. */
static void test_free(void)
{
    Solver z;
    memset(&z, 0, sizeof z);
    solver_free(&z);
    solver_free(&z);
    solver_free(NULL);

    CaseSpec c = default_spec(10, 10);
    Fixture f;
    if (fixture_open(&f, &c) == 0) {
        solver_step(&f.solver);
        solver_free(&f.solver);
        CHECK(f.solver.px == NULL && f.solver.model == NULL);
        solver_free(&f.solver);
        CHECK(f.solver.px == NULL && f.solver.model == NULL);
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
    RUN_TEST(test_energy_conservation_anisotropic_box);
    RUN_TEST(test_energy_conservation_release_box);
    RUN_TEST(test_energy_conservation_obstacles);
    RUN_TEST(test_pressure_release);
    RUN_TEST(test_pressure_release_all_sides);
    RUN_TEST(test_pressure_release_energy_bounded);
    RUN_TEST(test_pml_absorption);
    RUN_TEST(test_pml_absorption_anisotropic);
    RUN_TEST(test_pml_axis_assignment);
    RUN_TEST(test_obstacles_stay_rigid);
    RUN_TEST(test_propagation_speed);
    RUN_TEST(test_phase_velocity);
    RUN_TEST(test_phase_velocity_anisotropic);
    RUN_TEST(test_coefficients);
    RUN_TEST(test_two_media);
    RUN_TEST(test_two_media_horizontal);
    RUN_TEST(test_degenerate_sizes);
    RUN_TEST(test_init_rejects_unusable_coefficients);
    RUN_TEST(test_free);
    return TEST_REPORT();
}
