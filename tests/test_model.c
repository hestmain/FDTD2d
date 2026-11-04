/*
 * test_model.c - unit tests for the SimConfig -> Model layer.
 *
 * The TOML parser is a separate module (and test), so SimConfig structs are
 * filled in by hand here. They are always valid, as config_load_* would make
 * them: model_build() only checks what depends on the grid. Everything is
 * checked against the contract in
 * model.h: grid size and PML offsets, dt/CFL/n_steps, PML damping profiles,
 * the rigid obstacle mask, the resolution warning and source discretisation.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "boundary.h"
#include "common.h"
#include "config.h"
#include "geometry.h"
#include "grid.h"
#include "medium.h"
#include "model.h"
#include "waveform.h"
#include "source.h"

#include "test_util.h"

/* Storage the configs point at (model_build copies everything). */
static SourceConfig g_sources[2];
static ObstacleConfig g_obstacles[2];
static Vec2 g_polygon[8];

/* ------------------------------------------------------------- utilities */

static double speed_limit(const SimConfig *cfg)
{
    /* S = c_max sqrt(1/dx^2 + 1/dy^2) [1/s]; dt <= 1/S */
    return cfg->medium.c * sqrt(1.0 / (cfg->dx * cfg->dx) + 1.0 / (cfg->dy * cfg->dy));
}

static void set_point_source(SourceConfig *sc, double x, double y, double f, double amp)
{
    memset(sc, 0, sizeof *sc);
    sc->shape.type = SHAPE_POINT;
    sc->shape.u.point.position.x = x;
    sc->shape.u.point.position.y = y;
    sc->signal.type = SIGNAL_HARMONIC;
    sc->signal.u.harmonic.frequency = f;
    sc->signal.u.harmonic.amplitude = amp;
    sc->signal.u.harmonic.phase = 0.0;
    sc->signal.u.harmonic.ramp_cycles = 3.0;
}

static Shape *set_rect_obstacle(int k, double x0, double y0, double x1, double y1)
{
    memset(&g_obstacles[k], 0, sizeof g_obstacles[k]);
    Shape *s = &g_obstacles[k].shape;
    s->type = SHAPE_RECT;
    s->u.rect.min.x = x0;
    s->u.rect.min.y = y0;
    s->u.rect.max.x = x1;
    s->u.rect.max.y = y1;
    return s;
}

static Shape *set_circle_obstacle(int k, double cx, double cy, double r)
{
    memset(&g_obstacles[k], 0, sizeof g_obstacles[k]);
    Shape *s = &g_obstacles[k].shape;
    s->type = SHAPE_CIRCLE;
    s->u.circle.center.x = cx;
    s->u.circle.center.y = cy;
    s->u.circle.radius = r;
    return s;
}

static Shape *set_polygon_obstacle(int k, int n)
{
    memset(&g_obstacles[k], 0, sizeof g_obstacles[k]);
    Shape *s = &g_obstacles[k].shape;
    s->type = SHAPE_POLYGON;
    s->u.polygon.vertices = g_polygon;
    s->u.polygon.n = n;
    return s;
}

/* Sane defaults: 2 m x 1.5 m air domain at 5 mm, PML on every side,
 * one 2 kHz point source in the middle. */
static SimConfig base_config(void)
{
    SimConfig cfg;
    memset(&cfg, 0, sizeof cfg);
    cfg.duration = 0.004;
    cfg.cfl = CONFIG_DEFAULT_CFL;
    cfg.dt = 0.0;
    cfg.dt_set = false;
    cfg.x_range[0] = 0.0;
    cfg.x_range[1] = 2.0;
    cfg.y_range[0] = 0.0;
    cfg.y_range[1] = 1.5;
    cfg.dx = 0.005;
    cfg.dy = 0.005;
    cfg.medium.density = 1.21;
    cfg.medium.c = 343.0;
    for (int s = 0; s < SIDE_COUNT; s++) {
        cfg.boundary[s] = BC_PML;
    }
    cfg.pml.thickness = PML_DEFAULT_THICKNESS;
    cfg.pml.reflection = PML_DEFAULT_REFLECTION;
    cfg.pml.order = PML_DEFAULT_ORDER;
    memset(g_sources, 0, sizeof g_sources);
    set_point_source(&g_sources[0], 0.6, 0.75, 2000.0, 2e-3);
    cfg.sources = g_sources;
    cfg.n_sources = 1;
    memset(g_obstacles, 0, sizeof g_obstacles);
    cfg.obstacles = g_obstacles;
    cfg.n_obstacles = 0;
    return cfg;
}

/* Small, coarse domain: 4 x 4 cells of 2 m, rigid walls, 3-step friendly. */
static SimConfig coarse_config(void)
{
    SimConfig cfg = base_config();
    cfg.x_range[1] = 8.0;
    cfg.y_range[1] = 8.0;
    cfg.dx = 2.0;
    cfg.dy = 2.0;
    for (int s = 0; s < SIDE_COUNT; s++) {
        cfg.boundary[s] = BC_RIGID;
    }
    cfg.pml.thickness = 4;
    set_point_source(&g_sources[0], 2.0, 2.0, 100.0, 1.0);
    return cfg;
}

/* Number of cells whose centre is inside `shape` (independent oracle for the
 * obstacle rasterisation). pml_only restricts the count to non-physical
 * cells. */
static size_t count_inside(const Model *m, const Shape *shape, int pml_only)
{
    const Grid *g = &m->grid;
    size_t n = 0;
    for (int j = 0; j < g->ny; j++) {
        for (int i = 0; i < g->nx; i++) {
            if (pml_only && grid_cell_is_physical(g, i, j)) {
                continue;
            }
            const Vec2 centre = {grid_cell_x(g, i), grid_cell_y(g, j)};
            if (shape_contains(shape, centre)) {
                n++;
            }
        }
    }
    return n;
}

/* Compare the whole mask against the oracle cell by cell. */
static void check_mask_matches(const Model *m, const Shape *shape)
{
    const Grid *g = &m->grid;
    size_t mismatches = 0;
    size_t ones = 0;
    for (int j = 0; j < g->ny; j++) {
        for (int i = 0; i < g->nx; i++) {
            const Vec2 centre = {grid_cell_x(g, i), grid_cell_y(g, j)};
            const uint8_t want = shape_contains(shape, centre) ? 1u : 0u;
            const uint8_t got = m->solid[grid_cell_index(g, i, j)];
            if (got) {
                ones++;
            }
            if (got != want) {
                mismatches++;
            }
        }
    }
    CHECK_MSG(mismatches == 0, "%zu cells differ from the shape oracle", mismatches);
    CHECK_MSG(m->n_solid == ones, "n_solid = %zu but %zu cells are flagged",
              m->n_solid, ones);
}

/* ------------------------------------------------------------- grid/time */

static void test_build_default_pml(void)
{
    SimConfig cfg = base_config();
    Model m;
    FdtdError err = {{0}};

    fdtd_reset_warning_count();
    CHECK_MSG(model_build(&cfg, &m, &err) == 0, "%s", err.msg);

    CHECK(m.grid.nx_phys == 400);
    CHECK(m.grid.ny_phys == 300);
    CHECK(m.grid.nx == 440);
    CHECK(m.grid.ny == 340);
    CHECK(m.grid.off_x == 20);
    CHECK(m.grid.off_y == 20);
    CHECK(m.grid.dx == 0.005);
    CHECK(m.grid.dy == 0.005);
    CHECK(m.grid.x0 == 0.0);
    CHECK(m.grid.y0 == 0.0);
    CHECK_NEAR(grid_x1(&m.grid), 2.0, 1e-12);
    CHECK_NEAR(grid_y1(&m.grid), 1.5, 1e-12);

    for (int s = 0; s < SIDE_COUNT; s++) {
        CHECK(m.boundary[s] == BC_PML);
        CHECK(m.pml_cells[s] == 20);
    }
    CHECK(m.pml.thickness == 20);
    CHECK(m.pml.reflection == PML_DEFAULT_REFLECTION);
    CHECK(m.pml.order == PML_DEFAULT_ORDER);

    /* medium table: v1 only knows the background medium */
    CHECK(m.media.count == 1);
    CHECK(m.media.items != NULL);
    CHECK(m.media.items[0].density == 1.21);
    CHECK(m.media.items[0].c == 343.0);
    CHECK_NEAR(medium_bulk_modulus(&m.media.items[0]), 1.21 * 343.0 * 343.0, 1e-6);

    /* medium map and rigid mask are zero-initialised */
    const size_t n_cells = grid_cell_count(&m.grid);
    size_t non_zero_medium = 0;
    size_t flagged = 0;
    for (size_t c = 0; c < n_cells; c++) {
        if (m.medium_id[c] != 0) {
            non_zero_medium++;
        }
        if (m.solid[c]) {
            flagged++;
        }
    }
    CHECK(non_zero_medium == 0);
    CHECK(flagged == 0);
    CHECK(m.n_solid == 0);

    /* timestep from cfl */
    const double S = speed_limit(&cfg);
    CHECK_NEAR(m.dt, cfg.cfl / S, 1e-15 * m.dt);
    CHECK_NEAR(m.cfl, cfg.cfl, 1e-15);
    CHECK(m.duration == cfg.duration);
    CHECK(m.n_steps == 555); /* ceil(0.004 / dt) */

    /* one source, four bilinear cells of equal weight */
    CHECK(m.n_sources == 1);
    CHECK(m.sources != NULL);
    CHECK(m.sources[0].shape_type == SHAPE_POINT);
    CHECK(m.sources[0].signal.type == SIGNAL_HARMONIC);
    CHECK(m.sources[0].signal.u.harmonic.frequency == 2000.0);
    CHECK(m.sources[0].n_cells == 4);
    for (size_t k = 0; k < m.sources[0].n_cells; k++) {
        CHECK_NEAR(m.sources[0].weights[k], 0.25 / (cfg.dx * cfg.dy),
                   1e-9 * 0.25 / (cfg.dx * cfg.dy));
    }

    CHECK_MSG(fdtd_warning_count() == 0, "%d unexpected warnings",
              fdtd_warning_count());

    model_free(&m);
}

static void test_build_per_side_pml(void)
{
    SimConfig cfg = base_config();
    cfg.x_range[1] = 0.4;
    cfg.y_range[1] = 0.3;
    cfg.dx = 0.01;
    cfg.dy = 0.01;
    cfg.pml.thickness = 5;
    cfg.boundary[SIDE_X_MIN] = BC_PML;
    cfg.boundary[SIDE_X_MAX] = BC_RIGID;
    cfg.boundary[SIDE_Y_MIN] = BC_PRESSURE_RELEASE;
    cfg.boundary[SIDE_Y_MAX] = BC_PML;
    set_point_source(&g_sources[0], 0.205, 0.155, 1000.0, 1e-3);

    Model m;
    FdtdError err = {{0}};
    fdtd_reset_warning_count();
    CHECK_MSG(model_build(&cfg, &m, &err) == 0, "%s", err.msg);

    CHECK(m.grid.nx_phys == 40);
    CHECK(m.grid.ny_phys == 30);
    CHECK(m.grid.nx == 45); /* 5 + 40 + 0 */
    CHECK(m.grid.ny == 35); /* 0 + 30 + 5 */
    CHECK(m.grid.off_x == 5);
    CHECK(m.grid.off_y == 0);

    CHECK(m.pml_cells[SIDE_X_MIN] == 5);
    CHECK(m.pml_cells[SIDE_X_MAX] == 0);
    CHECK(m.pml_cells[SIDE_Y_MIN] == 0);
    CHECK(m.pml_cells[SIDE_Y_MAX] == 5);
    CHECK(m.boundary[SIDE_X_MIN] == BC_PML);
    CHECK(m.boundary[SIDE_X_MAX] == BC_RIGID);
    CHECK(m.boundary[SIDE_Y_MIN] == BC_PRESSURE_RELEASE);
    CHECK(m.boundary[SIDE_Y_MAX] == BC_PML);

    /* sigma only where a PML exists */
    CHECK(m.sigma_x_cell[4] > 0.0);
    CHECK(m.sigma_x_face[0] > 0.0);
    for (int i = 5; i < 45; i++) {
        CHECK_MSG(m.sigma_x_cell[i] == 0.0, "x cell %d inside domain is %g", i,
                  m.sigma_x_cell[i]);
        CHECK(m.sigma_x_face[i] == 0.0);
    }
    CHECK(m.sigma_x_face[45] == 0.0); /* rigid high side */

    for (int j = 0; j < 30; j++) {
        CHECK_MSG(m.sigma_y_cell[j] == 0.0, "y cell %d outside the PML is %g", j,
                  m.sigma_y_cell[j]);
        CHECK(m.sigma_y_face[j] == 0.0);
    }
    CHECK(m.sigma_y_cell[30] > 0.0);
    CHECK(m.sigma_y_cell[34] > 0.0);
    CHECK(m.sigma_y_face[35] > 0.0); /* outer edge of the y_max PML */

    /* source: (0.205, 0.155) is the centre of physical cell (20, 15) */
    CHECK(m.n_sources == 1);
    CHECK(m.sources[0].n_cells <= 2);
    model_free(&m);
}

static void test_non_divisible_range(void)
{
    SimConfig cfg = base_config();
    Model m;
    FdtdError err = {{0}};

    cfg.x_range[1] = 2.003;
    CHECK(model_build(&cfg, &m, &err) == -1);
    CHECK_CONTAINS(err.msg, "x_range");
    CHECK_CONTAINS(err.msg, "nearest valid dx");
    CHECK(m.solid == NULL);

    cfg = base_config();
    cfg.y_range[1] = 1.5025;
    err.msg[0] = '\0';
    CHECK(model_build(&cfg, &m, &err) == -1);
    CHECK_CONTAINS(err.msg, "y_range");
    CHECK_CONTAINS(err.msg, "nearest valid dy");
    CHECK(m.solid == NULL);

    /* the reported step must actually divide the requested length */
    cfg = base_config();
    cfg.x_range[1] = 2.003;
    CHECK(model_build(&cfg, &m, &err) == -1);
    double suggested = 0.0;
    if (sscanf(strstr(err.msg, "nearest valid dx: "), "nearest valid dx: %lf",
               &suggested) == 1) {
        const double cells = 2.003 / suggested; /* %g rounds: allow 0.1 % */
        CHECK_NEAR(cells, floor(cells + 0.5), 1e-3 * cells);
        CHECK(fabs(suggested - cfg.dx) < 1e-4);
    } else {
        CHECK_MSG(0, "no step suggestion in \"%s\"", err.msg);
    }
}

/* Grid-size limits that model_build() itself enforces (value ranges are
 * config_load_*'s job and tested in test_config.c). */
static void test_grid_size_errors(void)
{
    SimConfig cfg;
    Model m;
    FdtdError err = {{0}};

    /* shorter than one cell */
    cfg = base_config();
    cfg.x_range[1] = 0.001;
    CHECK(model_build(&cfg, &m, &err) == -1);
    CHECK_CONTAINS(err.msg, "smaller than one cell");
    CHECK(m.solid == NULL);

    /* more cells than an int can count */
    cfg = base_config();
    cfg.x_range[1] = 1e12; /* 2e11 cells of 5 mm */
    err.msg[0] = '\0';
    CHECK(model_build(&cfg, &m, &err) == -1);
    CHECK_CONTAINS(err.msg, "more than the supported maximum");
    CHECK(m.solid == NULL);
}

static void test_timestep(void)
{
    SimConfig cfg = base_config();
    Model m;
    FdtdError err = {{0}};
    const double S = speed_limit(&cfg);

    /* explicit dt below the CFL limit */
    cfg.dt_set = true;
    cfg.dt = 5e-6;
    CHECK_MSG(model_build(&cfg, &m, &err) == 0, "%s", err.msg);
    CHECK(m.dt == 5e-6);
    CHECK_NEAR(m.cfl, S * 5e-6, 1e-12);
    CHECK(m.cfl < 1.0);
    CHECK(m.n_steps == 800);
    model_free(&m);

    /* explicit dt above the limit is rejected and names the limit */
    cfg = base_config();
    cfg.dt_set = true;
    cfg.dt = 2e-5;
    err.msg[0] = '\0';
    CHECK(model_build(&cfg, &m, &err) == -1);
    CHECK_CONTAINS(err.msg, "CFL");
    CHECK(m.dt == 0.0 && m.sources == NULL);

    /* cfl = 1 is the stability limit and must be accepted */
    cfg = base_config();
    cfg.cfl = 1.0;
    CHECK_MSG(model_build(&cfg, &m, &err) == 0, "%s", err.msg);
    CHECK_NEAR(m.cfl, 1.0, 1e-12);
    model_free(&m);

    /* Valid but extreme input: a very slow medium and huge cells give a
     * finite, tiny denominator, so the derived dt overflows to inf. */
    cfg = coarse_config();
    cfg.medium.c = 1e-300;
    cfg.x_range[1] = 1e10;
    cfg.y_range[1] = 1e10;
    cfg.dx = 1e10;
    cfg.dy = 1e10;
    set_point_source(&g_sources[0], 5e9, 5e9, 100.0, 1.0);
    err.msg[0] = '\0';
    CHECK(model_build(&cfg, &m, &err) == -1);
    CHECK_CONTAINS(err.msg, "derived time step");
    CHECK(m.dt == 0.0 && m.sources == NULL);
}

static void test_n_steps(void)
{
    SimConfig cfg = coarse_config();
    Model m;
    FdtdError err = {{0}};

    cfg.dt_set = true;
    cfg.dt = 0.001;

    cfg.duration = 0.0025; /* 2.5 steps -> 3 */
    CHECK_MSG(model_build(&cfg, &m, &err) == 0, "%s", err.msg);
    CHECK(m.n_steps == 3);
    model_free(&m);

    cfg.duration = 0.002; /* exactly 2 */
    CHECK(model_build(&cfg, &m, &err) == 0);
    CHECK(m.n_steps == 2);
    model_free(&m);

    cfg.duration = 1e-6; /* far below dt: at least one step */
    CHECK(model_build(&cfg, &m, &err) == 0);
    CHECK(m.n_steps == 1);
    model_free(&m);

    /* duration/dt = 4 * (1 + 2e-10) must not round up to 5 steps */
    cfg.dt = 0.0025;
    cfg.duration = 0.01 * (1.0 + 2e-10);
    CHECK(model_build(&cfg, &m, &err) == 0);
    CHECK_MSG(m.n_steps == 4, "n_steps = %ld", m.n_steps);
    model_free(&m);

    /* a step count that does not fit in a long is rejected */
    cfg.duration = 1e30;
    err.msg[0] = '\0';
    CHECK(model_build(&cfg, &m, &err) == -1);
    CHECK_CONTAINS(err.msg, "too many steps");
    CHECK(m.solid == NULL && m.medium_id == NULL);
}

/* --------------------------------------------------------- PML profiles */

static void test_sigma_profiles(void)
{
    SimConfig cfg = base_config();
    cfg.dx = 0.01;
    cfg.dy = 0.02;
    cfg.x_range[1] = 0.4;
    cfg.y_range[1] = 0.6;
    cfg.pml.thickness = 5;
    set_point_source(&g_sources[0], 0.2, 0.3, 1000.0, 1.0);

    Model m;
    FdtdError err = {{0}};
    CHECK_MSG(model_build(&cfg, &m, &err) == 0, "%s", err.msg);

    CHECK(m.grid.nx == 50); /* 5 + 40 + 5 */
    CHECK(m.grid.ny == 40); /* 5 + 30 + 5 */
    CHECK(m.grid.off_x == 5);
    CHECK(m.grid.off_y == 5);

    const double c_ref = 343.0;
    const double order = cfg.pml.order;
    const double Lx = (double)cfg.pml.thickness * cfg.dx;
    const double Ly = (double)cfg.pml.thickness * cfg.dy;
    const double smax_x = -(order + 1.0) * c_ref * log(cfg.pml.reflection) / (2.0 * Lx);
    const double smax_y = -(order + 1.0) * c_ref * log(cfg.pml.reflection) / (2.0 * Ly);
    CHECK(smax_x > 0.0 && smax_y > 0.0);
    /* the y profile must use dy, not dx */
    CHECK_NEAR(smax_x / smax_y, 2.0, 1e-12);

    /* x_min PML: values at faces and cell centres */
    for (int i = 0; i <= 5; i++) {
        const double d = (5.0 - (double)i) * cfg.dx;
        const double want = smax_x * pow(d / Lx, order);
        CHECK_NEAR(m.sigma_x_face[i], want, 1e-12 * smax_x);
    }
    for (int i = 0; i < 5; i++) {
        const double d = (5.0 - (double)i - 0.5) * cfg.dx;
        const double want = smax_x * pow(d / Lx, order);
        CHECK_NEAR(m.sigma_x_cell[i], want, 1e-12 * smax_x);
    }
    CHECK(m.sigma_x_face[5] == 0.0); /* interface face */
    CHECK_NEAR(m.sigma_x_face[0], smax_x, 1e-12 * smax_x);
    CHECK_NEAR(m.sigma_x_face[2], smax_x * pow(0.6, order), 1e-12 * smax_x);
    CHECK_NEAR(m.sigma_x_cell[4], smax_x * pow(0.1, order), 1e-12 * smax_x);

    /* monotonic increase into the PML on both sides */
    for (int i = 0; i < 4; i++) {
        CHECK(m.sigma_x_face[i] > m.sigma_x_face[i + 1]);
        CHECK(m.sigma_x_cell[i] > m.sigma_x_cell[i + 1]);
    }
    CHECK(m.sigma_x_cell[44] == 0.0); /* last physical cell */
    for (int i = 45; i < 49; i++) {
        CHECK_MSG(m.sigma_x_cell[i] < m.sigma_x_cell[i + 1],
                  "cell %d = %g, cell %d = %g", i, m.sigma_x_cell[i], i + 1,
                  m.sigma_x_cell[i + 1]);
        CHECK(m.sigma_x_face[i] < m.sigma_x_face[i + 1]);
    }

    /* zero inside the physical domain, on every non-PML face and at both
     * interfaces */
    for (int i = 5; i < 45; i++) {
        CHECK(m.sigma_x_cell[i] == 0.0);
        CHECK(m.sigma_x_face[i] == 0.0);
    }
    CHECK(m.sigma_x_face[45] == 0.0);

    /* x_max PML */
    for (int i = 45; i <= 50; i++) {
        const double d = ((double)i - 45.0) * cfg.dx;
        CHECK_NEAR(m.sigma_x_face[i], smax_x * pow(d / Lx, order), 1e-12 * smax_x);
    }
    for (int i = 45; i < 50; i++) {
        const double d = ((double)i + 0.5 - 45.0) * cfg.dx;
        CHECK_NEAR(m.sigma_x_cell[i], smax_x * pow(d / Lx, order), 1e-12 * smax_x);
    }

    /* y profiles use dy */
    for (int j = 0; j <= 5; j++) {
        const double d = (5.0 - (double)j) * cfg.dy;
        CHECK_NEAR(m.sigma_y_face[j], smax_y * pow(d / Ly, order), 1e-12 * smax_y);
    }
    for (int j = 0; j < 5; j++) {
        const double d = (5.0 - (double)j - 0.5) * cfg.dy;
        CHECK_NEAR(m.sigma_y_cell[j], smax_y * pow(d / Ly, order), 1e-12 * smax_y);
    }
    CHECK(m.sigma_y_face[5] == 0.0);
    CHECK(m.sigma_y_face[0] > 0.0);
    CHECK_NEAR(m.sigma_y_face[0], smax_y, 1e-12 * smax_y);
    for (int j = 5; j < 35; j++) {
        CHECK(m.sigma_y_cell[j] == 0.0);
        CHECK(m.sigma_y_face[j] == 0.0);
    }
    /* y_max PML (interface face 35) */
    CHECK(m.sigma_y_face[35] == 0.0);
    for (int j = 35; j <= 40; j++) {
        const double d = ((double)j - 35.0) * cfg.dy;
        CHECK_NEAR(m.sigma_y_face[j], smax_y * pow(d / Ly, order), 1e-12 * smax_y);
    }
    for (int j = 35; j < 40; j++) {
        const double d = ((double)j + 0.5 - 35.0) * cfg.dy;
        CHECK_NEAR(m.sigma_y_cell[j], smax_y * pow(d / Ly, order), 1e-12 * smax_y);
    }
    CHECK_NEAR(m.sigma_y_face[40], smax_y, 1e-12 * smax_y);
    model_free(&m);
}

/* ------------------------------------------------------------ obstacles */

static void test_obstacle_circle(void)
{
    SimConfig cfg = base_config();
    /* fully inside the physical domain (touching the PML would warn) */
    const Shape *shape = set_circle_obstacle(0, 1.4, 1.1, 0.25);
    cfg.n_obstacles = 1;

    Model m;
    FdtdError err = {{0}};
    fdtd_reset_warning_count();
    CHECK_MSG(model_build(&cfg, &m, &err) == 0, "%s", err.msg);
    CHECK(fdtd_warning_count() == 0);

    check_mask_matches(&m, shape);

    /* the number of cells must be close to area / cell area */
    const double pi = acos(-1.0);
    const double expected = pi * 0.25 * 0.25 / (cfg.dx * cfg.dy);
    CHECK_MSG(fabs((double)m.n_solid - expected) / expected < 0.05,
              "%zu solid cells, expected about %.0f", m.n_solid, expected);
    CHECK(m.n_solid > 0);

    /* the source is far from the circle and must stay valid */
    CHECK(m.n_sources == 1);
    model_free(&m);
}

static void test_obstacle_polygon(void)
{
    SimConfig cfg = base_config();
    g_polygon[0].x = 1.3;
    g_polygon[0].y = 0.0;
    g_polygon[1].x = 1.7;
    g_polygon[1].y = 0.0;
    g_polygon[2].x = 1.5;
    g_polygon[2].y = 0.5;
    const Shape *shape = set_polygon_obstacle(0, 3);
    cfg.n_obstacles = 1;

    Model m;
    FdtdError err = {{0}};
    fdtd_reset_warning_count();
    CHECK_MSG(model_build(&cfg, &m, &err) == 0, "%s", err.msg);
    CHECK_MSG(fdtd_warning_count() == 0, "%d warnings", fdtd_warning_count());

    check_mask_matches(&m, shape);

    const double expected = 0.5 * 0.4 * 0.5 / (cfg.dx * cfg.dy); /* triangle area */
    CHECK_MSG(fabs((double)m.n_solid - expected) / expected < 0.06,
              "%zu solid cells, expected about %.0f", m.n_solid, expected);
    CHECK(m.n_solid > 0);

    /* two overlapping obstacles: the mask is the union */
    Model m2;
    SimConfig cfg2 = cfg;
    set_circle_obstacle(1, 1.5, 0.15, 0.12);
    cfg2.n_obstacles = 2;
    CHECK(model_build(&cfg2, &m2, &err) == 0);
    CHECK(m2.n_solid >= m.n_solid);
    CHECK(m2.n_solid <= m.n_solid + count_inside(&m2, &g_obstacles[1].shape, 0));
    model_free(&m2);
    model_free(&m);
}

static void test_obstacle_reaches_into_pml(void)
{
    SimConfig cfg = base_config();
    /* straddles the physical x_max edge at x = 2.0 */
    const Shape *shape = set_rect_obstacle(0, 1.95, 0.5, 2.05, 0.6);
    cfg.n_obstacles = 1;

    Model m;
    FdtdError err = {{0}};
    fdtd_reset_warning_count();
    CHECK_MSG(model_build(&cfg, &m, &err) == 0, "%s", err.msg);
    CHECK(fdtd_warning_count() == 1); /* "extends into the PML" */

    check_mask_matches(&m, shape);
    CHECK(count_inside(&m, shape, 1) > 0);
    CHECK(m.n_solid > 0);

    /* at least one solid cell must be a PML cell outside the physical domain */
    size_t pml_solid = 0;
    for (size_t c = 0; c < grid_cell_count(&m.grid); c++) {
        if (!m.solid[c]) {
            continue;
        }
        const int j = (int)(c / (size_t)m.grid.nx);
        const int i = (int)(c % (size_t)m.grid.nx);
        if (!grid_cell_is_physical(&m.grid, i, j)) {
            pml_solid++;
        }
    }
    CHECK(pml_solid > 0);
    CHECK(pml_solid == count_inside(&m, shape, 1));
    model_free(&m);
}

static void test_obstacle_without_cell_centre_warns(void)
{
    SimConfig cfg = base_config();
    /* cell centres are at 0.5025 + k*0.005: this rect contains none */
    set_rect_obstacle(0, 0.5005, 0.5005, 0.502, 0.502);
    cfg.n_obstacles = 1;

    Model m;
    FdtdError err = {{0}};
    fdtd_reset_warning_count();
    CHECK_MSG(model_build(&cfg, &m, &err) == 0, "%s", err.msg);
    CHECK_MSG(fdtd_warning_count() == 1, "%d warnings", fdtd_warning_count());
    CHECK(m.n_solid == 0);
    model_free(&m);
}

/* ------------------------------------------------- warnings / error paths */

static void test_low_resolution_warns(void)
{
    SimConfig cfg = coarse_config();
    /* 4 m cells at 2 kHz: 343 / (2000 * 4) << 10 points per wavelength */
    set_point_source(&g_sources[0], 2.0, 2.0, 2000.0, 1.0);

    Model m;
    FdtdError err = {{0}};
    fdtd_reset_warning_count();
    CHECK_MSG(model_build(&cfg, &m, &err) == 0, "%s", err.msg);
    CHECK_MSG(fdtd_warning_count() == 1, "%d warnings", fdtd_warning_count());
    model_free(&m);

    /* the same grid with a low enough frequency must not warn */
    set_point_source(&g_sources[0], 2.0, 2.0, 5.0, 1.0);
    fdtd_reset_warning_count();
    CHECK(model_build(&cfg, &m, &err) == 0);
    CHECK_MSG(fdtd_warning_count() == 0, "%d warnings", fdtd_warning_count());
    model_free(&m);
}

static void test_no_sources_warns(void)
{
    SimConfig cfg = base_config();
    cfg.n_sources = 0;

    Model m;
    FdtdError err = {{0}};
    fdtd_reset_warning_count();
    CHECK_MSG(model_build(&cfg, &m, &err) == 0, "%s", err.msg);
    CHECK(fdtd_warning_count() == 1);
    CHECK(m.n_sources == 0);
    CHECK(m.sources == NULL);
    model_free(&m);
}

static void test_source_error_is_prefixed(void)
{
    SimConfig cfg = base_config();
    set_point_source(&g_sources[1], 5.0, 0.75, 2000.0, 1e-3); /* outside */
    cfg.n_sources = 2;

    Model m;
    FdtdError err = {{0}};
    CHECK(model_build(&cfg, &m, &err) == -1);
    CHECK_CONTAINS(err.msg, "source[1]");
    CHECK_CONTAINS(err.msg, "outside the physical domain");
    CHECK(m.sources == NULL);
    CHECK(m.n_sources == 0);

    /* a source inside an obstacle is reported with the same prefix */
    cfg = base_config();
    set_rect_obstacle(0, 0.5, 0.5, 1.0, 1.0);
    cfg.n_obstacles = 1;
    err.msg[0] = '\0';
    CHECK(model_build(&cfg, &m, &err) == -1);
    CHECK_CONTAINS(err.msg, "source[0]");
    CHECK_CONTAINS(err.msg, "lies inside a rigid obstacle");
}

static void test_model_free_and_cleanup(void)
{
    SimConfig cfg = base_config();
    Model m;
    FdtdError err = {{0}};

    CHECK(model_build(&cfg, &m, &err) == 0);
    model_free(&m);
    /* model_free leaves a fully zeroed struct and is safe to repeat */
    CHECK(m.sources == NULL);
    CHECK(m.solid == NULL);
    CHECK(m.medium_id == NULL);
    CHECK(m.sigma_x_cell == NULL);
    CHECK(m.sigma_x_face == NULL);
    CHECK(m.sigma_y_cell == NULL);
    CHECK(m.sigma_y_face == NULL);
    CHECK(m.media.items == NULL);
    CHECK(m.media.count == 0);
    CHECK(m.n_sources == 0);
    CHECK(m.n_steps == 0);
    model_free(&m);

    /* a failed build leaves nothing to free */
    cfg.x_range[1] = 2.003;
    CHECK(model_build(&cfg, &m, &err) == -1);
    CHECK(m.sources == NULL && m.solid == NULL && m.medium_id == NULL);
    model_free(&m);
}

/* --------------------------------------------------------------- summary */

static void test_print_summary(void)
{
    SimConfig cfg = base_config();
    Model m;
    FdtdError err = {{0}};
    CHECK(model_build(&cfg, &m, &err) == 0);

    FILE *f = tmpfile();
    CHECK(f != NULL);
    if (!f) {
        model_free(&m);
        return;
    }
    model_print_summary(f, &m);
    rewind(f);

    static char buf[8192];
    const size_t n = fread(buf, 1, sizeof buf - 1, f);
    buf[n] = '\0';
    fclose(f);

    CHECK(n > 0);
    CHECK_CONTAINS(buf, "400 x 300 cells physical");
    CHECK_CONTAINS(buf, "440 x 340 cells total");
    CHECK_CONTAINS(buf, "dx = 0.005 m, dy = 0.005 m");
    CHECK_CONTAINS(buf, "domain x = [0, 2] m, y = [0, 1.5] m");
    CHECK_CONTAINS(buf, "pml cells: x_min 20, x_max 20, y_min 20, y_max 20");
    CHECK_CONTAINS(buf, "n_steps = 555");
    CHECK_CONTAINS(buf, "cfl = 0.7");
    CHECK_CONTAINS(buf, "points per wavelength = 34.30");
    CHECK_CONTAINS(buf, "medium: [0] rho = 1.21 kg/m^3, c = 343 m/s");
    CHECK_CONTAINS(buf, "boundary: x_min pml, x_max pml, y_min pml, y_max pml");
    CHECK_CONTAINS(buf, "obstacles: 0 solid cells");
    CHECK_CONTAINS(buf, "sources: 1");
    CHECK_CONTAINS(buf, "point source, 4 cells");
    CHECK_CONTAINS(buf, "f = 2000 Hz");

    model_free(&m);

    /* a model without periodic content prints "n/a" for the resolution */
    cfg = base_config();
    cfg.n_sources = 0;
    fdtd_reset_warning_count();
    CHECK(model_build(&cfg, &m, &err) == 0);
    f = tmpfile();
    CHECK(f != NULL);
    if (f) {
        model_print_summary(f, &m);
        rewind(f);
        const size_t n2 = fread(buf, 1, sizeof buf - 1, f);
        buf[n2] = '\0';
        fclose(f);
        CHECK_CONTAINS(buf, "points per wavelength = n/a");
        CHECK_CONTAINS(buf, "sources: 0");
    }
    model_free(&m);
}

static void test_anisotropic_cells(void)
{
    SimConfig cfg = base_config();
    cfg.dx = 0.01;
    cfg.dy = 0.02;
    cfg.x_range[1] = 0.4;
    cfg.y_range[1] = 0.4;
    set_point_source(&g_sources[0], 0.2, 0.2, 500.0, 1.0);

    Model m;
    FdtdError err = {{0}};
    CHECK_MSG(model_build(&cfg, &m, &err) == 0, "%s", err.msg);
    CHECK(m.grid.nx_phys == 40);
    CHECK(m.grid.ny_phys == 20);
    CHECK(m.grid.dx == 0.01);
    CHECK(m.grid.dy == 0.02);
    CHECK_NEAR(grid_cell_x(&m.grid, m.grid.off_x), 0.005, 1e-12);
    CHECK_NEAR(grid_cell_y(&m.grid, m.grid.off_y), 0.01, 1e-12);
    CHECK_NEAR(m.dt, cfg.cfl / speed_limit(&cfg), 1e-15 * m.dt);
    CHECK(m.n_sources == 1);
    CHECK(m.n_sources == 1 && m.sources[0].n_cells > 0);
    CHECK_NEAR(m.sources[0].weights[0] * m.grid.dx * m.grid.dy * 4.0,
               1.0 /* four equal cells */, 1e-9);
    model_free(&m);
}

int main(void)
{
    fdtd_set_warning_stream(NULL);

    RUN_TEST(test_build_default_pml);
    RUN_TEST(test_build_per_side_pml);
    RUN_TEST(test_non_divisible_range);
    RUN_TEST(test_grid_size_errors);
    RUN_TEST(test_timestep);
    RUN_TEST(test_n_steps);
    RUN_TEST(test_sigma_profiles);
    RUN_TEST(test_obstacle_circle);
    RUN_TEST(test_obstacle_polygon);
    RUN_TEST(test_obstacle_reaches_into_pml);
    RUN_TEST(test_obstacle_without_cell_centre_warns);
    RUN_TEST(test_low_resolution_warns);
    RUN_TEST(test_no_sources_warns);
    RUN_TEST(test_source_error_is_prefixed);
    RUN_TEST(test_model_free_and_cleanup);
    RUN_TEST(test_print_summary);
    RUN_TEST(test_anisotropic_cells);

    return TEST_REPORT();
}
