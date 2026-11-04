/*
 * test_config.c - unit tests for the TOML config parser (parser layer 1).
 *
 * Every error message is "<source>:<line>: <key path>: <problem>", so the
 * error tests assert on the whole "<source>:<line>: <key path>" prefix: that
 * pins down both the reported location and the key path.
 */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "test_util.h"

#define TEST_SOURCE "case.toml"

/* ---------------------------------------------------------------- */
/* helpers                                                          */
/* ---------------------------------------------------------------- */

/* Load `text` and require success; returns the config for inspection. */
static void load_ok(const char *text, SimConfig *cfg)
{
    FdtdError err = {{0}};
    memset(cfg, 0, sizeof *cfg);
    int rc = config_load_string(text, TEST_SOURCE, cfg, &err);
    CHECK_MSG(rc == 0, "unexpected failure: %s", err.msg);
    if (rc != 0) {
        config_free(cfg);
    }
}

/* Require a failure whose message contains every fragment in `fragments`.
 * Also checks that the message is a single line and that cfg is left zeroed
 * and safe to free. */
static void load_fails(const char *text, const char *fragments[], int n_fragments)
{
    SimConfig cfg;
    FdtdError err = {{0}};
    memset(&cfg, 0, sizeof cfg);
    memset(&err, 0, sizeof err);
    int rc = config_load_string(text, TEST_SOURCE, &cfg, &err);
    CHECK_MSG(rc != 0, "expected a parse failure, got success (message: %s)", err.msg);
    if (rc == 0) {
        config_free(&cfg);
        return;
    }
    CHECK_MSG(err.msg[0] != '\0', "failure without a message");
    CHECK_MSG(strchr(err.msg, '\n') == NULL, "message is not a single line: %s", err.msg);
    for (int i = 0; i < n_fragments; i++) {
        CHECK_CONTAINS(err.msg, fragments[i]);
    }
    /* A failed load must hand back an empty, freeable config. */
    CHECK(cfg.source_name == NULL);
    CHECK(cfg.sources == NULL);
    CHECK(cfg.obstacles == NULL);
    CHECK(cfg.output.directory == NULL);
    CHECK(cfg.n_sources == 0);
    CHECK(cfg.n_obstacles == 0);
    config_free(&cfg);
    config_free(&cfg); /* must be safe twice */
}

#define LOAD_FAILS(text, ...)                                                  \
    do {                                                                       \
        const char *fragments_[] = {__VA_ARGS__};                              \
        load_fails((text), fragments_, (int)(sizeof fragments_ / sizeof fragments_[0])); \
    } while (0)

/* Minimal valid document; the error tests append/replace pieces of it. */
static const char *k_minimal =
    "[simulation]\n"
    "duration = 0.1\n"
    "[grid]\n"
    "x_range = [0.0, 1.0]\n"
    "y_range = [0.0, 1.0]\n"
    "dx = 0.01\n"
    "[medium]\n"
    "density = 1.2\n"
    "c = 343.0\n";

/* ---------------------------------------------------------------- */
/* happy paths                                                      */
/* ---------------------------------------------------------------- */

static void test_full_document(void)
{
    static const char *text =
        "[simulation]\n"                                                   /* 1 */
        "duration = 0.5\n"                                                 /* 2 */
        "dt = 1e-6\n"                                                      /* 3 */
        "\n"                                                               /* 4 */
        "[grid]\n"                                                         /* 5 */
        "x_range = [0.0, 2.0]\n"                                           /* 6 */
        "y_range = [-0.5, 0.5]\n"                                          /* 7 */
        "dx = 0.01\n"                                                      /* 8 */
        "dy = 0.02\n"                                                      /* 9 */
        "\n"                                                               /* 10 */
        "[medium]\n"                                                       /* 11 */
        "density = 1000.0\n"                                               /* 12 */
        "c = 1500.0\n"                                                     /* 13 */
        "\n"                                                               /* 14 */
        "[boundary]\n"                                                     /* 15 */
        "x_min = \"rigid\"\n"                                              /* 16 */
        "x_max = \"pressure_release\"\n"                                   /* 17 */
        "y_min = \"pml\"\n"                                                /* 18 */
        "y_max = \"rigid\"\n"                                              /* 19 */
        "\n"                                                               /* 20 */
        "[boundary.pml]\n"                                                 /* 21 */
        "thickness = 12\n"                                                 /* 22 */
        "reflection = 1e-4\n"                                              /* 23 */
        "order = 3.5\n"                                                    /* 24 */
        "\n"                                                               /* 25 */
        "[[obstacle]]\n"                                                   /* 26 */
        "shape = { type = \"rect\", min = [0.2, 0.3], max = [0.4, 0.6] }\n" /* 27 */
        "\n"                                                               /* 28 */
        "[[obstacle]]\n"                                                   /* 29 */
        "shape = { type = \"circle\", center = [1.0, 0.0], radius = 0.25 }\n" /* 30 */
        "\n"                                                               /* 31 */
        "[[obstacle]]\n"                                                   /* 32 */
        "shape = { type = \"polygon\", vertices = [[0.0, 0.0], [1.0, 0.0], [0.5, 0.5]] }\n" /* 33 */
        "\n"                                                               /* 34 */
        "[[source]]\n"                                                     /* 35 */
        "shape  = { type = \"point\", position = [0.5, 0.25] }\n"          /* 36 */
        "signal = { type = \"harmonic\", frequency = 1000.0, amplitude = 2e-3, "\
        "phase = 1.5, ramp_cycles = 4.0 }\n"                               /* 37 */
        "\n"                                                               /* 38 */
        "[[source]]\n"                                                     /* 39 */
        "shape = { type = \"segment\", start = [0.1, -0.4], end = [0.1, 0.4] }\n" /* 40 */
        "signal = { type = \"harmonic\", frequency = 500.0, amplitude = 1e-3 }\n" /* 41 */
        "\n"                                                               /* 42 */
        "[output]\n"                                                       /* 43 */
        "directory = \"out/case\"\n"                                       /* 44 */
        "every = 25\n"                                                     /* 45 */
        "color_limit = 3.5\n"                                              /* 46 */
        "show_pml = true\n"                                                /* 47 */
        "pixel_scale = 4\n";                                               /* 48 */

    SimConfig cfg;
    load_ok(text, &cfg);

    CHECK_NEAR(cfg.duration, 0.5, 0.0);
    CHECK(cfg.dt_set);
    CHECK_NEAR(cfg.dt, 1e-6, 0.0);
    CHECK_NEAR(cfg.cfl, CONFIG_DEFAULT_CFL, 0.0); /* untouched default */

    CHECK_NEAR(cfg.x_range[0], 0.0, 0.0);
    CHECK_NEAR(cfg.x_range[1], 2.0, 0.0);
    CHECK_NEAR(cfg.y_range[0], -0.5, 0.0);
    CHECK_NEAR(cfg.y_range[1], 0.5, 0.0);
    CHECK_NEAR(cfg.dx, 0.01, 0.0);
    CHECK_NEAR(cfg.dy, 0.02, 0.0);

    CHECK_NEAR(cfg.medium.density, 1000.0, 0.0);
    CHECK_NEAR(cfg.medium.c, 1500.0, 0.0);

    CHECK(cfg.boundary[SIDE_X_MIN] == BC_RIGID);
    CHECK(cfg.boundary[SIDE_X_MAX] == BC_PRESSURE_RELEASE);
    CHECK(cfg.boundary[SIDE_Y_MIN] == BC_PML);
    CHECK(cfg.boundary[SIDE_Y_MAX] == BC_RIGID);

    CHECK(cfg.pml.thickness == 12);
    CHECK_NEAR(cfg.pml.reflection, 1e-4, 0.0);
    CHECK_NEAR(cfg.pml.order, 3.5, 0.0);

    CHECK(cfg.n_obstacles == 3);
    if (cfg.n_obstacles == 3) {
        CHECK(cfg.obstacles[0].shape.type == SHAPE_RECT);
        CHECK_NEAR(cfg.obstacles[0].shape.u.rect.min.x, 0.2, 0.0);
        CHECK_NEAR(cfg.obstacles[0].shape.u.rect.min.y, 0.3, 0.0);
        CHECK_NEAR(cfg.obstacles[0].shape.u.rect.max.x, 0.4, 0.0);
        CHECK_NEAR(cfg.obstacles[0].shape.u.rect.max.y, 0.6, 0.0);

        CHECK(cfg.obstacles[1].shape.type == SHAPE_CIRCLE);
        CHECK_NEAR(cfg.obstacles[1].shape.u.circle.center.x, 1.0, 0.0);
        CHECK_NEAR(cfg.obstacles[1].shape.u.circle.center.y, 0.0, 0.0);
        CHECK_NEAR(cfg.obstacles[1].shape.u.circle.radius, 0.25, 0.0);

        CHECK(cfg.obstacles[2].shape.type == SHAPE_POLYGON);
        CHECK(cfg.obstacles[2].shape.u.polygon.n == 3);
        if (cfg.obstacles[2].shape.u.polygon.n == 3) {
            CHECK_NEAR(cfg.obstacles[2].shape.u.polygon.vertices[0].x, 0.0, 0.0);
            CHECK_NEAR(cfg.obstacles[2].shape.u.polygon.vertices[0].y, 0.0, 0.0);
            CHECK_NEAR(cfg.obstacles[2].shape.u.polygon.vertices[1].x, 1.0, 0.0);
            CHECK_NEAR(cfg.obstacles[2].shape.u.polygon.vertices[1].y, 0.0, 0.0);
            CHECK_NEAR(cfg.obstacles[2].shape.u.polygon.vertices[2].x, 0.5, 0.0);
            CHECK_NEAR(cfg.obstacles[2].shape.u.polygon.vertices[2].y, 0.5, 0.0);
        }
    }

    CHECK(cfg.n_sources == 2);
    if (cfg.n_sources == 2) {
        const SourceConfig *s0 = &cfg.sources[0];
        CHECK(s0->shape.type == SHAPE_POINT);
        CHECK_NEAR(s0->shape.u.point.position.x, 0.5, 0.0);
        CHECK_NEAR(s0->shape.u.point.position.y, 0.25, 0.0);
        CHECK(s0->signal.type == SIGNAL_HARMONIC);
        CHECK_NEAR(s0->signal.u.harmonic.frequency, 1000.0, 0.0);
        CHECK_NEAR(s0->signal.u.harmonic.amplitude, 2e-3, 0.0);
        CHECK_NEAR(s0->signal.u.harmonic.phase, 1.5, 0.0);
        CHECK_NEAR(s0->signal.u.harmonic.ramp_cycles, 4.0, 0.0);

        const SourceConfig *s1 = &cfg.sources[1];
        CHECK(s1->shape.type == SHAPE_SEGMENT);
        CHECK_NEAR(s1->shape.u.segment.start.x, 0.1, 0.0);
        CHECK_NEAR(s1->shape.u.segment.start.y, -0.4, 0.0);
        CHECK_NEAR(s1->shape.u.segment.end.x, 0.1, 0.0);
        CHECK_NEAR(s1->shape.u.segment.end.y, 0.4, 0.0);
        CHECK_NEAR(s1->signal.u.harmonic.frequency, 500.0, 0.0);
        CHECK_NEAR(s1->signal.u.harmonic.amplitude, 1e-3, 0.0);
        CHECK_NEAR(s1->signal.u.harmonic.phase, 0.0, 0.0); /* default */
        CHECK_NEAR(s1->signal.u.harmonic.ramp_cycles, CONFIG_DEFAULT_RAMP_CYCLES, 0.0);
    }

    CHECK(cfg.output.directory != NULL);
    if (cfg.output.directory) {
        CHECK(strcmp(cfg.output.directory, "out/case") == 0);
    }
    CHECK(cfg.output.every == 25);
    CHECK(!cfg.output.color_auto);
    CHECK_NEAR(cfg.output.color_limit, 3.5, 0.0);
    CHECK(cfg.output.show_pml);
    CHECK(cfg.output.pixel_scale == 4);

    CHECK(cfg.source_name != NULL);
    if (cfg.source_name) {
        CHECK(strcmp(cfg.source_name, TEST_SOURCE) == 0);
    }

    config_free(&cfg);
}

static void test_defaults(void)
{
    SimConfig cfg;
    load_ok(k_minimal, &cfg);

    CHECK(cfg.dt_set == false);
    CHECK_NEAR(cfg.cfl, CONFIG_DEFAULT_CFL, 0.0);
    CHECK_NEAR(cfg.dy, cfg.dx, 0.0); /* dy defaults to dx */

    for (int side = 0; side < SIDE_COUNT; side++) {
        CHECK_MSG(cfg.boundary[side] == BC_PML, "side %d is not pml by default", side);
    }
    CHECK(cfg.pml.thickness == PML_DEFAULT_THICKNESS);
    CHECK_NEAR(cfg.pml.reflection, PML_DEFAULT_REFLECTION, 0.0);
    CHECK_NEAR(cfg.pml.order, PML_DEFAULT_ORDER, 0.0);

    CHECK(cfg.n_obstacles == 0);
    CHECK(cfg.obstacles == NULL);
    CHECK(cfg.n_sources == 0);
    CHECK(cfg.sources == NULL);

    CHECK(cfg.output.directory != NULL);
    if (cfg.output.directory) {
        CHECK(strcmp(cfg.output.directory, CONFIG_DEFAULT_OUTPUT_DIR) == 0);
    }
    CHECK(cfg.output.every == CONFIG_DEFAULT_OUTPUT_EVERY);
    CHECK(cfg.output.color_auto);
    CHECK_NEAR(cfg.output.color_limit, 0.0, 0.0);
    CHECK(!cfg.output.show_pml);
    CHECK(cfg.output.pixel_scale == CONFIG_DEFAULT_PIXEL_SCALE);

    config_free(&cfg);
}

static void test_integers_accepted_as_floats(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = 1\n"                /* int -> double */
        "cfl = 1\n"                     /* int, and 1 is the inclusive limit */
        "[grid]\n"
        "x_range = [0, 2]\n"            /* ints inside an array */
        "y_range = [0, 1]\n"
        "dx = 1\n"
        "[medium]\n"
        "density = 2\n"
        "c = 343\n"
        "[boundary.pml]\n"
        "thickness = 20\n"
        "reflection = 0.001\n"
        "order = 2\n"                   /* int grading exponent */
        "[[source]]\n"
        "shape = { type = \"point\", position = [1, 1] }\n"
        "signal = { type = \"harmonic\", frequency = 100, amplitude = 5, "\
        "phase = 0, ramp_cycles = 3 }\n"
        "[output]\n"
        "every = 5\n"
        "color_limit = 2\n"             /* int fixed colour scale */
        "pixel_scale = 1\n";

    SimConfig cfg;
    load_ok(text, &cfg);

    CHECK_NEAR(cfg.duration, 1.0, 0.0);
    CHECK_NEAR(cfg.cfl, 1.0, 0.0);
    CHECK_NEAR(cfg.x_range[1], 2.0, 0.0);
    CHECK_NEAR(cfg.dx, 1.0, 0.0);
    CHECK_NEAR(cfg.dy, 1.0, 0.0);
    CHECK_NEAR(cfg.pml.order, 2.0, 0.0);
    CHECK(cfg.n_sources == 1);
    if (cfg.n_sources == 1) {
        CHECK_NEAR(cfg.sources[0].signal.u.harmonic.frequency, 100.0, 0.0);
        CHECK_NEAR(cfg.sources[0].signal.u.harmonic.amplitude, 5.0, 0.0);
        CHECK_NEAR(cfg.sources[0].signal.u.harmonic.ramp_cycles, 3.0, 0.0);
        CHECK_NEAR(cfg.sources[0].shape.u.point.position.x, 1.0, 0.0);
    }
    CHECK(!cfg.output.color_auto);
    CHECK_NEAR(cfg.output.color_limit, 2.0, 0.0);

    config_free(&cfg);
}

static void test_dotted_shape_tables(void)
{
    /* Regular sub-tables (dotted keys) must behave like inline tables. */
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n"
        "[[source]]\n"
        "shape.type = \"segment\"\n"
        "shape.start = [0.2, 0.2]\n"
        "shape.end = [0.2, 0.8]\n"
        "signal.type = \"harmonic\"\n"
        "signal.frequency = 750.0\n"
        "signal.amplitude = 1e-3\n"
        "[[obstacle]]\n"
        "shape.type = \"circle\"\n"
        "shape.center = [0.5, 0.5]\n"
        "shape.radius = 0.1\n";

    SimConfig cfg;
    load_ok(text, &cfg);

    CHECK(cfg.n_sources == 1);
    if (cfg.n_sources == 1) {
        CHECK(cfg.sources[0].shape.type == SHAPE_SEGMENT);
        CHECK_NEAR(cfg.sources[0].shape.u.segment.end.y, 0.8, 0.0);
        CHECK_NEAR(cfg.sources[0].signal.u.harmonic.frequency, 750.0, 0.0);
    }
    CHECK(cfg.n_obstacles == 1);
    if (cfg.n_obstacles == 1) {
        CHECK(cfg.obstacles[0].shape.type == SHAPE_CIRCLE);
        CHECK_NEAR(cfg.obstacles[0].shape.u.circle.radius, 0.1, 0.0);
    }

    config_free(&cfg);
}

static void test_bad_document(void)
{
    SimConfig cfg;
    FdtdError err = {{0}};
    CHECK(config_load_string("this is not toml = = =\n", TEST_SOURCE, &cfg, &err) != 0);
    CHECK_CONTAINS(err.msg, TEST_SOURCE);
    CHECK(err.msg[0] != '\0');
    CHECK(cfg.source_name == NULL);
    config_free(&cfg);
}

/* ---------------------------------------------------------------- */
/* error classes                                                    */
/* ---------------------------------------------------------------- */

static void test_error_unknown_top_level_key(void)
{
    static const char *text =
        "foo = 1\n" /* line 1 */
        "[simulation]\n"
        "duration = 0.1\n";
    LOAD_FAILS(text, TEST_SOURCE ":1: ", "foo: unknown key");
}

static void test_error_unknown_key_in_grid(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "[grid]\n"                    /* line 3 */
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "dxx = 0.01\n"                /* line 7 */
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n";
    LOAD_FAILS(text, TEST_SOURCE ":7: ", "grid.dxx: unknown key");
}

static void test_error_unknown_key_in_shape(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n"
        "[[source]]\n"                                              /* line 10 */
        "shape = { type = \"point\", positon = [0.5, 0.5] }\n"      /* line 11 */
        "signal = { type = \"harmonic\", frequency = 100.0, amplitude = 1.0 }\n";
    LOAD_FAILS(text, TEST_SOURCE ":11: ", "source[0].shape.positon: unknown key");
}

static void test_error_unknown_key_in_signal(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n"
        "[[source]]\n"
        "shape = { type = \"point\", position = [0.5, 0.5] }\n"
        "signal = { type = \"harmonic\", frequency = 100.0, amplitude = 1.0, "\
        "amplitud = 1.0 }\n";                                        /* line 12 */
    LOAD_FAILS(text, TEST_SOURCE ":12: ", "source[0].signal.amplitud: unknown key");
}

static void test_error_wrong_type(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = \"0.1\"\n" /* line 2: string instead of number */
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n";
    LOAD_FAILS(text, TEST_SOURCE ":2: ", "simulation.duration: must be a finite number",
               "a string");
}

static void test_error_integer_field_rejects_float(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n"
        "[output]\n"                 /* line 10 */
        "every = 2.0\n";             /* line 11 */
    LOAD_FAILS(text, TEST_SOURCE ":11: ", "output.every: must be an integer",
               "a float");
}

static void test_error_non_finite(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n"
        "[[source]]\n"
        "shape = { type = \"point\", position = [0.5, 0.5] }\n"
        "signal = { type = \"harmonic\", frequency = inf, amplitude = 1.0 }\n"; /* 12 */
    LOAD_FAILS(text, TEST_SOURCE ":12: ", "source[0].signal.frequency: must be a finite number",
               "non-finite");
}

static void test_error_missing_required_key(void)
{
    static const char *text =
        "[simulation]\n"    /* line 1: no duration -> reported at the table line */
        "cfl = 0.7\n"
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n";
    LOAD_FAILS(text, TEST_SOURCE ":1: ", "simulation.duration: missing required key");
}

static void test_error_missing_section(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n";
    LOAD_FAILS(text, TEST_SOURCE ": ", "medium: missing required table");
}

static void test_error_missing_shape_key(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n"
        "[[obstacle]]\n"                                        /* line 10 */
        "shape = { type = \"circle\", center = [0.5, 0.5] }\n"; /* line 11 */
    LOAD_FAILS(text, TEST_SOURCE ":11: ", "obstacle[0].shape.radius: missing required key");
}

static void test_error_cfl_out_of_range(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "cfl = 1.5\n" /* line 3 */
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n";
    LOAD_FAILS(text, TEST_SOURCE ":3: ", "simulation.cfl: must satisfy 0 < cfl <= 1",
               "got 1.5");
}

static void test_error_negative_duration(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = -1.0\n" /* line 2 */
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n";
    LOAD_FAILS(text, TEST_SOURCE ":2: ", "simulation.duration: must be > 0", "got -1");
}

static void test_error_zero_dx(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0\n" /* line 6 */
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n";
    LOAD_FAILS(text, TEST_SOURCE ":6: ", "grid.dx: must be > 0");
}

static void test_error_bad_range(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "[grid]\n"
        "x_range = [1.0, 0.0]\n" /* line 4 */
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n";
    LOAD_FAILS(text, TEST_SOURCE ":4: ", "grid.x_range: must be [min, max] with max > min");
}

static void test_error_pixel_scale_out_of_range(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n"
        "[output]\n"
        "pixel_scale = 99\n"; /* line 11 */
    LOAD_FAILS(text, TEST_SOURCE ":11: ", "output.pixel_scale: must be in [1, 16]",
               "got 99");
}

static void test_error_int_out_of_range(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n"
        "[boundary.pml]\n"
        "thickness = 99999999999\n"; /* line 11: does not fit in int */
    LOAD_FAILS(text, TEST_SOURCE ":11: ", "boundary.pml.thickness: is out of range");
}

static void test_error_dt_and_cfl(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "cfl = 0.7\n"
        "dt = 1e-6\n" /* line 4 */
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n";
    LOAD_FAILS(text, TEST_SOURCE ":4: ", "simulation.dt: ", "cfl and dt are mutually exclusive");
}

static void test_error_bad_boundary_name(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n"
        "[boundary]\n"
        "x_min = \"pml\"\n"
        "x_max = \"absorbing\"\n"; /* line 12 */
    LOAD_FAILS(text, TEST_SOURCE ":12: ", "boundary.x_max: unknown boundary type",
               "rigid, pressure_release, pml");
}

static void test_error_bad_pml_params(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n"
        "[boundary.pml]\n" /* line 10 */
        "thickness = 0\n";
    LOAD_FAILS(text, TEST_SOURCE ":10: ", "boundary.pml: ", "thickness must be >= 1");
}

static void test_error_bad_medium(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n" /* line 7 */
        "density = -1.2\n"
        "c = 343.0\n";
    LOAD_FAILS(text, TEST_SOURCE ":7: ", "medium: ", "density must be a finite value > 0");
}

static void test_error_bad_shape_type(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n"
        "[[obstacle]]\n"                                          /* line 10 */
        "shape = { type = \"triangle\", min = [0.0, 0.0] }\n";    /* line 11 */
    LOAD_FAILS(text, TEST_SOURCE ":11: ", "obstacle[0].shape.type: unknown shape type",
               "point, segment, rect, circle, polygon");
}

static void test_error_area_source_rejected(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n"
        "[[source]]\n"                                                      /* line 10 */
        "shape = { type = \"circle\", center = [0.5, 0.5], radius = 0.1 }\n" /* line 11 */
        "signal = { type = \"harmonic\", frequency = 100.0, amplitude = 1.0 }\n";
    LOAD_FAILS(text, TEST_SOURCE ":11: ", "source[0].shape: ", "not supported in this version");
}

static void test_error_point_obstacle_rejected(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n"
        "[[obstacle]]\n"                                          /* line 10 */
        "shape = { type = \"point\", position = [0.5, 0.5] }\n";  /* line 11 */
    LOAD_FAILS(text, TEST_SOURCE ":11: ", "obstacle[0].shape: ",
               "thin obstacles (point) are not supported");
}

static void test_error_polygon_too_few_vertices(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n"
        "[[obstacle]]\n"                                              /* line 10 */
        "shape = { type = \"polygon\", vertices = [[0.0, 0.0], [1.0, 1.0]] }\n"; /* line 11 */
    LOAD_FAILS(text, TEST_SOURCE ":11: ", "obstacle[0].shape.vertices: ",
               "at least 3 vertices", "got 2");
}

static void test_error_degenerate_obstacle(void)
{
    /* geometry.c rejects an empty rect; config must not swallow that. */
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n"
        "[[obstacle]]\n"                                                      /* line 10 */
        "shape = { type = \"rect\", min = [0.5, 0.5], max = [0.5, 0.9] }\n";  /* line 11 */
    LOAD_FAILS(text, TEST_SOURCE ":11: ", "obstacle[0].shape: ", "rect requires max > min");
}

static void test_error_unknown_signal_type(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n"
        "[[source]]\n"                                          /* line 10 */
        "shape = { type = \"point\", position = [0.5, 0.5] }\n"
        "signal = { type = \"gaussian\", frequency = 100.0 }\n"; /* line 12 */
    LOAD_FAILS(text, TEST_SOURCE ":12: ", "source[0].signal.type: unknown signal type",
               "harmonic");
}

static void test_error_negative_frequency(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n"
        "[[source]]\n"
        "shape = { type = \"point\", position = [0.5, 0.5] }\n"
        "signal = { type = \"harmonic\", frequency = -5.0, amplitude = 1.0 }\n"; /* 12 */
    LOAD_FAILS(text, TEST_SOURCE ":12: ", "source[0].signal.frequency: must be > 0",
               "got -5");
}

static void test_error_bad_ramp_cycles(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n"
        "[[source]]\n" /* line 10 */
        "shape = { type = \"point\", position = [0.5, 0.5] }\n"
        "signal = { type = \"harmonic\", frequency = 100.0, amplitude = 1.0, "\
        "ramp_cycles = -1.0 }\n"; /* line 12 */
    LOAD_FAILS(text, TEST_SOURCE ":12: ", "source[0].signal: ", "ramp_cycles");
}

static void test_error_bad_color_limit(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n"
        "[output]\n"                 /* line 10 */
        "color_limit = \"auto2\"\n"; /* line 11 */
    LOAD_FAILS(text, TEST_SOURCE ":11: ", "output.color_limit: must be \"auto\"");
}

static void test_error_empty_directory(void)
{
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n"
        "[output]\n"           /* line 10 */
        "directory = \"\"\n"; /* line 11 */
    LOAD_FAILS(text, TEST_SOURCE ":11: ", "output.directory: must not be empty");
}

static void test_error_cross_section(void)
{
    /* A key that belongs to another table is still an unknown key here. */
    static const char *text =
        "[simulation]\n"
        "duration = 0.1\n"
        "[grid]\n"
        "x_range = [0.0, 1.0]\n"
        "y_range = [0.0, 1.0]\n"
        "dx = 0.01\n"
        "[medium]\n"
        "density = 1.2\n"
        "c = 343.0\n"
        "[boundary]\n"
        "thickness = 20\n"; /* line 11: pml key outside [boundary.pml] */
    LOAD_FAILS(text, TEST_SOURCE ":11: ", "boundary.thickness: unknown key");
}

static void test_error_syntax(void)
{
    static const char *text =
        "[simulation\n" /* line 1: unclosed table header */
        "duration = 0.1\n";
    LOAD_FAILS(text, TEST_SOURCE ": ", "missing right-bracket");
}

static void test_error_nonexistent_file(void)
{
    SimConfig cfg;
    FdtdError err = {{0}};
    memset(&cfg, 0, sizeof cfg);
    memset(&err, 0, sizeof err);
    const char *path = FDTD2D_SOURCE_DIR "/examples/does_not_exist.toml";
    int rc = config_load_file(path, &cfg, &err);
    CHECK(rc != 0);
    CHECK_CONTAINS(err.msg, path);
    CHECK_CONTAINS(err.msg, "cannot open");
    CHECK(cfg.source_name == NULL);
    CHECK(cfg.output.directory == NULL);
    config_free(&cfg);
    config_free(&cfg);
}

/* ---------------------------------------------------------------- */
/* files                                                           */
/* ---------------------------------------------------------------- */

static void test_example_point_source(void)
{
    const char *path = FDTD2D_SOURCE_DIR "/examples/point_source_free_field.toml";
    SimConfig cfg;
    FdtdError err = {{0}};
    memset(&cfg, 0, sizeof cfg);
    int rc = config_load_file(path, &cfg, &err);
    CHECK_MSG(rc == 0, "cannot load %s: %s", path, err.msg);
    if (rc != 0) {
        config_free(&cfg);
        return;
    }

    CHECK_NEAR(cfg.duration, 0.004, 0.0);
    CHECK(!cfg.dt_set);
    CHECK_NEAR(cfg.cfl, 0.7, 0.0);
    CHECK_NEAR(cfg.x_range[0], 0.0, 0.0);
    CHECK_NEAR(cfg.x_range[1], 2.0, 0.0);
    CHECK_NEAR(cfg.y_range[1], 1.5, 0.0);
    CHECK_NEAR(cfg.dx, 0.005, 0.0);
    CHECK_NEAR(cfg.dy, cfg.dx, 0.0);
    CHECK_NEAR(cfg.medium.density, 1.21, 0.0);
    CHECK_NEAR(cfg.medium.c, 343.0, 0.0);
    for (int side = 0; side < SIDE_COUNT; side++) {
        CHECK(cfg.boundary[side] == BC_PML);
    }
    CHECK(cfg.pml.thickness == 20);
    CHECK_NEAR(cfg.pml.reflection, 1e-6, 0.0);
    CHECK_NEAR(cfg.pml.order, 2.0, 0.0);
    CHECK(cfg.n_obstacles == 0);
    CHECK(cfg.n_sources == 1);
    if (cfg.n_sources == 1) {
        CHECK(cfg.sources[0].shape.type == SHAPE_POINT);
        CHECK_NEAR(cfg.sources[0].shape.u.point.position.x, 0.6, 0.0);
        CHECK_NEAR(cfg.sources[0].shape.u.point.position.y, 0.75, 0.0);
        CHECK_NEAR(cfg.sources[0].signal.u.harmonic.frequency, 2000.0, 0.0);
        CHECK_NEAR(cfg.sources[0].signal.u.harmonic.amplitude, 2e-3, 0.0);
        CHECK_NEAR(cfg.sources[0].signal.u.harmonic.phase, 0.0, 0.0);
        CHECK_NEAR(cfg.sources[0].signal.u.harmonic.ramp_cycles, 3.0, 0.0);
    }
    CHECK(cfg.output.directory != NULL);
    if (cfg.output.directory) {
        CHECK(strcmp(cfg.output.directory, "out/point_source_free_field") == 0);
    }
    CHECK(cfg.output.every == 10);
    CHECK(!cfg.output.color_auto);
    CHECK_NEAR(cfg.output.color_limit, 1.0, 0.0);
    CHECK(!cfg.output.show_pml);
    CHECK(cfg.output.pixel_scale == 2);
    CHECK(cfg.source_name != NULL);
    if (cfg.source_name) {
        CHECK(strcmp(cfg.source_name, path) == 0);
    }

    config_free(&cfg);
    config_free(&cfg);
}

static void test_example_line_source(void)
{
    const char *path = FDTD2D_SOURCE_DIR "/examples/line_source_obstacles.toml";
    SimConfig cfg;
    FdtdError err = {{0}};
    memset(&cfg, 0, sizeof cfg);
    int rc = config_load_file(path, &cfg, &err);
    CHECK_MSG(rc == 0, "cannot load %s: %s", path, err.msg);
    if (rc != 0) {
        config_free(&cfg);
        return;
    }

    CHECK_NEAR(cfg.duration, 0.008, 0.0);
    CHECK_NEAR(cfg.cfl, 0.7, 0.0);
    CHECK_NEAR(cfg.x_range[1], 3.0, 0.0);
    CHECK_NEAR(cfg.y_range[1], 2.0, 0.0);
    CHECK_NEAR(cfg.dx, 0.01, 0.0);
    CHECK_NEAR(cfg.dy, 0.01, 0.0);
    CHECK(cfg.boundary[SIDE_X_MIN] == BC_PML);
    CHECK(cfg.boundary[SIDE_X_MAX] == BC_PML);
    CHECK(cfg.boundary[SIDE_Y_MIN] == BC_RIGID);
    CHECK(cfg.boundary[SIDE_Y_MAX] == BC_PML);
    CHECK(cfg.pml.thickness == 20);
    CHECK_NEAR(cfg.pml.reflection, PML_DEFAULT_REFLECTION, 0.0); /* not given */
    CHECK_NEAR(cfg.pml.order, PML_DEFAULT_ORDER, 0.0);           /* not given */

    CHECK(cfg.n_obstacles == 2);
    if (cfg.n_obstacles == 2) {
        CHECK(cfg.obstacles[0].shape.type == SHAPE_CIRCLE);
        CHECK_NEAR(cfg.obstacles[0].shape.u.circle.center.x, 1.8, 0.0);
        CHECK_NEAR(cfg.obstacles[0].shape.u.circle.center.y, 1.1, 0.0);
        CHECK_NEAR(cfg.obstacles[0].shape.u.circle.radius, 0.25, 0.0);

        CHECK(cfg.obstacles[1].shape.type == SHAPE_POLYGON);
        CHECK(cfg.obstacles[1].shape.u.polygon.n == 3);
        if (cfg.obstacles[1].shape.u.polygon.n == 3) {
            CHECK_NEAR(cfg.obstacles[1].shape.u.polygon.vertices[1].x, 2.7, 0.0);
            CHECK_NEAR(cfg.obstacles[1].shape.u.polygon.vertices[2].y, 0.5, 0.0);
        }
    }

    CHECK(cfg.n_sources == 1);
    if (cfg.n_sources == 1) {
        CHECK(cfg.sources[0].shape.type == SHAPE_SEGMENT);
        CHECK_NEAR(cfg.sources[0].shape.u.segment.start.x, 0.4, 0.0);
        CHECK_NEAR(cfg.sources[0].shape.u.segment.start.y, 0.5, 0.0);
        CHECK_NEAR(cfg.sources[0].shape.u.segment.end.y, 1.5, 0.0);
        CHECK_NEAR(cfg.sources[0].signal.u.harmonic.frequency, 1000.0, 0.0);
        CHECK_NEAR(cfg.sources[0].signal.u.harmonic.amplitude, 5e-3, 0.0);
        CHECK_NEAR(cfg.sources[0].signal.u.harmonic.ramp_cycles, 2.0, 0.0);
    }

    CHECK(cfg.output.directory != NULL);
    if (cfg.output.directory) {
        CHECK(strcmp(cfg.output.directory, "out/line_source_obstacles") == 0);
    }
    CHECK(cfg.output.every == 10);
    CHECK(cfg.output.color_auto); /* color_limit = "auto" */
    CHECK(cfg.output.show_pml);
    CHECK(cfg.output.pixel_scale == 2);

    config_free(&cfg);
}

/* ---------------------------------------------------------------- */
/* lifetime                                                         */
/* ---------------------------------------------------------------- */

static void test_config_free_twice(void)
{
    SimConfig cfg;
    load_ok(k_minimal, &cfg);
    CHECK(cfg.source_name != NULL);
    config_free(&cfg);
    CHECK(cfg.source_name == NULL);
    CHECK(cfg.output.directory == NULL);
    CHECK(cfg.sources == NULL);
    CHECK(cfg.obstacles == NULL);
    config_free(&cfg);
    config_free(&cfg);
}

static void test_config_free_zeroed(void)
{
    SimConfig cfg;
    memset(&cfg, 0, sizeof cfg);
    config_free(&cfg);
    config_free(&cfg);
    CHECK(cfg.n_obstacles == 0);
    CHECK(cfg.n_sources == 0);
}

int main(void)
{
    RUN_TEST(test_full_document);
    RUN_TEST(test_defaults);
    RUN_TEST(test_integers_accepted_as_floats);
    RUN_TEST(test_dotted_shape_tables);
    RUN_TEST(test_bad_document);

    RUN_TEST(test_error_unknown_top_level_key);
    RUN_TEST(test_error_unknown_key_in_grid);
    RUN_TEST(test_error_unknown_key_in_shape);
    RUN_TEST(test_error_unknown_key_in_signal);
    RUN_TEST(test_error_wrong_type);
    RUN_TEST(test_error_integer_field_rejects_float);
    RUN_TEST(test_error_non_finite);
    RUN_TEST(test_error_missing_required_key);
    RUN_TEST(test_error_missing_section);
    RUN_TEST(test_error_missing_shape_key);
    RUN_TEST(test_error_cfl_out_of_range);
    RUN_TEST(test_error_negative_duration);
    RUN_TEST(test_error_zero_dx);
    RUN_TEST(test_error_bad_range);
    RUN_TEST(test_error_pixel_scale_out_of_range);
    RUN_TEST(test_error_int_out_of_range);
    RUN_TEST(test_error_dt_and_cfl);
    RUN_TEST(test_error_bad_boundary_name);
    RUN_TEST(test_error_bad_pml_params);
    RUN_TEST(test_error_bad_medium);
    RUN_TEST(test_error_bad_shape_type);
    RUN_TEST(test_error_area_source_rejected);
    RUN_TEST(test_error_point_obstacle_rejected);
    RUN_TEST(test_error_polygon_too_few_vertices);
    RUN_TEST(test_error_degenerate_obstacle);
    RUN_TEST(test_error_unknown_signal_type);
    RUN_TEST(test_error_negative_frequency);
    RUN_TEST(test_error_bad_ramp_cycles);
    RUN_TEST(test_error_bad_color_limit);
    RUN_TEST(test_error_empty_directory);
    RUN_TEST(test_error_cross_section);
    RUN_TEST(test_error_syntax);
    RUN_TEST(test_error_nonexistent_file);

    RUN_TEST(test_example_point_source);
    RUN_TEST(test_example_line_source);

    RUN_TEST(test_config_free_twice);
    RUN_TEST(test_config_free_zeroed);

    return TEST_REPORT();
}
