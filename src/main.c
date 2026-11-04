/*
 * main.c - fdtd2d command line tool.
 *
 *   fdtd2d [--check] [-o DIR] [-q] CONFIG.toml
 *
 * Pipeline: config (parser layer 1) -> model (parser layer 2) -> solver
 * time loop -> heat-map PNG frames + frames.csv index in the output dir.
 */
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>

#include "common.h"
#include "config.h"
#include "model.h"
#include "render.h"
#include "solver.h"

typedef struct CliOptions {
    const char *config_path;
    const char *output_dir; /* overrides [output].directory when set */
    int check_only;
    int quiet;
} CliOptions;

static void print_usage(FILE *out)
{
    fputs("usage: fdtd2d [options] CONFIG.toml\n"
          "\n"
          "Run a 2D acoustic FDTD simulation described by a TOML file and write\n"
          "pressure heat maps (PNG) plus frames.csv to the output directory.\n"
          "\n"
          "options:\n"
          "  --check            parse the config, build the model, print the\n"
          "                     summary and exit without simulating\n"
          "  -o, --output DIR   output directory (overrides [output].directory)\n"
          "  -q, --quiet        no progress messages\n"
          "  -h, --help         show this help\n"
          "  --version          show version\n",
          out);
}

/* Returns 0 to continue, 1 to exit successfully, 2 on usage error. */
static int parse_args(int argc, char **argv, CliOptions *o)
{
    memset(o, 0, sizeof *o);
    for (int k = 1; k < argc; k++) {
        const char *a = argv[k];
        if (strcmp(a, "--check") == 0) {
            o->check_only = 1;
        } else if (strcmp(a, "-q") == 0 || strcmp(a, "--quiet") == 0) {
            o->quiet = 1;
        } else if (strcmp(a, "-o") == 0 || strcmp(a, "--output") == 0) {
            if (k + 1 >= argc) {
                fprintf(stderr, "fdtd2d: %s needs a directory argument\n", a);
                return 2;
            }
            o->output_dir = argv[++k];
        } else if (strcmp(a, "-h") == 0 || strcmp(a, "--help") == 0) {
            print_usage(stdout);
            return 1;
        } else if (strcmp(a, "--version") == 0) {
            printf("fdtd2d %s\n", FDTD2D_VERSION);
            return 1;
        } else if (a[0] == '-' && a[1] != '\0') {
            fprintf(stderr, "fdtd2d: unknown option '%s'\n", a);
            print_usage(stderr);
            return 2;
        } else if (o->config_path) {
            fprintf(stderr, "fdtd2d: only one config file may be given\n");
            return 2;
        } else {
            o->config_path = a;
        }
    }
    if (!o->config_path) {
        print_usage(stderr);
        return 2;
    }
    return 0;
}

/* mkdir -p */
static int make_dirs(const char *path, FdtdError *err)
{
    size_t len = strlen(path);
    if (len == 0) {
        fdtd_error_set(err, "empty output directory");
        return -1;
    }
    char *buf = fdtd_strdup(path, err);
    if (!buf) {
        return -1;
    }
    int rc = 0;
    for (char *p = buf + 1;; p++) {
        if (*p == '/' || *p == '\0') {
            char saved = *p;
            *p = '\0';
            if (mkdir(buf, 0777) != 0 && errno != EEXIST) {
                fdtd_error_set(err, "cannot create directory '%s': %s", buf, strerror(errno));
                rc = -1;
                break;
            }
            *p = saved;
            if (saved == '\0') {
                break;
            }
        }
    }
    if (rc == 0) {
        struct stat st;
        if (stat(path, &st) != 0 || !S_ISDIR(st.st_mode)) {
            fdtd_error_set(err, "'%s' exists but is not a directory", path);
            rc = -1;
        }
    }
    free(buf);
    return rc;
}

static double now_seconds(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + 1e-9 * (double)ts.tv_nsec;
}

static int count_digits(long n)
{
    int d = 1;
    while (n >= 10) {
        n /= 10;
        d++;
    }
    return d;
}

static int write_summary_file(const char *dir, const char *config_path, const Model *m, FdtdError *err)
{
    char path[4096];
    if (snprintf(path, sizeof path, "%s/summary.txt", dir) >= (int)sizeof path) {
        fdtd_error_set(err, "output path too long");
        return -1;
    }
    FILE *f = fopen(path, "w");
    if (!f) {
        fdtd_error_set(err, "cannot write '%s': %s", path, strerror(errno));
        return -1;
    }
    fprintf(f, "fdtd2d %s\nconfig: %s\n\n", FDTD2D_VERSION, config_path);
    model_print_summary(f, m);
    if (fclose(f) != 0) {
        fdtd_error_set(err, "error writing '%s'", path);
        return -1;
    }
    return 0;
}

static int run(const CliOptions *cli, FdtdError *err)
{
    SimConfig cfg;
    Model model;
    Solver solver;
    memset(&solver, 0, sizeof solver);
    double *pressure = NULL;
    FILE *index = NULL;
    int rc = -1;

    if (config_load_file(cli->config_path, &cfg, err) != 0) {
        return -1;
    }
    if (model_build(&cfg, &model, err) != 0) {
        fdtd_error_prefix(err, "%s", cli->config_path);
        goto done;
    }
    model_print_summary(stdout, &model);
    fflush(stdout);
    /* The frame size only depends on the grid and [output]: reject an
     * image the PNG writer cannot produce before anything is written
     * (also in --check mode). */
    {
        const Grid *gg = &model.grid;
        const long long img_w = (long long)(cfg.output.show_pml ? gg->nx : gg->nx_phys) *
                                cfg.output.pixel_scale;
        const long long img_h = (long long)(cfg.output.show_pml ? gg->ny : gg->ny_phys) *
                                cfg.output.pixel_scale;
        if (img_w > 65535 || img_h > 65535) {
            fdtd_error_set(err, "%s: output: frames would be %lld x %lld pixels, above the "
                           "PNG limit of 65535; reduce pixel_scale",
                           cli->config_path, img_w, img_h);
            goto done;
        }
    }
    if (cli->check_only) {
        rc = 0;
        goto done;
    }

    const char *out_dir = cli->output_dir ? cli->output_dir : cfg.output.directory;
    if (make_dirs(out_dir, err) != 0) {
        goto done;
    }
    if (write_summary_file(out_dir, cli->config_path, &model, err) != 0) {
        goto done;
    }
    if (solver_init(&solver, &model, err) != 0) {
        goto done;
    }

    const Grid *g = &model.grid;
    pressure = fdtd_calloc(grid_cell_count(g), sizeof *pressure, err);
    if (!pressure) {
        goto done;
    }

    char path[4096];
    if (snprintf(path, sizeof path, "%s/frames.csv", out_dir) >= (int)sizeof path) {
        fdtd_error_set(err, "output path too long");
        goto done;
    }
    index = fopen(path, "w");
    if (!index) {
        fdtd_error_set(err, "cannot write '%s': %s", path, strerror(errno));
        goto done;
    }
    fputs("step,time_s,file,color_limit_pa,p_min_pa,p_max_pa\n", index);

    RenderField field = {
        .values = pressure,
        .mask = model.solid,
        .nx = g->nx,
        .ny = g->ny,
    };
    RenderOptions ropt = {
        .pixel_scale = cfg.output.pixel_scale,
        .color_limit = cfg.output.color_auto ? 0.0 : cfg.output.color_limit,
    };
    if (cfg.output.show_pml) {
        field.i0 = 0;
        field.j0 = 0;
        field.w = g->nx;
        field.h = g->ny;
        ropt.outline = g->nx != g->nx_phys || g->ny != g->ny_phys;
        ropt.outline_i0 = g->off_x;
        ropt.outline_j0 = g->off_y;
        ropt.outline_w = g->nx_phys;
        ropt.outline_h = g->ny_phys;
    } else {
        field.i0 = g->off_x;
        field.j0 = g->off_y;
        field.w = g->nx_phys;
        field.h = g->ny_phys;
    }
    const long n_steps = model.n_steps;
    const int every = cfg.output.every;
    const int digits = count_digits(n_steps) > 6 ? count_digits(n_steps) : 6;
    const size_t n_cells = grid_cell_count(g);
    long next_progress = n_steps / 10;
    long frames = 0;
    double t_start = now_seconds();

    for (long n = 1; n <= n_steps; n++) {
        solver_step(&solver);

        if (n % every == 0 || n == n_steps) {
            solver_get_pressure(&solver, pressure);
            for (size_t c = 0; c < n_cells; c++) {
                if (!isfinite(pressure[c])) {
                    fdtd_error_set(err, "solution became non-finite at step %ld (t = %.6g s); "
                                   "check dt/cfl and source amplitudes", n, solver.time);
                    goto done;
                }
            }
            char name[64];
            /* The name is at most 25 chars; the length test only lets the compiler
             * see that truncation is handled (-Wformat-truncation). */
            int name_len = snprintf(name, sizeof name, "p_%0*ld.png", digits, n);
            if (name_len >= (int)sizeof name ||
                snprintf(path, sizeof path, "%s/%s", out_dir, name) >= (int)sizeof path) {
                fdtd_error_set(err, "output path too long");
                goto done;
            }
            char comment[160];
            snprintf(comment, sizeof comment, "fdtd2d pressure, step=%ld, time=%.9e s", n, solver.time);
            ropt.comment = comment;
            RenderStats stats;
            if (render_field_png(path, &field, &ropt, &stats, err) != 0) {
                goto done;
            }
            fprintf(index, "%ld,%.9e,%s,%.6e,%.6e,%.6e\n", n, solver.time, name,
                    stats.color_limit, stats.vmin, stats.vmax);
            frames++;
        }

        if (!cli->quiet && next_progress > 0 && n >= next_progress && n < n_steps) {
            fprintf(stderr, "  step %ld / %ld (%.0f%%)\n", n, n_steps, 100.0 * (double)n / (double)n_steps);
            next_progress += n_steps / 10;
        }
    }

    double elapsed = now_seconds() - t_start;
    if (!cli->quiet) {
        double rate = elapsed > 0.0 ? (double)n_cells * (double)n_steps / elapsed / 1e6 : 0.0;
        fprintf(stderr, "done: %ld steps, %ld frames in %s, %.2f s (%.1f Mcell-updates/s)\n",
                n_steps, frames, out_dir, elapsed, rate);
    }
    rc = 0;

done:
    if (index && fclose(index) != 0 && rc == 0) {
        fdtd_error_set(err, "error writing frames.csv");
        rc = -1;
    }
    free(pressure);
    solver_free(&solver);
    model_free(&model);
    config_free(&cfg);
    return rc;
}

int main(int argc, char **argv)
{
    CliOptions cli;
    int arg_rc = parse_args(argc, argv, &cli);
    if (arg_rc == 1) {
        return 0;
    }
    if (arg_rc == 2) {
        return 2;
    }
    FdtdError err = {{0}};
    if (run(&cli, &err) != 0) {
        fprintf(stderr, "error: %s\n", err.msg);
        return 1;
    }
    return 0;
}
