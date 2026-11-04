/*
 * config.h - parser layer 1: TOML file -> validated SimConfig (pure data).
 *
 * SimConfig holds exactly what the user wrote (plus defaults); it knows
 * nothing about the grid. Cross-field checks that need the grid (domain
 * divisibility, CFL limit, source inside domain, ...) belong to
 * model_build(). See the files in examples/ and DEV.md for the format.
 */
#ifndef FDTD2D_CONFIG_H
#define FDTD2D_CONFIG_H

#include <stdbool.h>

#include "boundary.h"
#include "common.h"
#include "geometry.h"
#include "medium.h"
#include "waveform.h"

#define CONFIG_DEFAULT_CFL 0.7
#define CONFIG_DEFAULT_OUTPUT_DIR "out"
#define CONFIG_DEFAULT_OUTPUT_EVERY 10
#define CONFIG_DEFAULT_PIXEL_SCALE 1
#define CONFIG_MAX_PIXEL_SCALE 16
#define CONFIG_DEFAULT_RAMP_CYCLES 3.0

typedef struct SourceConfig {
    Shape shape;   /* v1: SHAPE_POINT or SHAPE_SEGMENT */
    Signal signal; /* v1: SIGNAL_HARMONIC */
} SourceConfig;

typedef struct ObstacleConfig {
    Shape shape;   /* area shape; cells whose centre is inside are rigid */
} ObstacleConfig;

typedef struct OutputConfig {
    char *directory;     /* owned; default "out" */
    int every;           /* write a frame every N steps, >= 1 */
    bool color_auto;     /* true: symmetric limit = max|p| of each frame */
    double color_limit;  /* [Pa] > 0, used when !color_auto */
    bool show_pml;       /* render PML cells too (physical domain outlined) */
    int pixel_scale;     /* pixels per cell, 1..CONFIG_MAX_PIXEL_SCALE */
} OutputConfig;

typedef struct SimConfig {
    /* [simulation] */
    double duration;     /* [s] > 0 */
    double cfl;          /* (0, 1]; used when !dt_set */
    double dt;           /* [s] > 0; valid when dt_set */
    bool dt_set;

    /* [grid] */
    double x_range[2];   /* [m], x_range[1] > x_range[0] */
    double y_range[2];
    double dx, dy;       /* [m] > 0; dy defaults to dx */

    /* [medium] - background medium (v1: the only medium) */
    Medium medium;

    /* [boundary] */
    BoundaryType boundary[SIDE_COUNT]; /* default BC_PML on every side */
    PmlParams pml;                     /* used by sides set to BC_PML */

    /* [[obstacle]] */
    ObstacleConfig *obstacles;
    int n_obstacles;

    /* [[source]] */
    SourceConfig *sources;
    int n_sources;

    /* [output] */
    OutputConfig output;

    char *source_name;   /* owned; file name used in messages */
} SimConfig;

/* Parse and validate a TOML file. path and cfg must not be NULL; err may be
 * NULL. On success returns 0 and cfg owns all memory (release with
 * config_free). On failure returns -1, cfg is left zeroed (nothing to free)
 * and err holds "<file>:<line>: <key path>: ...". */
int config_load_file(const char *path, SimConfig *cfg, FdtdError *err);

/* Same as config_load_file for an in-memory document. text, source_name and
 * cfg must not be NULL. source_name is used in messages (e.g. "<string>"). */
int config_load_string(const char *text, const char *source_name, SimConfig *cfg, FdtdError *err);

/* Release everything owned by cfg and zero it. Safe on a zeroed cfg. */
void config_free(SimConfig *cfg);

#endif /* FDTD2D_CONFIG_H */
