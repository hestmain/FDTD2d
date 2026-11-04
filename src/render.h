/*
 * render.h - offline heat-map rendering of a cell-centred scalar field.
 *
 * Grid-agnostic: the caller passes a row-major (x fastest) array, an
 * optional solid mask and a crop window in cell indices. Output is an
 * 8-bit RGB PNG (libpng), y axis pointing up (row j0+h-1 is the top image
 * row), each cell drawn as pixel_scale x pixel_scale pixels.
 *
 * Colour map: diverging blue-white-red, symmetric around 0:
 *   v <= -limit -> dark blue, 0 -> white, v >= +limit -> dark red,
 *   piecewise-linear in between (values outside are clamped).
 * Solid cells are drawn in neutral grey (RENDER_SOLID_RGB).
 */
#ifndef FDTD2D_RENDER_H
#define FDTD2D_RENDER_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#define RENDER_SOLID_R 128
#define RENDER_SOLID_G 128
#define RENDER_SOLID_B 128
#define RENDER_OUTLINE_R 0
#define RENDER_OUTLINE_G 0
#define RENDER_OUTLINE_B 0

typedef struct RenderField {
    const double *values;  /* [nx*ny] */
    const uint8_t *mask;   /* [nx*ny] nonzero = solid, may be NULL */
    int nx, ny;            /* array dimensions */
    int i0, j0, w, h;      /* crop window (cells), must lie inside the array */
} RenderField;

typedef struct RenderOptions {
    int pixel_scale;       /* >= 1 */
    double color_limit;    /* > 0 fixed symmetric limit; <= 0 -> auto */
    bool outline;          /* draw a 1-pixel outline along the rectangle */
    int outline_i0, outline_j0, outline_w, outline_h; /* cells, same frame as values */
    const char *comment;   /* optional PNG tEXt "Comment", may be NULL */
} RenderOptions;

typedef struct RenderStats {
    double vmin, vmax;     /* over non-solid cells in the crop (0,0 if none) */
    double color_limit;    /* limit actually used */
} RenderStats;

/* Auto limit = max |v| over non-solid cells in the crop; if that is 0 (or
 * there are no such cells) the limit is 1. The outline is drawn on the
 * pixels just inside the rectangle's border, clipped to the image.
 * stats may be NULL. Returns 0 on success, -1 with err set. */
int render_field_png(const char *path, const RenderField *field,
                     const RenderOptions *opt, RenderStats *stats, FdtdError *err);

/* Colour of normalised value t (t in [-1, 1], clamped) -> rgb[3]. */
void render_colormap(double t, uint8_t rgb[3]);

#endif /* FDTD2D_RENDER_H */
