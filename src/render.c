#include "render.h"

#include <errno.h>
#include <math.h>
#include <png.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#define PNG_MAX_DIM 65535
#define PNG_ZLIB_LEVEL 3

/* ---------------------------------------------------------------- */
/* colour map                                                       */
/* ---------------------------------------------------------------- */

/* Diverging blue-white-red control points at t = -1, -0.75, ..., +1. */
#define COLOR_SEGMENTS 8
static const uint8_t k_colormap[COLOR_SEGMENTS + 1][3] = {
    {5, 48, 97},
    {33, 102, 172},
    {67, 147, 195},
    {146, 197, 222},
    {247, 247, 247},
    {244, 165, 130},
    {214, 96, 77},
    {178, 24, 43},
    {103, 0, 31},
};

void render_colormap(double t, uint8_t rgb[3])
{
    if (isnan(t)) {
        t = 0.0;
    } else if (t < -1.0) {
        t = -1.0;
    } else if (t > 1.0) {
        t = 1.0;
    }
    double s = (t + 1.0) * (COLOR_SEGMENTS / 2.0);
    int k = (int)s;
    if (k > COLOR_SEGMENTS - 1) {
        k = COLOR_SEGMENTS - 1;
    }
    double f = s - (double)k;
    for (int c = 0; c < 3; c++) {
        double a = k_colormap[k][c];
        double b = k_colormap[k + 1][c];
        /* a + f*(b-a) lies in [0, 255]; +0.5 then truncation rounds to nearest. */
        rgb[c] = (uint8_t)(a + f * (b - a) + 0.5);
    }
}

/* ---------------------------------------------------------------- */
/* rendering job                                                    */
/* ---------------------------------------------------------------- */

typedef struct RenderJob {
    const RenderField *field;
    int scale;
    double limit;
    long long width, height;     /* image size in pixels */
    uint8_t zero_rgb[3];         /* colour of t = 0, used for non-finite values */
    bool has_outline;
    long long ox0, ox1, oy0, oy1; /* outline rectangle in image pixels, inclusive */
    uint8_t *pixels;             /* base row, width * 3 bytes */
    uint8_t *outline_row;        /* scratch row for rows touched by the outline */
} RenderJob;

/* Error state shared with the libpng callbacks. Heap allocated so that it is
 * unaffected by setjmp/longjmp register restoration rules. */
typedef struct PngContext {
    char msg[256];
} PngContext;

static void png_error_cb(png_structp png, png_const_charp message)
{
    PngContext *ctx = (PngContext *)png_get_error_ptr(png);
    snprintf(ctx->msg, sizeof ctx->msg, "%s", message ? message : "unknown libpng error");
    png_longjmp(png, 1);
}

static void png_warning_cb(png_structp png, png_const_charp message)
{
    (void)png;
    (void)message;
}

/* Colours of cell row j (crop window columns), expanded to scale x pixels. */
static void fill_pixel_row(const RenderJob *job, int j, uint8_t *dst)
{
    const RenderField *f = job->field;
    size_t offset = (size_t)j * (size_t)f->nx + (size_t)f->i0;
    const double *values = f->values + offset;
    const uint8_t *mask = f->mask ? f->mask + offset : NULL;
    const int scale = job->scale;
    uint8_t rgb[3];

    for (int c = 0; c < f->w; c++) {
        if (mask && mask[c]) {
            rgb[0] = RENDER_SOLID_R;
            rgb[1] = RENDER_SOLID_G;
            rgb[2] = RENDER_SOLID_B;
        } else if (isfinite(values[c])) {
            render_colormap(values[c] / job->limit, rgb);
        } else {
            memcpy(rgb, job->zero_rgb, 3);
        }
        for (int s = 0; s < scale; s++) {
            dst[0] = rgb[0];
            dst[1] = rgb[1];
            dst[2] = rgb[2];
            dst += 3;
        }
    }
}

static void set_black_pixel(const RenderJob *job, uint8_t *row, long long x)
{
    if (x >= 0 && x < job->width) {
        row[3 * x] = RENDER_OUTLINE_R;
        row[3 * x + 1] = RENDER_OUTLINE_G;
        row[3 * x + 2] = RENDER_OUTLINE_B;
    }
}

/* Returns the row to write for image row r: the base row, or a copy of it
 * with the outline drawn when the row intersects the outline border. */
static const uint8_t *apply_outline(const RenderJob *job, long long r)
{
    if (!job->has_outline || r < job->oy0 || r > job->oy1) {
        return job->pixels;
    }
    uint8_t *row = job->outline_row;
    memcpy(row, job->pixels, (size_t)job->width * 3);
    if (r == job->oy0 || r == job->oy1) {
        long long x0 = job->ox0 < 0 ? 0 : job->ox0;
        long long x1 = job->ox1 >= job->width ? job->width - 1 : job->ox1;
        for (long long x = x0; x <= x1; x++) {
            set_black_pixel(job, row, x);
        }
    } else {
        set_black_pixel(job, row, job->ox0);
        set_black_pixel(job, row, job->ox1);
    }
    return row;
}

/* Writes the whole PNG through an already created libpng write struct.
 * Contains the setjmp; every resource is owned by the caller, so the error
 * path just returns -1 (the message is in the PngContext). */
static int write_png(png_structp png, png_infop info, FILE *fp, const RenderJob *job,
                     const char *comment)
{
    if (setjmp(png_jmpbuf(png))) {
        return -1;
    }
    png_init_io(png, fp);
    png_set_compression_level(png, PNG_ZLIB_LEVEL);
    png_set_IHDR(png, info, (png_uint_32)job->width, (png_uint_32)job->height, 8,
                 PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT,
                 PNG_FILTER_TYPE_DEFAULT);

    char software[] = "fdtd2d " FDTD2D_VERSION;
    png_text text[2];
    int n_text = 0;
    memset(text, 0, sizeof text);
    if (comment) {
        text[n_text].compression = PNG_TEXT_COMPRESSION_NONE;
        text[n_text].key = (png_charp)"Comment";
        text[n_text].text = (png_charp)comment; /* libpng does not modify it */
        text[n_text].text_length = strlen(comment);
        n_text++;
    }
    text[n_text].compression = PNG_TEXT_COMPRESSION_NONE;
    text[n_text].key = (png_charp)"Software";
    text[n_text].text = software;
    text[n_text].text_length = strlen(software);
    n_text++;
    png_set_text(png, info, text, n_text);

    png_write_info(png, info);

    const RenderField *f = job->field;
    const int scale = job->scale;
    for (long long r = 0; r < job->height; r++) {
        if (r % scale == 0) {
            int j = f->j0 + f->h - 1 - (int)(r / scale);
            fill_pixel_row(job, j, job->pixels);
        }
        png_write_row(png, (png_const_bytep)apply_outline(job, r));
    }
    png_write_end(png, NULL);
    return 0;
}

/* ---------------------------------------------------------------- */
/* public entry point                                               */
/* ---------------------------------------------------------------- */

/* Preconditions (see render.h) are not re-checked: path, field, field->values
 * and opt are non-NULL, the crop window lies inside the array, pixel_scale >= 1,
 * color_limit is finite or <= 0. The PNG size limit is the one input check
 * kept here, because only this file knows it. */
int render_field_png(const char *path, const RenderField *field,
                     const RenderOptions *opt, RenderStats *stats, FdtdError *err)
{
    const long long img_w = (long long)field->w * opt->pixel_scale;
    const long long img_h = (long long)field->h * opt->pixel_scale;
    if (img_w > PNG_MAX_DIM || img_h > PNG_MAX_DIM) {
        fdtd_error_set(err, "render: image size %lld x %lld pixels exceeds the PNG limit of %d "
                       "(reduce pixel_scale or the crop window)", img_w, img_h, PNG_MAX_DIM);
        return -1;
    }

    /* Statistics over non-solid, finite cells of the crop window. */
    double vmin = 0.0, vmax = 0.0, abs_max = 0.0;
    bool have_value = false;
    for (int jj = 0; jj < field->h; jj++) {
        size_t offset = (size_t)(field->j0 + jj) * (size_t)field->nx + (size_t)field->i0;
        const double *values = field->values + offset;
        const uint8_t *mask = field->mask ? field->mask + offset : NULL;
        for (int ii = 0; ii < field->w; ii++) {
            double v = values[ii];
            if ((mask && mask[ii]) || !isfinite(v)) {
                continue;
            }
            if (!have_value) {
                vmin = vmax = v;
                have_value = true;
            } else if (v < vmin) {
                vmin = v;
            } else if (v > vmax) {
                vmax = v;
            }
        }
    }
    if (have_value) {
        abs_max = fmax(fabs(vmin), fabs(vmax));
    }

    RenderJob job;
    memset(&job, 0, sizeof job);
    job.field = field;
    job.scale = opt->pixel_scale;
    job.width = img_w;
    job.height = img_h;
    if (opt->color_limit > 0.0) {
        job.limit = opt->color_limit;
    } else {
        job.limit = abs_max > 0.0 ? abs_max : 1.0;
    }
    render_colormap(0.0, job.zero_rgb);

    if (opt->outline && opt->outline_w > 0 && opt->outline_h > 0 &&
        (long long)opt->outline_i0 < (long long)field->i0 + field->w &&
        (long long)opt->outline_i0 + opt->outline_w > field->i0 &&
        (long long)opt->outline_j0 < (long long)field->j0 + field->h &&
        (long long)opt->outline_j0 + opt->outline_h > field->j0) {
        job.has_outline = true;
        job.ox0 = ((long long)opt->outline_i0 - field->i0) * job.scale;
        job.ox1 = ((long long)opt->outline_i0 + opt->outline_w - field->i0) * job.scale - 1;
        /* Image rows run top-down, cell rows bottom-up. */
        job.oy0 = ((long long)field->h + field->j0 - opt->outline_j0 - opt->outline_h) * job.scale;
        job.oy1 = ((long long)field->h + field->j0 - opt->outline_j0) * job.scale - 1;
    }

    int rc = -1;
    PngContext *ctx = NULL;
    uint8_t *rows = NULL;
    FILE *fp = NULL;
    png_structp png = NULL;
    png_infop info = NULL;
    bool remove_on_failure = false;
    struct stat st_open; /* identity of the file we opened (and truncated) */
    memset(&st_open, 0, sizeof st_open);

    ctx = fdtd_calloc(1, sizeof *ctx, err);
    if (!ctx) {
        goto done;
    }
    size_t row_bytes = (size_t)job.width * 3;
    rows = fdtd_calloc(2, row_bytes, err);
    if (!rows) {
        goto done;
    }
    job.pixels = rows;
    job.outline_row = rows + row_bytes;

    fp = fopen(path, "wb");
    if (!fp) {
        fdtd_error_set(err, "cannot write '%s': %s", path, strerror(errno));
        goto done;
    }
    /* Only ever delete a regular file we truncated, never a device node. */
    remove_on_failure = fstat(fileno(fp), &st_open) == 0 && S_ISREG(st_open.st_mode);

    png = png_create_write_struct(PNG_LIBPNG_VER_STRING, ctx, png_error_cb, png_warning_cb);
    info = png ? png_create_info_struct(png) : NULL;
    if (!png || !info) {
        fdtd_error_set(err, "cannot write '%s': out of memory creating PNG writer", path);
        goto done;
    }

    if (write_png(png, info, fp, &job, opt->comment) != 0) {
        fdtd_error_set(err, "cannot write PNG '%s': %s", path, ctx->msg);
        goto done;
    }
    rc = 0;

done:
    png_destroy_write_struct(&png, &info); /* accepts NULL pointers */
    if (fp && fclose(fp) != 0 && rc == 0) {
        fdtd_error_set(err, "cannot write PNG '%s': %s", path, strerror(errno));
        rc = -1;
    }
    if (rc != 0 && remove_on_failure) {
        /* Remove the partial file only if `path` itself is still that file:
         * not a symlink to it (that would delete the link and keep the
         * truncated target) and not something that replaced it meanwhile. */
        struct stat st_now;
        if (lstat(path, &st_now) == 0 && S_ISREG(st_now.st_mode) &&
            st_now.st_dev == st_open.st_dev && st_now.st_ino == st_open.st_ino) {
            remove(path);
        }
    }
    free(rows);
    free(ctx);
    if (rc == 0 && stats) {
        stats->vmin = vmin;
        stats->vmax = vmax;
        stats->color_limit = job.limit;
    }
    return rc;
}
