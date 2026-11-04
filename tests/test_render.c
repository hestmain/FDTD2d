/*
 * test_render.c - unit tests for the PNG heat-map renderer.
 *
 * Images are written to FDTD2D_TEST_TMPDIR and read back with libpng
 * (simplified API for pixels, classic API for header fields and tEXt).
 */
#include <dirent.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <png.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "render.h"
#include "test_util.h"

#ifndef FDTD2D_TEST_TMPDIR
#error "FDTD2D_TEST_TMPDIR must be defined by the build system"
#endif

/* ---------------------------------------------------------------- */
/* helpers                                                          */
/* ---------------------------------------------------------------- */

typedef struct Img {
    unsigned width, height;
    uint8_t *rgb; /* width * height * 3 */
} Img;

typedef struct PngInfo {
    unsigned width, height;
    int bit_depth, color_type, interlace;
    bool has_comment;
    char comment[256];
    bool has_software;
    char software[64];
} PngInfo;

static const uint8_t WHITE[3] = {247, 247, 247};
static const uint8_t SOLID[3] = {RENDER_SOLID_R, RENDER_SOLID_G, RENDER_SOLID_B};
static const uint8_t BLACK[3] = {RENDER_OUTLINE_R, RENDER_OUTLINE_G, RENDER_OUTLINE_B};

static void make_tmpdir(void)
{
    if (mkdir(FDTD2D_TEST_TMPDIR, 0777) != 0 && errno != EEXIST) {
        fprintf(stderr, "cannot create %s: %s\n", FDTD2D_TEST_TMPDIR, strerror(errno));
    }
}

static void tmp_path(char *buf, size_t size, const char *name)
{
    snprintf(buf, size, "%s/%s", FDTD2D_TEST_TMPDIR, name);
}

static bool file_exists(const char *path)
{
    struct stat st;
    return stat(path, &st) == 0;
}

static int count_open_fds(void)
{
    DIR *d = opendir("/proc/self/fd");
    if (!d) {
        return -1;
    }
    int n = 0;
    while (readdir(d)) {
        n++;
    }
    closedir(d);
    return n;
}

static int load_png(const char *path, Img *img)
{
    memset(img, 0, sizeof *img);
    png_image pi;
    memset(&pi, 0, sizeof pi);
    pi.version = PNG_IMAGE_VERSION;
    if (!png_image_begin_read_from_file(&pi, path)) {
        fprintf(stderr, "read %s: %s\n", path, pi.message);
        return -1;
    }
    pi.format = PNG_FORMAT_RGB;
    img->width = pi.width;
    img->height = pi.height;
    img->rgb = malloc(PNG_IMAGE_SIZE(pi));
    if (!img->rgb || !png_image_finish_read(&pi, NULL, img->rgb, 0, NULL)) {
        fprintf(stderr, "read %s: %s\n", path, pi.message);
        free(img->rgb);
        img->rgb = NULL;
        png_image_free(&pi);
        return -1;
    }
    png_image_free(&pi);
    return 0;
}

static void free_img(Img *img)
{
    free(img->rgb);
    img->rgb = NULL;
}

static const uint8_t *px(const Img *img, int x, int y)
{
    return img->rgb + 3 * ((size_t)y * img->width + (size_t)x);
}

#define EXPECT_PX(img, x, y, expect)                                           \
    do {                                                                       \
        const uint8_t *got_ = px((img), (x), (y));                             \
        const uint8_t *want_ = (expect);                                       \
        CHECK_MSG(got_[0] == want_[0] && got_[1] == want_[1] &&                \
                      got_[2] == want_[2],                                     \
                  "pixel (%d,%d) = %d,%d,%d, expected %d,%d,%d", (int)(x),     \
                  (int)(y), got_[0], got_[1], got_[2], want_[0], want_[1],     \
                  want_[2]);                                                   \
    } while (0)

/* Colour the renderer must produce for normalised value t. */
static void color_of(double t, uint8_t rgb[3])
{
    render_colormap(t, rgb);
}

#define EXPECT_PX_T(img, x, y, t)                                              \
    do {                                                                       \
        uint8_t want_t_[3];                                                    \
        color_of((t), want_t_);                                                \
        EXPECT_PX((img), (x), (y), want_t_);                                   \
    } while (0)

static int count_color(const Img *img, const uint8_t rgb[3])
{
    int n = 0;
    for (unsigned y = 0; y < img->height; y++) {
        for (unsigned x = 0; x < img->width; x++) {
            if (memcmp(px(img, (int)x, (int)y), rgb, 3) == 0) {
                n++;
            }
        }
    }
    return n;
}

static RenderField make_field(const double *values, const uint8_t *mask, int nx, int ny)
{
    RenderField f = {.values = values, .mask = mask, .nx = nx, .ny = ny,
                     .i0 = 0, .j0 = 0, .w = nx, .h = ny};
    return f;
}

/* Render and read back; requires success. */
static int render_load(const char *name, const RenderField *f, const RenderOptions *o,
                       RenderStats *stats, Img *img)
{
    char path[512];
    FdtdError err = {{0}};
    tmp_path(path, sizeof path, name);
    int rc = render_field_png(path, f, o, stats, &err);
    CHECK_MSG(rc == 0, "render failed: %s", err.msg);
    if (rc != 0) {
        memset(img, 0, sizeof *img);
        return -1;
    }
    int lrc = load_png(path, img);
    CHECK(lrc == 0);
    return lrc;
}

/* Classic libpng read of IHDR and tEXt chunks. setjmp lives here; the
 * caller owns and frees png/info/fp. */
static int read_info_inner(png_structp png, png_infop info, FILE *fp, PngInfo *out)
{
    if (setjmp(png_jmpbuf(png))) {
        return -1;
    }
    png_init_io(png, fp);
    png_read_info(png, info);
    out->width = png_get_image_width(png, info);
    out->height = png_get_image_height(png, info);
    out->bit_depth = png_get_bit_depth(png, info);
    out->color_type = png_get_color_type(png, info);
    out->interlace = png_get_interlace_type(png, info);
    png_textp text = NULL;
    int n = 0;
    png_get_text(png, info, &text, &n);
    for (int i = 0; i < n; i++) {
        if (strcmp(text[i].key, "Comment") == 0) {
            out->has_comment = true;
            snprintf(out->comment, sizeof out->comment, "%s", text[i].text);
        } else if (strcmp(text[i].key, "Software") == 0) {
            out->has_software = true;
            snprintf(out->software, sizeof out->software, "%s", text[i].text);
        }
    }
    return 0;
}

static int read_info(const char *path, PngInfo *out)
{
    memset(out, 0, sizeof *out);
    FILE *fp = fopen(path, "rb");
    if (!fp) {
        return -1;
    }
    png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    png_infop info = png ? png_create_info_struct(png) : NULL;
    int rc = -1;
    if (png && info) {
        rc = read_info_inner(png, info, fp, out);
    }
    png_destroy_read_struct(&png, &info, NULL);
    fclose(fp);
    return rc;
}

/* ---------------------------------------------------------------- */
/* colour map                                                       */
/* ---------------------------------------------------------------- */

static void test_colormap_control_points(void)
{
    static const struct {
        double t;
        uint8_t rgb[3];
    } points[] = {
        {-1.0, {5, 48, 97}},    {-0.75, {33, 102, 172}}, {-0.5, {67, 147, 195}},
        {-0.25, {146, 197, 222}}, {0.0, {247, 247, 247}}, {0.25, {244, 165, 130}},
        {0.5, {214, 96, 77}},   {0.75, {178, 24, 43}},   {1.0, {103, 0, 31}},
    };
    for (size_t i = 0; i < sizeof points / sizeof points[0]; i++) {
        uint8_t rgb[3] = {1, 2, 3};
        render_colormap(points[i].t, rgb);
        CHECK_MSG(memcmp(rgb, points[i].rgb, 3) == 0, "t = %g gave %d,%d,%d", points[i].t,
                  rgb[0], rgb[1], rgb[2]);
    }
}

static void test_colormap_midpoints_and_white(void)
{
    uint8_t rgb[3];
    render_colormap(0.0, rgb);
    CHECK(rgb[0] == 247 && rgb[1] == 247 && rgb[2] == 247);

    /* Halfway between control points, rounded to nearest. */
    static const struct {
        double t;
        uint8_t rgb[3];
    } mids[] = {
        {0.125, {246, 206, 189}},
        {-0.125, {197, 222, 235}},
        {0.0625, {246, 227, 218}},
        {0.875, {141, 12, 37}},
        {-0.875, {19, 75, 135}},
    };
    for (size_t i = 0; i < sizeof mids / sizeof mids[0]; i++) {
        render_colormap(mids[i].t, rgb);
        CHECK_MSG(memcmp(rgb, mids[i].rgb, 3) == 0, "t = %g gave %d,%d,%d", mids[i].t,
                  rgb[0], rgb[1], rgb[2]);
    }

    /* Just below zero is bluish, just above is reddish. */
    uint8_t lo[3], hi[3];
    render_colormap(-0.01, lo);
    render_colormap(0.01, hi);
    CHECK(lo[2] > lo[0]);
    CHECK(hi[0] > hi[2]);
}

static void test_colormap_clamp_and_nan(void)
{
    uint8_t lo[3], hi[3], rgb[3];
    render_colormap(-1.0, lo);
    render_colormap(1.0, hi);

    render_colormap(-2.5, rgb);
    CHECK(memcmp(rgb, lo, 3) == 0);
    render_colormap(-INFINITY, rgb);
    CHECK(memcmp(rgb, lo, 3) == 0);
    render_colormap(1.0000001, rgb);
    CHECK(memcmp(rgb, hi, 3) == 0);
    render_colormap(1e300, rgb);
    CHECK(memcmp(rgb, hi, 3) == 0);
    render_colormap(INFINITY, rgb);
    CHECK(memcmp(rgb, hi, 3) == 0);

    render_colormap(NAN, rgb);
    CHECK(memcmp(rgb, WHITE, 3) == 0);
}

/* ---------------------------------------------------------------- */
/* geometry of the image                                            */
/* ---------------------------------------------------------------- */

static void test_dimensions_and_format(void)
{
    double v[6] = {0};
    RenderField f = make_field(v, NULL, 3, 2);
    RenderOptions o = {.pixel_scale = 4};
    Img img;
    if (render_load("dims.png", &f, &o, NULL, &img) == 0) {
        CHECK(img.width == 12);
        CHECK(img.height == 8);
        free_img(&img);
    }

    char path[512];
    tmp_path(path, sizeof path, "dims.png");
    PngInfo info;
    CHECK(read_info(path, &info) == 0);
    CHECK(info.width == 12 && info.height == 8);
    CHECK(info.bit_depth == 8);
    CHECK(info.color_type == PNG_COLOR_TYPE_RGB);
    CHECK(info.interlace == PNG_INTERLACE_NONE);

    /* 1x1 field, scale 1. */
    double one = 1.0;
    RenderField f1 = make_field(&one, NULL, 1, 1);
    RenderOptions o1 = {.pixel_scale = 1};
    if (render_load("one.png", &f1, &o1, NULL, &img) == 0) {
        CHECK(img.width == 1 && img.height == 1);
        EXPECT_PX_T(&img, 0, 0, 1.0);
        free_img(&img);
    }
}

static void test_y_flip(void)
{
    /* 3 x 3, only cell (i = 1, j = ny - 1) is positive: top image row, middle column. */
    double v[9] = {0};
    v[2 * 3 + 1] = 1.0;
    RenderField f = make_field(v, NULL, 3, 3);
    RenderOptions o = {.pixel_scale = 1};
    RenderStats st;
    Img img;
    if (render_load("flip.png", &f, &o, &st, &img) == 0) {
        EXPECT_PX_T(&img, 1, 0, 1.0);
        CHECK(count_color(&img, WHITE) == 8);
        CHECK_NEAR(st.color_limit, 1.0, 0.0);
        free_img(&img);
    }

    /* A negative value in row j = 0 must land in the bottom image row. */
    double w[9] = {0};
    w[0] = -1.0;
    RenderField fw = make_field(w, NULL, 3, 3);
    if (render_load("flip2.png", &fw, &o, NULL, &img) == 0) {
        EXPECT_PX_T(&img, 0, 2, -1.0);
        CHECK(count_color(&img, WHITE) == 8);
        free_img(&img);
    }

    /* With scale 2 every cell becomes a 2 x 2 block. */
    RenderOptions o2 = {.pixel_scale = 2};
    if (render_load("flip3.png", &f, &o2, NULL, &img) == 0) {
        CHECK(img.width == 6 && img.height == 6);
        for (int y = 0; y < 6; y++) {
            for (int x = 0; x < 6; x++) {
                bool red = y < 2 && x >= 2 && x < 4;
                if (red) {
                    EXPECT_PX_T(&img, x, y, 1.0);
                } else {
                    EXPECT_PX(&img, x, y, WHITE);
                }
            }
        }
        free_img(&img);
    }
}

static void test_crop_window(void)
{
    /* 5 x 4 array with value 10*j + i; crop i = 1..3, j = 1..2. */
    double v[20];
    for (int j = 0; j < 4; j++) {
        for (int i = 0; i < 5; i++) {
            v[j * 5 + i] = 10.0 * j + i;
        }
    }
    RenderField f = make_field(v, NULL, 5, 4);
    f.i0 = 1;
    f.j0 = 1;
    f.w = 3;
    f.h = 2;
    RenderOptions o = {.pixel_scale = 1, .color_limit = 100.0};
    RenderStats st;
    Img img;
    if (render_load("crop.png", &f, &o, &st, &img) == 0) {
        CHECK(img.width == 3 && img.height == 2);
        for (int r = 0; r < 2; r++) {
            int j = 1 + 1 - r; /* top row is j = 2 */
            for (int c = 0; c < 3; c++) {
                EXPECT_PX_T(&img, c, r, (10.0 * j + (1 + c)) / 100.0);
            }
        }
        /* Stats cover only the window: min 11, max 23. */
        CHECK_NEAR(st.vmin, 11.0, 0.0);
        CHECK_NEAR(st.vmax, 23.0, 0.0);
        CHECK_NEAR(st.color_limit, 100.0, 0.0);
        free_img(&img);
    }

    /* Window touching the last row and column. */
    f.i0 = 4;
    f.j0 = 3;
    f.w = 1;
    f.h = 1;
    if (render_load("crop2.png", &f, &o, &st, &img) == 0) {
        CHECK(img.width == 1 && img.height == 1);
        EXPECT_PX_T(&img, 0, 0, 0.34);
        CHECK_NEAR(st.vmin, 34.0, 0.0);
        CHECK_NEAR(st.vmax, 34.0, 0.0);
        free_img(&img);
    }
}

/* ---------------------------------------------------------------- */
/* colouring, limits, statistics                                    */
/* ---------------------------------------------------------------- */

static void test_solid_mask(void)
{
    /* Row j=0: 5 (solid, huge) and -1; row j=1: 1 and 2 (solid). */
    double v[4] = {1000.0, -1.0, 1.0, 2.0};
    uint8_t m[4] = {1, 0, 0, 1};
    RenderField f = make_field(v, m, 2, 2);
    RenderOptions o = {.pixel_scale = 2};
    RenderStats st;
    Img img;
    if (render_load("mask.png", &f, &o, &st, &img) == 0) {
        CHECK(img.width == 4 && img.height == 4);
        for (int y = 0; y < 4; y++) {
            for (int x = 0; x < 4; x++) {
                int i = x / 2;
                int j = 1 - y / 2;
                if (m[j * 2 + i]) {
                    EXPECT_PX(&img, x, y, SOLID);
                } else {
                    EXPECT_PX_T(&img, x, y, v[j * 2 + i] / 1.0);
                }
            }
        }
        /* Solid cells are excluded from stats and from the auto limit. */
        CHECK_NEAR(st.vmin, -1.0, 0.0);
        CHECK_NEAR(st.vmax, 1.0, 0.0);
        CHECK_NEAR(st.color_limit, 1.0, 0.0);
        free_img(&img);
    }

    /* Everything solid: no stats, limit 1. */
    uint8_t all[4] = {1, 1, 1, 1};
    RenderField fa = make_field(v, all, 2, 2);
    RenderOptions o1 = {.pixel_scale = 1};
    if (render_load("mask_all.png", &fa, &o1, &st, &img) == 0) {
        CHECK(count_color(&img, SOLID) == 4);
        CHECK_NEAR(st.vmin, 0.0, 0.0);
        CHECK_NEAR(st.vmax, 0.0, 0.0);
        CHECK_NEAR(st.color_limit, 1.0, 0.0);
        free_img(&img);
    }

    /* A mask outside the crop window does not influence the stats. */
    double v2[4] = {0.5, 7.0, -3.0, 0.25};
    uint8_t m2[4] = {0, 1, 0, 0};
    RenderField f2 = make_field(v2, m2, 2, 2);
    f2.j0 = 0;
    f2.h = 1; /* only row j = 0: values 0.5 and (solid) 7 */
    if (render_load("mask_crop.png", &f2, &o1, &st, &img) == 0) {
        CHECK(img.width == 2 && img.height == 1);
        EXPECT_PX_T(&img, 0, 0, 1.0);
        EXPECT_PX(&img, 1, 0, SOLID);
        CHECK_NEAR(st.vmin, 0.5, 0.0);
        CHECK_NEAR(st.vmax, 0.5, 0.0);
        CHECK_NEAR(st.color_limit, 0.5, 0.0);
        free_img(&img);
    }
}

static void test_auto_limit_and_stats(void)
{
    /* Row j=0: -2, 1; row j=1: 0, 4. Auto limit = 4. */
    double v[4] = {-2.0, 1.0, 0.0, 4.0};
    RenderField f = make_field(v, NULL, 2, 2);
    RenderOptions o = {.pixel_scale = 1, .color_limit = 0.0};
    RenderStats st = {.vmin = 99, .vmax = 99, .color_limit = 99};
    Img img;
    if (render_load("auto.png", &f, &o, &st, &img) == 0) {
        CHECK_NEAR(st.vmin, -2.0, 0.0);
        CHECK_NEAR(st.vmax, 4.0, 0.0);
        CHECK_NEAR(st.color_limit, 4.0, 0.0);
        EXPECT_PX_T(&img, 0, 1, -0.5); /* (i0, j0) -> bottom left */
        EXPECT_PX_T(&img, 1, 1, 0.25);
        EXPECT_PX_T(&img, 0, 0, 0.0);
        EXPECT_PX_T(&img, 1, 0, 1.0);
        free_img(&img);
    }

    /* The negative extreme can define the limit; negative limit means auto, too. */
    double w[3] = {-8.0, 2.0, 4.0};
    RenderField fw = make_field(w, NULL, 3, 1);
    RenderOptions on = {.pixel_scale = 1, .color_limit = -5.0};
    if (render_load("auto2.png", &fw, &on, &st, &img) == 0) {
        CHECK_NEAR(st.color_limit, 8.0, 0.0);
        CHECK_NEAR(st.vmin, -8.0, 0.0);
        CHECK_NEAR(st.vmax, 4.0, 0.0);
        EXPECT_PX_T(&img, 0, 0, -1.0);
        EXPECT_PX_T(&img, 1, 0, 0.25);
        EXPECT_PX_T(&img, 2, 0, 0.5);
        free_img(&img);
    }

    /* Tiny amplitudes still use the full colour range. */
    double t[2] = {-3e-12, 1e-12};
    RenderField ft = make_field(t, NULL, 2, 1);
    if (render_load("auto3.png", &ft, &o, &st, &img) == 0) {
        CHECK_NEAR(st.color_limit, 3e-12, 1e-24);
        EXPECT_PX_T(&img, 0, 0, -1.0);
        free_img(&img);
    }
}

static void test_all_zero_field(void)
{
    double v[12] = {0};
    RenderField f = make_field(v, NULL, 4, 3);
    RenderOptions o = {.pixel_scale = 3};
    RenderStats st = {.vmin = 5, .vmax = 5, .color_limit = 5};
    Img img;
    if (render_load("zero.png", &f, &o, &st, &img) == 0) {
        CHECK(img.width == 12 && img.height == 9);
        CHECK(count_color(&img, WHITE) == 12 * 9);
        CHECK_NEAR(st.vmin, 0.0, 0.0);
        CHECK_NEAR(st.vmax, 0.0, 0.0);
        CHECK_NEAR(st.color_limit, 1.0, 0.0);
        free_img(&img);
    }
}

static void test_fixed_limit_clamping(void)
{
    double v[4] = {0.5, 2.0, -3.0, 1.0};
    RenderField f = make_field(v, NULL, 4, 1);
    RenderOptions o = {.pixel_scale = 1, .color_limit = 1.0};
    RenderStats st;
    Img img;
    if (render_load("fixed.png", &f, &o, &st, &img) == 0) {
        EXPECT_PX_T(&img, 0, 0, 0.5);
        EXPECT_PX_T(&img, 1, 0, 1.0);
        EXPECT_PX_T(&img, 2, 0, -1.0);
        EXPECT_PX_T(&img, 3, 0, 1.0);
        /* Clamped values show the end colours. */
        static const uint8_t dark_red[3] = {103, 0, 31};
        static const uint8_t dark_blue[3] = {5, 48, 97};
        EXPECT_PX(&img, 1, 0, dark_red);
        EXPECT_PX(&img, 2, 0, dark_blue);
        /* Stats report the true range and the fixed limit. */
        CHECK_NEAR(st.vmin, -3.0, 0.0);
        CHECK_NEAR(st.vmax, 2.0, 0.0);
        CHECK_NEAR(st.color_limit, 1.0, 0.0);
        free_img(&img);
    }

    /* A large fixed limit keeps small values near white. */
    RenderOptions big = {.pixel_scale = 1, .color_limit = 1000.0};
    if (render_load("fixed2.png", &f, &big, &st, &img) == 0) {
        EXPECT_PX_T(&img, 0, 0, 0.0005);
        CHECK_NEAR(st.color_limit, 1000.0, 0.0);
        free_img(&img);
    }
}

static void test_non_finite_values(void)
{
    double v[5] = {NAN, INFINITY, -INFINITY, 2.0, -1.0};
    RenderField f = make_field(v, NULL, 5, 1);
    RenderOptions o = {.pixel_scale = 1};
    RenderStats st;
    Img img;
    if (render_load("nonfinite.png", &f, &o, &st, &img) == 0) {
        EXPECT_PX(&img, 0, 0, WHITE);
        EXPECT_PX(&img, 1, 0, WHITE);
        EXPECT_PX(&img, 2, 0, WHITE);
        EXPECT_PX_T(&img, 3, 0, 1.0);
        EXPECT_PX_T(&img, 4, 0, -0.5);
        CHECK_NEAR(st.vmin, -1.0, 0.0);
        CHECK_NEAR(st.vmax, 2.0, 0.0);
        CHECK_NEAR(st.color_limit, 2.0, 0.0);
        free_img(&img);
    }

    /* Only non-finite values: limit 1, stats 0. */
    double w[2] = {NAN, INFINITY};
    RenderField fw = make_field(w, NULL, 2, 1);
    if (render_load("nonfinite2.png", &fw, &o, &st, &img) == 0) {
        CHECK(count_color(&img, WHITE) == 2);
        CHECK_NEAR(st.vmin, 0.0, 0.0);
        CHECK_NEAR(st.vmax, 0.0, 0.0);
        CHECK_NEAR(st.color_limit, 1.0, 0.0);
        free_img(&img);
    }
}

/* ---------------------------------------------------------------- */
/* outline                                                          */
/* ---------------------------------------------------------------- */

static void test_outline(void)
{
    double v[48] = {0};
    RenderField f = make_field(v, NULL, 8, 6);
    RenderOptions o = {.pixel_scale = 2, .outline = true,
                       .outline_i0 = 2, .outline_j0 = 1, .outline_w = 4, .outline_h = 3};
    Img img;
    if (render_load("outline.png", &f, &o, NULL, &img) == 0) {
        CHECK(img.width == 16 && img.height == 12);
        /* Rectangle spans pixels x 4..11, y 4..9 (cell rows 1..3, flipped). */
        for (int y = 0; y < 12; y++) {
            for (int x = 0; x < 16; x++) {
                bool inside = x >= 4 && x <= 11 && y >= 4 && y <= 9;
                bool border = inside && (x == 4 || x == 11 || y == 4 || y == 9);
                EXPECT_PX(&img, x, y, border ? BLACK : WHITE);
            }
        }
        CHECK(count_color(&img, BLACK) == 24);
        free_img(&img);
    }

    /* The outline replaces solid and field colours on the border. */
    double u[9] = {0};
    u[4] = 1.0;
    uint8_t m[9] = {1, 1, 1, 1, 0, 1, 1, 1, 1};
    RenderField fm = make_field(u, m, 3, 3);
    RenderOptions o1 = {.pixel_scale = 1, .outline = true,
                        .outline_i0 = 0, .outline_j0 = 0, .outline_w = 3, .outline_h = 3};
    if (render_load("outline2.png", &fm, &o1, NULL, &img) == 0) {
        CHECK(count_color(&img, BLACK) == 8);
        EXPECT_PX_T(&img, 1, 1, 1.0);
        free_img(&img);
    }

    /* A 1 x 1 rectangle at scale 3 is a 3 x 3 box with a hollow centre pixel. */
    double z[9] = {0};
    RenderField fz = make_field(z, NULL, 3, 3);
    RenderOptions o3 = {.pixel_scale = 3, .outline = true,
                        .outline_i0 = 1, .outline_j0 = 1, .outline_w = 1, .outline_h = 1};
    if (render_load("outline3.png", &fz, &o3, NULL, &img) == 0) {
        CHECK(count_color(&img, BLACK) == 8);
        EXPECT_PX(&img, 4, 4, WHITE);
        EXPECT_PX(&img, 3, 3, BLACK);
        EXPECT_PX(&img, 5, 5, BLACK);
        free_img(&img);
    }

    /* The rectangle covering the whole window is drawn on the image border. */
    RenderOptions of = {.pixel_scale = 1, .outline = true,
                        .outline_i0 = 0, .outline_j0 = 0, .outline_w = 3, .outline_h = 3};
    if (render_load("outline4.png", &fz, &of, NULL, &img) == 0) {
        CHECK(count_color(&img, BLACK) == 8);
        EXPECT_PX(&img, 1, 1, WHITE);
        free_img(&img);
    }
}

/* Crop offset and pixel_scale > 1 together. Expected pixels are written out
 * by hand from the geometry, not derived from the renderer's formulas. */
static void test_outline_crop_offset_scaled(void)
{
    double v[80] = {0};
    RenderField f = make_field(v, NULL, 10, 8);
    f.i0 = 2;
    f.j0 = 1;
    f.w = 6;
    f.h = 5; /* cells x 2..7, y 1..5; image 18 x 15 at scale 3 */
    Img img;

    /* Cells x 3..5, y 2..3 -> crop columns 1..3 -> pixels x 3..11;
     * crop rows 1..2 (from the bottom) -> image rows 6..11. */
    RenderOptions o = {.pixel_scale = 3, .outline = true,
                       .outline_i0 = 3, .outline_j0 = 2, .outline_w = 3, .outline_h = 2};
    if (render_load("outline_crop_scaled.png", &f, &o, NULL, &img) == 0) {
        CHECK(img.width == 18 && img.height == 15);
        for (int y = 0; y < 15; y++) {
            for (int x = 0; x < 18; x++) {
                bool inside = x >= 3 && x <= 11 && y >= 6 && y <= 11;
                bool border = inside && (x == 3 || x == 11 || y == 6 || y == 11);
                EXPECT_PX(&img, x, y, border ? BLACK : WHITE);
            }
        }
        CHECK(count_color(&img, BLACK) == 26);
        free_img(&img);
    }

    /* Cells x 6..10, y 0..2: partly outside the crop on the right and at the
     * bottom. Crop columns 4..8 -> pixels x 12..26 (clipped at 17); crop rows
     * -1..1 -> image rows 9..17 (clipped at 14). Visible: the top edge
     * (y = 9, x 12..17) and the left edge (x = 12, y 9..14). */
    RenderOptions p = {.pixel_scale = 3, .outline = true,
                       .outline_i0 = 6, .outline_j0 = 0, .outline_w = 5, .outline_h = 3};
    if (render_load("outline_crop_partial.png", &f, &p, NULL, &img) == 0) {
        for (int y = 0; y < 15; y++) {
            for (int x = 0; x < 18; x++) {
                bool border = (y == 9 && x >= 12) || (x == 12 && y >= 9);
                EXPECT_PX(&img, x, y, border ? BLACK : WHITE);
            }
        }
        CHECK(count_color(&img, BLACK) == 11);
        free_img(&img);
    }
}

/* color_limit = -inf counts as "<= 0", i.e. automatic. */
static void test_negative_infinite_limit_is_auto(void)
{
    double v[4] = {0.0, 2.0, -4.0, 1.0};
    RenderField f = make_field(v, NULL, 2, 2);
    RenderOptions o = {.pixel_scale = 1, .color_limit = -INFINITY};
    RenderStats st = {0};
    Img img;
    if (render_load("limit_neg_inf.png", &f, &o, &st, &img) == 0) {
        CHECK_NEAR(st.color_limit, 4.0, 0.0);
        free_img(&img);
    }
}

static void test_outline_clipping_and_ignored(void)
{
    double v[36] = {0};
    RenderField f = make_field(v, NULL, 6, 6);
    f.i0 = 1;
    f.j0 = 1;
    f.w = 3;
    f.h = 3;
    /* Rectangle cells x 2..5, y 2..5: left edge and bottom edge are visible. */
    RenderOptions o = {.pixel_scale = 1, .outline = true,
                       .outline_i0 = 2, .outline_j0 = 2, .outline_w = 4, .outline_h = 4};
    Img img;
    if (render_load("outline_clip.png", &f, &o, NULL, &img) == 0) {
        CHECK(img.width == 3 && img.height == 3);
        CHECK(count_color(&img, BLACK) == 3);
        EXPECT_PX(&img, 1, 1, BLACK);
        EXPECT_PX(&img, 2, 1, BLACK);
        EXPECT_PX(&img, 1, 0, BLACK);
        free_img(&img);
    }

    /* Rectangle containing the window: nothing is visible. */
    RenderOptions big = {.pixel_scale = 1, .outline = true,
                         .outline_i0 = 0, .outline_j0 = 0, .outline_w = 6, .outline_h = 6};
    if (render_load("outline_big.png", &f, &big, NULL, &img) == 0) {
        CHECK(count_color(&img, BLACK) == 0);
        free_img(&img);
    }

    /* Ignored cases: disabled, non-intersecting, degenerate. */
    RenderOptions off = o;
    off.outline = false;
    RenderOptions far_away = o;
    far_away.outline_i0 = 10;
    RenderOptions below = o;
    below.outline_j0 = -9;
    below.outline_h = 4;
    RenderOptions zero_w = o;
    zero_w.outline_w = 0;
    RenderOptions neg_h = o;
    neg_h.outline_h = -3;
    const RenderOptions *cases[] = {&off, &far_away, &below, &zero_w, &neg_h};
    for (size_t k = 0; k < sizeof cases / sizeof cases[0]; k++) {
        if (render_load("outline_none.png", &f, cases[k], NULL, &img) == 0) {
            CHECK_MSG(count_color(&img, BLACK) == 0, "case %zu drew an outline", k);
            free_img(&img);
        }
    }
}

/* ---------------------------------------------------------------- */
/* PNG metadata                                                     */
/* ---------------------------------------------------------------- */

static void test_text_chunks(void)
{
    double v[4] = {0, 1, 2, 3};
    RenderField f = make_field(v, NULL, 2, 2);
    RenderOptions o = {.pixel_scale = 1, .comment = "fdtd2d pressure, step=7, time=1.5e-03 s"};
    char path[512];
    FdtdError err = {{0}};
    tmp_path(path, sizeof path, "text.png");
    CHECK(render_field_png(path, &f, &o, NULL, &err) == 0);

    PngInfo info;
    CHECK(read_info(path, &info) == 0);
    CHECK(info.has_comment);
    CHECK(strcmp(info.comment, "fdtd2d pressure, step=7, time=1.5e-03 s") == 0);
    CHECK(info.has_software);
    CHECK(strcmp(info.software, "fdtd2d " FDTD2D_VERSION) == 0);

    /* No comment requested: only Software is present. */
    RenderOptions o2 = {.pixel_scale = 1};
    tmp_path(path, sizeof path, "text2.png");
    CHECK(render_field_png(path, &f, &o2, NULL, &err) == 0);
    CHECK(read_info(path, &info) == 0);
    CHECK(!info.has_comment);
    CHECK(info.has_software);

    /* An empty comment is still written. */
    RenderOptions o3 = {.pixel_scale = 1, .comment = ""};
    tmp_path(path, sizeof path, "text3.png");
    CHECK(render_field_png(path, &f, &o3, NULL, &err) == 0);
    CHECK(read_info(path, &info) == 0);
    CHECK(info.has_comment);
    CHECK(info.comment[0] == '\0');
}

static void test_overwrite_existing_file(void)
{
    double a[4] = {1, 1, 1, 1};
    double b[2] = {-1, -1};
    RenderField fa = make_field(a, NULL, 2, 2);
    RenderField fb = make_field(b, NULL, 2, 1);
    RenderOptions o = {.pixel_scale = 1};
    Img img;
    CHECK(render_load("over.png", &fa, &o, NULL, &img) == 0);
    free_img(&img);
    if (render_load("over.png", &fb, &o, NULL, &img) == 0) {
        CHECK(img.width == 2 && img.height == 1);
        EXPECT_PX_T(&img, 0, 0, -1.0);
        free_img(&img);
    }
}

/* ---------------------------------------------------------------- */
/* limits and errors                                                */
/* ---------------------------------------------------------------- */

static void expect_failure(const char *name, const RenderField *f, const RenderOptions *o,
                           const char *fragment)
{
    char path[512];
    FdtdError err = {{0}};
    tmp_path(path, sizeof path, name);
    remove(path);
    int rc = render_field_png(path, f, o, NULL, &err);
    CHECK_MSG(rc == -1, "%s: expected failure", name);
    CHECK_MSG(err.msg[0] != '\0', "%s: empty error message", name);
    CHECK_MSG(strchr(err.msg, '\n') == NULL, "%s: multi-line message", name);
    if (fragment) {
        CHECK_CONTAINS(err.msg, fragment);
    }
    CHECK_MSG(!file_exists(path), "%s: a file was left behind", name);
}

static void test_oversize_images(void)
{
    /* One-row field, wide enough to reach the PNG limits (nothing is allocated per pixel). */
    enum { WIDE = 65536 };
    double *v = calloc(WIDE, sizeof *v);
    CHECK(v != NULL);
    if (!v) {
        return;
    }
    RenderOptions o = {.pixel_scale = 1};
    Img img;

    /* Exactly 65535 pixels wide is accepted. */
    RenderField ok = make_field(v, NULL, WIDE, 1);
    ok.w = 65535;
    if (render_load("wide_ok.png", &ok, &o, NULL, &img) == 0) {
        CHECK(img.width == 65535 && img.height == 1);
        free_img(&img);
    }

    /* One more is not. */
    RenderField too_wide = make_field(v, NULL, WIDE, 1);
    expect_failure("wide_bad.png", &too_wide, &o, "65535");
    /* err may be NULL. */
    char null_err_path[512];
    tmp_path(null_err_path, sizeof null_err_path, "wide_bad_null_err.png");
    CHECK(render_field_png(null_err_path, &too_wide, &o, NULL, NULL) == -1);
    CHECK(!file_exists(null_err_path));

    /* Same through pixel_scale (width and height). */
    RenderField small = make_field(v, NULL, 400, 1);
    RenderOptions s = {.pixel_scale = 164}; /* 400 * 164 = 65600 */
    expect_failure("scale_bad_w.png", &small, &s, "65535");
    RenderField one = make_field(v, NULL, 1, 1);
    s.pixel_scale = 65536;
    expect_failure("scale_bad_1.png", &one, &s, "65535");
    s.pixel_scale = INT_MAX;
    expect_failure("scale_bad_max.png", &one, &s, "65535");
    RenderField two = make_field(v, NULL, 2, 2);
    s.pixel_scale = 32768; /* 2 * 32768 = 65536 */
    expect_failure("scale_bad_2.png", &two, &s, "65535");
    RenderField wide_max = make_field(v, NULL, 65535, 1);
    s.pixel_scale = INT_MAX;
    expect_failure("scale_bad_ovf.png", &wide_max, &s, "65535");
    free(v);

    /* Tall: height limit (values array is 1 x 65536, a column). */
    double *col = calloc(WIDE, sizeof *col);
    CHECK(col != NULL);
    if (!col) {
        return;
    }
    RenderField tall_col = make_field(col, NULL, 1, WIDE);
    RenderOptions o1 = {.pixel_scale = 1};
    expect_failure("tall_bad.png", &tall_col, &o1, "65535");
    tall_col.h = 65535;
    if (render_load("tall_ok.png", &tall_col, &o1, NULL, &img) == 0) {
        CHECK(img.width == 1 && img.height == 65535);
        free_img(&img);
    }
    free(col);
}

static void test_unwritable_path(void)
{
    double v[4] = {0, 1, 2, 3};
    RenderField f = make_field(v, NULL, 2, 2);
    RenderOptions o = {.pixel_scale = 1, .comment = "x"};
    char path[512];
    FdtdError err = {{0}};
    int fds_before = count_open_fds();

    tmp_path(path, sizeof path, "no_such_dir/out.png");
    CHECK(render_field_png(path, &f, &o, NULL, &err) == -1);
    CHECK(err.msg[0] != '\0');
    CHECK_CONTAINS(err.msg, "no_such_dir/out.png");
    CHECK(strchr(err.msg, '\n') == NULL);

    /* A directory as target. */
    snprintf(path, sizeof path, "%s", FDTD2D_TEST_TMPDIR);
    memset(&err, 0, sizeof err);
    CHECK(render_field_png(path, &f, &o, NULL, &err) == -1);
    CHECK(err.msg[0] != '\0');
    struct stat st;
    CHECK(stat(FDTD2D_TEST_TMPDIR, &st) == 0 && S_ISDIR(st.st_mode));

    /* Empty path. */
    memset(&err, 0, sizeof err);
    CHECK(render_field_png("", &f, &o, NULL, &err) == -1);
    CHECK(err.msg[0] != '\0');

    /* Stats are not touched on failure. */
    RenderStats stats = {.vmin = 42, .vmax = 43, .color_limit = 44};
    tmp_path(path, sizeof path, "no_such_dir/out.png");
    CHECK(render_field_png(path, &f, &o, &stats, &err) == -1);
    CHECK_NEAR(stats.vmin, 42.0, 0.0);
    CHECK_NEAR(stats.color_limit, 44.0, 0.0);

    int fds_after = count_open_fds();
    CHECK_MSG(fds_before == fds_after, "file descriptors leaked: %d -> %d", fds_before, fds_after);
}

static void test_write_failure_device_full(void)
{
    /* /dev/full accepts open() and fails every write with ENOSPC. */
    FILE *probe = fopen("/dev/full", "wb");
    if (!probe) {
        fprintf(stderr, "note: /dev/full not available, test skipped\n");
        return;
    }
    fclose(probe);

    /* Noisy data so that the PNG is much larger than any stdio buffer. */
    enum { N = 128 };
    double *v = malloc((size_t)N * N * sizeof *v);
    CHECK(v != NULL);
    if (!v) {
        return;
    }
    uint32_t s = 12345u;
    for (int k = 0; k < N * N; k++) {
        s = s * 1664525u + 1013904223u;
        v[k] = (double)(s >> 8) / (double)(1u << 24) - 0.5;
    }
    RenderField f = make_field(v, NULL, N, N);
    RenderOptions o = {.pixel_scale = 1};
    FdtdError err = {{0}};
    int fds_before = count_open_fds();
    CHECK(render_field_png("/dev/full", &f, &o, NULL, &err) == -1);
    CHECK(err.msg[0] != '\0');
    CHECK_CONTAINS(err.msg, "/dev/full");
    CHECK(strchr(err.msg, '\n') == NULL);
    /* The device node must still exist (only regular files may be removed). */
    struct stat st;
    CHECK(stat("/dev/full", &st) == 0 && S_ISCHR(st.st_mode));
    int fds_after = count_open_fds();
    CHECK_MSG(fds_before == fds_after, "file descriptors leaked: %d -> %d", fds_before, fds_after);
    free(v);
}

static void test_large_noisy_image(void)
{
    /* A larger noisy image renders fine and leaves a complete, readable file;
     * repeated runs with the same name are stable (no leaked state). */
    enum { N = 150 };
    double *v = malloc((size_t)N * N * sizeof *v);
    CHECK(v != NULL);
    if (!v) {
        return;
    }
    uint32_t s = 99u;
    for (int k = 0; k < N * N; k++) {
        s = s * 1664525u + 1013904223u;
        v[k] = (double)(s >> 8) / (double)(1u << 24) - 0.5;
    }
    RenderField f = make_field(v, NULL, N, N);
    RenderOptions o = {.pixel_scale = 2, .comment = "noise"};
    int fds_before = count_open_fds();
    for (int rep = 0; rep < 3; rep++) {
        Img img;
        RenderStats st;
        if (render_load("noise.png", &f, &o, &st, &img) == 0) {
            CHECK(img.width == 2 * N && img.height == 2 * N);
            CHECK(st.vmin >= -0.5 && st.vmax <= 0.5 && st.vmin < st.vmax);
            /* Spot check against the colour map: cell (i = 7, j = 11). */
            double want = v[11 * N + 7] / st.color_limit;
            EXPECT_PX_T(&img, 2 * 7, 2 * (N - 1 - 11), want);
            EXPECT_PX_T(&img, 2 * 7 + 1, 2 * (N - 1 - 11) + 1, want);
            free_img(&img);
        }
    }
    CHECK(count_open_fds() == fds_before);
    free(v);
}

int main(void)
{
    make_tmpdir();
    RUN_TEST(test_colormap_control_points);
    RUN_TEST(test_colormap_midpoints_and_white);
    RUN_TEST(test_colormap_clamp_and_nan);
    RUN_TEST(test_dimensions_and_format);
    RUN_TEST(test_y_flip);
    RUN_TEST(test_crop_window);
    RUN_TEST(test_solid_mask);
    RUN_TEST(test_auto_limit_and_stats);
    RUN_TEST(test_all_zero_field);
    RUN_TEST(test_fixed_limit_clamping);
    RUN_TEST(test_non_finite_values);
    RUN_TEST(test_outline);
    RUN_TEST(test_outline_clipping_and_ignored);
    RUN_TEST(test_outline_crop_offset_scaled);
    RUN_TEST(test_negative_infinite_limit_is_auto);
    RUN_TEST(test_text_chunks);
    RUN_TEST(test_overwrite_existing_file);
    RUN_TEST(test_oversize_images);
    RUN_TEST(test_unwritable_path);
    RUN_TEST(test_write_failure_device_full);
    RUN_TEST(test_large_noisy_image);
    return TEST_REPORT();
}
