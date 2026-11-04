/*
 * config.c - parser layer 1: TOML document -> validated SimConfig.
 *
 * The file is organised bottom-up:
 *   - error helpers      every message is "<source>:<line>: <key path>: <problem>"
 *   - table helpers      key iteration / key checking, lookups
 *   - value accessors    look up one key, type-check it, report with key path
 *   - section parsers    one per TOML table (simulation, grid, medium, ...)
 *   - public entry points: config_load_file / config_load_string / config_free
 *
 * The tomlc17 tree is only borrowed while parsing. Everything SimConfig keeps
 * (strings, polygon vertices) is copied, and the tree is released before the
 * function returns. On failure the partial cfg is released, so callers always
 * see an empty config.
 */
#include "config.h"

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "tomlc17.h"

#define CFG_PATH_MAX 256 /* "obstacle[12].shape.vertices[34][0]" fits easily */
#define CFG_KEY_MAX 96   /* longer TOML keys are truncated in messages */

/* Parsing context: message prefix data plus the caller's error buffer. */
typedef struct ParseCtx {
    const char *source_name; /* never NULL */
    FdtdError *err;          /* may be NULL */
} ParseCtx;

/* Tri-state result of a value accessor. FIELD_INVALID means that the accessor
 * already filled ctx->err. */
enum {
    FIELD_INVALID = -1,
    FIELD_MISSING = 0,
    FIELD_FOUND = 1
};

static void cfg_error_at(ParseCtx *ctx, int line, const char *path, const char *fmt, ...)
    FDTD_PRINTF(4, 5);

/* ------------------------------------------------------------------ */
/* error helpers                                                      */
/* ------------------------------------------------------------------ */

/* Set "<source>:<line>: <path>: <problem>"; the line is omitted when unknown. */
static void cfg_error_at(ParseCtx *ctx, int line, const char *path, const char *fmt, ...)
{
    if (!ctx->err) {
        return;
    }
    char problem[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(problem, sizeof problem, fmt, ap);
    va_end(ap);

    bool has_path = path && path[0] != '\0';
    if (has_path && line > 0) {
        fdtd_error_set(ctx->err, "%s:%d: %s: %s", ctx->source_name, line, path, problem);
    } else if (has_path) {
        fdtd_error_set(ctx->err, "%s: %s: %s", ctx->source_name, path, problem);
    } else if (line > 0) {
        fdtd_error_set(ctx->err, "%s:%d: %s", ctx->source_name, line, problem);
    } else {
        fdtd_error_set(ctx->err, "%s: %s", ctx->source_name, problem);
    }
}

/* Turn a message from a *_validate() helper into "<source>:<line>: <path>: msg". */
static void cfg_locate_error(ParseCtx *ctx, int line, const char *path)
{
    if (!ctx->err) {
        return;
    }
    if (line > 0) {
        fdtd_error_prefix(ctx->err, "%s:%d: %s", ctx->source_name, line, path);
    } else {
        fdtd_error_prefix(ctx->err, "%s: %s", ctx->source_name, path);
    }
}

/* Append `s` to buf (stay NUL terminated inside cap bytes). */
static void buf_append(char *buf, size_t cap, size_t *pos, const char *s)
{
    while (*s != '\0' && *pos + 1 < cap) {
        buf[(*pos)++] = *s++;
    }
    buf[*pos] = '\0';
}

/* Compose "<base>.<key>" (or just "<key>" when base is empty). */
static void path_join(char *buf, size_t cap, const char *base, const char *key)
{
    size_t pos = 0;
    buf[0] = '\0';
    if (base[0] != '\0') {
        buf_append(buf, cap, &pos, base);
        buf_append(buf, cap, &pos, ".");
    }
    buf_append(buf, cap, &pos, key);
}

/* Compose "<base>[<index>]", truncating instead of overflowing. */
static void path_index(char *buf, size_t cap, const char *base, int index)
{
    char suffix[16]; /* "[%d]" of any int needs at most 14 bytes */
    snprintf(suffix, sizeof suffix, "[%d]", index);
    size_t pos = 0;
    buf[0] = '\0';
    buf_append(buf, cap, &pos, base);
    buf_append(buf, cap, &pos, suffix);
}

/* ------------------------------------------------------------------ */
/* table helpers                                                      */
/* ------------------------------------------------------------------ */

static bool table_lookup(const toml_datum_t *tab, const char *key, toml_datum_t *out)
{
    if (tab->type != TOML_TABLE) {
        return false;
    }
    toml_datum_t d = toml_get(*tab, key);
    if (d.type == TOML_UNKNOWN) {
        return false;
    }
    *out = d;
    return true;
}

static bool key_matches(const toml_datum_t *tab, int index, const char *name)
{
    size_t len = strlen(name);
    return tab->u.tab.len[index] == (int)len &&
           memcmp(tab->u.tab.key[index], name, len) == 0;
}

/* Copy a table key into buf (TOML keys are not NUL terminated by contract). */
static void key_copy(const toml_datum_t *tab, int index, char *buf, size_t cap)
{
    int len = tab->u.tab.len[index];
    if (len < 0) {
        len = 0;
    }
    if ((size_t)len > cap - 1) {
        len = (int)(cap - 1);
    }
    memcpy(buf, tab->u.tab.key[index], (size_t)len);
    buf[len] = '\0';
}

/* Source line of a key/value pair, falling back to the containing table. */
static int key_line(const toml_datum_t *tab, int index)
{
    int line = tab->u.tab.value[index].lineno;
    return line != 0 ? line : tab->lineno;
}

/* Reject every key of `tab` that is not in the NULL terminated `allowed`. */
static int check_keys(ParseCtx *ctx, const toml_datum_t *tab, const char *path,
                      const char *const *allowed)
{
    for (int i = 0; i < tab->u.tab.size; i++) {
        bool known = false;
        for (int k = 0; allowed[k] != NULL; k++) {
            if (key_matches(tab, i, allowed[k])) {
                known = true;
                break;
            }
        }
        if (!known) {
            char key[CFG_KEY_MAX];
            char p[CFG_PATH_MAX];
            key_copy(tab, i, key, sizeof key);
            path_join(p, sizeof p, path, key);
            cfg_error_at(ctx, key_line(tab, i), p, "unknown key");
            return -1;
        }
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* value accessors                                                    */
/* ------------------------------------------------------------------ */

static const char *datum_type_name(toml_type_t type)
{
    switch (type) {
    case TOML_STRING:
        return "a string";
    case TOML_INT64:
        return "an integer";
    case TOML_FP64:
        return "a float";
    case TOML_BOOLEAN:
        return "a boolean";
    case TOML_DATE:
        return "a date";
    case TOML_TIME:
        return "a time";
    case TOML_DATETIME:
        return "a datetime";
    case TOML_DATETIMETZ:
        return "an offset datetime";
    case TOML_ARRAY:
        return "an array";
    case TOML_TABLE:
        return "a table";
    case TOML_UNKNOWN:
        break;
    }
    return "nothing";
}

/* Human readable description of a value, used in "got ..." message parts. */
static const char *datum_desc(const toml_datum_t *d)
{
    if (d->type == TOML_FP64 && !isfinite(d->u.fp64)) {
        return "a non-finite float";
    }
    return datum_type_name(d->type);
}

/* Report a wrong value type at "<path>.<key>". */
static int type_error(ParseCtx *ctx, const char *path, const char *key,
                      const toml_datum_t *d, const char *expected)
{
    char p[CFG_PATH_MAX];
    path_join(p, sizeof p, path, key);
    cfg_error_at(ctx, d->lineno, p, "must be %s (got %s)", expected, datum_desc(d));
    return FIELD_INVALID;
}

/* Source line of `key` in `tab`, or 0 when the key is absent. */
static int lookup_line(const toml_datum_t *tab, const char *key)
{
    toml_datum_t d;
    if (!table_lookup(tab, key, &d)) {
        return 0;
    }
    return d.lineno;
}

/* Accept TOML integers where a float is expected (all numbers are SI). */
static bool datum_as_double(const toml_datum_t *d, double *out)
{
    if (d->type == TOML_FP64) {
        if (!isfinite(d->u.fp64)) {
            return false;
        }
        *out = d->u.fp64;
        return true;
    }
    if (d->type == TOML_INT64) {
        *out = (double)d->u.int64;
        return true;
    }
    return false;
}

/* Look up a number. `line_out` (may be NULL) receives the value's source line. */
static int get_double(const toml_datum_t *tab, ParseCtx *ctx, const char *path,
                      const char *key, double *out, int *line_out)
{
    toml_datum_t d;
    if (!table_lookup(tab, key, &d)) {
        return FIELD_MISSING;
    }
    if (line_out) {
        *line_out = d.lineno;
    }
    if (!datum_as_double(&d, out)) {
        return type_error(ctx, path, key, &d, "a finite number");
    }
    return FIELD_FOUND;
}

/* Look up a number that must be > 0. */
static int get_double_positive(const toml_datum_t *tab, ParseCtx *ctx, const char *path,
                               const char *key, double *out)
{
    int line = 0;
    int r = get_double(tab, ctx, path, key, out, &line);
    if (r != FIELD_FOUND) {
        return r;
    }
    if (!(*out > 0.0)) {
        char p[CFG_PATH_MAX];
        path_join(p, sizeof p, path, key);
        cfg_error_at(ctx, line, p, "must be > 0 (got %g)", *out);
        return FIELD_INVALID;
    }
    return FIELD_FOUND;
}

/* Look up an integer. `line_out` (may be NULL) receives the value's source line. */
static int get_int(const toml_datum_t *tab, ParseCtx *ctx, const char *path,
                   const char *key, int *out, int *line_out)
{
    toml_datum_t d;
    if (!table_lookup(tab, key, &d)) {
        return FIELD_MISSING;
    }
    if (line_out) {
        *line_out = d.lineno;
    }
    if (d.type != TOML_INT64) {
        return type_error(ctx, path, key, &d, "an integer");
    }
    if (d.u.int64 < INT_MIN || d.u.int64 > INT_MAX) {
        char p[CFG_PATH_MAX];
        path_join(p, sizeof p, path, key);
        cfg_error_at(ctx, d.lineno, p, "is out of range for a 32-bit integer (got %lld)",
                     (long long)d.u.int64);
        return FIELD_INVALID;
    }
    *out = (int)d.u.int64;
    return FIELD_FOUND;
}

/* Look up an integer in [lo, hi]. */
static int get_int_range(const toml_datum_t *tab, ParseCtx *ctx, const char *path,
                         const char *key, int lo, int hi, int *out)
{
    int line = 0;
    int r = get_int(tab, ctx, path, key, out, &line);
    if (r != FIELD_FOUND) {
        return r;
    }
    if (*out < lo || *out > hi) {
        char p[CFG_PATH_MAX];
        path_join(p, sizeof p, path, key);
        cfg_error_at(ctx, line, p, "must be in [%d, %d] (got %d)", lo, hi, *out);
        return FIELD_INVALID;
    }
    return FIELD_FOUND;
}

static int get_bool(const toml_datum_t *tab, ParseCtx *ctx, const char *path,
                    const char *key, bool *out)
{
    toml_datum_t d;
    if (!table_lookup(tab, key, &d)) {
        return FIELD_MISSING;
    }
    if (d.type != TOML_BOOLEAN) {
        return type_error(ctx, path, key, &d, "a boolean");
    }
    *out = d.u.boolean;
    return FIELD_FOUND;
}

/* Look up a string; *out borrows memory owned by the tomlc17 tree. */
static int get_string(const toml_datum_t *tab, ParseCtx *ctx, const char *path,
                      const char *key, const char **out)
{
    toml_datum_t d;
    if (!table_lookup(tab, key, &d)) {
        return FIELD_MISSING;
    }
    if (d.type != TOML_STRING) {
        return type_error(ctx, path, key, &d, "a string");
    }
    *out = d.u.s;
    return FIELD_FOUND;
}

/* Look up an array of exactly two numbers. `line_out` may be NULL. */
static int get_number_pair(const toml_datum_t *tab, ParseCtx *ctx, const char *path,
                           const char *key, double out[2], bool require_ordered, int *line_out)
{
    toml_datum_t d;
    if (!table_lookup(tab, key, &d)) {
        return FIELD_MISSING;
    }
    if (line_out) {
        *line_out = d.lineno;
    }
    if (d.type != TOML_ARRAY) {
        return type_error(ctx, path, key, &d, "an array of 2 numbers");
    }
    char p[CFG_PATH_MAX];
    path_join(p, sizeof p, path, key);
    if (d.u.arr.size != 2) {
        cfg_error_at(ctx, d.lineno, p, "must have exactly 2 numbers (got %d)", (int)d.u.arr.size);
        return FIELD_INVALID;
    }
    for (int i = 0; i < 2; i++) {
        const toml_datum_t *e = &d.u.arr.elem[i];
        if (!datum_as_double(e, &out[i])) {
            char ep[CFG_PATH_MAX];
            path_index(ep, sizeof ep, p, i);
            cfg_error_at(ctx, e->lineno, ep, "must be a finite number (got %s)", datum_desc(e));
            return FIELD_INVALID;
        }
    }
    if (require_ordered && !(out[1] > out[0])) {
        cfg_error_at(ctx, d.lineno, p, "must be [min, max] with max > min (got [%g, %g])",
                     out[0], out[1]);
        return FIELD_INVALID;
    }
    return FIELD_FOUND;
}

static int get_vec2(const toml_datum_t *tab, ParseCtx *ctx, const char *path,
                    const char *key, Vec2 *out)
{
    double v[2];
    int r = get_number_pair(tab, ctx, path, key, v, false, NULL);
    if (r == FIELD_FOUND) {
        out->x = v[0];
        out->y = v[1];
    }
    return r;
}

/* Look up an array of [x, y] pairs; *out is owned by the caller. */
static int get_vec2_list(const toml_datum_t *tab, ParseCtx *ctx, const char *path,
                         const char *key, Vec2 **out, int *count, int *line_out)
{
    toml_datum_t d;
    if (!table_lookup(tab, key, &d)) {
        return FIELD_MISSING;
    }
    if (line_out) {
        *line_out = d.lineno;
    }
    if (d.type != TOML_ARRAY) {
        return type_error(ctx, path, key, &d, "an array of [x, y] pairs");
    }
    char p[CFG_PATH_MAX];
    path_join(p, sizeof p, path, key);
    int n = (int)d.u.arr.size;
    Vec2 *v = NULL;
    if (n > 0) {
        v = fdtd_calloc((size_t)n, sizeof *v, ctx->err);
        if (!v) {
            return FIELD_INVALID;
        }
    }
    for (int i = 0; i < n; i++) {
        const toml_datum_t *e = &d.u.arr.elem[i];
        char ep[CFG_PATH_MAX];
        path_index(ep, sizeof ep, p, i);
        if (e->type != TOML_ARRAY) {
            cfg_error_at(ctx, e->lineno, ep, "must be an [x, y] pair (got %s)", datum_desc(e));
            free(v);
            return FIELD_INVALID;
        }
        if (e->u.arr.size != 2) {
            cfg_error_at(ctx, e->lineno, ep, "must have exactly 2 numbers (got %d)",
                         (int)e->u.arr.size);
            free(v);
            return FIELD_INVALID;
        }
        double xy[2];
        for (int k = 0; k < 2; k++) {
            const toml_datum_t *s = &e->u.arr.elem[k];
            if (!datum_as_double(s, &xy[k])) {
                char sp[CFG_PATH_MAX];
                path_index(sp, sizeof sp, ep, k);
                cfg_error_at(ctx, s->lineno, sp, "must be a finite number (got %s)",
                             datum_desc(s));
                free(v);
                return FIELD_INVALID;
            }
        }
        v[i].x = xy[0];
        v[i].y = xy[1];
    }
    *out = v;
    *count = n;
    return FIELD_FOUND;
}

/* Report a missing required key (line of the containing table) or pass on the
 * error that the accessor already reported. */
static int require_field(ParseCtx *ctx, const toml_datum_t *tab, const char *path,
                         const char *key, int accessor_result)
{
    if (accessor_result == FIELD_FOUND) {
        return 0;
    }
    if (accessor_result == FIELD_MISSING) {
        char p[CFG_PATH_MAX];
        path_join(p, sizeof p, path, key);
        cfg_error_at(ctx, tab->lineno, p, "missing required key");
    }
    return -1;
}

/* ------------------------------------------------------------------ */
/* shapes                                                             */
/* ------------------------------------------------------------------ */

static const char *const k_shape_keys_point[] = {"type", "position", NULL};
static const char *const k_shape_keys_segment[] = {"type", "start", "end", NULL};
static const char *const k_shape_keys_rect[] = {"type", "min", "max", NULL};
static const char *const k_shape_keys_circle[] = {"type", "center", "radius", NULL};
static const char *const k_shape_keys_polygon[] = {"type", "vertices", NULL};

static const char *const *shape_allowed_keys(ShapeType type)
{
    switch (type) {
    case SHAPE_POINT:
        return k_shape_keys_point;
    case SHAPE_SEGMENT:
        return k_shape_keys_segment;
    case SHAPE_RECT:
        return k_shape_keys_rect;
    case SHAPE_CIRCLE:
        return k_shape_keys_circle;
    case SHAPE_POLYGON:
        return k_shape_keys_polygon;
    case SHAPE_TYPE_COUNT:
        break;
    }
    return k_shape_keys_point;
}

/* Resolve the mandatory `type` key of a shape or signal table. */
static int get_type_key(ParseCtx *ctx, const toml_datum_t *tab, const char *path,
                        const char **name, int *line_out)
{
    char p[CFG_PATH_MAX];
    path_join(p, sizeof p, path, "type");
    toml_datum_t d;
    if (!table_lookup(tab, "type", &d)) {
        cfg_error_at(ctx, tab->lineno, p, "missing required key");
        return -1;
    }
    if (d.type != TOML_STRING) {
        cfg_error_at(ctx, d.lineno, p, "must be a string (got %s)", datum_desc(&d));
        return -1;
    }
    *name = d.u.s;
    if (line_out) {
        *line_out = d.lineno;
    }
    return 0;
}

static int parse_shape(ParseCtx *ctx, const toml_datum_t *tab, const char *path, Shape *out)
{
    memset(out, 0, sizeof *out);
    if (tab->type != TOML_TABLE) {
        cfg_error_at(ctx, tab->lineno, path, "must be a table (got %s)", datum_desc(tab));
        return -1;
    }

    const char *type_name = NULL;
    int type_line = 0;
    if (get_type_key(ctx, tab, path, &type_name, &type_line) != 0) {
        return -1;
    }
    ShapeType type;
    if (shape_type_from_name(type_name, &type) != 0) {
        char p[CFG_PATH_MAX];
        path_join(p, sizeof p, path, "type");
        cfg_error_at(ctx, type_line, p,
                     "unknown shape type \"%s\" (valid: point, segment, rect, circle, polygon)",
                     type_name);
        return -1;
    }
    if (check_keys(ctx, tab, path, shape_allowed_keys(type)) != 0) {
        return -1;
    }
    out->type = type;

    switch (type) {
    case SHAPE_POINT:
        if (require_field(ctx, tab, path, "position",
                          get_vec2(tab, ctx, path, "position", &out->u.point.position)) != 0) {
            goto fail;
        }
        break;
    case SHAPE_SEGMENT:
        if (require_field(ctx, tab, path, "start",
                          get_vec2(tab, ctx, path, "start", &out->u.segment.start)) != 0) {
            goto fail;
        }
        if (require_field(ctx, tab, path, "end",
                          get_vec2(tab, ctx, path, "end", &out->u.segment.end)) != 0) {
            goto fail;
        }
        break;
    case SHAPE_RECT:
        if (require_field(ctx, tab, path, "min",
                          get_vec2(tab, ctx, path, "min", &out->u.rect.min)) != 0) {
            goto fail;
        }
        if (require_field(ctx, tab, path, "max",
                          get_vec2(tab, ctx, path, "max", &out->u.rect.max)) != 0) {
            goto fail;
        }
        break;
    case SHAPE_CIRCLE:
        if (require_field(ctx, tab, path, "center",
                          get_vec2(tab, ctx, path, "center", &out->u.circle.center)) != 0) {
            goto fail;
        }
        if (require_field(ctx, tab, path, "radius",
                          get_double(tab, ctx, path, "radius", &out->u.circle.radius, NULL)) != 0) {
            goto fail;
        }
        break;
    case SHAPE_POLYGON: {
        Vec2 *vertices = NULL;
        int n = 0;
        int vertices_line = 0;
        int r = get_vec2_list(tab, ctx, path, "vertices", &vertices, &n, &vertices_line);
        if (r == FIELD_INVALID) {
            goto fail;
        }
        if (require_field(ctx, tab, path, "vertices", r) != 0) {
            goto fail;
        }
        if (n < 3) {
            char p[CFG_PATH_MAX];
            path_join(p, sizeof p, path, "vertices");
            cfg_error_at(ctx, vertices_line, p, "must have at least 3 vertices (got %d)", n);
            free(vertices);
            goto fail;
        }
        out->u.polygon.vertices = vertices;
        out->u.polygon.n = n;
        break;
    }
    case SHAPE_TYPE_COUNT:
        break;
    }

    if (shape_validate(out, ctx->err) != 0) {
        cfg_locate_error(ctx, tab->lineno, path);
        goto fail;
    }
    return 0;

fail:
    shape_free(out);
    return -1;
}

/* ------------------------------------------------------------------ */
/* signals                                                            */
/* ------------------------------------------------------------------ */

static const char *const k_signal_keys_harmonic[] = {
    "type", "frequency", "amplitude", "phase", "ramp_cycles", NULL,
};

static int parse_signal(ParseCtx *ctx, const toml_datum_t *tab, const char *path, Signal *out)
{
    memset(out, 0, sizeof *out);
    if (tab->type != TOML_TABLE) {
        cfg_error_at(ctx, tab->lineno, path, "must be a table (got %s)", datum_desc(tab));
        return -1;
    }

    const char *type_name = NULL;
    int type_line = 0;
    if (get_type_key(ctx, tab, path, &type_name, &type_line) != 0) {
        return -1;
    }
    SignalType type;
    if (signal_type_from_name(type_name, &type) != 0) {
        char p[CFG_PATH_MAX];
        path_join(p, sizeof p, path, "type");
        cfg_error_at(ctx, type_line, p, "unknown signal type \"%s\" (valid: harmonic)", type_name);
        return -1;
    }
    if (check_keys(ctx, tab, path, k_signal_keys_harmonic) != 0) {
        return -1;
    }
    out->type = type;

    switch (type) {
    case SIGNAL_HARMONIC: {
        HarmonicSignal *h = &out->u.harmonic;
        if (require_field(ctx, tab, path, "frequency",
                          get_double_positive(tab, ctx, path, "frequency", &h->frequency)) != 0) {
            return -1;
        }
        if (require_field(ctx, tab, path, "amplitude",
                          get_double(tab, ctx, path, "amplitude", &h->amplitude, NULL)) != 0) {
            return -1;
        }
        h->phase = 0.0;
        int r = get_double(tab, ctx, path, "phase", &h->phase, NULL);
        if (r == FIELD_INVALID) {
            return -1;
        }
        h->ramp_cycles = CONFIG_DEFAULT_RAMP_CYCLES;
        r = get_double(tab, ctx, path, "ramp_cycles", &h->ramp_cycles, NULL);
        if (r == FIELD_INVALID) {
            return -1;
        }
        break;
    }
    case SIGNAL_TYPE_COUNT:
        break;
    }

    if (signal_validate(out, ctx->err) != 0) {
        cfg_locate_error(ctx, tab->lineno, path);
        return -1;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* top level sections                                                 */
/* ------------------------------------------------------------------ */

static const char *const k_top_keys[] = {
    "simulation", "grid", "medium", "boundary", "obstacle", "source", "output", NULL,
};
static const char *const k_simulation_keys[] = {"duration", "cfl", "dt", NULL};
static const char *const k_grid_keys[] = {"x_range", "y_range", "dx", "dy", NULL};
static const char *const k_medium_keys[] = {"density", "c", NULL};
static const char *const k_boundary_keys[] = {"x_min", "x_max", "y_min", "y_max", "pml", NULL};
static const char *const k_pml_keys[] = {"thickness", "reflection", "order", NULL};
static const char *const k_obstacle_keys[] = {"shape", NULL};
static const char *const k_source_keys[] = {"shape", "signal", NULL};
static const char *const k_output_keys[] = {
    "directory", "every", "color_limit", "show_pml", "pixel_scale", NULL,
};

/* Look up a section table: 1 = found, 0 = absent (only if !required), -1 = error. */
static int get_section(ParseCtx *ctx, const toml_datum_t *root, const char *name,
                       bool required, toml_datum_t *out)
{
    if (!table_lookup(root, name, out)) {
        if (required) {
            cfg_error_at(ctx, 0, name, "missing required table");
            return -1;
        }
        return 0;
    }
    if (out->type != TOML_TABLE) {
        cfg_error_at(ctx, out->lineno, name, "must be a table (got %s)", datum_desc(out));
        return -1;
    }
    return 1;
}

static int parse_simulation(ParseCtx *ctx, const toml_datum_t *tab, SimConfig *cfg)
{
    const char *path = "simulation";
    if (check_keys(ctx, tab, path, k_simulation_keys) != 0) {
        return -1;
    }
    if (require_field(ctx, tab, path, "duration",
                      get_double_positive(tab, ctx, path, "duration", &cfg->duration)) != 0) {
        return -1;
    }

    bool cfl_given = false;
    int cfl_line = 0;
    int r = get_double(tab, ctx, path, "cfl", &cfg->cfl, &cfl_line);
    if (r == FIELD_INVALID) {
        return -1;
    }
    if (r == FIELD_FOUND) {
        cfl_given = true;
        if (!(cfg->cfl > 0.0 && cfg->cfl <= 1.0)) {
            char p[CFG_PATH_MAX];
            path_join(p, sizeof p, path, "cfl");
            cfg_error_at(ctx, cfl_line, p, "must satisfy 0 < cfl <= 1 (got %g)", cfg->cfl);
            return -1;
        }
    }

    int dt_line = 0;
    r = get_double(tab, ctx, path, "dt", &cfg->dt, &dt_line);
    if (r == FIELD_INVALID) {
        return -1;
    }
    if (r == FIELD_FOUND) {
        char p[CFG_PATH_MAX];
        path_join(p, sizeof p, path, "dt");
        if (!(cfg->dt > 0.0)) {
            cfg_error_at(ctx, dt_line, p, "must be > 0 (got %g)", cfg->dt);
            return -1;
        }
        cfg->dt_set = true;
        if (cfl_given) {
            cfg_error_at(ctx, dt_line, p,
                         "cfl and dt are mutually exclusive; set only one of them");
            return -1;
        }
    }
    return 0;
}

static int parse_grid(ParseCtx *ctx, const toml_datum_t *tab, SimConfig *cfg)
{
    const char *path = "grid";
    if (check_keys(ctx, tab, path, k_grid_keys) != 0) {
        return -1;
    }
    if (require_field(ctx, tab, path, "x_range",
                      get_number_pair(tab, ctx, path, "x_range", cfg->x_range, true, NULL)) != 0) {
        return -1;
    }
    if (require_field(ctx, tab, path, "y_range",
                      get_number_pair(tab, ctx, path, "y_range", cfg->y_range, true, NULL)) != 0) {
        return -1;
    }
    if (require_field(ctx, tab, path, "dx",
                      get_double_positive(tab, ctx, path, "dx", &cfg->dx)) != 0) {
        return -1;
    }
    int r = get_double_positive(tab, ctx, path, "dy", &cfg->dy);
    if (r == FIELD_INVALID) {
        return -1;
    }
    if (r == FIELD_MISSING) {
        cfg->dy = cfg->dx;
    }
    return 0;
}

static int parse_medium(ParseCtx *ctx, const toml_datum_t *tab, SimConfig *cfg)
{
    const char *path = "medium";
    if (check_keys(ctx, tab, path, k_medium_keys) != 0) {
        return -1;
    }
    if (require_field(ctx, tab, path, "density",
                      get_double(tab, ctx, path, "density", &cfg->medium.density, NULL)) != 0) {
        return -1;
    }
    if (require_field(ctx, tab, path, "c",
                      get_double(tab, ctx, path, "c", &cfg->medium.c, NULL)) != 0) {
        return -1;
    }
    if (medium_validate(&cfg->medium, ctx->err) != 0) {
        cfg_locate_error(ctx, tab->lineno, path);
        return -1;
    }
    return 0;
}

static int parse_boundary(ParseCtx *ctx, const toml_datum_t *tab, SimConfig *cfg)
{
    const char *path = "boundary";
    if (check_keys(ctx, tab, path, k_boundary_keys) != 0) {
        return -1;
    }
    for (int side = 0; side < SIDE_COUNT; side++) {
        const char *name = side_name((Side)side);
        const char *value = NULL;
        int r = get_string(tab, ctx, path, name, &value);
        if (r == FIELD_INVALID) {
            return -1;
        }
        if (r == FIELD_MISSING) {
            continue;
        }
        BoundaryType bc;
        if (boundary_type_from_name(value, &bc) != 0) {
            char p[CFG_PATH_MAX];
            path_join(p, sizeof p, path, name);
            cfg_error_at(ctx, lookup_line(tab, name), p,
                         "unknown boundary type \"%s\" (valid: rigid, pressure_release, pml)", value);
            return -1;
        }
        cfg->boundary[side] = bc;
    }

    toml_datum_t pml;
    if (table_lookup(tab, "pml", &pml)) {
        char ppath[CFG_PATH_MAX];
        path_join(ppath, sizeof ppath, path, "pml");
        if (pml.type != TOML_TABLE) {
            cfg_error_at(ctx, pml.lineno, ppath, "must be a table (got %s)", datum_desc(&pml));
            return -1;
        }
        if (check_keys(ctx, &pml, ppath, k_pml_keys) != 0) {
            return -1;
        }
        int r = get_int(&pml, ctx, ppath, "thickness", &cfg->pml.thickness, NULL);
        if (r == FIELD_INVALID) {
            return -1;
        }
        r = get_double(&pml, ctx, ppath, "reflection", &cfg->pml.reflection, NULL);
        if (r == FIELD_INVALID) {
            return -1;
        }
        r = get_double(&pml, ctx, ppath, "order", &cfg->pml.order, NULL);
        if (r == FIELD_INVALID) {
            return -1;
        }
        if (pml_params_validate(&cfg->pml, ctx->err) != 0) {
            cfg_locate_error(ctx, pml.lineno, ppath);
            return -1;
        }
    }
    return 0;
}

static int parse_output(ParseCtx *ctx, const toml_datum_t *tab, SimConfig *cfg)
{
    const char *path = "output";
    if (check_keys(ctx, tab, path, k_output_keys) != 0) {
        return -1;
    }

    const char *dir = NULL;
    int r = get_string(tab, ctx, path, "directory", &dir);
    if (r == FIELD_INVALID) {
        return -1;
    }
    if (r == FIELD_FOUND) {
        if (dir[0] == '\0') {
            char p[CFG_PATH_MAX];
            path_join(p, sizeof p, path, "directory");
            cfg_error_at(ctx, lookup_line(tab, "directory"), p, "must not be empty");
            return -1;
        }
        char *copy = fdtd_strdup(dir, ctx->err);
        if (!copy) {
            return -1;
        }
        free(cfg->output.directory);
        cfg->output.directory = copy;
    }

    r = get_int_range(tab, ctx, path, "every", 1, INT_MAX, &cfg->output.every);
    if (r == FIELD_INVALID) {
        return -1;
    }

    toml_datum_t cl;
    if (table_lookup(tab, "color_limit", &cl)) {
        char p[CFG_PATH_MAX];
        path_join(p, sizeof p, path, "color_limit");
        if (cl.type == TOML_STRING) {
            if (strcmp(cl.u.s, "auto") != 0) {
                cfg_error_at(ctx, cl.lineno, p, "must be \"auto\" or a number > 0 (got \"%s\")",
                             cl.u.s);
                return -1;
            }
            cfg->output.color_auto = true;
            cfg->output.color_limit = 0.0;
        } else {
            double v = 0.0;
            if (!datum_as_double(&cl, &v)) {
                cfg_error_at(ctx, cl.lineno, p, "must be \"auto\" or a number > 0 (got %s)",
                             datum_desc(&cl));
                return -1;
            }
            if (!(v > 0.0)) {
                cfg_error_at(ctx, cl.lineno, p, "must be > 0 (got %g)", v);
                return -1;
            }
            cfg->output.color_auto = false;
            cfg->output.color_limit = v;
        }
    }

    r = get_bool(tab, ctx, path, "show_pml", &cfg->output.show_pml);
    if (r == FIELD_INVALID) {
        return -1;
    }
    r = get_int_range(tab, ctx, path, "pixel_scale", 1, CONFIG_MAX_PIXEL_SCALE,
                      &cfg->output.pixel_scale);
    if (r == FIELD_INVALID) {
        return -1;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* [[obstacle]] and [[source]]                                        */
/* ------------------------------------------------------------------ */

/* Fetch the `shape` table of an obstacle/source entry and parse it. */
static int parse_entry_shape(ParseCtx *ctx, const toml_datum_t *entry, const char *entry_path,
                             Shape *shape, int *shape_line)
{
    char spath[CFG_PATH_MAX];
    path_join(spath, sizeof spath, entry_path, "shape");
    toml_datum_t d;
    if (!table_lookup(entry, "shape", &d)) {
        cfg_error_at(ctx, entry->lineno, spath, "missing required key");
        return -1;
    }
    if (shape_line) {
        *shape_line = d.lineno;
    }
    return parse_shape(ctx, &d, spath, shape);
}

static int parse_obstacles(ParseCtx *ctx, const toml_datum_t *arr, const char *name,
                           SimConfig *cfg)
{
    if (arr->type != TOML_ARRAY) {
        cfg_error_at(ctx, arr->lineno, name, "must be an array of tables (got %s)",
                     datum_desc(arr));
        return -1;
    }
    int n = (int)arr->u.arr.size;
    if (n == 0) {
        return 0;
    }
    cfg->obstacles = fdtd_calloc((size_t)n, sizeof *cfg->obstacles, ctx->err);
    if (!cfg->obstacles) {
        return -1;
    }
    for (int i = 0; i < n; i++) {
        const toml_datum_t *entry = &arr->u.arr.elem[i];
        char epath[CFG_PATH_MAX];
        path_index(epath, sizeof epath, name, i);
        if (entry->type != TOML_TABLE) {
            cfg_error_at(ctx, entry->lineno, epath, "must be a table (got %s)",
                         datum_desc(entry));
            return -1;
        }
        if (check_keys(ctx, entry, epath, k_obstacle_keys) != 0) {
            return -1;
        }
        Shape *shape = &cfg->obstacles[i].shape;
        cfg->n_obstacles = i + 1; /* counted before parsing, see parse_sources */
        int shape_line = 0;
        if (parse_entry_shape(ctx, entry, epath, shape, &shape_line) != 0) {
            return -1;
        }
        if (!shape_is_area(shape)) {
            char spath[CFG_PATH_MAX];
            path_join(spath, sizeof spath, epath, "shape");
            cfg_error_at(ctx, shape_line, spath,
                         "thin obstacles (%s) are not supported yet; use a rect, circle or polygon",
                         shape_type_name(shape->type));
            shape_free(shape);
            return -1;
        }
    }
    return 0;
}

static int parse_sources(ParseCtx *ctx, const toml_datum_t *arr, const char *name, SimConfig *cfg)
{
    if (arr->type != TOML_ARRAY) {
        cfg_error_at(ctx, arr->lineno, name, "must be an array of tables (got %s)",
                     datum_desc(arr));
        return -1;
    }
    int n = (int)arr->u.arr.size;
    if (n == 0) {
        return 0;
    }
    cfg->sources = fdtd_calloc((size_t)n, sizeof *cfg->sources, ctx->err);
    if (!cfg->sources) {
        return -1;
    }
    for (int i = 0; i < n; i++) {
        const toml_datum_t *entry = &arr->u.arr.elem[i];
        char epath[CFG_PATH_MAX];
        path_index(epath, sizeof epath, name, i);
        if (entry->type != TOML_TABLE) {
            cfg_error_at(ctx, entry->lineno, epath, "must be a table (got %s)",
                         datum_desc(entry));
            return -1;
        }
        if (check_keys(ctx, entry, epath, k_source_keys) != 0) {
            return -1;
        }
        SourceConfig *src = &cfg->sources[i];
        /* Count the entry before parsing it so config_free() releases a
         * shape that parsed fine when the signal fails (shapes are zeroed by
         * calloc, so freeing an unparsed entry is a no-op). */
        cfg->n_sources = i + 1;
        int shape_line = 0;
        if (parse_entry_shape(ctx, entry, epath, &src->shape, &shape_line) != 0) {
            return -1;
        }
        if (src->shape.type != SHAPE_POINT && src->shape.type != SHAPE_SEGMENT) {
            char spath[CFG_PATH_MAX];
            path_join(spath, sizeof spath, epath, "shape");
            cfg_error_at(ctx, shape_line, spath,
                         "%s source shapes are not supported in this version; use a point or "
                         "a segment",
                         shape_type_name(src->shape.type));
            shape_free(&src->shape);
            return -1;
        }

        char gpath[CFG_PATH_MAX];
        path_join(gpath, sizeof gpath, epath, "signal");
        toml_datum_t sd;
        if (!table_lookup(entry, "signal", &sd)) {
            cfg_error_at(ctx, entry->lineno, gpath, "missing required key");
            return -1;
        }
        if (parse_signal(ctx, &sd, gpath, &src->signal) != 0) {
            return -1;
        }
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* document                                                           */
/* ------------------------------------------------------------------ */

static int config_set_defaults(SimConfig *cfg, ParseCtx *ctx)
{
    cfg->cfl = CONFIG_DEFAULT_CFL;
    cfg->dt = 0.0;
    cfg->dt_set = false;
    for (int side = 0; side < SIDE_COUNT; side++) {
        cfg->boundary[side] = BC_PML;
    }
    cfg->pml.thickness = PML_DEFAULT_THICKNESS;
    cfg->pml.reflection = PML_DEFAULT_REFLECTION;
    cfg->pml.order = PML_DEFAULT_ORDER;
    cfg->output.every = CONFIG_DEFAULT_OUTPUT_EVERY;
    cfg->output.color_auto = true;
    cfg->output.color_limit = 0.0;
    cfg->output.show_pml = false;
    cfg->output.pixel_scale = CONFIG_DEFAULT_PIXEL_SCALE;
    cfg->output.directory = fdtd_strdup(CONFIG_DEFAULT_OUTPUT_DIR, ctx->err);
    if (!cfg->output.directory) {
        return -1;
    }
    cfg->source_name = fdtd_strdup(ctx->source_name, ctx->err);
    if (!cfg->source_name) {
        return -1;
    }
    return 0;
}

static int parse_document(ParseCtx *ctx, const toml_datum_t *root, SimConfig *cfg)
{
    if (check_keys(ctx, root, "", k_top_keys) != 0) {
        return -1;
    }

    toml_datum_t tab;
    int r = get_section(ctx, root, "simulation", true, &tab);
    if (r < 0) {
        return -1;
    }
    if (parse_simulation(ctx, &tab, cfg) != 0) {
        return -1;
    }
    r = get_section(ctx, root, "grid", true, &tab);
    if (r < 0) {
        return -1;
    }
    if (parse_grid(ctx, &tab, cfg) != 0) {
        return -1;
    }
    r = get_section(ctx, root, "medium", true, &tab);
    if (r < 0) {
        return -1;
    }
    if (parse_medium(ctx, &tab, cfg) != 0) {
        return -1;
    }
    r = get_section(ctx, root, "boundary", false, &tab);
    if (r < 0) {
        return -1;
    }
    if (r > 0 && parse_boundary(ctx, &tab, cfg) != 0) {
        return -1;
    }

    toml_datum_t arr;
    if (table_lookup(root, "obstacle", &arr)) {
        if (parse_obstacles(ctx, &arr, "obstacle", cfg) != 0) {
            return -1;
        }
    }
    if (table_lookup(root, "source", &arr)) {
        if (parse_sources(ctx, &arr, "source", cfg) != 0) {
            return -1;
        }
    }

    r = get_section(ctx, root, "output", false, &tab);
    if (r < 0) {
        return -1;
    }
    if (r > 0 && parse_output(ctx, &tab, cfg) != 0) {
        return -1;
    }
    return 0;
}

/* Take ownership of a parse result: fill defaults, parse, free the tree. */
static int config_from_result(toml_result_t res, const char *source_name, SimConfig *cfg,
                              FdtdError *err)
{
    ParseCtx ctx = {source_name, err};
    memset(cfg, 0, sizeof *cfg);
    if (!res.ok) {
        fdtd_error_set(err, "%s: %s", source_name, res.errmsg);
        toml_free(res);
        return -1;
    }
    if (config_set_defaults(cfg, &ctx) != 0) {
        config_free(cfg);
        toml_free(res);
        return -1;
    }
    int rc = parse_document(&ctx, &res.toptab, cfg);
    toml_free(res);
    if (rc != 0) {
        config_free(cfg);
        return -1;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* public API                                                         */
/* ------------------------------------------------------------------ */

int config_load_string(const char *text, const char *source_name, SimConfig *cfg, FdtdError *err)
{
    if (!cfg) {
        fdtd_error_set(err, "config_load_string: cfg is NULL");
        return -1;
    }
    memset(cfg, 0, sizeof *cfg);
    const char *name = source_name ? source_name : "<string>";
    if (!text) {
        fdtd_error_set(err, "%s: null document", name);
        return -1;
    }
    size_t len = strlen(text);
    if (len > (size_t)INT_MAX) {
        fdtd_error_set(err, "%s: document too large", name);
        return -1;
    }
    return config_from_result(toml_parse_named(text, (int)len, name), name, cfg, err);
}

int config_load_file(const char *path, SimConfig *cfg, FdtdError *err)
{
    if (!cfg) {
        fdtd_error_set(err, "config_load_file: cfg is NULL");
        return -1;
    }
    memset(cfg, 0, sizeof *cfg);
    if (!path) {
        fdtd_error_set(err, "config_load_file: path is NULL");
        return -1;
    }
    FILE *fp = fopen(path, "rb");
    if (!fp) {
        fdtd_error_set(err, "%s: cannot open file: %s", path, strerror(errno));
        return -1;
    }
    toml_result_t res = toml_parse_file_named(fp, path);
    fclose(fp);
    return config_from_result(res, path, cfg, err);
}

void config_free(SimConfig *cfg)
{
    if (!cfg) {
        return;
    }
    int n_obstacles = cfg->n_obstacles > 0 ? cfg->n_obstacles : 0;
    int n_sources = cfg->n_sources > 0 ? cfg->n_sources : 0;
    for (int i = 0; i < n_obstacles; i++) {
        shape_free(&cfg->obstacles[i].shape);
    }
    for (int i = 0; i < n_sources; i++) {
        shape_free(&cfg->sources[i].shape);
    }
    free(cfg->obstacles);
    free(cfg->sources);
    free(cfg->output.directory);
    free(cfg->source_name);
    memset(cfg, 0, sizeof *cfg);
}
