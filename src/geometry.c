#include "geometry.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static const char *const k_shape_names[SHAPE_TYPE_COUNT] = {
    [SHAPE_POINT] = "point",
    [SHAPE_SEGMENT] = "segment",
    [SHAPE_RECT] = "rect",
    [SHAPE_CIRCLE] = "circle",
    [SHAPE_POLYGON] = "polygon",
};

const char *shape_type_name(ShapeType type)
{
    return k_shape_names[type];
}

int shape_type_from_name(const char *name, ShapeType *out)
{
    for (int t = 0; t < SHAPE_TYPE_COUNT; t++) {
        if (strcmp(name, k_shape_names[t]) == 0) {
            *out = (ShapeType)t;
            return 0;
        }
    }
    return -1;
}

bool shape_is_area(const Shape *s)
{
    return s->type == SHAPE_RECT || s->type == SHAPE_CIRCLE || s->type == SHAPE_POLYGON;
}

static bool polygon_contains(const Vec2 *v, int n, Vec2 p)
{
    /* Even-odd rule, W. Randolph Franklin's crossing test. */
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        if ((v[i].y > p.y) != (v[j].y > p.y)) {
            double x_cross = (v[j].x - v[i].x) * (p.y - v[i].y) / (v[j].y - v[i].y) + v[i].x;
            if (p.x < x_cross) {
                inside = !inside;
            }
        }
    }
    return inside;
}

bool shape_contains(const Shape *s, Vec2 p)
{
    switch (s->type) {
    case SHAPE_RECT:
        return p.x >= s->u.rect.min.x && p.x <= s->u.rect.max.x &&
               p.y >= s->u.rect.min.y && p.y <= s->u.rect.max.y;
    case SHAPE_CIRCLE: {
        double dx = p.x - s->u.circle.center.x;
        double dy = p.y - s->u.circle.center.y;
        return dx * dx + dy * dy <= s->u.circle.radius * s->u.circle.radius;
    }
    case SHAPE_POLYGON:
        return polygon_contains(s->u.polygon.vertices, s->u.polygon.n, p);
    case SHAPE_POINT:
    case SHAPE_SEGMENT:
    case SHAPE_TYPE_COUNT:
        break;
    }
    return false;
}

void shape_bbox(const Shape *s, Vec2 *min, Vec2 *max)
{
    switch (s->type) {
    case SHAPE_POINT:
        *min = *max = s->u.point.position;
        return;
    case SHAPE_SEGMENT:
        min->x = fmin(s->u.segment.start.x, s->u.segment.end.x);
        min->y = fmin(s->u.segment.start.y, s->u.segment.end.y);
        max->x = fmax(s->u.segment.start.x, s->u.segment.end.x);
        max->y = fmax(s->u.segment.start.y, s->u.segment.end.y);
        return;
    case SHAPE_RECT:
        *min = s->u.rect.min;
        *max = s->u.rect.max;
        return;
    case SHAPE_CIRCLE:
        min->x = s->u.circle.center.x - s->u.circle.radius;
        min->y = s->u.circle.center.y - s->u.circle.radius;
        max->x = s->u.circle.center.x + s->u.circle.radius;
        max->y = s->u.circle.center.y + s->u.circle.radius;
        return;
    case SHAPE_POLYGON: {
        const Vec2 *v = s->u.polygon.vertices;
        *min = *max = v[0];
        for (int i = 1; i < s->u.polygon.n; i++) {
            min->x = fmin(min->x, v[i].x);
            min->y = fmin(min->y, v[i].y);
            max->x = fmax(max->x, v[i].x);
            max->y = fmax(max->y, v[i].y);
        }
        return;
    }
    case SHAPE_TYPE_COUNT:
        break;
    }
    min->x = min->y = max->x = max->y = 0.0;
}

static bool vec_finite(Vec2 v)
{
    return isfinite(v.x) && isfinite(v.y);
}

int shape_validate(const Shape *s, FdtdError *err)
{
    switch (s->type) {
    case SHAPE_POINT:
        if (!vec_finite(s->u.point.position)) {
            fdtd_error_set(err, "point position must be finite");
            return -1;
        }
        return 0;
    case SHAPE_SEGMENT: {
        Vec2 a = s->u.segment.start, b = s->u.segment.end;
        if (!vec_finite(a) || !vec_finite(b)) {
            fdtd_error_set(err, "segment end points must be finite");
            return -1;
        }
        if (hypot(b.x - a.x, b.y - a.y) <= 0.0) {
            fdtd_error_set(err, "segment has zero length");
            return -1;
        }
        return 0;
    }
    case SHAPE_RECT:
        if (!vec_finite(s->u.rect.min) || !vec_finite(s->u.rect.max)) {
            fdtd_error_set(err, "rect corners must be finite");
            return -1;
        }
        if (!(s->u.rect.max.x > s->u.rect.min.x) || !(s->u.rect.max.y > s->u.rect.min.y)) {
            fdtd_error_set(err, "rect requires max > min on both axes (got min=[%g, %g], max=[%g, %g])",
                           s->u.rect.min.x, s->u.rect.min.y, s->u.rect.max.x, s->u.rect.max.y);
            return -1;
        }
        return 0;
    case SHAPE_CIRCLE:
        if (!vec_finite(s->u.circle.center) || !isfinite(s->u.circle.radius)) {
            fdtd_error_set(err, "circle center and radius must be finite");
            return -1;
        }
        if (!(s->u.circle.radius > 0.0)) {
            fdtd_error_set(err, "circle radius must be > 0 (got %g)", s->u.circle.radius);
            return -1;
        }
        return 0;
    case SHAPE_POLYGON: {
        int n = s->u.polygon.n;
        const Vec2 *v = s->u.polygon.vertices;
        if (n < 3 || !v) {
            fdtd_error_set(err, "polygon needs at least 3 vertices (got %d)", n);
            return -1;
        }
        double area2 = 0.0;
        for (int i = 0, j = n - 1; i < n; j = i++) {
            if (!vec_finite(v[i])) {
                fdtd_error_set(err, "polygon vertex %d must be finite", i);
                return -1;
            }
            area2 += v[j].x * v[i].y - v[i].x * v[j].y;
        }
        if (area2 == 0.0) {
            fdtd_error_set(err, "polygon has zero area");
            return -1;
        }
        return 0;
    }
    case SHAPE_TYPE_COUNT:
        break;
    }
    fdtd_error_set(err, "unknown shape type %d", (int)s->type);
    return -1;
}

int shape_copy(Shape *dst, const Shape *src, FdtdError *err)
{
    *dst = *src;
    if (src->type == SHAPE_POLYGON) {
        dst->u.polygon.vertices = NULL;
        Vec2 *v = fdtd_calloc((size_t)src->u.polygon.n, sizeof *v, err);
        if (!v) {
            dst->u.polygon.n = 0;
            return -1;
        }
        memcpy(v, src->u.polygon.vertices, (size_t)src->u.polygon.n * sizeof *v);
        dst->u.polygon.vertices = v;
    }
    return 0;
}

void shape_free(Shape *s)
{
    if (!s) {
        return;
    }
    if (s->type == SHAPE_POLYGON) {
        free(s->u.polygon.vertices);
    }
    memset(s, 0, sizeof *s); /* a zeroed Shape is an empty point (SHAPE_POINT == 0) */
}

/* Liang-Barsky: clip parameter range [t0,t1] of a + t (b - a). */
static bool clip_params(Vec2 a, Vec2 b, Vec2 bmin, Vec2 bmax, double *t0_out, double *t1_out)
{
    double t0 = 0.0, t1 = 1.0;
    double d[2] = {b.x - a.x, b.y - a.y};
    double lo[2] = {bmin.x - a.x, bmin.y - a.y}; /* need d*t >= lo */
    double hi[2] = {bmax.x - a.x, bmax.y - a.y}; /* need d*t <= hi */
    for (int k = 0; k < 2; k++) {
        if (d[k] == 0.0) {
            if (lo[k] > 0.0 || hi[k] < 0.0) {
                return false;
            }
            continue;
        }
        double ta = lo[k] / d[k];
        double tb = hi[k] / d[k];
        if (ta > tb) {
            double tmp = ta;
            ta = tb;
            tb = tmp;
        }
        if (ta > t0) {
            t0 = ta;
        }
        if (tb < t1) {
            t1 = tb;
        }
        if (t0 > t1) {
            return false;
        }
    }
    *t0_out = t0;
    *t1_out = t1;
    return true;
}

double geom_segment_length_in_box(Vec2 a, Vec2 b, Vec2 bmin, Vec2 bmax)
{
    double t0, t1;
    if (!clip_params(a, b, bmin, bmax, &t0, &t1)) {
        return 0.0;
    }
    return (t1 - t0) * hypot(b.x - a.x, b.y - a.y);
}

bool geom_clip_segment(Vec2 *a, Vec2 *b, Vec2 bmin, Vec2 bmax)
{
    double t0, t1;
    if (!clip_params(*a, *b, bmin, bmax, &t0, &t1)) {
        return false;
    }
    Vec2 d = {b->x - a->x, b->y - a->y};
    Vec2 na = {a->x + t0 * d.x, a->y + t0 * d.y};
    Vec2 nb = {a->x + t1 * d.x, a->y + t1 * d.y};
    *a = na;
    *b = nb;
    return true;
}
