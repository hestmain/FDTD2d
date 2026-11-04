/*
 * test_geometry.c - unit tests for the grid-independent shape helpers:
 * point-in-shape, bounding boxes, validation, deep copies, name/type
 * round trips and the Liang-Barsky segment clipping/measuring routines.
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "geometry.h"

#include "test_util.h"

static Shape rect_shape(double x0, double y0, double x1, double y1)
{
    Shape s;
    memset(&s, 0, sizeof s);
    s.type = SHAPE_RECT;
    s.u.rect.min.x = x0;
    s.u.rect.min.y = y0;
    s.u.rect.max.x = x1;
    s.u.rect.max.y = y1;
    return s;
}

static Shape circle_shape(double cx, double cy, double r)
{
    Shape s;
    memset(&s, 0, sizeof s);
    s.type = SHAPE_CIRCLE;
    s.u.circle.center.x = cx;
    s.u.circle.center.y = cy;
    s.u.circle.radius = r;
    return s;
}

/* Polygon over caller owned vertex storage (no copy is made). */
static Shape polygon_shape(Vec2 *vertices, int n)
{
    Shape s;
    memset(&s, 0, sizeof s);
    s.type = SHAPE_POLYGON;
    s.u.polygon.vertices = vertices;
    s.u.polygon.n = n;
    return s;
}

static Shape point_shape(double x, double y)
{
    Shape s;
    memset(&s, 0, sizeof s);
    s.type = SHAPE_POINT;
    s.u.point.position.x = x;
    s.u.point.position.y = y;
    return s;
}

static Shape segment_shape(double x0, double y0, double x1, double y1)
{
    Shape s;
    memset(&s, 0, sizeof s);
    s.type = SHAPE_SEGMENT;
    s.u.segment.start.x = x0;
    s.u.segment.start.y = y0;
    s.u.segment.end.x = x1;
    s.u.segment.end.y = y1;
    return s;
}

static Vec2 vec(double x, double y)
{
    Vec2 v;
    v.x = x;
    v.y = y;
    return v;
}

/* ------------------------------------------------------------------ names */

static void test_name_type_conversions(void)
{
    CHECK(strcmp(shape_type_name(SHAPE_POINT), "point") == 0);
    CHECK(strcmp(shape_type_name(SHAPE_SEGMENT), "segment") == 0);
    CHECK(strcmp(shape_type_name(SHAPE_RECT), "rect") == 0);
    CHECK(strcmp(shape_type_name(SHAPE_CIRCLE), "circle") == 0);
    CHECK(strcmp(shape_type_name(SHAPE_POLYGON), "polygon") == 0);
    CHECK(strcmp(shape_type_name((ShapeType)SHAPE_TYPE_COUNT), "unknown") == 0);
    CHECK(strcmp(shape_type_name((ShapeType)-1), "unknown") == 0);

    for (int t = 0; t < SHAPE_TYPE_COUNT; t++) {
        const char *name = shape_type_name((ShapeType)t);
        ShapeType back = SHAPE_TYPE_COUNT;
        CHECK(name != NULL && name[0] != '\0');
        CHECK(shape_type_from_name(name, &back) == 0);
        CHECK(back == (ShapeType)t);
    }

    ShapeType out = SHAPE_TYPE_COUNT;
    CHECK(shape_type_from_name("hexagon", &out) == -1);
    CHECK(shape_type_from_name("Point", &out) == -1); /* case sensitive */
    CHECK(shape_type_from_name("", &out) == -1);
    CHECK(shape_type_from_name(NULL, &out) == -1);
}

static void test_is_area(void)
{
    Vec2 square[4] = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
    Shape p = point_shape(0.5, 0.5);
    Shape seg = segment_shape(0.0, 0.0, 1.0, 1.0);
    Shape rec = rect_shape(0.0, 0.0, 1.0, 1.0);
    Shape cir = circle_shape(0.5, 0.5, 0.4);
    Shape pol = polygon_shape(square, 4);

    CHECK(!shape_is_area(&p));
    CHECK(!shape_is_area(&seg));
    CHECK(shape_is_area(&rec));
    CHECK(shape_is_area(&cir));
    CHECK(shape_is_area(&pol));
}

/* --------------------------------------------------------------- contains */

static void test_contains_rect(void)
{
    Shape s = rect_shape(0.0, 0.0, 2.0, 1.0);

    CHECK(shape_contains(&s, vec(1.0, 0.5)));
    CHECK(!shape_contains(&s, vec(-0.01, 0.5)));
    CHECK(!shape_contains(&s, vec(2.01, 0.5)));
    CHECK(!shape_contains(&s, vec(1.0, -0.01)));
    CHECK(!shape_contains(&s, vec(1.0, 1.01)));
    /* boundary counts as inside for rect */
    CHECK(shape_contains(&s, vec(0.0, 0.0)));
    CHECK(shape_contains(&s, vec(2.0, 1.0)));
    CHECK(shape_contains(&s, vec(0.0, 0.5)));
}

static void test_contains_circle(void)
{
    Shape s = circle_shape(1.0, 1.0, 0.5);

    CHECK(shape_contains(&s, vec(1.0, 1.0)));
    CHECK(shape_contains(&s, vec(1.25, 1.0)));
    /* exactly on the circle (boundary counts as inside) */
    CHECK(shape_contains(&s, vec(1.5, 1.0)));
    CHECK(shape_contains(&s, vec(1.0, 0.5)));
    /* 0.4^2 + 0.4^2 = 0.32 > 0.25 */
    CHECK(!shape_contains(&s, vec(1.4, 1.4)));
    CHECK(!shape_contains(&s, vec(0.49, 1.0)));
    CHECK(!shape_contains(&s, vec(1.0, 1.51)));
}

static void test_contains_polygon(void)
{
    /* Convex square, counter clockwise and clockwise. */
    Vec2 ccw[4] = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
    Vec2 cw[4] = {{0.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}, {1.0, 0.0}};
    Shape a = polygon_shape(ccw, 4);
    Shape b = polygon_shape(cw, 4);

    CHECK(shape_contains(&a, vec(0.5, 0.5)));
    CHECK(!shape_contains(&a, vec(1.5, 0.5)));
    CHECK(!shape_contains(&a, vec(-0.5, 0.5)));
    CHECK(!shape_contains(&a, vec(0.5, -0.5)));
    CHECK(!shape_contains(&a, vec(0.5, 1.5)));

    CHECK(shape_contains(&b, vec(0.5, 0.5)));
    CHECK(!shape_contains(&b, vec(1.5, 0.5)));
    CHECK(!shape_contains(&b, vec(-0.5, 0.5)));

    /* Concave L-shape: the notch (1..2, 1..2) is outside. */
    Vec2 ell[6] = {{0.0, 0.0}, {2.0, 0.0}, {2.0, 1.0},
                   {1.0, 1.0}, {1.0, 2.0}, {0.0, 2.0}};
    Vec2 ell_rev[6] = {{0.0, 2.0}, {1.0, 2.0}, {1.0, 1.0},
                       {2.0, 1.0}, {2.0, 0.0}, {0.0, 0.0}};
    Shape c = polygon_shape(ell, 6);
    Shape d = polygon_shape(ell_rev, 6);

    CHECK(shape_contains(&c, vec(0.5, 0.5)));
    CHECK(shape_contains(&c, vec(0.5, 1.5)));
    CHECK(shape_contains(&c, vec(1.5, 0.5)));
    CHECK(!shape_contains(&c, vec(1.5, 1.5))); /* inside the notch */

    CHECK(shape_contains(&d, vec(0.5, 0.5)));
    CHECK(shape_contains(&d, vec(0.5, 1.5)));
    CHECK(!shape_contains(&d, vec(1.5, 1.5)));

    /* Triangle: interior is x >= 0, y >= 0, x + y <= 2. Boundary points of
     * polygons are unspecified by the contract, so only strict interior and
     * clearly external points are checked. */
    Vec2 tri[3] = {{0.0, 0.0}, {2.0, 0.0}, {0.0, 2.0}};
    Shape t = polygon_shape(tri, 3);
    CHECK(shape_contains(&t, vec(0.5, 0.5)));
    CHECK(shape_contains(&t, vec(1.5, 0.1)));
    CHECK(!shape_contains(&t, vec(1.5, 1.5)));
    CHECK(!shape_contains(&t, vec(1.4, 0.7))); /* x + y = 2.1 > 2 */
}

static void test_contains_non_area_is_false(void)
{
    Shape p = point_shape(0.5, 0.5);
    Shape s = segment_shape(0.0, 0.0, 1.0, 1.0);

    CHECK(!shape_contains(&p, vec(0.5, 0.5)));
    CHECK(!shape_contains(&s, vec(0.5, 0.5)));
}

/* ------------------------------------------------------------------- bbox */

static void test_bbox(void)
{
    Vec2 lo, hi;

    Shape p = point_shape(1.0, -2.0);
    shape_bbox(&p, &lo, &hi);
    CHECK(lo.x == 1.0 && lo.y == -2.0);
    CHECK(hi.x == 1.0 && hi.y == -2.0);

    /* start/end given in "wrong" order must still yield min/max. */
    Shape s = segment_shape(3.0, 0.0, 1.0, 2.0);
    shape_bbox(&s, &lo, &hi);
    CHECK(lo.x == 1.0 && lo.y == 0.0);
    CHECK(hi.x == 3.0 && hi.y == 2.0);

    Shape r = rect_shape(-1.0, 0.5, 2.0, 4.0);
    shape_bbox(&r, &lo, &hi);
    CHECK(lo.x == -1.0 && lo.y == 0.5);
    CHECK(hi.x == 2.0 && hi.y == 4.0);

    Shape c = circle_shape(1.0, 1.0, 0.5);
    shape_bbox(&c, &lo, &hi);
    CHECK(lo.x == 0.5 && lo.y == 0.5);
    CHECK(hi.x == 1.5 && hi.y == 1.5);

    Vec2 poly[5] = {{0.0, 0.0}, {2.0, -1.0}, {3.0, 1.0}, {1.0, 4.0}, {-1.0, 2.0}};
    Shape pl = polygon_shape(poly, 5);
    shape_bbox(&pl, &lo, &hi);
    CHECK(lo.x == -1.0 && lo.y == -1.0);
    CHECK(hi.x == 3.0 && hi.y == 4.0);
}

/* --------------------------------------------------------------- validate */

static void test_validate_point_and_segment(void)
{
    FdtdError err = {{0}};

    Shape ok_point = point_shape(1.0, 2.0);
    CHECK(shape_validate(&ok_point, &err) == 0);

    Shape bad_point = point_shape(NAN, 0.0);
    CHECK(shape_validate(&bad_point, &err) == -1);
    CHECK_CONTAINS(err.msg, "finite");

    bad_point = point_shape(0.0, INFINITY);
    CHECK(shape_validate(&bad_point, &err) == -1);
    CHECK_CONTAINS(err.msg, "finite");

    Shape ok_seg = segment_shape(0.0, 0.0, 1.0, 1.0);
    CHECK(shape_validate(&ok_seg, &err) == 0);

    Shape zero_seg = segment_shape(1.0, 1.0, 1.0, 1.0);
    CHECK(shape_validate(&zero_seg, &err) == -1);
    CHECK_CONTAINS(err.msg, "zero length");

    Shape nan_seg = segment_shape(0.0, 0.0, NAN, 1.0);
    CHECK(shape_validate(&nan_seg, &err) == -1);
    CHECK_CONTAINS(err.msg, "finite");
}

static void test_validate_rect(void)
{
    FdtdError err = {{0}};

    Shape ok = rect_shape(0.0, 0.0, 1.0, 1.0);
    CHECK(shape_validate(&ok, &err) == 0);

    Shape flipped = rect_shape(1.0, 0.0, 0.0, 1.0);
    CHECK(shape_validate(&flipped, &err) == -1);
    CHECK_CONTAINS(err.msg, "max > min");

    Shape flat = rect_shape(0.0, 0.0, 1.0, 0.0);
    CHECK(shape_validate(&flat, &err) == -1);
    CHECK_CONTAINS(err.msg, "max > min");

    Shape not_finite = rect_shape(0.0, 0.0, NAN, 1.0);
    CHECK(shape_validate(&not_finite, &err) == -1);
    CHECK_CONTAINS(err.msg, "finite");
}

static void test_validate_circle(void)
{
    FdtdError err = {{0}};

    Shape ok = circle_shape(0.0, 0.0, 0.25);
    CHECK(shape_validate(&ok, &err) == 0);

    Shape zero = circle_shape(0.0, 0.0, 0.0);
    CHECK(shape_validate(&zero, &err) == -1);
    CHECK_CONTAINS(err.msg, "radius");

    Shape negative = circle_shape(0.0, 0.0, -0.5);
    CHECK(shape_validate(&negative, &err) == -1);
    CHECK_CONTAINS(err.msg, "radius");

    Shape not_finite = circle_shape(NAN, 0.0, 0.25);
    CHECK(shape_validate(&not_finite, &err) == -1);
    CHECK_CONTAINS(err.msg, "finite");
}

static void test_validate_polygon(void)
{
    FdtdError err = {{0}};

    Vec2 square[4] = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
    Shape ok = polygon_shape(square, 4);
    CHECK(shape_validate(&ok, &err) == 0);

    Vec2 cw[4] = {{0.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}, {1.0, 0.0}};
    Shape ok_cw = polygon_shape(cw, 4);
    CHECK(shape_validate(&ok_cw, &err) == 0);

    Shape too_few = polygon_shape(square, 2);
    CHECK(shape_validate(&too_few, &err) == -1);
    CHECK_CONTAINS(err.msg, "at least 3");

    Shape no_storage = polygon_shape(NULL, 4);
    CHECK(shape_validate(&no_storage, &err) == -1);
    CHECK_CONTAINS(err.msg, "at least 3");

    Vec2 collinear[3] = {{0.0, 0.0}, {1.0, 1.0}, {2.0, 2.0}};
    Shape zero_area = polygon_shape(collinear, 3);
    CHECK(shape_validate(&zero_area, &err) == -1);
    CHECK_CONTAINS(err.msg, "zero area");

    Vec2 bad_vertex[4] = {{0.0, 0.0}, {1.0, 0.0}, {1.0, INFINITY}, {0.0, 1.0}};
    Shape not_finite = polygon_shape(bad_vertex, 4);
    CHECK(shape_validate(&not_finite, &err) == -1);
    CHECK_CONTAINS(err.msg, "finite");
}

static void test_validate_unknown_type(void)
{
    FdtdError err = {{0}};
    Shape s;
    memset(&s, 0, sizeof s);
    s.type = (ShapeType)99;
    CHECK(shape_validate(&s, &err) == -1);
    CHECK_CONTAINS(err.msg, "unknown shape type");
}

/* ------------------------------------------------------------ copy / free */

static void test_copy_polygon_is_deep(void)
{
    FdtdError err = {{0}};
    Vec2 *verts = malloc(4 * sizeof *verts);
    CHECK(verts != NULL);
    if (!verts) {
        return;
    }
    verts[0] = vec(0.0, 0.0);
    verts[1] = vec(1.0, 0.0);
    verts[2] = vec(1.0, 1.0);
    verts[3] = vec(0.0, 1.0);

    Shape src = polygon_shape(verts, 4);
    Shape dst;
    memset(&dst, 0, sizeof dst);

    CHECK(shape_copy(&dst, &src, &err) == 0);
    CHECK(dst.type == SHAPE_POLYGON);
    CHECK(dst.u.polygon.n == 4);
    CHECK(dst.u.polygon.vertices != NULL);
    CHECK(dst.u.polygon.vertices != verts);
    CHECK(shape_contains(&dst, vec(0.5, 0.5)));

    /* Modify the source after the copy: the copy must not change. */
    src.u.polygon.vertices[0].x = 7.0;
    src.u.polygon.vertices[2].y = 9.0;
    CHECK(dst.u.polygon.vertices[0].x == 0.0);
    CHECK(dst.u.polygon.vertices[2].y == 1.0);

    shape_free(&dst);
    shape_free(&src); /* frees the malloc'ed vertices */
    CHECK(dst.type == SHAPE_POINT && dst.u.polygon.vertices == NULL);
    CHECK(src.type == SHAPE_POINT && src.u.polygon.vertices == NULL);
}

static void test_copy_non_polygon(void)
{
    FdtdError err = {{0}};
    Shape src = circle_shape(1.0, 2.0, 0.5);
    Shape dst;
    memset(&dst, 0, sizeof dst);

    CHECK(shape_copy(&dst, &src, &err) == 0);
    CHECK(dst.type == SHAPE_CIRCLE);
    CHECK(dst.u.circle.center.x == 1.0);
    CHECK(dst.u.circle.center.y == 2.0);
    CHECK(dst.u.circle.radius == 0.5);
    shape_free(&dst);
    shape_free(&src);
}

static void test_free_twice_and_zeroed(void)
{
    Vec2 *verts = malloc(3 * sizeof *verts);
    CHECK(verts != NULL);
    if (!verts) {
        return;
    }
    verts[0] = vec(0.0, 0.0);
    verts[1] = vec(1.0, 0.0);
    verts[2] = vec(0.0, 1.0);

    Shape s = polygon_shape(verts, 3);
    shape_free(&s);
    CHECK(s.type == SHAPE_POINT);
    CHECK(s.u.polygon.vertices == NULL);
    CHECK(s.u.polygon.n == 0);
    shape_free(&s); /* second call must be safe */
    CHECK(s.type == SHAPE_POINT);

    Shape zeroed;
    memset(&zeroed, 0, sizeof zeroed);
    shape_free(&zeroed);
    CHECK(zeroed.type == SHAPE_POINT);

    shape_free(NULL); /* must not crash */
}

/* --------------------------------------------------- segment measurement */

static void test_segment_length_in_box(void)
{
    const Vec2 bmin = {0.0, 0.0};
    const Vec2 bmax = {1.0, 1.0};

    /* fully inside */
    CHECK_NEAR(geom_segment_length_in_box(vec(0.2, 0.2), vec(0.8, 0.8), bmin, bmax),
               0.6 * sqrt(2.0), 1e-15);

    /* starts outside, ends inside: only the part inside counts */
    CHECK_NEAR(geom_segment_length_in_box(vec(-1.0, 0.5), vec(0.5, 0.5), bmin, bmax),
               0.5, 1e-15);
    CHECK_NEAR(geom_segment_length_in_box(vec(0.5, 0.5), vec(1.5, 0.5), bmin, bmax),
               0.5, 1e-15);

    /* diagonal crossing the whole box */
    CHECK_NEAR(geom_segment_length_in_box(vec(-1.0, -1.0), vec(2.0, 2.0), bmin, bmax),
               sqrt(2.0), 1e-15);

    /* completely outside */
    CHECK(geom_segment_length_in_box(vec(2.0, 2.0), vec(3.0, 3.0), bmin, bmax) == 0.0);
    CHECK(geom_segment_length_in_box(vec(-3.0, 0.5), vec(-2.0, 0.5), bmin, bmax) == 0.0);

    /* lying exactly on a box edge is counted in full (callers normalise) */
    CHECK_NEAR(geom_segment_length_in_box(vec(0.0, 0.0), vec(1.0, 0.0), bmin, bmax),
               1.0, 1e-15);
    CHECK_NEAR(geom_segment_length_in_box(vec(-1.0, 1.0), vec(2.0, 1.0), bmin, bmax),
               1.0, 1e-15);
    CHECK_NEAR(geom_segment_length_in_box(vec(0.0, 0.3), vec(0.0, 0.7), bmin, bmax),
               0.4, 1e-15);

    /* degenerate point */
    CHECK(geom_segment_length_in_box(vec(0.5, 0.5), vec(0.5, 0.5), bmin, bmax) == 0.0);
    /* single point of contact */
    CHECK(geom_segment_length_in_box(vec(1.0, 1.0), vec(2.0, 2.0), bmin, bmax) == 0.0);

    /* sub-box of a wider grid (the way source.c uses it) */
    const Vec2 cmin = {0.2, 0.2};
    const Vec2 cmax = {0.3, 0.3};
    CHECK_NEAR(geom_segment_length_in_box(vec(0.0, 0.25), vec(1.0, 0.25), cmin, cmax),
               0.1, 1e-15);
}

static void test_clip_segment(void)
{
    const Vec2 bmin = {0.0, 0.0};
    const Vec2 bmax = {1.0, 1.0};

    /* fully inside: unchanged */
    Vec2 a = vec(0.2, 0.2);
    Vec2 b = vec(0.8, 0.6);
    CHECK(geom_clip_segment(&a, &b, bmin, bmax));
    CHECK_NEAR(a.x, 0.2, 1e-15);
    CHECK_NEAR(a.y, 0.2, 1e-15);
    CHECK_NEAR(b.x, 0.8, 1e-15);
    CHECK_NEAR(b.y, 0.6, 1e-15);

    /* crossing: clipped to the box edge */
    a = vec(-1.0, 0.5);
    b = vec(0.5, 0.5);
    CHECK(geom_clip_segment(&a, &b, bmin, bmax));
    CHECK_NEAR(a.x, 0.0, 1e-15);
    CHECK_NEAR(a.y, 0.5, 1e-15);
    CHECK_NEAR(b.x, 0.5, 1e-15);
    CHECK_NEAR(hypot(b.x - a.x, b.y - a.y), 0.5, 1e-15);

    /* outside: false and the end points stay untouched */
    a = vec(2.0, 2.0);
    b = vec(3.0, 3.0);
    CHECK(!geom_clip_segment(&a, &b, bmin, bmax));
    CHECK(a.x == 2.0 && a.y == 2.0);
    CHECK(b.x == 3.0 && b.y == 3.0);

    /* touching a corner only: true but zero length */
    a = vec(-1.0, -1.0);
    b = vec(0.0, 0.0);
    CHECK(geom_clip_segment(&a, &b, bmin, bmax));
    CHECK_NEAR(a.x, 0.0, 1e-15);
    CHECK_NEAR(a.y, 0.0, 1e-15);
    CHECK_NEAR(b.x, 0.0, 1e-15);
    CHECK_NEAR(b.y, 0.0, 1e-15);

    /* reversed direction is clipped too */
    a = vec(0.5, 0.5);
    b = vec(-2.0, 0.5);
    CHECK(geom_clip_segment(&a, &b, bmin, bmax));
    CHECK_NEAR(b.x, 0.0, 1e-15);
    CHECK_NEAR(a.x, 0.5, 1e-15);
}

int main(void)
{
    fdtd_set_warning_stream(NULL);

    RUN_TEST(test_name_type_conversions);
    RUN_TEST(test_is_area);
    RUN_TEST(test_contains_rect);
    RUN_TEST(test_contains_circle);
    RUN_TEST(test_contains_polygon);
    RUN_TEST(test_contains_non_area_is_false);
    RUN_TEST(test_bbox);
    RUN_TEST(test_validate_point_and_segment);
    RUN_TEST(test_validate_rect);
    RUN_TEST(test_validate_circle);
    RUN_TEST(test_validate_polygon);
    RUN_TEST(test_validate_unknown_type);
    RUN_TEST(test_copy_polygon_is_deep);
    RUN_TEST(test_copy_non_polygon);
    RUN_TEST(test_free_twice_and_zeroed);
    RUN_TEST(test_segment_length_in_box);
    RUN_TEST(test_clip_segment);

    return TEST_REPORT();
}
