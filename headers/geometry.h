/*
 * geometry.h - grid-independent 2D shapes.
 *
 * One Shape type is shared by everything geometric in the model:
 *   - rigid obstacles            (area shapes: rect, circle, polygon)
 *   - medium regions (future)    (area shapes)
 *   - sources                    (point -> point source, segment -> line
 *                                 source; area shapes reserved for future
 *                                 radiating surfaces)
 * Rasterisation onto the grid lives in model.c / source.c, so this module
 * stays pure geometry. All coordinates are in metres.
 */
#ifndef FDTD2D_GEOMETRY_H
#define FDTD2D_GEOMETRY_H

#include <stdbool.h>

#include "common.h"

typedef struct Vec2 {
    double x, y;
} Vec2;

typedef enum ShapeType {
    SHAPE_POINT = 0,
    SHAPE_SEGMENT,
    SHAPE_RECT,
    SHAPE_CIRCLE,
    SHAPE_POLYGON,
    SHAPE_TYPE_COUNT
} ShapeType;

typedef struct Shape {
    ShapeType type;
    union {
        struct { Vec2 position; } point;
        struct { Vec2 start, end; } segment;
        struct { Vec2 min, max; } rect;             /* axis aligned */
        struct { Vec2 center; double radius; } circle;
        struct { Vec2 *vertices; int n; } polygon;  /* owned, n >= 3 */
    } u;
} Shape;

/* "point", "segment", "rect", "circle", "polygon" (the TOML type names).
 * type must be a valid enumerator (< SHAPE_TYPE_COUNT). */
const char *shape_type_name(ShapeType type);
/* Returns 0 and sets *out if name (not NULL) is a known type name, else -1. */
int shape_type_from_name(const char *name, ShapeType *out);

/* True for shapes that enclose an area: rect, circle, polygon. */
bool shape_is_area(const Shape *s);

/* Point-in-shape test for area shapes. Boundary points count as inside
 * for rect and circle; polygon uses the even-odd rule (boundary points
 * are unspecified). Always false for point and segment. */
bool shape_contains(const Shape *s, Vec2 p);

/* Axis-aligned bounding box. */
void shape_bbox(const Shape *s, Vec2 *min, Vec2 *max);

/* Check that all coordinates are finite and the shape is non-degenerate:
 * segment length > 0, rect max > min on both axes, circle radius > 0,
 * polygon n >= 3 with non-zero signed area. Message names the problem
 * without location context (callers prefix it). */
int shape_validate(const Shape *s, FdtdError *err);

/* Deep copy (polygon vertices are duplicated). dst must not alias src. */
int shape_copy(Shape *dst, const Shape *src, FdtdError *err);

/* Release owned memory and reset to a zero point shape. Safe on zeroed
 * shapes and safe to call twice. */
void shape_free(Shape *s);

/* Length of the part of segment [a,b] that lies inside the closed box
 * [bmin,bmax] (Liang-Barsky clipping). Returns 0 if there is no overlap.
 * Note: a segment lying exactly on the shared edge of two boxes is counted
 * fully in both; callers that partition a segment must normalise. */
double geom_segment_length_in_box(Vec2 a, Vec2 b, Vec2 bmin, Vec2 bmax);

/* Clip segment [*a,*b] to the closed box in place. Returns true if a
 * non-empty part remains (possibly a single point). */
bool geom_clip_segment(Vec2 *a, Vec2 *b, Vec2 bmin, Vec2 bmax);

#endif /* FDTD2D_GEOMETRY_H */
