/*
 * grid.h - staggered (Yee) grid layout and index helpers.
 *
 * The computational grid = physical domain + PML cells added OUTSIDE it.
 *
 *   cells     : nx * ny, pressure p at cell centres, index j*nx + i
 *   x-faces   : (nx+1) * ny, vx at x = face_x(i), index j*(nx+1) + i
 *               face i lies between cells i-1 and i
 *   y-faces   : nx * (ny+1), vy at y = face_y(j), index j*nx + i
 *               face j lies between cells j-1 and j
 *
 * Cell (i, j) with off_x <= i < off_x + nx_phys and off_y <= j < off_y +
 * ny_phys belongs to the physical domain; the rest is PML. off_x / off_y
 * equal the PML thickness (in cells) on the x_min / y_min sides (0 if
 * that side is not a PML).
 *
 * All arrays are row major with x varying fastest.
 */
#ifndef FDTD2D_GRID_H
#define FDTD2D_GRID_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct Grid {
    int nx, ny;           /* total cells including PML */
    int nx_phys, ny_phys; /* cells in the physical domain */
    int off_x, off_y;     /* index of the first physical cell */
    double dx, dy;        /* cell size [m] */
    double x0, y0;        /* physical domain lower-left corner [m] */
} Grid;

static inline size_t grid_cell_count(const Grid *g)
{
    return (size_t)g->nx * (size_t)g->ny;
}

static inline size_t grid_xface_count(const Grid *g)
{
    return (size_t)(g->nx + 1) * (size_t)g->ny;
}

static inline size_t grid_yface_count(const Grid *g)
{
    return (size_t)g->nx * (size_t)(g->ny + 1);
}

static inline size_t grid_cell_index(const Grid *g, int i, int j)
{
    return (size_t)j * (size_t)g->nx + (size_t)i;
}

static inline size_t grid_xface_index(const Grid *g, int i, int j)
{
    return (size_t)j * (size_t)(g->nx + 1) + (size_t)i;
}

static inline size_t grid_yface_index(const Grid *g, int i, int j)
{
    return (size_t)j * (size_t)g->nx + (size_t)i;
}

/* Physical coordinate of cell centre i (may be outside the physical
 * domain for PML cells). */
static inline double grid_cell_x(const Grid *g, int i)
{
    return g->x0 + ((double)(i - g->off_x) + 0.5) * g->dx;
}

static inline double grid_cell_y(const Grid *g, int j)
{
    return g->y0 + ((double)(j - g->off_y) + 0.5) * g->dy;
}

/* Physical coordinate of x-face i / y-face j. */
static inline double grid_face_x(const Grid *g, int i)
{
    return g->x0 + (double)(i - g->off_x) * g->dx;
}

static inline double grid_face_y(const Grid *g, int j)
{
    return g->y0 + (double)(j - g->off_y) * g->dy;
}

/* Upper-right corner of the physical domain. */
static inline double grid_x1(const Grid *g)
{
    return g->x0 + (double)g->nx_phys * g->dx;
}

static inline double grid_y1(const Grid *g)
{
    return g->y0 + (double)g->ny_phys * g->dy;
}

/* Cell containing physical point (x, y), clamped to the grid. Returns true
 * if the point lies inside the full grid (physical domain + PML). */
static inline bool grid_locate(const Grid *g, double x, double y, int *i, int *j)
{
    double fi = floor((x - g->x0) / g->dx) + (double)g->off_x;
    double fj = floor((y - g->y0) / g->dy) + (double)g->off_y;
    bool inside = fi >= 0.0 && fi < (double)g->nx && fj >= 0.0 && fj < (double)g->ny;
    if (fi < 0.0) fi = 0.0;
    if (fj < 0.0) fj = 0.0;
    if (fi > (double)(g->nx - 1)) fi = (double)(g->nx - 1);
    if (fj > (double)(g->ny - 1)) fj = (double)(g->ny - 1);
    *i = (int)fi;
    *j = (int)fj;
    return inside;
}

static inline bool grid_cell_is_physical(const Grid *g, int i, int j)
{
    return i >= g->off_x && i < g->off_x + g->nx_phys &&
           j >= g->off_y && j < g->off_y + g->ny_phys;
}

#endif /* FDTD2D_GRID_H */
