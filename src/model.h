/*
 * model.h - parser layer 2: SimConfig -> simulation model on the grid.
 *
 * model_build() performs every grid-dependent check and precomputation:
 * grid size, time step, medium map, rigid-obstacle mask, PML damping
 * profiles and discretised sources. The solver only reads a Model.
 *
 * PML damping profile (same formula on every PML side):
 *   sigma(d) = sigma_max * (d / L)^m,   0 <= d <= L
 *   sigma_max = -(m + 1) * c_ref * ln(R) / (2 * L)
 * d = distance from the physical-domain edge into the PML [m],
 * L = thickness * dx (x sides) or thickness * dy (y sides),
 * m = pml.order, R = pml.reflection, c_ref = maximum sound speed.
 * sigma is evaluated at cell centres (for p) and at faces (for v); it is 0
 * inside the physical domain and on non-PML sides.
 */
#ifndef FDTD2D_MODEL_H
#define FDTD2D_MODEL_H

#include <stdint.h>
#include <stdio.h>

#include "boundary.h"
#include "common.h"
#include "config.h"
#include "grid.h"
#include "medium.h"
#include "source.h"

#define MODEL_MIN_POINTS_PER_WAVELENGTH 10.0

typedef struct Model {
    Grid grid;

    double dt;          /* [s] */
    long n_steps;       /* ceil(duration / dt) */
    double duration;    /* requested duration [s] */
    double cfl;         /* c_max * dt * sqrt(1/dx^2 + 1/dy^2), <= 1 */

    MediumTable media;  /* entry 0 = background */
    MediumId *medium_id;/* [nx*ny] index into media */

    uint8_t *solid;     /* [nx*ny] 1 = rigid obstacle cell, 0 = fluid */
    size_t n_solid;

    BoundaryType boundary[SIDE_COUNT];
    PmlParams pml;
    int pml_cells[SIDE_COUNT];  /* thickness on PML sides, else 0 */

    double *sigma_x_cell;  /* [nx]   damping at cell centres [1/s] */
    double *sigma_x_face;  /* [nx+1] damping at x-faces */
    double *sigma_y_cell;  /* [ny] */
    double *sigma_y_face;  /* [ny+1] */

    ModelSource *sources;
    int n_sources;
} Model;

/* Build the model. Checks, in addition to SimConfig's own validation:
 *  - (x1-x0)/dx and (y1-y0)/dy are integers within 1e-6 relative
 *    tolerance (error names the axis and suggests a nearest valid step);
 *  - total cell counts fit in int and nx*ny fits in size_t;
 *  - dt: if cfg->dt_set, cfl = c_max*dt*sqrt(1/dx^2+1/dy^2) must be <= 1
 *    (error otherwise); else dt = cfg->cfl / (c_max*sqrt(1/dx^2+1/dy^2));
 *    n_steps = max(1, ceil(duration/dt - 1e-9));
 *  - warning if c_min / (f_max * max(dx,dy)) < MODEL_MIN_POINTS_PER_WAVELENGTH;
 *  - obstacles: cells (including PML cells) whose centre is inside are
 *    solid; warning if an obstacle covers no cell centre;
 *  - sources: source_discretize() for each, errors prefixed "source[k]";
 *    warning if there are no sources.
 * On error returns -1 with *m zeroed. */
int model_build(const SimConfig *cfg, Model *m, FdtdError *err);

void model_free(Model *m);

/* Human readable multi-line summary: grid (phys + total cells, dx, dy,
 * PML per side), dt, n_steps, cfl, points per wavelength, medium,
 * boundaries, solid cell count, sources (type, cells, frequency). */
void model_print_summary(FILE *out, const Model *m);

#endif /* FDTD2D_MODEL_H */
