/*
 * boundary.h - outer boundary conditions of the computational domain.
 *
 *   rigid            : normal particle velocity = 0 on the domain edge
 *   pressure_release : p = 0 on the domain edge (ghost cell p = -p_inside)
 *   pml              : split-field PML of `thickness` cells added OUTSIDE
 *                      the physical domain; its outer edge is rigid.
 */
#ifndef FDTD2D_BOUNDARY_H
#define FDTD2D_BOUNDARY_H

#include "common.h"

typedef enum BoundaryType {
    BC_RIGID = 0,
    BC_PRESSURE_RELEASE,
    BC_PML,
    BC_TYPE_COUNT
} BoundaryType;

typedef enum Side {
    SIDE_X_MIN = 0,
    SIDE_X_MAX,
    SIDE_Y_MIN,
    SIDE_Y_MAX,
    SIDE_COUNT
} Side;

typedef struct PmlParams {
    int thickness;     /* cells, >= 1 */
    double reflection; /* theoretical normal-incidence reflection, 0 < R < 1 */
    double order;      /* polynomial grading exponent m, >= 0 */
} PmlParams;

#define PML_DEFAULT_THICKNESS 20
#define PML_DEFAULT_REFLECTION 1e-6
#define PML_DEFAULT_ORDER 2.0

/* "rigid", "pressure_release", "pml" */
const char *boundary_type_name(BoundaryType type);
int boundary_type_from_name(const char *name, BoundaryType *out);

/* "x_min", "x_max", "y_min", "y_max" */
const char *side_name(Side side);

int pml_params_validate(const PmlParams *p, FdtdError *err);

#endif /* FDTD2D_BOUNDARY_H */
