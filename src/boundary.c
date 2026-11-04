#include "boundary.h"

#include <math.h>
#include <string.h>

static const char *const k_bc_names[BC_TYPE_COUNT] = {
    [BC_RIGID] = "rigid",
    [BC_PRESSURE_RELEASE] = "pressure_release",
    [BC_PML] = "pml",
};

static const char *const k_side_names[SIDE_COUNT] = {
    [SIDE_X_MIN] = "x_min",
    [SIDE_X_MAX] = "x_max",
    [SIDE_Y_MIN] = "y_min",
    [SIDE_Y_MAX] = "y_max",
};

const char *boundary_type_name(BoundaryType type)
{
    return k_bc_names[type];
}

int boundary_type_from_name(const char *name, BoundaryType *out)
{
    for (int t = 0; t < BC_TYPE_COUNT; t++) {
        if (strcmp(name, k_bc_names[t]) == 0) {
            *out = (BoundaryType)t;
            return 0;
        }
    }
    return -1;
}

const char *side_name(Side side)
{
    return k_side_names[side];
}

int pml_params_validate(const PmlParams *p, FdtdError *err)
{
    if (p->thickness < 1) {
        fdtd_error_set(err, "pml thickness must be >= 1 cell (got %d)", p->thickness);
        return -1;
    }
    if (!isfinite(p->reflection) || !(p->reflection > 0.0 && p->reflection < 1.0)) {
        fdtd_error_set(err, "pml reflection must satisfy 0 < R < 1 (got %g)", p->reflection);
        return -1;
    }
    if (!isfinite(p->order) || p->order < 0.0) {
        fdtd_error_set(err, "pml order must be a finite value >= 0 (got %g)", p->order);
        return -1;
    }
    return 0;
}
