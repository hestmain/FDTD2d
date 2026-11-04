/*
 * solver.h - 2D linear-acoustics FDTD on a staggered grid (leapfrog).
 *
 * Equations (inviscid fluid at rest), split-field PML form:
 *   d(px)/dt + sx px = -K d(vx)/dx + K q / 2
 *   d(py)/dt + sy py = -K d(vy)/dy + K q / 2          p = px + py
 *   d(vx)/dt + sx vx = -(1/rho) dp/dx
 *   d(vy)/dt + sy vy = -(1/rho) dp/dy
 * sx, sy = PML damping (0 in the physical domain), K = rho c^2.
 *
 * Time levels: p at t_n = n dt, v at t_{n+1/2}. One solver_step():
 *   1. v^{n+1/2} from v^{n-1/2} and p^n
 *   2. p^{n+1}  from p^n and v^{n+1/2}, plus sources evaluated at t_{n+1/2}
 * Semi-implicit damping: f_new = a f_old - c * dt * rhs with
 *   a = (1 - sigma dt/2) / (1 + sigma dt/2),  c = 1 / (1 + sigma dt/2).
 *
 * Boundaries:
 *   - x/y-face between two fluid cells: updated normally, rho_face =
 *     arithmetic mean of the two cell densities;
 *   - face touching a solid cell: v = 0 (rigid);
 *   - outermost faces (i = 0, i = nx for vx; j = 0, j = ny for vy):
 *       rigid or pml side      -> v = 0
 *       pressure_release side  -> ghost cell p = -p_inside, rho_face =
 *                                 rho of the inside cell (v = 0 if solid)
 *   - solid cells: p stays 0.
 */
#ifndef FDTD2D_SOLVER_H
#define FDTD2D_SOLVER_H

#include <stddef.h>

#include "common.h"
#include "model.h"

typedef struct Solver {
    const Model *model;  /* borrowed; must outlive the solver */
    int nx, ny;

    /* fields */
    double *px, *py;     /* [nx*ny] split pressure, p = px + py */
    double *vx;          /* [(nx+1)*ny] */
    double *vy;          /* [nx*(ny+1)] */

    /* precomputed coefficients (implementation detail, read-only) */
    double *kdt;         /* [nx*ny]      K*dt for fluid cells, 0 for solid */
    double *bx;          /* [(nx+1)*ny]  dt/rho_face for active faces, else 0 */
    double *by;          /* [nx*(ny+1)] */
    double *ax_c, *cx_c; /* [nx]   damping coefficients at cell centres (x) */
    double *ax_f, *cx_f; /* [nx+1] at x-faces */
    double *ay_c, *cy_c; /* [ny] */
    double *ay_f, *cy_f; /* [ny+1] */

    long step;           /* completed steps; p is at time = step * dt */
    double time;         /* step * dt */
} Solver;

/* Allocate fields (zero initial state) and precompute coefficients.
 * Preconditions: s and m are non-NULL; *m was built by model_build() and
 * stays valid (not modified, not freed) until solver_free(); err may be
 * NULL. The model's grid, dt, medium map, damping tables, boundary types
 * and source cells are not re-validated. Checked here: the grid size does
 * not overflow size_t, allocations succeed, and every fluid cell has finite
 * rho > 0 and K > 0 and every sigma is finite.
 * On error returns -1 with *s zeroed (nothing to free). */
int solver_init(Solver *s, const Model *m, FdtdError *err);

/* Releases all arrays and zeroes *s. Accepts NULL, a zeroed struct and
 * repeated calls. Call before model_free(). */
void solver_free(Solver *s);

/* Advance one time step, injecting all m->sources (read on every call, so
 * callers may change model->n_sources between steps, e.g. in tests).
 * Precondition for this and the three functions below: *s was initialised
 * by a successful solver_init() and not freed since. */
void solver_step(Solver *s);

/* Total pressure p = px + py for all nx*ny cells (solid cells: 0).
 * out must have room for nx*ny values. */
void solver_get_pressure(const Solver *s, double *out);

/* Pressure at one cell index; cell < nx*ny. */
double solver_pressure_at(const Solver *s, size_t cell);

/* Acoustic energy per unit depth [J/m] over the WHOLE grid:
 *   sum_fluid p^2/(2K) dx dy + sum_active_faces w rho_face v^2/2 dx dy,
 * w = 1 for interior faces and w = 1/2 for the outer faces of
 * pressure_release sides (only half of their control volume lies inside
 * the grid; with this weight the discrete energy balance closes exactly).
 * (p and v are half a step apart; the value oscillates slightly but must
 * not drift in a lossless domain with rigid / pressure_release sides.) */
double solver_acoustic_energy(const Solver *s);

#endif /* FDTD2D_SOLVER_H */
