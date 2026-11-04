/*
 * source.h - spatial discretisation of sources onto the grid.
 *
 * A model source = spatial distribution (from its Shape) x time signal.
 * Sources are monopole volume-injection ("soft") sources entering the
 * pressure equation as
 *
 *     dp/dt = -K div(v) + K * q(x, t),     K = rho c^2,
 *     q(x, t) = s(t) * D(x)                 [1/s]
 *
 * where s(t) is the signal and D(x) the spatial density:
 *   point   : D = delta(x - x_s);           s = volume velocity per unit
 *                                           depth Q [m^2/s]
 *   segment : D = delta on the line;        s = volume velocity per unit
 *                                           length and depth Q' [m/s]
 * On the grid D becomes per-cell weights w_c such that
 *   point   : sum_c w_c * dx * dy = 1                          w_c [1/m^2]
 *   segment : sum_c w_c * dx * dy = (radiating) segment length [m]
 *                                                              w_c [1/m]
 * In both cases w_c * s has the unit of a volume source rate [1/s].
 */
#ifndef FDTD2D_SOURCE_H
#define FDTD2D_SOURCE_H

#include <stddef.h>
#include <stdint.h>

#include "common.h"
#include "geometry.h"
#include "grid.h"
#include "waveform.h"

typedef struct ModelSource {
    ShapeType shape_type; /* where the distribution came from */
    Signal signal;
    size_t n_cells;
    size_t *cells;        /* cell indices (grid_cell_index), unique, fluid */
    double *weights;      /* > 0; [1/m^2] point, [1/m] segment */
} ModelSource;

/* Discretise `shape` on grid `g`. `solid` (nx*ny, nonzero = rigid) may be
 * NULL.
 * Preconditions (not checked here; model_build() satisfies them): `shape`,
 * `signal`, `g` and `out` are non-NULL; `g` has dx, dy > 0 and nx, ny >= 1;
 * the shape and signal already passed shape_validate() / signal_validate()
 * (config_load_* does this), so a segment has non-zero length.
 * Rules:
 * Only PHYSICAL cells ever receive weight (never PML cells), so a source's
 * strength does not depend on the PML thickness.
 *  - point: must lie inside the closed physical domain, else error; error if
 *    the physical cell containing it is solid. Weights are bilinear over the
 *    4 nearest physical cell centres (indices clamped to the physical
 *    domain), solid cells dropped and the rest renormalised to sum 1.
 *    Weight = bilinear / (dx * dy).
 *  - segment: clipped to the physical domain (warning if clipped, error if
 *    nothing remains). Each physical cell gets the clipped segment length
 *    inside it, normalised so the total equals the clipped length (handles
 *    segments on cell edges). Solid cells are then dropped (warning with the
 *    dropped fraction, so the radiating length = clipped length minus the
 *    part on solid cells); error if nothing remains.
 *    Weight = length / (dx * dy). Cost is O(length / cell size).
 *  - any other shape (rect, circle, polygon): error "<shape> source is not
 *    supported in this version".
 * On error *out is left zeroed. */
int source_discretize(const Shape *shape, const Signal *signal, const Grid *g,
                      const uint8_t *solid, ModelSource *out, FdtdError *err);

void model_source_free(ModelSource *s);

#endif /* FDTD2D_SOURCE_H */
