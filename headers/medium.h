/*
 * medium.h - acoustic media (inviscid ideal fluid, linear acoustics).
 *
 * The model stores a per-cell medium index into a MediumTable. Version 1
 * only has the background medium (index 0); medium regions will append
 * entries with medium_table_add() and paint their index into the map,
 * without changes to the solver.
 */
#ifndef FDTD2D_MEDIUM_H
#define FDTD2D_MEDIUM_H

#include <stdint.h>

#include "common.h"

typedef struct Medium {
    double density; /* rho [kg/m^3] */
    double c;       /* speed of sound [m/s] */
} Medium;

typedef uint16_t MediumId;
#define MEDIUM_MAX_COUNT 65535

typedef struct MediumTable {
    Medium *items;
    int count;
} MediumTable;

/* Check density > 0, c > 0, both finite. This is the only place media are
 * validated (config_load_* calls it); the table functions below expect
 * media that already passed it. */
int medium_validate(const Medium *m, FdtdError *err);

/* Bulk modulus K = rho * c^2 [Pa]. */
double medium_bulk_modulus(const Medium *m);

/* Initialise the table with the background medium as entry 0. `background`
 * must already be valid. Returns -1 (with *t zeroed) on out of memory. */
int medium_table_init(MediumTable *t, const Medium *background, FdtdError *err);

/* Append a medium, which must already be valid; returns its index (>= 1) or
 * -1 on error (table full or out of memory; the table is left unchanged). */
int medium_table_add(MediumTable *t, const Medium *m, FdtdError *err);

/* Largest / smallest sound speed in a table initialised by
 * medium_table_init() (count >= 1). */
double medium_table_c_max(const MediumTable *t);
double medium_table_c_min(const MediumTable *t);

void medium_table_free(MediumTable *t);

#endif /* FDTD2D_MEDIUM_H */
