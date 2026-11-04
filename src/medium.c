#include "medium.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

int medium_validate(const Medium *m, FdtdError *err)
{
    if (!isfinite(m->density) || !(m->density > 0.0)) {
        fdtd_error_set(err, "density must be a finite value > 0 (got %g)", m->density);
        return -1;
    }
    if (!isfinite(m->c) || !(m->c > 0.0)) {
        fdtd_error_set(err, "sound speed c must be a finite value > 0 (got %g)", m->c);
        return -1;
    }
    return 0;
}

double medium_bulk_modulus(const Medium *m)
{
    return m->density * m->c * m->c;
}

int medium_table_init(MediumTable *t, const Medium *background, FdtdError *err)
{
    memset(t, 0, sizeof *t);
    if (medium_validate(background, err) != 0) {
        return -1;
    }
    t->items = fdtd_calloc(1, sizeof *t->items, err);
    if (!t->items) {
        return -1;
    }
    t->items[0] = *background;
    t->count = 1;
    return 0;
}

int medium_table_add(MediumTable *t, const Medium *m, FdtdError *err)
{
    if (medium_validate(m, err) != 0) {
        return -1;
    }
    if (t->count >= MEDIUM_MAX_COUNT) {
        fdtd_error_set(err, "too many media (max %d)", MEDIUM_MAX_COUNT);
        return -1;
    }
    Medium *grown = realloc(t->items, (size_t)(t->count + 1) * sizeof *grown);
    if (!grown) {
        fdtd_error_set(err, "out of memory growing medium table");
        return -1;
    }
    t->items = grown;
    t->items[t->count] = *m;
    return t->count++;
}

double medium_table_c_max(const MediumTable *t)
{
    double c = 0.0;
    for (int k = 0; k < t->count; k++) {
        c = fmax(c, t->items[k].c);
    }
    return c;
}

double medium_table_c_min(const MediumTable *t)
{
    double c = INFINITY;
    for (int k = 0; k < t->count; k++) {
        c = fmin(c, t->items[k].c);
    }
    return t->count > 0 ? c : 0.0;
}

void medium_table_free(MediumTable *t)
{
    if (!t) {
        return;
    }
    free(t->items);
    t->items = NULL;
    t->count = 0;
}
