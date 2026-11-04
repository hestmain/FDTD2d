#include "signal.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *const k_signal_names[SIGNAL_TYPE_COUNT] = {
    [SIGNAL_HARMONIC] = "harmonic",
};

const char *signal_type_name(SignalType type)
{
    if ((int)type < 0 || type >= SIGNAL_TYPE_COUNT) {
        return "unknown";
    }
    return k_signal_names[type];
}

int signal_type_from_name(const char *name, SignalType *out)
{
    if (!name) {
        return -1;
    }
    for (int t = 0; t < SIGNAL_TYPE_COUNT; t++) {
        if (strcmp(name, k_signal_names[t]) == 0) {
            *out = (SignalType)t;
            return 0;
        }
    }
    return -1;
}

int signal_validate(const Signal *s, FdtdError *err)
{
    switch (s->type) {
    case SIGNAL_HARMONIC: {
        const HarmonicSignal *h = &s->u.harmonic;
        if (!isfinite(h->frequency) || !(h->frequency > 0.0)) {
            fdtd_error_set(err, "frequency must be a finite value > 0 (got %g)", h->frequency);
            return -1;
        }
        if (!isfinite(h->amplitude)) {
            fdtd_error_set(err, "amplitude must be finite");
            return -1;
        }
        if (!isfinite(h->phase)) {
            fdtd_error_set(err, "phase must be finite");
            return -1;
        }
        if (!isfinite(h->ramp_cycles) || h->ramp_cycles < 0.0) {
            fdtd_error_set(err, "ramp_cycles must be a finite value >= 0 (got %g)", h->ramp_cycles);
            return -1;
        }
        return 0;
    }
    case SIGNAL_TYPE_COUNT:
        break;
    }
    fdtd_error_set(err, "unknown signal type %d", (int)s->type);
    return -1;
}

double signal_eval(const Signal *s, double t)
{
    if (t < 0.0) {
        return 0.0;
    }
    switch (s->type) {
    case SIGNAL_HARMONIC: {
        const HarmonicSignal *h = &s->u.harmonic;
        double ramp = 1.0;
        if (h->ramp_cycles > 0.0) {
            double t_ramp = h->ramp_cycles / h->frequency;
            if (t < t_ramp) {
                ramp = 0.5 * (1.0 - cos(M_PI * t / t_ramp));
            }
        }
        return h->amplitude * ramp * sin(2.0 * M_PI * h->frequency * t + h->phase);
    }
    case SIGNAL_TYPE_COUNT:
        break;
    }
    return 0.0;
}

double signal_max_frequency(const Signal *s)
{
    switch (s->type) {
    case SIGNAL_HARMONIC:
        return s->u.harmonic.frequency;
    case SIGNAL_TYPE_COUNT:
        break;
    }
    return 0.0;
}
