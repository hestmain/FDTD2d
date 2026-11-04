/*
 * waveform.h - source time functions (signals).
 *
 * Version 1 only implements the harmonic (time-harmonic, CW) signal:
 *
 *   s(t) = A * ramp(t) * sin(2*pi*f*t + phase)            for t >= 0
 *   ramp(t) = 0.5 * (1 - cos(pi * t / T_r))  for t < T_r,  1 afterwards
 *   T_r = ramp_cycles / f   (ramp_cycles == 0 disables the ramp)
 *   s(t) = 0 for t < 0
 *
 * The meaning/unit of A is defined by the source kind (see source.h).
 * New time functions (Gaussian pulse, Ricker, tone burst, file input) are
 * added as new SignalType values plus a union member.
 */
#ifndef FDTD2D_WAVEFORM_H
#define FDTD2D_WAVEFORM_H

#include "common.h"

typedef enum SignalType {
    SIGNAL_HARMONIC = 0,
    SIGNAL_TYPE_COUNT
} SignalType;

typedef struct HarmonicSignal {
    double frequency;   /* f [Hz], > 0 */
    double amplitude;   /* A, finite */
    double phase;       /* [rad], finite */
    double ramp_cycles; /* >= 0 */
} HarmonicSignal;

typedef struct Signal {
    SignalType type;
    union {
        HarmonicSignal harmonic;
    } u;
} Signal;

/* "harmonic". type must be a valid enumerator (< SIGNAL_TYPE_COUNT). */
const char *signal_type_name(SignalType type);
/* Returns 0 and sets *out if name (not NULL) is a known type name, else -1. */
int signal_type_from_name(const char *name, SignalType *out);

/* Range checks listed in the struct comments. */
int signal_validate(const Signal *s, FdtdError *err);

/* Value at time t [s]. */
double signal_eval(const Signal *s, double t);

/* Highest frequency of significant content [Hz]; used for the
 * points-per-wavelength check. Harmonic: f. */
double signal_max_frequency(const Signal *s);

#endif /* FDTD2D_WAVEFORM_H */
