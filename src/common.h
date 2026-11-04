/*
 * common.h - error reporting and warnings shared by all fdtd2d modules.
 *
 * Convention: functions that can fail return 0 on success and -1 on failure.
 * On failure they fill an FdtdError (if the caller passed a non-NULL one)
 * with a human readable, single-line message. Callers that add context use
 * fdtd_error_prefix() so the final message reads "outer: inner".
 */
#ifndef FDTD2D_COMMON_H
#define FDTD2D_COMMON_H

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#define FDTD2D_VERSION "0.1.0"

#if defined(__GNUC__) || defined(__clang__)
#define FDTD_PRINTF(fmt_idx, arg_idx) __attribute__((format(printf, fmt_idx, arg_idx)))
#else
#define FDTD_PRINTF(fmt_idx, arg_idx)
#endif

typedef struct FdtdError {
    char msg[1024];
} FdtdError;

/* Overwrite err->msg with the formatted message. err may be NULL. */
void fdtd_error_set(FdtdError *err, const char *fmt, ...) FDTD_PRINTF(2, 3);

/* Prepend "<formatted>: " to the existing message. err may be NULL. */
void fdtd_error_prefix(FdtdError *err, const char *fmt, ...) FDTD_PRINTF(2, 3);

/* Print "warning: <message>\n" to the warning stream (stderr by default)
 * and increment the global warning counter. */
void fdtd_warn(const char *fmt, ...) FDTD_PRINTF(1, 2);

/* Redirect warnings. NULL silences them (the counter still increments). */
void fdtd_set_warning_stream(FILE *stream);

/* Number of warnings issued since start or since the last reset. */
int fdtd_warning_count(void);
void fdtd_reset_warning_count(void);

/* Checked allocation helpers: return NULL and set err on failure or on
 * size_t overflow of n * size. n == 0 returns a valid unique pointer. */
void *fdtd_calloc(size_t n, size_t size, FdtdError *err);

/* strdup replacement (strdup is not ISO C11). Returns NULL and sets err on
 * allocation failure. s must not be NULL. */
char *fdtd_strdup(const char *s, FdtdError *err);

#endif /* FDTD2D_COMMON_H */
