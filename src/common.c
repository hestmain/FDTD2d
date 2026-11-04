#include "common.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static FILE *g_warn_stream = NULL;
static bool g_warn_stream_set = false;
static int g_warn_count = 0;

void fdtd_error_set(FdtdError *err, const char *fmt, ...)
{
    if (!err) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(err->msg, sizeof err->msg, fmt, ap);
    va_end(ap);
}

/* Append s to dst (capacity cap, current length *pos), truncating. */
static void append_truncated(char *dst, size_t cap, size_t *pos, const char *s)
{
    while (*s && *pos + 1 < cap) {
        dst[(*pos)++] = *s++;
    }
    dst[*pos] = '\0';
}

void fdtd_error_prefix(FdtdError *err, const char *fmt, ...)
{
    if (!err) {
        return;
    }
    char prefix[sizeof err->msg];
    char old[sizeof err->msg];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(prefix, sizeof prefix, fmt, ap);
    va_end(ap);
    memcpy(old, err->msg, sizeof old);
    old[sizeof old - 1] = '\0';
    size_t pos = 0;
    append_truncated(err->msg, sizeof err->msg, &pos, prefix);
    if (old[0] != '\0') {
        append_truncated(err->msg, sizeof err->msg, &pos, ": ");
        append_truncated(err->msg, sizeof err->msg, &pos, old);
    }
}

void fdtd_warn(const char *fmt, ...)
{
    g_warn_count++;
    FILE *out = g_warn_stream_set ? g_warn_stream : stderr;
    if (!out) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    fputs("warning: ", out);
    vfprintf(out, fmt, ap);
    fputc('\n', out);
    va_end(ap);
}

void fdtd_set_warning_stream(FILE *stream)
{
    g_warn_stream = stream;
    g_warn_stream_set = true;
}

int fdtd_warning_count(void)
{
    return g_warn_count;
}

void fdtd_reset_warning_count(void)
{
    g_warn_count = 0;
}

void *fdtd_calloc(size_t n, size_t size, FdtdError *err)
{
    if (size != 0 && n > SIZE_MAX / size) {
        fdtd_error_set(err, "allocation size overflow (%zu x %zu bytes)", n, size);
        return NULL;
    }
    void *p = calloc(n ? n : 1, size ? size : 1);
    if (!p) {
        fdtd_error_set(err, "out of memory allocating %zu x %zu bytes", n, size);
    }
    return p;
}

char *fdtd_strdup(const char *s, FdtdError *err)
{
    size_t len = strlen(s);
    char *copy = fdtd_calloc(len + 1, 1, err);
    if (copy) {
        memcpy(copy, s, len + 1);
    }
    return copy;
}
