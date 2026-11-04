/*
 * test_util.h - minimal self-contained test helpers (no framework).
 *
 * Usage:
 *   static void test_something(void) { CHECK(x == 1); CHECK_NEAR(a, b, 1e-12); }
 *   int main(void) { RUN_TEST(test_something); return TEST_REPORT(); }
 *
 * A failing CHECK prints file:line and the expression, marks the current
 * test failed and continues. TEST_REPORT() returns 0 if all passed, 1
 * otherwise. Return 77 from main to report "skipped" to CTest.
 */
#ifndef FDTD2D_TEST_UTIL_H
#define FDTD2D_TEST_UTIL_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_test_failures = 0;
static int g_test_count = 0;
static int g_test_current_failed = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__,   \
                    #cond);                                                    \
            g_test_current_failed = 1;                                         \
        }                                                                      \
    } while (0)

#define CHECK_MSG(cond, ...)                                                   \
    do {                                                                       \
        if (!(cond)) {                                                         \
            fprintf(stderr, "%s:%d: CHECK failed: %s: ", __FILE__, __LINE__,   \
                    #cond);                                                    \
            fprintf(stderr, __VA_ARGS__);                                      \
            fputc('\n', stderr);                                               \
            g_test_current_failed = 1;                                         \
        }                                                                      \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                  \
    do {                                                                       \
        double check_a_ = (double)(a), check_b_ = (double)(b);                 \
        double check_t_ = (double)(tol);                                       \
        if (!(fabs(check_a_ - check_b_) <= check_t_)) {                        \
            fprintf(stderr,                                                    \
                    "%s:%d: CHECK_NEAR failed: %s = %.17g, %s = %.17g, "       \
                    "|diff| = %.3g > %.3g\n",                                  \
                    __FILE__, __LINE__, #a, check_a_, #b, check_b_,            \
                    fabs(check_a_ - check_b_), check_t_);                      \
            g_test_current_failed = 1;                                         \
        }                                                                      \
    } while (0)

/* Substring check, e.g. for error messages. */
#define CHECK_CONTAINS(haystack, needle)                                       \
    do {                                                                       \
        const char *check_h_ = (haystack);                                     \
        const char *check_n_ = (needle);                                       \
        if (!check_h_ || !strstr(check_h_, check_n_)) {                        \
            fprintf(stderr, "%s:%d: CHECK_CONTAINS failed: \"%s\" not in "     \
                    "\"%s\"\n", __FILE__, __LINE__, check_n_,                  \
                    check_h_ ? check_h_ : "(null)");                           \
            g_test_current_failed = 1;                                         \
        }                                                                      \
    } while (0)

#define RUN_TEST(fn)                                                           \
    do {                                                                       \
        g_test_current_failed = 0;                                             \
        g_test_count++;                                                        \
        fn();                                                                  \
        if (g_test_current_failed) {                                           \
            g_test_failures++;                                                 \
            fprintf(stderr, "[FAIL] %s\n", #fn);                               \
        } else {                                                               \
            fprintf(stdout, "[ OK ] %s\n", #fn);                               \
        }                                                                      \
    } while (0)

#define TEST_REPORT()                                                          \
    (fprintf(stdout, "%d/%d tests passed\n", g_test_count - g_test_failures,   \
             g_test_count),                                                    \
     g_test_failures == 0 ? 0 : 1)

#endif /* FDTD2D_TEST_UTIL_H */
