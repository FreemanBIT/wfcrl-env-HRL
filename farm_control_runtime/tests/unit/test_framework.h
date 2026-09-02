/*
 * test_framework.h — farm_control_runtime 单元测试迷你框架（声明）
 */
#ifndef FCR_TEST_FRAMEWORK_H
#define FCR_TEST_FRAMEWORK_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <setjmp.h>

#define TEST_MAX 256

typedef void (*test_fn)(void);

void test_register(const char *name, test_fn fn);
int  test_run_all(const char *suite);
extern jmp_buf g_env;

#define ASSERT(cond) do {                                                        \
    if (!(cond)) {                                                               \
        fprintf(stderr, "  [FAIL] %s:%d: ASSERT(%s)\n", __FILE__, __LINE__, #cond); \
        longjmp(g_env, 1);                                                       \
    }                                                                            \
} while (0)

#define ASSERT_NEAR(a, b, eps) do {                                              \
    double _a = (a), _b = (b);                                                   \
    if (!(fabs(_a - _b) <= (eps))) {                                              \
        fprintf(stderr, "  [FAIL] %s:%d: ASSERT_NEAR(%s=%g, %s=%g, eps=%g)\n",   \
                __FILE__, __LINE__, #a, _a, #b, _b, (double)(eps));               \
        longjmp(g_env, 1);                                                        \
    }                                                                             \
} while (0)

#define RUN_TEST(name, fn) test_register(name, fn)

#endif /* FCR_TEST_FRAMEWORK_H */
