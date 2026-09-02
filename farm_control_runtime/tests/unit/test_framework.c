#include "test_framework.h"

typedef struct {
    const char *name;
    test_fn     fn;
} TestEntry;

static TestEntry g_tests[TEST_MAX];
static int g_n_tests = 0;
jmp_buf g_env;

void test_register(const char *name, test_fn fn)
{
    if (g_n_tests < TEST_MAX) {
        g_tests[g_n_tests].name = name;
        g_tests[g_n_tests].fn = fn;
        g_n_tests++;
    } else {
        fprintf(stderr, "test registry overflow\n");
        exit(2);
    }
}

int test_run_all(const char *suite)
{
    int i, failed = 0;
    printf("== %s: %d tests ==\n", suite, g_n_tests);
    for (i = 0; i < g_n_tests; ++i) {
        if (setjmp(g_env) == 0) {
            g_tests[i].fn();
            printf("  [PASS] %s\n", g_tests[i].name);
        } else {
            printf("  [FAIL] %s\n", g_tests[i].name);
            failed++;
        }
    }
    printf("== %s done: %d/%d passed ==\n", suite, g_n_tests - failed, g_n_tests);
    return failed;
}
