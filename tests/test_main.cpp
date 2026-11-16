#include "test_common.hpp"

void test_tuning();
void test_collision();
void test_motion();
void test_mecanum();
void test_control();
void test_library();
void test_sim();
void test_integration();
void test_macro();

int g_test_failures = 0;

int main() {
    test_tuning();
    test_collision();
    test_motion();
    test_mecanum();
    test_control();
    test_library();
    test_sim();
    test_integration();
    test_macro();

    if (g_test_failures == 0) {
        std::printf("all host tests passed\n");
        return 0;
    }

    std::printf("%d host test(s) failed\n", g_test_failures);
    return 1;
}
