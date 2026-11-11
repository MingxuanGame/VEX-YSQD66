#pragma once
#include <cstdio>

/**
 * @file test_common.hpp
 * @brief 极简断言宏，避免为了几个 host 测试引入额外的测试框架依赖。
 */

extern int g_test_failures;

#define CHECK(cond)                                                                                \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                            \
            ++g_test_failures;                                                                     \
        }                                                                                          \
    } while (0)
