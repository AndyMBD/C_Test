#include <stdlib.h>
#include <stdio.h>

// #include <mlxunit.h>
// #include <debug.h>
// #include "config.h"
extern "C" {
    #include "lib_svm_table.h"
}
// #include "lib_svm_table.h"



// #include "cmocks_mock.h"

#define UNUSED(x) ((void)(x))

static void lib_svm_table_test(unsigned int tst);

// #include "sample3-inl.h"
#include "gtest/gtest.h"
namespace {
/***** Test case setup and teardown ***********************************/
// static void setUp(void)
// {
//     cmocks_mock_Init();
// }
// static void tearDown(void)
// {
//     cmocks_mock_Verify();
// }

/***** Test case: General ***********************************************/
// static void lib_svm_table_test(unsigned int tst)
TEST(svm_table,lib_svm_table_test_case)
{
    // UNUSED(tst);

    uint16_t microstep_per_erot = MOTOR_SVM_PERIOD;

    for (uint16_t i = 0u; i < microstep_per_erot; i++)
    {
        if ((i + microstep_per_erot) < (MOTOR_SVM_PERIOD*5)/3)
        {
            int expected = SVM_Table_getValue(i);
            int actual = SVM_Table_getValue(i + microstep_per_erot);

            EXPECT_EQ(expected, actual) << "Mismatch detected:\n"
                                        << "SVM_Table_getValue(" << i << ") = " << expected << "\n"
                                        << "SVM_Table_getValue(" << i + microstep_per_erot << ") = " << actual;
            // TEST_ASSERT_EQUAL_UINT(SVM_Table_getValue(i), SVM_Table_getValue(i + microstep_per_erot));
            // EXPECT_EQ(SVM_Table_getValue(i), SVM_Table_getValue(i + microstep_per_erot));
            // std::cout << "SVM_Table_getValue(" << i << ") = " << SVM_Table_getValue(i) << std::endl;
            // std::cout << "SVM_Table_getValue(" << i + microstep_per_erot << ") = " << SVM_Table_getValue(i + microstep_per_erot) << std::endl;
        }
    }
}

// MLX_UNIT_TEST_SET(lib_svm_table_test_case, setUp, tearDown,
//                   MLX_UNIT_ADD_TEST_TO_SET(lib_svm_table_test),
//                   );

// /***** Main for test-suite ********************************************/
// int main(int argc, char** argv)
// {
//     TextUIRunner_start("svm_table");

//     MLX_UNIT_RUN(lib_svm_table_test_case);

//     TextUIRunner_end();
//     EXIT_SIM(0);
//     return RUN_ALL_TESTS();
// }
}