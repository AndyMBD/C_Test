# option(
#   gtest_force_shared_crt
#   "Use shared (DLL) run-time lib even when Google Test is built as static lib."
#   OFF)

# option(gtest_build_tests "Build all of gtest's own tests." ON)

# option(gtest_build_samples "Build gtest's sample programs." ON)

# option(gtest_disable_pthreads "Disable uses of pthreads in gtest." OFF)

# option(
#   gtest_hide_internal_symbols
#   "Build gtest with internal symbols hidden in shared libraries."
#   OFF)

cmake_minimum_required(VERSION 3.13)
project(gtest VERSION ${GOOGLETEST_VERSION} LANGUAGES CXX C)
include(CTest)

set(cmake_package_name GTest CACHE INTERNAL "")

# Where Google Test's .h files can be found.
set(gtest_build_include_dirs
  "${gtest_SOURCE_DIR}/include"
  "${gtest_SOURCE_DIR}/samples"
  "${gtest_SOURCE_DIR}/svm_table"
  "${gtest_SOURCE_DIR}/svm_table/src"
  "${gtest_SOURCE_DIR}/svm_table/unit_test"
  "${gtest_SOURCE_DIR}")
# include_directories(${gtest_build_include_dirs})
# message(STATUS "gtest_SOURCE_DIR: ${gtest_SOURCE_DIR}")
# message(STATUS "gtest_build_include_dirs: ${gtest_build_include_dirs}")
# add_library(gtest_main STATIC src/gtest_main.cc)

# include(Dart)
enable_testing()

add_executable(sample1_unittest 
              ${gtest_SOURCE_DIR}/samples/sample1.cc
              ${gtest_SOURCE_DIR}/samples/sample1_unittest.cc)              
target_link_libraries(sample1_unittest gtest gtest_main)
target_compile_options(sample1_unittest PRIVATE -coverage)
target_link_options(sample1_unittest PRIVATE -coverage)
add_test(NAME sample1_unittest COMMAND sample1_unittest)

add_executable(sample2_unittest 
              ${gtest_SOURCE_DIR}/samples/sample2.cc
              ${gtest_SOURCE_DIR}/samples/sample2_unittest.cc)
target_include_directories(sample2_unittest PRIVATE ${gtest_build_include_dirs})
target_link_libraries(sample2_unittest gtest gtest_main)
target_compile_options(sample2_unittest PRIVATE -coverage)
target_link_options(sample2_unittest PRIVATE -coverage)
add_test(NAME sample2_unittest COMMAND sample2_unittest)

add_executable(sample3_unittest 
              ${gtest_SOURCE_DIR}/samples/sample3_unittest.cc)
target_include_directories(sample3_unittest PRIVATE ${gtest_build_include_dirs})
target_link_libraries(sample3_unittest gtest gtest_main)
target_compile_options(sample3_unittest PRIVATE -coverage)
target_link_options(sample3_unittest PRIVATE -coverage)
add_test(NAME sample3_unittest COMMAND sample3_unittest)

add_executable(sample4_unittest 
              ${gtest_SOURCE_DIR}/samples/sample4.cc
              ${gtest_SOURCE_DIR}/samples/sample4_unittest.cc)
target_include_directories(sample4_unittest PRIVATE ${gtest_build_include_dirs})
target_link_libraries(sample4_unittest gtest gtest_main)
target_compile_options(sample4_unittest PRIVATE -coverage)
target_link_options(sample4_unittest PRIVATE -coverage)

add_test(NAME sample4_unittest COMMAND sample4_unittest)

# add_executable(sample5_unittest
#               ${gtest_SOURCE_DIR}/samples/sample1.h
#               ${gtest_SOURCE_DIR}/samples/sample3-inl.h
#               ${gtest_SOURCE_DIR}/samples/sample5_unittest.cc)
# target_include_directories(sample4_unittest PRIVATE ${gtest_build_include_dirs})
# target_link_libraries(sample5_unittest gtest gtest_main)
# # target_compile_options(sample5_unittest PRIVATE -coverage)
# target_link_options(sample5_unittest PRIVATE -coverage)
# add_test(NAME sample5_unittest COMMAND sample5_unittest)

add_executable(sample6_unittest
              ${gtest_SOURCE_DIR}/samples/sample6_unittest.cc)
target_link_libraries(sample6_unittest gtest gtest_main)
target_compile_options(sample6_unittest PRIVATE -coverage)
target_link_options(sample6_unittest PRIVATE -coverage)
add_test(NAME sample6_unittest COMMAND sample6_unittest)

add_executable(sample7_unittest
              ${gtest_SOURCE_DIR}/samples/sample7_unittest.cc)
target_link_libraries(sample7_unittest gtest gtest_main)
target_compile_options(sample7_unittest PRIVATE -coverage)
target_link_options(sample7_unittest PRIVATE -coverage)
add_test(NAME sample7_unittest COMMAND sample7_unittest)

add_executable(sample8_unittest
              ${gtest_SOURCE_DIR}/samples/sample8_unittest.cc)
target_link_libraries(sample8_unittest gtest gtest_main)
target_compile_options(sample8_unittest PRIVATE -coverage)
target_link_options(sample8_unittest PRIVATE -coverage)
add_test(NAME sample8_unittest COMMAND sample8_unittest)

add_executable(sample9_unittest
              ${gtest_SOURCE_DIR}/samples/sample9_unittest.cc)
target_link_libraries(sample9_unittest gtest gtest_main)
target_compile_options(sample9_unittest PRIVATE -coverage)
target_link_options(sample9_unittest PRIVATE -coverage)
add_test(NAME sample9_unittest COMMAND sample9_unittest)

add_executable(utest_svm_table
              ${gtest_SOURCE_DIR}/svm_table/src/lib_svm_table_inline_impl.h
              ${gtest_SOURCE_DIR}/svm_table/src/lib_svm_table.c
              ${gtest_SOURCE_DIR}/svm_table/src/lib_svm_table.h
              ${gtest_SOURCE_DIR}/svm_table/unit_test/utest_lib_svm_table.cc)
target_include_directories(utest_svm_table PRIVATE ${gtest_build_include_dirs})
target_link_libraries(utest_svm_table gtest gtest_main)
target_compile_options(utest_svm_table PRIVATE -coverage)
target_link_options(utest_svm_table PRIVATE -coverage)
add_test(NAME utest_svm_table COMMAND utest_svm_table)
