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
add_subdirectory( 04_C_Learn )

option(SAMPLE1_UNITTEST_OPTION "enable sample1_unittest." OFF)
option(SAMPLE2_UNITTEST_OPTION "enable sample2_unittest." OFF)
option(SAMPLE3_UNITTEST_OPTION "enable sample3_unittest." OFF)
option(SAMPLE4_UNITTEST_OPTION "enable sample4_unittest." OFF)
# option(SAMPLE5_UNITTEST_OPTION "enable sample5_unittest." ON)
option(SAMPLE6_UNITTEST_OPTION "enable sample6_unittest." OFF)
option(SAMPLE7_UNITTEST_OPTION "enable sample7_unittest." OFF)
option(SAMPLE8_UNITTEST_OPTION "enable sample8_unittest." OFF)
option(SAMPLE9_UNITTEST_OPTION "enable sample9_unittest." OFF)
option(UTEST_SVM_TABLE_OPTION "enable utest_svm_table."   OFF)
option(GSL_MAIN_OPTION "enable gsl_main."   OFF)

set(cmake_package_name GTest CACHE INTERNAL "")

# Where Google Test's .h files can be found.
set(gtest_build_include_dirs
  "${gtest_SOURCE_DIR}/include"
  "${gtest_SOURCE_DIR}/samples"
  "${gtest_SOURCE_DIR}/svm_table"
  "${gtest_SOURCE_DIR}/03_svm_table/src"
  "${gtest_SOURCE_DIR}/03_svm_table/unit_test"
  "${gtest_SOURCE_DIR}")
# include_directories(${gtest_build_include_dirs})
# message(STATUS "gtest_SOURCE_DIR: ${gtest_SOURCE_DIR}")
# message(STATUS "gtest_build_include_dirs: ${gtest_build_include_dirs}")
# add_library(gtest_main STATIC src/gtest_main.cc)
if(GSL_MAIN_OPTION)
add_executable(gsl_main 
              # ${gtest_SOURCE_DIR}/02_gsl/src/gsl_main.c
              ${gtest_SOURCE_DIR}/02_gsl/src/gsl_main.c)
target_include_directories(gsl_main PRIVATE ${gtest_build_include_dirs})
target_link_libraries(gsl_main ${GSL_LIBRARIES})
target_compile_options(gsl_main PRIVATE -coverage)
target_link_options(gsl_main PRIVATE -coverage)
create_vsc_launch_json(gsl_main)
endif()
# include(Dart)


enable_testing()
if(SAMPLE1_UNITTEST_OPTION)
  add_executable(sample1_unittest 
                ${gtest_SOURCE_DIR}/01_samples/sample1.cc
                ${gtest_SOURCE_DIR}/01_samples/sample1_unittest.cc)              
  target_link_libraries(sample1_unittest gtest gtest_main)
  target_compile_options(sample1_unittest PRIVATE -coverage)
  target_link_options(sample1_unittest PRIVATE -coverage)
  add_test(NAME sample1_unittest COMMAND sample1_unittest)
  create_vsc_launch_json(sample1_unittest)
  # 0(sample1_unittest)
#   message(STATUS "CMAKE_CURRENT_BINARY_DIR: ${CMAKE_CURRENT_BINARY_DIR}")
#   message(STATUS "CMAKE_EXECUTABLE_NAME: ${CMAKE_EXECUTABLE_NAME}")
#   message(STATUS "CMAKE_EXECUTABLE_SUFFIX: ${CMAKE_EXECUTABLE_SUFFIX}")
endif()

if(SAMPLE2_UNITTEST_OPTION)
add_executable(sample2_unittest 
              ${gtest_SOURCE_DIR}/01_samples/sample2.cc
              ${gtest_SOURCE_DIR}/01_samples/sample2_unittest.cc)
target_include_directories(sample2_unittest PRIVATE ${gtest_build_include_dirs})
target_link_libraries(sample2_unittest gtest gtest_main)
target_compile_options(sample2_unittest PRIVATE -coverage)
target_link_options(sample2_unittest PRIVATE -coverage)
add_test(NAME sample2_unittest COMMAND sample2_unittest)
create_vsc_launch_json(sample2_unittest)
endif()

if(SAMPLE3_UNITTEST_OPTION)
add_executable(sample3_unittest 
              ${gtest_SOURCE_DIR}/01_samples/sample3_unittest.cc)
target_include_directories(sample3_unittest PRIVATE ${gtest_build_include_dirs})
target_link_libraries(sample3_unittest gtest gtest_main)
target_compile_options(sample3_unittest PRIVATE -coverage)
target_link_options(sample3_unittest PRIVATE -coverage)
add_test(NAME sample3_unittest COMMAND sample3_unittest)
create_vsc_launch_json(sample3_unittest)
endif()

if(SAMPLE4_UNITTEST_OPTION)
add_executable(sample4_unittest 
              ${gtest_SOURCE_DIR}/01_samples/sample4.cc
              ${gtest_SOURCE_DIR}/01_samples/sample4_unittest.cc)
target_include_directories(sample4_unittest PRIVATE ${gtest_build_include_dirs})
target_link_libraries(sample4_unittest gtest gtest_main)
target_compile_options(sample4_unittest PRIVATE -coverage)
target_link_options(sample4_unittest PRIVATE -coverage)
add_test(NAME sample4_unittest COMMAND sample4_unittest)
create_vsc_launch_json(sample4_unittest)
endif()
# add_executable(sample5_unittest
#               ${gtest_SOURCE_DIR}/01_samples/sample1.h
#               ${gtest_SOURCE_DIR}/01_samples/sample3-inl.h
#               ${gtest_SOURCE_DIR}/01_samples/sample5_unittest.cc)
# target_include_directories(sample4_unittest PRIVATE ${gtest_build_include_dirs})
# target_link_libraries(sample5_unittest gtest gtest_main)
# # target_compile_options(sample5_unittest PRIVATE -coverage)
# target_link_options(sample5_unittest PRIVATE -coverage)
# add_test(NAME sample5_unittest COMMAND sample5_unittest)

if(SAMPLE6_UNITTEST_OPTION)
add_executable(sample6_unittest
              ${gtest_SOURCE_DIR}/01_samples/sample6_unittest.cc)
target_link_libraries(sample6_unittest gtest gtest_main)
target_compile_options(sample6_unittest PRIVATE -coverage)
target_link_options(sample6_unittest PRIVATE -coverage)
add_test(NAME sample6_unittest COMMAND sample6_unittest)
create_vsc_launch_json(sample6_unittest)
endif()

if(SAMPLE7_UNITTEST_OPTION)
add_executable(sample7_unittest
              ${gtest_SOURCE_DIR}/01_samples/sample7_unittest.cc)
target_link_libraries(sample7_unittest gtest gtest_main)
target_compile_options(sample7_unittest PRIVATE -coverage)
target_link_options(sample7_unittest PRIVATE -coverage)
add_test(NAME sample7_unittest COMMAND sample7_unittest)
create_vsc_launch_json(sample7_unittest)
endif()

if(SAMPLE8_UNITTEST_OPTION)
add_executable(sample8_unittest
              ${gtest_SOURCE_DIR}/01_samples/sample8_unittest.cc)
target_link_libraries(sample8_unittest gtest gtest_main)
target_compile_options(sample8_unittest PRIVATE -coverage)
target_link_options(sample8_unittest PRIVATE -coverage)
add_test(NAME sample8_unittest COMMAND sample8_unittest)
create_vsc_launch_json(sample8_unittest)
endif()

if(SAMPLE9_UNITTEST_OPTION)
add_executable(sample9_unittest
              ${gtest_SOURCE_DIR}/01_samples/sample9_unittest.cc)
target_link_libraries(sample9_unittest gtest gtest_main)
target_compile_options(sample9_unittest PRIVATE -coverage)
target_link_options(sample9_unittest PRIVATE -coverage)
add_test(NAME sample9_unittest COMMAND sample9_unittest)
create_vsc_launch_json(sample9_unittest)
endif()

if(UTEST_SVM_TABLE_OPTION)
add_executable(utest_svm_table
              ${gtest_SOURCE_DIR}/03_svm_table/src/lib_svm_table_inline_impl.h
              ${gtest_SOURCE_DIR}/03_svm_table/src/lib_svm_table.c
              ${gtest_SOURCE_DIR}/03_svm_table/src/lib_svm_table.h
              ${gtest_SOURCE_DIR}/03_svm_table/unit_test/utest_lib_svm_table.cc)
target_include_directories(utest_svm_table PRIVATE ${gtest_build_include_dirs})
target_link_libraries(utest_svm_table gtest gtest_main)
target_compile_options(utest_svm_table PRIVATE -coverage)
target_link_options(utest_svm_table PRIVATE -coverage)
add_test(NAME utest_svm_table COMMAND utest_svm_table)
create_vsc_launch_json(utest_svm_table)
endif()