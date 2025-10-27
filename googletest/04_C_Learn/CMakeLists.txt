cmake_minimum_required(VERSION 3.13)
project(c_learn VERSION ${GOOGLETEST_VERSION} LANGUAGES CXX C)
include(CTest)
option(C_LEARN_OPTION "enable c learn." ON)

set(cmake_package_name GTest CACHE INTERNAL "")

# Where Google Test's .h files can be found.
set(c_learn_build_include_dirs
  "${c_learn_SOURCE_DIR}/include"
  "${c_learn_SOURCE_DIR}")
# include_directories(${c_learn_build_include_dirs})
# message(STATUS "gtest_SOURCE_DIR: ${gtest_SOURCE_DIR}")
# message(STATUS "gtest_build_include_dirs: ${gtest_build_include_dirs}")
# add_library(gtest_main STATIC src/gtest_main.cc)
enable_testing()
if(C_LEARN_OPTION)
  add_executable(c_learn_001 
                ${c_learn_SOURCE_DIR}/src/main.c)
  target_link_libraries(c_learn_001 gtest gtest_main)
  target_compile_options(c_learn_001 PRIVATE -coverage)
  target_link_options(c_learn_001 PRIVATE -coverage)
  add_test(NAME c_learn_001 COMMAND c_learn_001)
  create_vsc_launch_json(c_learn_001)
  # 0(c_learn_001)
#   message(STATUS "CMAKE_CURRENT_BINARY_DIR: ${CMAKE_CURRENT_BINARY_DIR}")
#   message(STATUS "CMAKE_EXECUTABLE_NAME: ${CMAKE_EXECUTABLE_NAME}")
#   message(STATUS "CMAKE_EXECUTABLE_SUFFIX: ${CMAKE_EXECUTABLE_SUFFIX}")
endif()
