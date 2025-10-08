# Note: CMake support is community-based. The maintainers do not use CMake
# internally.

cmake_minimum_required(VERSION 3.16)

project(googletest-distribution)
set(GOOGLETEST_VERSION 1.16.0)

if(NOT CYGWIN AND NOT MSYS AND NOT ${CMAKE_SYSTEM_NAME} STREQUAL QNX)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
enable_testing()

include(CMakeDependentOption)
include(GNUInstallDirs)
# Note that googlemock target already builds googletest.
option(BUILD_GMOCK "Builds the googlemock subproject" OFF)
option(INSTALL_GTEST "Enable installation of googletest. (Projects embedding googletest may want to turn this OFF.)" OFF)
option(GTEST_HAS_ABSL "Use Abseil and RE2. Requires Abseil and RE2 to be separately added to the build." OFF)

find_package(GSL REQUIRED)
message(STATUS "GSL found: ${GSL_FOUND}")
message(STATUS "GSL_INCLUDE_DIRS: ${GSL_INCLUDE_DIRS}")
message(STATUS "GSL_LIBRARIES: ${GSL_LIBRARIES}")

find_package(GTest REQUIRED)
message(STATUS "GTEST_FOUND:        ${GTEST_FOUND}")
message(STATUS "GTEST_INCLUDE_DIRS: ${GTEST_INCLUDE_DIRS}")
message(STATUS "GTEST_LIBRARIES:    ${GTEST_BOTH_LIBRARIES}")
message(STATUS "CMAKE_C_COMPILER:    ${CMAKE_C_COMPILER}")
message(STATUS "CMAKE_CXX_COMPILER:    ${CMAKE_CXX_COMPILER}")

if(GTEST_HAS_ABSL)
  if(NOT TARGET absl::base)
    find_package(absl REQUIRED)
  endif()
  if(NOT TARGET re2::re2)
    find_package(re2 REQUIRED)
  endif()
endif()

if(BUILD_GMOCK)
  add_subdirectory( googlemock )
  message(STATUS "googlemock added")
else()
  add_subdirectory( googletest )
  message(STATUS "googletest added")
endif()



# gcov相关编译选项
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-arcs -ftest-coverage")

# SET(GCC_COVERAGE_COMPILE_FLAGS "-g -O0 -coverage -fprofile-arcs -ftest-coverage")
# SET(GCC_COVERAGE_LINK_FLAGS "-coverage -lgcov")
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
# SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")


