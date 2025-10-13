cmake_minimum_required(VERSION 3.16)

project(googletest_Sample)
set(GOOGLETEST_VERSION 1.16.0)

include(CTest)
include(CMakeDependentOption)
include(GNUInstallDirs)
# Note that googlemock target already builds googletest.
option(BUILD_GMOCK "Builds the googlemock subproject" OFF)
option(INSTALL_GTEST "Enable installation of googletest. (Projects embedding googletest may want to turn this OFF.)" OFF)
option(GTEST_HAS_ABSL "Use Abseil and RE2. Requires Abseil and RE2 to be separately added to the build." OFF)
option(GCOVR_OPTION "Use shared (DLL) run-time lib even when Google Test is built as static lib." ON)

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
enable_testing()


if (GCOVR_OPTION)
    find_program(GCOVR gcovr REQUIRED)
    message (STATUS "GCOVR found: ${GCOVR}")
    # gcov compile options
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-arcs -ftest-coverage")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
    
    # message(STATUS "${GCOVR}")
    # same function as ctest -T coverage
    # ninja coverage
    add_custom_target(coverage
        COMMAND ${CMAKE_CTEST_COMMAND} -T coverage
        COMMENT "Running coverage analysis"
    )

    add_custom_target(coverage_report
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/coverage_reports
    COMMAND ${GCOVR} --root ${CMAKE_SOURCE_DIR} 
        --html --html-details -o ${CMAKE_BINARY_DIR}/coverage_reports/coverage.html
    COMMAND ${CMAKE_COMMAND} -E echo "Opening coverage report..."
    COMMAND cmd /C start "" "${CMAKE_BINARY_DIR}/coverage_reports/coverage.html"
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Generating and opening HTML coverage report..."
    )
endif()