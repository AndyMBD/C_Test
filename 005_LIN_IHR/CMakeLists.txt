cmake_minimum_required(VERSION 3.18)
# 1.create a Project
set(PROJECT_NAME "005_LIN_IHR")
PROJECT(${PROJECT_NAME})
# set(PROJECT_BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build/bin)
# 2. add subdirectory and include directory
# add_subdirectory(lpf)
# include_directories(${PROJECT_SOURCE_DIR}/dll/inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
# 2. to get all source files
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src main_src)
# 3. create a executable file
add_executable(${PROJECT_NAME} ${main_src})

