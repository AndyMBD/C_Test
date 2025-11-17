cmake_minimum_required(VERSION 3.18)
# 1.create a Project
set(PROJECT_NAME "008_C_Learn")
PROJECT(${PROJECT_NAME})
# set(PROJECT_BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build/bin)
# 2. add subdirectory and include directory
# add_subdirectory(01_CAN)
# add_subdirectory(lib)
# include_directories(${PROJECT_SOURCE_DIR}/dll/inc)
# include_directories(${PROJECT_SOURCE_DIR}/lib/inc)
# 2. to get all source files
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src demo_src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/01_CAN)
# target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)
# target_sources(${PROJECT_NAME} PRIVATE
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c
#     ${CMAKE_CURRENT_SOURCE_DIR}/01_CAN/header.h
# )
# 3. create a executable file
add_executable(${PROJECT_NAME} ${demo_src})
# 4. Set Project Properties

# SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES
# 										RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
# 4. Set Project Properties
target_include_directories(${PROJECT_NAME} PUBLIC 
											${CMAKE_CURRENT_SOURCE_DIR}/01_CAN)
# target_link_libraries(${PROJECT_NAME} PUBLIC 
# 									  iqmath_dll)
# target_include_directories(${PROJECT_NAME} PUBLIC 
# 											${PROJECT_SOURCE_DIR}/lib/inc)
# target_link_libraries(${PROJECT_NAME} PUBLIC 
# 									 my_lib)
message(STATUS "--------------------------------------------")
message(STATUS "PROJECT_SOURCE_DIR " ${PROJECT_SOURCE_DIR})
message(STATUS "demo_src " ${demo_src})
message(STATUS "01_CAN_INC " ${01_CAN_INC})
message(STATUS "CMAKE_CURRENT_SOURCE_DIR " ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS "--------------------------------------------")
# 5. link library, include directory
