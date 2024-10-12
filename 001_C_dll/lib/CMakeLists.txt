cmake_minimum_required(VERSION 3.18)
set(PROJECT_NAME my_lib)
# Set SRC INC folder location
set(LIB_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(LIB_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/inc)
# list all SRC INC file
file(GLOB LIB_SRC_FILES 
            "${LIB_SRC_DIR}/*.c")
file(GLOB LIB_INC_FILES 
            "${LIB_INC_DIR}/*.h")

# SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ./)
# set(CMAKE_CXX_VISIBILITY_PRESET )
# 1st param is dll name 2nd define dll or a
# Set Share library output directory
# SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
# Set static library output directory
# SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
ADD_LIBRARY(${PROJECT_NAME} STATIC 
                        ${LIB_SRC_FILES} 
                        ${LIB_INC_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC ${LIB_INC_DIR})
# message("${LIB_INC_DIR}")
# output name 
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES
                                LINKER_LANGUAGE C                                
                                ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}
                                # RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}
                                # LIBRARY_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}
                                # LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}
                                # LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}
                                # RUNTIME_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
                                # LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
                                OUTPUT_NAME "${PROJECT_NAME}"
                                PREFIX "")
message("lib output dir is ${PROJECT_BINARY_DIR}")