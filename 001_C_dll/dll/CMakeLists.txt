cmake_minimum_required(VERSION 3.18)
set(PROJECT_NAME my_dll)
# Set SRC INC folder location
set(DLL_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(DLL_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/inc)
# list all SRC INC file
file(GLOB DLL_SRC_FILES 
            "${DLL_SRC_DIR}/*.c")
file(GLOB  DLL_INC_FILES 
                    "${DLL_INC_DIR}/*.h")

# Add library add source and inc file to target
ADD_LIBRARY(${PROJECT_NAME} SHARED 
                        ${DLL_SRC_FILES}
                        ${DLL_INC_FILES})
# message("dll SRC file is  ${DLL_SRC_FILES}")
# message("dll INC file is  ${DLL_INC_FILES}")
# add INC file to target
target_include_directories(${PROJECT_NAME} PUBLIC ${DLL_INC_DIR})
# INSTALL(TARGETS ${PROJECT_NAME})
# Set target property
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE C
                                      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}
                                    #   LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}
                                    #   ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}
                                      # RUNTIME_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
                                      # LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
                                      OUTPUT_NAME "${PROJECT_NAME}"
                                      PREFIX "")
