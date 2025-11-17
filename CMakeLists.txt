
cmake_minimum_required(VERSION 3.10)

# Compiler selection options (must be set before project() call)
option(USE_CLANG "Use clang/clang++ compiler" OFF)
option(USE_GCC "Use gcc/g++ compiler" ON)

# Set compilers based on options (before project call)
if(USE_CLANG AND USE_GCC)
    message(FATAL_ERROR "Cannot specify both USE_CLANG and USE_GCC. Choose one or neither.")
elseif(USE_CLANG)
    message(STATUS "Forcing clang compiler")
    set(CMAKE_C_COMPILER clang)
    set(CMAKE_CXX_COMPILER clang++)
elseif(USE_GCC)
    message(STATUS "Forcing GCC compiler")
    set(CMAKE_C_COMPILER gcc)
    set(CMAKE_CXX_COMPILER g++)
else()
    message(STATUS "Using default system compiler")
endif()

project(xcplib VERSION 0.9.2 LANGUAGES C CXX)

# Find required packages
find_package(Threads REQUIRED)


# Set default generator for Windows to Visual Studio 2022
if(WIN32 AND NOT DEFINED CMAKE_GENERATOR)
    set(CMAKE_GENERATOR "Visual Studio 17 2022")
endif()

# Enable compile commands for VS Code IntelliSense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set C and C++ standards globally
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# Set C++ standard based on platform
if(WIN32)
    set(CMAKE_CXX_STANDARD 20)
    message(STATUS "Using C++20 for Windows")
else()
    set(CMAKE_CXX_STANDARD 17)
    message(STATUS "Using C++17 for non-Windows platforms")
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable test
if(NOT DEFINED BUILD_TEST)
    set(BUILD_TEST TRUE)
endif()
if(BUILD_TEST)
    message(STATUS "Build test targets enabled")    
endif()

# Platform detection for library selection
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(STATUS "64-bit platform")
    set(PLATFORM_64BIT TRUE)
else()
    message(STATUS "32-bit platform")
    set(PLATFORM_32BIT TRUE)
endif()

# Platform-specific settings
if(WIN32)
   message(STATUS "Building for Windows")
   set(WINDOWS TRUE)
elseif(APPLE)
   message(STATUS "Building for macOS")
   set(MACOS TRUE)
elseif(UNIX)
    message(STATUS "Building for Unix/Linux")
    set(LINUX TRUE)
endif()
add_compile_options(-fdiagnostics-color=always)
# Compiler-specific settings for better compatibility
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    # Enable additional warnings for GCC and Clang
    set(COMMON_WARNING_FLAGS -pedantic -Wall)
    if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
        message(STATUS "Using GCC compiler")
    elseif(CMAKE_C_COMPILER_ID STREQUAL "Clang")
        message(STATUS "Using Clang compiler")
    endif()
elseif(CMAKE_C_COMPILER_ID STREQUAL "MSVC")
    message(STATUS "Using MSVC compiler")
    set(COMMON_WARNING_FLAGS /W3)
endif()
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/008_C_Learn)


