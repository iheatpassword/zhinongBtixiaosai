cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME C30D_CubeMX)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# ---------------------------------------------------------------------------
# Application + board layer
#
# Everything lives under Core/App and Core/BSP so that Core/Inc and Core/Src
# stay exactly as CubeMX generated them.  Recursive globs with CONFIGURE_DEPENDS
# keep adding files a matter of dropping them into BSP/<device>/.
# ---------------------------------------------------------------------------
file(GLOB_RECURSE APP_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/Core/App/*.c)
file(GLOB_RECURSE BSP_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/Core/BSP/*.c)

file(GLOB BSP_DIRS CONFIGURE_DEPENDS LIST_DIRECTORIES true
     ${CMAKE_CURRENT_SOURCE_DIR}/Core/BSP/*)

set(BSP_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/Core/BSP)
foreach(_d ${BSP_DIRS})
    if(IS_DIRECTORY ${_d})
        list(APPEND BSP_INCLUDE_DIRS ${_d})
    endif()
endforeach()

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${APP_SRC}
    ${BSP_SRC}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/App
    ${BSP_INCLUDE_DIRS}
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
