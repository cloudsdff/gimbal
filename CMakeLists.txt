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
set(CMAKE_PROJECT_NAME f103_w1)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# ---- 用户自定义：启用 C++17（任务01 工程要求 1） ----
enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-exceptions -fno-rtti -fno-threadsafe-statics")
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -Wall -Wextra")

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    app/app_main.cpp
    bsp/bsp_uart.cpp
    common/log.cpp
    common/rtos_hooks.c
    bsp/bsp_i2c.cpp
    bsp/bsp_i2c_soft.cpp
    bsp/bsp_pwm.cpp
    bsp/bsp_bluetooth.cpp
    # bsp/bsp_can.cpp   ← 任务02 阶段一不需要 CAN，先不编译（源码保留，后续要 CAN 时取消注释）
    modules/mpu6050/mpu6050.cpp
    modules/attitude/attitude.cpp
    modules/servo/servo.cpp
    modules/gimbal/gimbal.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_SOURCE_DIR}/app
    ${CMAKE_SOURCE_DIR}/bsp
    ${CMAKE_SOURCE_DIR}/common
    ${CMAKE_SOURCE_DIR}/modules
    ${CMAKE_SOURCE_DIR}/modules/mpu6050
    ${CMAKE_SOURCE_DIR}/modules/attitude
    ${CMAKE_SOURCE_DIR}/modules/servo
    ${CMAKE_SOURCE_DIR}/modules/gimbal
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

# ?? CAN ??:???? A ??(?? 0x123),? -DCAN_NODE_B=ON ?? B ??(?? 0x456)
option(CAN_NODE_B "Build firmware for CAN node B" OFF)
if(CAN_NODE_B)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE CAN_NODE_B=1)
endif()
if(LOG_OVERFLOW_TEST)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE LOG_OVERFLOW_TEST=1)
endif()

if(STACK_TEST_SMALL)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE STACK_TEST_SMALL=1)
endif()
