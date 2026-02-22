cmake_minimum_required(VERSION 3.22)
# 让CMake检测编译器时只编译静态库，不生成可执行文件（避开Windows链接参数）
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
# 直接标记编译器可用，跳过检测
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

# ===================== 强制指定ARM工具链 =====================
# 你的STM32CubeCLT编译器路径
set(TOOLCHAIN_PREFIX "C:/ST/STM32CubeCLT_1.20.0/GNU-tools-for-STM32/bin/arm-none-eabi-")
set(CMAKE_C_COMPILER    ${TOOLCHAIN_PREFIX}gcc.exe)
set(CMAKE_CXX_COMPILER  ${TOOLCHAIN_PREFIX}g++.exe)
set(CMAKE_ASM_COMPILER  ${TOOLCHAIN_PREFIX}gcc.exe)
set(CMAKE_OBJCOPY       ${TOOLCHAIN_PREFIX}objcopy.exe)
set(CMAKE_SIZE          ${TOOLCHAIN_PREFIX}size.exe)

# ===================== 声明嵌入式系统（非Windows）=====================
set(CMAKE_SYSTEM_NAME Generic)  # 无操作系统
set(CMAKE_SYSTEM_PROCESSOR arm) # 目标架构是ARM

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
set(CMAKE_PROJECT_NAME LidarModule)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

set(CPU_PARAMETERS
        -mcpu=cortex-m3
        -mthumb
        -mfloat-abi=soft
        -ffunction-sections
        -fdata-sections
)

# Core project settings
project(${CMAKE_PROJECT_NAME})


add_compile_options(
        ${CPU_PARAMETERS}  # 复用已定义的参数，避免重复写
        -Wall
        -Wextra
        -O0                      # 调试模式，发布可改-O2
        -std=gnu11
)

add_definitions(
        -DSTM32F103xB    # 芯片型号
        -DUSE_HAL_DRIVER # 启用HAL库
        -DDEBUG          # 调试模式
)


set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F103XX_FLASH.ld)

message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Core/Src/lidar.c
        Core/Inc/lidar.h)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined library search paths
        #Core/Src/lidar.c  # 注释：这里是源码，不是库路径，保留注释即可
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user sources here
        Core/Src/lidar.c
        C:/ST/STM32CubeCLT_1.20.0/GNU-tools-for-STM32/lib/gcc/arm-none-eabi/13.3.1/thumb/v7-m
        C:/ST/STM32CubeCLT_1.20.0/GNU-tools-for-STM32/arm-none-eabi/lib/thumb/v7-m
)

# ========== 给目标加指令集参数 ==========
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
        ${CPU_PARAMETERS}  # 复用参数，更简洁
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined include paths
        Core/Inc
        Core/Src/lidar.c
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# ========== 补充链接选项 ==========
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
        ${CPU_PARAMETERS}          # 指令集参数
        -T${LINKER_SCRIPT}         # 链接脚本
        -Wl,--gc-sections          # 移除未使用的段
        -Wl,-Map=${CMAKE_PROJECT_NAME}.map  # 生成map文件
        -nostartfiles              # 不链接系统启动文件
        -Wl,--undefined=_init -Wl,--entry=Reset_Handler
        -lgcc -lc -lm -lnosys               # 不链接系统标准库
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx

        # Add user defined libraries
)