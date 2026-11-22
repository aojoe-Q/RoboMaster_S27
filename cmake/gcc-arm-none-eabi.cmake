# =============================================================================
# CMake toolchain file: arm-none-eabi-gcc (Cortex-M7, hard float, STM32H723)
# -----------------------------------------------------------------------------
# 用法（二选一）：
#   1) 把 arm-none-eabi-gcc 所在目录加进 PATH（Linux: /usr/bin 一般已有）；
#   2) 指定工具链目录，例如 Windows 下 CubeIDE 自带的：
#      E:/STM32 IDE/STM32CubeIDE_2.1.1/STM32CubeIDE/plugins/
#        com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740/tools/bin
#      用法：cmake --preset Debug -DARM_TOOLCHAIN_PATH="<上面的目录>"
#      或设置同名环境变量 ARM_TOOLCHAIN_PATH。
# =============================================================================

set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(TOOLCHAIN_PREFIX "arm-none-eabi-" CACHE STRING "GCC toolchain binary prefix")

if(NOT ARM_TOOLCHAIN_PATH AND DEFINED ENV{ARM_TOOLCHAIN_PATH})
    file(TO_CMAKE_PATH "$ENV{ARM_TOOLCHAIN_PATH}" ARM_TOOLCHAIN_PATH)
endif()

if(ARM_TOOLCHAIN_PATH)
    file(TO_CMAKE_PATH "${ARM_TOOLCHAIN_PATH}" ARM_TOOLCHAIN_PATH)
endif()

# Windows 上 gcc 等程序带 .exe 后缀，必须给 CMake 完整可执行文件名
if(CMAKE_HOST_WIN32)
    set(_ARM_EXE ".exe")
else()
    set(_ARM_EXE "")
endif()

# 在 ARM_TOOLCHAIN_PATH（或 PATH）里定位某个工具
function(_arm_find_tool _var _name)
    if(ARM_TOOLCHAIN_PATH)
        find_program(${_var} ${_name} HINTS "${ARM_TOOLCHAIN_PATH}" NO_DEFAULT_PATH NO_CACHE)
    endif()
    if(NOT ${_var})
        find_program(${_var} ${_name} NO_CACHE)
    endif()
    set(${_var} "${${_var}}" PARENT_SCOPE)
endfunction()

_arm_find_tool(ARM_GCC     "${TOOLCHAIN_PREFIX}gcc${_ARM_EXE}")
if(NOT ARM_GCC)
    message(FATAL_ERROR
        "未找到 ${TOOLCHAIN_PREFIX}gcc。\n"
        "请把它加入 PATH，或传入 -DARM_TOOLCHAIN_PATH=<bin 目录> / 设置环境变量 ARM_TOOLCHAIN_PATH。")
endif()

_arm_find_tool(ARM_GXX      "${TOOLCHAIN_PREFIX}g++${_ARM_EXE}")
_arm_find_tool(ARM_GCC_ASM  "${TOOLCHAIN_PREFIX}gcc${_ARM_EXE}")
_arm_find_tool(ARM_LD       "${TOOLCHAIN_PREFIX}ld${_ARM_EXE}")
_arm_find_tool(ARM_AR       "${TOOLCHAIN_PREFIX}ar${_ARM_EXE}")
_arm_find_tool(ARM_RANLIB   "${TOOLCHAIN_PREFIX}ranlib${_ARM_EXE}")
_arm_find_tool(ARM_OBJCOPY  "${TOOLCHAIN_PREFIX}objcopy${_ARM_EXE}")
_arm_find_tool(ARM_OBJDUMP  "${TOOLCHAIN_PREFIX}objdump${_ARM_EXE}")
_arm_find_tool(ARM_SIZE     "${TOOLCHAIN_PREFIX}size${_ARM_EXE}")
_arm_find_tool(ARM_READELF  "${TOOLCHAIN_PREFIX}readelf${_ARM_EXE}")

get_filename_component(ARM_TOOLCHAIN_BIN "${ARM_GCC}" DIRECTORY)
message(STATUS "ARM toolchain: ${ARM_TOOLCHAIN_BIN} (${ARM_GCC})")

set(CMAKE_C_COMPILER   "${ARM_GCC}")
set(CMAKE_CXX_COMPILER "${ARM_GXX}")
set(CMAKE_ASM_COMPILER "${ARM_GCC_ASM}")
set(CMAKE_AR           "${ARM_AR}")
set(CMAKE_RANLIB       "${ARM_RANLIB}")
set(CMAKE_OBJCOPY      "${ARM_OBJCOPY}")
set(CMAKE_OBJDUMP      "${ARM_OBJDUMP}")
set(CMAKE_SIZE         "${ARM_SIZE}")
set(CMAKE_READELF      "${ARM_READELF}")

set(CMAKE_EXECUTABLE_SUFFIX_ASM ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C   ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX ".elf")

# 交叉编译时不要去目标机上跑 try_compile 出来的程序
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# ----------------------------------------------------------------------------
# MCU 参数：STM32H723VGTx / Cortex-M7 / 双精度 FPU（与 Keil 工程 FPU3 = DFPU 一致）
# ----------------------------------------------------------------------------
set(TARGET_FLAGS "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb")

set(CMAKE_C_FLAGS   "${TARGET_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage")
set(CMAKE_ASM_FLAGS "${TARGET_FLAGS} -x assembler-with-cpp -MMD -MP")

set(CMAKE_C_FLAGS_DEBUG   "-Og -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS} --specs=nano.specs -Wl,--gc-sections")

# 交叉编译时只在工具链里找头文件/库，程序仍在主机 PATH 里找
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
