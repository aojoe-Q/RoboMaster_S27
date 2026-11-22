# =============================================================================
# S27_TEST_2 —— STM32H723VGTx 机械臂控制板固件
# 由 MDK-ARM/project.uvprojx 的编译配置翻译而来，可脱离 Keil 用 arm-none-eabi-gcc 构建。
#
# 配置（Windows / Linux 通用）：
#   cmake --preset Debug          # 生成 build/Debug
#   cmake --build --preset Debug  # 编译，产物 build/Debug/S27_TEST_2.elf|.hex|.bin
#
# 若 arm-none-eabi-gcc 不在 PATH：
#   cmake --preset Debug -DARM_TOOLCHAIN_PATH="<gcc 所在 bin 目录>"
#   或设置同名环境变量（Windows 下见 cmake/build.ps1）
# =============================================================================
cmake_minimum_required(VERSION 3.20)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Debug / Release" FORCE)
endif()

project(S27_TEST_2 C ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

message(STATUS "Build type : ${CMAKE_BUILD_TYPE}")
message(STATUS "C compiler : ${CMAKE_C_COMPILER}")

# -----------------------------------------------------------------------------
# 源文件（全部来自 project.uvprojx，arm_cortexM7lfdp_math.lib 换成源码/头文件不参与链接）
# -----------------------------------------------------------------------------
set(PROJECT_SOURCES
    # --- Application/Task ---
    Application/Task/Src/INS_Task.c
    Application/Task/Src/Control_Task.c
    Application/Task/Src/CAN_Task.c
    Application/Task/Src/Detect_Task.c
    # --- Application/APP ---
    APP/Arm_L_task.c
    APP/DM_motor.c
    APP/Moter_task.c
    APP/referee_usart_task.c
    APP/Arm_USB_task.c
    APP/arm_usb_protocol.c
    APP/ring_buffer.c
    # --- Application/Modules ---
    Modules/DJI_motor.c
    Modules/HT_motor.c
    Modules/LK_motor.c
    # --- Application/Compat ---
    APP/Compat/app_can.c
    APP/Compat/app_utils.c
    APP/Compat/app_uart.c
    APP/Compat/user_lib.c
    # --- Components/Device ---
    Components/Device/Src/Motor.c
    Components/Device/Src/Remote_Control.c
    Components/Device/Src/Referee_System.c
    Components/Device/Src/Image_Transmission.c
    Components/Device/Src/MiniPC.c
    Components/Device/Src/Bmi088.c
    # --- BSP ---
    BSP/Src/bsp_can.c
    BSP/Src/bsp_uart.c
    BSP/Src/bsp_adc.c
    BSP/Src/bsp_pwm.c
    BSP/Src/bsp_gpio.c
    BSP/Src/bsp_spi.c
    BSP/Src/bsp_tick.c
    BSP/Src/bsp_mcu.c
    BSP/Src/bsp_dwt.c
    BSP/Src/bsp_led.c
    # --- Components/Controller ---
    Components/Controller/Src/PID.c
    # --- Components/Algorithm ---
    Components/Algorithm/Src/Kalman_Filter.c
    Components/Algorithm/Src/Quaternion.c
    Components/Algorithm/Src/LPF.c
    Components/Algorithm/Src/Ramp.c
    Components/Algorithm/Src/RLS.c
    Components/Algorithm/Src/CRC.c
    # --- Application/User/Core ---
    Core/Src/main.c
    Core/Src/gpio.c
    Core/Src/freertos.c
    Core/Src/adc.c
    Core/Src/bdma.c
    Core/Src/dma.c
    Core/Src/fdcan.c
    Core/Src/memorymap.c
    Core/Src/spi.c
    Core/Src/tim.c
    Core/Src/usart.c
    Core/Src/stm32h7xx_it.c
    Core/Src/stm32h7xx_hal_msp.c
    Core/Src/stm32h7xx_hal_timebase_tim.c
    # --- Drivers/STM32H7xx_HAL_Driver ---
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usb.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_adc_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c
    # --- Drivers/CMSIS ---
    Core/Src/system_stm32h7xx.c
    # --- Middlewares/FreeRTOS ---
    Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    Middlewares/Third_Party/FreeRTOS/Source/list.c
    Middlewares/Third_Party/FreeRTOS/Source/queue.c
    Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    Middlewares/Third_Party/FreeRTOS/Source/timers.c
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
    Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F/port.c
    # --- User/SystemView (SEGGER RTT / SystemView) ---
    SystemView/SEGGER/SEGGER_RTT.c
    SystemView/SEGGER/SEGGER_SYSVIEW.c
    SystemView/Sample/FreeRTOSV10/SEGGER_SYSVIEW_FreeRTOS.c
    SystemView/Sample/FreeRTOSV10/Config/Cortex-M/SEGGER_SYSVIEW_Config_FreeRTOS.c
    # --- USB_DEVICE ---
    USB_DEVICE/App/usb_device.c
    USB_DEVICE/App/usbd_desc.c
    USB_DEVICE/App/usbd_cdc_if.c
    USB_DEVICE/Target/usbd_conf.c
    # --- Middlewares/USB_Device_Library ---
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
    # --- 启动文件（GCC 语法版本，含 .axi_data 拷贝）---
    Core/Startup/startup_stm32h723xx.s
)

# -----------------------------------------------------------------------------
# 头文件路径（来自 project.uvprojx 的 IncludePath）
# -----------------------------------------------------------------------------
set(PROJECT_INCLUDE_DIRS
    Core/Inc
    Drivers/STM32H7xx_HAL_Driver/Inc
    Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32H7xx/Include
    Drivers/CMSIS/Include
    Middlewares/Third_Party/FreeRTOS/Source/include
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F
    Application/Task/Inc
    Components/Device/Inc
    Components/Controller/Inc
    Components/Algorithm/Inc
    BSP/Inc
    SystemView/SEGGER/Syscalls
    SystemView/Sample/FreeRTOSV10
    SystemView/Sample/FreeRTOSV10/Patch
    SystemView/Config
    SystemView/SEGGER
    Middlewares/ST/ARM/DSP/Inc
    USB_DEVICE/App
    USB_DEVICE/Target
    Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    APP
    APP/Compat
    Modules
)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_INCLUDE_DIRS})

# -----------------------------------------------------------------------------
# 宏定义（与 Keil 工程一致）
# -----------------------------------------------------------------------------
target_compile_definitions(${PROJECT_NAME} PRIVATE
    USE_HAL_DRIVER
    STM32H723xx
    ARM_MATH_CM7
    __FPU_PRESENT=1U
    ARM_MATH_CHECK
)

# -----------------------------------------------------------------------------
# 链接
# -----------------------------------------------------------------------------
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32H723VGTX_FLASH.ld")

target_link_options(${PROJECT_NAME} PRIVATE
    -T${LINKER_SCRIPT}
    -Wl,-Map=${PROJECT_NAME}.map
    -Wl,--print-memory-usage
    -Wl,--cref
)

# binutils >= 2.39 对 RWX 段会报警告（本项目 .data/.axi_sram 布局使然，属正常），支持时关掉
# 注意：不能用 CheckLinkerFlag —— 交叉编译时 CMAKE_TRY_COMPILE_TARGET_TYPE 是 STATIC_LIBRARY，探测不了链接器
execute_process(COMMAND ${ARM_LD} --version
    OUTPUT_VARIABLE _LD_VERSION_STR OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
if(_LD_VERSION_STR MATCHES "\\)[ \t]*([0-9]+)\\.([0-9]+)")
    if(CMAKE_MATCH_1 GREATER 2 OR (CMAKE_MATCH_1 EQUAL 2 AND CMAKE_MATCH_2 GREATER_EQUAL 39))
        target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-warn-rwx-segments)
        message(STATUS "LD ${CMAKE_MATCH_1}.${CMAKE_MATCH_2}: 已关闭 RWX 段警告")
    endif()
endif()

# newlib-nano 中数学函数（arm_math.h 里部分内联实现会用到）
target_link_libraries(${PROJECT_NAME} PRIVATE m)

# 生成 .hex / .bin，并打印尺寸
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} -A -x $<TARGET_FILE:${PROJECT_NAME}>
    BYPRODUCTS
        ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex
        ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMENT "Generating HEX/BIN and size report"
    VERBATIM
)
