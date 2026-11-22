/*
 * Linker script for STM32H723VGTx (1024K FLASH / 128K DTCM / 128K AXI SRAM)
 * 由 MDK-ARM\COD_H7_Template\COD_H7_Template.sct 翻译而来，供 arm-none-eabi-gcc 使用。
 *
 * 内存划分（与原 Keil 工程保持一致）：
 *   FLASH   0x08000000 1024K  代码 + 常量 + .data/.axi_data 的初值
 *   DTCMRAM 0x20000000  128K  .data / .bss / 栈 / 堆（CPU 快速访问；注意 DMA 不能访问 DTCM）
 *   RAM_D1  0x24000000  128K  .axi_data(.RamFunc) + .AXI_SRAM*（DMA 可访问）
 *   RAM_D2  0x30000000   32K  未使用（保留给外设缓冲）
 *   RAM_D3  0x38000000   16K  未使用
 *
 * 工程里用 __attribute__((section(".AXI_SRAM_DMA"))) 声明的 DMA 缓冲区，
 * 通过下面的 .axi_sram 段落到 AXI SRAM（32 字节对齐，满足 D-Cache 行宽要求）。
 */

ENTRY(Reset_Handler)

MEMORY
{
    FLASH   (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
    DTCMRAM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
    RAM_D1  (xrw) : ORIGIN = 0x24000000, LENGTH = 128K
    RAM_D2  (xrw) : ORIGIN = 0x30000000, LENGTH = 32K
    RAM_D3  (xrw) : ORIGIN = 0x38000000, LENGTH = 16K
}

/* 栈顶：DTCM 末端 */
_estack = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM);

/* 链接时检查堆栈是否放得下（FreeRTOS 用 heap_4，堆是独立数组，这里只是兜底） */
_Min_Heap_Size  = 0x200;   /* 512 B  */
_Min_Stack_Size = 0x800;   /* 2 KB   */

SECTIONS
{
    /* 中断向量表放最前面，Reset 后硬件从这里取初始 SP/PC */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)
        KEEP (*(.init))
        KEEP (*(.fini))
        . = ALIGN(4);
        _etext = .;
    } >FLASH

    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } >FLASH

    .ARM.extab :
    {
        . = ALIGN(4);
        *(.ARM.extab* .gnu.linkonce.armextab.*)
        . = ALIGN(4);
    } >FLASH

    .ARM :
    {
        . = ALIGN(4);
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
        . = ALIGN(4);
    } >FLASH

    .preinit_array :
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
        . = ALIGN(4);
    } >FLASH

    .init_array :
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
        . = ALIGN(4);
    } >FLASH

    .fini_array :
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT(.fini_array.*)))
        KEEP (*(.fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);
        . = ALIGN(4);
    } >FLASH

    /* ---- 已初始化数据：运行在 DTCM，初值在 FLASH ---- */
    _sidata = LOADADDR(.data);

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } >DTCMRAM AT> FLASH

    /* ---- 需要放到 AXI SRAM 的已初始化数据（目前用于 .RamFunc）---- */
    _saxi_data_load = LOADADDR(.axi_data);

    .axi_data :
    {
        . = ALIGN(32);
        _saxi_data = .;
        *(.RamFunc)
        *(.RamFunc*)
        . = ALIGN(32);
        _eaxi_data = .;
    } >RAM_D1 AT> FLASH

    /* ---- 未初始化数据（BSS）---- */
    .bss (NOLOAD) :
    {
        . = ALIGN(8);
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(8);
        _ebss = .;
        __bss_end__ = _ebss;
    } >DTCMRAM

    /* ---- 显式放在 AXI SRAM 的 DMA 缓冲区（SUBALIGN(32)：每个变量都按 32 字节
           即 D-Cache 行宽对齐，避免 DMA 与 cache 争抢同一个 cache line）---- */
    .axi_sram (NOLOAD) : SUBALIGN(32)
    {
        . = ALIGN(32);
        __axi_sram_start__ = .;
        *(.AXI_SRAM)
        *(.AXI_SRAM_DMA)
        *(.AXI_SRAM*)
        *(.axi_sram)
        *(.axi_sram*)
        . = ALIGN(32);
        __axi_sram_end__ = .;
    } >RAM_D1

    /* ---- 堆栈空间：链接时检查剩余 RAM 是否够用 ---- */
    ._user_heap_stack (NOLOAD) :
    {
        . = ALIGN(8);
        PROVIDE ( end = . );
        PROVIDE ( _end = . );
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } >DTCMRAM

    .ARM.attributes 0 : { *(.ARM.attributes) }
}
