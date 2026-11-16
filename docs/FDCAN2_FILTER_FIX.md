# FDCAN2 硬件过滤器补齐

**位置**：`BSP/Src/bsp_can.c:76-79`

**问题**：
- 硬件过滤器只允许 0x212/0x213
- 代码中有对 0x207/0x208/0x209/0x210 的处理（CC_data float 数据）
- 这些报文被硬件层拒收，无法进入 APP_CAN_RxHandler()

**协议映射**：
```
FDCAN2 (0x200-0x2FF 范围)：
  0x212     - app_board_rc_data 通道 0-2 和拨杆
  0x213     - app_board_rc_data 通道 3、键盘、鼠标
  0x207     - CC_Key_grip / J4
  0x208     - CC float data 1-2
  0x209     - CC float data 3-4
  0x210     - DJ float data 1-2
```

**修复**：
```c
else if (i == 1U) {
    /* FDCAN2：双板 0x207/0x208/0x209/0x210（CC float）和 0x212/0x213（遥控） */
    filter.FilterID1 = 0x200U;      // 基础 ID
    filter.FilterID2 = 0x700U;      // 掩码允许 0x200..0x2FF 范围
}
```

**掩码解释**：
- 0x700 = 0111 0000 0000b
- 接受 bit[10:8] 为 0 的所有标准 ID
- 范围：0x200-0x2FF（足以覆盖所有双板协议）

**验证**：
- StdFiltersNbr = 1（已正确配置）
- RxFifo1ElmtsNbr = 8（FDCAN2 使用 RxFIFO1）
- 所有 FDCAN2 报文现在都能通过硬件层

