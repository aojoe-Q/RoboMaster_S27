# P0/P1 问题修复总结

修复日期：2026-11-11  
分支：fix/p0-uart-dma-buffer

## 修复的问题

### P0-1: UART DMA 双缓冲配置错误

**问题描述：**
- `SBUS_MultiRx_Buf[2][18]` 单缓冲 18 字节，但重装时使用 36 字节
- `Image_Trans_MultiRx_Buff[2][39]` 单缓冲 39 字节，但重装时使用 78 字节
- `RxXferSize` 初始化为 `DataLength * 2`，但 NDTR 设置为 `DataLength`
- 在双缓冲模式下，NDTR 表示当前缓冲区的传输数量，不是两个缓冲区之和

**修复内容：**
1. `BSP/Src/bsp_uart.c:77` - 修正 `RxXferSize = DataLength`（去掉 *2）
2. `BSP/Src/bsp_uart.c:128,149` - 修正 SBUS 重装为 `SBUS_RX_BUF_NUM`（去掉 *2）
3. `BSP/Src/bsp_uart.c:193,210` - 修正图传重装为 `IMAGE_TRANS_RX_LENGTH`（去掉 *2）
4. `BSP/Src/bsp_uart.c:225,240` - 修正裁判系统重装为 `REFEREE_RXFRAME_LENGTH`（去掉 *2）

**影响：**
- 消除了 DMA 越界写入的风险
- 确保双缓冲切换时长度计算正确
- 不改变正常帧的解析逻辑

### P0-2: 图传解析边界检查缺失

**问题描述：**
- `Image_Transmission_Info_Update()` 在验证长度之前就调用 CRC16，可能读取越界内存
- 从包头提取的长度未经验证就用于访问缓冲区
- Size 参数未传递给解析函数

**修复内容：**
1. `Components/Device/Inc/Image_Transmission.h:165` - 添加 Size 参数到函数声明
2. `Components/Device/Src/Image_Transmission.c:26-71` - 重构解析流程：
   - 添加 Size 参数
   - 先检查最小长度（FrameHeader_Length）
   - 提取声明的长度到局部变量
   - 验证总帧长不超过 Size 和 IMAGE_TRANS_RX_LENGTH
   - 再进行 CRC16 校验
   - VT13 分支也增加 21 字节长度检查
3. `BSP/Src/bsp_uart.c:188,207` - 更新调用点传入 Size

**影响：**
- 防止 CRC 校验时读取越界内存
- 拒绝畸形长度的报文
- 保持合法报文的解析结果一致

### P1: 遥控超时监视竞态条件

**问题描述：**
- `Remote_Message_Moniter()` 在任务中每 1ms 调用，修改 `remote_ctrl`
- `SBUS_TO_RC()` 在 UART ISR 中更新 `remote_ctrl`
- 两个写入方之间无同步保护，可能导致：
  - 超时清零覆盖刚接收的新帧
  - online_cnt 更新被部分覆盖

**修复内容：**
1. `Components/Device/Src/Remote_Control.c:5-6` - 添加 FreeRTOS 头文件
2. `Components/Device/Src/Remote_Control.c:89-108` - 在 `Remote_Message_Moniter()` 中添加临界区保护：
   - `taskENTER_CRITICAL()` 在函数开始
   - `taskEXIT_CRITICAL()` 在函数结束
   - 与 `APP_CopyRemoteToRCData()` 的临界区保持一致

**影响：**
- 消除监视函数与 ISR 之间的竞态
- 保持超时判断与数据快照的一致性
- 不改变超时阈值和逻辑

## 验证建议

### 单元测试
- 运行 `python tools/selftest/run_selftest.py` - USB 协议测试
- 运行 `python tools/selftest/run_control_selftest.py` - 控制逻辑测试
- 运行 `python tools/selftest/full_arm_build.py` - 完整编译测试

### 硬件测试
1. **UART DMA 验证：**
   - 串口首帧接收
   - 连续多帧接收
   - 单帧恰好填满缓冲区（18/39 字节）
   - M0/M1 轮换测试
   - 缓冲区边界哨兵检查

2. **图传解析验证：**
   - 合法 39 字节自定义数据帧
   - 合法 21 字节遥控帧
   - 合法 21 字节 VT13 帧
   - 畸形长度（超过 39/21）
   - CRC8 正确但长度非法
   - 短于 5 字节的半包

3. **遥控超时验证：**
   - 正常遥控接收
   - 遥控失联后自动清零
   - 失联临界点（online_cnt = 50）的行为
   - 失联后恢复

## 未修改的内容

按照文档建议，以下内容保持不变：
- 波特率、帧格式、遥控映射
- USB 协议版本和字段顺序
- CAN ID 和总线拓扑
- 控制周期（4/8/10 ms）
- PD 参数、零点、限位
- 故障保持/无力策略

## 后续工作

根据评审文档，建议按顺序进行：
1. ✅ **已完成：P0/P1 边界修复**
2. 修正编译检查脚本（P2）
3. 等价拆分控制任务（阶段 C）
4. 性能优化（阶段 D）
5. 功能变更（阶段 E，需单独定义验收）
