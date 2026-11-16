# P0/P1/P2 修复验证清单

修复日期：2026-11-11  
验证人：待填写  
验证日期：待填写

## 代码审查验证 ✅

### P0-1: UART DMA 双缓冲配置
- [x] `bsp_uart.c:77` - RxXferSize 已改为 `DataLength`（去掉 *2）
- [x] `bsp_uart.c:128` - SBUS DMA 重装长度改为 `SBUS_RX_BUF_NUM`
- [x] `bsp_uart.c:149` - SBUS DMA 重装长度改为 `SBUS_RX_BUF_NUM`
- [x] `bsp_uart.c:193` - 图传 DMA 重装长度改为 `IMAGE_TRANS_RX_LENGTH`
- [x] `bsp_uart.c:210` - 图传 DMA 重装长度改为 `IMAGE_TRANS_RX_LENGTH`
- [x] `bsp_uart.c:225` - 裁判系统 DMA 重装长度改为 `REFEREE_RXFRAME_LENGTH`
- [x] `bsp_uart.c:240` - 裁判系统 DMA 重装长度改为 `REFEREE_RXFRAME_LENGTH`

### P0-2: 图传解析边界检查
- [x] `Image_Transmission.h:85` - 函数签名增加 `uint16_t Size` 参数
- [x] `Image_Transmission.c:26` - 实现增加 Size 参数
- [x] `Image_Transmission.c:32-34` - 最小长度检查（FrameHeader_Length）
- [x] `Image_Transmission.c:49-51` - 总帧长度边界检查
- [x] `Image_Transmission.c:73-75` - VT13 分支长度检查
- [x] `bsp_uart.c:188` - 调用点传入 Size 参数
- [x] `bsp_uart.c:207` - 调用点传入 Size 参数

### P1: 遥控超时监视竞态
- [x] `Remote_Control.c:16` - 包含 FreeRTOS.h
- [x] `Remote_Control.c:90` - 添加 taskENTER_CRITICAL()
- [x] `Remote_Control.c:110` - 添加 taskEXIT_CRITICAL()

### P2: 编译检查脚本
- [x] `armclang_check.py:10-12` - 新增 import (json, hashlib, datetime)
- [x] `armclang_check.py:36-48` - 从 .dep 动态提取文件列表
- [x] `armclang_check.py:45` - 排除 Arm_R_task.c
- [x] `armclang_check.py:60-68` - 创建构建记录结构
- [x] `armclang_check.py:78-82` - 计算源文件 SHA256 哈希
- [x] `armclang_check.py:91-98` - 记录每个文件的编译结果
- [x] `armclang_check.py:113-117` - 保存 JSON 构建记录

---

## 编译验证 ⏳

### 语法检查
```bash
cd C:\Users\lenovo\Desktop\S27_TEST_2
python -m py_compile tools/selftest/armclang_check.py
```
- [ ] Python 语法无错误

### 单元测试
```bash
cd tools/selftest
python run_selftest.py
```
- [ ] 所有测试通过
- [ ] 无新增失败用例

```bash
python run_control_selftest.py
```
- [ ] encoding 测试通过
- [ ] motion 测试通过
- [ ] enable 测试通过
- [ ] retry 测试通过
- [ ] fifo 测试通过
- [ ] bus 测试通过
- [ ] queue 测试通过

### 编译检查（更新后）
```bash
python armclang_check.py
```
- [ ] 不再尝试编译 Arm_R_task.c
- [ ] 输出 "Found XX project C files in .dep"
- [ ] 输出 "Excluded: ..\APP\Arm_R_task.c"
- [ ] 所有文件编译成功（rc=0, obj ok）
- [ ] 生成 `build/armclang_obj/build_record.json`
- [ ] 记录包含时间戳、编译器、标志
- [ ] 记录包含文件哈希和告警

### 完整构建
```bash
python full_arm_build.py
```
- [ ] 107 个编译单元成功
- [ ] 0 编译错误
- [ ] 链接成功
- [ ] 生成 .hex 和 .bin 文件

---

## 硬件验证 ⏳

### UART DMA 测试（USART5 - SBUS 遥控）
**测试环境：** 
- 遥控器型号：
- 波特率：100000
- 帧格式：8E1
- 预期帧长：18 字节

**测试用例：**
- [ ] TC-01: 首帧接收正确（冷启动后第一帧）
- [ ] TC-02: 连续多帧接收正确（至少 100 帧）
- [ ] TC-03: 单帧填满缓冲区（18 字节）
- [ ] TC-04: M0 缓冲区数据正确
- [ ] TC-05: M1 缓冲区数据正确
- [ ] TC-06: M0/M1 自动切换正常
- [ ] TC-07: 无缓冲区越界（使用哨兵字节验证）
- [ ] TC-08: DMA 计数器重装正确（18 而非 36）

**失败判据：**
- 遥控数据解析错误
- 通道值异常
- 缓冲区哨兵字节被破坏
- DMA 传输错误标志置位

---

### 图传解析测试（USART1）
**测试环境：**
- 图传模块型号：
- 波特率：115200
- 协议：自定义 + VT13

**测试用例：**
- [ ] TC-09: 39 字节自定义数据帧正确解析
- [ ] TC-10: 21 字节遥控帧正确解析
- [ ] TC-11: 21 字节 VT13 帧正确解析
- [ ] TC-12: 畸形长度帧（> 39）被拒绝
- [ ] TC-13: CRC8 正确但 claimed_len 非法的帧被拒绝
- [ ] TC-14: 短包（< 5 字节）被拒绝
- [ ] TC-15: CRC16 错误的帧被拒绝
- [ ] TC-16: 半包不导致崩溃或越界

**边界注入测试：**
```
帧结构: 0xA5 [len_L] [len_H] [seq] [CRC8] [CMDID] [data...] [CRC16_L] [CRC16_H]

测试帧 1 (正常 39 字节):
  A5 1E 00 00 XX YY [30 bytes data] ZZ ZZ

测试帧 2 (claimed_len = 255, 会超出缓冲区):
  A5 FF 00 00 XX YY [数据不足]

测试帧 3 (短包 3 字节):
  A5 00 00

测试帧 4 (CRC8 正确，claimed_len = 200):
  A5 C8 00 00 [计算正确的 CRC8] YY [...]
```

**验证方法：**
- [ ] 使用串口助手发送测试帧
- [ ] 检查 `Image_Transmission_Info` 结构体内容
- [ ] 确认非法帧不触发数据更新
- [ ] 使用 ASAN/边界检测工具（如可用）

---

### 遥控超时测试
**测试环境：**
- 遥控器：DT7 或双板遥控
- 超时阈值：`RC_LOST_CNT_MAX = 50`
- 监视周期：10 ms

**测试用例：**
- [ ] TC-17: 正常遥控持续接收（online_cnt 保持 > 50）
- [ ] TC-18: 遥控失联后 online_cnt 递减
- [ ] TC-19: online_cnt = 50 临界点行为正确
- [ ] TC-20: online_cnt = 0 后数据清零
- [ ] TC-21: 失联后重新连接恢复正常
- [ ] TC-22: 失联临界点不发生竞态（多次重复测试）

**竞态检测：**
```c
// 在 Remote_Control.c 添加调试输出（临时）
if(online_cnt == 51 || online_cnt == 50 || online_cnt == 49){
    printf("DEBUG: online_cnt=%d at %lu ms\n", online_cnt, HAL_GetTick());
}
```

**验证方法：**
- [ ] 遥控器开机 → 关机 → 开机循环测试 20 次
- [ ] 临界点附近无数据撕裂
- [ ] 失联判定延迟合理（约 500 ms）

---

### 集成测试
**测试场景：**
- [ ] TC-23: 机械臂正常启动
- [ ] TC-24: 夹爪标零成功
- [ ] TC-25: 手动模式正常控制
- [ ] TC-26: PC 模式正常控制
- [ ] TC-27: 模式切换流畅无卡顿
- [ ] TC-28: 遥控失联后进入无力模式
- [ ] TC-29: USB 断开后进入无力模式
- [ ] TC-30: 所有轴反馈正常接收

---

## 回归测试 ⏳

### 行为一致性验证
- [ ] 合法遥控输入产生相同的控制输出
- [ ] PC 命令响应与修复前一致
- [ ] 七轴快照时序保持不变
- [ ] 故障行为（bus-off、超时）保持不变
- [ ] 启动、标零、回零流程不变

### 性能基准
**测试前（基线）：**
- 控制任务周期：8 ms ±___ μs
- 输出任务周期：4 ms ±___ μs
- USB 任务周期：10 ms ±___ μs
- 最大栈使用：___/___/___

**测试后（修复后）：**
- [ ] 控制任务周期：8 ms ±___ μs（偏差 < 5%）
- [ ] 输出任务周期：4 ms ±___ μs（偏差 < 5%）
- [ ] USB 任务周期：10 ms ±___ μs（偏差 < 5%）
- [ ] 最大栈使用：___/___/___（无显著变化）

---

## 文档验证 ✅

- [x] `docs/P0_P1_FIXES.md` 存在且完整
- [x] `docs/P2_ARMCLANG_CHECK_FIX.md` 存在且完整
- [x] `docs/P0_P1_P2_SUMMARY.md` 存在且完整
- [x] 所有修改都有详细说明
- [x] 修复前后对比清晰
- [x] 验收标准明确

---

## Git 提交验证 ⏳

### 提交前检查
```bash
git status
git diff --stat
git diff BSP/Src/bsp_uart.c
git diff Components/Device/Src/Image_Transmission.c
git diff Components/Device/Inc/Image_Transmission.h
git diff Components/Device/Src/Remote_Control.c
git diff tools/selftest/armclang_check.py
```
- [ ] 只包含 P0/P1/P2 修复
- [ ] 没有意外的修改
- [ ] 没有调试代码残留

### 创建分支
```bash
git checkout -b fix/p0-p1-p2-boundary-fixes
```
- [ ] 分支名称清晰

### 提交修复
```bash
git add BSP/Src/bsp_uart.c
git add Components/Device/Src/Image_Transmission.c
git add Components/Device/Inc/Image_Transmission.h
git add Components/Device/Src/Remote_Control.c
git add tools/selftest/armclang_check.py
git add docs/P0_P1_FIXES.md
git add docs/P2_ARMCLANG_CHECK_FIX.md
git add docs/P0_P1_P2_SUMMARY.md
git add docs/VERIFICATION_CHECKLIST.md

git commit -m "fix: P0/P1/P2 边界修复与编译检查改进

P0-1: 修正 UART DMA 双缓冲配置
- RxXferSize 和所有 DMA 重装长度统一为单缓冲大小
- SBUS: 18 字节 (去掉 *2)
- 图传: 39 字节 (去掉 *2)
- 裁判系统: 对应长度 (去掉 *2)
- 消除缓冲区溢出风险

P0-2: 图传解析增加边界检查
- Image_Transmission_Info_Update() 增加 Size 参数
- CRC 校验前先验证长度边界
- 防止读取越界内存
- 拒绝畸形报文

P1: 遥控超时监视增加同步保护
- Remote_Message_Moniter() 增加临界区
- 消除监视函数与 ISR 的竞态条件
- 保持超时判断一致性

P2: 编译检查脚本改进
- 从 .dep 动态提取文件列表
- 排除已归档的 Arm_R_task.c
- 生成 JSON 构建记录
- 记录源码哈希、编译结果、告警

不改变控制算法、协议、周期和故障策略。

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```
- [ ] 提交信息清晰完整
- [ ] 包含所有修改文件
- [ ] 包含 Co-Authored-By

---

## 验收标准 ⏳

### 必须满足（P0）
- [ ] 所有编译测试通过
- [ ] UART DMA 不发生缓冲区溢出
- [ ] 图传畸形报文不导致越界访问
- [ ] 遥控超时无竞态导致的数据撕裂

### 应当满足（P1）
- [ ] 硬件测试覆盖所有正常和边界场景
- [ ] 行为一致性验证通过
- [ ] 性能无显著退化

### 建议满足（P2）
- [ ] 长时间稳定性测试（> 1 小时）
- [ ] 多次启动/重启测试（> 20 次）
- [ ] 极限输入测试（最大速度/加速度）

---

## 验证签名

**代码审查：** __________ 日期：____  
**编译验证：** __________ 日期：____  
**硬件测试：** __________ 日期：____  
**最终验收：** __________ 日期：____

---

## 问题记录

如果验证中发现问题，记录在此：

| 编号 | 发现日期 | 问题描述 | 严重程度 | 状态 | 解决方案 |
|------|----------|----------|----------|------|----------|
|      |          |          |          |      |          |
