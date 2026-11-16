# S27_TEST_2 P0/P1/P2 修复总结

修复日期：2026-11-11  
分支：fix/p0-uart-dma-buffer  
基于提交：2b6bcbf

## 概述

本次修复针对代码检查报告中的 P0（严重）、P1（高优先级）和 P2（中等优先级）问题，采用"小范围缺陷修复 + 等价结构整理"的策略，保留现有机械臂主链路的分层与保护机制。

## 修复清单

### ✅ P0-1: UART DMA 双缓冲配置错误

**文件：** `BSP/Src/bsp_uart.c`

**问题：**
- 单缓冲区容量与 DMA 重装长度不匹配
- SBUS: 18 字节缓冲，却重装 36 字节
- 图传: 39 字节缓冲，却重装 78 字节
- RxXferSize 错误地设置为 `DataLength * 2`

**修复：**
- 第 77 行：`RxXferSize = DataLength`（去掉 *2）
- 第 128、149 行：`SBUS_RX_BUF_NUM`（去掉 *2）
- 第 193、210 行：`IMAGE_TRANS_RX_LENGTH`（去掉 *2）
- 第 225、240 行：`REFEREE_RXFRAME_LENGTH`（去掉 *2）

**影响：**
- 消除 DMA 缓冲区溢出风险
- 保持正常帧解析逻辑不变

---

### ✅ P0-2: 图传解析边界检查缺失

**文件：**
- `Components/Device/Inc/Image_Transmission.h`
- `Components/Device/Src/Image_Transmission.c`
- `BSP/Src/bsp_uart.c`

**问题：**
- 在 CRC 校验前未验证长度边界
- 可能读取越界内存
- Size 参数未传递给解析函数

**修复：**
1. 函数签名更新：`Image_Transmission_Info_Update(uint8_t *Buff, uint16_t Size)`
2. 添加最小长度检查（FrameHeader_Length）
3. 验证总帧长 ≤ Size 且 ≤ IMAGE_TRANS_RX_LENGTH
4. 先边界检查，再 CRC 校验
5. VT13 分支增加 21 字节长度检查
6. 更新调用点传入 Size 参数

**影响：**
- 防止越界内存访问
- 拒绝畸形报文
- 合法报文的解析结果保持一致

---

### ✅ P1: 遥控超时监视竞态条件

**文件：** `Components/Device/Src/Remote_Control.c`

**问题：**
- `Remote_Message_Moniter()` 在任务中修改 `remote_ctrl`
- `SBUS_TO_RC()` 在 UART ISR 中更新 `remote_ctrl`
- 两个写入方无同步保护

**修复：**
1. 添加 FreeRTOS 头文件
2. 在 `Remote_Message_Moniter()` 中使用临界区：
   - `taskENTER_CRITICAL()` 在函数开始
   - `taskEXIT_CRITICAL()` 在函数结束

**影响：**
- 消除监视函数与 ISR 的竞态
- 保持超时判断一致性
- 不改变超时阈值和逻辑

---

### ✅ P2: 编译检查脚本同步问题

**文件：** `tools/selftest/armclang_check.py`

**问题：**
- 硬编码文件列表引用已归档的 `Arm_R_task.c`
- 与实际工程清单不同步
- 没有构建记录和源码摘要

**修复：**
1. 从 `.dep` 文件动态提取 C 文件列表
2. 过滤项目源码（APP、Components、Modules、BSP）
3. 配置排除列表（已归档文件）
4. 生成 JSON 格式的构建记录：
   - 时间戳、编译器、标志
   - 每个文件的路径、SHA256 哈希、编译结果、告警
   - 总结：文件数、失败数、告警数
5. 改进错误处理和报告

**影响：**
- 自动同步工程文件列表
- 记录构建历史便于追踪
- 维护性改善

---

## 修改文件统计

| 类型 | 文件数 | 文件列表 |
|------|--------|----------|
| BSP | 1 | `BSP/Src/bsp_uart.c` |
| 设备驱动 | 2 | `Components/Device/Src/Image_Transmission.c`<br>`Components/Device/Src/Remote_Control.c` |
| 头文件 | 1 | `Components/Device/Inc/Image_Transmission.h` |
| 测试工具 | 1 | `tools/selftest/armclang_check.py` |
| 文档 | 3 | `docs/P0_P1_FIXES.md`<br>`docs/P2_ARMCLANG_CHECK_FIX.md`<br>`docs/P0_P1_P2_SUMMARY.md` |

**总计：** 5 个源文件，1 个头文件，1 个工具脚本，3 个文档

---

## 未修改内容

按照评审文档建议，以下内容保持不变：
- ✅ 波特率、帧格式、遥控映射
- ✅ USB 协议版本和字段顺序
- ✅ CAN ID 和总线拓扑
- ✅ 控制周期（4/8/10 ms）
- ✅ PD 参数、零点、限位
- ✅ 故障保持/无力策略
- ✅ 控制算法和增益
- ✅ 启动流程和自动回零

---

## 验证要求

### 单元测试
```bash
cd tools/selftest
python run_selftest.py              # USB 协议测试
python run_control_selftest.py     # 控制逻辑测试
python armclang_check.py            # 编译检查（新）
python full_arm_build.py            # 完整构建
```

### 硬件验证

#### UART DMA 测试
- [ ] 串口首帧接收正确
- [ ] 连续多帧接收正确
- [ ] 单帧填满缓冲区（18/39 字节）
- [ ] M0/M1 双缓冲轮换
- [ ] 缓冲区边界无越界

#### 图传解析测试
- [ ] 39 字节自定义数据帧
- [ ] 21 字节遥控帧
- [ ] 21 字节 VT13 帧
- [ ] 畸形长度（> 39）拒绝
- [ ] CRC8 正确但长度非法的帧拒绝
- [ ] 短包（< 5 字节）拒绝

#### 遥控超时测试
- [ ] 正常遥控持续接收
- [ ] 遥控失联后自动清零
- [ ] 失联临界点（online_cnt = 50）行为正确
- [ ] 失联后重新连接恢复正常

#### 编译检查测试
- [ ] 不再尝试编译 Arm_R_task.c
- [ ] 成功编译所有当前工程源文件
- [ ] 生成 build_record.json
- [ ] 退出码反映实际失败数

---

## 下一步工作

按照评审文档的实施顺序：

1. **✅ 已完成：阶段 A - 建立基线**
   - 备份当前工作树
   - 记录参数、宏、协议、任务周期

2. **✅ 已完成：阶段 B - 边界修复**
   - P0-1: DMA 缓冲区配置
   - P0-2: 图传解析边界
   - P1: 遥控超时竞态
   - P2: 编译检查脚本

3. **待进行：阶段 C - 等价拆分**
   - 提取初始化、夹爪、模式、PC 控制模块
   - 整理 USB 任务
   - 保持原入口及调用顺序
   - 不改算法、常量、时序

4. **待进行：阶段 D - 性能优化**
   - 加周期观测
   - 据测量缩栈/减少轮询
   - 不直接改频率或优先级

5. **待进行：阶段 E - 功能变更**
   - 接通自定义控制器
   - 失联夹爪立即卸力
   - 启动改为可取消状态机
   - 单独验收标准

---

## Git 提交建议

```bash
git add BSP/Src/bsp_uart.c
git add Components/Device/Src/Image_Transmission.c
git add Components/Device/Inc/Image_Transmission.h
git add Components/Device/Src/Remote_Control.c
git add tools/selftest/armclang_check.py
git add docs/P0_P1_FIXES.md
git add docs/P2_ARMCLANG_CHECK_FIX.md
git add docs/P0_P1_P2_SUMMARY.md

git commit -m "fix: P0/P1/P2 边界修复与编译检查改进

P0-1: 修正 UART DMA 双缓冲配置，RxXferSize 和重装长度统一为单缓冲大小
P0-2: 图传解析增加 Size 参数和边界检查，CRC 前验证长度
P1: 遥控超时监视增加临界区保护，消除与 ISR 的竞态
P2: 编译检查脚本从 .dep 动态提取文件，排除已归档文件，记录构建历史

不改变控制算法、协议、周期和故障策略。

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

## 参考文档

- 原始评审：`S27_TEST_2_代码检查与优化方案_20261111.md`
- P0/P1 详细修复：`docs/P0_P1_FIXES.md`
- P2 详细修复：`docs/P2_ARMCLANG_CHECK_FIX.md`
