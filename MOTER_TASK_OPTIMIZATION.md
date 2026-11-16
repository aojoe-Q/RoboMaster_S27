# Moter_task.c 代码复杂度分析与优化方案

## 📋 当前问题

`Arm_Output_Step()` 函数混杂 9 个职责（115-257 行，143 行代码）：

1. **启动门控** - 未启动返回
2. **CAN 恢复检测** - can_recovery_epoch 变化
3. **Retry 判定** - 清除总线抑制
4. **Startup 失败处理** - 清零增益/力矩
5. **7 轴轮转遍历** - rotating_first 轮转
6. **反馈超时检查** - age > 200ms
7. **使能握手判定** - 状态机 EN_WAIT_BOOT -> EN_SEND -> EN_OK
8. **FIFO 检查** - 预留 1 个槽位
9. **指令超时回落** - cmd.tick_ms > 100ms

---

## ✅ 优化方案

### 第 1 层：状态检查类（4 个函数）
```c
static void Arm_CheckBusRecovery(uint32_t now)
    └─ 检测 can_recovery_epoch 变化，设置 bus_inhibit

static uint8_t Arm_CheckRetryRequest(const Arm_Control_Snapshot *cmd, uint32_t now)
    └─ 判定 retry 是否有效（新鲜 + 无动力）

static void Arm_CheckFeedbackTimeout(unsigned i, const joint_motor_measure_t *fb, uint32_t now)
    └─ 单关节反馈超时判定，EN_OK -> EN_FAIL

static uint8_t Arm_CheckFifoSpace(void)
    └─ 检查 FIFO 是否有 ≥2 个槽位
```

### 第 2 层：使能握手类（2 个函数）
```c
static uint8_t Arm_CheckEnableHandshake(unsigned i, const joint_motor_measure_t *fb, uint32_t now)
    └─ 判定是否收到有效使能确认帧

static void Arm_SendEnableFrame(unsigned i, uint32_t now)
    └─ 发送使能帧 + 零增益 MIT 帧
```

### 第 3 层：指令发送类（1 个函数）
```c
static void Arm_SendControlFrame(unsigned i, const joint_motor_measure_t *fb,
                                 const Arm_Control_Snapshot *cmd, uint32_t now)
    └─ 发送 MIT 控制帧（指令新鲜 or 超时回落）
```

### 第 4 层：单关节处理（1 个函数）
```c
static void Arm_ProcessJoint(unsigned i, const joint_motor_measure_t *fb,
                             const Arm_Control_Snapshot *cmd, uint32_t now)
    └─ 组织上述 2/3 层逻辑，处理一个关节的完整周期
```

### 第 5 层：主状态机（简化）
```c
void Arm_Output_Step(uint32_t now) {
    BSP_CAN_HealthStep(now);
    Arm_HandleStartup(now);      // 启动门控
    Arm_CheckBusRecovery(now);    // 总线恢复
    Arm_HandleRetry(now);         // Retry 判定
    Arm_HandleStartupFailed(now); // Startup 失败
    Arm_ProcessAllJoints(now);    // 7 轴轮转
}
```

---

## 📊 改进对比

| 项目 | 改前 | 改后 |
|------|------|------|
| **主函数行数** | 143 行（混杂） | 20 行（编排） |
| **单个函数职责** | 1 个函数 9 职 | 9 个函数各 1 职 |
| **圈复杂度** | O(n²)（嵌套 loop + 多个 if） | O(n)（分离 check + 循环） |
| **可测试性** | 难：必须模拟整个 CAN 环境 | 易：各函数可独立单测 |
| **可维护性** | 低：改一处影响全局 | 高：改 XX 检查仅改 `Arm_CheckXX()` |
| **性能** | 相同：无额外开销 | 相同：编译器内联 static 函数 |

---

## 🔧 关键提取点

### 反馈超时检查
**改前**：
```c
for (unsigned n=0; n<7; ++n) {
    unsigned i = (rotating_first + n) % 7U;
    uint8_t state = arm_output_diag.state[i];
    uint32_t age = now - fb[i].update_tick;
    if (state == EN_OK && (age > ARM_OUTPUT_FB_TIMEOUT_MS || fb[i].status != 1U)) {
        arm_output_diag.state[i] = EN_FAIL;
        state = EN_FAIL;
    }
}
```

**改后**：
```c
static void Arm_CheckFeedbackTimeout(unsigned i, const joint_motor_measure_t *fb, uint32_t now) {
    if (arm_output_diag.state[i] != EN_OK) return;
    
    uint32_t age = now - fb[i].update_tick;
    if (age > ARM_OUTPUT_FB_TIMEOUT_MS || fb[i].status != 1U) {
        arm_output_diag.state[i] = EN_FAIL;
    }
}

// 在 Arm_ProcessJoint() 中调用
Arm_CheckFeedbackTimeout(i, &fb[i], now);
```

---

## 🎯 实施步骤

### Phase 1：提取检查类函数（4 个）
- `Arm_CheckBusRecovery()`
- `Arm_CheckRetryRequest()`
- `Arm_CheckFeedbackTimeout()`
- `Arm_CheckFifoSpace()`

### Phase 2：提取握手/发送类函数（3 个）
- `Arm_CheckEnableHandshake()`
- `Arm_SendEnableFrame()`
- `Arm_SendControlFrame()`

### Phase 3：提取单关节处理函数（1 个）
- `Arm_ProcessJoint()`

### Phase 4：简化主状态机（5 个辅助）
- `Arm_HandleStartup()`
- `Arm_HandleRetry()`
- `Arm_HandleStartupFailed()`
- `Arm_ProcessAllJoints()`
- `Arm_Output_Step()` 简化为编排

---

## 📈 预期改进

- **代码可读性** ↑↑↑
- **单元测试覆盖** ↑↑
- **维护成本** ↓↓
- **bug 发现率** ↓↓（职责清晰，逻辑隐藏的 bug 更少）

