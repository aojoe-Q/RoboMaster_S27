# Arm_PrepareAndPublish() 优化：分离 Blend 更新与快照发布

## 🎯 问题

`Arm_PrepareAndPublish()` 在多处被反复调用，但**夹爪混合系数计算**不需要每次都做：

```c
static void Arm_PrepareAndPublish(void) {
    static uint32_t last_ms;
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    float dt_ms = (float)(now - last_ms);  // ← 时间差计算
    last_ms = now;

    Arm_UpdateGripBlend(dt_ms);  // ← 每次都计算，即使是快速循环
    Arm_Control_Snapshot cmd = Arm_CollectSnapshot(now);
    Arm_ApplyGripBlend(&cmd);
    arm_pending_command = cmd;
    Arm_Control_Publish(&arm_pending_command);
}
```

**问题**：
- 夹爪标零循环（20ms）：每 20ms 调一次 → 20ms 内重复计算时间差
- 回零循环（5ms）：每 5ms 调一次 → 5ms 内重复计算时间差
- 主循环（8ms）：正常，每 8ms 调一次 ✓

---

## ✅ 方案

分离为 4 个函数，职责清晰：

### 1. `Arm_UpdateGripBlendOnce()` 
**仅在主循环调用**，计算时间差并推进状态
```c
static void Arm_UpdateGripBlendOnce(void) {
    static uint32_t last_ms;
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    float dt_ms = (float)(now - last_ms);
    last_ms = now;
    Arm_UpdateGripBlend(dt_ms);  // 仅调用一次
}
```

### 2. `Arm_PrepareSnapshot()`
**收集快照，复用当前 blend 值**
```c
static void Arm_PrepareSnapshot(void) {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    Arm_Control_Snapshot cmd = Arm_CollectSnapshot(now);
    Arm_ApplyGripBlend(&cmd);  // 用当前 s_grip_blend，无重新计算
    arm_pending_command = cmd;
}
```

### 3. `Arm_PublishSnapshot()`
**转发快照**
```c
static void Arm_PublishSnapshot(void) {
    Arm_Control_Publish(&arm_pending_command);
}
```

### 4. `Arm_PrepareAndPublish()` - 主循环用
```c
static void Arm_PrepareAndPublish(void) {
    Arm_UpdateGripBlendOnce();   // 仅主循环调用：计算时间差
    Arm_PrepareSnapshot();       // 收集快照
    Arm_PublishSnapshot();       // 发布
}
```

### 5. `Arm_PublishCurrent()` - 初始化用
```c
static void Arm_PublishCurrent(void) {
    Arm_PublishSnapshot();  // 直接发布，无计算
}
```

---

## 📊 调用点更新

| 调用位置 | 原函数 | 改为 | 理由 |
|---------|--------|------|------|
| `Arm_L_Task()` 主循环 | `Arm_PrepareAndPublish()` | ✓ 不变 | 需要计算时间差 |
| `Arm_Grip_Stall_Zero()` 标零循环 20ms | `Arm_PrepareAndPublish()` | `Arm_PublishCurrent()` | 快速循环，无需重算 |
| 存零确认循环 10ms | `Arm_PrepareAndPublish()` | `Arm_PublishCurrent()` | 快速循环，无需重算 |
| 回零 do-while 循环 5ms | `Arm_PrepareAndPublish()` | `Arm_PublishCurrent()` | 快速循环，无需重算 |
| `Arm_L_Init()` 初始化阶段 | `Arm_PrepareAndPublish()` | ✓ 不变 | 保持兼容，中间仍需计算 |

---

## 🚀 性能改进

### 标零流程（12s 内最多 600 循环 × 20ms）
**改前**：每 20ms 调用 → 600 次时间差计算
**改后**：0 次时间差计算 → **节省 600 次系统调用**

### 回零流程（最多 20s × 200 循环 × 5ms）
**改前**：每 5ms 调用 → ~4000 次时间差计算  
**改后**：0 次时间差计算 → **节省 4000 次系统调用**

### 总体
- ✅ 减少冗余时间戳读取
- ✅ 减少不必要的浮点运算
- ✅ 提升初始化流程性能
- ✅ 代码意图更清晰

---

## 📝 总结

| 方面 | 改前 | 改后 |
|------|------|------|
| **Blend 计算频率** | 每次 `Arm_PrepareAndPublish()` | 仅主循环 8ms 一次 |
| **快速循环开销** | O(时间戳读取 + 浮点) | O(1) 发布 |
| **代码清晰度** | 混乱：同一个函数做多件事 | 清晰：各函数职责分明 |
| **可维护性** | 难：改 blend 要改 `Arm_PrepareAndPublish` | 易：改 blend 仅改 `Arm_UpdateGripBlendOnce` |

