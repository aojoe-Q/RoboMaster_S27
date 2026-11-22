# PC 模式接管说明

> 对应提交：`84ec9ee`（行为改动）、`f304a09`（自测脚本修复）、`1905d94`（恢复）。
> 适用：RoboMaster S27 电控固件（STM32H723VGT6，`main` 分支）。
> 相关上位机：`aojoe-Q/ARM_ROS` 仓库的 `dm_mc02_hardware` 包。

---

## 1. 为什么改

### 1.1 旧行为的问题

进入 PC 模式的那一拍（`Arm_PC_Control()` 第一次执行），代码先写 PD 增益、再把位置目标锁存成当前实测位置。结果就是：上位机一个字节都还没发，七个电机已经"稳住"了。ROS 侧只想开只读遥测、或者人想手动摆姿态时，机械臂是带力的，得跟它对抗。

同时，"右摇杆下 + 左摇杆上"这条映射被整块注释掉，等于没有从遥控器直接切手动通道（`ARM_rc`）的入口。

### 1.2 改动后的行为

PC 链路改成三阶段，由一个枚举记录当前处于哪一段：

```c
static enum { PC_READ_ONLY, PC_TRACKING, PC_HOLDING } pc_phase = PC_READ_ONLY;
```

| 阶段 | 怎么进 | 每拍做什么 | 怎么出 |
|---|---|---|---|
| `PC_READ_ONLY` | 切入 PC 模式，或从保持态重新进入 | 7 轴 `torque = 0`；没有有效目标就调 `Arm_Powerless()` 后直接 `return`——不写 PD 增益、不写位置目标 | 收到首条有效目标 → `PC_TRACKING` |
| `PC_TRACKING` | 首次拿到有效目标 | 写 PD 增益；第一拍用实测位置重置插补器；之后按 `vmax` / `amax` 逐关节插补，写 `send_set` | 目标失效（超时 / USB 断开 / CAN 反馈丢失）→ `PC_HOLDING` |
| `PC_HOLDING` | 跟随后目标失效 | 失效第一拍锁存七轴当前位置，之后每拍都发这组位置，不再插补 | 目标恢复 → `PC_TRACKING`；或遥控器重进 PC 模式 → `PC_READ_ONLY` |

"重新进入只读无力"的操作路径：先用支架托住机械臂 → 遥控器切"双下无力" → 再切回"双上 PC"。这样会重新调用 `Arm_PC_Control_Enter()`，把状态机打回 `PC_READ_ONLY`。

上电初始化和夹爪标零不受这套规则影响。

### 1.3 另外两处改动

- **夹爪在 PC 模式下不再叠加力矩混合。** `use_grip_torque = Arm_L_Ready && grip_torque_mode && ArmData.ArmMode != ARM_MODE_PC_CONTROL`。PC 模式里夹爪完全由上位机的位置指令驱动，避免位置控制时力矩还在往上顶。
- **摇杆新增一条映射**：右摇杆下 + 左摇杆上 → `ARM_RC`（原来的注释块被启用）。

---

## 2. 改了哪些文件

| 文件 | 改动 |
|---|---|
| `APP/Arm_L_task.c` | 三阶段状态机、摇杆映射、夹爪条件（净 +36 行 / −31 行） |
| `tools/selftest/pc_mode_selftest.c` | 新增，76 行，四段断言：只读 → 接管 → 保持 → 重进只读 |
| `tools/selftest/run_regression_selftest.py` | 增加 `--pc-only` 开关与 pc_mode 用例；修正 `TAKE_POSE_1` / `Arm_L_Movement_J1_J4` 等改名后失配的函数名 |
| `tools/selftest/arm_mode_selftest.c` | 同步改名：`Arm_L_Movement_J1_J4()` → `ARM_rc()` |

关键位置：

| 位置 | 内容 |
|---|---|
| `APP/Arm_L_task.c:1160` | `pc_phase` 定义与初值 `PC_READ_ONLY` |
| `APP/Arm_L_task.c:1168` | `Arm_PC_Control_Enter()`：置 `PC_READ_ONLY`，重置时间基准 |
| `APP/Arm_L_task.c:1181` | `Arm_PC_Control()`：取目标 → 清零力矩 → 只读则撤力返回 → 否则 PD 增益 + 插补 / 保持 |
| `APP/Arm_L_task.c:288` | `use_grip_torque`（排除 PC 模式） |
| `APP/Arm_L_task.c:837` | `ARM_RC` 摇杆映射 |
| `APP/arm_motion.h:16-18` | 插补上限：`ARM_PC_VMAX_RAD_S 0.5`、`ARM_PC_AMAX_RAD_S2 1.0`、`ARM_PC_DT_MAX_S 0.016` |

---

## 3. 自测怎么跑

在仓库根目录，用主机 gcc 编译真实函数体（不碰硬件）：

```powershell
# 只跑 PC 模式用例
python tools/selftest/run_regression_selftest.py --pc-only

# 全量
python tools/selftest/run_regression_selftest.py
```

当前结果（gcc 8.1.0，`-std=c99 -Wall -Wextra -Werror`）：

```
PC mode: read-only, takeover, hold and re-entry OK
bus_hold: OK   explicit: OK   loss: OK      reconnect: OK   grip_retry: OK
invalid: OK    debounce: OK   unready: OK   grip_limits: OK
uart: OK (alternating frames, partial/oversize rejection, overflow recovery)
REGRESSION SELFTEST OK
```

`pc_mode` 用例断言的是四件事：只读阶段下发 `kp = kd = torque = 0` 且位置等于实测；接管首拍位移小于 1 mrad 且 `kp > 0`、`torque = 0`；丢目标后位置锁死不再跟随；`Arm_PC_Control_Enter()` 后再回只读无力。

**这只能证明逻辑自洽，不能替代上车。**

---

## 4. 上车前的注意事项

1. **Keil 必须关掉重开，然后 Rebuild 全量**。改的是跨函数调用顺序（`Arm_Powerless()` 被提前到 `Arm_PC_Control()` 开头），增量编译容易用到旧 `.o`。
2. **第一次联调先 `allow_enable:=0`**：此时只回传关节角，确认方向、零位、限位都正确，再放开使能。
3. **注意新手势**：右摇杆下 + 左摇杆上 现在会进 `ARM_RC`，之前什么都不会发生。举着机械臂时别误触。
4. **夹爪改由位置指令驱动**：上位机如果原来是发力矩目标，现在要改发位置。

---

## 5. 怎么回滚

```powershell
# 撤销整个 PC 模式改动
git revert 1905d94

# 只想退回改动前的电控行为，同时保留脚本修复
git revert 84ec9ee
```

历史里另有 `be7dc7d` 是一条回退提交（把 `84ec9ee` + `f304a09` 一起退掉），`1905d94` 已把内容恢复回 `f304a09` 的状态。要看当时到底删了什么：`git show be7dc7d`。

---

## 6. 提交历史

| 提交 | 说明 |
|---|---|
| `84ec9ee` | PC 模式接管改为三阶段状态机 |
| `f304a09` | 修复回归脚本：同步接口改名 |
| `be7dc7d` | 回退上面两条（保留在历史里） |
| `1905d94` | 恢复 `84ec9ee` + `f304a09` 的内容 |
