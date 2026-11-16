# 运行时问题修复

**日期**：2025 年 1 月  
**优先级**：高  
**状态**：已修复

---

## 问题 1：BMI088 初始化无限循环

**位置**：`Components/Device/Src/Bmi088.c:267`

**原因**：
```c
do {
    status |= BMI088_Accel_Init();    // 一旦设置失败位
    status |= BMI088_Gyro_Init();     // 状态位 |= 后永不清零
    Delay_ms(2);
} while (status);  // 即使后续都成功，仍然卡住
```

**修复**：
- 每轮重置 `status = BMI088_NO_ERROR`
- 添加有限重试机制（10 次）
- 重试失败后 `break`，不再阻塞启动

**代码**：
```c
void BMI088_Init(void)
{
  BMI088_Status_e status = BMI088_NO_ERROR;
  uint32_t max_retries = 10U;
  uint32_t retry_count = 0U;

  do {
    status = BMI088_NO_ERROR;  /* 每轮重置 */
    status |= BMI088_Accel_Init();
    status |= BMI088_Gyro_Init();
    Delay_ms(2);
    retry_count++;
    if (retry_count >= max_retries) break;
  } while (status);

  BMI088_Offset_Update(&BMI088_Info);
}
```

---

## 问题 2：ARM 配置中不必要的 IMU 初始化

**位置**：`BSP/Src/bsp_mcu.c:40`

**原因**：
- INS_Task 已禁用，但 `BMI088_Init()` 仍在 `BSP_Init()` 中调用
- 调用发生在调度器启动前
- IMU 故障会阻止所有机械臂任务启动

**修复**：
- 用条件编译检查 `APP_ARM_ENABLE`
- ARM 配置下跳过 BMI088 初始化

**代码**：
```c
/* ----------------------- Device Init ----------------------- */
/* BMI088 仅在需要时初始化（当前 ARM 配置不需要 IMU） */
#ifndef APP_ARM_ENABLE
  BMI088_Init();
#endif
```

---

## 问题 3：双板遥控时钟不统一

**位置**：`APP/Compat/app_can.c:59-62`

**原因**：
- CAN 接收时戳使用 FreeRTOS 时钟：`xTaskGetTickCount() * portTICK_PERIOD_MS`
- 在线判断使用 HAL 时钟：`HAL_GetTick()`
- 两个时钟启动时刻不同：
  - HAL 时钟：系统初始化开始
  - FreeRTOS 时钟：调度器启动
  - BMI088 初始化中有 2 × 80 ms 延时
- 导致差距 > 100 ms，刚收到的报文立即被判为离线

**现象**：
```
接收时戳：1000 ms（FreeRTOS）
HAL 时间：1162 ms
差值：162 ms > 100 ms → 判为离线 ✗
```

**修复**：统一使用 FreeRTOS 时钟

**代码**：
```c
static bool APP_Board_IsOnline(void)
{
    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return (app_board_last_tick != 0U) &&
           ((now_ms - app_board_last_tick) < 100U);
}
```

---

## 问题 4：双板两类报文共用超时判断

**位置**：`APP/Compat/app_can.c:188-205`

**原因**：
- 0x212：更新 ch[0-2] 和拨杆
- 0x213：更新 ch[3]、键盘、鼠标
- 两种报文共用 `app_board_last_tick`
- 若 0x212 停止但 0x213 继续，仍被认为在线
- 导致使用旧的摇杆值

**修复**：
- 分别记录两种报文的接收时刻
- 两者都在线才发布完整数据

**代码**：
```c
static uint32_t app_board_ch012_tick = 0;   /* 0x212 时刻 */
static uint32_t app_board_ch3key_tick = 0;  /* 0x213 时刻 */

static bool APP_Board_IsOnline(void)
{
    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return (app_board_ch012_tick != 0U) && (app_board_ch3key_tick != 0U) &&
           ((now_ms - app_board_ch012_tick) < 100U) &&
           ((now_ms - app_board_ch3key_tick) < 100U);
}
```

**更新报文处理**：
```c
if (id == (uint32_t)CAN_Board_ID_1)  /* 0x212 */
{
    /* 更新 ch[0-2] 和拨杆 */
    app_board_ch012_tick = tick_ms;
}
else if (id == (uint32_t)CAN_Board_ID_2)  /* 0x213 */
{
    /* 更新 ch[3]、键盘、鼠标 */
    app_board_ch3key_tick = tick_ms;
}
```

---

## 编译验证

```bash
cd "C:\Users\lenovo\Desktop\S27_TEST_2\MDK-ARM"
# Rebuild All
# 期望：107 个编译单元，0 错误，0 警告
```

---

## 后续验证

1. 系统启动：不再卡住在 BMI088
2. DT7 离线切换：使用统一时钟判定
3. 双板遥控稳定性：两类报文都必须在线

