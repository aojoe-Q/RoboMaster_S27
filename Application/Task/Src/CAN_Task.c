/**
  ******************************************************************************
  * @file           : CAN_Task.c
  * @brief          : CAN task
  * @author         : GrassFam Wang
  * @date           : 2025/1/22
  * @version        : v1.1
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */
/* 模块职责：FDCAN 接收分发任务：把 BSP 层环形队列里的帧交给 APP_CAN_RxHandler，并维护发送节拍。 */
/* 运行方式：整车分支按 osDelay(1) 跑 1 ms 节拍；机械臂分支由 RX 中断任务通知唤醒、10 ms 超时兜底；优先级/栈见 cmsis_os 配置。 */
/* 总线映射：FDCAN1 = 达妙关节 0x01..0x07；FDCAN2 = 双板/自定义控制器 0x212/0x213；FDCAN3 = 抬升 DJI 0x201..0x204。 */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "task.h"
#include "can_health.h"
#include "CAN_Task.h"
#include "Control_Task.h"
#include "INS_Task.h"
#include "Motor.h"
#include "bsp_can.h"
#include "Remote_Control.h"
#include "can_bsp.h"
#include "Control_Task.h"

/* USER CODE BEGIN Header_CAN_Task */
/**
* @brief Function implementing the StartCANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Task */

 /* 任务入口：注册 RX 任务句柄后进入主循环。 */
 // 参数：argument 未使用；返回：无（常驻任务）。
 void CAN_Task(void const * argument)
{
  (void)argument;  // 显式丢弃未使用参数，消除编译告警
  BSP_CAN_RxTaskInit();  // 记录当前任务句柄，供 RX 中断 vTaskNotifyGiveFromISR 唤醒

// 禁用分支：仅在 APP_ARM_ENABLE == 0（整车/双板构建）时编译，纯机械臂构建（=1）整段不参与编译。
// 内容为遗留的 DM8009 双板联调代码：上电依次使能 4 个电机，之后每 1 ms 重复下发全零 MIT 指令。
// 其中重复发送属于历史行为，按“只加注释”的要求原样保留，不做修正。
#if (APP_ARM_ENABLE == 0)

 
  // 本任务节拍快照（历史代码用于 500 Hz 分频判断）。
  TickType_t CAN_Task_SysTick = 0;
    // 上电使能序列：4 个 DM8009（FDCAN2），每帧间隔 30 ms，等待驱动器完成使能。
	DM_Motor_Command(&FDCAN2_TxFrame,&DM_8009_Motor[0],Motor_Enable);
  osDelay(30);
	DM_Motor_Command(&FDCAN2_TxFrame,&DM_8009_Motor[1],Motor_Enable);
  osDelay(30);
  DM_Motor_Command(&FDCAN2_TxFrame,&DM_8009_Motor[2],Motor_Enable);
  osDelay(30);
	DM_Motor_Command(&FDCAN2_TxFrame,&DM_8009_Motor[3],Motor_Enable);
  osDelay(30);
    // 主循环：1 ms 周期：4 路 DM8009 零指令 -> FDCAN1 控制帧 -> 接收分发。
	for(;;)
  {
	
  CAN_Task_SysTick = xTaskGetTickCount();  // 记录本轮节拍
		
	 // CAN-FD	
     // 每个周期对 4 个 DM8009 下发 MIT 指令；此处全零，仅保活不输出力矩。
	 DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[0],0,0,0,0,0);
	 DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[1],0,0,0,0,0);
   DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[2],0,0,0,0,0);	
   DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&DM_8009_Motor[3],0,0,0,0,0);
		
   FDCAN1_TxFrame.Header.Identifier = CAN_SHOOT_ALL_ID;  // 底盘/发射统一控制帧 ID
   FDCAN1_TxFrame.Data[0] = (uint8_t)(Control_Info.SendValue[0]>>8);	  // 大端打包：高字节在前
	 FDCAN1_TxFrame.Data[1] = (uint8_t)(Control_Info.SendValue[0]);	
	 FDCAN1_TxFrame.Data[2] = (uint8_t)(Control_Info.SendValue[1]>>8);	
	 FDCAN1_TxFrame.Data[3] = (uint8_t)(Control_Info.SendValue[1]);	
   USER_FDCAN_AddMessageToTxFifoQ(&FDCAN1_TxFrame);  // 直接写 HAL 发送 FIFO，返回值在此丢弃
		
	 if(CAN_Task_SysTick % 2 == 0){  // 预留的 500 Hz 分频占位，当前分支为空
	 
	 //500Hz发送 请保证所有任务osDelay(1)
	 
	 }	
        // 分发本轮环形队列中的接收帧（每路最多 32 帧）。
		BSP_CAN_ProcessRx();
        osDelay(1);  // 1 ms 调度周期
  }
 

#else
  // 机械臂构建：无发送逻辑，只处理接收；被中断唤醒或 10 ms 超时后继续。
  for (;;)
  {
    BSP_CAN_ProcessRx();  // 消费三路 FDCAN 的环形队列
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));  // 等待 RX 中断通知，10 ms 超时兜底
  }
#endif
}


