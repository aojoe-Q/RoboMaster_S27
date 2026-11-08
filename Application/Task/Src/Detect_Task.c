/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Detect_Task.h
  * @brief          : Detect task
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */
/* 模块职责：失联检测：周期性刷新遥控器/图传链路在线状态，供控制层做安全降级。 */
/* 运行方式：Detect_Task 任务，osDelay(1) -> 标称 1 kHz；纯状态轮询，无阻塞操作。 */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "Detect_Task.h"
#include "Control_Task.h"
#include "Remote_Control.h"
#include "bsp_gpio.h"

// 在线/掉线判定阈值：turn on = 800，turn off = 4150（沿用旧版检测逻辑，具体单位由 Remote_Message_Moniter 定义）。
/**
  * @note turn on:  800
	*       turn off: 4150
	*/

/* USER CODE BEGIN Header_Detect_Task */
/**
* @brief Function implementing the StartDetectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Detect_Task */
/* 任务入口：持续调用 Remote_Message_Moniter() 刷新 remote_ctrl 的在线标志。 */
// 参数：argument 未使用；返回：无（常驻任务）。
void Detect_Task(void const * argument)
{
  /* USER CODE BEGIN Detect_Task */
//  TickType_t systick = 0;
	

  /* Infinite loop */
  for(;;)
  {

    Remote_Message_Moniter(&remote_ctrl);   // 内部按节拍累计/清零失联计数并更新在线状态
    osDelay(1);  // 1 ms 轮询
  }
  /* USER CODE END Detect_Task */
}

		
		
		
		
		
		
		
	