/**
  ******************************************************************************
  * @file           : Control_Task.c
  * @brief          : Control task
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.1
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */
/* 模块职责：底盘速度环：读取电机反馈速度，按遥控器通道给出目标速度，经位置式 PID 得到控制输出。 */
/* 运行方式：Control_Task 任务，osDelay(1) -> 标称 1 kHz；输出经 Control_Info.SendValue 交给 CAN_Task 打包。 */
/* 依赖：Chassis_Motor（CAN 电机反馈）、remote_ctrl（DBUS 遥控）、USART_Vofa 遥测与 PID 模块。 */

/* Includes ------------------------------------------------------------------*/
#include "Control_Task.h"
#include "cmsis_os.h"
#include "Control_Task.h"
#include "bsp_uart.h"
#include "Remote_Control.h"
#include "PID.h"
#include "Motor.h"
#include "arm_math.h"

// 内部接口：初始化、反馈更新、目标更新、PID 更新（static，仅本文件可见）。
static void Control_Init(Control_Info_Typedef *Control_Info);
static void Control_Measure_Update(Control_Info_Typedef *Control_Info);
static void Control_Target_Update(Control_Info_Typedef *Control_Info);
static void Control_Info_Update(Control_Info_Typedef *Control_Info);

// 全局控制信息：Measure/Target/SendValue 三组量；CAN_Task 读取 SendValue 发送。
Control_Info_Typedef Control_Info;
// 底盘速度环参数：比例 13、积分 0.1，微分/滤波/死区为 0，积分限幅 5000，输出限幅 12000。
//                                  KP   KI   KD  Alpha Deadband  I_MAX   Output_MAX
static float Chassis_PID_Param[7] = {13.f,0.1f,0.f,0.f,  0.f,      5000.f,  12000.f};

// 底盘速度环 PID 实例（位置式）。
PID_Info_TypeDef Chassis_PID;

/* 任务入口：初始化 PID 后进入 1 kHz 循环：反馈 -> 目标 -> PID -> VOFA 遥测。 */
// 参数：argument 未使用；返回：无（常驻任务）。
void Control_Task(void const * argument)
{
  /* USER CODE BEGIN Control_Task */
  TickType_t Control_Task_SysTick = 0;
  
	Control_Init(&Control_Info);
 /* Infinite loop */
	for(;;)
  {
		Control_Task_SysTick = osKernelSysTick();  // 本轮节拍（当前仅记录，未参与分频）

		
      // 控制链路固定顺序：先采样、再给目标、最后统一算 PID，保证同周期数据一致。
	  Control_Measure_Update(&Control_Info);
		Control_Target_Update(&Control_Info);
    Control_Info_Update(&Control_Info);
    USART_Vofa_Justfloat_Transmit(Control_Info.Measure.Chassis_Velocity,0.f,0.f);  // VOFA+ JustFloat 遥测：仅上传实测速度
		
		osDelay(1);  // 1 ms 控制周期
  }
}
  /* USER CODE END Control_Task */

/* 初始化：以 PID_POSITION 模式装载底盘速度环参数。 */
static void Control_Init(Control_Info_Typedef *Control_Info){

  PID_Init(&Chassis_PID,PID_POSITION,Chassis_PID_Param);  // 参数数组顺序与 PID_Init 的约定一致

}

/* 反馈更新：取 0 号底盘电机的反馈速度作为速度环测量值。 */
static void Control_Measure_Update(Control_Info_Typedef *Control_Info){

  Control_Info->Measure.Chassis_Velocity = Chassis_Motor[0].Data.Velocity;  // 当前只用单电机反馈，四轮底盘可扩展为平均/合成速度

}

/* 目标更新：遥控器通道 3（右摇杆纵向）线性映射为目标速度。 */
static void Control_Target_Update(Control_Info_Typedef *Control_Info){

  Control_Info->Target.Chassis_Velocity = remote_ctrl.rc.ch[3] * 5.f;  // 比例系数 5：把摇杆量程映射到速度量纲


}

/* 输出更新：执行 PID 计算，输出截断为 int16_t 存入 SendValue[0]。 */
static void Control_Info_Update(Control_Info_Typedef *Control_Info){
  
   PID_Calculate(&Chassis_PID, Control_Info->Target.Chassis_Velocity, Control_Info->Measure.Chassis_Velocity);  // 误差 = Target - Measure

	 Control_Info->SendValue[0] = (int16_t)(Chassis_PID.Output);  // 强转 int16_t：超范围会截断，依赖 PID 输出限幅保护
	
}

/* 五次多项式平滑插值：Time 归一化到 [0,1] 后给出 S 形过渡曲线；当前无调用者，保留工具函数。 */
// 参数：NowTime 当前时间，UseTime 总时长；返回：0 到 1 的平滑系数。
static float FivePower(float NowTime,float UseTime){
     
	  float Time = (NowTime/UseTime);
	     
    return 10*powf(Time,3) - 15*powf(Time,4) + 6*powf(Time,5);  // 10t^3 - 15t^4 + 6t^5：两端一阶、二阶导数均为 0


}
	