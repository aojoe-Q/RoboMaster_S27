/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : INS_Task.c
  * @brief          : INS task
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */
/* 模块职责：IMU（BMI088）姿态解算：加速度二阶低通 -> 四元数 EKF -> 欧拉角/角速度，并做加热片温度闭环。 */
/* 运行方式：INS_Task 任务，osDelayUntil(1) -> 标称 1 kHz；EKF 的 dt 固定按 1 ms 传入。 */
/* 依赖：bmi088（SPI 传感器）、Quaternion（EKF）、lpf（二阶低通）、bsp_pwm（加热片 PWM）。 */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "INS_Task.h"
#include "bmi088.h"
#include "lpf.h"
#include "pid.h"
#include "config.h"
#include "tim.h"
#include "Quaternion.h"
#include "bsp_pwm.h"

// 全局姿态信息：加速度、角速度（rad/s 与 度/秒 两套）、欧拉角、yaw 累计圈数与总角度。
/**
  * @brief the structure that contains the information for the INS.
  */
INS_Info_Typedef INS_Info; 

// 二阶低通系数（3 个）：三轴加速度共用，系数由采样率与截止频率整定。
/**
  * @brief the array that contains the data of LPF2p coefficients.
  */
static float INS_LPF2p_Alpha[3]={1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

// 三轴加速度各一个二阶低通滤波器实例（状态独立）。
/**
  * @brief the structure that contains the Information of accel LPF2p.
  */
LowPassFilter2p_Info_TypeDef INS_AccelPF2p[3];  

// 四元数 EKF 的状态转移矩阵 A 初值：6x6 单位阵（姿态四元数 4 维 + 陀螺零偏 2 维）。
/**
  * @brief the Initialize data of state transition matrix.
  */
static float QuaternionEKF_A_Data[36]={1, 0, 0, 0, 0, 0,
                                       0, 1, 0, 0, 0, 0,
                                       0, 0, 1, 0, 0, 0,
                                       0, 0, 0, 1, 0, 0,
                                       0, 0, 0, 0, 1, 0,
                                       0, 0, 0, 0, 0, 1};

// 后验协方差矩阵 P 初值：四元数 4 维对角 100000、陀螺零偏 2 维对角 100，其余非对角元素 0.1。
/**
  * @brief the Initialize data of posteriori covariance matrix.
  */																
static float QuaternionEKF_P_Data[36]= {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                        0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                        0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                        0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                        0.1, 0.1, 0.1,   0.1,  100, 0.1,
                                        0.1, 0.1, 0.1,   0.1,  0.1, 100};

// 温控 PID 参数：KP=1200、KI=20，其余为 0，末位 Output_MAX=2000 为输出限幅。
/**
  * @brief the Initialize data of Temperature Control PID.
  */																				
static float TemCtrl_PID_Param[PID_PARAMETER_NUM]={1200,20,0,0,0,0,2000};

// 温度控制 PID 实例（位置式）。
/**
  * @brief the structure that contains the Information of Temperature Control PID.
  */
PID_Info_TypeDef TempCtrl_PID;

// 内部接口：初始化滤波器/EKF/PID，以及 200 Hz 的加热片温度闭环。
/**
 * @brief Initializes the INS_Task.
 */
static void INS_Task_Init(void);

/**
  * @brief  Control the BMI088 temperature
  */
static void BMI088_Temp_Control(float temp);

/* USER CODE BEGIN Header_INS_Task */
/**
  * @brief  Function implementing the StartINSTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_INS_Task */
/* 任务入口：初始化后进入 1 kHz 循环：采样 -> 低通 -> EKF -> 欧拉角/角速度 -> 温控分频。 */
// 参数：argument 未使用；返回：无（常驻任务）。
void INS_Task(void const * argument)
{
  /* USER CODE BEGIN INS_Task */
  TickType_t INS_Task_SysTick = 0;


	/* Initializes the INS_Task. */
	INS_Task_Init();
	
  /* Infinite loop */
  for(;;)
  {
    INS_Task_SysTick = osKernelSysTick();  // 本轮起始节拍（osDelayUntil 以此为基准）
		
		/* Update the BMI088 measurement */
    BMI088_Info_Update(&BMI088_Info);  // 读取加速度/角速度/温度（SPI 阻塞读）

    /* Accel measurement LPF2p */
    INS_Info.Accel[0]   =   LowPassFilter2p_Update(&INS_AccelPF2p[0],BMI088_Info.Accel[0]);
    INS_Info.Accel[1]   =   LowPassFilter2p_Update(&INS_AccelPF2p[1],BMI088_Info.Accel[1]);
    INS_Info.Accel[2]   =   LowPassFilter2p_Update(&INS_AccelPF2p[2],BMI088_Info.Accel[2]);
		
    /* Update the INS gyro in radians */
		INS_Info.Gyro[0]   =   BMI088_Info.Gyro[0];
    INS_Info.Gyro[1]   =   BMI088_Info.Gyro[1];
    INS_Info.Gyro[2]   =   BMI088_Info.Gyro[2];
		
		/* Update the QuaternionEKF */
    QuaternionEKF_Update(&Quaternion_Info,INS_Info.Gyro,INS_Info.Accel,0.001f);  // dt = 1 ms，与任务周期一致
		
    memcpy(INS_Info.Angle,Quaternion_Info.EulerAngle,sizeof(INS_Info.Angle));  // 四元数解算出的欧拉角（弧度）

		/* Update the Euler angle in degrees. */
    INS_Info.Pitch_Angle = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_PITCH]*57.295779513f;  // 57.295779513 = 180/pi，弧度转角度
    INS_Info.Yaw_Angle   = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_YAW]   *57.295779513f;
    INS_Info.Roll_Angle  = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_ROLL]*57.295779513f;
		
		/* Update the yaw total angle */
		if(INS_Info.Yaw_Angle - INS_Info.Last_Yaw_Angle < -180.f)  // 过 -180/180 度边界时累计圈数，得到连续 yaw
		{
			INS_Info.YawRoundCount++;
		}
		else if(INS_Info.Yaw_Angle - INS_Info.Last_Yaw_Angle > 180.f)
		{
			INS_Info.YawRoundCount--;
		}
		INS_Info.Last_Yaw_Angle = INS_Info.Yaw_Angle;
		
		INS_Info.Yaw_TolAngle = INS_Info.Yaw_Angle + INS_Info.YawRoundCount*360.f;  // 展开后的连续 yaw：yaw + 圈数 x 360 度
		
    /* Update the INS gyro in degrees */
    INS_Info.Pitch_Gyro = INS_Info.Gyro[IMU_GYRO_INDEX_PITCH]*RadiansToDegrees;  // 角速度转 度/秒，供云台/底盘控制使用
    INS_Info.Yaw_Gyro   = INS_Info.Gyro[IMU_GYRO_INDEX_YAW]*RadiansToDegrees;
    INS_Info.Roll_Gyro  = INS_Info.Gyro[IMU_GYRO_INDEX_ROLL]*RadiansToDegrees;
		
		if(INS_Task_SysTick%5 == 0)  // 5 分频 -> 200 Hz 温控，避免频繁写 PWM
		{
			BMI088_Temp_Control(BMI088_Info.Temperature);
		}

    osDelayUntil(&INS_Task_SysTick,1);  // 严格 1 kHz 周期，错过节拍时自动对齐下一拍
		
  }
  /* USER CODE END INS_Task */
}
//------------------------------------------------------------------------------
/**
 * @brief Initializes the INS_Task.
 */
/* 初始化：装载三轴低通系数、温控 PID 与四元数 EKF。 */
// 调用上下文：INS_Task 启动时执行一次。
static void INS_Task_Init(void)
{
  /* Initializes the Second order lowpass filter  */
  LowPassFilter2p_Init(&INS_AccelPF2p[0],INS_LPF2p_Alpha);
  LowPassFilter2p_Init(&INS_AccelPF2p[1],INS_LPF2p_Alpha);
  LowPassFilter2p_Init(&INS_AccelPF2p[2],INS_LPF2p_Alpha);
	
  /* Initializes the Temperature Control PID  */
	PID_Init(&TempCtrl_PID,PID_POSITION,TemCtrl_PID_Param);  // 位置式 PID
	
  /* Initializes the Quaternion EKF */
	QuaternionEKF_Init(&Quaternion_Info,10.f, 0.001f, 1000000.f,QuaternionEKF_A_Data,QuaternionEKF_P_Data);  // 传入过程噪声/测量噪声/初始协方差以及 A、P 初值
}
//------------------------------------------------------------------------------
/**
  * @brief  Control the BMI088 temperature
  * @param  temp  measure of the BMI088 temperature
  * @retval none
  */
/* 温度闭环：以 40 摄氏度为目标做 PID，输出限幅后写入加热片 PWM。 */
// 参数：Temp 当前温度（摄氏度）；返回：无。
static void BMI088_Temp_Control(float Temp)
{
	PID_Calculate(&TempCtrl_PID,40.f,Temp);  // 目标温度固定 40 摄氏度
	
	VAL_LIMIT(TempCtrl_PID.Output,-TempCtrl_PID.Param.LimitOutput,TempCtrl_PID.Param.LimitOutput);  // 限幅到正负 LimitOutput，防止积分饱和/过冲

	Heat_Power_Control((uint16_t)(TempCtrl_PID.Output));  // 输出转 uint16_t 占空比，驱动加热片
}
//------------------------------------------------------------------------------


