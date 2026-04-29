#ifndef __HEADFILE_H
#define __HEADFILE_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "stm32f4xx_hal_gpio.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "MagneticSensor.h"
#include "foc_utils.h"
#include "FOCMotor.h"
#include "BLDCMotor.h"
#include "lowpass_filter.h"
#include "pid.h"

/* 驱动使能 */
#define M1_Enable    HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET)
#define M1_Disable   HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET)

/* 传感器选择：AS5600(I2C) 或 TLE5012B(SPI) */
#define M1_AS5600   1
#define M1_TLE5012B 0

/* PWM 周期（与 TIM2 ARR 保持一致，中心对齐模式 duty 范围 0~PWM_Period）*/
#define PWM_Period  3360u

/* DWT 微秒级计时：使用 Cortex-M4 DWT->CYCCNT 32位递增计数器
   调用 DWT_Init() 后即可通过 DWT->CYCCNT 读取当前 CPU 周期数
   elapsed_s = (uint32_t)(now - prev) / (float)SystemCoreClock  */
static inline void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

#endif

