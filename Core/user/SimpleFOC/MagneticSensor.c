

#include "headfile.h"


/************************************************
本程序仅供学习，引用代码请标明出处
使用教程：https://blog.csdn.net/loop222/article/details/120471390
创建日期：20210925
作    者：loop222 @郑州
************************************************/
/******************************************************************************/
long  cpr;
float full_rotation_offset;
long  angle_data_prev;
unsigned long velocity_calc_timestamp;
float angle_prev;
/******************************************************************************/

/******************************************************************************/
/* AS5600 via I2C1 (HAL)                                                      */
/******************************************************************************/
#define  AS5600_Address  0x36
#define  RAW_Angle_Hi    0x0C   /* 12-bit raw angle high byte register */
#define  AS5600_CPR      4096
/******************************************************************************/
static unsigned short I2C_getRawCount(void)
{
    uint8_t buf[2] = {0, 0};
    /* HAL 使用左移1位的7位地址 */
    HAL_I2C_Mem_Read(&hi2c1,
                     (uint16_t)(AS5600_Address << 1),
                     RAW_Angle_Hi,
                     I2C_MEMADD_SIZE_8BIT,
                     buf, 2, 100);
    return ((unsigned short)buf[0] << 8) | buf[1];
}
/******************************************************************************/



/******************************************************************************/
/* TLE5012B via SPI2 (HAL) — 仅当 M1_TLE5012B==1 时编译                      */
/******************************************************************************/
#if M1_TLE5012B

#define READ_ANGLE_VALUE  0x8020
#define TLE5012B_CPR      32768

/* CS: PB8 */
#define SPI2_CS1_L  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET)
#define SPI2_CS1_H  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET)

extern SPI_HandleTypeDef hspi2;

/* TLE5012B SSC 半双工：读数据时将 MOSI(PB15) 切为浮空输入 */
static void SPI2_TX_OFF(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pin  = GPIO_PIN_15;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &g);
}

/* 将 PB15 切回 SPI2_MOSI 复用推挽输出 */
static void SPI2_TX_ON(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_15;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &g);
}

static unsigned short SPIx_ReadWriteByte(unsigned short byte)
{
    uint16_t rx = 0;
    HAL_SPI_TransmitReceive(&hspi2,
                            (uint8_t *)&byte,
                            (uint8_t *)&rx,
                            1, 100);
    return rx;
}

static unsigned short ReadTLE5012B_1(unsigned short Comm)
{
    unsigned short u16Data;

    SPI2_CS1_L;
    SPIx_ReadWriteByte(Comm);
    SPI2_TX_OFF();
    /* Twr_delay ≥ 130 ns */
    __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP();
    u16Data = SPIx_ReadWriteByte(0xFFFF);
    SPI2_CS1_H;
    SPI2_TX_ON();
    return u16Data;
}

#endif /* M1_TLE5012B */

/******************************************************************************/
void MagneticSensor_Init(void)
{
#if M1_AS5600
    cpr = AS5600_CPR;
    angle_data_prev = I2C_getRawCount();
#elif M1_TLE5012B
    cpr = TLE5012B_CPR;
    angle_data_prev = ReadTLE5012B_1(READ_ANGLE_VALUE) & 0x7FFF;
#endif

    full_rotation_offset = 0;
    velocity_calc_timestamp = DWT->CYCCNT;
}
/******************************************************************************/
float getAngle(void)
{
    float angle_data, d_angle;

#if M1_AS5600
    angle_data = (float)I2C_getRawCount();
#elif M1_TLE5012B
    angle_data = (float)(ReadTLE5012B_1(READ_ANGLE_VALUE) & 0x7FFF);
#else
    angle_data = 0.0f;
#endif

    /* 追踪整圈圈数，将角度范围从 [0, 2π] 扩展 */
    d_angle = angle_data - (float)angle_data_prev;
    if(fabs(d_angle) > (0.8f * (float)cpr))
        full_rotation_offset += (d_angle > 0) ? -_2PI : _2PI;
    angle_data_prev = (long)angle_data;

    return full_rotation_offset + (angle_data / (float)cpr) * _2PI;
}
/******************************************************************************/
/* 轴角速度计算 —— 使用 DWT->CYCCNT 32 位递增计数器（@168 MHz，~6 ns 精度）  */
float getVelocity(void)
{
    uint32_t now_us;
    float Ts, angle_c, vel;

    now_us = DWT->CYCCNT;
    /* 无符号减法自动处理 32 位回绕，除以 CPU 频率得秒数 */
    Ts = (float)(uint32_t)(now_us - (uint32_t)velocity_calc_timestamp)
         / (float)SystemCoreClock;
    if(Ts == 0.0f || Ts > 0.5f) Ts = 1e-3f;

    angle_c = getAngle();
    vel = (angle_c - angle_prev) / Ts;

    angle_prev = angle_c;
    velocity_calc_timestamp = (unsigned long)now_us;
    return vel;
}
/******************************************************************************/




