/*
 * 文件：Hardware/Motor.c
 * 作用：TB6612FNG 双路电机驱动封装。
 *
 * 逻辑说明：
 *   1. PWM 由 TIM2_CH1/CH2 输出，对应训练板 PWMA=PA0、PWMB=PA1。
 *   2. 方向由 PB12~PB15 控制，对应 TB6612 的 AIN1/AIN2/BIN1/BIN2。
 *   3. LEFT_MOTOR_DIR_SIGN / RIGHT_MOTOR_DIR_SIGN 用来适配电机安装方向。
 *
 * 本版说明：已按小车训练板修正 PWM 通道映射，方向引脚逻辑保持不变。
 */
#include "stm32f10x.h"
#include "PWM.h"
#include "Motor.h"

/* 若前进时左轮反转，只改这个符号，不要改 PID 或状态机。 */
/*
 * 电机方向适配：
 *   你之前实测“进入循迹后整车向后跑”，说明程序正方向和实车正方向相反。
 *   因此这里采用 LEFT=+1、RIGHT=-1。
 *   如果你更换电机线序或左右电机安装方向后前进又反了，只需要改这两个宏，
 *   不要去改蓝牙解析、循迹 PID 或状态机。
 */
#define LEFT_MOTOR_DIR_SIGN     (-1)
#define RIGHT_MOTOR_DIR_SIGN    (+1)

static int16_t Motor_ClampPWM(int16_t pwm)
{
    if (pwm > (int16_t)PWM_MAX_DUTY)
    {
        return (int16_t)PWM_MAX_DUTY;
    }
    if (pwm < -(int16_t)PWM_MAX_DUTY)
    {
        return -(int16_t)PWM_MAX_DUTY;
    }
    return pwm;
}

/* 初始化方向 GPIO 和 PWM 模块。 */
void Motor_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

    PWM_Init();
    Motor_StopAll();
}

/* 设置左电机 PWM：PWM 正负决定方向，绝对值决定占空比。 */
void Motor_SetLeftPWM(int16_t PWM)
{
    int16_t signedPwm = Motor_ClampPWM(PWM * LEFT_MOTOR_DIR_SIGN);

    if (signedPwm >= 0)
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);
        GPIO_SetBits(GPIOB, GPIO_Pin_13);
        PWM_SetCompareA((uint16_t)signedPwm);
    }
    else
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_12);
        GPIO_ResetBits(GPIOB, GPIO_Pin_13);
        PWM_SetCompareA((uint16_t)(-signedPwm));
    }
}

void Motor_SetRightPWM(int16_t PWM)
{
    int16_t signedPwm = Motor_ClampPWM(PWM * RIGHT_MOTOR_DIR_SIGN);

    if (signedPwm >= 0)
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_14);
        GPIO_SetBits(GPIOB, GPIO_Pin_15);
        PWM_SetCompareB((uint16_t)signedPwm);
    }
    else
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_14);
        GPIO_ResetBits(GPIOB, GPIO_Pin_15);
        PWM_SetCompareB((uint16_t)(-signedPwm));
    }
}

void Motor_SetPWM(int16_t LeftPWM, int16_t RightPWM)
{
    Motor_SetLeftPWM(LeftPWM);
    Motor_SetRightPWM(RightPWM);
}

/* 两路 PWM 清零，方向引脚也拉低，保证停车状态干净。 */
void Motor_StopAll(void)
{
    PWM_SetCompareA(0);
    PWM_SetCompareB(0);
    GPIO_ResetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
}
