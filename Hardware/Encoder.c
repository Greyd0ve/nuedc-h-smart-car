/*
 * 文件：Hardware/Encoder.c
 * 作用：左右编码器读取。
 *
 * 当前硬件接线：
 *   左轮编码器 E1：E1A -> PA6，E1B -> PA7
 *   右轮编码器 E2：E2A -> PB6，E2B -> PB7
 *
 * 说明：
 *   PA6/PA7 正好对应 TIM3_CH1/TIM3_CH2，可使用 TIM3 编码器接口模式。
 *   PB6/PB7 正好对应 TIM4_CH1/TIM4_CH2，可使用 TIM4 编码器接口模式。
 *
 *   旧版 EXTI 四倍频方案不适合当前接线，因为 PA6 和 PB6 同属 EXTI_Line6，
 *   PA7 和 PB7 同属 EXTI_Line7，STM32F1 每条 EXTI Line 同一时刻只能映射到一个 GPIO 端口。
 *   因此本版改为硬件定时器编码器模式，避免 EXTI 线冲突。
 */
#include "stm32f10x.h"
#include "Encoder.h"

/*
 * 如果小车前进时网页 CH5 实际前进速度为负，优先把这两个符号同时取反。
 * 如果只有某一侧轮子方向反了，只单独修改对应一侧。
 */
#define LEFT_ENCODER_SIGN   (-1)
#define RIGHT_ENCODER_SIGN  (+1)

void Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);

    /* PA6/PA7 = TIM3_CH1/TIM3_CH2，PB6/PB7 = TIM4_CH1/TIM4_CH2。 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* TIM3/TIM4 均使用 16 位向上计数，ARR 设为 0xFFFF 方便通过 int16_t 读取增量。 */
    TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
    TIM_TimeBaseInitStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

    /*
     * 编码器模式 TI12：CH1/CH2 都参与计数。
     * 如果方向整体相反，优先修改 LEFT_ENCODER_SIGN / RIGHT_ENCODER_SIGN，
     * 不建议先改这里的极性，便于后续维护。
     */
    TIM_EncoderInterfaceConfig(TIM3,
                               TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising,
                               TIM_ICPolarity_Rising);
    TIM_EncoderInterfaceConfig(TIM4,
                               TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising,
                               TIM_ICPolarity_Rising);

    /* 输入滤波，抑制编码器毛刺。若高速下丢脉冲，可把 Filter 从 6 调小到 3 或 0。 */
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 6;
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
    TIM_ICInit(TIM4, &TIM_ICInitStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
    TIM_ICInit(TIM4, &TIM_ICInitStructure);

    TIM_SetCounter(TIM3, 0);
    TIM_SetCounter(TIM4, 0);

    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

/* 读取左轮过去一个控制周期内的增量，并立即清零。 */
int16_t Encoder_GetLeftDelta(void)
{
    int16_t temp;

    temp = (int16_t)TIM_GetCounter(TIM3);
    TIM_SetCounter(TIM3, 0);

    return (int16_t)(temp * LEFT_ENCODER_SIGN);
}

/* 读取右轮过去一个控制周期内的增量，并立即清零。 */
int16_t Encoder_GetRightDelta(void)
{
    int16_t temp;

    temp = (int16_t)TIM_GetCounter(TIM4);
    TIM_SetCounter(TIM4, 0);

    return (int16_t)(temp * RIGHT_ENCODER_SIGN);
}

void Encoder_ClearAll(void)
{
    TIM_SetCounter(TIM3, 0);
    TIM_SetCounter(TIM4, 0);
}
