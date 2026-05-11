/* 项目适配说明：八路灰度循迹模块通过 AD0/AD1/AD2 选通，OUT 读取当前通道，用于黑色弧线循迹。 */
/*
 * 文件：Hardware/Grayscale.c
 * 作用：八路灰度循迹传感器读取。
 *
 * 本版用于重新分配八路灰度地址线：
 *   AD0 -> PA8
 *   AD1 -> PB3   需要把灰度模块 AD1 飞线/改线到 PB3
 *   AD2 -> PB4   需要把灰度模块 AD2 飞线/改线到 PB4
 *   OUT -> PB0
 *
 * 注意：PB3/PB4 默认也是 JTAG 相关引脚，所以初始化中关闭 JTAG，只保留 SWD。
 */
#include "stm32f10x.h"
#include "Grayscale.h"

#define GS_AD0_PORT GPIOA
#define GS_AD0_PIN  GPIO_Pin_8
#define GS_AD1_PORT GPIOB
#define GS_AD1_PIN  GPIO_Pin_3
#define GS_AD2_PORT GPIOB
#define GS_AD2_PIN  GPIO_Pin_4
#define GS_OUT_PORT GPIOB
#define GS_OUT_PIN  GPIO_Pin_0

static void Grayscale_Select(uint8_t channel)
{
    if (channel & 0x01U)
    {
        GPIO_SetBits(GS_AD0_PORT, GS_AD0_PIN);
    }
    else
    {
        GPIO_ResetBits(GS_AD0_PORT, GS_AD0_PIN);
    }

    if (channel & 0x02U)
    {
        GPIO_SetBits(GS_AD1_PORT, GS_AD1_PIN);
    }
    else
    {
        GPIO_ResetBits(GS_AD1_PORT, GS_AD1_PIN);
    }

    if (channel & 0x04U)
    {
        GPIO_SetBits(GS_AD2_PORT, GS_AD2_PIN);
    }
    else
    {
        GPIO_ResetBits(GS_AD2_PORT, GS_AD2_PIN);
    }
}

void Grayscale_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = GS_AD0_PIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GS_AD1_PIN | GS_AD2_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GS_OUT_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    Grayscale_Select(0);
}

uint8_t Grayscale_RawOUT(void)
{
    return (uint8_t)GPIO_ReadInputDataBit(GS_OUT_PORT, GS_OUT_PIN);
}

uint8_t Grayscale_ReadOne(uint8_t channel)
{
    uint8_t a;
    uint8_t b;
    uint8_t c;

    if (channel >= GRAYSCALE_CHANNELS)
    {
        channel = 0;
    }

    Grayscale_Select(channel);

    for (volatile uint16_t d = 0; d < 2000; d++)
    {
        __NOP();
    }

    a = Grayscale_RawOUT();

    for (volatile uint16_t d = 0; d < 300; d++)
    {
        __NOP();
    }

    b = Grayscale_RawOUT();

    for (volatile uint16_t d = 0; d < 300; d++)
    {
        __NOP();
    }

    c = Grayscale_RawOUT();

    return (uint8_t)(((uint16_t)a + (uint16_t)b + (uint16_t)c) >= 2U);
}

void Grayscale_ReadAll(uint8_t sensor[GRAYSCALE_CHANNELS])
{
    uint8_t i;

    for (i = 0; i < GRAYSCALE_CHANNELS; i++)
    {
        sensor[i] = Grayscale_ReadOne(i);
    }
}
