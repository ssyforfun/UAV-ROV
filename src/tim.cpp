/************************************************************
 * FileName:        tim.cpp
 * Description:     定时器配置[除了timebase定时器TIM14以外]
 *                  STM32
 * Auther:          Jinsheng
 * CreateDate:      2021-09-05
 * ModifyDate:
 * Company:         Haivoo
 * Contact:         sales@haivoo.com
 *                  www.haivoo.com
 * **********************************************************/

#include <main.h>
#include "tim.h"
#include "stm32f4xx_hal_tim_ex.h"
#include "stm32f4xx_hal_tim.h"

#define PWM_FREQUENCY 50 // 开关频率500kHz

static const float PWM_PERIOD = 1.0f / PWM_FREQUENCY;
static void pwm_init(TIM_HandleTypeDef *htim, TIM_TypeDef *TIM, uint8_t channelmask);

static uint32_t *timCcrAddr[9];
static uint32_t *timArrAddr[9];

void tim_init(void)
{
    TIM_HandleTypeDef htim1, htim3, htim8, htim12;
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();
    __HAL_RCC_TIM12_CLK_ENABLE();

    pwm_init(&htim1, TIM1, 0x07);
    pwm_init(&htim3, TIM3, 0x03);
    pwm_init(&htim8, TIM8, 0x0C);
    pwm_init(&htim12, TIM12, 0x03);

    // add pin configurations here
    // done in gpio_msp.cpp

    // enable
    // register: cr1(cen bit), ccer(cc1e, cc1ne, ...bits),  bdtr(moe bit)
    // see HAL_TIM_PWM_Start, HAL_TIMEx_PWMN_Start
    htim1.Instance->CCER |= 0x0111;  // PWM通道使能
    htim1.Instance->CCMR1 |= 0x0808; // ocref1, ocref2
    htim1.Instance->CCMR2 |= 0x0008; // ocref3

    htim3.Instance->CCER |= 0x0011;
    htim3.Instance->CCMR1 |= 0x0808; // ocref1, ocref2

    htim8.Instance->CCER |= 0x1100;
    htim8.Instance->CCMR2 |= 0x0808; // ocref3, ocref4

    htim12.Instance->CCER |= 0x0011;
    htim12.Instance->CCMR1 |= 0x0808; // ocref1, ocref2

    __HAL_TIM_MOE_DISABLE(&htim1); // 定时器的pin脚输出
    __HAL_TIM_MOE_ENABLE(&htim1);  // 使能定时器的pin脚输出
    __HAL_TIM_MOE_DISABLE(&htim8); // 定时器的pin脚输出
    __HAL_TIM_MOE_ENABLE(&htim8);  // 使能定时器的pin脚输出

    __HAL_TIM_ENABLE(&htim1);
    __HAL_TIM_ENABLE(&htim3);
    __HAL_TIM_ENABLE(&htim8);
    __HAL_TIM_ENABLE(&htim12);

    timCcrAddr[0] = (uint32_t *)&TIM3->CCR1;
    timCcrAddr[1] = (uint32_t *)&TIM3->CCR2;
    timCcrAddr[2] = (uint32_t *)&TIM1->CCR3;
    timCcrAddr[3] = (uint32_t *)&TIM1->CCR2;
    timCcrAddr[4] = (uint32_t *)&TIM1->CCR1;
    timCcrAddr[5] = (uint32_t *)&TIM8->CCR4;
    timCcrAddr[6] = (uint32_t *)&TIM8->CCR3;
    timCcrAddr[7] = (uint32_t *)&TIM12->CCR2;
    timCcrAddr[8] = (uint32_t *)&TIM12->CCR1;

    timArrAddr[0] = (uint32_t *)&TIM3->ARR;
    timArrAddr[1] = (uint32_t *)&TIM3->ARR;
    timArrAddr[2] = (uint32_t *)&TIM1->ARR;
    timArrAddr[3] = (uint32_t *)&TIM1->ARR;
    timArrAddr[4] = (uint32_t *)&TIM1->ARR;
    timArrAddr[5] = (uint32_t *)&TIM8->ARR;
    timArrAddr[6] = (uint32_t *)&TIM8->ARR;
    timArrAddr[7] = (uint32_t *)&TIM12->ARR;
    timArrAddr[8] = (uint32_t *)&TIM12->ARR;

    //PWM_duty_set(3, 0.1f);

    // 设置中断
    // TIM8->DIER |= TIM_DIER_UIE; // update interrupt enable
    // TIM8->SR = 0;               // clear all flags
    // HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
    // HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);

    // TIM1->DIER |= TIM_DIER_UIE; // update interrupt enable
    // TIM1->SR = 0; // clear all flags
    // HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
    // HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

static void pwm_init(TIM_HandleTypeDef *htim, TIM_TypeDef *TIM, uint8_t channelmask)
{
    TIM_MasterConfigTypeDef sMasterConfig = {};
    TIM_OC_InitTypeDef sConfigOC = {};

    uint32_t uwTimclock = tim_CK_INT_get(TIM);                  // get TIM clock cnt frequency
    uint32_t period = (uint32_t)(uwTimclock / (PWM_FREQUENCY)); // calc period
    uint32_t prescaler = period / 65536 + 1;                    // clock prescaler
    period = period / prescaler;

    // 配置三角波PWM格式
    htim->Instance = TIM;
    htim->Init.Prescaler = (prescaler - 1);                        // 分频
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;                   // 计数模式，向上计数
    htim->Init.Period = period - 1;                                // ARR,周期
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;             // 时钟分频因子
    htim->Init.RepetitionCounter = 0;                              // 重复计数器
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; // 自动重装载，disable：计数器立即产生计数溢出，开始新的计数周期；enable：计数器完成当前的计数后，再开始新的计数周期
    HAL_TIM_PWM_Init(htim);

    // 从模式关闭 [独立的PWM], 选择信号给TRGO输出
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;         // 触发更新事件
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE; // 主/从模式选择
    HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig);

    // PWM1模式(out=1 when cnt<ccr), idle和reset状态下输出为0
    sConfigOC.OCMode = TIM_OCMODE_PWM1;              // PWM模式1下，TIMx_CNT<TIMx_CCRn时，输出有效电平；PWM模式2下，TIMx_CNT>TIMx_CCRn时，输出有效电平
    sConfigOC.Pulse = 0;                             // 脉冲值，通过比较计数器中的计数值与该值的大小关系，来转变输出电平的高低
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;      // 有效电平的极性
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;    // 互补有效电平的极性
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;       // 输出通道快速模式，
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;   // 输出通道闲置状态
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; // 互补输出通道闲置状态
    if (channelmask & 0x01)
        HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_1);
    if (channelmask & 0x02)
        HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_2);
    if (channelmask & 0x04)
        HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_3);
    if (channelmask & 0x08)
        HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_4);
}

// speed -1.0f --> 1.0f 代表 1ms->2ms,
void pwm_speed_set(uint8_t index, float speed)
{
    if (speed < -1.0f)
        return;
    if (speed > 1.0f)
        return;

    float ton = (speed + 1.0f) * 5e-4f + 1e-3f;
    float duty = ton * PWM_FREQUENCY;
    PWM_duty_set(index, duty);
}

float pwm_speed_get(uint8_t index)
{
    float duty = PWM_duty_get(index);
    float ton = duty * PWM_PERIOD;
    float speed = (ton - 1e-3f) * 2e3f - 1.0f;
    if (speed < -2.0f)
        speed = 0;
    return speed;
}

void pwm_shutdown(void)
{
    for (int i = 0; i < 9; i++)
    {
        *timCcrAddr[i] = 0;
    }
}

void pwm_stop(int index)
{
    if (index < 9)
        *timCcrAddr[index] = 0;
}

void PWM_duty_set(uint8_t index, float duty)
{
    if (index > 8)
        return;
    *timCcrAddr[index] = (uint32_t)(duty * (*timArrAddr[index] + 1));
}

float PWM_duty_get(uint8_t index)
{
    if (index > 8)
        return 0;

    return (((float)*timCcrAddr[index]) / (*timArrAddr[index] + 1));
}

void pwm_caputure_init(void)
{
    TIM_HandleTypeDef TimHandle;
    TIM_IC_InitTypeDef sConfig;
    TIM_SlaveConfigTypeDef sSlaveConfig;

    __HAL_RCC_TIM5_CLK_ENABLE();
    uint32_t clk = tim_CK_INT_get(TIM5);
    uint32_t prescaler = (uint32_t)((clk / 1000000) - 1); // 使时钟频率变为1MHz
    // ##-1- Configure the TIM peripheral
    TimHandle.Instance = TIM5;
    //  Initialize TIMx peripheral as follow:
    // + Period = 0xFFFF
    // + Prescaler = 0
    // + ClockDivision = 0
    // + Counter direction = Up
    TimHandle.Init = {0};
    TimHandle.Init.Period = 0xFFFF;
    TimHandle.Init.Prescaler = prescaler;
    TimHandle.Init.ClockDivision = 0;
    TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    TimHandle.Init.RepetitionCounter = 0;
    HAL_TIM_IC_Init(&TimHandle);

    // ##-2- Configure the Input Capture channels
    //  Common configuration
    sConfig.ICPrescaler = TIM_ICPSC_DIV1;
    sConfig.ICFilter = 0;
    // Configure the Input Capture of channel 1
    sConfig.ICPolarity = TIM_ICPOLARITY_FALLING;      // 下降沿采样
    sConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI; // IC1映射到TI2 // TIM_ICSELECTION_DIRECTTI
    HAL_TIM_IC_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1);
    // Configure the Input Capture of channel 2
    sConfig.ICPolarity = TIM_ICPOLARITY_RISING;     // 上升沿采样
    sConfig.ICSelection = TIM_ICSELECTION_DIRECTTI; // IC2映射到TI2 // TIM_ICSELECTION_INDIRECTTI
    HAL_TIM_IC_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2);

    // ##-3- Configure the slave mode
    // Select the slave Mode: Reset Mode
    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
    sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
    sSlaveConfig.TriggerFilter = 0;
    sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
    sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
    HAL_TIM_SlaveConfigSynchronization(&TimHandle, &sSlaveConfig);

    // ##-4- Start the Input Capture in interrupt mode
    HAL_TIM_IC_Start_IT(&TimHandle, TIM_CHANNEL_1);

    // ##-5- Start the Input Capture in interrupt mode
    HAL_TIM_IC_Start_IT(&TimHandle, TIM_CHANNEL_2);

    TIM5->SR = 0x00;
    HAL_NVIC_SetPriority(TIM5_IRQn, POS_InputCapture_Prior, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
}

void input_caputure_pwm_callback(void)
{
    uint32_t status = TIM5->SR;

    TIM5->SR = 0x00; // clear all flags

    if (status & TIM_SR_CC1IF) // 占空比位置触发
    {
        ppmInst.pwmCallback(false, TIM5->CCR1);
    }

    if (status & TIM_SR_CC2IF) // 周期位置触发
    {
        ppmInst.pwmCallback(true, TIM5->CCR2);
    }
}

