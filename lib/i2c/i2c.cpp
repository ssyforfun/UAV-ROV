#include "i2c.h"
#include "cmsis_os.h"

// MS5837-30BA
// 过采样是 OSR=512 转换时间 1.06ms, OSR=8192-->18.08ms
// OSR越大精度越高
// SCL max = 400kHz, SDA duty range 20% min 80% max
// piezo-resistive sensor, 24bit pressure adc, 24bit temperature adc, 实际内部共有一个adc
// address 7bit = 0x76
//
// step:
// -- 1 reset
// -- 2 read prom (128 bit of calibration words), 计算温度和压力要用到
// -- 3 write start d1 conversion, wait, read adc value
// -- 4 write start d2 conversion, wait, read adc value
// -- 5 calculate pressure & temperature
//

// 用户需要定义的
#define I2C_ADDRESS (0x76 << 1)  // i2c地址 MS5837-30BA
#define I2C_CLOCK_SPEED 400000   // i2c通讯速率
#define I2C_INTERRUPT_PRIOR (13) // I2C中断优先级, 要在5-15之间的数值 (freertos有要求)

static I2C_HandleTypeDef I2cHandle;
static uint8_t aTxBuffer[16];
static uint8_t aRxBuffer[16];
static volatile bool sendOK = false;
static volatile bool receivedOK = false;
static volatile bool errOccur = false;
static volatile bool wakeup = false;

uint32_t d1Word, d2Word;
uint16_t promWords[8];
static uint8_t crc4_Gen(uint16_t *n_prom);

double Tref, Tempsens, offt1, tco, senst1, tcs;

float temperature, pressure;

static void i2c_init();

void vThreadI2c(void *pvParameters)
{
    i2c_init();

    osDelay(10);
    uint32_t delayBase = 10;
    if (I2C_CLOCK_SPEED == 400000)
        delayBase = 1;
    else
        delayBase = 2;

    while (1)
    {
        sendOK = false;
        receivedOK = false;

        if (!wakeup)
        {
            // 唤醒
            aTxBuffer[0] = 0x1E; // reset command
            HAL_I2C_Master_Transmit_IT(&I2cHandle, (uint16_t)I2C_ADDRESS, (uint8_t *)aTxBuffer, 1);
            osDelay(1000);
            // read prom words C0-C6, address: 0xA0-AC
            int i;
            for (i = 0; i < 7; i++)
            {
                aTxBuffer[0] = ((0x50 + i) << 1);
                HAL_I2C_Master_Transmit_IT(&I2cHandle, (uint16_t)I2C_ADDRESS, (uint8_t *)aTxBuffer, 1);
                osDelay(10 * delayBase);
                if (!sendOK) // 如果不成功则停止, 重新复位操作
                    break;
                HAL_I2C_Master_Receive_IT(&I2cHandle, (uint16_t)I2C_ADDRESS, (uint8_t *)aRxBuffer, 2);
                osDelay(2 * delayBase);
                if (!receivedOK)
                    break;
                else
                {
                    promWords[i] = ((aRxBuffer[0] << 8) + aRxBuffer[1]);
                }
                sendOK = false;
                receivedOK = false;
            }
            if (i == 7)
            { // 检查校验C0-C6的值
                uint8_t crcReceived = ((promWords[0] & 0xF000) >> 12);
                uint8_t crcgen = crc4_Gen(promWords);
                promWords[0] |= ((((int16_t)crcReceived) << 12) & 0xF000);
                if (crcReceived == crcgen)
                {
                    Tref = promWords[5] * 256.0;
                    Tempsens = (promWords[6] * 1.0) / (1 << 23);
                    offt1 = (promWords[2] * 1.0) * (1 << 16);
                    tco = (promWords[4] * 1.0) / (1 << 7);
                    senst1 = (promWords[1] * 1.0) * (1 << 15);
                    tcs = (promWords[3] * 1.0) / (1 << 8);
                    wakeup = true;
                }
                else
                    continue; // 校验失败
            }
            else
                continue;
        }
        sendOK = false;
        receivedOK = false;

        // start D1 conversion
        aTxBuffer[0] = 0x48; // 0x4x-->D1(pressure) conversion, bit0-3: conversion time, 8->4096
        HAL_I2C_Master_Transmit_IT(&I2cHandle, (uint16_t)I2C_ADDRESS, (uint8_t *)aTxBuffer, 1);
        osDelay(30 * delayBase);
        if (!sendOK)
            continue;
        sendOK = false;
        // start adc read
        aTxBuffer[0] = 0x00;
        HAL_I2C_Master_Transmit_IT(&I2cHandle, (uint16_t)I2C_ADDRESS, (uint8_t *)aTxBuffer, 1);
        osDelay(10 * delayBase);
        if (!sendOK)
            continue;
        sendOK = false;
        HAL_I2C_Master_Receive_IT(&I2cHandle, (uint16_t)I2C_ADDRESS, (uint8_t *)aRxBuffer, 3);
        osDelay(2 * delayBase);
        if (!receivedOK)
            continue;
        receivedOK = false;
        d1Word = ((aRxBuffer[0] << 16) + (aRxBuffer[1] << 8) + (aRxBuffer[2] << 0));

        // start D2 conversion
        aTxBuffer[0] = 0x58; // 0x5x-->D2(temperature) conversion, bit0-3: conversion time, 8->4096
        HAL_I2C_Master_Transmit_IT(&I2cHandle, (uint16_t)I2C_ADDRESS, (uint8_t *)aTxBuffer, 1);
        osDelay(30 * delayBase);
        if (!sendOK)
            continue;
        sendOK = false;
        // start adc read
        aTxBuffer[0] = 0x00;
        HAL_I2C_Master_Transmit_IT(&I2cHandle, (uint16_t)I2C_ADDRESS, (uint8_t *)aTxBuffer, 1);
        osDelay(10 * delayBase);
        if (!sendOK)
            continue;
        sendOK = false;
        HAL_I2C_Master_Receive_IT(&I2cHandle, (uint16_t)I2C_ADDRESS, (uint8_t *)aRxBuffer, 3);
        osDelay(2 * delayBase);
        if (!receivedOK)
            continue;
        receivedOK = false;
        d2Word = ((aRxBuffer[0] << 16) + (aRxBuffer[1] << 8) + (aRxBuffer[2] << 0));

        // 提取数据
        int dT = (int)(d2Word - Tref);
        int temp = 2000 + dT * Tempsens;
        int64_t off = offt1 + tco * dT;
        int64_t sens = senst1 + tcs * dT;
        int press = ((((d1Word * sens) >> 21) - off) >> 13);

        temperature = temp * 0.01f; // ℃
        pressure = press * 0.1f;    // mbar
        osDelay(1);
    }
}

/** I2C外设配置 ---- I2C寄存器初始化
 *
 */
static void i2c_init()
{
    // 原来有值，要复位
    if (I2cHandle.Instance != NULL)
    {
        HAL_NVIC_DisableIRQ(I2Cx_ER_IRQn);
        HAL_NVIC_DisableIRQ(I2Cx_EV_IRQn);
    }

    I2cHandle.Instance = I2Cx;

    I2cHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    I2cHandle.Init.ClockSpeed = I2C_CLOCK_SPEED;
    I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2cHandle.Init.DutyCycle = I2C_DUTYCYCLE_2;
    I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2cHandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    I2cHandle.Init.OwnAddress1 = I2C_ADDRESS;
    I2cHandle.Init.OwnAddress2 = 0xFE;

    I2cHandle.State = HAL_I2C_STATE_RESET;
    HAL_I2C_Init(&I2cHandle);
}

/** I2C外设配置 ---- GPIO & DMA & intterupt
 * @brief I2C MSP Initialization
 *        This function configures the hardware resources used in this example:
 *           - Peripheral's clock enable
 *           - Peripheral's GPIO Configuration
 *           - DMA configuration for transmission request by peripheral
 *           - NVIC configuration for DMA interrupt request enable
 * @param hi2c: I2C handle pointer
 * @retval None
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    I2Cx_SCL_GPIO_CLK_ENABLE();
    I2Cx_SDA_GPIO_CLK_ENABLE();

    /* Enable I2Cx clock */
    I2Cx_CLK_ENABLE();

    /*##-2- Configure peripheral GPIO ##########################################*/
    /* I2C TX GPIO pin configuration  */
    GPIO_InitStruct.Pin = I2Cx_SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = I2Cx_SCL_AF;

    HAL_GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStruct);

    /* I2C RX GPIO pin configuration  */
    GPIO_InitStruct.Pin = I2Cx_SDA_PIN;
    GPIO_InitStruct.Alternate = I2Cx_SDA_AF;

    HAL_GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStruct);

    /*##-5- Configure the NVIC for I2C #########################################*/
    /* NVIC for I2C1 */
    HAL_NVIC_SetPriority(I2Cx_ER_IRQn, I2C_INTERRUPT_PRIOR, 0);
    HAL_NVIC_EnableIRQ(I2Cx_ER_IRQn);
    HAL_NVIC_SetPriority(I2Cx_EV_IRQn, I2C_INTERRUPT_PRIOR, 0);
    HAL_NVIC_EnableIRQ(I2Cx_EV_IRQn);
}

/** I2C外设复位 ---- GPIO & DMA & intterupt
 * @brief I2C MSP De-Initialization
 *        This function frees the hardware resources used in this example:
 *          - Disable the Peripheral's clock
 *          - Revert GPIO, DMA and NVIC configuration to their default state
 * @param hi2c: I2C handle pointer
 * @retval None
 */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    /*##-1- Reset peripherals ##################################################*/
    I2Cx_FORCE_RESET();
    I2Cx_RELEASE_RESET();

    /*##-2- Disable peripherals and GPIO Clocks ################################*/
    /* Configure I2C Tx as alternate function  */
    HAL_GPIO_DeInit(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_PIN);
    /* Configure I2C Rx as alternate function  */
    HAL_GPIO_DeInit(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_PIN);

    /*##-5- Disable the NVIC for I2C ###########################################*/
    HAL_NVIC_DisableIRQ(I2Cx_ER_IRQn);
    HAL_NVIC_DisableIRQ(I2Cx_EV_IRQn);
}

extern "C" // 中断函数需要用C语言写
{
    /**
     * @brief  This function handles I2C event interrupt request.
     * @param  None
     * @retval None
     * @Note   This function is redefined in "main.h" and related to I2C data transmission
     */
    void I2Cx_EV_IRQHandler(void)
    {
        HAL_I2C_EV_IRQHandler(&I2cHandle);
    }

    /**
     * @brief  This function handles I2C error interrupt request.
     * @param  None
     * @retval None
     * @Note   This function is redefined in "main.h" and related to I2C error
     */
    void I2Cx_ER_IRQHandler(void)
    {
        HAL_I2C_ER_IRQHandler(&I2cHandle);
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
    sendOK = true;
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
    receivedOK = true;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
    errOccur = true;
}

static uint8_t crc4_Gen(uint16_t *n_prom) // n_prom is n_prom[8]
{
    int cnt;
    uint16_t n_rem = 0;
    uint8_t n_bit;
    n_prom[0] = ((n_prom[0]) & 0x0FFF); // CRC byte is replaced by 0
    n_prom[7] = 0;                      // subsidiary value, set to 0
    for (cnt = 0; cnt < 16; cnt++)      // operation is performed on bytes
    {                                   // choose LSB or MSB
        if ((cnt % 2) == 1)
            n_rem ^= (uint16_t)((n_prom[cnt >> 1]) & 0x00FF);
        else
            n_rem ^= (uint16_t)(n_prom[cnt >> 1] >> 8);
        for (n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & 0x8000)
                n_rem = (n_rem << 1) ^ 0x3000;
            else
                n_rem = (n_rem << 1);
        }
    }
    n_rem = ((n_rem >> 12) & 0x000F); // final 4-bit remainder is CRC code
    return (n_rem ^ 0x00);
}
