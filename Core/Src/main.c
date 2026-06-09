/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void
SystemClock_Config(void);
static void
MPU_Config(void);
static void
MX_GPIO_Init(void);
static void
MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef USE_STM32H7XX_NUCLEO
#define ACC_INIT_SUCCESS() BSP_LED_Toggle(LED_YELLOW);
#define ACC_INIT_FAILURE() BSP_LED_Toggle(LED_RED);
#endif

/// The ADXL355 captures data in LSB/g.
/// To turn this into a properly usable unit we have to take some steps.
/// 1. We calculate the full range of values.
/// To do this we obtain the maximum LSB value: 2 ^ n-1, where n is
/// the amount of bits in the ADC, the ADXL355 has a 20-bit ADC, thus
/// 2^19 is our LSB.
/// Then we obtain our range of g, since we use ±2g,
/// our range of possible values is 2g since data provided by
/// ADXL355 is already in twos complement.
/// We do 2^19 LSB / 2g which gives 262,144 LSB/g.
/// 2. Convert the captured raw data (LSB) to g.
/// This step is done by dividing a raw value with the full range value.
/// e.g. -245000 LSB / 262144 LSB/g = -0.935 g.
/// Please note that the sign comes from the fact that the raw data
/// is in twos complement.
///
/// If you grab this full range value 262,144 LSB/g, which is our "sensitivity value"
/// which we will call w from now, and you do 1/w, you get the sensitivity value
/// found in the data sheet, but but but, now that we know where it comes from, we
/// are better off using the sensitivity value 3.9 𝝁g/LSB,
/// which we will call sm, the m for the manufacturer.
/// data ⋅ sm is better than data / w for efficiency.
/// In fact, I would advice against doing these corrections at the MCU level,
/// and focus on doing them after retrieval or at the broker.

/// Status register bit flags
/// x, y and z axes measurements are ready and can be read.
#define ADXL_FLAG_DATA_RDY    0x01
/// The FIFO is full, this value can be set
#define ADXL_FLAG_FIFO_FULL   0x02
#define ADXL_FLAG_FIFO_OVR    0x04
#define ADXL_FLAG_ACTIVITY    0x08

#define ADXL_WRITE            0x00
#define ADXL_READ             0x01
/// Contains the Analog Devices ID - RO - Returns 0xAD
#define ADXL_REG_DEVID        0x00
/// Status register, uses bit flags to describe different states - RO
#define ADXL_REG_STATUS       0x04
/// Number of data samples in the FIFO buffer - RO
#define ADXL_REG_FIFO_ENTRIES 0x05
/// Data stored in the FIFO lives here - RO
#define ADXL_REG_FIFO_DATA    0x11
/// Stores the number of values the FIFO can hold - RW - Defaults to 0x60 (96)
#define ADXL_REG_FIFO_SAMPLES 0x29
/// Allows toggling different power modes of the ADXL355 - RW
#define ADXL_REG_POWER_CTL    0x2D
/// Used to change some settings on the 355 - RW
#define ADXL_REG_RANGE        0x2C
void
adxl355_read(SPI_HandleTypeDef* spi,
             const uint8_t      reg,
             uint8_t*           data,
             uint16_t           size)
{
    uint8_t reg_op[1] = {(reg << 1) | ADXL_READ};
    HAL_GPIO_WritePin(ADXL_CS_SS_GPIO_Port, ADXL_CS_SS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi, reg_op, size, HAL_MAX_DELAY);
    HAL_SPI_Receive(spi, data, size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ADXL_CS_SS_GPIO_Port, ADXL_CS_SS_Pin, GPIO_PIN_SET);
}
void
adxl355_write(SPI_HandleTypeDef* spi,
              const uint8_t      reg,
              const uint8_t      data,
              uint16_t           size)
{
    uint8_t reg_opd[2];
    reg_opd[0] = (reg << 1) | ADXL_WRITE;
    reg_opd[1] = data;
    HAL_GPIO_WritePin(ADXL_CS_SS_GPIO_Port, ADXL_CS_SS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi, reg_opd, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ADXL_CS_SS_GPIO_Port, ADXL_CS_SS_Pin, GPIO_PIN_SET);
}
void
adxl355_fifo_read(SPI_HandleTypeDef* spi,
                  uint8_t*           x_data,
                  uint8_t*           y_data,
                  uint8_t*           z_data)
{
    uint8_t r = (ADXL_REG_FIFO_DATA << 1) | ADXL_READ;
    HAL_GPIO_WritePin(ADXL_CS_SS_GPIO_Port, ADXL_CS_SS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(spi, &r, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(spi, x_data, 3, HAL_MAX_DELAY);
    HAL_SPI_Receive(spi, y_data, 3, HAL_MAX_DELAY);
    HAL_SPI_Receive(spi, z_data, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ADXL_CS_SS_GPIO_Port, ADXL_CS_SS_Pin, GPIO_PIN_SET);
}
void
adxl355_init()
{
    uint8_t r          = 0;
    uint8_t range_bits = 0xFF;
    uint8_t power_bits = 0xFF;
    adxl355_read(&hspi1, ADXL_REG_DEVID, &r, 1);
    adxl355_read(&hspi1, ADXL_REG_RANGE, &range_bits, 1);
    adxl355_read(&hspi1, ADXL_REG_POWER_CTL, &power_bits, 1);
    if (range_bits != 0x01)
    {
        adxl355_write(&hspi1, ADXL_REG_RANGE, 0x01, 1);
    }
    // The last bit needs to be 0 to stop standby mode
    if ((power_bits & 0x01) != 0x00)
    {
        adxl355_write(&hspi1, ADXL_REG_POWER_CTL, power_bits & 0xFE, 1);
    }
    // TODO: Add the range to init
    if (r == 0xAD)
    {
        ACC_INIT_SUCCESS();
    }
    else
    {
        ACC_INIT_FAILURE();
    }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int
main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MPU Configuration--------------------------------------------------------*/
    MPU_Config();

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_SPI1_Init();
    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* Initialize leds */
    BSP_LED_Init(LED_GREEN);
    BSP_LED_Init(LED_YELLOW);
    BSP_LED_Init(LED_RED);

    /* Initialize USER push-button, will be used to trigger an interrupt each time it's
     * pressed.*/
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

    /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
    BspCOMInit.BaudRate   = 115200;
    BspCOMInit.WordLength = COM_WORDLENGTH_8B;
    BspCOMInit.StopBits   = COM_STOPBITS_1;
    BspCOMInit.Parity     = COM_PARITY_NONE;
    BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
    if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
    {
        Error_Handler();
    }

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    adxl355_init();
    uint8_t stat;
    uint8_t x[3];
    uint8_t y[3];
    uint8_t z[3];
    while (1)
    {
        /* USER CODE END WHILE */
        adxl355_read(&hspi1, ADXL_REG_STATUS, &stat, 1);
        // Fifo is full
        if (stat & ADXL_FLAG_FIFO_FULL)
        {
            adxl355_fifo_read(&hspi1, x, y, z);
        }
        // TODO: order the data read from fifo
        // THINK: We might need to consider DMA
        // TODO: Implement Ethernet interface
        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void
SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Supply configuration update enable
     */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
    {
    }

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = 64;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 4;
    RCC_OscInitStruct.PLL.PLLN            = 12;
    RCC_OscInitStruct.PLL.PLLP            = 1;
    RCC_OscInitStruct.PLL.PLLQ            = 4;
    RCC_OscInitStruct.PLL.PLLR            = 2;
    RCC_OscInitStruct.PLL.PLLRGE          = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL       = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN        = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                  RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void
MX_SPI1_Init(void)
{

    /* USER CODE BEGIN SPI1_Init 0 */

    /* USER CODE END SPI1_Init 0 */

    /* USER CODE BEGIN SPI1_Init 1 */

    /* USER CODE END SPI1_Init 1 */
    /* SPI1 parameter configuration*/
    hspi1.Instance                        = SPI1;
    hspi1.Init.Mode                       = SPI_MODE_MASTER;
    hspi1.Init.Direction                  = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize                   = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity                = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase                   = SPI_PHASE_1EDGE;
    hspi1.Init.NSS                        = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler          = SPI_BAUDRATEPRESCALER_4;
    hspi1.Init.FirstBit                   = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode                     = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation             = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial              = 0x0;
    hspi1.Init.NSSPMode                   = SPI_NSS_PULSE_ENABLE;
    hspi1.Init.NSSPolarity                = SPI_NSS_POLARITY_LOW;
    hspi1.Init.FifoThreshold              = SPI_FIFO_THRESHOLD_01DATA;
    hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi1.Init.MasterSSIdleness           = SPI_MASTER_SS_IDLENESS_00CYCLE;
    hspi1.Init.MasterInterDataIdleness    = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    hspi1.Init.MasterReceiverAutoSusp     = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi1.Init.MasterKeepIOState          = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    hspi1.Init.IOSwap                     = SPI_IO_SWAP_DISABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void
MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */

    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(ADXL_CS_SS_GPIO_Port, ADXL_CS_SS_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin : ADXL_CS_SS_Pin */
    GPIO_InitStruct.Pin   = ADXL_CS_SS_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ADXL_CS_SS_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void
MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    /* Disables the MPU */
    HAL_MPU_Disable();

    /** Initializes and configures the Region and the memory to be protected
     */
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress      = 0x0;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_4GB;
    MPU_InitStruct.SubRegionDisable = 0x87;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    /* Enables the MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void
Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void
assert_failed(uint8_t* file,
              uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
