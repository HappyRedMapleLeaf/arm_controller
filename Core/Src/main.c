/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f401xe.h" // to fix intellisense
#include "imu.h"
#include "math_utils.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_TX_DELAY 50

#define CMD_LEN 64

// structure (0x00 to 0x3F):
// 0x00 - command
// 0x08 - servo0 angle: double
// 0x10 - servo1 angle: double
// 0x18 - servo2 angle: double
// 0x20 - servo3 angle: double
// 0x28 - servo4 angle: double
// 0x30 - servo5 angle: double
// 0x38 - servo6 angle: double
#define CMD_SET_ANGLE 0x01

// structure (0x00 to 0x3F):
// 0x00 - command
// 0x04 - servo0 pwm: uint32_t
// 0x08 - servo1 pwm: uint32_t
// 0x0C - servo2 pwm: uint32_t
// 0x10 - servo3 pwm: uint32_t
// 0x14 - servo4 pwm: uint32_t
// 0x18 - servo5 pwm: uint32_t
// 0x1C - servo6 pwm: uint32_t
// 0x20 - unused
#define CMD_SET_PWM 0x02

#define PRINT_BUF_SIZE 128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for highFreq */
osThreadId_t highFreqHandle;
const osThreadAttr_t highFreq_attributes = {
    .name = "highFreq",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};
/* Definitions for handleUART */
osThreadId_t handleUARTHandle;
const osThreadAttr_t handleUART_attributes = {
    .name = "handleUART",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};
/* USER CODE BEGIN PV */
char print_buf[PRINT_BUF_SIZE];
uint32_t print_len;
volatile uint8_t uart_rx_complete = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void StartHighFreq(void *argument);
void StartHandleUART(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t command[CMD_LEN];

double arm_targets[7] = {0, M_PI_2, M_PI_2, 0, 0, 0, 0};
double arm_angles[7] = {0, M_PI_2, M_PI_2, 0, 0, 0, 0};
double arm_speeds[7] = {0, 0, 0, 0, 0, 0, 0};
uint32_t arm_pwms[7] = {0, 0, 0, 0, 0, 0, 0};
const double tx_rate = 20.0;                // ms
const double max_speed_abs = M_PI_2 / 1000; // rad/ms

Vec3 imu_pos = {0, 0, 0};
Vec3 imu_vel = {0, 0, 0};
Mat3 imu_dir = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

bool txReady = true;

/*
if range is pi, shift is pi/2, then -pi/2 maps to 500, pi/2 maps to 2500
nudge is for small adjustments in case the servo is not centered
*/
const double ranges[7] = {3 * M_PI / 2 * 0.98, M_PI * 1.03, 3 * M_PI / 2 * 0.98, 3 * M_PI / 2, M_PI, M_PI, M_PI};
const double shifts[7] = {0, M_PI * 0.03 / 2, 3 * M_PI / 2 * 0.49 - M_PI / 2, 3 * M_PI / 4, M_PI / 2, M_PI / 2, M_PI / 2};
const double nudges[7] = {0, 0, 65, 0, 50, 50, 0};

void set_angles(double *angles, uint32_t *pwms) {
    for (int i = 0; i < 7; i++)
    {
        pwms[i] = 500 + nudges[i] + (angles[i] + shifts[i]) * 2000.0 / ranges[i];
    }
}

void set_pwms(uint32_t *pwms) {
    // timer 3 PC6-1 PC7-2 PC8-3 PC9-4
    // timer 4 PB6-1 PB8-3 PB9-4
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwms[0]);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwms[1]);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwms[2]);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwms[3]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwms[4]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwms[5]);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwms[6]);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        uart_rx_complete = 1;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // Notify the task that an interrupt has occurred
        vTaskNotifyGiveFromISR(handleUARTHandle, &xHigherPriorityTaskWoken);

        // Force a context switch if xHigherPriorityTaskWoken is now set to pdTRUE
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	txReady = true;
}

int UART_Send(uint8_t *data, uint16_t size)
{
    if (txReady)
    {
        txReady = false;
        return HAL_UART_Transmit_IT(&huart2, data, size);;
    }
    return -1;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

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
    MX_USART2_UART_Init();
    MX_TIM3_Init();
    MX_ADC1_Init();
    MX_I2C3_Init();
    MX_TIM4_Init();
    MX_TIM2_Init();
    /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM2->CR1 = TIM_CR1_CEN;

    IMU_Init();

    // HAL_UART_Transmit(&huart2, (uint8_t *)"WE STARTING! 1\n", 15, UART_TX_DELAY);
    // HAL_UART_Transmit(&huart2, (uint8_t *)"WE STARTING! 2\n", 15, UART_TX_DELAY);
    // HAL_UART_Transmit(&huart2, (uint8_t *)"WE STARTING! 3\n", 15, UART_TX_DELAY);
    uint8_t start_signal[4*8] = {0};
    HAL_UART_Transmit(&huart2, start_signal, 32, UART_TX_DELAY);

    set_angles(arm_angles, arm_pwms);
    set_pwms(arm_pwms);
    /* USER CODE END 2 */

    /* Init scheduler */
    osKernelInitialize();

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of defaultTask */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* creation of highFreq */
    highFreqHandle = osThreadNew(StartHighFreq, NULL, &highFreq_attributes);

    /* creation of handleUART */
    handleUARTHandle = osThreadNew(StartHandleUART, NULL, &handleUART_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */

    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
     */
    sConfig.Channel = ADC_CHANNEL_10;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void) {

    /* USER CODE BEGIN I2C3_Init 0 */

    /* USER CODE END I2C3_Init 0 */

    /* USER CODE BEGIN I2C3_Init 1 */

    /* USER CODE END I2C3_Init 1 */
    hi2c3.Instance = I2C3;
    hi2c3.Init.ClockSpeed = 100000;
    hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c3.Init.OwnAddress1 = 0;
    hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c3.Init.OwnAddress2 = 0;
    hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C3_Init 2 */

    /* USER CODE END I2C3_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 83;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 999999999;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 83;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 19999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 500 + nudges[0] + (arm_angles[0] + shifts[0]) * 2000.0 / ranges[0];
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.Pulse = 500 + nudges[1] + (arm_angles[1] + shifts[1]) * 2000.0 / ranges[1];
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.Pulse = 500 + nudges[2] + (arm_angles[2] + shifts[2]) * 2000.0 / ranges[2];
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.Pulse = 500 + nudges[3] + (arm_angles[3] + shifts[3]) * 2000.0 / ranges[3];
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
    HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM4_Init 1 */

    /* USER CODE END TIM4_Init 1 */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 83;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 19999;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_OC_Init(&htim4) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 500 + nudges[4] + (arm_angles[4] + shifts[4]) * 2000.0 / ranges[4];
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_TIMING;
    sConfigOC.Pulse = 500 + nudges[5] + (arm_angles[5] + shifts[5]) * 2000.0 / ranges[5];
    if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.Pulse = 500 + nudges[6] + (arm_angles[6] + shifts[6]) * 2000.0 / ranges[6];
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */

    /* USER CODE END TIM4_Init 2 */
    HAL_TIM_MspPostInit(&htim4);
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 230400;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : LD2_Pin */
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
    /* USER CODE BEGIN 5 */
    int64_t prev_us = TIM2->CNT; // microseconds
    int64_t prev_us_print = TIM2->CNT; // microseconds
    
    Vec3 average_accel = IMU_Read_Accel_Vec3();
    Vec3 rate_drift = IMU_Read_Gyro_Vec3();
    uint32_t init_cycle = 1;


    // todo: calibration first second
    while (TIM2->CNT - prev_us < 1000000) {
        init_cycle++;
        Vec3 accel = IMU_Read_Accel_Vec3();
        average_accel = vec3_add(average_accel, accel);
        Vec3 rate = IMU_Read_Gyro_Vec3();
        rate_drift = vec3_add(rate_drift, rate);
        osDelay(10);
    }
    average_accel = vec3_scale(average_accel, 1.0 / init_cycle);
    rate_drift = vec3_scale(rate_drift, 1.0 / init_cycle);

    // when stationary, there is a 1g acceleration directly upwards
    // here we calculate the world frame relative to the imu, then invert (transpose) to get the
    // imu frame relative to the world
    Vec3 new_z = vec3_norm(average_accel);
    Vec3 new_x = vec3_norm(vec3_sub(imu_dir.col1, vec3_scale(new_z, dot(new_z, imu_dir.col1))));
    Vec3 new_y = vec3_cross(new_z, new_x);

    imu_dir = (Mat3){new_x, new_y, new_z};
    imu_dir = mat3_transpose(imu_dir);

    /* Infinite loop */
    for (;;) {
        int64_t elapsed_us = TIM2->CNT - prev_us;
        // handle timer value wraparound (once per 1000 seconds)
        if (elapsed_us < 0) {
            elapsed_us += 1000000000;
        }
        double elapsed_s = elapsed_us / 1000000.0;
        prev_us = TIM2->CNT;

        // // we can do imu reads once every 2ms without a problem
        // Vec3 accel = IMU_Read_Accel_Vec3();

        // // remove acceleration due to gravity
        // // 3rd row of imu_dir is the z axis of the world frame relative to the imu
        // accel = vec3_sub(accel, vec3_scale((Vec3) {imu_dir.col1.z, imu_dir.col2.z, imu_dir.col3.z}, G_TO_MM_S_2));
        
        // imu_vel.x += dot(accel, imu_dir.col1) * elapsed_s;
        // imu_vel.y += dot(accel, imu_dir.col2) * elapsed_s;
        // imu_vel.z += dot(accel, imu_dir.col3) * elapsed_s;
        // imu_pos.x += imu_vel.x * elapsed_s;
        // imu_pos.y += imu_vel.y * elapsed_s;
        // imu_pos.z += imu_vel.z * elapsed_s;

        // Vec3 rate = IMU_Read_Gyro_Vec3();
        Vec3 rate = vec3_sub(IMU_Read_Gyro_Vec3(), rate_drift);

        double rate_mag = vec3_mag(rate);
        Vec3 rate_norm = vec3_norm(rate);

        Mat3 rate_mat = {{0, -rate_norm.z, rate_norm.y},
                         {rate_norm.z, 0, -rate_norm.x},
                         {-rate_norm.y, rate_norm.x, 0}};
        Mat3 identity_mat = {{1, 0, 0},
                             {0, 1, 0},
                             {0, 0, 1}};

        double angle = rate_mag * elapsed_s;
        // print_len = snprintf(print_buf, PRINT_BUF_SIZE, "%.3f\n", angle);
        // HAL_UART_Transmit(&huart2, (uint8_t *)print_buf, print_len, 1);

        // Rodrigues' rotation formula
        // Is the relativity correct? The rotation should be relative to the current orientation, not the world frame
        // I think it's correct though
        Mat3 rot_mat = mat3_add(identity_mat, mat3_add(mat3_scale(rate_mat, sin(angle)), mat3_scale(mat3_mul(rate_mat, rate_mat), 1 - cos(angle))));

        imu_dir = mat3_mul(imu_dir, rot_mat);

        // // SERVO SETTING
        // for (uint8_t i = 0; i < 7; i++) {
        //     prev_us = TIM2->CNT;
        //     arm_angles[i] += arm_speeds[i] * elapsed_ms;
        //     if (arm_speeds[i] > 0.0 && arm_angles[i] > arm_targets[i]) {
        //         arm_angles[i] = arm_targets[i];
        //         arm_speeds[i] = 0.0;
        //     }
        //     else if (arm_speeds[i] < 0.0 && arm_angles[i] < arm_targets[i]) {
        //         arm_angles[i] = arm_targets[i];
        //         arm_speeds[i] = 0.0;
        //     }
        // }
        // set_angles(arm_angles, arm_pwms);
        // set_pwms(arm_pwms);

        if (TIM2->CNT - prev_us_print > 1 * 1000000) {
            uint8_t who = IMU_WhoAmI();
            // print_len = snprintf(print_buf, PRINT_BUF_SIZE, "%d\n%6.4f %6.4f %6.4f\n", 
            //     who, imu_dir.col3.x, imu_dir.col3.y, imu_dir.col3.z);
            // HAL_UART_Transmit(&huart2, (uint8_t *)print_buf, print_len, 2);

            uint8_t send[4*8] = {0};
            send[1] = who;
            QuatF q = quatf_from_mat3(imu_dir);
            float data[7] = {imu_pos.x, imu_pos.y, imu_pos.z, q.x, q.y, q.z, q.w};
            memcpy(&send[4], &data, 7*sizeof(float));
            UART_Send(send, 32);
            prev_us_print = TIM2->CNT;
        }
        // osDelay(1);
    }
    /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartHighFreq */
/**
 * @brief Function implementing the UpdateServos thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartHighFreq */
void StartHighFreq(void *argument) {
    /* USER CODE BEGIN StartHighFreq */
    osDelay(2000);
    /* Infinite loop */
    for (;;)
    {
        // send back the imu position
        // 3 doubles for position
        // 3 doubles for +z axis
        // 2 doubles for x and y components of the +x axis
        // full rotation can be extrapolated from this
        // will probably need to update this system later (extend cmd len? use quaternions?)
        // double send[12] = {
        //     imu_pos.x, imu_pos.y, imu_pos.z, 
        //     imu_dir.col1.x, imu_dir.col1.y, imu_dir.col1.z,
        //     imu_dir.col2.x, imu_dir.col2.y, imu_dir.col2.z,
        //     imu_dir.col3.x, imu_dir.col3.y, imu_dir.col3.z
        // };
        // HAL_UART_Transmit(&huart2, (uint8_t *)send, 96, UART_TX_DELAY);
        // print_len = snprintf(print_buf, PRINT_BUF_SIZE, "%f %f %f\n%f %f %f\n%f %f %f\n%f %f %f\n", 
        //     imu_pos.x, imu_pos.y, imu_pos.z, 
        //     imu_dir.col1.x, imu_dir.col1.y, imu_dir.col1.z,
        //     imu_dir.col2.x, imu_dir.col2.y, imu_dir.col2.z,
        //     imu_dir.col3.x, imu_dir.col3.y, imu_dir.col3.z
        // );
        // HAL_UART_Transmit(&huart2, (uint8_t *)print_buf, print_len, UART_TX_DELAY);
        // uint8_t who = IMU_WhoAmI();
        // print_len = snprintf(print_buf, PRINT_BUF_SIZE, "who: %d pos:\n%.2f %.2f %.2f\ndir:\n%.6f %.6f %.6f\n%.6f %.6f %.6f\n%.6f %.6f %.6f\n", 
        //     who, imu_pos.x, imu_pos.y, imu_pos.z, 
        //     imu_dir.col1.x, imu_dir.col2.x, imu_dir.col3.x,
        //     imu_dir.col1.y, imu_dir.col2.y, imu_dir.col3.y,
        //     imu_dir.col1.z, imu_dir.col2.z, imu_dir.col3.z
        // );
        // print_len = snprintf(print_buf, PRINT_BUF_SIZE, "%d\n%9.6f %9.6f %9.6f\n", 
        //     who, imu_dir.col1.x, imu_dir.col1.y, imu_dir.col1.z);
        // HAL_UART_Transmit(&huart2, (uint8_t *)print_buf, print_len, 5);
        osDelay(1000);
    }
    /* USER CODE END StartHighFreq */
}

/* USER CODE BEGIN Header_StartHandleUART */
/**
 * @brief Function implementing the ReceiveUART thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartHandleUART */
void StartHandleUART(void *argument) {
    /* USER CODE BEGIN StartHandleUART */
    HAL_UART_Receive_IT(&huart2, command, CMD_LEN);

    /* Infinite loop */
    // the timing of this loop is determined by the host. We assume it's 20ms (is that scuffed? idk)
    for (;;) {
        // Wait for the UART interrupt to notify this task
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        switch (command[0]) {
        case CMD_SET_ANGLE:
            for (uint8_t i = 0; i < 7; i++) {
                uint8_t offset = 8 + i * 8;
                double angle = *((double *)&command[offset]);
                arm_targets[i] = angle;
                arm_speeds[i] = (arm_targets[i] - arm_angles[i]) / tx_rate;

                // limit arm speed (especially for arm up/down switching movements)
                arm_speeds[i] = (arm_speeds[i] > max_speed_abs) ? max_speed_abs : arm_speeds[i];
                arm_speeds[i] = (arm_speeds[i] < -max_speed_abs) ? -max_speed_abs : arm_speeds[i];
            }
            break;
        case CMD_SET_PWM:
            for (uint8_t i = 0; i < 7; i++) {
                uint8_t offset = 4 + i * 4;
                arm_pwms[i] = *((uint32_t *)&command[offset]);
            }
        default:
            break;
        }

        // Wait for the UART reception to complete before starting a new reception
        while (!uart_rx_complete) {
            taskYIELD();
        }
        uart_rx_complete = 0;

        HAL_UART_Receive_IT(&huart2, command, CMD_LEN);
    }
    /* USER CODE END StartHandleUART */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM1) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
