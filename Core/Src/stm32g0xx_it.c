/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32g0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern QueueHandle_t dataQueueHandle;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern EventGroupHandle_t All_EventHandle;
extern osMessageQueueId_t Temperature_QueueHandle;
extern QueueHandle_t Force_QueueHandle;
static TickType_t xLastWakeTime = 0;
const char *str = "interrupt";

#define RX_BUFFER_SIZE 100
uint8_t rx_buffer[RX_BUFFER_SIZE];
uart_data uart_RX_data;
uint8_t rx_index = 0;
uint8_t last_byte = 0;
int frame_started = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_i2c2_rx;
extern DMA_HandleTypeDef hdma_i2c2_tx;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_tim16_ch1;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim16;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/******************************************************************************/
/* STM32G0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles Flash global interrupt.
  */
void FLASH_IRQHandler(void)
{
  /* USER CODE BEGIN FLASH_IRQn 0 */

  /* USER CODE END FLASH_IRQn 0 */
  HAL_FLASH_IRQHandler();
  /* USER CODE BEGIN FLASH_IRQn 1 */

  /* USER CODE END FLASH_IRQn 1 */
}

/**
  * @brief This function handles RCC global interrupt.
  */
void RCC_IRQHandler(void)
{
  /* USER CODE BEGIN RCC_IRQn 0 */

  /* USER CODE END RCC_IRQn 0 */
  /* USER CODE BEGIN RCC_IRQn 1 */

  /* USER CODE END RCC_IRQn 1 */
}

/**
  * @brief This function handles EXTI line 0 and line 1 interrupts.
  */
void EXTI0_1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_1_IRQn 0 */

  /* USER CODE END EXTI0_1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(CHG_DSEL_Pin);
  /* USER CODE BEGIN EXTI0_1_IRQn 1 */

  /* USER CODE END EXTI0_1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line 2 and line 3 interrupts.
  */
void EXTI2_3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_3_IRQn 0 */

  /* USER CODE END EXTI2_3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(SW_CNT_Pin);
  /* USER CODE BEGIN EXTI2_3_IRQn 1 */

  /* USER CODE END EXTI2_3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line 4 to 15 interrupts.
  */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */

  /* USER CODE END EXTI4_15_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(CHG_STAT_Pin);
  HAL_GPIO_EXTI_IRQHandler(CHG_INT_Pin);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel 1 interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c1_rx);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel 2 and channel 3 interrupts.
  */
void DMA1_Channel2_3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 0 */

  /* USER CODE END DMA1_Channel2_3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c1_tx);
  HAL_DMA_IRQHandler(&hdma_i2c2_rx);
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 1 */

  /* USER CODE END DMA1_Channel2_3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel 4, channel 5, channel 6, channel 7 and DMAMUX1 interrupts.
  */
void DMA1_Ch4_7_DMAMUX1_OVR_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Ch4_7_DMAMUX1_OVR_IRQn 0 */

  /* USER CODE END DMA1_Ch4_7_DMAMUX1_OVR_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c2_tx);
  HAL_DMA_IRQHandler(&hdma_adc1);
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  HAL_DMA_IRQHandler(&hdma_tim16_ch1);
  /* USER CODE BEGIN DMA1_Ch4_7_DMAMUX1_OVR_IRQn 1 */

  /* USER CODE END DMA1_Ch4_7_DMAMUX1_OVR_IRQn 1 */
}

/**
  * @brief This function handles ADC1 interrupt.
  */
void ADC1_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_IRQn 0 */

  /* USER CODE END ADC1_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_IRQn 1 */

  /* USER CODE END ADC1_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt.
  */
void TIM6_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_IRQn 0 */

  /* USER CODE END TIM6_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_IRQn 1 */

  /* USER CODE END TIM6_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles TIM14 global interrupt.
  */
void TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */

  /* USER CODE END TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM14_IRQn 1 */

  /* USER CODE END TIM14_IRQn 1 */
}

/**
  * @brief This function handles TIM16 global interrupt.
  */
void TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM16_IRQn 0 */

  /* USER CODE END TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM16_IRQn 1 */

  /* USER CODE END TIM16_IRQn 1 */
}

/**
  * @brief This function handles I2C1 event global interrupt / I2C1 wake-up interrupt through EXTI line 23.
  */
void I2C1_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_IRQn 0 */

  /* USER CODE END I2C1_IRQn 0 */
  if (hi2c1.Instance->ISR & (I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_OVR)) {
    HAL_I2C_ER_IRQHandler(&hi2c1);
  } else {
    HAL_I2C_EV_IRQHandler(&hi2c1);
  }
  /* USER CODE BEGIN I2C1_IRQn 1 */

  /* USER CODE END I2C1_IRQn 1 */
}

/**
  * @brief This function handles I2C2 global interrupt.
  */
void I2C2_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_IRQn 0 */

  /* USER CODE END I2C2_IRQn 0 */
  if (hi2c2.Instance->ISR & (I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_OVR)) {
    HAL_I2C_ER_IRQHandler(&hi2c2);
  } else {
    HAL_I2C_EV_IRQHandler(&hi2c2);
  }
  /* USER CODE BEGIN I2C2_IRQn 1 */

  /* USER CODE END I2C2_IRQn 1 */
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */

  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
// 使用DMA的方式在缓冲区和寄存器之间传递�??
    //  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
    //  {
    //    __HAL_UART_CLEAR_IDLEFLAG(&huart1);                                              // 清除空闲中断标志
    //    HAL_UART_DMAStop(&huart1);                                                       // 停止 DMA 传输
    //    size_t data_length = sizeof(rx_buffer) - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx); // 算出接本帧数据长�???????????????
    //    xQueueSendFromISR(dataQueueHandle, &rx_buffer, NULL);                            // 在中断中向队列添加数�???????????????
    //     HAL_UART_Transmit(&huart1, (uint8_t *)&rx_buffer,data_length, 0xFFFF);//验证打印数据
    //    HAL_UART_Receive_DMA(&huart1, rx_buffer, data_length); // 重新�???????????????启DMA
    //  }

    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))
    {
        uint8_t received_data = (uint8_t)(huart1.Instance->RDR & 0xFF); // 取出缓冲区的字符
        if (last_byte == 0x5A && received_data == 0xA5)
        {
            frame_started = 1; // �??测到帧头时设置的--标志�??
            uart_RX_data.buffer[0] = 0x5A;
            rx_index = 1; // 重置索引
        }
        last_byte = received_data; // 更新上一个字�??
        if (frame_started)
        {
            uart_RX_data.buffer[rx_index++] = received_data;
        }
    }
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        // HAL_UART_Transmit(&huart1, (uint8_t *)&rx_buffer, (size_t)rx_index, 0xFFFF); // 验证打印数据
        uart_RX_data.length = rx_index;
        xQueueSend(dataQueueHandle, &uart_RX_data, 0); // 在中断中向队列添加数�???????????????
        // rx_index = 0;                                            //
        frame_started = 0; // 标记帧的结束
    }
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    //BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    //EventBits_t xBits = xEventGroupGetBitsFromISR(All_EventHandle);
    // if (((xBits & Heat_BIT_0) != 0) || ((xBits & Motor_BIT_2) != 0) || ((xBits & Auto_BIT_3) != 0)) // 电机或�?�加热膜有一个事件发生了，都可以进入�?????????????关检测状�?????????????
    // {
    // if (HAL_GPIO_ReadPin(SW_CNT_GPIO_Port, SW_CNT_Pin) == 0) // 物理�?????????????关是否被按下
    // {
    //   // 设置事件组的标志�?????????????
    //   if (((xBits & SW_BIT_1) == 0) && (((xBits & Heat_BIT_0) != 0) || ((xBits & Motor_BIT_2) != 0) || ((xBits & Auto_BIT_3) != 0))) // �?????????????关事件是否发�?????????????
    //   {
    //     // 如果SW_BIT_1当前是清除的，那么设置它
    //     // xEventGroupSetBitsFromISR(All_EventHandle, SW_BIT_1, pdFALSE);
    //     xEventGroupSetBitsFromISR(All_EventHandle, SW_BIT_1, &xHigherPriorityTaskWoken); // 设置�?????????????关事件发�?????????????

    //     if ((xBits & Motor_BIT_2) != 0)
    //     {
    //       ScreenTimerStart(0x07);
    //       HAL_GPIO_WritePin(TMC_ENN_GPIO_Port, TMC_ENN_Pin, GPIO_PIN_RESET); // 使能tmc电机引脚
    //       TMC5130_Write(0xa7, 0x8000);
    //       TMC5130_Write(0xa0, 1);
    //     }
    //     if (((xBits & Heat_BIT_0) != 0))
    //     {
    //       ScreenTimerStart(0x03);
    //     }
    //     if (((xBits & Auto_BIT_3) != 0))
    //     {
    //       HAL_GPIO_WritePin(TMC_ENN_GPIO_Port, TMC_ENN_Pin, GPIO_PIN_RESET); // 使能tmc电机引脚
    //       TMC5130_Write(0xa7, 0x8000);
    //       TMC5130_Write(0xa0, 1);
    //       ScreenTimerStart(0x0C);
    //     }
    //   }
    //   else
    //   {
    //     // 如果SW_BIT_1当前是设置的，那么清除它

    //     xEventGroupClearBits(All_EventHandle, SW_BIT_1);
    //     if ((xBits & Heat_BIT_0) != 0)
    //     {
    //       ScreenWorkModeQuit(0x03);
    //       xEventGroupClearBits(All_EventHandle, Heat_BIT_0); // 清除加热事件
    //       xQueueReset(Temperature_QueueHandle);
    //     }
    //     if ((xBits & Motor_BIT_2) != 0)
    //     {
    //       ScreenWorkModeQuit(0x07);
    //       xEventGroupClearBits(All_EventHandle, Motor_BIT_2);
    //       MotorChecking();
    //       xQueueReset(Force_QueueHandle);
    //     }
    //     if ((xBits & Auto_BIT_3) != 0)
    //     {
    //       ScreenWorkModeQuit(0x0C);
    //       xEventGroupClearBits(All_EventHandle, Auto_BIT_3);
    //       MotorChecking();
    //       xQueueReset(Temperature_QueueHandle);
    //       xQueueReset(Force_QueueHandle);
    //     }
    //   }
    // }


    TickType_t xCurrentTime = xTaskGetTickCountFromISR();
    EventBits_t xBits = xEventGroupGetBitsFromISR(All_EventHandle);

    //if (GPIO_Pin == SW_CNT_Pin) // 假设 SW_CNT_Pin 是你的按键对应的 GPIO pin
    if (HAL_GPIO_ReadPin(SW_CNT_GPIO_Port, SW_CNT_Pin) == 0) // 假设 SW_CNT_Pin 是你的按键对应的 GPIO pin
    {
        // �?????查两次按键事件之间的时间�?????
        if (xCurrentTime - xLastWakeTime >= pdMS_TO_TICKS(50))
        {
            // 更新上一次按键时�?????
            xLastWakeTime = xCurrentTime;

            // 设置事件组的标志�?????????????
            if (((xBits & SW_BIT_1) == 0) && (((xBits & Heat_BIT_0) != 0) || ((xBits & Motor_BIT_2) != 0) || ((xBits & Auto_BIT_3) != 0)))
            {
                // 如果SW_BIT_1当前是清除的，那么设置它
                // xEventGroupSetBitsFromISR(All_EventHandle, SW_BIT_1, pdFALSE);
                //xEventGroupSetBitsFromISR(All_EventHandle, SW_BIT_1, &xHigherPriorityTaskWoken);
                xEventGroupSetBits(All_EventHandle, SW_BIT_1); // 设置�?关事件发�?

                if ((xBits & Motor_BIT_2) != 0)
                {
                    ScreenTimerStart(0x07);
                    HAL_GPIO_WritePin(TMC_ENN_GPIO_Port, TMC_ENN_Pin, GPIO_PIN_RESET); // 使能tmc电机引脚
                    TMC5130_Write(0xa7, 0x8000);
                    TMC5130_Write(0xa0, 1);
                }
                if (((xBits & Heat_BIT_0) != 0))
                {
                    ScreenTimerStart(0x03);
                    //HeatPIDInit(42.5);
                    HeatPIDInit(41.0);
                }
                if (((xBits & Auto_BIT_3) != 0))
                {
                    //HeatPIDInit(42.5);
                    HeatPIDInit(41.0);
                    HAL_GPIO_WritePin(TMC_ENN_GPIO_Port, TMC_ENN_Pin, GPIO_PIN_RESET); // 使能tmc电机引脚
                    TMC5130_Write(0xa7, 0x8000);
                    TMC5130_Write(0xa0, 1);
                    ScreenTimerStart(0x0C);
                }
            }
            if (((xBits & SW_BIT_1) != 0) && (((xBits & Heat_BIT_0) != 0) || ((xBits & Motor_BIT_2) != 0) || ((xBits & Auto_BIT_3) != 0))) // �?关事件是否发�?
            {
                // 如果SW_BIT_1当前是设置的，那么清除它
                xEventGroupClearBits(All_EventHandle, SW_BIT_1);

                if ((xBits & Heat_BIT_0) != 0)
                {
                    ScreenWorkModeQuit(0x03);
                    xEventGroupClearBits(All_EventHandle, Heat_BIT_0); // 清除加热事件
                    HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1);		 // disable pwm for heating film
                    //xQueueReset(Temperature_QueueHandle);
                }
                if ((xBits & Motor_BIT_2) != 0)
                {xEventGroupClearBits(All_EventHandle, Motor_BIT_2);
                    xEventGroupSetBits(All_EventHandle, Reset_Motor_BIT_4);
                    ScreenWorkModeQuit(0x07);
                    HAL_TIM_Base_Stop_IT(&htim7);
                    //xQueueReset(Force_QueueHandle);
                }
                if ((xBits & Auto_BIT_3) != 0)
                {
                    ScreenWorkModeQuit(0x0C);
                    xEventGroupClearBits(All_EventHandle, Auto_BIT_3);
                    HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1);		 // disable pwm for heating film
                    xEventGroupSetBits(All_EventHandle, Reset_Motor_BIT_4);

                    //xQueueReset(Temperature_QueueHandle);
                    //xQueueReset(Force_QueueHandle);
                }

            }
        }
    }
}
/* USER CODE END 1 */
