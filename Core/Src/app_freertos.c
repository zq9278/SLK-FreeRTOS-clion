/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

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
/* USER CODE BEGIN Variables */
uart_data uart_rx_data;
extern int32_t ForceRawOffset;
uint8_t MotorCompareState = 0;
int32_t Force_Q;
int32_t Force_R_Q;

extern int32_t ForceRawSet;
int32_t ForceRawActual;

extern TIM_HandleTypeDef htim7;

extern I2C_HandleTypeDef hi2c2;

extern BQ27441_typedef BQ27441;
extern uint8_t PowerState;
extern uint8_t BQ25895Reg[21];
extern TIM_HandleTypeDef htim16;
/* USER CODE END Variables */
/* Definitions for Motor_Task */
osThreadId_t Motor_TaskHandle;
const osThreadAttr_t Motor_Task_attributes = {
  .name = "Motor_Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for HeatTask */
osThreadId_t HeatTaskHandle;
const osThreadAttr_t HeatTask_attributes = {
  .name = "HeatTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for Uart_ProcessTas */
osThreadId_t Uart_ProcessTasHandle;
const osThreadAttr_t Uart_ProcessTas_attributes = {
  .name = "Uart_ProcessTas",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 130 * 4
};
/* Definitions for Charge_Task */
osThreadId_t Charge_TaskHandle;
const osThreadAttr_t Charge_Task_attributes = {
  .name = "Charge_Task",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for dataQueue */
osMessageQueueId_t dataQueueHandle;
const osMessageQueueAttr_t dataQueue_attributes = {
  .name = "dataQueue"
};
/* Definitions for Temperature_Queue */
osMessageQueueId_t Temperature_QueueHandle;
const osMessageQueueAttr_t Temperature_Queue_attributes = {
  .name = "Temperature_Queue"
};
/* Definitions for Force_Queue */
osMessageQueueId_t Force_QueueHandle;
const osMessageQueueAttr_t Force_Queue_attributes = {
  .name = "Force_Queue"
};
/* Definitions for All_Event */
osEventFlagsId_t All_EventHandle;
const osEventFlagsAttr_t All_Event_attributes = {
  .name = "All_Event"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void AppMotor_Task(void *argument);
void APP_HeatTask(void *argument);
void App_Uart_ProcessTask(void *argument);
void App_Charge_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of dataQueue */
  dataQueueHandle = osMessageQueueNew (5, sizeof(uart_rx_data), &dataQueue_attributes);

  /* creation of Temperature_Queue */
  Temperature_QueueHandle = osMessageQueueNew (10, sizeof(float), &Temperature_Queue_attributes);

  /* creation of Force_Queue */
  Force_QueueHandle = osMessageQueueNew (10, sizeof(int32_t), &Force_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Motor_Task */
  Motor_TaskHandle = osThreadNew(AppMotor_Task, NULL, &Motor_Task_attributes);

  /* creation of HeatTask */
  HeatTaskHandle = osThreadNew(APP_HeatTask, NULL, &HeatTask_attributes);

  /* creation of Uart_ProcessTas */
  Uart_ProcessTasHandle = osThreadNew(App_Uart_ProcessTask, NULL, &Uart_ProcessTas_attributes);

  /* creation of Charge_Task */
  Charge_TaskHandle = osThreadNew(App_Charge_Task, NULL, &Charge_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of All_Event */
  All_EventHandle = osEventFlagsNew(&All_Event_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_AppMotor_Task */
/**
  * @brief  Function implementing the Motor_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_AppMotor_Task */
void AppMotor_Task(void *argument)
{
  /* USER CODE BEGIN AppMotor_Task */
    EventBits_t Motor_Event_Bit;
    uint8_t ReadData[4];
    TMC5130_Init();
    xEventGroupSetBits(All_EventHandle, Reset_Motor_BIT_4); // 设定脉动任务�??????启标志位
    for (;;) {
        Motor_Event_Bit = xEventGroupWaitBits(
                All_EventHandle,                     // Event group handle
                Motor_BIT_2 | Auto_BIT_3 | SW_BIT_1, // flag bits to wait for
                pdFALSE,                             // clear these bits when the function responds
                pdFALSE,                             // Whether to wait for all flag bits
                100                                  // Whether to wait indefinitely
                // portMAX_DELAY    // Whether to wait indefinitely
        );
        vTaskDelay(200 / portTICK_PERIOD_MS);
        if ((((Motor_Event_Bit & Motor_BIT_2) != 0) || ((Motor_Event_Bit & Auto_BIT_3) != 0)) &&
            ((Motor_Event_Bit & SW_BIT_1) == 0)) // 脉动或自动事件发生，按钮事件没发生（电机预模式）
        {
            vTaskDelay(200 / portTICK_PERIOD_MS);

            // printf("电机预启动模�??????");
        }
            // else if (((Motor_Event_Bit & (Motor_BIT_2 | SW_BIT_1)) == (Motor_BIT_2 | SW_BIT_1)) || (Motor_Event_Bit & (Auto_BIT_3 | SW_BIT_1)) == (Auto_BIT_3 | SW_BIT_1)) // 脉动或自动事件发生，按钮事件发生（正式脉动模式启动）
        else if ((((Motor_Event_Bit & Motor_BIT_2) != 0 || (Motor_Event_Bit & Auto_BIT_3)) != 0) &&
                 ((Motor_Event_Bit & SW_BIT_1) != 0)) // 脉动或自动事件发生，按钮事件发生（正式脉动模式启动）
        {
//            vTaskDelay(50/portTICK_PERIOD_MS);
//            ForceRawActual = HX711_Read();
//            if ((ForceRawActual >= 500000) && (MotorCompareState == 0))
//            {
//                //HAL_TIM_Base_Start_IT(&htim7);
//                MotorCompareState = 1;
//            }
//            if((ForceRawActual <= 300) && (MotorCompareState != 0)){
//                MotorCompareState = 0;
//                TMC5130_Write(0xa7, 0x8000); // 暴力�??????回后前进的�?�度
//                TMC5130_Write(0xa0, 1);// 暴力�??????回后运动的方�??????
//            }
//            switch (MotorCompareState)
//            {
//                case 1:
//                    MotorCompare(ForceRawOffset + ForceRawSet, ForceRawActual);
//                    vTaskDelay(10/portTICK_PERIOD_MS);
//                    break;
//                case 2:
//                    TMC5130_Write(0xa7, 0x8000); // 治疗阶段的回�??????速度
//                    TMC5130_Write(0xa0, 2);
//                    vTaskDelay(10/portTICK_PERIOD_MS);
//                    break;
//                default:
//                    break;
//            }
//            Force_Q = (ForceRawActual - ForceRawOffset < 0) ? 0 : (ForceRawActual - ForceRawOffset);
//            Limit(Force_Q,0,ForceRawSet+82617);
//            xQueueSend(Force_QueueHandle, &Force_Q, 0);
            if (xQueueReceive(Force_QueueHandle, &Force_R_Q, 10)) // 阻塞接受队列消息
            {
                if ((Force_R_Q >= 500000) && (MotorCompareState == 0)) {
                    //HAL_TIM_Base_Start_IT(&htim7);
                    MotorCompareState = 1;
                }
                if ((Force_R_Q <= 300) && (MotorCompareState != 0)) {
                    MotorCompareState = 0;
                    TMC5130_Write(0xa7, 0x8000); // 暴力�??????回后前进的�?�度
                    TMC5130_Write(0xa0, 1);// 暴力�??????回后运动的方�??????
                }
                switch (MotorCompareState) {
                    case 1:
                        MotorCompare(ForceRawOffset + ForceRawSet, Force_R_Q);
                        vTaskDelay(10 / portTICK_PERIOD_MS);
                        break;
                    case 2:
                        TMC5130_Write(0xa7, 0x8000); // 治疗阶段的回�??????速度
                        TMC5130_Write(0xa0, 2);
                        vTaskDelay(10 / portTICK_PERIOD_MS);
                        break;
                    default:
                        break;
                }

            }


        } else if ((Motor_Event_Bit & Reset_Motor_BIT_4) != 0) {
            HAL_GPIO_WritePin(TMC_ENN_GPIO_Port, TMC_ENN_Pin, GPIO_PIN_RESET); // 使能tmc电机引脚
            TMC5130_Write(0xa7, 0x10000);
            TMC5130_Write(0xa0, 2);
            TMC5130_Read(0x04, ReadData); // 继续读位置寄存器
            if ((ReadData[3] & 0x02) != 0x02) {
                HAL_GPIO_WritePin(TMC_ENN_GPIO_Port, TMC_ENN_Pin, GPIO_PIN_SET); // 使能tmc电机引脚
                xEventGroupClearBits(All_EventHandle, Reset_Motor_BIT_4);
            }
        }
    }

  /* USER CODE END AppMotor_Task */
}

/* USER CODE BEGIN Header_APP_HeatTask */
/**
* @brief Function implementing the HeatTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_APP_HeatTask */
void APP_HeatTask(void *argument)
{
  /* USER CODE BEGIN APP_HeatTask */
    /* Infinite loop */
    EventBits_t Heat_Event_Bit;
    // HeatPIDInit();
    for (;;) {
        Heat_Event_Bit = xEventGroupWaitBits(
                All_EventHandle,                    // Event group handle
                Heat_BIT_0 | Auto_BIT_3 | SW_BIT_1, // flag bits to wait for
                pdFALSE,                            // clear these bits when the function responds
                pdFALSE,                            // Whether to wait for all flag bits
                100                                 // Whether to wait indefinitely
                // portMAX_DELAY                      // Whether to wait indefinitely
        );
        if ((((Heat_Event_Bit & Heat_BIT_0) != 0) || ((Heat_Event_Bit & Auto_BIT_3) != 0)) &&
            ((Heat_Event_Bit & SW_BIT_1) == 0)) {
            // printf("预加热模式\n");

            start_Heat(Temperature_QueueHandle);
            //HAL_I2C_DeInit(&hi2c2);
            vTaskDelay(200 / portTICK_PERIOD_MS);
        } else if (((Heat_Event_Bit & (Heat_BIT_0 | SW_BIT_1)) == (Heat_BIT_0 | SW_BIT_1)) ||
                   ((Heat_Event_Bit & (Auto_BIT_3 | SW_BIT_1)) ==
                    (Auto_BIT_3 | SW_BIT_1))) // 加热或�?�自动事件发生，按钮事件发生（正式加热模式�?
        {
            // printf("正式加热模式");
            start_Heat(Temperature_QueueHandle);
            vTaskDelay(200 / portTICK_PERIOD_MS);
        } else if ((((Heat_Event_Bit & Heat_BIT_0) == 0) || ((Heat_Event_Bit & Auto_BIT_3) == 0)) &&
                   ((Heat_Event_Bit & SW_BIT_1) == 0)) {
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
    }

  /* USER CODE END APP_HeatTask */
}

/* USER CODE BEGIN Header_App_Uart_ProcessTask */
/**
* @brief Function implementing the Uart_ProcessTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_App_Uart_ProcessTask */
void App_Uart_ProcessTask(void *argument)
{
  /* USER CODE BEGIN App_Uart_ProcessTask */
    /* Infinite loop */

    // HAL_TIM_Base_Start(&htim16);
    // HAL_TIM_PWM_Start(&htim16, LED_TIM_CHANNEL);
    __HAL_TIM_ENABLE_DMA(&htim16, TIM_DMA_CC1);
    // sendColor(0, 0, 25);
    // UCS1903Show();
    EventBits_t Data_Event_Bit;

    for (;;) {

        Data_Event_Bit = xEventGroupWaitBits(
                All_EventHandle,                                  // Event group handle
                Heat_BIT_0 | Auto_BIT_3 | Motor_BIT_2 | SW_BIT_1, // flag bits to wait for
                pdFALSE,                                          // clear these bits when the function responds
                pdFALSE,                                          // Whether to wait for all flag bits
                100                                               // Whether to wait indefinitely
                // portMAX_DELAY    // Whether to wait indefinitely
        );
        //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        if ((Data_Event_Bit & Heat_BIT_0) != 0) { // printf("打开热敷数据");
            ProcessTemperatureData(0x0302);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        if (((Data_Event_Bit & Motor_BIT_2) != 0) && ((Data_Event_Bit & SW_BIT_1) != 0)) { // printf("打开压力数据");
            ProcessForceData(0x0702);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        if ((Data_Event_Bit & Auto_BIT_3) != 0) { // printf("打开自动数据");
            ProcessTemperatureData(0x0C03);
            vTaskDelay(10 / portTICK_PERIOD_MS);
            if ((Data_Event_Bit & SW_BIT_1) != 0) {
                ProcessForceData(0x0C04);
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        }
        if (xQueueReceive(dataQueueHandle, &uart_rx_data, 100)) // 阻塞接受队列消息
        {
            // HAL_UART_Transmit(&huart1, (uint8_t *)&(uart_rx_data.buffer), uart_rx_data.length, 0xFFFF);
            processData((PCTRL_MSG) uart_rx_data.buffer); // 处理接收到的数据
        }
    }
  /* USER CODE END App_Uart_ProcessTask */
}

/* USER CODE BEGIN Header_App_Charge_Task */
/**
* @brief Function implementing the Charge_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_App_Charge_Task */
void App_Charge_Task(void *argument)
{
  /* USER CODE BEGIN App_Charge_Task */
    BQ25895_Init(); // 充电芯片初始�??????
    BQ27441_Init(); // 电量芯片初始�??????
    AT24CXX_Init(); // 非易失�?�存储芯片初始化


    //EventBits_t Force_Event_Bit;
    HX711_Init();
    int32_t tem[3];
    int32_t force_temp1;
    /* Infinite loop */
    for (;;) {
        BQ25895_MultiRead(BQ25895Reg); // 读取充电状�??
        PowerStateUpdate();
        BQ27441_MultiRead(&BQ27441);              // 获取电量计数�?????
        ScreenUpdateSOC(BQ27441.SOC, PowerState); // 电量上传
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
        vTaskDelay(200 / portTICK_PERIOD_MS);
//        Force_Event_Bit = xEventGroupWaitBits(
//                All_EventHandle,                     // Event group handle
//                Motor_BIT_2 | Auto_BIT_3 | SW_BIT_1, // flag bits to wait for
//                pdFALSE,                             // clear these bits when the function responds
//                pdFALSE,                             // Whether to wait for all flag bits
//                100                                  // Whether to wait indefinitely
//                // portMAX_DELAY    // Whether to wait indefinitely
//        );
        //vTaskDelay(200 / portTICK_PERIOD_MS);
        //if ((Force_Event_Bit & SW_BIT_1) != 0)// 脉动或自动事件发生，按钮事件发生（正式脉动模式启动）
        //|| (Force_Event_Bit & Auto_BIT_3)) != 0) ||((Force_Event_Bit & SW_BIT_1) != 0))((Force_Event_Bit & Motor_BIT_2) != 0 )&&
        {
            vTaskDelay(50 / portTICK_PERIOD_MS);
            for (int i = 0; i < 3; ++i) {
                tem[i] = HX711_Read();
            }
            ForceRawActual = processFilter_force(tem);
            force_temp1 = ForceRawActual - ForceRawOffset;//当前读取值减去初始偏执�?�，得到实际值（去皮�????
            Force_Q = (force_temp1 < 0) ? 0 : (force_temp1);//实际的�?�可能存在负�????
            Limit(Force_Q, 0, ForceRawSet + ForceRawOffset);//去除超过屏幕设定�????
            xQueueSend(Force_QueueHandle, &Force_Q, 0);
        }
    }
  /* USER CODE END App_Charge_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

