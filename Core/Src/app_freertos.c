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

extern int32_t ForceRawSet;
int32_t ForceRawActual;

extern TIM_HandleTypeDef htim7;

extern I2C_HandleTypeDef hi2c2;

extern BQ27441_typedef BQ27441;
extern uint8_t PowerState;
extern uint8_t BQ25895Reg[21];



#define BREATHE_STEPS 256 // ������256����������

volatile int breatheIndex = 0; // ��ǰ����Ч��������
 uint32_t breatheLUT[BREATHE_STEPS] = {
        0, 0, 0, 0, 1, 1, 1, 2, 2, 3, 4, 5, 6, 6, 8, 9, 10, 11, 12, 14, 15, 17, 18, 20, 22, 23, 25, 27, 29, 31, 33, 35, 38, 40, 42, 45, 47, 49, 52, 54, 57, 60, 62, 65, 68, 71, 73, 76, 79, 82, 85, 88, 91, 94, 97, 100, 103, 106, 109, 113, 116, 119, 122, 125, 128, 131, 135, 138, 141, 144, 147, 150, 153, 156, 159, 162, 165, 168, 171, 174, 177, 180, 183, 186, 189, 191, 194, 197, 199, 202, 204, 207, 209, 212, 214, 216, 218, 221, 223, 225, 227, 229, 231, 232, 234, 236, 238, 239, 241, 242, 243, 245, 246, 247, 248, 249, 250, 251, 252, 252, 253, 253, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 253, 253, 252, 252, 251, 250, 249, 248, 247, 246, 245, 243, 242, 241, 239, 238, 236, 234, 232, 231, 229, 227, 225, 223, 221, 218, 216, 214, 212, 209, 207, 204, 202, 199, 197, 194, 191, 189, 186, 183, 180, 177, 174, 171, 168, 165, 162, 159, 156, 153, 150, 147, 144, 141, 138, 135, 131, 128, 125, 122, 119, 116, 113, 109, 106, 103, 100, 97, 94, 91, 88, 85, 82, 79, 76, 73, 71, 68, 65, 62, 60, 57, 54, 52, 49, 47, 45, 42, 40, 38, 35, 33, 31, 29, 27, 25, 23, 22, 20, 18, 17, 15, 14, 12, 11, 10, 9, 8, 6, 6, 5, 4, 3, 2, 2, 1, 1, 1, 0, 0, 0, 0
};

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
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
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
/* Definitions for breatheTimer */
osTimerId_t breatheTimerHandle;
const osTimerAttr_t breatheTimer_attributes = {
  .name = "breatheTimer"
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
void BreatheTimerCallback(void *argument);

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

  /* Create the timer(s) */
  /* creation of breatheTimer */
  breatheTimerHandle = osTimerNew(BreatheTimerCallback, osTimerPeriodic, NULL, &breatheTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of dataQueue */
  dataQueueHandle = osMessageQueueNew (5, sizeof(uart_rx_data), &dataQueue_attributes);

  /* creation of Temperature_Queue */
  Temperature_QueueHandle = osMessageQueueNew (10, sizeof(float), &Temperature_Queue_attributes);

  /* creation of Force_Queue */
  Force_QueueHandle = osMessageQueueNew (10, sizeof(uint32_t), &Force_Queue_attributes);

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
    int count = 0; // ��ʼ��������
    EventBits_t Motor_Event_Bit;
    uint8_t Motor_Reset_Position_ReadData[4];
    TMC5130_Init();
    HX711_Init();

    vTaskDelay(200);
    xEventGroupSetBits(All_EventHandle, Reset_Motor_BIT_4);//���õ����λ�¼�
    uint32_t Force_Raw_Data[3];
    for (;;) {
        Motor_Event_Bit = xEventGroupWaitBits(
                All_EventHandle,                     // Event group handle
                Motor_BIT_2 | Auto_BIT_3 | SW_BIT_1 | Reset_Motor_BIT_4, // flag bits to wait for
                pdFALSE,                             // clear these bits when the function responds
                pdFALSE,                             // Whether to wait for all flag bits
                100                                  // Whether to wait indefinitely
                // portMAX_DELAY    // Whether to wait indefinitely
        );

        if ((((Motor_Event_Bit & Motor_BIT_2) != 0) || ((Motor_Event_Bit & Auto_BIT_3) != 0)) &&
            ((Motor_Event_Bit & SW_BIT_1) == 0)) // �������Զ��¼���������ť�¼�û���������Ԥģʽ��
        {
            vTaskDelay(200);
            // printf("���Ԥģ");
        }
        else if ((((Motor_Event_Bit & Motor_BIT_2) != 0 || (Motor_Event_Bit & Auto_BIT_3)) != 0) &&
                 ((Motor_Event_Bit & SW_BIT_1) != 0)) // �������Զ��¼���������ť�¼���������ʽ����ģʽ������
        {
            //vTaskDelay(1000);

            for (int i = 0; i < 3; ++i) {
                Force_Raw_Data[i] = HX711_Read();
            }
            ForceRawActual = processFilter_force(Force_Raw_Data,5);//������ȶ�ʱ����ѹ������������ֵ���й���
            if ((ForceRawActual >= 500000) && (MotorCompareState == 0)) {//���ѹ���������Ƿ�ﵽ500000�����״̬�Ƿ����
                HAL_TIM_Base_Start_IT(&htim7);//������ʱ�����е��״̬�л�
                MotorCompareState = 1;//�����Ϊǰ��״̬
            } else if ((ForceRawActual <= 300) && (MotorCompareState != 0)) {//ѹ��������ѹ��С��300�ҵ��״̬������
                TMC5130_Write(0xa7, 0x8000); // ���ƽ׶ε�ǰ���ٶ�
                TMC5130_Write(0xa0, 1);
                MotorCompareState = 0;//������ڷ����ƽ׶�
            }
            switch (MotorCompareState) {
                case 1:
                    MotorCompare(ForceRawOffset + ForceRawSet, ForceRawActual);
                    vTaskDelay(10);
                    break;
                case 2:
                    TMC5130_Write(0xa7, 0x6000); // ���ƽ׶εĻ��˵��ٶ�
                    TMC5130_Write(0xa0, 2);
                    vTaskDelay(10);
                    break;
                default:
                    break;
            }
            Force_Q = (ForceRawActual - ForceRawOffset < 0) ? 0 : (ForceRawActual - ForceRawOffset);//ȥƤ+ȥ����
            Limit(Force_Q, 0, ForceRawSet + 82617);//���ֵ��Сֵ���޷�
            xQueueSend(Force_QueueHandle, &Force_Q, 0);
        } else if ((Motor_Event_Bit & Reset_Motor_BIT_4) != 0) {//�����λ�¼�������
            HAL_GPIO_WritePin(TMC_ENN_GPIO_Port, TMC_ENN_Pin, GPIO_PIN_RESET); // ʹ��tmc�������
            TMC5130_Write(0xa7, 0x10000);
            TMC5130_Write(0xa0, 2);
            TMC5130_Read(0x04, Motor_Reset_Position_ReadData); // ������ λ�üĴ���
            if ((Motor_Reset_Position_ReadData[3] & 0x02) != 0x02) {//���λ�üĴ�����ȡ�ɹ�����������λ�¼�
                HAL_GPIO_WritePin(TMC_ENN_GPIO_Port, TMC_ENN_Pin, GPIO_PIN_SET); // ʹ��tmc�������
                xEventGroupClearBits(All_EventHandle, Reset_Motor_BIT_4);
                HAL_TIM_Base_Stop_IT(&htim7);
            }
        }
        // TMC5130_Write(0xa0, 1); // ����tmc���������ǰ
        // vTaskDelay(1000);
        // TMC5130_Write(0xa0, 2);
        // vTaskDelay(1000);
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
    //HeatPIDInit();
    for (;;) {
        Heat_Event_Bit = xEventGroupWaitBits(
                All_EventHandle,                    // Event group handle
                Heat_BIT_0 | Auto_BIT_3 | SW_BIT_1, // flag bits to wait for
                pdFALSE,                            // clear these bits when the function responds
                pdFALSE,                            // Whether to wait for all flag bits
                100                                 // Whether to wait indefinitely
                // portMAX_DELAY                      // Whether to wait indefinitely
        );
        // if ((((Heat_Event_Bit & Heat_BIT_0) || ((Heat_Event_Bit & Auto_BIT_3) != 0)) != 0) && ((Heat_Event_Bit & SW_BIT_1) == 0)) // ���Ȼ��Զ��¼���������ť�¼�û������Ԥ��ģʽ��
        if ((((Heat_Event_Bit & Heat_BIT_0) != 0) || ((Heat_Event_Bit & Auto_BIT_3) != 0)) &&
            ((Heat_Event_Bit & SW_BIT_1) == 0)) {
            // printf("Ԥ����ģʽ\n");
            start_Heat(Temperature_QueueHandle);
            vTaskDelay(20);
        } else if (((Heat_Event_Bit & (Heat_BIT_0 | SW_BIT_1)) == (Heat_BIT_0 | SW_BIT_1)) ||
                   ((Heat_Event_Bit & (Auto_BIT_3 | SW_BIT_1)) ==
                    (Auto_BIT_3 | SW_BIT_1))) // ���Ȼ�???�Զ��¼���������ť�¼���������ʽ����ģʽ��
        {
            // printf("��ʽ����ģʽ");
            start_Heat(Temperature_QueueHandle);
            vTaskDelay(200);
        }
        vTaskDelay(20);
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
        //vTaskDelay(5);

        if ((Data_Event_Bit & Heat_BIT_0) != 0) { // printf("���ȷ�����");
            ProcessTemperatureData(0x0302);
            //vTaskDelay(10);
        }
        if (((Data_Event_Bit & Motor_BIT_2) != 0) && ((Data_Event_Bit & SW_BIT_1) != 0)) { // printf("��ѹ������");
            ProcessForceData(0x0702);
            //vTaskDelay(10);
        }
        if ((Data_Event_Bit & Auto_BIT_3) != 0) { // printf("���Զ�����");
            ProcessTemperatureData(0x0C03);
            //vTaskDelay(10);
            if ((Data_Event_Bit & SW_BIT_1) != 0) {
                ProcessForceData(0x0C04);
                //vTaskDelay(10);
            }
        }
        if (xQueueReceive(dataQueueHandle, &uart_rx_data, 1)) // �������ܶ�����Ϣ
        {
            // HAL_UART_Transmit(&huart1, (uint8_t *)&(uart_rx_data.buffer), uart_rx_data.length, 0xFFFF);
            processData((PCTRL_MSG) uart_rx_data.buffer); // ������յ�������
        }
        vTaskDelay(20);
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
    BQ25895_Init(); // ���оƬ��ʼ��
    BQ27441_Init(); // ����оƬ��ʼ��
    AT24CXX_Init(); // ����ʧ�Դ洢оƬ��ʼ��
    PWM_WS2812B_Init();
    UCS1903Show();
    EventBits_t Power_Event_Bit;
    //TimerHandle_t breatheTimer; // ����Ч����ʱ�����
    //PWM_WS2812B_Write_24Bits(4,100);
    /* Infinite loop */

    for (;;) {
        Power_Event_Bit = xEventGroupWaitBits(
                All_EventHandle,                                  // Event group handle
                PowerState_BIT_5, // flag bits to wait for
                pdFALSE,                                          // clear these bits when the function responds
                pdFALSE,                                          // Whether to wait for all flag bits
                100                                               // Whether to wait indefinitely
                 //                                                                                                                                                               portMAX_DELAY    // Whether to wait indefinitely7
        );
        BQ25895_MultiRead(BQ25895Reg); // ��ȡ���״̬
        PowerStateUpdate();
        BQ27441_MultiRead(&BQ27441);              // ��ȡ��������
        osTimerStart(breatheTimerHandle,10);
        if ((Power_Event_Bit & PowerState_BIT_5) != 0) {//���״̬
            //HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);
            //loopBreatheEffect();//�����⵽��磬�Ϳ���������
            //HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1);		 // disable pwm for heating film





            xEventGroupClearBits(All_EventHandle, Auto_BIT_3);
            xEventGroupClearBits(All_EventHandle, Motor_BIT_2);
            xEventGroupClearBits(All_EventHandle, Heat_BIT_0); // �����¼����
        }
    if ((Power_Event_Bit & PowerState_BIT_5) == 0) {//�����
        //PWM_WS2812B_Write_24Bits(4, 0x1f1f1f);
        //HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
        vTaskDelay(100);
        ScreenUpdateSOC(BQ27441.SOC, PowerState); // �����ϴ�
        osTimerStop(breatheTimerHandle);

    }
        //vTaskDelay(20);

    }

  /* USER CODE END App_Charge_Task */
}

/* BreatheTimerCallback function */
void BreatheTimerCallback(void *argument)
{
  /* USER CODE BEGIN BreatheTimerCallback */
    uint32_t blueColor = breatheLUT[breatheIndex];
    PWM_WS2812B_Write_24Bits(4, blueColor); // ����LED����
    breatheIndex = (breatheIndex + 1) % BREATHE_STEPS; // ��������
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
  /* USER CODE END BreatheTimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

