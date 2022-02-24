/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
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

#include "attributes.h"
#include "can.h"
#include "tim.h"

#include "SEGGER_RTT.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ENCODER_QUEUE_MSG_CNT               5   /* maximum number of messages allowed in encoder queue */
#define ENCODER_QUEUE_MSG_SIZE              2   /* size of each message in encoder queue 2 bytes (uint16_t) */

#define DEFAULT_CRUISE_SPEED                10  /* TODO: calibrate this */

#define BATTERY_REGEN_THRESHOLD             90  /* battery percentage beyond which regen is disabled */

#define CAN_FIFO0                           0
#define CAN_FIFO1                           1

#define TRUE                                1
#define FALSE                               0

#define INIT_EVENT_FLAGS_SEMAPHORE_VAL      0
#define MAX_EVENT_FLAGS_SEMAPHORE_VAL       1

#define PEDAL_MAX                           0x64 /* TODO: calibrate this */
#define PEDAL_MIN                           0x00 /* TODO: calibrate this */

#define MOTOR_OVERTEMP_THRESHOLD            150				/**< Maximum allowed motor temperature in celsius. */
#define MOTOR_OVERTEMP_CLEAR_THRESHOLD      100				/**< Temperature at which MCB will leave the MOTOR_OVERHEAT state */

#define STATE_LED_DELAY                     200
#define KERNEL_LED_DELAY                    150
#define MOTOR_OVERHEAT_DELAY                1000			/**< Delay between each time the motor temperature is read fro CAN (in ms).*/
#define EVENT_FLAG_UPDATE_DELAY             25
#define ENCODER_READ_DELAY                  50
#define READ_BATTERY_SOC_DELAY              5000
#define CAN_READ_MESSAGE_DELAY              1000

#define DEBUG 								1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

osThreadId_t kernelLEDTaskHandle;

osThreadId_t readEncoderTaskHandle;
osThreadId_t computeNextStateTaskHandle;

osThreadId_t canReadMessagesTaskHandle;

osThreadId_t sendMotorCommandTaskHandle;
osThreadId_t sendRegenCommandTaskHandle;
osThreadId_t sendCruiseCommandTaskHandle;
osThreadId_t sendNextScreenMessageTaskHandle;
osThreadId_t sendIdleCommandTaskHandle;

osThreadId_t sendMotorOverheatTaskHandle;

osThreadId_t receiveBatteryMessageTaskHandle;

osThreadId_t initialSetupTaskHandle;

osThreadId_t monitorStateTaskHandle;

osMessageQueueId_t encoderQueueHandle;

osEventFlagsId_t commandEventFlagsHandle;
osEventFlagsId_t canIdEventFlagsHandle;

osSemaphoreId_t eventFlagsSemaphoreHandle;
osSemaphoreId_t nextScreenSemaphoreHandle;

//struct CAN_Tx_Message {
//	CAN_TxHeaderTypeDef header;
//	uint8_t data[8];
//};

// indicates the current state of the main control node
enum states {
    STATE_ERROR = (uint32_t) 0x0001,
    IDLE = (uint32_t) 0x0002,
    NORMAL_READY = (uint32_t) 0x0004,
    REGEN_READY = (uint32_t) 0x0008,
    CRUISE_READY = (uint32_t) 0x0010,
    MOTOR_OVERHEAT = (uint32_t) 0x0020
} mcb_state;


// indicates the CAN ID that is in the mailbox and ready to be used by a thread
enum can_ids {
    INVALID = (uint32_t) 0x0001,
    MOTOR_ID = (uint32_t) 0x0002,
    BATTERY_ID = (uint32_t) 0x0004
} can_id;

// store the CAN message data that is read in the canReadMessages task as a global variable
uint8_t current_can_data[8];

volatile uint16_t encoder_reading = 0x0000;

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void kernelLEDTask(void *argument);

void readEncoderTask(void *argument);

void computeNextStateTask(void *argument);

void canReadMessagesTask(void *argument);

void sendMotorCommandTask(void *argument);
void sendRegenCommandTask(void *argument);
void sendCruiseCommandTask(void *argument);
void sendIdleCommandTask(void *argument);
void sendNextScreenMessageTask(void *argument);

void sendMotorOverheatTask(void *argument);

void initialSetupTask(void *argument);

void monitorStateTask(void *argument);

void receiveBatteryMessageTask(void *argument);

/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
    /* USER CODE BEGIN Init */

    encoderQueueHandle = osMessageQueueNew(ENCODER_QUEUE_MSG_CNT,
    ENCODER_QUEUE_MSG_SIZE, &encoderQueue_attributes);

    // <----- Thread object handles ----->

    kernelLEDTaskHandle = osThreadNew(kernelLEDTask, NULL, &kernelLEDTask_attributes);

    readEncoderTaskHandle = osThreadNew(readEncoderTask, NULL, &readEncoderTask_attributes);
    computeNextStateTaskHandle = osThreadNew(computeNextStateTask, NULL, &computeNextStateTask_attributes);

    canReadMessagesTaskHandle = osThreadNew(canReadMessagesTask, NULL, &canReadMessagesTask_attributes);

    sendMotorCommandTaskHandle = osThreadNew(sendMotorCommandTask, NULL, &sendMotorCommandTask_attributes);
    sendRegenCommandTaskHandle = osThreadNew(sendRegenCommandTask, NULL, &sendRegenCommandTask_attributes);
    sendCruiseCommandTaskHandle = osThreadNew(sendCruiseCommandTask, NULL, &sendCruiseCommandTask_attributes);
    sendIdleCommandTaskHandle = osThreadNew(sendIdleCommandTask, NULL, &sendIdleCommandTask_attributes);

    sendMotorOverheatTaskHandle = osThreadNew(sendMotorOverheatTask, NULL, &sendMotorOverheatTask_attributes);

    initialSetupTaskHandle = osThreadNew(initialSetupTask, NULL, &initialSetupTask_attributes);

    // sendNextScreenMessageTaskHandle = osThreadNew(sendNextScreenMessageTask, NULL, &sendNextScreenTask_attributes);

    receiveBatteryMessageTaskHandle = osThreadNew(receiveBatteryMessageTask,
    NULL, &receiveBatteryMessageTask_attributes);

    monitorStateTaskHandle = osThreadNew(monitorStateTask, NULL, &monitorStateTask_attributes);

    // <----- Event flag object handles ----->

    commandEventFlagsHandle = osEventFlagsNew(NULL);
    canIdEventFlagsHandle = osEventFlagsNew(NULL);

    // <----- Semaphore object handles ----->

    eventFlagsSemaphoreHandle = osSemaphoreNew(MAX_EVENT_FLAGS_SEMAPHORE_VAL,
    INIT_EVENT_FLAGS_SEMAPHORE_VAL, NULL);
    // nextScreenSemaphoreHandle = osSemaphoreNew(1, 0, NULL);

    assert_param(eventFlagsSemaphoreHandle != NULL);

    // set initial MCB state
    mcb_state = IDLE;

    /* USER CODE END Init */

}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

__NO_RETURN void kernelLEDTask(void *argument) {
    osKernelState_t kernel_state;

    while (1) {
        kernel_state = osKernelGetState();

        if (kernel_state == osKernelRunning) {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        }

        osDelay(KERNEL_LED_DELAY);
    }
}

void initialSetupTask(void *argument) {
    uint8_t data_send[CAN_DATA_LENGTH];

    osKernelState_t kernel_state = osKernelGetState();

    if (kernel_state == osKernelRunning) {
        for (uint8_t i = 0; i < CAN_DATA_LENGTH; i++) {
            data_send[i] = 0xff;
        }
        HAL_CAN_AddTxMessage(&hcan, &kernel_state_header, data_send, &can_mailbox);
    } else {
        for (uint8_t i = 0; i < CAN_DATA_LENGTH; i++) {
            data_send[i] = 0x00;
        }

        HAL_CAN_AddTxMessage(&hcan, &kernel_state_header, data_send, &can_mailbox);
    }
    osThreadExit();
}

/**
 * @brief  reads the encoder and places the value in the encoder queue
 * @retval None
 */
__NO_RETURN void readEncoderTask(void *argument) {
    static uint16_t old_encoder_reading = 0x0000;

    while (1) {
        encoder_reading = __HAL_TIM_GET_COUNTER(&htim2);

        // update the event flags struct
        event_flags.encoder_value_is_zero = (encoder_reading == 0);

        osMessageQueuePut(encoderQueueHandle, &encoder_reading, 0U, 0U);

        if (encoder_reading > old_encoder_reading) {
            event_flags.encoder_value_increasing = TRUE;
            event_flags.cruise_status = DISABLE;
        } else {
            event_flags.encoder_value_increasing = FALSE;
        }

        old_encoder_reading = encoder_reading;

        osDelay(ENCODER_READ_DELAY);
    }
}

/**
 * @brief  sends motor command (torque-control) CAN message once encoder value is read and a NORMAL_STATE flag is signalled
 * @retval None
 */
__NO_RETURN void sendMotorCommandTask(void *argument) {
    uint8_t data_send[CAN_DATA_LENGTH];
    uint16_t encoder_value;

    osStatus_t queue_status;

    while (1) {
        // blocks thread waiting for encoder value to be added to queue
        queue_status = osMessageQueueGet(encoderQueueHandle, &encoder_value,
        NULL, 0U);

        if (queue_status == osOK) {
            // current is linearly scaled to pedal position
            current.float_value = (float) encoder_value / (PEDAL_MAX - PEDAL_MIN);
        } else {
            // TODO: could maybe output to UART for debugging or even send a CAN message
            osThreadYield();
        }

        osEventFlagsWait(commandEventFlagsHandle, NORMAL_READY, osFlagsWaitAll,
        osWaitForever);

        // velocity is set to unattainable value for motor torque-control mode
        if (event_flags.reverse_enable) {
            velocity.float_value = -100.0;
        } else {
            velocity.float_value = 100.0;
        }

        // writing data into data_send array which will be sent as a CAN message
        for (int i = 0; i < (uint8_t) CAN_DATA_LENGTH / 2; i++) {
            data_send[i] = velocity.bytes[i];
            data_send[4 + i] = current.bytes[i];
        }

        HAL_CAN_AddTxMessage(&hcan, &drive_command_header, data_send, &can_mailbox);
    }
}

/**
 * @brief  sends regen command (velocity control) CAN message
 * @retval None
 */
__NO_RETURN void sendRegenCommandTask(void *argument) {
    uint8_t data_send[CAN_DATA_LENGTH];

    while (1) {
        // waits for event flag that signals the decision to send a regen command
        osEventFlagsWait(commandEventFlagsHandle, REGEN_READY, osFlagsWaitAll, osWaitForever);

        // velocity is set to zero for regen CAN command
        velocity.float_value = 0.0;

        // current is linearly scaled with the read regen value
        current.float_value = (float) regen_value / (ADC_MAX - ADC_MIN);

        // writing data into data_send array which will be sent as a CAN message
        for (int i = 0; i < (uint8_t) CAN_DATA_LENGTH / 2; i++) {
            data_send[i] = velocity.bytes[i];
            data_send[4 + i] = current.bytes[i];
        }

        HAL_CAN_AddTxMessage(&hcan, &drive_command_header, data_send, &can_mailbox);
    }
}

/**
 * @brief  sends cruise-control command (velocity control) CAN message
 * @retval None
 */
__NO_RETURN void sendCruiseCommandTask(void *argument) {
    uint8_t data_send[CAN_DATA_LENGTH];

    while (1) {
        // waits for event flag that signals the decision to send a cruise control command
        osEventFlagsWait(commandEventFlagsHandle, CRUISE_READY, osFlagsWaitAll,
        osWaitForever);

        // current set to maximum for a cruise control message
        current.float_value = 100.0;

        // set velocity to cruise value
        velocity.float_value = (float) cruise_value;


        // writing data into data_send array which will be sent as a CAN message
        for (int i = 0; i < (uint8_t) CAN_DATA_LENGTH / 2; i++) {
            data_send[i] = velocity.bytes[i];
            data_send[4 + i] = current.bytes[i];
        }

        HAL_CAN_AddTxMessage(&hcan, &drive_command_header, data_send, &can_mailbox);
    }
}

/**
 * @brief  sends an "idle" CAN message when the car is not moving
 * @retval None
 */
__NO_RETURN void sendIdleCommandTask(void *argument) {
    uint8_t data_send[CAN_DATA_LENGTH];

    while (1) {
        // waits for event flag that signals the decision to send an idle command
        osEventFlagsWait(commandEventFlagsHandle, IDLE, osFlagsWaitAll, osWaitForever);

        // zeroed since car would not be moving in idle state
        current.float_value = 0.0;
        velocity.float_value = 0.0;

        // writing data into data_send array which will be sent as a CAN message
        for (int i = 0; i < (uint8_t) CAN_DATA_LENGTH / 2; i++) {
            data_send[i] = velocity.bytes[i];
            data_send[4 + i] = current.bytes[i];
        }

        HAL_CAN_AddTxMessage(&hcan, &drive_command_header, data_send, &can_mailbox);
    }
}

/**
 * @brief  sends next_screen command (to switch telemetry output) CAN message
 * @param  argument: Not used
 * @retval None
 */
__NO_RETURN void sendNextScreenMessageTask(void *argument) {
    // maybe make this a global variable
    uint8_t data_send[CAN_CONTROL_DATA_LENGTH];

    while (1) {
        // wait for next-screen semaphore
        osSemaphoreAcquire(nextScreenSemaphoreHandle, osWaitForever);

        data_send[1] = 0x01;
        HAL_CAN_AddTxMessage(&hcan, &screen_cruise_control_header, data_send, &can_mailbox);
    }
}


/**
 * @brief  Decides what the next thread will be given certain event flags
 * @retval An enum of type `states` describing the next state for the MCB
 */
enum states calculateNextState() {
    if (event_flags.motor_overheat) {
        return MOTOR_OVERHEAT;
    } else if (event_flags.regen_enable && regen_value > 0 && battery_soc < BATTERY_REGEN_THRESHOLD) {
        return REGEN_READY;
    } else if (!event_flags.encoder_value_is_zero && !event_flags.cruise_status) {
        return NORMAL_READY;
    } else if (event_flags.cruise_status && !event_flags.brake_in && !event_flags.encoder_value_increasing
            && (cruise_value > 0 || encoder_reading > 0)) {
        return CRUISE_READY;
    } else {
        return IDLE;
    }

    return STATE_ERROR;
}

/**
 * @brief  Decides which thread will send a CAN message
 * @param  argument: Not used
 * @retval None
 */
__NO_RETURN void computeNextStateTask(void *argument) {
    char *state_string = "NULL";
    enum states current_mcb_state = IDLE;

    while (1) {
        current_mcb_state = mcb_state;

        // order of priorities beginning with most important: regen braking, encoder motor command, cruise control

        // next state depends on previous state and other flag inputs (Mealy machine)
        switch (current_mcb_state) {

        case STATE_ERROR:
            mcb_state = calculateNextState();
            break;

        case IDLE:
            mcb_state = calculateNextState();
            break;

        case NORMAL_READY:
            mcb_state = calculateNextState();
            if (mcb_state == CRUISE_READY) {
                cruise_value = encoder_reading;
            }
            break;

        case REGEN_READY:
            mcb_state = calculateNextState();
            break;

        case CRUISE_READY:
            mcb_state = calculateNextState();
            break;

        case MOTOR_OVERHEAT:
            mcb_state = calculateNextState();
            break;

        default:
            mcb_state = calculateNextState();
        }

        SEGGER_RTT_SetTerminal(0);

        switch (mcb_state) {
        case IDLE:
            state_string = "IDLE";
            break;
        case NORMAL_READY:
            state_string = "NORMAL_READY";
            break;
        case REGEN_READY:
            state_string = "REGEN_READY";
            break;
        case CRUISE_READY:
            state_string = "CRUISE_READY";
            break;
        case MOTOR_OVERHEAT:
            state_string = "MOTOR_OVERHEAT";
            break;
        case STATE_ERROR:
            state_string = "STATE_ERROR";
            break;

        default:
            state_string = "MAJOR_ERROR";
        }

        // writes the current state of the MCB to the SEGGER JLINK terminal (can use J-Link RTT Viewer to see output)
        SEGGER_RTT_printf(0, "MCB state: 0x%x (%s)\n", mcb_state, state_string);

        SEGGER_RTT_SetTerminal(2);
        SEGGER_RTT_printf(0, "Current: %d | Velocity: %d\n", current.float_value, velocity.float_value);

        osEventFlagsSet(commandEventFlagsHandle, mcb_state);
        osDelay(EVENT_FLAG_UPDATE_DELAY);
    }
}

/**
 * @brief  Decides which thread will receive a CAN message
 * @param  argument: Not used
 * @retval None
 */
__NO_RETURN void canReadMessagesTask(void *argument) {
    HAL_StatusTypeDef status;

    while (1) {
        // wait for CAN RX ISR to set thread flags
        osThreadFlagsWait(CAN_READY, osFlagsWaitAll, osWaitForever);

        // there are multiple CAN IDs being passed through the filter, pull out the current message
        HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &can_rx_header, current_can_data);

        // motor ID
        if (can_rx_header.StdId == 0x50B) {
            can_id = MOTOR_ID;
        }
        // battery SOC ID
        else if (can_rx_header.StdId == 0x626) {
            can_id = BATTERY_ID;
        } else {
            // Major error. A CAN ID was received even though the filter only accepts 0x50B and 0x626
            can_id = INVALID;
        }

        osEventFlagsSet(canIdEventFlagsHandle, can_id);

        status = HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

        assert_param(status == HAL_OK);
    }
}

/**
 * @brief  Reads battery SOC from CAN bus (message ID 0x626, data byte 0)
 * @param  argument: Not used
 * @retval None
 */
__NO_RETURN void receiveBatteryMessageTask(void *argument) {

    while (1) {

        // waits for event flag that signals the Battery SOC ID has been received
        osEventFlagsWait(canIdEventFlagsHandle, BATTERY_ID, osFlagsWaitAll,
        osWaitForever);

        // if the battery SOC is out of range, assume it is at 100% as a safety measure
        if (current_can_data[0] < 0 || current_can_data[0] > 100) {
            // TODO: should send out a CAN message here
            battery_soc = 100;
        } else {
            battery_soc = current_can_data[0];
        }
    }
}

/**
 * @brief  	Reads motor temperature from CAN bus (message ID 0x50B). Since the the Tritium BMS does not have over-temperature
 * 			shutdown, the motor will need to stop sending commands when it reaches a temperature of MAX_MOTOR_TEMPERATURE.
 *
 * @param  	argument: Not used
 * @retval 	None
 */
__NO_RETURN void sendMotorOverheatTask(void *argument) {
    while (1) {

        // waits for event flag that signals the Motor ID has been received
        osEventFlagsWait(canIdEventFlagsHandle, MOTOR_ID, osFlagsWaitAll,
        osWaitForever);

        // assign the values from the CAN message into the
        // use the union to convert the 4 bytes to a 32-bit float
        for (int i = 0; i < (uint8_t) CAN_DATA_LENGTH / 2; i++) {
            /*
             * transfer the incoming bytes LIFO 	TODO: test IEEE 754 conversion on hardware
             * for example, 0xAABBCCDD
             * 		received[0] = 0xDD -> copied[3] = 0xDD
             * 		received[1] = 0xCC -> copied[2] - 0xCC
             * 		received[2] = 0xBB -> copied[1] = 0xBB
             * 		received[3] = 0xAA -> copied[0] - 0xAA
             */
            motor_temperature.bytes[i] = current_can_data[i];
        }

        // if the motor temperature is over heating, stop sending commands
        if (motor_temperature.float_value >= MOTOR_OVERTEMP_THRESHOLD) {
            // change the state so that sendMotorCommandTask will not run
            event_flags.motor_overheat = TRUE;
        } else {
            // change the state so that sendMotorCommandTask will not run
            event_flags.motor_overheat = FALSE;
        }
    }
}

void monitorStateTask(void *argument) {
    while (1) {
        switch (mcb_state) {
        case NORMAL_READY:
            HAL_GPIO_TogglePin(SEND_NORMAL_GPIO_Port, SEND_NORMAL_Pin);
            HAL_GPIO_WritePin(SEND_REGEN_GPIO_Port, SEND_REGEN_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(SEND_CRUISE_GPIO_Port, SEND_CRUISE_Pin, GPIO_PIN_RESET);
            break;
        case REGEN_READY:
            HAL_GPIO_WritePin(SEND_NORMAL_GPIO_Port, SEND_NORMAL_Pin, GPIO_PIN_RESET);
            HAL_GPIO_TogglePin(SEND_REGEN_GPIO_Port, SEND_REGEN_Pin);
            HAL_GPIO_WritePin(SEND_CRUISE_GPIO_Port, SEND_CRUISE_Pin, GPIO_PIN_RESET);
            break;
        case CRUISE_READY:
            HAL_GPIO_WritePin(SEND_NORMAL_GPIO_Port, SEND_NORMAL_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(SEND_REGEN_GPIO_Port, SEND_REGEN_Pin, GPIO_PIN_RESET);
            HAL_GPIO_TogglePin(SEND_CRUISE_GPIO_Port, SEND_CRUISE_Pin);
            break;
        default:
            HAL_GPIO_WritePin(GPIOA, SEND_NORMAL_Pin | SEND_CRUISE_Pin | SEND_REGEN_Pin, GPIO_PIN_RESET);
            break;
        }

        if (mcb_state == CRUISE_READY) {
            HAL_GPIO_WritePin(CRUISE_STAT_GPIO_Port, CRUISE_STAT_Pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(CRUISE_STAT_GPIO_Port, CRUISE_STAT_Pin, GPIO_PIN_RESET);
        }

        SEGGER_RTT_SetTerminal(1);
        SEGGER_RTT_printf(0, "Motor temperature (degC): %d.0 | Battery SOC: %d%% \n",
                (uint32_t) motor_temperature.float_value, battery_soc);

        osDelay(STATE_LED_DELAY);
    }
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
