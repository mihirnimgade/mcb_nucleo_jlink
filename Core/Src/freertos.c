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

#define ENCODER_QUEUE_MSG_CNT 				5  /* maximum number of messages allowed in encoder queue */
#define ENCODER_QUEUE_MSG_SIZE 				2  /* size of each message in encoder queue 2 bytes (uint16_t) */

#define DEFAULT_CRUISE_SPEED 				10 /* TODO: calibrate this */

#define BATTERY_REGEN_THRESHOLD 			90 /* maximum battery percentage at which regen is enabled */

#define CAN_FIFO0 							0
#define CAN_FIFO1 							1

#define TRUE 								1
#define FALSE 								0

#define INIT_EVENT_FLAGS_SEMAPHORE_VAL 		0
#define MAX_EVENT_FLAGS_SEMAPHORE_VAL 		1

#define PEDAL_MAX 							0x64 /* TODO: calibrate this */
#define PEDAL_MIN 							0x00 /* TODO: calibrate this */

#define EVENT_FLAG_UPDATE_DELAY 			25
#define ENCODER_READ_DELAY 					50
#define READ_BATTERY_SOC_DELAY 				5000

#define MAX_MOTOR_TEMPERATURE				60			/**< Maximum motor temperature in celsius. */

#define STATE_LED_DELAY 					200

#define KERNEL_LED_DELAY 					150

#define MOTOR_OVERHEAT_DELAY				1000			/**< Delay between each time the motor temperature is read fro CAN (in ms).*/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

osThreadId_t kernelLEDTaskHandle;

osThreadId_t readEncoderTaskHandle;
osThreadId_t updateEventFlagsTaskHandle;

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

osSemaphoreId_t eventFlagsSemaphoreHandle;
osSemaphoreId_t nextScreenSemaphoreHandle;

// indicates the current state of the main control node
enum states {
    IDLE = (uint32_t) 0x0001,
    NORMAL_READY = (uint32_t) 0x0002,
    REGEN_READY = (uint32_t) 0x0004,
    CRUISE_READY = (uint32_t) 0x0008,
	MOTOR_OVERHEAT = (uint32_t) 0x0010
} state;

volatile uint16_t encoder_reading = 0x0000;

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void kernelLEDTask(void *argument);

void readEncoderTask(void *argument);

void updateEventFlagsTask(void *argument);

void sendMotorCommandTask(void *argument);
void sendRegenCommandTask(void *argument);
void sendCruiseCommandTask(void *argument);
void sendIdleCommandTask(void *argument);
void sendNextScreenMessageTask(void *argument);

void sendMotorOverheatTask (void *argument);

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

    encoderQueueHandle = osMessageQueueNew(ENCODER_QUEUE_MSG_CNT, ENCODER_QUEUE_MSG_SIZE, &encoderQueue_attributes);

    // <----- Thread object handles ----->

    kernelLEDTaskHandle = osThreadNew(kernelLEDTask, NULL, &kernelLEDTask_attributes);

    readEncoderTaskHandle = osThreadNew(readEncoderTask, NULL, &readEncoderTask_attributes);
    updateEventFlagsTaskHandle = osThreadNew(updateEventFlagsTask, NULL, &updateEventFlagsTask_attributes);

    sendMotorCommandTaskHandle = osThreadNew(sendMotorCommandTask, NULL, &sendMotorCommandTask_attributes);
    sendRegenCommandTaskHandle = osThreadNew(sendRegenCommandTask, NULL, &sendRegenCommandTask_attributes);
    sendCruiseCommandTaskHandle = osThreadNew(sendCruiseCommandTask, NULL, &sendCruiseCommandTask_attributes);
    sendIdleCommandTaskHandle = osThreadNew(sendIdleCommandTask, NULL, &sendIdleCommandTask_attributes);

	sendMotorOverheatTaskHandle = osThreadNew(sendMotorOverheatTask, NULL, &sendMotorOverheatTask_attributes);

	initialSetupTaskHandle = osThreadNew(initialSetupTask, NULL, &initialSetupTask_attributes);

    // sendNextScreenMessageTaskHandle = osThreadNew(sendNextScreenMessageTask, NULL, &sendNextScreenTask_attributes);

    // receiveBatteryMessageTaskHandle = osThreadNew(receiveBatteryMessageTask, NULL, &receiveBatteryMessageTask_attributes);

    monitorStateTaskHandle = osThreadNew(monitorStateTask, NULL, &monitorStateTask_attributes);

    // <----- Event flag object handles ----->

    commandEventFlagsHandle = osEventFlagsNew(NULL);

    // <----- Semaphore object handles ----->

    eventFlagsSemaphoreHandle = osSemaphoreNew(MAX_EVENT_FLAGS_SEMAPHORE_VAL, INIT_EVENT_FLAGS_SEMAPHORE_VAL, NULL);
    nextScreenSemaphoreHandle = osSemaphoreNew(1, 0, NULL);

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

        if (encoder_reading != old_encoder_reading) {
            osMessageQueuePut(encoderQueueHandle, &encoder_reading, 0U, 0U);
        }

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
        queue_status = osMessageQueueGet(encoderQueueHandle, &encoder_value, NULL, 0U);

        if (queue_status == osOK) {
            // current is linearly scaled to pedal position
            current.float_value = (float) encoder_value / (PEDAL_MAX - PEDAL_MIN);
        } else {
            // TODO: could maybe output to UART for debugging or even send a CAN message
            osThreadYield();
        }

        osEventFlagsWait(commandEventFlagsHandle, NORMAL_READY, osFlagsWaitAll, osWaitForever);

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
__NO_RETURN void sendCruiseCommandTask (void *argument) {
    uint8_t data_send[CAN_DATA_LENGTH];

    while (1) {
        // waits for event flag that signals the decision to send a cruise control command
        osEventFlagsWait(commandEventFlagsHandle, CRUISE_READY, osFlagsWaitAll, osWaitForever);

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
__NO_RETURN void sendIdleCommandTask (void *argument) {
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
__NO_RETURN void sendNextScreenMessageTask (void *argument) {
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
  * @brief  Decides which thread will send a CAN message
  * @param  argument: Not used
  * @retval None
  */
__NO_RETURN void updateEventFlagsTask(void *argument) {
    while (1) {
        // order of priorities beginning with most important: regen braking, encoder motor command, cruise control
    	if (event_flags.motor_overheat) {
    		state = MOTOR_OVERHEAT;
    	}
		else if (event_flags.regen_enable && regen_value > 0 && battery_soc < BATTERY_REGEN_THRESHOLD) {
            state = REGEN_READY;
        }
        else if (!event_flags.encoder_value_is_zero && !event_flags.cruise_status) {
            state = NORMAL_READY;
        }
        else if (event_flags.cruise_status && !event_flags.brake_in && !event_flags.encoder_value_increasing && (cruise_value > 0 || encoder_reading > 0)) {
			if (state == NORMAL_READY) {
				cruise_value = encoder_reading;
			}
            state = CRUISE_READY;
        }
        else {
            state = IDLE;
        }

        osEventFlagsSet(commandEventFlagsHandle, state);
        osDelay(EVENT_FLAG_UPDATE_DELAY);
    }
}

/**
  * @brief  Reads battery SOC from CAN bus (message ID 0x626, data byte 0)
  * @param  argument: Not used
  * @retval None
  */
__NO_RETURN void receiveBatteryMessageTask (void *argument) {
    uint8_t battery_msg_data[8];

    while (1) {
        if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_FIFO0)) {
            // filtering for 0x626 ID done in hardware 
            HAL_CAN_GetRxMessage(&hcan, CAN_FIFO0, &can_rx_header, battery_msg_data);

            // if the battery SOC is out of range, assume it is at 100% as a safety measure
            if (battery_msg_data[0] < 0 || battery_msg_data[0] > 100) {
                // TODO: somehow indicate to the outside world that this has happened
                battery_soc = 100;
            } else {
                battery_soc = battery_msg_data[0];
            }
        }

        osDelay(READ_BATTERY_SOC_DELAY);
    }
}

/**
 * @brief  	Reads motor temperature from CAN bus (message ID 0x50B). Since the the Tritium BMS does not have over-temperature
 * 			shutdown, the motor will need to stop sending commands when it reaches a temperature of MAX_MOTOR_TEMPERATURE.
 *
 * @param  	argument: Not used
 * @retval 	None
 */
__NO_RETURN void sendMotorOverheatTask (void *argument) {
	uint8_t motor_temperature_data[CAN_DATA_LENGTH]; // the motor temperature is bytes [3:0] TODO: this is an assumption

	while (1) {
		if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0)) {
			// there are multiple CAN IDs being passed through the filter, check if the message is the motor temperature
			HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &can_rx_header, motor_temperature_data);
			if (can_rx_header.StdId == 0x50B) {

				// assign the values from the CAN message into the
				// use the union to convert the 4 bytes to a 32-bit float
				for (int i = 0; i < (uint8_t)CAN_DATA_LENGTH / 2; i++)
				{
					/*
					 * transfer the incoming bytes LIFO 	TODO: test IEEE 754 conversion on hardware
					 * for example, 0xAABBCCDD
					 * 		received[0] = 0xDD -> copied[3] = 0xDD
					 * 		received[1] = 0xCC -> copied[2] - 0xCC
					 * 		received[2] = 0xBB -> copied[1] = 0xBB
					 * 		received[3] = 0xAA -> copied[0] - 0xAA
					 */
					motor_temperature.bytes[i] = motor_temperature_data[i];
				}

				// if the motor temperature is over heating, stop sending commands
				if (motor_temperature.float_value >= MAX_MOTOR_TEMPERATURE) {
					// change the state so that sendMotorCommandTask will not run
					event_flags.motor_overheat = 0x01;
				} else {
					// change the state so that sendMotorCommandTask will not run
					event_flags.motor_overheat = 0x00;
				}

			}
		}
		osDelay(MOTOR_OVERHEAT_DELAY);
	}
}

void monitorStateTask(void *argument) {
	while (1) {
		switch (state) {
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
			HAL_GPIO_WritePin(GPIOA, SEND_NORMAL_Pin|SEND_CRUISE_Pin|SEND_REGEN_Pin, GPIO_PIN_RESET);
			break;
		}

		if (state == CRUISE_READY) {
			HAL_GPIO_WritePin(CRUISE_STAT_GPIO_Port, CRUISE_STAT_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(CRUISE_STAT_GPIO_Port, CRUISE_STAT_Pin, GPIO_PIN_RESET);
		}


		osDelay(STATE_LED_DELAY);
	}
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
