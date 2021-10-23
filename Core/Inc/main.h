/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

extern TIM_HandleTypeDef htim3;

extern union FloatBytes {
	float float_value;
	uint8_t bytes[4];
} current, velocity;

// this struct is updated when an external interrupt comes in and combines boolean flags 
// that helps to decide whether to send a regen command, normal command
typedef struct input_flags {
  volatile uint8_t regen_enable;
  volatile uint8_t reverse_enable;
  volatile uint8_t cruise_status;
  volatile uint8_t brake_in;
  volatile uint8_t regen_value_is_zero;
  volatile uint8_t encoder_value_is_zero;
  volatile uint8_t encoder_value_increasing;
  volatile uint8_t next_screen;
} input_flags;

extern input_flags event_flags;

extern uint32_t regen_value;
extern int8_t cruise_value;

extern uint8_t battery_soc;

extern uint32_t buffer;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define REGEN_EN_Pin GPIO_PIN_0
#define REGEN_EN_GPIO_Port GPIOC
#define REGEN_EN_EXTI_IRQn EXTI0_IRQn
#define CRUISE_EN_Pin GPIO_PIN_1
#define CRUISE_EN_GPIO_Port GPIOC
#define CRUISE_EN_EXTI_IRQn EXTI1_IRQn
#define RVRS_EN_Pin GPIO_PIN_2
#define RVRS_EN_GPIO_Port GPIOC
#define RVRS_EN_EXTI_IRQn EXTI2_IRQn
#define CRUISE_DIS_Pin GPIO_PIN_3
#define CRUISE_DIS_GPIO_Port GPIOC
#define CRUISE_DIS_EXTI_IRQn EXTI3_IRQn
#define ENC_A_Pin GPIO_PIN_0
#define ENC_A_GPIO_Port GPIOA
#define ENC_B_Pin GPIO_PIN_1
#define ENC_B_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define REGEN_VAL_Pin GPIO_PIN_6
#define REGEN_VAL_GPIO_Port GPIOA
#define CRUISE_UP_Pin GPIO_PIN_8
#define CRUISE_UP_GPIO_Port GPIOC
#define CRUISE_UP_EXTI_IRQn EXTI9_5_IRQn
#define CRUISE_DOWN_Pin GPIO_PIN_9
#define CRUISE_DOWN_GPIO_Port GPIOC
#define CRUISE_DOWN_EXTI_IRQn EXTI9_5_IRQn
#define CRUISE_STAT_Pin GPIO_PIN_9
#define CRUISE_STAT_GPIO_Port GPIOA
#define SEND_CRUISE_Pin GPIO_PIN_10
#define SEND_CRUISE_GPIO_Port GPIOA
#define SEND_REGEN_Pin GPIO_PIN_11
#define SEND_REGEN_GPIO_Port GPIOA
#define SEND_NORMAL_Pin GPIO_PIN_12
#define SEND_NORMAL_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define ADC_MAX 0xFFF
#define ADC_MIN 0

#define ENCODER_TIMER_TICKS (uint32_t) 1

#define CRUISE_INCREMENT_VALUE 	1
#define CRUISE_MAX              100         /* value is in km/h */
#define CRUISE_MIN              0           /* value is in km/h */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
