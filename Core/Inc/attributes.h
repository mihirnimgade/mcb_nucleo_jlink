/*
 * attributes.h
 *
 *  Created on: Feb. 23, 2021
 *      Author: Mihir Nimgade
 */

#ifndef INC_ATTRIBUTES_H_
#define INC_ATTRIBUTES_H_

#include "cmsis_os.h"

extern const osThreadAttr_t kernelLEDTask_attributes;

extern const osThreadAttr_t readEncoderTask_attributes;
extern const osThreadAttr_t computeNextStateTask_attributes;

extern const osThreadAttr_t canReadMessagesTask_attributes;

extern const osThreadAttr_t sendMotorCommandTask_attributes;
extern const osThreadAttr_t sendRegenCommandTask_attributes;
extern const osThreadAttr_t sendCruiseCommandTask_attributes;
extern const osThreadAttr_t sendIdleCommandTask_attributes;

extern const osThreadAttr_t sendNextScreenTask_attributes;

extern const osThreadAttr_t readRegenValueTask_attributes;

extern const osThreadAttr_t receiveBatteryMessageTask_attributes;

extern const osTimerAttr_t encoderTimer_attributes;

extern const osMessageQueueAttr_t encoderQueue_attributes;

extern const osThreadAttr_t monitorStateTask_attributes;

extern const osThreadAttr_t initialSetupTask_attributes;

extern const osThreadAttr_t sendMotorOverheatTask_attributes;

#endif /* INC_ATTRIBUTES_H_ */
