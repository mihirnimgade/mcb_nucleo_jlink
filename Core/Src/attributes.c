/*
 * attributes.c
 *
 *  Created on: Feb. 23, 2021
 *      Author: Mihir Nimgade
 */

#include "attributes.h"

// <----- RTOS object attributes ----->

const osThreadAttr_t kernelLEDTask_attributes = { .name = "kernelLEDTask",
        .priority = (osPriority_t) osPriorityLow, .stack_size = 128 * 2 };

const osThreadAttr_t readEncoderTask_attributes = { .name = "readEncoderTask",
        .priority = (osPriority_t) osPriorityHigh, .stack_size = 128 * 2 };

const osThreadAttr_t sendMotorCommandTask_attributes = { .name =
        "sendMotorCommandTask", .priority = (osPriority_t) osPriorityNormal,
        .stack_size = 128 * 4 };

const osThreadAttr_t sendRegenCommandTask_attributes = { .name =
        "sendRegenCommandTask", .priority = (osPriority_t) osPriorityHigh,
        .stack_size = 128 * 2 };

const osThreadAttr_t sendCruiseCommandTask_attributes = { .name =
        "sendCruiseCommandTask", .priority =
        (osPriority_t) osPriorityBelowNormal, .stack_size = 128 * 2 };

const osThreadAttr_t sendNextScreenTask_attributes = { .name =
        "sendNextScreenTask", .priority = (osPriority_t) osPriorityBelowNormal,
        .stack_size = 128 * 2 };

const osThreadAttr_t sendIdleCommandTask_attributes = { .name =
        "sendIdleCommandTask", .priority = (osPriority_t) osPriorityNormal,
        .stack_size = 128 * 4 };

const osThreadAttr_t computeNextStateTask_attributes = { .name =
        "computeNextStateTask", .priority = (osPriority_t) osPriorityHigh,
        .stack_size = 128 * 4 };

const osThreadAttr_t canReadMessagesTask_attributes = { .name =
        "canReadMessagesTask", .priority = (osPriority_t) osPriorityLow,
        .stack_size = 128 * 4 };

const osThreadAttr_t receiveBatteryMessageTask_attributes = { .name =
        "receiveBatteryMessageTask", .priority =
        (osPriority_t) osPriorityAboveNormal, .stack_size = 128 * 4 };

const osThreadAttr_t monitorStateTask_attributes = { .name = "monitorStateTask",
        .priority = (osPriority_t) osPriorityLow, .stack_size = 128 * 4 };

const osThreadAttr_t initialSetupTask_attributes = { .name = "initialSetupTask",
        .priority = (osPriority_t) osPriorityLow, .stack_size = 128 * 2 };

const osThreadAttr_t sendMotorOverheatTask_attributes = { .name =
        "sendMotorOverheat", .priority = (osPriority_t) osPriorityAboveNormal,
        .stack_size = 128 * 4 };

const osMessageQueueAttr_t encoderQueue_attributes = { .name = "encoderQueue" };
