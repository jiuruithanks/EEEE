/**
 * @file key_tasks.h
 * @author Zhao Haofei
 * @brief
 * @version 1.0
 * @date 2023-04-18
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef LED_KEY_TASKS_H
#define LED_KEY_TASKS_H

#include "task.h"
#include "pca9539.h"

void Update_Key_status( PCA9555_HandleTypeDef *hdev_key, PCA9555_HandleTypeDef *hdev_led);
void pca9555_decode(PCA9555_HandleTypeDef *hdev);

void Update_I2CLed( PCA9555_HandleTypeDef *hdev_key, PCA9555_HandleTypeDef *hdev_led);
void I2C_LEDON(PCA9555_HandleTypeDef *hdev,uint8_t num);
void Changeled( PCA9555_HandleTypeDef *hdev,uint8_t num);
#endif	/* LED_KEY_TASK_H */

