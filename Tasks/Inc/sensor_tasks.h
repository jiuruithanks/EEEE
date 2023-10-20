#ifndef SENSOR_TASKS_H
#define SENSOR_TASKS_H

#include "task.h"
#include "bsp_dcmotor.h"
#include "bsp_as5048b.h"
#include "bsp.h"


void high_transition_low(DCmotor_TypeDef *DCmotor, AS5048B_HandleTypeDef *as5048bxx);
void cal_wire_device_length(AS5048B_HandleTypeDef *as5048bxx);
void motor_real_speed(AS5048B_HandleTypeDef* as5048bxx);


#endif /* SENSOR_TASKS_H */
