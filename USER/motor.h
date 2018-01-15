#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f10x.h"

#define MOTOR_LIFT_O1_GPIO	GPIO_Pin_14
#define MOTOR_LIFT_O1_PORT	GPIOB

#define MOTOR_LIFT_O2_GPIO	GPIO_Pin_15
#define MOTOR_LIFT_O2_PORT	GPIOB

#define MOTOR_RIGHT_O1_GPIO	GPIO_Pin_13
#define MOTOR_RIGHT_O1_PORT	GPIOB

#define MOTOR_RIGHT_O2_GPIO	GPIO_Pin_12
#define MOTOR_RIGHT_O2_PORT	GPIOB

#define MOTOR_LIFT_PORT_CLOCK		RCC_APB2Periph_GPIOB
#define MOTOR_RIGHT_PORT_CLOCK	RCC_APB2Periph_GPIOB

void Motor_Init(void);
void Motor_Set(int motol1, int motol2);


#endif /* __MOTOR_H__ */