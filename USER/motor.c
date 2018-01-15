#include "motor.h"

int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

void Motor_Init(void) {

		GPIO_InitTypeDef GPIO_InitStructer;
    RCC_APB2PeriphClockCmd(MOTOR_LIFT_PORT_CLOCK | MOTOR_RIGHT_PORT_CLOCK, ENABLE);
    GPIO_InitStructer.GPIO_Pin = MOTOR_LIFT_O1_GPIO | MOTOR_LIFT_O2_GPIO;
    GPIO_InitStructer.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructer.GPIO_Mode = GPIO_Mode_Out_PP;
	
    GPIO_Init(MOTOR_LIFT_O1_PORT, &GPIO_InitStructer);
	
	  GPIO_InitStructer.GPIO_Pin = MOTOR_RIGHT_O1_GPIO | MOTOR_RIGHT_O2_GPIO;
    GPIO_Init(MOTOR_RIGHT_O2_PORT, &GPIO_InitStructer);
	
		GPIO_ResetBits(MOTOR_LIFT_O1_PORT, MOTOR_LIFT_O1_GPIO);
		GPIO_ResetBits(MOTOR_LIFT_O2_PORT, MOTOR_LIFT_O2_GPIO);
		GPIO_ResetBits(MOTOR_RIGHT_O1_PORT, MOTOR_RIGHT_O1_GPIO);
		GPIO_ResetBits(MOTOR_RIGHT_O2_PORT, MOTOR_RIGHT_O2_GPIO);

}

void Motor_Set(int motol1, int motol2) {

	if(motol1 == 0) {
			GPIO_ResetBits(MOTOR_LIFT_O1_PORT, MOTOR_LIFT_O1_GPIO);
			GPIO_ResetBits(MOTOR_LIFT_O2_PORT, MOTOR_LIFT_O2_GPIO);
	}
	if(motol2 == 0) {
		GPIO_ResetBits(MOTOR_RIGHT_O1_PORT, MOTOR_RIGHT_O1_GPIO);
		GPIO_ResetBits(MOTOR_RIGHT_O2_PORT, MOTOR_RIGHT_O2_GPIO);

	}
	
	if(motol1 < 0) {
			GPIO_SetBits(MOTOR_LIFT_O1_PORT, MOTOR_LIFT_O1_GPIO);
			GPIO_ResetBits(MOTOR_LIFT_O2_PORT, MOTOR_LIFT_O2_GPIO);
	}
	else {
			GPIO_ResetBits(MOTOR_LIFT_O1_PORT, MOTOR_LIFT_O1_GPIO);
			GPIO_SetBits(MOTOR_LIFT_O2_PORT, MOTOR_LIFT_O2_GPIO);
	}
	TIM3->CCR4 = myabs(motol1);
	
	if(motol2 < 0) {
		GPIO_SetBits(MOTOR_RIGHT_O1_PORT, MOTOR_RIGHT_O1_GPIO);
		GPIO_ResetBits(MOTOR_RIGHT_O2_PORT, MOTOR_RIGHT_O2_GPIO);
	}
	else {
		GPIO_ResetBits(MOTOR_RIGHT_O1_PORT, MOTOR_RIGHT_O1_GPIO);
		GPIO_SetBits(MOTOR_RIGHT_O2_PORT, MOTOR_RIGHT_O2_GPIO);
	}
	TIM3->CCR3 = myabs(motol2);


}