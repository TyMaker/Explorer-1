#ifndef __BALANCE_H__
#define __BALANCE_H__

#include "stm32f10x.h"



void Kalman_Filter(float Accel,float Gyro);
void Yijielvbo(float angle_m, float gyro_m);


#endif /* __BALANCE_H__ */
