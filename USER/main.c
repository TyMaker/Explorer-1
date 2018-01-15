/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   用3.5.0版本库建的工程模板
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
  
#include "stm32f10x.h"
#include "stm32f10x_systick.h"
#include "ARM_GFX.h"
#include "ARM_SSD1306.h"
#include "delay.h"
#include "stdio.h"
#include "mpu6050.h"
#include "math.h"
#include "balance.h"
#include "timer.h"
#include "Encoder.h"
#include "motor.h"

#define PI									3.141592653



extern float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
extern float angle;

extern int Balance_Pwm;
extern int Encoder_Left,Encoder_Right;

void a(void) {
				short bufa[3];
			  short bufg[3];
				char buf[10];
  			// text display tests
        setTextSize(1);
        setTextColor(WHITE);
        setCursor(0, 0);
        write_string("ARM_GFX Library test");
				setCursor(0, 16);
				setTextSize(1);
        setTextColorbg(BLACK, WHITE); // 'inverted' text
			  sprintf(buf, "%2.2fC",Read_Temperature());
        write_string(buf);
				setCursor(0, 32);
        setTextSize(1);
        setTextColor(WHITE);
				mpu6050Read_Acc(bufa);
			  sprintf(buf, "%1.1f\t%1.1f\t%1.1f",bufa[0]/16384.0, bufa[1]/16384.0, bufa[2]/16384.0);
        write_string(buf);
				setCursor(0, 40);
        setTextSize(1);
        setTextColor(WHITE);
				mpu6050Read_Gyro(bufa);
			  sprintf(buf, "%1.1f\t%1.1f\t%1.1f",bufg[0]/131.0, bufg[1]/131.0, bufg[2]/131.0);
        write_string(buf);
        //print("0x"); display.println(0xDEADBEEF, HEX);
        ssd1306_display();
        //delay(50);
        ssd1306_clearDisplay();

        // invert the display
        invertDisplay(1);
        //delay(1000); 
        invertDisplay(0);
        //delay(1000); 
        ssd1306_clearDisplay();

}

void get(void) {
	
		float Ax,Ay,Az;//(9.8m/s^2)
		float Gx,Gy,Gz;//
		float Angel_accX,Angel_accY,Angel_accZ;
		float aax,aay,aaz,ggx,ggy,ggz;
		
		short bufa[3];
		short bufg[3];
	
	  long LastTime,NowTime,TimeSpan;
	
		float Gx_offset = 0,Gy_offset = 0, Gz_offset = 0;
		
	  char buf[256];
		
	  mpu6050Read_Acc(bufa);
	  mpu6050Read_Gyro(bufg);
	
    Ax=bufa[0]/16384.00;
    Ay=bufa[1]/16384.00;
    Az=bufa[2]/16384.00;
	
	  Angel_accX=atan(Ax/sqrt(Az*Az+Ay*Ay))*180/3.14;
    Angel_accY=atan(Ay/sqrt(Ax*Ax+Az*Az))*180/3.14;
    Angel_accZ=atan(Az/sqrt(Ax*Ax+Ay*Ay))*180/3.14;

    ggx=bufg[0]/131.00;
    ggy=bufg[1]/131.00;
    ggz=bufg[2]/131.00;
		
		NowTime=millis();
	  
	  Gx=Gx+(ggx-Gx_offset)*TimeSpan/1000;
    Gy=Gy+(ggy-Gy_offset)*TimeSpan/1000;
    Gz=Gz+(ggz-Gz_offset)*TimeSpan/1000;
		
		LastTime=NowTime;
		
		setTextSize(1);
		setTextColor(WHITE);
		setCursor(0, 0);
		write_string("ARM_GFX Library test");
		sprintf(buf, "%f\n%f\n%f",Angel_accX, Angel_accY, Angel_accZ);
		write_string(buf);
		ssd1306_display();
		delay(5);
		ssd1306_clearDisplay();
}


void get_Angle(void) {
		
		/*short bufa[3];
		short bufg[3];*/
		char buf[256];
		uint16_t cc;
		float Accel_Y,Accel_X,Accel_Z,Gyro_Y,Gyro_Z;
	
		/*mpu6050Read_Acc(bufa);
	  mpu6050Read_Gyro(bufg);
	
		Gyro_Y = bufg[1];
		Gyro_Z = bufg[2];
	
		Accel_X = bufa[1];
		Accel_Y = bufa[2];
		*/
	
		Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //读取Y轴陀螺仪
		Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪
		Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //读取X轴加速度记
	  Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度记

		if(Gyro_Y>32768)  Gyro_Y-=65536;     //数据类型转换  也可通过short强制类型转换
		if(Gyro_Z>32768)  Gyro_Z-=65536;     //数据类型转换
	  if(Accel_X>32768) Accel_X-=65536;    //数据类型转换
		if(Accel_Z>32768) Accel_Z-=65536;    //数据类型转换
	
		Gyro_Balance=-Gyro_Y;                                  //更新平衡角速度
		Accel_Y=atan2(Accel_X,Accel_Z)*180/PI;                 //计算与地面的夹角	
		Gyro_Y=Gyro_Y/16.4;                                    //陀螺仪量程转换	
		
		Kalman_Filter(Accel_Y,-Gyro_Y);												 // 卡尔曼滤波器
		//Yijielvbo(Accel_Y,-Gyro_Y);														 //一节互补滤波器
	  Angle_Balance=angle;                                   //更新平衡倾角
	  Gyro_Turn=Gyro_Z;                                      //更新转向角速度
		
		setTextSize(1);
		setTextColor(WHITE);
		setCursor(0, 0);
		write_string("ARM_GFX Library test");
		sprintf(buf, "%2.2f\n%2.2f\n%2.2f",Accel_Y, Read_Temperature(), Angle_Balance);
		write_string(buf);

		cc = TIM2 -> CNT;
		sprintf(buf, "%d-%d", 64999-cc,TIM4->CNT);
		write_string(buf);
		ssd1306_display();
		delay(5);
		ssd1306_clearDisplay();
}

void show(void) {
	  char buf[20];
		setTextSize(1);
		setTextColor(WHITE);
		setCursor(0, 0);
		write_string("ARM_GFX Library test");
		sprintf(buf, "%d\n%d\n%d\n",Balance_Pwm,Encoder_Left,Encoder_Right);
		write_string(buf);
		ssd1306_display();
		delay(3);
		ssd1306_clearDisplay();

}

int main(void)
{
    //SystemInit();

		Init_SysTick();
		ssd1306_begin(0X02, 0X78, SOFTWARE_I2C, 1);
		setRotation(1);
		ARM_GFX(128, 64);
		
		ssd1306_display();
		delay(2000);
		
		MPU6050_initialize();
		MiniBalance_PWM_Init();
		
		Motor_Init();
	  Encoder_LIFT_Init();
		Encoder_RIGHT_Init();
	  Timer1_Init(99,7199);  

	  while(1)
		{
			//a();
			//get_Angle();
			//get();
			show();
    }
}

/*********************************************END OF FILE**********************/

