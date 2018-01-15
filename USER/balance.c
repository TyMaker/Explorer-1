#include "balance.h"
#include "mpu6050.h"
#include "math.h"
#include "motor.h"


#define PI 3.1415926

int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
int Encoder_Left,Encoder_Right;             //���ұ��������������


float Angle_Balance,Gyro_Balance,Gyro_Turn; //ƽ����� ƽ�������� ת��������
float K1 =0.02; 
float angle, angle_dot; 	
float Q_angle=0.001;// ����������Э����
float Q_gyro=0.003;//0.03 ����������Э���� ����������Э����Ϊһ��һ�����о���
float R_angle=0.5;// ����������Э���� �Ȳ���ƫ��
float dt=0.005;//                 
char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };


void Kalman_Filter(float Accel,float Gyro)		
{
	angle+=(Gyro - Q_bias) * dt; //�������
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
	PP[0][1] += Pdot[1] * dt;   // =����������Э����
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - angle;	//zk-�������
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //����������Э����
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle	+= K_0 * Angle_err;	 //�������
	Q_bias	+= K_1 * Angle_err;	 //�������
	angle_dot   = Gyro - Q_bias;	 //���ֵ(�������)��΢��=���ٶ�
}

void Yijielvbo(float angle_m, float gyro_m)
{
	 //float K1 =0.1;  
   angle = K1 * angle_m+ (1-K1) * (angle + gyro_m * 0.005);
}

/*float K2 =0.2;  
float x1,x2,y1;  
float angle2; 

void Erjielvbo(float angle_m,float gyro_m)  
{  
    x1=(angle_m-angle2)*(1-K2)*(1-K2);  
    y1=y1+x1*dt;  
    x2=y1+2*(1-K2)*(angle_m-angle2)+gyro_m;  
    angle2=angle2+ x2*dt;  
    A2_P = angle2;  
}  */

void readEncoder(void)
{
	  u16 Encoder_L,Encoder_R;       //===���ұ��������������
		Encoder_R = TIM4 -> CNT;       //===��ȡ��������1����	
		TIM4 -> CNT=0;                 //===����������  
	  Encoder_L= TIM2 -> CNT;        //===��ȡ��������2����	
	  TIM2 -> CNT=0;	               //===����������
		if(Encoder_L>32768)  Encoder_Left=Encoder_L-65000; else  Encoder_Left=Encoder_L;  
	  //=��������ԭ���ǣ���������0�������65000���¼��������������������ڿ��Ƴ�����ʹ��
	  if(Encoder_R>32768)  Encoder_Right=Encoder_R-65000; else  Encoder_Right=Encoder_R;
	 	Encoder_Left=-Encoder_Left;//����ȡ������Ϊ��ƽ��С���������������ת��180�Ȱ�װ�ģ�Ϊ�˱�֤ǰ������ʱ��ı��������ݷ���һ��
}

void Get_Angle(void) {
	
		char buf[256];
		uint16_t cc;
		float Accel_Y,Accel_X,Accel_Z,Gyro_Y,Gyro_Z;
	
		Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //��ȡY��������
		Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //��ȡZ��������
		Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //��ȡX����ٶȼ�
	  Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //��ȡZ����ٶȼ�

		if(Gyro_Y>32768)  Gyro_Y-=65536;     //��������ת��  Ҳ��ͨ��shortǿ������ת��
		if(Gyro_Z>32768)  Gyro_Z-=65536;     //��������ת��
	  if(Accel_X>32768) Accel_X-=65536;    //��������ת��
		if(Accel_Z>32768) Accel_Z-=65536;    //��������ת��
	
		Gyro_Balance=-Gyro_Y;                                  //����ƽ����ٶ�
		Accel_Y=atan2(Accel_X,Accel_Z)*180/PI;                 //���������ļн�	
		Gyro_Y=Gyro_Y/16.4;                                    //����������ת��	
		
		Kalman_Filter(Accel_Y,-Gyro_Y);												 // �������˲���
		//Yijielvbo(Accel_Y,-Gyro_Y);														 //һ�ڻ����˲���
	  Angle_Balance=angle;                                   //����ƽ�����
	  Gyro_Turn=Gyro_Z;                                      //����ת����ٶ�

}

int balance(float Angle,float Gyro)
{  
   float Bias;
	 int balance;
	 Bias=Angle-2.0;              //===���ƽ��ĽǶ���ֵ �ͻ�е��� -0��ζ������������0�ȸ��� �������������5�ȸ��� �Ǿ�Ӧ�ü�ȥ5
	 balance=235*Bias+Gyro*0.125;//===����ƽ����Ƶĵ��PWM  PD����   35��Pϵ�� 0.125��Dϵ�� 
	 return balance;
}


int velocity(int encoder_left,int encoder_right)
{  
	  static int Velocity,Encoder_Least,Encoder,Movement;
	  static int Encoder_Integral;
	  //=============ң��ǰ�����˲���=======================//
		Movement=0;	
   //=============�ٶ�PI������======================//	
		Encoder_Least =(Encoder_Left+Encoder_Right)-0;  //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩 
		Encoder *= 0.8;		                             //===һ�׵�ͨ�˲���       
		Encoder += Encoder_Least*0.2;	                 //===һ�׵�ͨ�˲���    
	  if(0==0)         //Ϊ�˷�ֹ����Ӱ���û����飬ֻ�е��������ʱ��ſ�ʼ����
		{
  	Encoder_Integral +=Encoder;                     //===���ֳ�λ�� ����ʱ�䣺5ms
		Encoder_Integral=Encoder_Integral-Movement;     //===����ң�������ݣ�����ǰ������
		}
		if(Encoder_Integral>360000)  	Encoder_Integral=360000;          //===�����޷�
		if(Encoder_Integral<-360000)	Encoder_Integral=-360000;         //===�����޷�	
		Velocity=Encoder*4+Encoder_Integral/140;                        //===�ٶ�PI������	�ٶ�Pϵ����4  Iϵ����1/140
		if(0==1)   Encoder_Integral=0;    //===����رպ��������
	  return Velocity;
}


void TIM1_UP_IRQHandler(void)  
{
	int Turn_Pwm = 0;
	uint16_t Moto1, Moto2;
	
	if(TIM1->SR&0X0001)//5ms��ʱ�ж�
	{
		  TIM1->SR&=~(1<<0);                                       //===�����ʱ��1�жϱ�־λ		 
			readEncoder();                                           //===��ȡ��������ֵ
  		//Led_Flash(400);                                          //===LED��˸;	
  		//Get_battery_volt();                                      //===��ȡ��ص�ѹ	          
			//key(100);                                                //===ɨ�谴��״̬
		  Get_Angle();                                    //===������̬	
 			Balance_Pwm =balance(Angle_Balance,Gyro_Balance);        //===ƽ��PID����	
 			//Velocity_Pwm=velocity(Encoder_Left,Encoder_Right);       //===�ٶȻ�PID����
 	    //Turn_Pwm    =turn(Encoder_Left,Encoder_Right,Gyro_Turn); //===ת��PID����     
 		  //Moto1=Balance_Pwm+Velocity_Pwm-Turn_Pwm;                 //===�������ֵ������PWM
 	  	//Moto2=Balance_Pwm+Velocity_Pwm+Turn_Pwm;                 //===�������ֵ������PWM
   		//Xianfu_Pwm();                                            //===PWM�޷�
      //if(Turn_Off(Angle_Balance,Voltage)==0)                   //===����������쳣
 			//Set_Pwm(Moto1,Moto2);                                    //===��ֵ��PWM�Ĵ���    	
			if(Angle_Balance < 35) {
					Motor_Set(Balance_Pwm,Balance_Pwm);
			}
			else
				Motor_Set(0,0);
			
	}       
} 

