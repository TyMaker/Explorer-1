#include "balance.h"
#include "mpu6050.h"
#include "math.h"
#include "motor.h"


#define PI 3.1415926

int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
int Encoder_Left,Encoder_Right;             //左右编码器的脉冲计数


float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
float K1 =0.02; 
float angle, angle_dot; 	
float Q_angle=0.001;// 过程噪声的协方差
float Q_gyro=0.003;//0.03 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
float R_angle=0.5;// 测量噪声的协方差 既测量偏差
float dt=0.005;//                 
char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };


void Kalman_Filter(float Accel,float Gyro)		
{
	angle+=(Gyro - Q_bias) * dt; //先验估计
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - angle;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle	+= K_0 * Angle_err;	 //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
	angle_dot   = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
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
	  u16 Encoder_L,Encoder_R;       //===左右编码器的脉冲计数
		Encoder_R = TIM4 -> CNT;       //===获取正交解码1数据	
		TIM4 -> CNT=0;                 //===计数器清零  
	  Encoder_L= TIM2 -> CNT;        //===获取正交解码2数据	
	  TIM2 -> CNT=0;	               //===计数器清零
		if(Encoder_L>32768)  Encoder_Left=Encoder_L-65000; else  Encoder_Left=Encoder_L;  
	  //=这个处理的原因是：编码器到0后会跳到65000向下计数，这样处理方便我们在控制程序中使用
	  if(Encoder_R>32768)  Encoder_Right=Encoder_R-65000; else  Encoder_Right=Encoder_R;
	 	Encoder_Left=-Encoder_Left;//这里取反是因为，平衡小车的两个电机是旋转了180度安装的，为了保证前进后退时候的编码器数据符号一致
}

void Get_Angle(void) {
	
		char buf[256];
		uint16_t cc;
		float Accel_Y,Accel_X,Accel_Z,Gyro_Y,Gyro_Z;
	
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

}

int balance(float Angle,float Gyro)
{  
   float Bias;
	 int balance;
	 Bias=Angle-2.0;              //===求出平衡的角度中值 和机械相关 -0意味着身重中心在0度附近 如果身重中心在5度附近 那就应该减去5
	 balance=235*Bias+Gyro*0.125;//===计算平衡控制的电机PWM  PD控制   35是P系数 0.125是D系数 
	 return balance;
}


int velocity(int encoder_left,int encoder_right)
{  
	  static int Velocity,Encoder_Least,Encoder,Movement;
	  static int Encoder_Integral;
	  //=============遥控前进后退部分=======================//
		Movement=0;	
   //=============速度PI控制器======================//	
		Encoder_Least =(Encoder_Left+Encoder_Right)-0;  //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此次为零） 
		Encoder *= 0.8;		                             //===一阶低通滤波器       
		Encoder += Encoder_Least*0.2;	                 //===一阶低通滤波器    
	  if(0==0)         //为了防止积分影响用户体验，只有电机开启的时候才开始积分
		{
  	Encoder_Integral +=Encoder;                     //===积分出位移 积分时间：5ms
		Encoder_Integral=Encoder_Integral-Movement;     //===接收遥控器数据，控制前进后退
		}
		if(Encoder_Integral>360000)  	Encoder_Integral=360000;          //===积分限幅
		if(Encoder_Integral<-360000)	Encoder_Integral=-360000;         //===积分限幅	
		Velocity=Encoder*4+Encoder_Integral/140;                        //===速度PI控制器	速度P系数是4  I系数是1/140
		if(0==1)   Encoder_Integral=0;    //===电机关闭后清除积分
	  return Velocity;
}


void TIM1_UP_IRQHandler(void)  
{
	int Turn_Pwm = 0;
	uint16_t Moto1, Moto2;
	
	if(TIM1->SR&0X0001)//5ms定时中断
	{
		  TIM1->SR&=~(1<<0);                                       //===清除定时器1中断标志位		 
			readEncoder();                                           //===读取编码器的值
  		//Led_Flash(400);                                          //===LED闪烁;	
  		//Get_battery_volt();                                      //===获取电池电压	          
			//key(100);                                                //===扫描按键状态
		  Get_Angle();                                    //===更新姿态	
 			Balance_Pwm =balance(Angle_Balance,Gyro_Balance);        //===平衡PID控制	
 			//Velocity_Pwm=velocity(Encoder_Left,Encoder_Right);       //===速度环PID控制
 	    //Turn_Pwm    =turn(Encoder_Left,Encoder_Right,Gyro_Turn); //===转向环PID控制     
 		  //Moto1=Balance_Pwm+Velocity_Pwm-Turn_Pwm;                 //===计算左轮电机最终PWM
 	  	//Moto2=Balance_Pwm+Velocity_Pwm+Turn_Pwm;                 //===计算右轮电机最终PWM
   		//Xianfu_Pwm();                                            //===PWM限幅
      //if(Turn_Off(Angle_Balance,Voltage)==0)                   //===如果不存在异常
 			//Set_Pwm(Moto1,Moto2);                                    //===赋值给PWM寄存器    	
			if(Angle_Balance < 35) {
					Motor_Set(Balance_Pwm,Balance_Pwm);
			}
			else
				Motor_Set(0,0);
			
	}       
} 

