#include "mpu6050.h"

#define MPU6050_PORT_CLOCK	RCC_APB2Periph_GPIOB

#define MPU6050_SDA_GPIO		GPIO_Pin_9
#define MPU6050_SDA_PORT		GPIOB

#define MPU6050_CLK_GPIO		GPIO_Pin_8
#define MPU6050_CLK_PORT		GPIOB

#define MPU6050_CLK_Clr() 	GPIO_ResetBits(MPU6050_CLK_PORT,MPU6050_CLK_GPIO)//SCL
#define MPU6050_CLK_Set() 	GPIO_SetBits  (MPU6050_CLK_PORT,MPU6050_CLK_GPIO)

#define MPU6050_SDIN_Clr()	GPIO_ResetBits(MPU6050_SDA_PORT,MPU6050_SDA_GPIO)//SDA
#define MPU6050_SDIN_Set() 	GPIO_SetBits  (MPU6050_SDA_PORT,MPU6050_SDA_GPIO)

#define READ_MPU6050_SDA		GPIO_ReadInputDataBit(MPU6050_SDA_PORT, MPU6050_SDA_GPIO)


void MPU6050_IIC_Init(void)
{			
	
		GPIO_InitTypeDef GPIO_InitStructer;
    RCC_APB2PeriphClockCmd(MPU6050_PORT_CLOCK, ENABLE);
    GPIO_InitStructer.GPIO_Pin = MPU6050_SDA_GPIO;
    GPIO_InitStructer.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructer.GPIO_Mode = GPIO_Mode_Out_OD;
	
    GPIO_Init(MPU6050_SDA_PORT, &GPIO_InitStructer);
	
	  GPIO_InitStructer.GPIO_Pin = MPU6050_CLK_GPIO;
    GPIO_InitStructer.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructer.GPIO_Mode = GPIO_Mode_Out_PP;

    GPIO_Init(MPU6050_CLK_PORT, &GPIO_InitStructer);
	
		GPIO_SetBits(MPU6050_CLK_PORT, MPU6050_CLK_GPIO);
		GPIO_SetBits(MPU6050_SDA_PORT, MPU6050_SDA_GPIO);

}

int MPU6050_IIC_Start(void)
{
	//SDA_OUT();     //sda线输出
	
	MPU6050_SDIN_Set();
	//IIC_SDA=1;
	if(!READ_MPU6050_SDA)return 0;	
	MPU6050_CLK_Set();
	//IIC_SCL=1;
	//delay_us(1);
	MPU6050_SDIN_Clr();	//START:when CLK is high,DATA change form high to low 
 	//IIC_SDA=0;
	if(READ_MPU6050_SDA)return 0;
	//delay_us(1);
	MPU6050_CLK_Clr();
	//IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
	return 1;
}

/**************************实现函数********************************************
*函数原型:		void IIC_Stop(void)
*功　　能:	    //产生IIC停止信号
*******************************************************************************/	  
void MPU6050_IIC_Stop(void)
{
	MPU6050_CLK_Clr();
	MPU6050_SDIN_Clr(); //STOP:when CLK is high DATA change form low to high
	MPU6050_CLK_Set();
	MPU6050_SDIN_Set();//发送I2C总线结束信号
	//SDA_OUT();//sda线输出
	//IIC_SCL=0;
	//IIC_SDA=0;
 	//delay_us(1);
	//IIC_SCL=1; 
	//IIC_SDA=1;
	//delay_us(1);							   	
}

int MPU6050_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	//SDA_IN();      //SDA设置为输入  
	//IIC_SDA=1;
	//delay_us(1);	   
	//IIC_SCL=1;
	//delay_us(1);	 
	MPU6050_SDIN_Set();
	MPU6050_CLK_Set();
	while(READ_MPU6050_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			MPU6050_IIC_Stop();
			return 0;
		}
	  //delay_us(1);
	}
	MPU6050_CLK_Clr();
	//IIC_SCL=0;//时钟输出0 	   
	return 1;  
} 

void MPU6050_IIC_Ack(void)
{
	/*IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;*/
	MPU6050_CLK_Clr();
	MPU6050_SDIN_Clr();
	MPU6050_CLK_Set();
	MPU6050_CLK_Clr();
	
}

void MPU6050_IIC_NAck(void)
{
	/*IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;*/
	
	MPU6050_CLK_Clr();
	MPU6050_SDIN_Set();
	MPU6050_CLK_Set();
	MPU6050_CLK_Clr();
	
}

void MPU6050_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	  //SDA_OUT(); 	    
    //IIC_SCL=0;//拉低时钟开始数据传输
		MPU6050_CLK_Clr();
    for(t=0;t<8;t++)
    {              
        //IIC_SDA=(txd&0x80)>>7;
				//delay_us(1);   
				//IIC_SCL=1;
				//delay_us(1); 
				//IIC_SCL=0;	
				//delay_us(1);
			
				if((txd&0x80)>>7)
					MPU6050_SDIN_Set();
				else
					MPU6050_SDIN_Clr();
        txd<<=1;
				MPU6050_CLK_Set();
				MPU6050_CLK_Clr();
				
    }	 
} 	 

/**************************实现函数********************************************
*函数原型:		bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
*功　　能:		
*******************************************************************************/
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
		int i;
    if (!MPU6050_IIC_Start())
        return 1;
    MPU6050_IIC_Send_Byte(addr << 1 );
    if (!MPU6050_IIC_Wait_Ack()) {
        MPU6050_IIC_Stop();
        return 1;
    }
    MPU6050_IIC_Send_Byte(reg);
    MPU6050_IIC_Wait_Ack();
		for (i = 0; i < len; i++) {
        MPU6050_IIC_Send_Byte(data[i]);
        if (!MPU6050_IIC_Wait_Ack()) {
            MPU6050_IIC_Stop();
            return 0;
        }
    }
    MPU6050_IIC_Stop();
    return 0;
}

/**************************实现函数********************************************
*函数原型:		u8 IIC_Read_Byte(unsigned char ack)
*功　　能:	    //读1个字节，ack=1时，发送ACK，ack=0，发送nACK 
*******************************************************************************/  
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	//SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        //IIC_SCL=0; 
        //delay_us(2);
				//IIC_SCL=1;
				MPU6050_CLK_Clr();
				MPU6050_CLK_Set();
        receive<<=1;
        if(READ_MPU6050_SDA)receive++;   
				;//delay_us(2); 
    }					 
    if (ack)
        MPU6050_IIC_Ack(); //发送ACK 
    else
        MPU6050_IIC_NAck();//发送nACK  
    return receive;
}


/**************************实现函数********************************************
*函数原型:		bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
*功　　能:		
*******************************************************************************/
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!MPU6050_IIC_Start())
        return 1;
    MPU6050_IIC_Send_Byte(addr << 1);
    if (!MPU6050_IIC_Wait_Ack()) {
        MPU6050_IIC_Stop();
        return 1;
    }
    MPU6050_IIC_Send_Byte(reg);
    MPU6050_IIC_Wait_Ack();
    MPU6050_IIC_Start();
    MPU6050_IIC_Send_Byte((addr << 1)+1);
    MPU6050_IIC_Wait_Ack();
    while (len) {
        if (len == 1)
            *buf = IIC_Read_Byte(0);
        else
            *buf = IIC_Read_Byte(1);
        buf++;
        len--;
    }
    MPU6050_IIC_Stop();
    return 0;
}

/**************************实现函数********************************************
*函数原型:		unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*功　　能:	    读取指定设备 指定寄存器的一个值
输入	I2C_Addr  目标设备地址
		addr	   寄存器地址
返回   读出来的值
*******************************************************************************/ 
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
	unsigned char res=0;
	
	MPU6050_IIC_Start();	
	MPU6050_IIC_Send_Byte(I2C_Addr);	   //发送写命令
	res++;
	MPU6050_IIC_Wait_Ack();
	MPU6050_IIC_Send_Byte(addr); res++;  //发送地址
	MPU6050_IIC_Wait_Ack();	  
	//MPU6050_IIC_Stop();//产生一个停止条件	
	MPU6050_IIC_Start();
	MPU6050_IIC_Send_Byte(I2C_Addr+1); res++;          //进入接收模式			   
	MPU6050_IIC_Wait_Ack();
	res=IIC_Read_Byte(0);	   
    MPU6050_IIC_Stop();//产生一个停止条件

	return res;
}


/**************************实现函数********************************************
*函数原型:		u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*功　　能:	    读取指定设备 指定寄存器的 length个值
输入	dev  目标设备地址
		reg	  寄存器地址
		length 要读的字节数
		*data  读出的数据将要存放的指针
返回   读出来的字节数量
*******************************************************************************/ 
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data){
    u8 count = 0;
	
	MPU6050_IIC_Start();
	MPU6050_IIC_Send_Byte(dev);	   //发送写命令
	MPU6050_IIC_Wait_Ack();
	MPU6050_IIC_Send_Byte(reg);   //发送地址
  MPU6050_IIC_Wait_Ack();	  
	MPU6050_IIC_Start();
	MPU6050_IIC_Send_Byte(dev+1);  //进入接收模式	
	MPU6050_IIC_Wait_Ack();
	
    for(count=0;count<length;count++){
		 
		 if(count!=length-1)data[count]=IIC_Read_Byte(1);  //带ACK的读数据
		 	else  data[count]=IIC_Read_Byte(0);	 //最后一个字节NACK
	}
    MPU6050_IIC_Stop();//产生一个停止条件
    return count;
}

/**************************实现函数********************************************
*函数原型:		u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*功　　能:	    将多个字节写入指定设备 指定寄存器
输入	dev  目标设备地址
		reg	  寄存器地址
		length 要写的字节数
		*data  将要写的数据的首地址
返回   返回是否成功
*******************************************************************************/ 
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data){
  
 	u8 count = 0;
	MPU6050_IIC_Start();
	MPU6050_IIC_Send_Byte(dev);	   //发送写命令
	MPU6050_IIC_Wait_Ack();
	MPU6050_IIC_Send_Byte(reg);   //发送地址
  MPU6050_IIC_Wait_Ack();	  
	for(count=0;count<length;count++){
		MPU6050_IIC_Send_Byte(data[count]); 
		MPU6050_IIC_Wait_Ack(); 
	 }
	MPU6050_IIC_Stop();//产生一个停止条件

    return 1; //status == 0;
}

/**************************实现函数********************************************
*函数原型:		u8 IICreadByte(u8 dev, u8 reg, u8 *data)
*功　　能:	    读取指定设备 指定寄存器的一个值
输入	dev  目标设备地址
		reg	   寄存器地址
		*data  读出的数据将要存放的地址
返回   1
*******************************************************************************/ 
u8 IICreadByte(u8 dev, u8 reg, u8 *data){
	*data=I2C_ReadOneByte(dev, reg);
    return 1;
}

/**************************实现函数********************************************
*函数原型:		unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
*功　　能:	    写入指定设备 指定寄存器一个字节
输入	dev  目标设备地址
		reg	   寄存器地址
		data  将要写入的字节
返回   1
*******************************************************************************/ 
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data){
    return IICwriteBytes(dev, reg, 1, &data);
}

/**************************实现函数********************************************
*函数原型:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的多个位
输入	dev  目标设备地址
		reg	   寄存器地址
		bitStart  目标字节的起始位
		length   位长度
		data    存放改变目标字节位的值
返回   成功 为1 
 		失败为0
*******************************************************************************/ 
u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
{

    u8 b;
    if (IICreadByte(dev, reg, &b) != 0) {
        u8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
    } else {
        return 0;
    }
}

/**************************实现函数********************************************
*函数原型:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的1个位
输入	dev  目标设备地址
		reg	   寄存器地址
		bitNum  要修改目标字节的bitNum位
		data  为0 时，目标位将被清0 否则将被置位
返回   成功 为1 
 		失败为0
*******************************************************************************/ 
u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data){
    u8 b;
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}

void MPU6050_setClockSource(uint8_t source){
    IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);

}


/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_setFullScaleGyroRange(uint8_t range) {
    IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setFullScaleAccelRange(uint8_t range)
*功　　能:	    设置  MPU6050 加速度计的最大量程
*******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
    IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setSleepEnabled(uint8_t enabled)
*功　　能:	    设置  MPU6050 是否进入睡眠模式
				enabled =1   睡觉
			    enabled =0   工作
*******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

void MPU6050_initialize(void) {
	
		MPU6050_IIC_Init();
    MPU6050_setClockSource(MPU6050_CLOCK_PLL_YGYRO); //设置时钟
    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_1000);//陀螺仪最大量程 +-1000度每秒
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);	//加速度度最大量程 +-2G
    MPU6050_setSleepEnabled(0); //进入工作状态
	MPU6050_setI2CMasterModeEnabled(0);	 //不让MPU6050 控制AUXI2C
	MPU6050_setI2CBypassEnabled(0);	 //主控制器的I2C与	MPU6050的AUXI2C	直通。控制器可以直接访问HMC5883L
}

float Read_Temperature(void)
{
	  float Temp;
	  Temp=(I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_L);
		if(Temp>32768) Temp-=65536;
		Temp=(36.53+Temp/340) * 1.0;
	  return Temp;
}

void mpu6050Read_Acc(short *accData)
{
	  accData[0] = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_H) << 8) | (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_XOUT_H));
		accData[1] = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_YOUT_H) << 8) | (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_YOUT_H));
		accData[2] = (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_H) << 8) | (I2C_ReadOneByte(devAddr, MPU6050_RA_ACCEL_ZOUT_H));
}

void mpu6050Read_Gyro(short *gyroData) {
	
		gyroData[0] = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_XOUT_H) << 8) | (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_XOUT_L));
		gyroData[1] = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_H) << 8) | (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_YOUT_L));
		gyroData[2] = (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_H) << 8) | (I2C_ReadOneByte(devAddr, MPU6050_RA_GYRO_ZOUT_L));

}
