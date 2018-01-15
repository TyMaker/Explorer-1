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
	//SDA_OUT();     //sda�����
	
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
	//IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
	return 1;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Stop(void)
*��������:	    //����IICֹͣ�ź�
*******************************************************************************/	  
void MPU6050_IIC_Stop(void)
{
	MPU6050_CLK_Clr();
	MPU6050_SDIN_Clr(); //STOP:when CLK is high DATA change form low to high
	MPU6050_CLK_Set();
	MPU6050_SDIN_Set();//����I2C���߽����ź�
	//SDA_OUT();//sda�����
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
	//SDA_IN();      //SDA����Ϊ����  
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
	//IIC_SCL=0;//ʱ�����0 	   
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
    //IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:		bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
*��������:		
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IIC_Read_Byte(unsigned char ack)
*��������:	    //��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK 
*******************************************************************************/  
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	//SDA_IN();//SDA����Ϊ����
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
        MPU6050_IIC_Ack(); //����ACK 
    else
        MPU6050_IIC_NAck();//����nACK  
    return receive;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
*��������:		
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:		unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*��������:	    ��ȡָ���豸 ָ���Ĵ�����һ��ֵ
����	I2C_Addr  Ŀ���豸��ַ
		addr	   �Ĵ�����ַ
����   ��������ֵ
*******************************************************************************/ 
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
	unsigned char res=0;
	
	MPU6050_IIC_Start();	
	MPU6050_IIC_Send_Byte(I2C_Addr);	   //����д����
	res++;
	MPU6050_IIC_Wait_Ack();
	MPU6050_IIC_Send_Byte(addr); res++;  //���͵�ַ
	MPU6050_IIC_Wait_Ack();	  
	//MPU6050_IIC_Stop();//����һ��ֹͣ����	
	MPU6050_IIC_Start();
	MPU6050_IIC_Send_Byte(I2C_Addr+1); res++;          //�������ģʽ			   
	MPU6050_IIC_Wait_Ack();
	res=IIC_Read_Byte(0);	   
    MPU6050_IIC_Stop();//����һ��ֹͣ����

	return res;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*��������:	    ��ȡָ���豸 ָ���Ĵ����� length��ֵ
����	dev  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫ�����ֽ���
		*data  ���������ݽ�Ҫ��ŵ�ָ��
����   ���������ֽ�����
*******************************************************************************/ 
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data){
    u8 count = 0;
	
	MPU6050_IIC_Start();
	MPU6050_IIC_Send_Byte(dev);	   //����д����
	MPU6050_IIC_Wait_Ack();
	MPU6050_IIC_Send_Byte(reg);   //���͵�ַ
  MPU6050_IIC_Wait_Ack();	  
	MPU6050_IIC_Start();
	MPU6050_IIC_Send_Byte(dev+1);  //�������ģʽ	
	MPU6050_IIC_Wait_Ack();
	
    for(count=0;count<length;count++){
		 
		 if(count!=length-1)data[count]=IIC_Read_Byte(1);  //��ACK�Ķ�����
		 	else  data[count]=IIC_Read_Byte(0);	 //���һ���ֽ�NACK
	}
    MPU6050_IIC_Stop();//����һ��ֹͣ����
    return count;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*��������:	    ������ֽ�д��ָ���豸 ָ���Ĵ���
����	dev  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫд���ֽ���
		*data  ��Ҫд�����ݵ��׵�ַ
����   �����Ƿ�ɹ�
*******************************************************************************/ 
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data){
  
 	u8 count = 0;
	MPU6050_IIC_Start();
	MPU6050_IIC_Send_Byte(dev);	   //����д����
	MPU6050_IIC_Wait_Ack();
	MPU6050_IIC_Send_Byte(reg);   //���͵�ַ
  MPU6050_IIC_Wait_Ack();	  
	for(count=0;count<length;count++){
		MPU6050_IIC_Send_Byte(data[count]); 
		MPU6050_IIC_Wait_Ack(); 
	 }
	MPU6050_IIC_Stop();//����һ��ֹͣ����

    return 1; //status == 0;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICreadByte(u8 dev, u8 reg, u8 *data)
*��������:	    ��ȡָ���豸 ָ���Ĵ�����һ��ֵ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		*data  ���������ݽ�Ҫ��ŵĵ�ַ
����   1
*******************************************************************************/ 
u8 IICreadByte(u8 dev, u8 reg, u8 *data){
	*data=I2C_ReadOneByte(dev, reg);
    return 1;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
*��������:	    д��ָ���豸 ָ���Ĵ���һ���ֽ�
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		data  ��Ҫд����ֽ�
����   1
*******************************************************************************/ 
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data){
    return IICwriteBytes(dev, reg, 1, &data);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �еĶ��λ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		bitStart  Ŀ���ֽڵ���ʼλ
		length   λ����
		data    ��Ÿı�Ŀ���ֽ�λ��ֵ
����   �ɹ� Ϊ1 
 		ʧ��Ϊ0
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �е�1��λ
����	dev  Ŀ���豸��ַ
		reg	   �Ĵ�����ַ
		bitNum  Ҫ�޸�Ŀ���ֽڵ�bitNumλ
		data  Ϊ0 ʱ��Ŀ��λ������0 ���򽫱���λ
����   �ɹ� Ϊ1 
 		ʧ��Ϊ0
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setFullScaleAccelRange(uint8_t range)
*��������:	    ����  MPU6050 ���ٶȼƵ��������
*******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
    IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setSleepEnabled(uint8_t enabled)
*��������:	    ����  MPU6050 �Ƿ����˯��ģʽ
				enabled =1   ˯��
			    enabled =0   ����
*******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
*��������:	    ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
*******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
*��������:	    ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
*******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

void MPU6050_initialize(void) {
	
		MPU6050_IIC_Init();
    MPU6050_setClockSource(MPU6050_CLOCK_PLL_YGYRO); //����ʱ��
    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_1000);//������������� +-1000��ÿ��
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);	//���ٶȶ�������� +-2G
    MPU6050_setSleepEnabled(0); //���빤��״̬
	MPU6050_setI2CMasterModeEnabled(0);	 //����MPU6050 ����AUXI2C
	MPU6050_setI2CBypassEnabled(0);	 //����������I2C��	MPU6050��AUXI2C	ֱͨ������������ֱ�ӷ���HMC5883L
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
