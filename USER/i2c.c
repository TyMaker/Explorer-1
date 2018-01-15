#include "i2c.h"
#include "delay.h"


void IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructer;
    RCC_APB2PeriphClockCmd(SSD1306_PORT_CLOCK, ENABLE);
    GPIO_InitStructer.GPIO_Pin = SSD1306_SDA_GPIO; //10--SCL   11--SDA
    GPIO_InitStructer.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructer.GPIO_Mode = GPIO_Mode_Out_PP;
	
    GPIO_Init(SSD1306_SDA_PORT, &GPIO_InitStructer);
	
	  GPIO_InitStructer.GPIO_Pin = SSD1306_CLK_GPIO;
    GPIO_InitStructer.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructer.GPIO_Mode = GPIO_Mode_Out_PP;

    GPIO_Init(SSD1306_CLK_PORT, &GPIO_InitStructer);
	
		GPIO_ResetBits(SSD1306_SDA_PORT, SSD1306_SDA_GPIO | SSD1306_CLK_GPIO);

}

void SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStructer;
		//GPIO_SetBits(SSD1306_CLK_PORT, SSD1306_CLK_GPIO | SSD1306_SDA_GPIO);
    GPIO_InitStructer.GPIO_Pin= SSD1306_SDA_GPIO;
    GPIO_InitStructer.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_InitStructer.GPIO_Mode=GPIO_Mode_Out_PP;
    GPIO_Init(SSD1306_SDA_PORT, &GPIO_InitStructer);
}


void SDA_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStructer;
    GPIO_InitStructer.GPIO_Pin= SSD1306_SDA_GPIO;
    GPIO_InitStructer.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_InitStructer.GPIO_Mode=GPIO_Mode_IPU;
    GPIO_Init(SSD1306_SDA_PORT, &GPIO_InitStructer);
}

void IIC_Start(void)
{

	OLED_SCLK_Set();
	OLED_SDIN_Set();
	OLED_SDIN_Clr();
	OLED_SCLK_Clr();
}

void IIC_Stop(void)
{
	OLED_SCLK_Set() ;
	OLED_SDIN_Clr();
	OLED_SDIN_Set();
}

void IIC_Wait_Ask(void)
{
		OLED_SCLK_Set();
		OLED_SCLK_Clr();
}

void IIC_WriteByte(u8 data)
{
	unsigned char i;
	unsigned char m,da;
	da=data;
	OLED_SCLK_Clr();
	for(i=0;i<8;i++)		
	{
			m=da;
		//	OLED_SCLK_Clr();
		m=m&0x80;
		if(m==0x80)
		{OLED_SDIN_Set();}
		else OLED_SDIN_Clr();
			da=da<<1;
		OLED_SCLK_Set();
		OLED_SCLK_Clr();
		}
}

uint8_t IIC_ReadByte(void)
{
    u8 data,i;
    GPIO_SetBits(SSD1306_SDA_PORT, SSD1306_SDA_GPIO);
		//IIC_SDA=1;
    delay_us(2);
    for(i=0;i<8;i++)
    {
        data<<=1;
        GPIO_ResetBits(SSD1306_CLK_PORT, SSD1306_CLK_GPIO);
				//IIC_SCL=0;
        delay_us(2);
				GPIO_SetBits(SSD1306_CLK_PORT, SSD1306_CLK_GPIO);
        //IIC_SCL=1;
        delay_us(2);
        if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7))
            data=data | 0x01;
        else 
            data=data & 0xFE;

    }
		GPIO_ResetBits(SSD1306_CLK_PORT, SSD1306_CLK_GPIO);
    //IIC_SCL=0;
    delay_us(2);
    return data;

}

void WriteCmd(uint8_t i2caddr, uint8_t control, uint8_t command)
{
    IIC_Start();
    IIC_WriteByte(i2caddr);//OLED µØÖ·
    IIC_Wait_Ask();
    IIC_WriteByte(control);//¼Ä´æÆ÷µØÖ·
    IIC_Wait_Ask();
    IIC_WriteByte(command);
    IIC_Wait_Ask();
    IIC_Stop();
}


void WriteDat(uint8_t i2caddr, uint8_t control, uint8_t data)
{
    IIC_Start();
    IIC_WriteByte(i2caddr);//OLEDµØÖ·
    IIC_Wait_Ask();
    IIC_WriteByte(control);//¼Ä´æÆ÷µØÖ·
    IIC_Wait_Ask();
    IIC_WriteByte(data);
    IIC_Wait_Ask();
    IIC_Stop();
}



