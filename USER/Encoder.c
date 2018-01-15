#include "Encoder.h"

void Encoder_LIFT_Init(void) {
	
		/*GPIO_InitTypeDef GPIO_InitStructure;
	
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		TIM_ICInitTypeDef TIM_ICInitStructure;
	
		//PB6 ch1  A,PB7 ch2 
		
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//ʹ�ܶ�ʱ��4ʱ��
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//ʹ��GPIOʱ��
		
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;         
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//PA6 PA7��������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_StructInit(&GPIO_InitStructure);
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	
		TIM_DeInit(TIM4);
	
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 65000-1;  //�趨��������װ��ֵ   TIMx_ARR = 359*4
    TIM_TimeBaseStructure.TIM_Prescaler = 0; //TIM4Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;//����ʱ�ӷָ� T_dts = T_ck_int    
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
		
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);//ʹ�ñ�����ģʽ3���������½��ض�����
    TIM_ICInitStructure.TIM_ICFilter = 6;  //ѡ������Ƚ��˲���
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInit(TIM4, &TIM_ICInitStructure);//��ʼ��TIM4
		
		//Reset counter
		TIM4->CNT = 0;
		
		TIM_Cmd(TIM4, ENABLE);   //����TIM4��ʱ��
		*/

	/* TIM2 clock source enable */ 
	RCC->APB1ENR|=1<<0;       //TIM2ʱ��ʹ��
	/* Enable GPIOA, clock */
	RCC->APB2ENR|=1<<2;    //ʹ��PORTAʱ��

	/* Configure PA.00,01 as encoder input */
	GPIOA->CRL&=0XFFFFFFF0;//PA0
	GPIOA->CRL|=0X00000004;//��������
	GPIOA->CRL&=0XFFFFFF0F;//PA1
	GPIOA->CRL|=0X00000040;//��������

	/* Enable the TIM2 Update Interrupt */
	//����������Ҫͬʱ���òſ���ʹ���ж�
	TIM2->DIER|=1<<0;   //��������ж�				
	TIM2->DIER|=1<<6;   //�������ж�

	/* Timer configuration in Encoder mode */ 
	TIM2->PSC = 0x0;//Ԥ��Ƶ��
	TIM2->ARR = 65000-1;//�趨�������Զ���װֵ 
	TIM2->CR1 &=~(3<<8);// ѡ��ʱ�ӷ�Ƶ������Ƶ
	TIM2->CR1 &=~(3<<5);// ѡ�����ģʽ:���ض���ģʽ
		
	TIM2->CCMR1 |= 1<<0; //CC1S='01' IC1FP1ӳ�䵽TI1
	TIM2->CCMR1 |= 1<<8; //CC2S='01' IC2FP2ӳ�䵽TI2
	TIM2->CCER &= ~(1<<1);	 //CC1P='0'	 IC1FP1�����࣬IC1FP1=TI1
	TIM2->CCER &= ~(1<<5);	 //CC2P='0'	 IC2FP2�����࣬IC2FP2=TI2
	TIM2->CCMR1 |= 3<<4; //	IC1F='1000' ���벶��1�˲���
	TIM2->SMCR |= 3<<0;	 //SMS='011' ���е�������������غ��½�����Ч
	TIM2->CNT = 0;
	TIM2->CR1 |= 0x01;    //CEN=1��ʹ�ܶ�ʱ��

}

void Encoder_RIGHT_Init(void) {
	/* TIM4 clock source enable */ 
	RCC->APB1ENR|=1<<2;       //TIM3ʱ��ʹ��
	/* Enable GPIOB, clock */
	RCC->APB2ENR|=1<<3;    //ʹ��PORTBʱ��

	/* Configure PB.06,07 as encoder input */
	GPIOB->CRL&=0XF0FFFFFF;//PA6
	GPIOB->CRL|=0X08000000;//��������
	GPIOB->CRL&=0X0FFFFFFF;//PA7
	GPIOB->CRL|=0X40000000;//��������

	/* Enable the TIM3 Update Interrupt */
	//����������Ҫͬʱ���òſ���ʹ���ж�
	TIM4->DIER|=1<<0;   //��������ж�				
	TIM4->DIER|=1<<6;   //�������ж�
	//MY_NVIC_Init(1,3,TIM4_IRQn,1);

	/* Timer configuration in Encoder mode */ 
	TIM4->PSC = 0x0;//Ԥ��Ƶ��
	TIM4->ARR = 65000-1;//�趨�������Զ���װֵ 
	TIM4->CR1 &=~(3<<8);// ѡ��ʱ�ӷ�Ƶ������Ƶ
	TIM4->CR1 &=~(3<<5);// ѡ�����ģʽ:���ض���ģʽ
		
	TIM4->CCMR1 |= 1<<0; //CC1S='01' IC1FP1ӳ�䵽TI1
	TIM4->CCMR1 |= 1<<8; //CC2S='01' IC2FP2ӳ�䵽TI2
	TIM4->CCER &= ~(1<<1);	 //CC1P='0'	 IC1FP1�����࣬IC1FP1=TI1
	TIM4->CCER &= ~(1<<5);	 //CC2P='0'	 IC2FP2�����࣬IC2FP2=TI2
	TIM4->CCMR1 |= 3<<4; //	IC1F='1000' ���벶��1�˲���
	TIM4->SMCR |= 3<<0;	 //SMS='011' ���е�������������غ��½�����Ч
	TIM4->CNT = 0;
	TIM4->CR1 |= 0x01;    //CEN=1��ʹ�ܶ�ʱ��
}

