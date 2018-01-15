#include "Encoder.h"

void Encoder_LIFT_Init(void) {
	
		/*GPIO_InitTypeDef GPIO_InitStructure;
	
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		TIM_ICInitTypeDef TIM_ICInitStructure;
	
		//PB6 ch1  A,PB7 ch2 
		
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//使能定时器4时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能GPIO时钟
		
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;         
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//PA6 PA7浮空输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_StructInit(&GPIO_InitStructure);
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	
		TIM_DeInit(TIM4);
	
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 65000-1;  //设定计数器重装载值   TIMx_ARR = 359*4
    TIM_TimeBaseStructure.TIM_Prescaler = 0; //TIM4预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;//设置时钟分割 T_dts = T_ck_int    
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
		
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);//使用编码器模式3，上升沿下降沿都计数
    TIM_ICInitStructure.TIM_ICFilter = 6;  //选择输入比较滤波器
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInit(TIM4, &TIM_ICInitStructure);//初始化TIM4
		
		//Reset counter
		TIM4->CNT = 0;
		
		TIM_Cmd(TIM4, ENABLE);   //启动TIM4定时器
		*/

	/* TIM2 clock source enable */ 
	RCC->APB1ENR|=1<<0;       //TIM2时钟使能
	/* Enable GPIOA, clock */
	RCC->APB2ENR|=1<<2;    //使能PORTA时钟

	/* Configure PA.00,01 as encoder input */
	GPIOA->CRL&=0XFFFFFFF0;//PA0
	GPIOA->CRL|=0X00000004;//浮空输入
	GPIOA->CRL&=0XFFFFFF0F;//PA1
	GPIOA->CRL|=0X00000040;//浮空输入

	/* Enable the TIM2 Update Interrupt */
	//这两个东东要同时设置才可以使用中断
	TIM2->DIER|=1<<0;   //允许更新中断				
	TIM2->DIER|=1<<6;   //允许触发中断

	/* Timer configuration in Encoder mode */ 
	TIM2->PSC = 0x0;//预分频器
	TIM2->ARR = 65000-1;//设定计数器自动重装值 
	TIM2->CR1 &=~(3<<8);// 选择时钟分频：不分频
	TIM2->CR1 &=~(3<<5);// 选择计数模式:边沿对齐模式
		
	TIM2->CCMR1 |= 1<<0; //CC1S='01' IC1FP1映射到TI1
	TIM2->CCMR1 |= 1<<8; //CC2S='01' IC2FP2映射到TI2
	TIM2->CCER &= ~(1<<1);	 //CC1P='0'	 IC1FP1不反相，IC1FP1=TI1
	TIM2->CCER &= ~(1<<5);	 //CC2P='0'	 IC2FP2不反相，IC2FP2=TI2
	TIM2->CCMR1 |= 3<<4; //	IC1F='1000' 输入捕获1滤波器
	TIM2->SMCR |= 3<<0;	 //SMS='011' 所有的输入均在上升沿和下降沿有效
	TIM2->CNT = 0;
	TIM2->CR1 |= 0x01;    //CEN=1，使能定时器

}

void Encoder_RIGHT_Init(void) {
	/* TIM4 clock source enable */ 
	RCC->APB1ENR|=1<<2;       //TIM3时钟使能
	/* Enable GPIOB, clock */
	RCC->APB2ENR|=1<<3;    //使能PORTB时钟

	/* Configure PB.06,07 as encoder input */
	GPIOB->CRL&=0XF0FFFFFF;//PA6
	GPIOB->CRL|=0X08000000;//浮空输入
	GPIOB->CRL&=0X0FFFFFFF;//PA7
	GPIOB->CRL|=0X40000000;//浮空输入

	/* Enable the TIM3 Update Interrupt */
	//这两个东东要同时设置才可以使用中断
	TIM4->DIER|=1<<0;   //允许更新中断				
	TIM4->DIER|=1<<6;   //允许触发中断
	//MY_NVIC_Init(1,3,TIM4_IRQn,1);

	/* Timer configuration in Encoder mode */ 
	TIM4->PSC = 0x0;//预分频器
	TIM4->ARR = 65000-1;//设定计数器自动重装值 
	TIM4->CR1 &=~(3<<8);// 选择时钟分频：不分频
	TIM4->CR1 &=~(3<<5);// 选择计数模式:边沿对齐模式
		
	TIM4->CCMR1 |= 1<<0; //CC1S='01' IC1FP1映射到TI1
	TIM4->CCMR1 |= 1<<8; //CC2S='01' IC2FP2映射到TI2
	TIM4->CCER &= ~(1<<1);	 //CC1P='0'	 IC1FP1不反相，IC1FP1=TI1
	TIM4->CCER &= ~(1<<5);	 //CC2P='0'	 IC2FP2不反相，IC2FP2=TI2
	TIM4->CCMR1 |= 3<<4; //	IC1F='1000' 输入捕获1滤波器
	TIM4->SMCR |= 3<<0;	 //SMS='011' 所有的输入均在上升沿和下降沿有效
	TIM4->CNT = 0;
	TIM4->CR1 |= 0x01;    //CEN=1，使能定时器
}

