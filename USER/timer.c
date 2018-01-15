#include "timer.h"

void Timer1_Init(u16 arr,u16 psc)  
{  
  NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC->APB2ENR|=1<<11;//TIM2时钟使能    
 	TIM1->ARR=arr;  //设定计数器自动重装值//刚好1ms    
	TIM1->PSC=psc;  //预分频器7200,得到10Khz的计数时钟
	TIM1->DIER|=1<<0;   //允许更新中断				
	TIM1->DIER|=1<<6;   //允许触发中断	   
	TIM1->CR1|=0x01;    //使能定时器
	//MY_NVIC_Init(1,3,TIM1_UP_IRQChannel,1);
	
  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  /* 配置P[A|B|C|D|E]0为中断源 */
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}  

void TIM3_GPIO_Config(void) {
	
	//配置TIM3复用输出PWM
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE); 
  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
  //GPIO_Init(GPIOA, &GPIO_InitStructure);

}

void TIM3_Mode_Config(void)
{
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;//初始化TIM3的时间基数
		TIM_OCInitTypeDef  TIM_OCInitStructure;
		
		u16 CCR3_Val = 350;
		u16 CCR4_Val = 350;//pwm信号跳变值(计数到这个数值后都是低电平)
	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能

/* ----------------------------------------------------------------------- 
    TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR+1)* 100% = 50%
    TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR+1)* 100% = 37.5%
    TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR+1)* 100% = 25%
    TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR+1)* 100% = 12.5%
  ----------------------------------------------------------------------- */
  //TIM3的时间技术单位设置(计数终止值:999，从0开始,计数方式:向上计数)
  TIM_TimeBaseStructure.TIM_Period = 3599;       
  TIM_TimeBaseStructure.TIM_Prescaler = 0;            
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;        
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;        //TIM脉冲宽度调制模式1
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
  /*TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;									//设置待装入捕捉比较寄存器的脉冲值
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //TIM输出比较极性高
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
         
  
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;         
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);          
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);*/


  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;        
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);         
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;        
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);        
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM3, ENABLE);

  TIM_Cmd(TIM3, ENABLE);//
}


void MiniBalance_PWM_Init(void) {
	
		TIM3_GPIO_Config();
		TIM3_Mode_Config();
}

void TIM3_IRQHandler(void)   //TIM3中断
{
//			GPIO_SetBits(GPIOB, GPIO_Pin_15);

	  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
		{
		    TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx更新中断标志 
				GPIO_WriteBit(GPIOB, GPIO_Pin_15, 
						 (BitAction)((1-GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_15))));
 
		}
}
