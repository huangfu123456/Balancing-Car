#include "stm32f10x.h"                  // Device header
#include "config.h"


void SR04_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitTypeDef GPIO_Initstructure;
	GPIO_Initstructure.GPIO_Mode= GPIO_Mode_Out_PP;
	GPIO_Initstructure.GPIO_Pin= SR04_Pin_Trig ;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_Initstructure);
	
	GPIO_Initstructure.GPIO_Mode= GPIO_Mode_IPD;
	GPIO_Initstructure.GPIO_Pin= SR04_Pin_Echo;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_Initstructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource8);
	
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line8;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger  = EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel=EXTI9_5_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=2;
	NVIC_Init(&NVIC_InitStruct);
	
	
	TIM4_Init();
	

}

void TIM4_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	TIM_InternalClockConfig(TIM4);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period  = 65536-1;			//ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 7200-1;			//PSC
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure );
	
	
	TIM_SetCounter(TIM4, 0);
	
	TIM_Cmd(TIM4,DISABLE);
}

void SR04_Start_Measure(void)
{	
	GPIO_SetBits(GPIOB, SR04_Pin_Trig);
	delay_us(10);
	GPIO_ResetBits(GPIOB, SR04_Pin_Trig);

}


uint8_t SR04_EXTI_Sign = 0;
uint16_t SR04_count;

void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line8) == SET)
	{
//			TIM_SetCounter(TIM4,0);																		
//			TIM_Cmd(TIM4, ENABLE);                          //开启时钟
//			while(GPIO_ReadInputDataBit(GPIOB,SR04_Pin_Echo));	//等待低电平
//			TIM_Cmd(TIM4, DISABLE);			                //定时器4失能
//			SR04_length = 1.0*SR04_count*1.7;		//距离单位cm
		
		
		if(SR04_EXTI_Sign == 0)
		{
			TIM_SetCounter(TIM4, 0);
			TIM_Cmd(TIM4,ENABLE);
			
			EXTI_InitTypeDef EXTI_InitStructure;
			EXTI_InitStructure.EXTI_Line = EXTI_Line8;
			EXTI_InitStructure.EXTI_LineCmd = ENABLE;
			EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
			EXTI_InitStructure.EXTI_Trigger  = EXTI_Trigger_Falling;
			EXTI_Init(&EXTI_InitStructure);
			
			SR04_EXTI_Sign = 1;
			
		}
		else if(SR04_EXTI_Sign == 1)
		{	
			
			TIM_Cmd(TIM4,DISABLE);
			SR04_count = TIM_GetCounter(TIM4);
			SR04_length = 1.0*SR04_count*1.7;		//距离单位cm

			EXTI_InitTypeDef EXTI_InitStructure;
			EXTI_InitStructure.EXTI_Line = EXTI_Line8;
			EXTI_InitStructure.EXTI_LineCmd = ENABLE;
			EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
			EXTI_InitStructure.EXTI_Trigger  = EXTI_Trigger_Rising;
			EXTI_Init(&EXTI_InitStructure);
			
			SR04_EXTI_Sign = 0;
			
		}
		
		EXTI_ClearITPendingBit( EXTI_Line8);
	}

}
















