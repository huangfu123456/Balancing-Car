#include "stm32f10x.h"                  // Device header
#include "encoder.h"


void Encoder_TIM2_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	GPIO_InitTypeDef GPIO_Initstructure;
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_IN_FLOATING  ;
	GPIO_Initstructure.GPIO_Pin = TIM2_ch1 | TIM2_ch2;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOA, &GPIO_Initstructure);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65536-1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1-1;//预分频器PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

	TIM_ClearFlag(TIM2,TIM_FLAG_Update);
	
	TIM_ICInitTypeDef	TIM_ICInitStructure;
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 0xA;
	TIM_ICInit(TIM2,&TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter = 0xA;
	TIM_ICInit(TIM2,&TIM_ICInitStructure);

	TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Falling,TIM_ICPolarity_Falling);

	TIM_ClearFlag(TIM2, TIM_FLAG_Update);//清除TIM的更新标志位
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_SetCounter(TIM2,0);

	TIM_Cmd(TIM2, ENABLE);


}

void Encoder_TIM3_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	GPIO_InitTypeDef GPIO_Initstructure;
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_IN_FLOATING  ;
	GPIO_Initstructure.GPIO_Pin = TIM3_ch1 | TIM3_ch2;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOA, &GPIO_Initstructure);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65536-1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1-1;//预分频器PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

	TIM_ClearFlag(TIM3,TIM_FLAG_Update);
	
	TIM_ICInitTypeDef	TIM_ICInitStructure;
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 0xA;
	TIM_ICInit(TIM3,&TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICFilter = 0xA;
	TIM_ICInit(TIM3,&TIM_ICInitStructure);

	TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_Falling,TIM_ICPolarity_Falling);

	TIM_ClearFlag(TIM3, TIM_FLAG_Update);//清除TIM的更新标志位
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_SetCounter(TIM3,0);

	TIM_Cmd(TIM3, ENABLE);


}

int Read_Encoder(uint8_t TIMX)
{
    int Encoder_TIM;    
	switch(TIMX)
	{
	   case 2:  
		 Encoder_TIM= (short)TIM2 -> CNT; 
		 TIM2 -> CNT=0;
		 break;
	   case 3:  
		 Encoder_TIM= (short)TIM3 -> CNT;  
		TIM3 -> CNT=0;
	     break;	
	   default: Encoder_TIM=0;
	}
	
	return Encoder_TIM;
}

