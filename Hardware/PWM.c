#include "stm32f10x.h"                  // Device header
#include "PWM.h"
void TIM1_PWM_Init(u16 ARR,u16 PSC)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_Initstructure;
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_AF_PP  ;
	GPIO_Initstructure.GPIO_Pin = PWMA_OUT_Pin | PWMB_OUT_Pin;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOA, &GPIO_Initstructure);
	
	TIM_InternalClockConfig(TIM1);//选择内部时钟
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = ARR;//ARR	
	TIM_TimeBaseInitStructure.TIM_Prescaler = PSC;//预分频器PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

	TIM_ClearFlag(TIM2,TIM_FLAG_Update);

	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0;                           //CCR的值
	TIM_OCInitStructure.TIM_Pulse = ARR >> 1;     
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM1, &TIM_OCInitStructure); 
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);  
	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);	//MOE 主输出使能	

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH1预装载使能	 
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH4预装载使能	 
	
	TIM_ARRPreloadConfig(TIM1, ENABLE); //使能TIMx在ARR上的预装载寄存器
	
	TIM_Cmd(TIM1, ENABLE);  //使能TIM1
	
	
	
}





void TIM1_SET_PWMA_CCR(uint16_t Compare1)
{
	TIM_SetCompare1(TIM1, Compare1);
}

void TIM1_SET_PWMB_CCR(uint16_t Compare4)
{
	TIM_SetCompare1(TIM1, Compare4);
}

void TIM1_SET_PSC(uint16_t TIM1_Prescaler)
{
	TIM_PrescalerConfig(TIM1, TIM1_Prescaler,TIM1_Prescaler );

}











