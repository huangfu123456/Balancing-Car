#include "stm32f10x.h"                  // Device header
#include "EXTI_MPU.h"
void EXTI_MPU_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = MPU6050_INT;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_EXTILineConfig( GPIO_PortSourceGPIOB,GPIO_PinSource1);
	
	EXTI_InitTypeDef EXTI_Initstructure;
	EXTI_Initstructure.EXTI_Line = EXTI_Line1 ;
	EXTI_Initstructure.EXTI_LineCmd = ENABLE;
	EXTI_Initstructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_Initstructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&EXTI_Initstructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	/////////////////外部中断5优先级配置也就是MPU6050 INT引脚的配置///////////因为是控制中断，故此优先级应是最高。
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级0，
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//子优先级1
	NVIC_Init(&NVIC_InitStructure);
	
	
	
	
}










