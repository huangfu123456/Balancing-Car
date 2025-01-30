#include "stm32f10x.h"                  // Device header
#include "motor.h"
#include "config.h"
void tb6612_AIN1_Set(uint8_t bitvalue)
{

	GPIO_WriteBit(GPIOB, tb6612_AIN1, (BitAction) bitvalue);
}

void tb6612_AIN2_Set(uint8_t bitvalue)
{

	GPIO_WriteBit(GPIOB, tb6612_AIN2, (BitAction) bitvalue);
}

void tb6612_BIN1_Set(uint8_t bitvalue)
{

	GPIO_WriteBit(GPIOB, tb6612_BIN1, (BitAction) bitvalue);
}

void tb6612_BIN2_Set(uint8_t bitvalue)
{

	GPIO_WriteBit(GPIOB, tb6612_BIN2, (BitAction) bitvalue);
}

void Motor_Init(void)
{
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitTypeDef GPIO_Initstructure;
	GPIO_Initstructure.GPIO_Mode = GPIO_Mode_Out_PP ;
	GPIO_Initstructure.GPIO_Pin = tb6612_AIN1 | tb6612_AIN2 | tb6612_BIN1 |tb6612_BIN2;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOB, &GPIO_Initstructure);
	tb6612_AIN1_Set(0);
	tb6612_AIN2_Set(0);
	tb6612_AIN2_Set(0);
	tb6612_BIN2_Set(0);
	
}

void PWM_Set(int32_t moto_1,int32_t moto_2)
{	
	if(moto_1 < 0)
	{
	tb6612_AIN1_Set(1);
	tb6612_AIN2_Set(0);
	} else
	{
	tb6612_AIN1_Set(0);
	tb6612_AIN2_Set(1);
	
	}
	int temp1 = GFP_abs(moto_1);
	TIM_SetCompare1(TIM1,temp1);	//控制左轮
	
	
	if(moto_2 < 0)
	{
	tb6612_BIN1_Set(1);
	tb6612_BIN2_Set(0);
	} else
	{
	tb6612_BIN1_Set(0);
	tb6612_BIN2_Set(1);
	
	}
	int temp2 = GFP_abs(moto_2);
	TIM_SetCompare4(TIM1,temp2);		//控制右轮
}


/*绝对值函数*/
int GFP_abs(int p)
{
	int q;
	q=p>0?p:(-p);
	return q;
}


void limit_moto_1_2(void)
{
	 //===PWM满幅是7200 限制在7000
	if(moto_1<PWM_MIN )  moto_1=PWM_MIN ;
	if(moto_1>PWM_MAX )  moto_1=PWM_MAX ;
	if(moto_2<PWM_MIN )  moto_2=PWM_MIN ;
	if(moto_2>PWM_MAX )  moto_2=PWM_MAX ;
}

void Turn_Off(float *Med_Angle,float *Angle, float voltage)
{
		if( GFP_abs(*Med_Angle - *Angle)> 60 || voltage<11.1 )	 //电池电压低于11.1V关闭电机
		{	                                   //===倾角大于40度关闭电机																			 
			PWM_Set(0,0);	
			stop=1;
			
		}		
}
	


