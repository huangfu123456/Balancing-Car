#ifndef __ENCODER_H
#define __ENCODER_H

#define TIM2_ch1		GPIO_Pin_0		//PA0
#define TIM2_ch2		GPIO_Pin_1		//PA1

#define TIM3_ch1		GPIO_Pin_6		//PA6
#define TIM3_ch2		GPIO_Pin_7		//PA7




void Encoder_TIM2_Init(void);
void Encoder_TIM3_Init(void);
int Read_Encoder(uint8_t TIMX);




#endif

