#ifndef __PWM_H
#define __PWM_H

#define PWMA_OUT_Pin	GPIO_Pin_8	//GPIOA
#define PWMB_OUT_Pin	GPIO_Pin_11	//GPIOA

void TIM1_PWM_Init(u16 ARR,u16 PSC);
void TIM1_SET_PWMA_CCR(uint16_t Compare1);
void TIM1_SET_PWMB_CCR(uint16_t Compare4);
void TIM1_SET_PSC(uint16_t TIM1_Prescaler);




#endif
