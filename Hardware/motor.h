#ifndef __MOTOR_H
#define __MOTOR_H

#define tb6612_AIN1		GPIO_Pin_14
#define tb6612_AIN2		GPIO_Pin_15
#define tb6612_BIN1		GPIO_Pin_13
#define tb6612_BIN2		GPIO_Pin_12

#define PWMA   TIM1->CCR1  //PA8
#define PWMB   TIM1->CCR4  //PA11


void tb6612_AIN1_Set(uint8_t bitvalue);
void tb6612_AIN2_Set(uint8_t bitvalue);
void tb6612_BIN1_Set(uint8_t bitvalue);
void tb6612_BIN2_Set(uint8_t bitvalue);
void Motor_Init(void);
int GFP_abs(int p);
void PWM_Set(int32_t moto_1,int32_t moto_2);
void Turn_Off(float *Med_Angle,float *Angle, float voltage);
void limit_moto_1_2(void);

#endif
