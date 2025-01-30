#ifndef __USART_BLUE_H
#define __USART_BLUE_H

#define stm32_Usart2_TX		GPIO_Pin_2				//PA2
#define stm32_Usart2_RX		GPIO_Pin_3				//PA3

void Usart3_Blue_Init(u32 bound);
void Serial_SendByte(uint8_t Byte);
void  Serial_SendArray(uint8_t*Array,uint16_t Length);
void  Serial_SendString(char* string);
uint32_t Serial_pow(uint32_t X,uint32_t Y);
void Serial_SendNumber(uint32_t Number,uint8_t Length);
uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(void);

#endif 

