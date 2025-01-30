#include "mpuiic.h"
#include "Delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK��ӢSTM32������V3
//MPU6050 IIC���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2015/1/17
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
 void MPU_IIC_Delay(void)
{
	delay_us(2);
}
 void MyI2C_W_SCL(uint8_t bitvalue)
{
	GPIO_WriteBit(GPIOB,STM32_MPU6050_SCL,(BitAction)bitvalue);
	delay_us(3);
}

void MyI2C_W_SDA(uint8_t bitvalue)
{
	GPIO_WriteBit(GPIOB,STM32_MPU6050_SDA,(BitAction)bitvalue);
	delay_us(3);
}

uint8_t MyI2C_R_SDA(void)
{
	uint8_t bitvalue;
	bitvalue = GPIO_ReadInputDataBit(GPIOB,STM32_MPU6050_SDA);
	delay_us(3);
	return bitvalue;
}

 
 void MPU_SDA_OUT(void)
{	GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ��PB�˿�ʱ��
  GPIO_InitStructure.GPIO_Pin = STM32_MPU6050_SDA;	//�˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //��©���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
  GPIO_Init(GPIOB, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOB 
}

void MPU_SDA_IN(void)
{	GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ��PB�˿�ʱ��
  GPIO_InitStructure.GPIO_Pin = STM32_MPU6050_SDA;	//�˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;      //��©���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
  GPIO_Init(GPIOB, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOB 
 
}


//��ʼ��IIC
void MPU_IIC_Init(void)
{					     
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//��ʹ������IO PORTBʱ�� 
		
	GPIO_InitStructure.GPIO_Pin = STM32_MPU6050_SCL|STM32_MPU6050_SDA;	 // �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIO 
		
	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);					
 
}
//����IIC��ʼ�ź�
void MPU_IIC_Start(void)
{
	MPU_SDA_OUT();     //sda�����
	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(0);//			������ʼ�ź�
	MyI2C_W_SCL(0);
}	  
//����IICֹͣ�ź�
void MPU_IIC_Stop(void)
{
	MPU_SDA_OUT();     //sda�����
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(1);//			����ֹͣ�ź�
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 MPU_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	MPU_SDA_IN();      //SDA����Ϊ����  
	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);
	while(MyI2C_R_SDA())
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			MPU_IIC_Stop();
			return 1;
		}
	}
	MyI2C_W_SCL(0);
	return 0;  
} 
//����ACKӦ��
void MPU_IIC_Ack(void)
{
	MyI2C_W_SCL(0);
	MPU_SDA_OUT();
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(1);
	MyI2C_W_SCL(0);
}
//������ACKӦ��		    
void MPU_IIC_NAck(void)
{
	MyI2C_W_SCL(0);
	MPU_SDA_OUT();
	MyI2C_W_SDA(1);
	
	MyI2C_W_SCL(1);
	
	MyI2C_W_SCL(0);
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void MPU_IIC_Send_Byte(u8 txd)
{                        
     u8 t;   
	MPU_SDA_OUT(); 	    
    MPU_IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        MPU_IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		    MPU_IIC_SCL=1;
		    MPU_IIC_Delay(); 
		    MPU_IIC_SCL=0;	
		    MPU_IIC_Delay();
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 MPU_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        MPU_IIC_SCL=0; 
        MPU_IIC_Delay();
		MPU_IIC_SCL=1;
        receive<<=1;
        if(MPU_READ_SDA)receive++;   
		MPU_IIC_Delay(); 
    }					 
    if (!ack)
        MPU_IIC_NAck();//����nACK
    else
        MPU_IIC_Ack(); //����ACK   
    return receive;
}



