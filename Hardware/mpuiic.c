#include "mpuiic.h"
#include "Delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK精英STM32开发板V3
//MPU6050 IIC驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/1/17
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
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
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能PB端口时钟
  GPIO_InitStructure.GPIO_Pin = STM32_MPU6050_SDA;	//端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //开漏输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
  GPIO_Init(GPIOB, &GPIO_InitStructure);					      //根据设定参数初始化GPIOB 
}

void MPU_SDA_IN(void)
{	GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能PB端口时钟
  GPIO_InitStructure.GPIO_Pin = STM32_MPU6050_SDA;	//端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;      //开漏输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
  GPIO_Init(GPIOB, &GPIO_InitStructure);					      //根据设定参数初始化GPIOB 
 
}


//初始化IIC
void MPU_IIC_Init(void)
{					     
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//先使能外设IO PORTB时钟 
		
	GPIO_InitStructure.GPIO_Pin = STM32_MPU6050_SCL|STM32_MPU6050_SDA;	 // 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIO 
		
	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);					
 
}
//产生IIC起始信号
void MPU_IIC_Start(void)
{
	MPU_SDA_OUT();     //sda线输出
	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(0);//			产生起始信号
	MyI2C_W_SCL(0);
}	  
//产生IIC停止信号
void MPU_IIC_Stop(void)
{
	MPU_SDA_OUT();     //sda线输出
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(1);//			产生停止信号
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 MPU_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	MPU_SDA_IN();      //SDA设置为输入  
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
//产生ACK应答
void MPU_IIC_Ack(void)
{
	MyI2C_W_SCL(0);
	MPU_SDA_OUT();
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(1);
	MyI2C_W_SCL(0);
}
//不产生ACK应答		    
void MPU_IIC_NAck(void)
{
	MyI2C_W_SCL(0);
	MPU_SDA_OUT();
	MyI2C_W_SDA(1);
	
	MyI2C_W_SCL(1);
	
	MyI2C_W_SCL(0);
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void MPU_IIC_Send_Byte(u8 txd)
{                        
     u8 t;   
	MPU_SDA_OUT(); 	    
    MPU_IIC_SCL=0;//拉低时钟开始数据传输
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
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 MPU_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_SDA_IN();//SDA设置为输入
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
        MPU_IIC_NAck();//发送nACK
    else
        MPU_IIC_Ack(); //发送ACK   
    return receive;
}



