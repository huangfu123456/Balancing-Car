#include "stm32f10x.h"                  // Device header
#include "mpu6050.h"
#include "delay.h"
#include "usart.h"   
#include "mpu6050.h"
#include "mpuiic.h"
#include "MPU6050_Reg.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK精英STM32开发板V3
//MPU6050 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/1/17
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
 u8 MPU6050_WriteReg(uint8_t RegAddress,uint8_t Data)
{
	MPU_IIC_Start();
	MPU_IIC_Send_Byte(MPU6050_ADDRESS);
	MPU_IIC_Wait_Ack();
	MPU_IIC_Send_Byte(RegAddress);
	MPU_IIC_Wait_Ack();
	MPU_IIC_Send_Byte(Data);
	MPU_IIC_Wait_Ack();
	MPU_IIC_Stop();
	return 0;
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{	
	uint8_t Data = 0x00;
	
	MPU_IIC_Start();
	MPU_IIC_Send_Byte(MPU6050_ADDRESS);
	MPU_IIC_Wait_Ack();
	MPU_IIC_Send_Byte(RegAddress);
		MPU_IIC_Wait_Ack();
	
	MPU_IIC_Start();
	MPU_IIC_Send_Byte(MPU6050_ADDRESS | 0x01);
	MPU_IIC_Wait_Ack();
	Data = MPU_IIC_Read_Byte(0);
	MPU_IIC_Stop();
	return Data;
	
}
//初始化MPU6050
//返回值:0,成功
//    其他,错误代码
u8 MPU_Init(void)
{ 
	u8 res; 
	MPU_IIC_Init();
	MPU_Write_Byte(MPU6050_PWR_MGMT_1,0X80);	//复位MPU6050
	delay_ms(100);
	MPU_Write_Byte(MPU6050_PWR_MGMT_1,0X00);	//唤醒MPU6050 
//	MPU6050_WriteReg(MPU6050_PWR_MGMT_1,0x80);//解除睡眠，选择x轴陀螺仪时钟
//	MPU6050_WriteReg(MPU6050_PWR_MGMT_2,0x00);//6个轴都不待机
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV,0x04);//5分频,采样频率200Hz
	MPU6050_WriteReg(MPU6050_CONFIG,0x03);//设置数字低通滤波器DLPF
	MPU6050_WriteReg(MPU6050_INT_ENABLE,0x00);//关闭所有中断
	MPU6050_WriteReg(MPU6050_USER_CTRL,0x00);//I2C主模式关闭
	MPU6050_WriteReg(MPU6050_FIFO_EN,0x00);//关闭FIFO
	MPU6050_WriteReg(MPU6050_INT_PIN_CFG,0x80);		//INT引脚低电平有效
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG,0x18);//陀螺仪量程最大  	± 2000 °/s
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG,0x00);//加速度计量程最大	± 2g
	res=MPU6050_ReadReg(MPU6050_WHO_AM_I);
	if(res==MPU_ADDR)//器件ID正确
	{
		MPU6050_WriteReg(MPU6050_PWR_MGMT_1,0X01);	//设置CLKSEL,PLL X轴为参考
		MPU6050_WriteReg(MPU6050_PWR_MGMT_2,0X00);	//加速度与陀螺仪都工作
		MPU6050_WriteReg(MPU6050_SMPLRT_DIV,0x04);					//设置采样率为200Hz
 	}else return 1;
	
	return 0;
}
//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器  
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

void MPU6050_GetData(MPU6050_Get_Data *Data){

	uint8_t DataH,DataL;
	DataH =	MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
	Data->AccX = (DataH<<8) | DataL;

	DataH =	MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
	Data->AccY = (DataH<<8) | DataL;

	DataH =	MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
	Data->AccZ = (DataH<<8) | DataL;
	
	DataH =	MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
	Data->GyroX = (DataH<<8) | DataL;
	
	DataH =	MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
	Data->GyroY = (DataH<<8) | DataL;
	
	DataH =	MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
	Data->GyroZ = (DataH<<8) | DataL;
	
	DataH =	MPU6050_ReadReg(MPU6050_TEMP_OUT_H);
	DataL = MPU6050_ReadReg(MPU6050_TEMP_OUT_L);
	Data->Temperature_REG = (DataH<<8) | DataL;
	
	Data->Temperature_Real = 36.53+((double)Data->Temperature_REG)/340;
}


uint8_t MPU6050_GetID(void)
{

	return MPU6050_ReadReg(MPU6050_WHO_AM_I);

}

//得到温度值
//返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(MPU_IIC_Wait_Ack())	//等待应答
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答
	for(i=0;i<len;i++)
	{
		MPU_IIC_Send_Byte(buf[i]);	//发送数据
		if(MPU_IIC_Wait_Ack())		//等待ACK
		{
			MPU_IIC_Stop();	 
			return 1;		 
		}		
	}    
    MPU_IIC_Stop();	 
	return 0;	
} 
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(MPU_IIC_Wait_Ack())	//等待应答
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答
    MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令	
    MPU_IIC_Wait_Ack();		//等待应答 
	while(len)
	{
		if(len==1)*buf=MPU_IIC_Read_Byte(0);//读数据,发送nACK 
		else *buf=MPU_IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++; 
	}    
    MPU_IIC_Stop();	//产生一个停止条件 
	return 0;	
}
//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Byte(u8 reg,u8 data) 				 
{ 
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	if(MPU_IIC_Wait_Ack())	//等待应答
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答 
	MPU_IIC_Send_Byte(data);//发送数据
	if(MPU_IIC_Wait_Ack())	//等待ACK
	{
		MPU_IIC_Stop();	 
		return 1;		 
	}		 
    MPU_IIC_Stop();	 
	return 0;
}
//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	MPU_IIC_Wait_Ack();		//等待应答 
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答
    MPU_IIC_Start();
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|1);//发送器件地址+读命令	
    MPU_IIC_Wait_Ack();		//等待应答 
	res=MPU_IIC_Read_Byte(0);//读取数据,发送nACK 
    MPU_IIC_Stop();			//产生一个停止条件 
	return res;		
}



