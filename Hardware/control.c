#include "stm32f10x.h"                  // Device header
#include "encoder.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "config.h"


	
int Velocity_out,Vertical_out,Turn_out;
	



int Vertical(float Med,float Angle,float gyro)//直立环控制
{
	int temp;
	temp = 1.0*Vertical_Kp/100*(Angle-Med) + 1.0*Vertical_Kd/100*gyro;
	
	return temp;

}

int Velocity(int Target,int encoder_L,int encoder_R)//速度环控制
{
	static int Error_Low_out_last,encoder_S;
	static float a = 0.7;
	int Err,Error_Low_out,temp;
//	Velocity_Ki = Velocity_Kp/200;
	Err = (encoder_L + encoder_L) - Target;
	Error_Low_out = (1-a)*Err + a*Error_Low_out_last;	//低通滤波
			
	encoder_S += Error_Low_out;
	Error_Low_out_last = Error_Low_out;		

	if(stop==1)
	{
		encoder_S=0,stop=0;//清零积分量
	}
	
	if(encoder_S > 20000 | encoder_S < -20000)			//积分限幅
	{
		if(encoder_S > 20000)
		{
			encoder_S = 20000;
		} else
		{
			encoder_S = -20000;
		}
	
	} 
	
	temp = 1.0*Velocity_Kp/100*Error_Low_out + 1.0*Velocity_Ki/1000*encoder_S;
	
	return temp;

}

int Turn(float gyroZ,int Target_turn)		//转向环控制
{
	int temp;
	temp = 1.0*Turn_Kp/100*Target_turn + 1.0*Turn_Kd/100*gyroZ;
	
	return temp;

}

int b2f(uint8_t  m0, uint8_t m1, uint8_t m2, uint8_t m3)
{	
	int temp;
	
	temp = ((int)m0)<<24 | ((int)m1)<<16 |((int)m2)<<8 |(int)m3;
	return temp;
}

void change_PID(void)
{	
	int PID_temporary[6] = {0};
	uint8_t i,j = 0;
	for( i = 0;i<25;i+=4)
	{
		PID_temporary[j++] = b2f(PID_Data[i+3],PID_Data[i+2],PID_Data[i+1],PID_Data[i]);
	
	}
	Velocity_Kp = PID_temporary[0];
	Velocity_Ki = PID_temporary[1]; 
	Vertical_Kp = PID_temporary[2]; 
	Vertical_Kd = PID_temporary[3]; 
	Turn_Kp = PID_temporary[4]; 
	Turn_Kd = PID_temporary[5]; 
}
int8_t EXTIFlag = 0; 

uint8_t GetEXTIFlag(void)
{

	if(EXTIFlag == 1)
	{
		EXTIFlag = 0;
		return 1;
	} else
	{
		return 0;
	}
}


uint8_t biaozhi  = 0;
extern uint8_t biaozhi;
void Blue_control_move(void)
{	
	
	if((Fore==0)&&(Back==0))
				Target_speed=0;//未接受到前进后退指令-->速度清零，稳在原地
			if(Fore==1)
			{	
				if(biaozhi == 0)
					Target_speed--;//前进1标志位拉高-->需要前进
				if(Target_speed == 31 )
					biaozhi  = 1;
				if(biaozhi == 1 && Target_speed != 0)
				{
					Target_speed--;
					if(Target_speed == 0)
					{
						Fore = 0;
						biaozhi  = 0;
					}
				}
			}
			if(Back==1)
			{	
				if(biaozhi == 0)
					Target_speed++;//后退1标志位拉高-->需要后退
				if(Target_speed == -31 )
					biaozhi  = 1;
				if(biaozhi == 1 && Target_speed != 0)
				{
					Target_speed++;
					if(Target_speed == 0)
					{
						Back = 0;
						biaozhi  = 0;
					}
				}
				
			}
			Target_speed=Target_speed>SPEED_Y?SPEED_Y:(Target_speed<-SPEED_Y?(-SPEED_Y):Target_speed);  // 限幅
			if((Left==0)&&(Right==0))
				Turn_speed=0;
			if(Left==1)
				Turn_speed-=30;	//左转
			if(Right==1)
				Turn_speed+=30;	//右转
			Turn_speed=Turn_speed>SPEED_Z?SPEED_Z:(Turn_speed<-SPEED_Z?(-SPEED_Z):Turn_speed);//限幅

}




void EXTI1_IRQHandler(void)
{	

	if(EXTI_GetITStatus(EXTI_Line1) == SET)
	{

		int PWM_out;
		
		Encoder_Left = -Read_Encoder(2);//电机是相对安装，刚好相差180度，为了编码器输出极性一致，就需要对其中一个取反。
		Encoder_Right = Read_Encoder(3);
			
		MPU6050_GetData(&Data);						 						//MPU6050输出的原始数据
			
		mpu_dmp_get_data( &pitch, &roll, &yaw); 							//欧拉角(姿态角)
			
	
	
		Velocity_out = Velocity( Target_speed, Encoder_Left, Encoder_Right);//速度环
		Vertical_out =  Vertical(Velocity_out + Med_Angle,roll, Data.GyroX);//直立环
		Turn_out =  Turn( Data.GyroZ,Turn_speed);							//转向环
		PWM_out = Vertical_out;		
		moto_1 = PWM_out - Turn_out;										//右电机
		moto_2 = PWM_out + Turn_out;										//左电机
		
		limit_moto_1_2();													//PWM限幅				
		PWM_Set(moto_1, moto_2);//右，左

		Turn_Off(&Med_Angle,&roll, voltage);								//安全检测

		control_mode();
		
	}
	
	EXTI_ClearITPendingBit(EXTI_Line1);
	
	
}


uint8_t zishu = 0;
void control_mode(void)
{
	switch(mode_select) 
	{
		case 16:													//超声波测距避障模式
			zishu++;
			if(zishu >= 15)
			{
				zishu = 0;
				SR04_Start_Measure();								//每100ms触发SR04的测距
				
			}
			if(SR04_length <= 40)
			{
				Target_speed = 0;
				Turn_speed = 20;
			
			}
			else if(SR04_length >= 40)
			{
				Target_speed = -20;
				Turn_speed = 0;
			
			}
			break;
	
		case 17:													//语音控制模式
				SR04_length = 0;
				Blue_control_move();								//蓝牙控制
			break;
		default :
				Target_speed = 0;
				Turn_speed = 0;
			break;
	}

}








