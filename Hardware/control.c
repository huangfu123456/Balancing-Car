#include "stm32f10x.h"                  // Device header
#include "encoder.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "config.h"


	
int Velocity_out,Vertical_out,Turn_out;
	



int Vertical(float Med,float Angle,float gyro)//ֱ��������
{
	int temp;
	temp = 1.0*Vertical_Kp/100*(Angle-Med) + 1.0*Vertical_Kd/100*gyro;
	
	return temp;

}

int Velocity(int Target,int encoder_L,int encoder_R)//�ٶȻ�����
{
	static int Error_Low_out_last,encoder_S;
	static float a = 0.7;
	int Err,Error_Low_out,temp;
//	Velocity_Ki = Velocity_Kp/200;
	Err = (encoder_L + encoder_L) - Target;
	Error_Low_out = (1-a)*Err + a*Error_Low_out_last;	//��ͨ�˲�
			
	encoder_S += Error_Low_out;
	Error_Low_out_last = Error_Low_out;		

	if(stop==1)
	{
		encoder_S=0,stop=0;//���������
	}
	
	if(encoder_S > 20000 | encoder_S < -20000)			//�����޷�
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

int Turn(float gyroZ,int Target_turn)		//ת�򻷿���
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
				Target_speed=0;//δ���ܵ�ǰ������ָ��-->�ٶ����㣬����ԭ��
			if(Fore==1)
			{	
				if(biaozhi == 0)
					Target_speed--;//ǰ��1��־λ����-->��Ҫǰ��
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
					Target_speed++;//����1��־λ����-->��Ҫ����
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
			Target_speed=Target_speed>SPEED_Y?SPEED_Y:(Target_speed<-SPEED_Y?(-SPEED_Y):Target_speed);  // �޷�
			if((Left==0)&&(Right==0))
				Turn_speed=0;
			if(Left==1)
				Turn_speed-=30;	//��ת
			if(Right==1)
				Turn_speed+=30;	//��ת
			Turn_speed=Turn_speed>SPEED_Z?SPEED_Z:(Turn_speed<-SPEED_Z?(-SPEED_Z):Turn_speed);//�޷�

}




void EXTI1_IRQHandler(void)
{	

	if(EXTI_GetITStatus(EXTI_Line1) == SET)
	{

		int PWM_out;
		
		Encoder_Left = -Read_Encoder(2);//�������԰�װ���պ����180�ȣ�Ϊ�˱������������һ�£�����Ҫ������һ��ȡ����
		Encoder_Right = Read_Encoder(3);
			
		MPU6050_GetData(&Data);						 						//MPU6050�����ԭʼ����
			
		mpu_dmp_get_data( &pitch, &roll, &yaw); 							//ŷ����(��̬��)
			
	
	
		Velocity_out = Velocity( Target_speed, Encoder_Left, Encoder_Right);//�ٶȻ�
		Vertical_out =  Vertical(Velocity_out + Med_Angle,roll, Data.GyroX);//ֱ����
		Turn_out =  Turn( Data.GyroZ,Turn_speed);							//ת��
		PWM_out = Vertical_out;		
		moto_1 = PWM_out - Turn_out;										//�ҵ��
		moto_2 = PWM_out + Turn_out;										//����
		
		limit_moto_1_2();													//PWM�޷�				
		PWM_Set(moto_1, moto_2);//�ң���

		Turn_Off(&Med_Angle,&roll, voltage);								//��ȫ���

		control_mode();
		
	}
	
	EXTI_ClearITPendingBit(EXTI_Line1);
	
	
}


uint8_t zishu = 0;
void control_mode(void)
{
	switch(mode_select) 
	{
		case 16:													//������������ģʽ
			zishu++;
			if(zishu >= 15)
			{
				zishu = 0;
				SR04_Start_Measure();								//ÿ100ms����SR04�Ĳ��
				
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
	
		case 17:													//��������ģʽ
				SR04_length = 0;
				Blue_control_move();								//��������
			break;
		default :
				Target_speed = 0;
				Turn_speed = 0;
			break;
	}

}








