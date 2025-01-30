#include "config.h"                  // ������а������еĿ�

/****************************ȫ�ֱ���*************************************/   

int PWM_MAX = 7000,PWM_MIN = -7000;							 //PWM����޷�
int Encoder_Left,Encoder_Right;								 //�������ٶ�
int32_t moto_1=0,moto_2=0;									 //PID������������������

MPU6050_Get_Data Data;										 //MPU6050�����ԭʼ����
float pitch,roll,yaw; 								  		 //ŷ����(��̬��)
float voltage = 12;
int8_t temp;

int  	Vertical_Kp = 25000,
		Vertical_Kd = 180;								 		//PDֱ��������*100

int  	Velocity_Kp  = -60,
		Velocity_Ki = -3;						//5*Kp		 		//PI�ٶȻ�����Kp*100,Ki*1000 (һ��Ki = Kp/200)

int  	Turn_Kp = 1000,
		Turn_Kd = 20;										 	//PDת�򻷲���*100

int 	Target_speed,							//�����ٶȣ�������
		Turn_speed;								//�����ٶȣ�ƫ����

float 	Med_Angle = 2;											 //��е��ֵ
			
u8 PID_Data[25] = {0};
int8_t stop = 0;
float SR04_length = 0;
int mode_select = 0;
/**************************************************************************/
// uint32_t timeout = 10000;
int main(void)
{	
	delay_init();
	OLED_Init();
	OLED_Clear();   
	OLED_ShowString(1,1,"Enco_Left:");
	OLED_ShowString(2,1,"Enco_Right:");
	OLED_ShowString(3,1,"roll:");
	OLED_ShowString(4,1,"SR04:     cm");
	
	SR04_Init();
	
	Encoder_TIM2_Init();
	Encoder_TIM3_Init();
	
	MPU_Init();
	mpu_dmp_init();
	EXTI_MPU_Init();
	
	Usart3_Blue_Init(9600);
	
	TIM1_PWM_Init(7199,10);					//PWMƵ��10kHz

	Motor_Init();
	
	
	

	
	NVIC_Config();

	while(1)
	{

		
		OLED_ShowSignedNum(1,12,Encoder_Left,3);
		OLED_ShowSignedNum(2,12,Encoder_Right,3);
		OLED_ShowSignedNum(3,12,roll,3);
		OLED_ShowNum(4,6,(uint32_t)SR04_length,5);
		

	}
}


