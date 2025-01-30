#ifndef __CONTROL_H
#define __CONTROL_H




#define SPEED_Y 20  //俯仰(前后)最大设定速度
#define SPEED_Z 100//偏航(左右)最大设定速度 



int Vertical(float Med,float Angle,float gyro);
int Velocity(int Target,int encoder_L,int encoder_R);
int Turn(float gyroZ,int Target_turn);	
void control(void);
int b2f(uint8_t  m0, uint8_t m1, uint8_t m2, uint8_t m3);
void change_PID(void);
uint8_t GetEXTIFlag(void);

void Blue_control_move(void);
void control_mode(void);





#endif
