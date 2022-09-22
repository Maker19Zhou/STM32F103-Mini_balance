#ifndef __PID_H__
#define __PID_H__
#include "sys.h"
//PID�ṹ��
typedef struct 
{
	float  Sv;			//�û��趨ƽ��λ��ֵ
	float Pv;
					/*ƽ�⻷�����趨 */
	float  Kd;					//ƽ��΢����ϵ��
	float  Kp;			//ƽ�������ϵ��
	

	
					/*�ٶȻ������趨 */
	signed int target_speed_left;      //Ŀ�����ٶ�
	signed int target_speed_right;     //Ŀ�����ٶ�
	float  Ki_speed;				//�ٶȻ�������ϵ��
	signed int Kp_speed;		//�ٶȻ�������ϵ��
	signed int EK_speed_M1;  	//�ٶ�ƫ��
	signed int SEK_speed_M1;   //��ʷƫ��֮��
	signed int EK_speed_M2;  	//�ٶ�ƫ��
	signed int SEK_speed_M2;   //��ʷƫ��֮��

					/*ת�򻷲����趨 */
	float Kp_turn;					//ת�򻷱�����ϵ��
	float Angle_turn;				//Ŀ��ת��Ƕ�
	
}PID;

#define MAX_iout 5500
#define MAX_speed 72
extern PID pid;

int balance(float Angle);
void PID_Init(void);
int velocity_M1(int speed,int speed_set);
int velocity_M2(int speed,int speed_set);
int turn(float gyro);

#endif
