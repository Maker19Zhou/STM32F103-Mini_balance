#ifndef __PID_H__
#define __PID_H__
#include "sys.h"
//PID结构体
typedef struct 
{
	float  Sv;			//用户设定平衡位置值
	float Pv;
					/*平衡环参数设定 */
	float  Kd;					//平衡微分项系数
	float  Kp;			//平衡比例项系数
	

	
					/*速度环参数设定 */
	signed int target_speed_left;      //目标左速度
	signed int target_speed_right;     //目标右速度
	float  Ki_speed;				//速度环积分项系数
	signed int Kp_speed;		//速度环比例项系数
	signed int EK_speed_M1;  	//速度偏差
	signed int SEK_speed_M1;   //历史偏差之和
	signed int EK_speed_M2;  	//速度偏差
	signed int SEK_speed_M2;   //历史偏差之和

					/*转向环参数设定 */
	float Kp_turn;					//转向环比例项系数
	float Angle_turn;				//目标转向角度
	
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
