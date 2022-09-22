#include "pid.h"


 PID pid;

/**************************************************************************
函数功能：PID数据初始化
入口参数：无
返回  值：无
**************************************************************************/
void PID_Init()
{
	          /*平衡PID环控制参数初始化*/
		pid.Sv = 0;		//7.2f
		pid.Kp = 4;
		pid.Kd = 0.04;  
	/*速度PID环控制参数初始化*/		
		pid.Kp_speed = 850;         //光电编码器KP 150   KI  0.5 MAX——iout 4000    
		pid.Ki_speed = 0.9;						//霍尔编码器  呜呜呜
		pid.EK_speed_M1 = 0;   //速度误差
		pid.SEK_speed_M1= 0;  //速度误差的积分
	
		pid.EK_speed_M2 = 0;   //速度误差
		pid.SEK_speed_M2= 0;  //速度误差的积分
	
		pid.target_speed_left = 0;       //速度的设定值  max 70
		pid.target_speed_right = 0;				//速度初始值
							/*转向PID环控制参数初始化*/
		pid.Kp_turn =35;
		pid.Angle_turn = 0;
}

/**************************************************************************
函数功能：以下三个函数，分别计算各个环的PID值，并返回
入口参数：未知
返回  值：无
**************************************************************************/


				/*小车平衡环部分，微分+比例控制
	        微分变量为直接读取的加速度*/
extern float Gyro_Z;
float last_Angle=0;
int balance(float Angle)
{  
    signed  int  Bias;
		int balance_set;

	  Bias=(Angle-pid.Sv);
	  balance_set=pid.Kp*Bias+Gyro_Z*pid.Kd;  
		last_Angle=Angle;
	/****************************************速度限幅**************************************/
		if(balance_set>MAX_speed)
		balance_set=MAX_speed;
		else if(balance_set<-MAX_speed)
		balance_set=-MAX_speed;
		else balance_set=balance_set;
		//**//
	  return (int)balance_set;
}

				/*小车速度环部分， 积分+比例控制*/
int velocity_M1(int speed,int speed_set)
{    
	     
		int output_cali;	
		int iout;	
		pid.EK_speed_M1 = speed_set-speed;
		pid.SEK_speed_M1 += pid.EK_speed_M1;
		iout=(int)(pid.Ki_speed*pid.SEK_speed_M1);
	/***积分限幅*****/
		if(iout>MAX_iout)
			iout=MAX_iout;
		else if(iout<-MAX_iout)
			iout=-MAX_iout;
		else iout=iout;
		
		output_cali = pid.Kp_speed*pid.EK_speed_M1 +iout;
		return output_cali;
}
int velocity_M2(int speed,int speed_set)
{    
	     
		int output_cali;	
		int iout;	
		pid.EK_speed_M2 = speed_set-speed;
		pid.SEK_speed_M2 += pid.EK_speed_M2;
		iout=(int)(pid.Ki_speed*pid.SEK_speed_M2);
	/***积分限幅*****/
		if(iout>MAX_iout)
			iout= MAX_iout;
		else if(iout<-MAX_iout)
			iout= - MAX_iout;
		else iout= iout;
		
		output_cali = pid.Kp_speed*pid.EK_speed_M2 +iout;
		return output_cali;
}
				/*小车转向环部分，比例控制*/
int turn(float gyro)//转向控制
{
    int Turn;
	  float Bias;
			
			if(pid.Angle_turn <= -180&&gyro>0)
				Bias = (gyro-(pid.Angle_turn+360))*10;	
			else if(pid.Angle_turn <= -360&&gyro<0)
				Bias = (gyro-(pid.Angle_turn+360))*10;	
			else if(pid.Angle_turn >= 180&&gyro<0)
				 Bias = (gyro-(pid.Angle_turn-360))*10;
			else if(pid.Angle_turn >= 360&&gyro>0)
					Bias = (gyro-(pid.Angle_turn-360))*10;
		  else
				 Bias = (gyro-pid.Angle_turn)*10;	

	  Turn = (int)(Bias*pid.Kp_turn); 
	
				/*进行转向速度的单独限幅*/
		if(Turn>=1500)	Turn =1500;
		else if(Turn<=-1500)Turn =-1500;
			
		return Turn;
}
