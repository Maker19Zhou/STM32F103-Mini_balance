#include "pid.h"


 PID pid;

/**************************************************************************
�������ܣ�PID���ݳ�ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void PID_Init()
{
	          /*ƽ��PID�����Ʋ�����ʼ��*/
		pid.Sv = 0;		//7.2f
		pid.Kp = 4;
		pid.Kd = 0.04;  
	/*�ٶ�PID�����Ʋ�����ʼ��*/		
		pid.Kp_speed = 850;         //��������KP 150   KI  0.5 MAX����iout 4000    
		pid.Ki_speed = 0.9;						//����������  ������
		pid.EK_speed_M1 = 0;   //�ٶ����
		pid.SEK_speed_M1= 0;  //�ٶ����Ļ���
	
		pid.EK_speed_M2 = 0;   //�ٶ����
		pid.SEK_speed_M2= 0;  //�ٶ����Ļ���
	
		pid.target_speed_left = 0;       //�ٶȵ��趨ֵ  max 70
		pid.target_speed_right = 0;				//�ٶȳ�ʼֵ
							/*ת��PID�����Ʋ�����ʼ��*/
		pid.Kp_turn =35;
		pid.Angle_turn = 0;
}

/**************************************************************************
�������ܣ����������������ֱ�����������PIDֵ��������
��ڲ�����δ֪
����  ֵ����
**************************************************************************/


				/*С��ƽ�⻷���֣�΢��+��������
	        ΢�ֱ���Ϊֱ�Ӷ�ȡ�ļ��ٶ�*/
extern float Gyro_Z;
float last_Angle=0;
int balance(float Angle)
{  
    signed  int  Bias;
		int balance_set;

	  Bias=(Angle-pid.Sv);
	  balance_set=pid.Kp*Bias+Gyro_Z*pid.Kd;  
		last_Angle=Angle;
	/****************************************�ٶ��޷�**************************************/
		if(balance_set>MAX_speed)
		balance_set=MAX_speed;
		else if(balance_set<-MAX_speed)
		balance_set=-MAX_speed;
		else balance_set=balance_set;
		//**//
	  return (int)balance_set;
}

				/*С���ٶȻ����֣� ����+��������*/
int velocity_M1(int speed,int speed_set)
{    
	     
		int output_cali;	
		int iout;	
		pid.EK_speed_M1 = speed_set-speed;
		pid.SEK_speed_M1 += pid.EK_speed_M1;
		iout=(int)(pid.Ki_speed*pid.SEK_speed_M1);
	/***�����޷�*****/
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
	/***�����޷�*****/
		if(iout>MAX_iout)
			iout= MAX_iout;
		else if(iout<-MAX_iout)
			iout= - MAX_iout;
		else iout= iout;
		
		output_cali = pid.Kp_speed*pid.EK_speed_M2 +iout;
		return output_cali;
}
				/*С��ת�򻷲��֣���������*/
int turn(float gyro)//ת�����
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
	
				/*����ת���ٶȵĵ����޷�*/
		if(Turn>=1500)	Turn =1500;
		else if(Turn<=-1500)Turn =-1500;
			
		return Turn;
}
