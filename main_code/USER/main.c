#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"	
#include "JY61.h"
#include "usart3.h"
#include "string.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"

//霍尔编码器的线数是13，光电编码器是500
/*
-------------------------------------IO口对应功能表-------------------------------------------------------
电机驱动：                B12->BIN2       B13->BIN1      B14->AIN1     B15->AIN2
	PWM   ：                A8 ->PWMA                        A11->PWMB
MUP6050 :                 B6 ->SCL                         B9 ->SDA
电机编码器：              B7 B8 一组                       A0 A1一组
JY-61陀螺仪:      		    RXD ->B10                        TXD->B11

                    电机负值为正方向，速度最大值在60左右


*/
int Encoder_Left=0,Encoder_Right=0;       //左右编码器的脉冲计数
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;
float yaw,pitch,roll,Gyro_X,Gyro_Y,Gyro_Z;
int left=6000,right=6000;
u8 stop_flag=0;
extern int Moto1,Moto2;  							//电机PWM变量

			int balance_speed=0;


//			yaw=(float)stcAngle.Angle[2]/32768*180;
//			pitch=(float)stcAngle.Angle[1]/32768*180;
//			roll=(float)stcAngle.Angle[0]/32768*180;
extern void calculation(void);//读取编码值，角度值
void sendcmd(char cmd[]);
void CopeSerial2Data(unsigned char ucData);

/********************************************************************main函数**************************************************************************/
 int main(void)
 {	
	SystemInit();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init();	    	 //延时函数初始化	  

	PID_Init();
	MiniBalance_PWM_Init(7199,0);   //初始化PWM 10KHZ，用于驱动电机 ,PWM最多7200  初始化顺序在所有串口之前
	uart3_init(115200);
	uart_init(9600);	 							//串口初始化为9600

	TIM3_Int_Init(100-1,7200-1);//不分频。PWM频率=72000/(7199+1)=1Khz 1ms   72000/7200/100=100HZ  10ms
	Encoder_Init_TIM2();            //=====编码器接口
	Encoder_Init_TIM4();            //=====初始化编码器2
	 
/************角度值、加速度初始化为0，水平放置*****************/ 
	sendcmd(YAWCMD);
	sendcmd(ACCCMD);
  sendcmd(MODEDISPLAY);
	 
	delay_ms(1000);
   	while(1)
	{
//			Encoder_Left=Read_Encoder(2);                             //===读取编码器的值
//			Encoder_Right=Read_Encoder(4); 
//      Set_Pwm(left,right);

	} 
}
/********************************************************************main函数**************************************************************************/
//定时器3中断服务程序
void TIM3_IRQHandler(void)   //TIM3中断
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
		{
			yaw=(float)stcAngle.Angle[2]/32768*180;
			pitch=(float)stcAngle.Angle[0]/32768*180;
			roll=(float)stcAngle.Angle[1]/32768*180;
			Gyro_X=(float)stcGyro.w[2]/32768*2000;
			Gyro_Y=(float)stcGyro.w[1]/32768*2000;
			Gyro_Z=(float)stcGyro.w[0]/32768*2000;

			calculation();
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源 
		}
}

//用串口2给JY模块发送指令
void sendcmd(char cmd[])
{
	char i;
	for(i=0;i<3;i++)
	UART3_Put_Char(cmd[i]);
}

//CopeSerialData为串口3中断调用函数，串口每收到一个数据，调用一次这个函数。
void CopeSerial2Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	
	ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//数据不满11个，则返回
	else
	{
		switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
			//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;

		}
		ucRxCnt=0;//清空缓存区
	}
}

//void CopeSerial1Data(unsigned char ucData)
//{	
//	UART3_Put_Char(ucData);//转发串口1收到的数据给串口2（JY模块）
//}

void usart1_send_str(u8 *Data) //串口1发送函数
{
	while(*Data) 
	{
		USART_SendData(USART1, *Data++);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); 
	}
}
/**************************************************************************
函数功能：整体计算PID，读取角度值等等
入口参数：未知
返回  值：无
						本应该不写在这，但是因为各种原因只能放这
**************************************************************************/
void calculation(void)
{
			Encoder_Left=Read_Encoder(2);                             //===读取编码器的值
			Encoder_Right=-Read_Encoder(4); 

//			pid.Pv = pitch*10;																				//角度*十倍		
				pid.Pv = pitch;	
			if(Turn_Off(pid.Pv)) stop_flag=1;													//检测是否小车异常，异常就停止
			else stop_flag=0;
									
//			Moto1 = balance(pid.Pv) + velocity(Encoder_Left,Encoder_Right)-turn(yaw);		//获得PWM输出值
//			Moto2 =balance(pid.Pv) + velocity(Encoder_Left,Encoder_Right)+turn(yaw);
			balance_speed=balance(pid.Pv);
			Moto1 = velocity_M1(Encoder_Left,balance_speed);
			Moto2 = velocity_M2(Encoder_Right,balance_speed);
			
			Limit_Pwm();																							//对PWM进行限幅
  		Set_Pwm(Moto1,Moto2);  																		//设置PWM

}










