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

//������������������13������������500
/*
-------------------------------------IO�ڶ�Ӧ���ܱ�-------------------------------------------------------
���������                B12->BIN2       B13->BIN1      B14->AIN1     B15->AIN2
	PWM   ��                A8 ->PWMA                        A11->PWMB
MUP6050 :                 B6 ->SCL                         B9 ->SDA
�����������              B7 B8 һ��                       A0 A1һ��
JY-61������:      		    RXD ->B10                        TXD->B11

                    �����ֵΪ�������ٶ����ֵ��60����


*/
int Encoder_Left=0,Encoder_Right=0;       //���ұ��������������
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;
float yaw,pitch,roll,Gyro_X,Gyro_Y,Gyro_Z;
int left=6000,right=6000;
u8 stop_flag=0;
extern int Moto1,Moto2;  							//���PWM����

			int balance_speed=0;


//			yaw=(float)stcAngle.Angle[2]/32768*180;
//			pitch=(float)stcAngle.Angle[1]/32768*180;
//			roll=(float)stcAngle.Angle[0]/32768*180;
extern void calculation(void);//��ȡ����ֵ���Ƕ�ֵ
void sendcmd(char cmd[]);
void CopeSerial2Data(unsigned char ucData);

/********************************************************************main����**************************************************************************/
 int main(void)
 {	
	SystemInit();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init();	    	 //��ʱ������ʼ��	  

	PID_Init();
	MiniBalance_PWM_Init(7199,0);   //��ʼ��PWM 10KHZ������������� ,PWM���7200  ��ʼ��˳�������д���֮ǰ
	uart3_init(115200);
	uart_init(9600);	 							//���ڳ�ʼ��Ϊ9600

	TIM3_Int_Init(100-1,7200-1);//����Ƶ��PWMƵ��=72000/(7199+1)=1Khz 1ms   72000/7200/100=100HZ  10ms
	Encoder_Init_TIM2();            //=====�������ӿ�
	Encoder_Init_TIM4();            //=====��ʼ��������2
	 
/************�Ƕ�ֵ�����ٶȳ�ʼ��Ϊ0��ˮƽ����*****************/ 
	sendcmd(YAWCMD);
	sendcmd(ACCCMD);
  sendcmd(MODEDISPLAY);
	 
	delay_ms(1000);
   	while(1)
	{
//			Encoder_Left=Read_Encoder(2);                             //===��ȡ��������ֵ
//			Encoder_Right=Read_Encoder(4); 
//      Set_Pwm(left,right);

	} 
}
/********************************************************************main����**************************************************************************/
//��ʱ��3�жϷ������
void TIM3_IRQHandler(void)   //TIM3�ж�
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
		{
			yaw=(float)stcAngle.Angle[2]/32768*180;
			pitch=(float)stcAngle.Angle[0]/32768*180;
			roll=(float)stcAngle.Angle[1]/32768*180;
			Gyro_X=(float)stcGyro.w[2]/32768*2000;
			Gyro_Y=(float)stcGyro.w[1]/32768*2000;
			Gyro_Z=(float)stcGyro.w[0]/32768*2000;

			calculation();
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ 
		}
}

//�ô���2��JYģ�鷢��ָ��
void sendcmd(char cmd[])
{
	char i;
	for(i=0;i<3;i++)
	UART3_Put_Char(cmd[i]);
}

//CopeSerialDataΪ����3�жϵ��ú���������ÿ�յ�һ�����ݣ�����һ�����������
void CopeSerial2Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	
	ucRxBuffer[ucRxCnt++]=ucData;	//���յ������ݴ��뻺������
	if (ucRxBuffer[0]!=0x55) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//���ݲ���11�����򷵻�
	else
	{
		switch(ucRxBuffer[1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
		{
			//memcpyΪ�������Դ����ڴ濽��������������"string.h"�������ջ��������ַ����������ݽṹ�����棬�Ӷ�ʵ�����ݵĽ�����
			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;

		}
		ucRxCnt=0;//��ջ�����
	}
}

//void CopeSerial1Data(unsigned char ucData)
//{	
//	UART3_Put_Char(ucData);//ת������1�յ������ݸ�����2��JYģ�飩
//}

void usart1_send_str(u8 *Data) //����1���ͺ���
{
	while(*Data) 
	{
		USART_SendData(USART1, *Data++);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); 
	}
}
/**************************************************************************
�������ܣ��������PID����ȡ�Ƕ�ֵ�ȵ�
��ڲ�����δ֪
����  ֵ����
						��Ӧ�ò�д���⣬������Ϊ����ԭ��ֻ�ܷ���
**************************************************************************/
void calculation(void)
{
			Encoder_Left=Read_Encoder(2);                             //===��ȡ��������ֵ
			Encoder_Right=-Read_Encoder(4); 

//			pid.Pv = pitch*10;																				//�Ƕ�*ʮ��		
				pid.Pv = pitch;	
			if(Turn_Off(pid.Pv)) stop_flag=1;													//����Ƿ�С���쳣���쳣��ֹͣ
			else stop_flag=0;
									
//			Moto1 = balance(pid.Pv) + velocity(Encoder_Left,Encoder_Right)-turn(yaw);		//���PWM���ֵ
//			Moto2 =balance(pid.Pv) + velocity(Encoder_Left,Encoder_Right)+turn(yaw);
			balance_speed=balance(pid.Pv);
			Moto1 = velocity_M1(Encoder_Left,balance_speed);
			Moto2 = velocity_M2(Encoder_Right,balance_speed);
			
			Limit_Pwm();																							//��PWM�����޷�
  		Set_Pwm(Moto1,Moto2);  																		//����PWM

}










