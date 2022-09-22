#ifndef __JY61_H
#define __JY61_H

struct SAcc
{
	short a[3];
	short T;
};
struct SGyro
{
	short w[3];
	short T;
};
struct SAngle
{
	short Angle[3];
	short T;
};
 
char YAWCMD[3] = {0XFF,0XAA,0X52}; //角度初始化
char ACCCMD[3] = {0XFF,0XAA,0X67};  //加速度计校准
char SLEEPCMD[3] = {0XFF,0XAA,0X60};
char UARTMODECMD[3] = {0XFF,0XAA,0X61};
char IICMODECMD[3] = {0XFF,0XAA,0X62};
char MODEDISPLAY[3]={0xFF,0xAA,0x65}; //水平安装
								
#endif
