#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H 
#include "sys.h"	  	
#include "led.h"
#include "pid.h"
void uart3_init(u32 bound);
void USART3_IRQHandler(void);
void UART3_Put_Char(unsigned char DataToSend);
void UART3_Put_String(unsigned char *Str);
#endif

