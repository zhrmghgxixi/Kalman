#ifndef __LED_H
#define __LED_H	 
#include "sys.h"

#define LED0 PAout(0)		//
#define LED1 PAout(1)		//
#define LED2 PCout(13)  //板子上LED指示灯

void LED_Init(void);	

		 				    
#endif
