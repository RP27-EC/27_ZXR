#include "led_task.h"


void StartLedTask(void const * argument)
{
	led.state = LED_BLINK;       // 设置为闪烁模式 
  led.colour = LED_colour_red; // 红灯 
  led.blink_fre = 2;           // 闪烁频率：2Hz (每 500ms 闪烁一次)
  for(;;)
  {		
	  led_work(&led);
		osDelay(1);
  }
}



