#include <stdint.h>
#include "STM32F10xx.h"
#include "STM32F103xx_GPIO.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void delay(uint32_t num)
{
	for(volatile uint32_t i = 0; i < num; i++);
}
int main(void)
{
	GPIO_Handle RGB_LED, Button, Built_In_LED;

	Built_In_LED.GPIOx = GPIOC;
	Built_In_LED.Pin_Config.GPIO_PinNum = 13;
	Built_In_LED.Pin_Config.GPIO_PinMode = GPIO_OUT_10MHz;
	Built_In_LED.Pin_Config.GPIO_PinOutputMode = GPIO_OUT_PP;

	GPIO_ClockControl(GPIOC, Enable);
	GPIO_Init(&Built_In_LED);

	RGB_LED.GPIOx = GPIOA;
	RGB_LED.Pin_Config.GPIO_PinNum = 3;
	RGB_LED.Pin_Config.GPIO_PinMode = GPIO_OUT_10MHz;
	RGB_LED.Pin_Config.GPIO_PinOutputMode = GPIO_OUT_PP;

	GPIO_ClockControl(GPIOA, Enable);
	GPIO_Init(&RGB_LED);

	RGB_LED.GPIOx = GPIOA;
	RGB_LED.Pin_Config.GPIO_PinNum = 4;
	RGB_LED.Pin_Config.GPIO_PinMode = GPIO_OUT_10MHz;
	RGB_LED.Pin_Config.GPIO_PinOutputMode = GPIO_OUT_PP;
	GPIO_Init(&RGB_LED);

	RGB_LED.GPIOx = GPIOA;
	RGB_LED.Pin_Config.GPIO_PinNum = 5;
	RGB_LED.Pin_Config.GPIO_PinMode = GPIO_OUT_10MHz;
	RGB_LED.Pin_Config.GPIO_PinOutputMode = GPIO_OUT_PP;
	GPIO_Init(&RGB_LED);

	Button.GPIOx = GPIOA;
	Button.Pin_Config.GPIO_PinNum = 7;
	Button.Pin_Config.GPIO_PinMode = GPIO_EXTI_FT;
	Button.Pin_Config.GPIO_PinInputMode = GPIO_IN_PULUD;
	Button.Pin_Config.GPIO_PinPUD_Mode = GPIO_IN_PULUP;
	GPIO_Init(&Button);

	GPIO_IRQ_Priority(EXTI5_9_IRQ, 0x20);
	GPIO_IRQEnable(EXTI5_9_IRQ, Enable);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOC, 13);
		delay(200000);
	}
}

void EXTI9_5_IRQHandler(void)
{
	delay(100000);
	volatile static uint8_t c = 0;
	GPIO_IRQHandling(7);

	if(c <= 2)
    {
	   GPIO_WriteToPin(GPIOA, c+3, HIGH);
	   delay(800000);
	   GPIO_WriteToPin(GPIOA, c+3, LOW);
	   delay(800000);
	   c++;
    }else c = 0;
}
