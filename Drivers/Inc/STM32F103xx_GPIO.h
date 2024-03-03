#ifndef INC_GPIO_H_
#define INC_GPIO_H_
#include "STM32F10xx.h"

/* the struct to have the pins configurations */
typedef struct{
	uint8_t GPIO_PinNum;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinInputMode;
	uint8_t GPIO_PinOutputMode;
	uint8_t GPIO_PinPUD_Mode;
	AFIO_REGs GPIO_AFIO;
}GPIO_PinConfig;

/* the struct to handle the configurations of the GPIO */
typedef struct{
	GPIO_REGs *GPIOx;
	GPIO_PinConfig Pin_Config;
}GPIO_Handle;

#define PA                0
#define PB                1
#define PC                2
#define PD                3
#define PE                4
#define PF                5
#define PG                6

/* this is the section to define the possible values of GPIO configurations */
/* GPIO modes */
#define GPIO_IN           0   // this is to configure the GPIO pin as an input
#define GPIO_OUT_10MHz    1   // this is to configure the GPIO pin as an output with 10MHz max speed
#define GPIO_OUT_2MHz     2   // this is to configure the GPIO pin as an output with 2MHz max speed
#define GPIO_OUT_50MHz    3   // this is to configure the GPIO pin as an output with 50MHz max speed
#define GPIO_EXTI_RT      4   // this is to configure the GPIO pin as an EXTI source with rising triggering
#define GPIO_EXTI_FT      5   // this is to configure the GPIO pin as an EXTI source with falling triggering
#define GPIO_EXTI_RFT     6   // this is to configure the GPIO pin as an EXTI source with rising and falling triggering

/* input modes */
#define GPIO_IN_ANLG      0   // this is for the inputs as analog mode
#define GPIO_IN_FLT       1   // this is for the inputs as floating (reset state)
#define GPIO_IN_PULUD     2   // this is for the inputs with pull up or down

#define GPIO_IN_PULUP     0   // this is to make the pin in input pull up mode
#define GPIO_IN_PULDWN    1   // this is to make the pin in input pull down mode


/* output modes */
#define GPIO_OUT_PP       0   // this is for the outputs in push-pull mode
#define GPIO_OUT_OD       1   // this is for the outputs in open-drain mode
#define GPIO_OUT_AFPP     2   // this is for the outputs in alternate function mode in push-pull mode
#define GPIO_OUT_AFOD     3   // this is for the outputs in alternate function mode in open-drain mode



/* the APIs for the GPIO */
/* control the clock of the GPIO */
void GPIO_ClockControl(GPIO_REGs *pGPIOx, Status status);

/* init and reset to the reset value */
void GPIO_Init(GPIO_Handle *pGPIOx);
void GPIO_Deinit(GPIO_REGs *pGPIOx);

/* GPIO read and write */
void GPIO_WriteToPin(GPIO_REGs *pGPIOx, uint8_t PinNum, uint8_t data);
uint8_t GPIO_ReadFromPin(GPIO_REGs*pGPIOx, uint8_t PinNum);
void GPIO_WriteToPort(GPIO_REGs *pGPIOx, uint16_t data);
uint16_t GPIO_ReadFromPort(GPIO_REGs *pGPIOx);
void GPIO_ToggleOutputPin(GPIO_REGs *pGPIOx, uint8_t PinNum);

/* GPIO IRQs handling */
void GPIO_IRQEnable(uint8_t IRQ, Status status);
void GPIO_IRQ_Priority(uint8_t IRQ, uint8_t priority);
void GPIO_IRQHandling(uint8_t PinNum);

#endif /* INC_GPIO_H_ */
