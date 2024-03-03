#include "STM32F103xx_GPIO.h"

/* control the clock of the GPIO */

/*
 * @fun_name:  GPIO_ClockControl
 * @fun_brief: this is a function to enable or disable the clock for the GPIO
 * @param[in]: the base address for the specific GPIO port you need to enable the clock for
 * @param[in]: the enable or disable state
 * @return:    none
 * @notes:     none
 */
void GPIO_ClockControl(GPIO_REGs *pGPIOx, Status status)
{
	switch(status)
	{
	case Enable:
		if(pGPIOx == GPIOA)
			GPIOA_CLK_EN();
		else if(pGPIOx == GPIOB)
			GPIOB_CLK_EN();
		else if(pGPIOx == GPIOC)
			GPIOC_CLK_EN();
		else if(pGPIOx == GPIOD)
			GPIOD_CLK_EN();
		else if(pGPIOx == GPIOE)
			GPIOE_CLK_EN();
		break;
	case Disable:
		if(pGPIOx == GPIOA)
			GPIOA_CLK_DI();
		else if(pGPIOx == GPIOB)
			GPIOB_CLK_DI();
		else if(pGPIOx == GPIOC)
			GPIOC_CLK_DI();
		else if(pGPIOx == GPIOD)
			GPIOD_CLK_DI();
		else if(pGPIOx == GPIOE)
			GPIOE_CLK_DI();
		break;
	}
}

/* init and reset to the reset value */

/*
 * @fun_name:  GPIO_Init
 * @fun_brief: this is a function to initialize the GPIO port needed
 * @param[in]: the structure of the GPIO_Handle
 * @return:    none
 * @notes:     none
 */
void GPIO_Init(GPIO_Handle *pGPIOx)
{
	volatile uint32_t val = 0;
    volatile uint8_t flag = 0, Reg_Sel = 0, Exti_Sel = 0;

	// configure the input mode
	if(pGPIOx->Pin_Config.GPIO_PinMode == GPIO_IN || pGPIOx->Pin_Config.GPIO_PinMode == GPIO_EXTI_FT || pGPIOx->Pin_Config.GPIO_PinMode == GPIO_EXTI_RT || pGPIOx->Pin_Config.GPIO_PinMode == GPIO_EXTI_RFT)
	{
		    // it will be executed whatever the mode of input you need
			if(pGPIOx->Pin_Config.GPIO_PinNum <= 7)
			{
				val = (0x3 << ((4 * pGPIOx->Pin_Config.GPIO_PinNum) + 2));
				pGPIOx->GPIOx->CRL &= ~val;
				val = (pGPIOx->Pin_Config.GPIO_PinInputMode << ((4 * pGPIOx->Pin_Config.GPIO_PinNum) + 2));
				pGPIOx->GPIOx->CRL |= val;
			}
			else if(pGPIOx->Pin_Config.GPIO_PinNum > 7 && pGPIOx->Pin_Config.GPIO_PinNum <= 15)
			{
				val = (0x3 << ((4 * pGPIOx->Pin_Config.GPIO_PinNum) + 2));
				pGPIOx->GPIOx->CRL &= ~val;
				val = (pGPIOx->Pin_Config.GPIO_PinInputMode  << ((4 * (pGPIOx->Pin_Config.GPIO_PinNum - 8)) + 2));
				pGPIOx->GPIOx->CRH |= val;
			}

			// configure the pull-up and down
			if(pGPIOx->Pin_Config.GPIO_PinInputMode == GPIO_IN_PULUD)
			{
				if(pGPIOx->Pin_Config.GPIO_PinPUD_Mode == GPIO_IN_PULUP)
				      pGPIOx->GPIOx->ODR |= (1 << pGPIOx->Pin_Config.GPIO_PinNum);
				else
				      pGPIOx->GPIOx->ODR &= ~(1 << pGPIOx->Pin_Config.GPIO_PinNum);
		    }

			// it will be executed only if the mode of input relates to EXTI
			if(pGPIOx->Pin_Config.GPIO_PinMode == GPIO_EXTI_FT || pGPIOx->Pin_Config.GPIO_PinMode == GPIO_EXTI_RT || pGPIOx->Pin_Config.GPIO_PinMode == GPIO_EXTI_RFT)
			{
				// configuring the edge triggering mode
				switch(pGPIOx->Pin_Config.GPIO_PinMode)
				{
				case GPIO_EXTI_FT:
					EXTI->EXTI_FTSR |= (1 << pGPIOx->Pin_Config.GPIO_PinNum);
					EXTI->EXTI_RTSR &= ~(1 << pGPIOx->Pin_Config.GPIO_PinNum);
					break;
				case GPIO_EXTI_RT:
					EXTI->EXTI_RTSR |= (1 << pGPIOx->Pin_Config.GPIO_PinNum);
					EXTI->EXTI_FTSR &= ~(1 << pGPIOx->Pin_Config.GPIO_PinNum);
					break;
				case GPIO_EXTI_RFT:
					EXTI->EXTI_FTSR |= (1 << pGPIOx->Pin_Config.GPIO_PinNum);
					EXTI->EXTI_RTSR |= (1 << pGPIOx->Pin_Config.GPIO_PinNum);
					break;
				}

				// configuring the GPIO port selection
				Reg_Sel = pGPIOx->Pin_Config.GPIO_PinNum / 4;
				Exti_Sel = pGPIOx->Pin_Config.GPIO_PinNum % 4;
				pGPIOx->Pin_Config.GPIO_AFIO.EXTICR[Reg_Sel] &= ~(0xF << (Exti_Sel * 4));

					if(pGPIOx->GPIOx == GPIOA)
					   pGPIOx->Pin_Config.GPIO_AFIO.EXTICR[Reg_Sel] |= (PA << (Exti_Sel * 4));
					else if(pGPIOx->GPIOx == GPIOB)
					   pGPIOx->Pin_Config.GPIO_AFIO.EXTICR[Reg_Sel] |= (PB << (Exti_Sel * 4));
					else if(pGPIOx->GPIOx == GPIOC)
					   pGPIOx->Pin_Config.GPIO_AFIO.EXTICR[Reg_Sel] |= (PC << (Exti_Sel * 4));
					else if(pGPIOx->GPIOx == GPIOD)
					   pGPIOx->Pin_Config.GPIO_AFIO.EXTICR[Reg_Sel] |= (PD << (Exti_Sel * 4));
					else if(pGPIOx->GPIOx == GPIOE)
					   pGPIOx->Pin_Config.GPIO_AFIO.EXTICR[Reg_Sel] |= (PE << (Exti_Sel * 4));
					else if(pGPIOx->GPIOx == GPIOF)
					   pGPIOx->Pin_Config.GPIO_AFIO.EXTICR[Reg_Sel] |= (PF << (Exti_Sel * 4));
					else if(pGPIOx->GPIOx == GPIOG)
					   pGPIOx->Pin_Config.GPIO_AFIO.EXTICR[Reg_Sel] |= (PG << (Exti_Sel * 4));


				// enable the interrupt delivery from the EXTI line
					EXTI->EXTI_IMR |= (1 << pGPIOx->Pin_Config.GPIO_PinNum);
				}

			flag = 1;

	}

	// reset the value to 0
	val = 0;

	// configure the output mode
	if((pGPIOx->Pin_Config.GPIO_PinOutputMode == GPIO_OUT_PP || pGPIOx->Pin_Config.GPIO_PinOutputMode == GPIO_OUT_OD || pGPIOx->Pin_Config.GPIO_PinOutputMode == GPIO_OUT_AFPP || pGPIOx->Pin_Config.GPIO_PinOutputMode == GPIO_OUT_AFOD) && flag == 0)
	{
		if(pGPIOx->Pin_Config.GPIO_PinNum <= 7)
		{
			            val = (0x3 << ((4 * pGPIOx->Pin_Config.GPIO_PinNum) + 2));
			            pGPIOx->GPIOx->CRL &= ~val;
						val = (pGPIOx->Pin_Config.GPIO_PinOutputMode << ((4 * pGPIOx->Pin_Config.GPIO_PinNum) + 2));
						pGPIOx->GPIOx->CRL |= val;
		}
	    else if(pGPIOx->Pin_Config.GPIO_PinNum > 7 && pGPIOx->Pin_Config.GPIO_PinNum <= 15)
		{
			    	   val = (0x3 << ((4 * pGPIOx->Pin_Config.GPIO_PinNum) + 2));
			    	   pGPIOx->GPIOx->CRL &= ~val;
					    val = (pGPIOx->Pin_Config.GPIO_PinOutputMode  << ((4 * (pGPIOx->Pin_Config.GPIO_PinNum - 8)) + 2));
						pGPIOx->GPIOx->CRH |= val;
		}

	if(pGPIOx->Pin_Config.GPIO_PinMode == GPIO_OUT_2MHz || pGPIOx->Pin_Config.GPIO_PinMode == GPIO_OUT_10MHz || pGPIOx->Pin_Config.GPIO_PinMode == GPIO_OUT_50MHz)
	{
		if(pGPIOx->Pin_Config.GPIO_PinNum <= 7)
		{
				val = (pGPIOx->Pin_Config.GPIO_PinMode << ((4 * pGPIOx->Pin_Config.GPIO_PinNum)));
				pGPIOx->GPIOx->CRL |= val;
		}
	    else if(pGPIOx->Pin_Config.GPIO_PinNum > 7 && pGPIOx->Pin_Config.GPIO_PinNum <= 15)
		{
			    val = (pGPIOx->Pin_Config.GPIO_PinMode  << ((4 * (pGPIOx->Pin_Config.GPIO_PinNum - 8))));
				pGPIOx->GPIOx->CRH |= val;
	    }
	}
	}

	if(pGPIOx->Pin_Config.GPIO_PinOutputMode == GPIO_OUT_AFPP || pGPIOx->Pin_Config.GPIO_PinOutputMode == GPIO_OUT_AFOD)
	{
		// configure the atlernate function registers.
	}

	// configure the mode and speed in case of output mode of the GPIO pin
		if(pGPIOx->Pin_Config.GPIO_PinMode <= 3)
		{
			if(pGPIOx->Pin_Config.GPIO_PinNum <= 7)
			{
				val = (pGPIOx->Pin_Config.GPIO_PinMode << (4 * pGPIOx->Pin_Config.GPIO_PinNum));
				pGPIOx->GPIOx->CRL |= val;
			}
			else if(pGPIOx->Pin_Config.GPIO_PinNum > 7 && pGPIOx->Pin_Config.GPIO_PinNum <= 15)
			{
				val = (pGPIOx->Pin_Config.GPIO_PinMode << (4 * (pGPIOx->Pin_Config.GPIO_PinNum - 8)));
				pGPIOx->GPIOx->CRH |= val;
			}
		}

		// reset the value to 0
		val = 0;


}

/*
 * @fun_name:  GPIO_Deinit
 * @fun_brief: this is a function to reset registers for the GPIO port needed to the reset value
 * @param[in]: the base address for the specific GPIO port you need to enable the clock for
 * @return:    none
 * @notes:     none
 */
void GPIO_Deinit(GPIO_REGs *pGPIOx)
{
	if(pGPIOx == GPIOA)
          GPIOA_RESET();
	else if(pGPIOx == GPIOB)
		  GPIOB_RESET();
	else if(pGPIOx == GPIOC)
		  GPIOC_RESET();
	else if(pGPIOx == GPIOD)
		  GPIOD_RESET();
	else if(pGPIOx == GPIOE)
		  GPIOE_RESET();

}

/* GPIO read and write */

/*
 * @fun_name:  GPIO_WriteToPin
 * @fun_brief: this is a function to write on the specified pin given
 * @param[in]: the base addresses for the GPIO registers, the pin number, and the status rather HIGH or LOW
 * @return:    none
 * @notes:     none
 */
void GPIO_WriteToPin(GPIO_REGs *pGPIOx, uint8_t PinNum, uint8_t data)
{
	if(data == 1)
		pGPIOx->ODR |= (1 << PinNum);
	else if(data == 0)
		pGPIOx->ODR &= ~(1 << PinNum);

}

/*
 * @fun_name:  GPIO_ReadFromPin
 * @fun_brief: this is a function to read from the specified pin given
 * @param[in]: the base addresses for the GPIO registers, the pin number
 * @return:    the value entered from the out world to the pin specified
 * @notes:     none
 */
uint8_t GPIO_ReadFromPin(GPIO_REGs *pGPIOx, uint8_t PinNum)
{
	volatile uint8_t val = (uint8_t)((pGPIOx->IDR >> PinNum) & 0x00000001);
	return val;
}

/*
 * @fun_name:  GPIO_WriteToPort
 * @fun_brief: this is a function to write on the specified port given
 * @param[in]: the base addresses for the GPIO registers, and the data to write
 * @return:    none
 * @notes:     none
 */
void GPIO_WriteToPort(GPIO_REGs  *pGPIOx, uint16_t data)
{
	pGPIOx->ODR = data;
}

/*
 * @fun_name:  GPIO_ReadFromPort
 * @fun_brief: this is a function to read from the specified port given
 * @param[in]: the base addresses for the GPIO registers
 * @return:    the data received from the out world
 * @notes:     none
 */
uint16_t GPIO_ReadFromPort(GPIO_REGs *pGPIOx)
{
	uint16_t val = (uint16_t)pGPIOx->IDR;
	return val;
}

/*
 * @fun_name:  GPIO_ToggleOutputPin
 * @fun_brief: this is a function to toggle the pin specified
 * @param[in]: the base addresses for the GPIO registers, and the pin to toggle
 * @return:    none
 * @notes:     none
 */
void GPIO_ToggleOutputPin(GPIO_REGs *pGPIOx, uint8_t PinNum)
{
	pGPIOx->ODR ^= (1 << PinNum);
}

/* GPIO IRQs handling */

/*
 * @fun_name:  GPIO_IRQEnable
 * @fun_brief: this is a function to configure the exceptions due to the status sent
 * @param[in]: the IRQ number, and the status
 * @return:    none
 * @notes:     none
 */
void GPIO_IRQEnable(uint8_t IRQ, Status status)
{
	uint8_t nvic_reg = IRQ / 32, nvic_irq = IRQ % 32;
	// enable or disable the IRQ in the NVIC registers
	switch(status)
	{
	case Enable:
		*(ISER0 + nvic_reg) |= (1 << nvic_irq);
		break;
	case Disable:
		*(ICER0+ nvic_reg) |= (1 << nvic_irq);
	}

}

/*
 * @fun_name:  GPIO_IRQ_Priority
 * @fun_brief: this is a function to configure the exceptions prioriry
 * @param[in]: the IRQ number, and the priority
 * @return:    none
 * @notes:     none
 */
void GPIO_IRQ_Priority(uint8_t IRQ, uint8_t priority)
{
	uint8_t iprx = IRQ / 4, iprx_slot = IRQ % 4;

	*(IPR0 + iprx) &= ~(0xFF << (iprx_slot * 8));
	*(IPR0 + iprx) |= (priority << (iprx_slot * 8));
}

/*
 * @fun_name:  GPIO_IRQHandling
 * @fun_brief: this is a function to handle the exception came from the out world
 * @param[in]: the pin number
 * @return:    none
 * @notes:     none
 */
void GPIO_IRQHandling(uint8_t PinNum)
{
	if(EXTI->EXTI_PR & (1 << PinNum))
		EXTI->EXTI_PR |= (1 << PinNum);
}

