#ifndef INC_STM32F10XX_H_
#define INC_STM32F10XX_H_
#include <stdint.h>

/*****************************MPU-PERIPHERALS**************************/
/* NVIC */
#define ISER0 ((volatile uint32_t *)0xE000E100)
#define ICER0 ((volatile uint32_t *)0xE000E180)
#define ISPR0 ((volatile uint32_t *)0xE000E200)
#define ICPR0 ((volatile uint32_t *)0xE000E280)
#define IABR0 ((volatile uint32_t *)0xE000E300)
#define IPR0  ((volatile uint32_t *)0xE000E400)
#define STIR  ((volatile uint32_t *)0xE000EF00)


/* faults handling */
#define SHCSR ((volatile uint32_t*)0xE000ED24)
#define MEMF_ACT    0
#define BUSF_ACT    1
#define USGF_ACT    3
#define SVCACT      7
#define MONITOR_ACT 8
#define PENDSV_ACT  10
#define SYSTICK_ACT 11
#define USGF_PEND   12
#define MEMF_PEND   13
#define BUSF_PEND   14
#define SVC_PEND    15
#define MEMFENA     16
#define BUSFENA     17
#define USGFENA     18

#define MMSR ((volatile uint8_t*)0xE000ED28)
#define BFSR ((volatile uint8_t*)0xE000ED29)
#define UFSR ((volatile uint16_t*)0xE000ED2A)
#define HFSR ((volatile uint32_t*)0xE000ED2C)

/**********************************************************************/

/* base addresses for different memories */
#define SRAM         0x20000000U
#define FLASH        0x08000000U
#define ROM          0x1FFFF800U

/* base addresses for AHB & APB busses */
#define APB1_BASE         0x40000000U
#define APB2_BASE         0x40010000U
#define AHB_BASE          0x40018000U

/* the base addresses for different peripherals */
/* APB 1 */
#define TIM2_BASE                       (APB1_BASE + 0x0000)
#define TIM3_BASE                       (APB1_BASE + 0x0400)
#define TIM4_BASE                       (APB1_BASE + 0x0800)
#define TIM5_BASE                       (APB1_BASE + 0x0C00)
#define TIM6_BASE                       (APB1_BASE + 0x1000)
#define TIM7_BASE                       (APB1_BASE + 0x1400)
#define TIM12_BASE                      (APB1_BASE + 0x1800)
#define TIM13_BASE                      (APB1_BASE + 0x1C00)
#define TIM14_BASE                      (APB1_BASE + 0x2000)
#define RTC_BASE                        (APB1_BASE + 0x2800)
#define WWDG_BASE                       (APB1_BASE + 0x2C00)
#define IWDG_BASE                       (APB1_BASE + 0x3000)
#define SPI2_I2S_BASE                   (APB1_BASE + 0x3800)
#define SPI3_I2S_BASE                   (APB1_BASE + 0x3C00)
#define USART2_BASE                     (APB1_BASE + 0x4400)
#define USART3_BASE                     (APB1_BASE + 0x4800)
#define UART4_BASE                      (APB1_BASE + 0x4C00)
#define UART5_BASE                      (APB1_BASE + 0x5000)
#define I2C1_BASE                       (APB1_BASE + 0x5400)
#define I2C2_BASE                       (APB1_BASE + 0x5800)
#define USBFS_REG_BASE                  (APB1_BASE + 0x5C00)
#define SHRDUSB_CAN_SRAM_BASE           (APB1_BASE + 0x6000)
#define BXCAN1_BASE                     (APB1_BASE + 0x6400)
#define BXCAN2_BASE                     (APB1_BASE + 0x6800)
#define BKP_BASE                        (APB1_BASE + 0x6C00)
#define PWR_BASE                        (APB1_BASE + 0x7000)
#define DAC_BASE                        (APB1_BASE + 0x7400)

/* APB2 */
#define AFIO_BASE                       (APB2_BASE + 0x0000)
#define EXTI_BASE                       (APB2_BASE + 0x0400)
#define GPIOA_BASE                      (APB2_BASE + 0x0800)
#define GPIOB_BASE                      (APB2_BASE + 0x0C00)
#define GPIOC_BASE                      (APB2_BASE + 0x1000)
#define GPIOD_BASE                      (APB2_BASE + 0x1400)
#define GPIOE_BASE                      (APB2_BASE + 0x1800)
#define GPIOF_BASE                      (APB2_BASE + 0x1C00)
#define GPIOG_BASE                      (APB2_BASE + 0x2000)
#define ADC1_BASE                       (APB2_BASE + 0x2400)
#define ADC2_BASE                       (APB2_BASE + 0x2800)
#define TIM1_BASE                       (APB2_BASE + 0x2C00)
#define SPI1_BASE                       (APB2_BASE + 0x3000)
#define TIM8_BASE                       (APB2_BASE + 0x3400)
#define USART1_BASE                     (APB2_BASE + 0x3800)
#define ADC3_BASE                       (APB2_BASE + 0x3C00)
#define TIM9_BASE                       (APB2_BASE + 0x4C00)
#define TIM10_BASE                      (APB2_BASE + 0x5000)
#define TIM11_BASE                      (APB2_BASE + 0x5400)

/* AHB */
#define SDIO_BASE                       (AHB_BASE + 0x00000000)
#define DMA1_BASE                       (AHB_BASE + 0x00008000)
#define DMA2_BASE                       (AHB_BASE + 0x00008400)
#define RCC_BASE                        (AHB_BASE + 0x00009000)
#define FLASHINT_BASE                   (AHB_BASE + 0x0000A000)
#define CRC_BASE                        (AHB_BASE + 0x0000B000)
#define EATHERNET_BASE                  (AHB_BASE + 0x00010000)
#define USBOTG_FS_BASE                  (AHB_BASE + 0x0FFE8000)
#define FSMC_BASE                       (AHB_BASE + 0x5FFE8000)

/* GPIOs */
typedef struct{
	  volatile uint32_t CRL; /*Port configuration register low, offset: 0x00*/
	  volatile uint32_t CRH; /*Port configuration register high, offset: 0x04*/
	  volatile uint32_t IDR; /*Port input data register, offset: 0x08*/
	  volatile uint32_t ODR; /*Port output data register, offset: 0x0C*/
	  volatile uint32_t BSRR; /*Port bit set/reset register, offset: 0x10*/
	  volatile uint32_t BRR; /*Port bit reset register, offset: 0x14*/
	  volatile uint32_t LCKR; /*Port configuration lock register, offset: 0x18*/
}GPIO_REGs;

#define GPIOA  ((GPIO_REGs *)GPIOA_BASE)
#define GPIOB  ((GPIO_REGs *)GPIOB_BASE)
#define GPIOC  ((GPIO_REGs *)GPIOC_BASE)
#define GPIOD  ((GPIO_REGs *)GPIOD_BASE)
#define GPIOE  ((GPIO_REGs *)GPIOE_BASE)
#define GPIOF  ((GPIO_REGs *)GPIOF_BASE)
#define GPIOG  ((GPIO_REGs *)GPIOG_BASE)

/* resetting the GPIO registers */
#define GPIOA_RESET()               do{(RCC->RCC_APB2RSTR |= (1 << 2)); (RCC->RCC_APB2RSTR &= ~(1 << 2));}while(0)
#define GPIOB_RESET()               do{(RCC->RCC_APB2RSTR |= (1 << 3)); (RCC->RCC_APB2RSTR &= ~(1 << 3));}while(0)
#define GPIOC_RESET()               do{(RCC->RCC_APB2RSTR |= (1 << 4)); (RCC->RCC_APB2RSTR &= ~(1 << 4));}while(0)
#define GPIOD_RESET()               do{(RCC->RCC_APB2RSTR |= (1 << 5)); (RCC->RCC_APB2RSTR &= ~(1 << 5));}while(0)
#define GPIOE_RESET()               do{(RCC->RCC_APB2RSTR |= (1 << 6)); (RCC->RCC_APB2RSTR &= ~(1 << 6));}while(0)

#define HIGH   1
#define LOW    0

typedef struct{
	  volatile uint32_t EVCR;         /*Offset: 0x00 >> Event control register*/
	  volatile uint32_t MAPR;         /*Offset: 0x04 >> AF remap and debug I/O configuration register*/
	  volatile uint32_t EXTICR[4];      /*Offset: 0x08 >> External interrupt configuration registers 1, 2, 3, and 4*/
	  volatile uint32_t reserved;
	  volatile uint32_t MAPR2;        /*Offset: 0x1C >> AF remap and debug I/O configuration register2*/
}AFIO_REGs;

#define AFIO  (      (AFIO_REGs *)AFIO_BASE)


/* RCC */
typedef struct{
	volatile uint32_t RCC_CR;          /*Offset: 0x00 >> RCC clock control register*/
	volatile uint32_t RCC_CFGR;        /*Offset: 0x04 >> RCC clock configuration register*/
	volatile uint32_t RCC_CIR;         /*Offset: 0x08 >> RCC clock interrupt register*/
	volatile uint32_t RCC_APB2RSTR;    /*Offset: 0x0C >> APB2 peripheral reset register*/
	volatile uint32_t RCC_APB1RSTR;    /*Offset: 0x10 >> APB1 peripheral reset register*/
	volatile uint32_t RCC_AHBENR;      /*Offset: 0x14 >> AHB Peripheral Clock enable register*/
	volatile uint32_t RCC_APB2ENR;     /*Offset: 0x18 >> APB2 Peripheral Clock enable register*/
	volatile uint32_t RCC_APB1ENR;     /*Offset: 0x1C >> APB1 Peripheral Clock enable register*/
	volatile uint32_t RCC_BDCR;        /*Offset: 0x20 >> Backup domain control register*/
	volatile uint32_t RCC_CSR;         /*Offset: 0x24 >> Control/status registe*/
	volatile uint32_t RCC_AHBSTR;      /*Offset: 0x28 >> AHB peripheral clock reset register*/
	volatile uint32_t RCC_CFGR2;       /*Offset: 0x2C >> Clock configuration register2 */
}RCC_REGs;

#define RCC          ((RCC_REGs *)RCC_BASE)

typedef enum{
	Disable = 0,
	Enable
}Status;


/* the struct for the EXTI registers*/
typedef struct{
	volatile uint32_t EXTI_IMR;       /*Offset: 0x00 >> Interrupt mask register*/
	volatile uint32_t EXTI_EMR;       /*Offset: 0x04 >> Event mask register*/
	volatile uint32_t EXTI_RTSR;      /*Offset: 0x08 >> Rising trigger selection register*/
	volatile uint32_t EXTI_FTSR;      /*Offset: 0x0C >> EFalling trigger selection register*/
	volatile uint32_t EXTI_SWIER;     /*Offset: 0x10 >> Software interrupt event register*/
	volatile uint32_t EXTI_PR;        /*Offset: 0x14 >> Pending register*/
}EXTI_REGs;

#define EXTI         ((EXTI_REGs *)EXTI_BASE)

#define EXTI0_IRQ        6
#define EXTI1_IRQ        7
#define EXTI2_IRQ        8
#define EXTI3_IRQ        9
#define EXTI4_IRQ        10
#define EXTI5_9_IRQ      23
#define EXTI10_15_IRQ    40


/* the macros for enabling and disabling the clocks for each peripheral */
/* enabling */
#define GPIOA_CLK_EN()              (RCC->RCC_APB2ENR |= (1 << 2))
#define GPIOB_CLK_EN()              (RCC->RCC_APB2ENR |= (1 << 3))
#define GPIOC_CLK_EN()              (RCC->RCC_APB2ENR |= (1 << 4))
#define GPIOD_CLK_EN()              (RCC->RCC_APB2ENR |= (1 << 5))
#define GPIOE_CLK_EN()              (RCC->RCC_APB2ENR |= (1 << 6))

#define AFIO_CLK_EN()               (RCC->RCC_APB2ENR |= (1 << 0))

#define TIM1_CLK_EN()               (RCC->RCC_APB2ENR |= (1 << 11))
#define TIM2_CLK_EN()               (RCC->RCC_APB1ENR |= (1 << 0))
#define TIM3_CLK_EN()               (RCC->RCC_APB1ENR |= (1 << 1))
#define TIM4_CLK_EN()               (RCC->RCC_APB1ENR |= (1 << 2))
#define TIM5_CLK_EN()               (RCC->RCC_APB1ENR |= (1 << 3))
#define TIM6_CLK_EN()               (RCC->RCC_APB1ENR |= (1 << 4))
#define TIM7_CLK_EN()               (RCC->RCC_APB1ENR |= (1 << 5))
#define WWDGRST_CLK_EN()            (RCC->RCC_APB1ENR |= (1 << 11))

#define SPI1_CLK_EN()               (RCC->RCC_APB2ENR |= (1 << 12))
#define SPI2_CLK_EN()               (RCC->RCC_APB1ENR |= (1 << 14))
#define SPI3_CLK_EN()               (RCC->RCC_APB1ENR |= (1 << 15))

#define USART1_CLK_EN()             (RCC->RCC_APB2ENR |= (1 << 14))
#define USART2_CLK_EN()             (RCC->RCC_APB1ENR |= (1 << 17))
#define USART3_CLK_EN()             (RCC->RCC_APB1ENR |= (1 << 18))
#define UART4_CLK_EN()              (RCC->RCC_APB1ENR |= (1 << 19))
#define UART5_CLK_EN()              (RCC->RCC_APB1ENR |= (1 << 20))

#define I2C1_CLK_EN()               (RCC->RCC_APB1ENR |= (1 << 21))
#define I2C2_CLK_EN()               (RCC->RCC_APB1ENR |= (1 << 22))

#define CAN1_CLK_EN()               (RCC->RCC_APB1ENR |= (1 << 25))
#define CAN2_CLK_EN()               (RCC->RCC_APB1ENR |= (1 << 26))

#define ADC1_CLK_EN()               (RCC->RCC_APB2ENR |= (1 << 9))
#define ADC2_CLK_EN()               (RCC->RCC_APB2ENR |= (1 << 10))

#define BKP_CLK_EN()                (RCC->RCC_APB1ENR |= (1 << 27))
#define PWR_CLK_EN()                (RCC->RCC_APB1ENR |= (1 << 28))
#define DAC_CLK_EN()                (RCC->RCC_APB1ENR |= (1 << 29))

#define DMA1_CLK_EN()               (RCC->RCC_AHBENR |= (1 << 0))
#define DMA2_CLK_EN()               (RCC->RCC_AHBENR |= (1 << 1))
#define SRAM_CLK_EN()               (RCC->RCC_AHBENR |= (1 << 2))
#define FLITF_CLK_EN()              (RCC->RCC_AHBENR |= (1 << 4))
#define CRC_CLK_EN()                (RCC->RCC_AHBENR |= (1 << 6))
#define OTGFS_CLK_EN()              (RCC->RCC_AHBENR |= (1 << 12))
#define ETHMAC_CLK_EN()             (RCC->RCC_AHBENR |= (1 << 14))
#define ETHMACTX_CLK_EN()           (RCC->RCC_AHBENR |= (1 << 15))
#define ETHMACRX_CLK_EN()           (RCC->RCC_AHBENR |= (1 << 16))



/* disabling */
#define GPIOA_CLK_DI()              (RCC->RCC_APB2ENR &= ~(1 << 2))
#define GPIOB_CLK_DI()              (RCC->RCC_APB2ENR &= ~(1 << 3))
#define GPIOC_CLK_DI()              (RCC->RCC_APB2ENR &= ~(1 << 4))
#define GPIOD_CLK_DI()              (RCC->RCC_APB2ENR &= ~(1 << 5))
#define GPIOE_CLK_DI()              (RCC->RCC_APB2ENR &= ~(1 << 6))

#define AFIO_CLK_DI()               (RCC->RCC_APB2ENR &= ~(1 << 0))

#define TIM1_CLK_DI()               (RCC->RCC_APB2ENR &= ~(1 << 11))
#define TIM2_CLK_DI()               (RCC->RCC_APB1ENR &= ~(1 << 0))
#define TIM3_CLK_DI()               (RCC->RCC_APB1ENR &= ~(1 << 1))
#define TIM4_CLK_DI()               (RCC->RCC_APB1ENR &= ~(1 << 2))
#define TIM5_CLK_DI()               (RCC->RCC_APB1ENR &= ~(1 << 3))
#define TIM6_CLK_DI()               (RCC->RCC_APB1ENR &= ~(1 << 4))
#define TIM7_CLK_DI()               (RCC->RCC_APB1ENR &= ~(1 << 5))
#define WWDGRST_CLK_DI()            (RCC->RCC_APB1ENR &= ~(1 << 11))

#define SPI1_CLK_DI()               (RCC->RCC_APB2ENR &= ~(1 << 12))
#define SPI2_CLK_DI()               (RCC->RCC_APB1ENR &= ~(1 << 14))
#define SPI3_CLK_DI()               (RCC->RCC_APB1ENR &= ~(1 << 15))

#define USART1_CLK_DI()             (RCC->RCC_APB2ENR &= ~(1 << 14))
#define USART2_CLK_DI()             (RCC->RCC_APB1ENR &= ~(1 << 17))
#define USART3_CLK_DI()             (RCC->RCC_APB1ENR &= ~(1 << 18))
#define UART4_CLK_DI()              (RCC->RCC_APB1ENR &= ~(1 << 19))
#define UART5_CLK_DI()              (RCC->RCC_APB1ENR &= ~(1 << 20))

#define I2C1_CLK_DI()               (RCC->RCC_APB1ENR &= ~(1 << 21))
#define I2C2_CLK_DI()               (RCC->RCC_APB1ENR &= ~(1 << 22))

#define CAN1_CLK_DI()               (RCC->RCC_APB1ENR &= ~(1 << 25))
#define CAN2_CLK_DI()               (RCC->RCC_APB1ENR &= ~(1 << 26))

#define ADC1_CLK_DI()               (RCC->RCC_APB2ENR &= ~(1 << 9))
#define ADC2_CLK_DI()               (RCC->RCC_APB2ENR &= ~(1 << 10))

#define BKP_CLK_DI()                (RCC->RCC_APB1ENR &= ~(1 << 27))
#define PWR_CLK_DI()                (RCC->RCC_APB1ENR &= ~(1 << 28))
#define DAC_CLK_DI()                (RCC->RCC_APB1ENR &= ~(1 << 29))

#define DMA1_CLK_DI()               (RCC->RCC_AHBENR &= ~(1 << 0))
#define DMA2_CLK_DI()               (RCC->RCC_AHBENR &= ~(1 << 1))
#define SRAM_CLK_DI()               (RCC->RCC_AHBENR &= ~(1 << 2))
#define FLITF_CLK_DI()              (RCC->RCC_AHBENR &= ~(1 << 4))
#define CRC_CLK_DI()                (RCC->RCC_AHBENR &= ~(1 << 6))
#define OTGFS_CLK_DI()              (RCC->RCC_AHBENR &= ~(1 << 12))
#define ETHMAC_CLK_DI()             (RCC->RCC_AHBENR &= ~(1 << 14))
#define ETHMACTX_CLK_DI()           (RCC->RCC_AHBENR &= ~(1 << 15))
#define ETHMACRX_CLK_DI()           (RCC->RCC_AHBENR &= ~(1 << 16))




#endif /* INC_STM32F10XX_H_ */
