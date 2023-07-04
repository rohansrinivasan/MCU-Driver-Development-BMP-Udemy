/*
 * stm32f407xx.h
 *
 *  Created on: May 12, 2023
 *      Author: Rohan
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define _vo volatile


/********************************** START: Processor Specific Details **********************************/

//ARM Cortex M4 Processor NVIC ISERx Register Addresses
#define NVIC_ISER0		((_vo uint32_t *)0xE000E100)
#define NVIC_ISER1		((_vo uint32_t *)0xE000E104)
#define NVIC_ISER2		((_vo uint32_t *)0xE000E108)
#define NVIC_ISER3		((_vo uint32_t *)0xE000E10C)
#define NVIC_ISER4		((_vo uint32_t *)0xE000E110)
#define NVIC_ISER5		((_vo uint32_t *)0xE000E114)
#define NVIC_ISER6		((_vo uint32_t *)0xE000E118)
#define NVIC_ISER7		((_vo uint32_t *)0xE000E11C)

//ARM Cortex M4 Processor NVIC ICERx Register Addresses
#define NVIC_ICER0		((_vo uint32_t *)0xE000E180)
#define NVIC_ICER1		((_vo uint32_t *)0xE000E184)
#define NVIC_ICER2		((_vo uint32_t *)0xE000E188)
#define NVIC_ICER3		((_vo uint32_t *)0xE000E18C)
#define NVIC_ICER4		((_vo uint32_t *)0xE000E190)
#define NVIC_ICER5		((_vo uint32_t *)0xE000E194)
#define NVIC_ICER6		((_vo uint32_t *)0xE000E198)
#define NVIC_ICER7		((_vo uint32_t *)0xE000E19C)

//ARM Cortex M4 Processor Priority Register Base Address
#define NVIC_PR_BASEADDR		((_vo uint32_t *)0xE000E400)


/********************************** START: MCU Specific Details **********************************/

//base addresses of Flash and SRAM memories
#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x20000000U //112KB
#define SRAM2_BASEADDR			0x20001C00U //112 *  1024 (1024 bits = 1 byte)
#define ROM						0x1FFF0000U
#define SRAM 					SRAM1_BASEADDR

//base addresses of bus domains (APBx, AHBx etc.)
#define  PERIPH_BASEADDR		0x40000000U
#define  APB1PERIPH_BASEADDR	PERIPH_BASEADDR
#define  APB2PERIPH_BASEADDR	0x40010000U
#define	 AHB1PERIPH_BASEADDR	0x40020000U
#define  AHB2PERIPH_BASEADDR	0x50000000U

//base addresses of peripherals which are hanging on the AHB1 bus
#define GPIOA_BASEAADDR			(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEAADDR			(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEAADDR			(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEAADDR			(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEAADDR			(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEAADDR			(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEAADDR			(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEAADDR			(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEAADDR			(AHB1PERIPH_BASEADDR + 0x2000)
#define GPIOJ_BASEAADDR			(AHB1PERIPH_BASEADDR + 0x2400)
#define GPIOK_BASEAADDR			(AHB1PERIPH_BASEADDR + 0x2800)
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800)

//base addresses of peripherals hanging on APB1 bus
//I2C
#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR 			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR 			(APB1PERIPH_BASEADDR + 0x5C00)
//SPI
#define SPI2_BASEADDR 			(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR 			(APB1PERIPH_BASEADDR + 0x3C00)
//U(S)ART
#define USART2_BASEADDR 		(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR 		(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR 			(APB1PERIPH_BASEADDR + 0x4c00)
#define UART5_BASEADDR 			(APB1PERIPH_BASEADDR + 0x5000)

//base addresses of peripherals hanging on APB2 bus
#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR 			(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR			(APB2PERIPH_BASEADDR + 0x3400)
#define SYSCFG_BASEADDR 		(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR 		(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR 		(APB2PERIPH_BASEADDR + 0x1400)


/********************************** PERIPHERAL REGISTER DEFINITION STRUCTURE **********************************/
/*
 * Note : Registers of a peripheral are specific to MCU
 * e.g : Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different(more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your Device RM
 */

/********************************** GPIO *********************************/
typedef struct
{
	_vo uint32_t MODER;     /* GPIO port mode register (Address offset: 0x00) */
	_vo uint32_t OTYPER;    /* GPIO port output type register (Address offset: 0x04) */
	_vo uint32_t OSPEEDR;	/* GPIO port output speed register (Address offset: 0x08) */
	_vo uint32_t PUPDR;		/* GPIO port pull-up/pull-down register (Address offset: 0x0C) */
	_vo uint32_t IDR;		/* GPIO port input data register (Address offset: 0x10) */
	_vo uint32_t ODR;		/* GPIO port output data register (Address offset: 0x14) */
	_vo uint32_t BSRR;		/* GPIO port bit set/reset register (Address offset: 0x18) */
	_vo uint32_t LCKR;		/* GPIO port configuration lock register (Address offset: 0x1C) */
	_vo uint32_t AFR[2];	/* GPIO port alternate function register:
							AFR[0] : alternate function low register (Address offset: 0x20),
							AFR[1] : alternate function high register (Address offset: 0x24) */
} GPIO_RegDef_t;


/********************************* RCC ***********************************/
typedef struct
{
	_vo uint32_t RCC_CR;			/* RCC clock control register (Address offset: 0x00) */
	_vo uint32_t RCC_PLLCFGR;   	/* RCC PLL configuration register (Address offset: 0x04) */
	_vo uint32_t RCC_CFGR;			/* RCC clock configuration register (Address offset: 0x08) */
	_vo uint32_t RCC_CIR;			/* RCC clock interrupt register (Address offset: 0x0C) */
	_vo uint32_t RCC_AHB1RSTR;		/* RCC AHB1 peripheral reset register (Address offset: 0x10) */
	_vo uint32_t RCC_AHB2RSTR;		/* RCC AHB2 peripheral reset register (Address offset: 0x14) */
	_vo uint32_t RCC_AHB3RSTR;		/* RCC AHB3 peripheral reset register (Address offset: 0x18) */
	uint32_t RESERVED1;				/* Reserved bit (Address offset: 0x1C) */
	_vo uint32_t RCC_APB1RSTR;		/* RCC APB1 peripheral reset register (Address offset: 0x20) */
	_vo uint32_t RCC_APB2RSTR;  	/* RCC APB2 peripheral reset register (Address offset: 0x24)) */
	uint32_t RESERVED2[2];   		/* Reserved bit (Address offset: 0x28 - 0x2C) */
	_vo uint32_t RCC_AHB1ENR;		/* RCC AHB1 peripheral enable register (Address offset: 0x30) */
	_vo uint32_t RCC_AHB2ENR;		/* RCC AHB2 peripheral enable register (Address offset: 0x34) */
	_vo uint32_t RCC_AHB3ENR;		/* RCC AHB3 peripheral enable register (Address offset: 0x38) */
	uint32_t RESERVED3;				/* Reserved bit (Address offset: 0x3C) */
	_vo uint32_t RCC_APB1ENR;		/* RCC APB1 peripheral enable register (Address offset: 0x40) */
	_vo uint32_t RCC_APB2ENR;		/* RCC APB2 peripheral enable register (Address offset: 0x44) */
	uint32_t RESERVED4[2];			/* Reserved bit (Address offset: 0x48 - 0x4C) */
	_vo uint32_t RCC_AHB1LPENR;		/* RCC AHB1 peripheral enable in low power mode register (Address offset: 0x50) */
	_vo uint32_t RCC_AHB2LPENR; 	/* RCC AHB2 peripheral enable in low power mode register (Address offset: 0x54) */
	_vo uint32_t RCC_AHB3LPENR; 	/* RCC AHB3 peripheral enable in low power mode register (Address offset: 0x58) */
	uint32_t RESERVED5;				/* Reserved bit (Address offset: 0x5C) */
	_vo uint32_t RCC_APB1LPENR;		/* RCC APB1 peripheral enable in low power mode register (Address offset: 0x60) */
	_vo uint32_t RCC_APB2LPENR;		/* RCC APB2 peripheral enable in low power mode register (Address offset: 0x64) */
	uint32_t RESERVED6[2];			/* Reserved bit (Address offset: 0x68 - 0x6C) */
	_vo uint32_t RCC_BDCR;			/* RCC backup domain control register (Address offset: 0x70) */
	_vo uint32_t RCC_CSR;			/* RCC clock control and status register (Address offset: 0x74) */
	uint32_t RESERVED7[2];			/* Reserved bit (Address offset: 0x78 - 0x7C) */
	_vo uint32_t RCC_SSCGR;   		/* RCC spread spectrum clock generation register (Address offset: 0x80) */
	_vo uint32_t RCC_PLLI2SCFGR;	/* RCC PLLI2S configuration register (Address offset: 0x84) */
//	_vo uint32_t RCC_PLLSAICFGR;	/* RCC PLLSAI configuration register (Address offset: 0x88) */
//	_vo uint32_t RCC_DCKCFGR;		/* RCC DCK configuration register (Address offset: 0x8C) */
//	_vo uint32_t RCC_CKGATENR;   	/* RCC CKGAT enable register (Address offset: 0x90) */
//	_vo uint32_t RCC_DCKCCFGR2;		/* RCC DCKC configuration 2 register (Address offset: 0x94) */

} RCC_RegDef_t;


/********************************* EXTI ***********************************/
typedef struct
{
	_vo uint32_t IMR;		/* EXTI Interrupt Mask Register (Address offset: 0x00) */
	_vo uint32_t EMR;   	/* EXTI Event Mask Register (Address offset: 0x04) */
	_vo uint32_t RTSR;		/* EXTI Rising Trigger Selection Register (Address offset: 0x08) */
	_vo uint32_t FTSR;		/* EXTI Falling Trigger Selection Register (Address offset: 0x0C) */
	_vo uint32_t SWIER;		/* EXTI software Interrupt Event Register (Address offset: 0x10) */
	_vo uint32_t PR;		/* EXTI Pending Register (Address offset: 0x14) */

} EXTI_RegDef_t;


/********************************* SYSCFG ***********************************/
typedef struct
{
	_vo uint32_t MEMRMP;	/* SYSCFG Memory Remap Register (Address offset: 0x00) */
	_vo uint32_t PMC;   	/* SYSCFG Peripheral Mode COnfiguration Register (Address offset: 0x04) */
	_vo uint32_t EXTICR[4];	/* SYSCFG External Interrupt Configuration Register 1 - 4 (Address offset: 0x08 - 0x14) */
	uint32_t RESERVED0[2];	/* Reserved Bit (Address Offset: 0x18 - 0x1C) */
	_vo uint32_t CMPCR;		/* SYSCFG Compensation Cell COntrol Register (Address offset: 0x20) */
	//uint32_t RESERVED1[2];	/* Reserved Bit (Address Offset: 0x2C) */
	//_vo uint32_t CFGR;		/* SYSCFG Control Register (Address Offset: 0x2C) */

} SYSCFG_RegDef_t;


/********************************** SPI *********************************/
typedef struct
{
	_vo uint32_t CR1;     	/* SPI Control Register 1 (Address offset: 0x00) */
	_vo uint32_t CR2;    	/* SPI Control Register 2 (Address offset: 0x04) */
	_vo uint32_t SR;		/* SPI Status Register (Address offset: 0x08) */
	_vo uint32_t DR;		/* SPI Data Register (Address offset: 0x0C) */
	_vo uint32_t CRCPR;		/* SPI CRC Polynomial Register (Address offset: 0x10) */
	_vo uint32_t RXCRCR;	/* SPI CRC Receiving Register (Address offset: 0x14) */
	_vo uint32_t TXCRCR;	/* SPI CRC Transmitting Register (Address offset: 0x18) */
	_vo uint32_t I2SCFGR;	/* SPI I2C Configuration Register (Address offset: 0x1C) */
	_vo uint32_t I2SPR;		/* SPI I2C Prescaler Register (Address offset: 0x20) */
} SPI_RegDef_t;


/*
 * Peripheral definitions (typecasted to xxx_RegDef_t)
 */

/********** GPIO ************/
#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASEAADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASEAADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASEAADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASEAADDR)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASEAADDR)
#define GPIOF ((GPIO_RegDef_t*)GPIOF_BASEAADDR)
#define GPIOG ((GPIO_RegDef_t*)GPIOG_BASEAADDR)
#define GPIOH ((GPIO_RegDef_t*)GPIOH_BASEAADDR)
#define GPIOI ((GPIO_RegDef_t*)GPIOI_BASEAADDR)
#define GPIOJ ((GPIO_RegDef_t*)GPIOJ_BASEAADDR)
#define GPIOK ((GPIO_RegDef_t*)GPIOK_BASEAADDR)

/*********** RCC *************/
#define RCC ((RCC_RegDef_t*)RCC_BASEADDR)

/*********** EXTI *************/
#define EXTI ((EXTI_RegDef_t*)EXTI_BASEADDR)

/*********** SYSCFG *************/
#define SYSCFG ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/********** SPI ************/
#define SPI1 ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4 ((SPI_RegDef_t*)SPI4_BASEADDR)



/*
 * Clock Enable Macros for x Peripherals
 */

/****** GPIO ******/
#define GPIOA_PCLK_EN()	(RCC->RCC_AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()	(RCC->RCC_AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()	(RCC->RCC_AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()	(RCC->RCC_AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()	(RCC->RCC_AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()	(RCC->RCC_AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()	(RCC->RCC_AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()	(RCC->RCC_AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()	(RCC->RCC_AHB1ENR |= (1<<8))
#define GPIOJ_PCLK_EN()	(RCC->RCC_AHB1ENR |= (1<<9))
#define GPIOK_PCLK_EN()	(RCC->RCC_AHB1ENR |= (1<<10))

/****** SPI ******/
#define SPI1_PCLK_EN()	(RCC->RCC_APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()	(RCC->RCC_APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()	(RCC->RCC_APB1ENR |= (1<<15))
#define SPI4_PCLK_EN()	(RCC->RCC_APB2ENR |= (1<<13))
//#define SPI5_PCLK_EN()	(RCC->APB2ENR |= (1<<13))
//#define SPI6_PCLK_EN()	(RCC->APB2ENR |= (1<<13))

 /****** I2C ******/
#define I2C1_PCLK_EN()	(RCC->RCC_APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()	(RCC->RCC_APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()	(RCC->RCC_APB1ENR |= (1<<23))

/****** U(S)ART ******/
#define USART1_PCLK_EN()	(RCC->RCC_APB2ENR |= (1<<4))
#define USART2_PCLK_EN()	(RCC->RCC_APB1ENR |= (1<<17))
#define USART3_PCLK_EN()	(RCC->RCC_APB1ENR |= (1<<18))
#define UART4_PCLK_EN()		(RCC->RCC_APB1ENR |= (1<<19))
#define UART5_PCLK_EN()		(RCC->RCC_APB1ENR |= (1<<20))
#define USART6_PCLK_EN()	(RCC->RCC_APB1ENR |= (1<<5))

/****** SYSCFG ******/
#define SYSCFG_PCLK_EN()	(RCC->RCC_APB2ENR |= (1<<14))


/*
 * Clock Disable Macros for x Peripherals
 */

/****** GPIO ******/
#define GPIOA_PCLK_DI()	(RCC->RCC_AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()	(RCC->RCC_AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()	(RCC->RCC_AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()	(RCC->RCC_AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()	(RCC->RCC_AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()	(RCC->RCC_AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()	(RCC->RCC_AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()	(RCC->RCC_AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()	(RCC->RCC_AHB1ENR &= ~(1<<8))
#define GPIOJ_PCLK_DI()	(RCC->RCC_AHB1ENR &= ~(1<<9))
#define GPIOK_PCLK_DI()	(RCC->RCC_AHB1ENR &= ~(1<<10))

/****** SPI ******/
#define SPI1_PCLK_DI()	(RCC->RCC_APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()	(RCC->RCC_APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()	(RCC->RCC_APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DI()	(RCC->RCC_APB2ENR &= ~(1<<13))
//#define SPI5_PCLK_DI()	(RCC->APB2ENR &= ~(1<<13))
//#define SPI6_PCLK_DI()	(RCC->APB2ENR &= ~(1<<13))

 /****** I2C ******/
#define I2C1_PCLK_DI()	(RCC->RCC_APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()	(RCC->RCC_APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()	(RCC->RCC_APB1ENR &= ~(1<<23))

/****** U(S)ART ******/
#define USART1_PCLK_DI()	(RCC->RCC_APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()	(RCC->RCC_APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()	(RCC->RCC_APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI()		(RCC->RCC_APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI()		(RCC->RCC_APB1ENR &= ~(1<<20))
#define USART6_PCLK_DI()	(RCC->RCC_APB1ENR &= ~(1<<5))

/****** SYSCFG ******/
#define SYSCFG_PCLCK_DI()	(RCC->RCC_APB2ENR &= ~(1<<14))


/****** Macros to reset GPIOx peripherals ******/
#define GPIOA_REG_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1<<0)); (RCC->RCC_AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1<<1)); (RCC->RCC_AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1<<2)); (RCC->RCC_AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1<<3)); (RCC->RCC_AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1<<4)); (RCC->RCC_AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOF_REG_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1<<5)); (RCC->RCC_AHB1RSTR &= ~(1<<5));}while(0)
#define GPIOG_REG_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1<<6)); (RCC->RCC_AHB1RSTR &= ~(1<<6));}while(0)
#define GPIOH_REG_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1<<7)); (RCC->RCC_AHB1RSTR &= ~(1<<7));}while(0)
#define GPIOI_REG_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1<<8)); (RCC->RCC_AHB1RSTR &= ~(1<<8));}while(0)
#define GPIOJ_REG_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1<<9)); (RCC->RCC_AHB1RSTR &= ~(1<<9));}while(0)
#define GPIOK_REG_RESET()	do{ (RCC->RCC_AHB1RSTR |= (1<<10)); (RCC->RCC_AHB1RSTR &= ~(1<<10));}while(0)

/****** Macros to reset GPIOx peripherals ******/
#define SPI1_REG_RESET()	do{ (RCC->RCC_APB2RSTR |= (1<<12)); (RCC->RCC_APB2RSTR &= ~(1<<12));}while(0)
#define SPI2_REG_RESET()	do{ (RCC->RCC_APB1RSTR |= (1<<14)); (RCC->RCC_APB1RSTR &= ~(1<<14));}while(0)
#define SPI3_REG_RESET()	do{ (RCC->RCC_APB1RSTR |= (1<<15)); (RCC->RCC_APB1RSTR &= ~(1<<15));}while(0)
#define SPI4_REG_RESET()	do{ (RCC->RCC_APB2RSTR |= (1<<13)); (RCC->RCC_APB2RSTR &= ~(1<<13));}while(0)


/****** Macro to return code for given GPIO base address ******/
#define GPIO_BASEADDR_TO_CODE(x)	(	(x == GPIOA) ? 0 :\
										(x == GPIOB) ? 1 :\
										(x == GPIOC) ? 2 :\
										(x == GPIOD) ? 3 :\
										(x == GPIOE) ? 4 :\
										(x == GPIOF) ? 5 :\
										(x == GPIOG) ? 6 :\
										(x == GPIOH) ? 7 :\
										(x == GPIOI) ? 8 :\
										(x == GPIOJ) ? 9 :\
										(x == GPIOK) ? 10 :0	)


/****** IRQ(Interrupt Request) Numbers of STM32F407x MCU ******/
#define IRQ_EXTI0 		6
#define IRQ_EXTI1 		7
#define IRQ_EXTI2 		8
#define IRQ_EXTI3 		9
#define IRQ_EXTI4 		10
#define IRQ_EXTI5_9 	23
#define IRQ_EXTI10_15 	40

/****** Macros for all possible priority levels ******/
#define NVIC_IRQ_PRIORITY0 			0
#define NVIC_IRQ_PRIORITY15 		15


/***** Some generic macros *****/
#define NO_OF_PR_BITS_IMPLEMENTED	4
#define ENABLE 						1
#define DISABLE 					0
#define SET 						ENABLE
#define RESET 						DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET
#define HIGH						ENABLE
#define LOW							DISABLE

/****** Driver Header Files ******/
#include "STM32F407XX_GPIO_DRIVER.h"
#include "STM32F407XX_SPI_DRIVER.h"


#endif /* INC_STM32F407XX_H_ */
