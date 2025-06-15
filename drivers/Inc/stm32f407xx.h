/*
 * stm32f407xx.h
 *
 *  Created on: Feb 12, 2025
 *      Author: gk422
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile

/*
 * base address of SRAM and Flash memories
 */
#define FLASH_BASEADDR		0x08000000U 	//from data-manual
#define SRAM1_BASEADDR		0x20000000U 	//112KB from data-manual
#define SRAM 				SRAM1_BASEADDR
#define SRAM2_BASEADDR		0x20001C00U		//(calculated) after 112Kb SRAM1, start SRAM2, also can be found in data manual of dev board in memory map->embedded sram
#define ROM_BASEADDR		0x1FFF0000U 	//from data-manual


/*
 * AHBx and ABPx BUS Peripheral base address
 */
#define PERIPH_BASEADDR			0x40000000U			//from data-manual
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR		//from data-manual
#define APB2PERIPH_BASEADDR		0x40010000U			//from data-manual
#define AHB1PERIPH_BASEADDR		0x40020000U			//from data-manual
#define AHB2PERIPH_BASEADDR		0x50000000U			//from data-manual

/*
 * Base address of peripherals which are hanging/connected on AHB1 bus
 */
#define GPIOA_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0000)		//from data-manual
#define GPIOB_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR		(AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR		(AHB1PERIPH_BASEADDR + 0x3800)		// from data manual RCC is connected to AHB1 bus with offset value 0x3800

/*
 * Base address of peripherals which are hanging/connected on APB1 bus
 */
#define I2C1_BASEADDR 			(APB1PERIPH_BASEADDR + 0x5400)		//from data-manual
#define I2C2_BASEADDR 			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR 			(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR 			(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR 			(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR 		(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR 		(APB1PERIPH_BASEADDR + 0x4800)

#define UART4_BASEADDR 			(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR 			(APB1PERIPH_BASEADDR + 0x5000)

/*
 * Base address of peripherals which are hanging/connected on APB2 bus
 */
#define EXTI_BASEADDR 			(APB2PERIPH_BASEADDR + 0x3C00)		//from data-manual
#define SYSCFG_BASEADDR 		(APB2PERIPH_BASEADDR + 0x3800)

#define SPI1_BASEADDR 			(APB2PERIPH_BASEADDR + 0x3000)

#define USART1_BASEADDR 		(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR 		(APB2PERIPH_BASEADDR + 0x1400)



/*******************************peripheral registers definition structures***********************************/

/*
 * Note: Registers of the peripheral are specific to MCU
 * e.g. : number of registers of SPI peripheral of STM32f4x family of MCU may be different (more or less)
 * compare to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCU
 */

typedef struct
{
	__vo uint32_t MODER;				/* !< GPIO port mode register 						Address offset: 0x00 */
	__vo uint32_t OTYPER;				/* !< GPIO port output type register 				Address offset: 0x04 */
	__vo uint32_t OSPEEDR;				/* !< GPIO port output speed register 				Address offset: 0x08 */
	__vo uint32_t PUPDR;				/* !< GPIO port pull-up/pull-down register 			Address offset: 0x0C */
	__vo uint32_t IDR;					/* !< GPIO port input data register 				Address offset: 0x10 */
	__vo uint32_t ODR;					/* !< GPIO port bit output data register 			Address offset: 0x14 */
	__vo uint32_t BSRR;					/* !< GPIO port bit set/reset register 				Address offset: 0x18 */
	__vo uint32_t LCKR;					/* !< GPIO port configuration lock register 		Address offset: 0x1C */
	__vo uint32_t AFR[2];				/* !< AFR[0]: GPIO alternate function low register 	Address offset: 0x20
	 	 	 	 	 	 	 	 	       !< AFR[1]: GPIO alternate function high register Address offset: 0x24 */
}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t RCC_CR;					/* !< TODO 						Address offset: 0x00 */
	__vo uint32_t RCC_PLLCFGR;				/* !< TODO 						Address offset: 0x00 */
	__vo uint32_t RCC_CFGR;					/* !< TODO 						Address offset: 0x00 */
	__vo uint32_t RCC_CIR;					/* !< TODO 						Address offset: 0x00 */
	__vo uint32_t RCC_AHB1RSTR;				/* !< TODO 						Address offset: 0x00 */
	__vo uint32_t RCC_AHB2RSTR;				/* !< TODO 						Address offset: 0x00 */
	__vo uint32_t RCC_AHB3RSTR;				/* !< TODO 						Address offset: 0x00 */
	uint32_t RESERVED0;						/* !< TODO 						Address offset: 0x00 */
	__vo uint32_t RCC_APB1RSTR;				/* !< TODO 						Address offset: 0x00 */
	__vo uint32_t RCC_APB2RSTR;				/* !< TODO 						Address offset: 0x00 */
	uint32_t RESERVED1[2];					/* !< TODO 						Address offset: 0x00 */
	__vo uint32_t RCC_AHB1ENR;				/* !< TODO 						Address offset: 0x00 */
	__vo uint32_t RCC_AHB2ENR;				/* !< TODO 						Address offset: 0x00 */
	__vo uint32_t RCC_AHB3ENR;				/* !< TODO 						Address offset: 0x00 */
	uint32_t RESERVED2;						/* !< TODO 						Address offset: 0x00 */
	__vo uint32_t RCC_APB1ENR;				/* !< TODO 						Address offset: 0x00 */
	__vo uint32_t RCC_APB2ENR;				/* !< TODO 						Address offset: 0x00 */
	uint32_t RESERVED3[2];					/* !< TODO 						Address offset: 0x00 */
	__vo uint32_t RCC_AHB1LPENR;			/* !< TODO 						Address offset: 0x00 */
	__vo uint32_t RCC_AHB2LPENR;			/* !< TODO 						Address offset: 0x00 */
	__vo uint32_t RCC_AHB3LPENR;			/* !< TODO 						Address offset: 0x00 */
	uint32_t RESERVED4;						/* !< TODO 						Address offset: 0x00 */
	__vo uint32_t RCC_APB1LPENR;			/* !< TODO 						Address offset: 0x00 */
	__vo uint32_t RCC_APB2LPENR;			/* !< TODO 						Address offset: 0x00 */
	uint32_t RESERVED5[2];					/* !< TODO 						Address offset: 0x00 */
	__vo uint32_t RCC_BDCR;					/* !< TODO 						Address offset: 0x00 */
	__vo uint32_t RCC_CSR;					/* !< TODO 						Address offset: 0x00 */
	uint32_t RESERVED6[2];					/* !< TODO 						Address offset: 0x00 */
	__vo uint32_t RCC_SSCGR;				/* !< TODO 						Address offset: 0x00 */
	__vo uint32_t RCC_PLLI2SCFGR;			/* !< TODO 						Address offset: 0x00 */
	__vo uint32_t RCC_PLLSAICFGR;			/* !< TODO 						Address offset: 0x00 */
	__vo uint32_t RCC_DCKCFGR;				/* !< TODO 						Address offset: 0x00 */
}RCC_RegDef_t;

/*
 * peripheral definitions (peripherals base addresses type-casted to xx RegDef_t)
 */
#define GPIOA 		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 		((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 		((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 		((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI 		((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC 		((RCC_RegDef_t*)RCC_BASEADDR)


/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()			(RCC->RCC_AHB1ENR |=(1 << 0))				//using memory map and rcc registers form data manual
#define GPIOB_PCLK_EN()			(RCC->RCC_AHB1ENR |=(1 << 1))
#define GPIOC_PCLK_EN()			(RCC->RCC_AHB1ENR |=(1 << 2))
#define GPIOD_PCLK_EN()			(RCC->RCC_AHB1ENR |=(1 << 3))
#define GPIOE_PCLK_EN()			(RCC->RCC_AHB1ENR |=(1 << 4))
#define GPIOF_PCLK_EN()			(RCC->RCC_AHB1ENR |=(1 << 5))
#define GPIOG_PCLK_EN()			(RCC->RCC_AHB1ENR |=(1 << 6))
#define GPIOH_PCLK_EN()			(RCC->RCC_AHB1ENR |=(1 << 7))
#define GPIOI_PCLK_EN()			(RCC->RCC_AHB1ENR |=(1 << 8))

/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()			(RCC->RCC_APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()			(RCC->RCC_APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()			(RCC->RCC_APB1ENR |= (1 << 23))

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()			(RCC->RCC_APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()			(RCC->RCC_APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()			(RCC->RCC_APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()			(RCC->RCC_APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN()			(RCC->RCC_APB2ENR |= (1 << 20))
#define SPI6_PCLK_EN()			(RCC->RCC_APB2ENR |= (1 << 21))


/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN()			(RCC->RCC_APB2ENR |= (1 << 4))

/*
 * Clock Enable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN()			(RCC->RCC_APB2ENR |= (1 << 14))

/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()			(RCC->RCC_AHB1ENR &= ~(1 << 8))

/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()			(RCC->RCC_APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()			(RCC->RCC_APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()			(RCC->RCC_APB1ENR &= ~(1 << 23))

/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()			(RCC->RCC_APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()			(RCC->RCC_APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()			(RCC->RCC_APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()			(RCC->RCC_APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DI()			(RCC->RCC_APB2ENR &= ~(1 << 20))
#define SPI6_PCLK_DI()			(RCC->RCC_APB2ENR &= ~(1 << 21))

/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI()			(RCC->RCC_APB2ENR &= ~(1 << 4))

/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI()			(RCC->RCC_APB2ENR &= ~(1 << 4))

/*
 *Macros to reset GPIOx Peripherals
 */
#define GPIOA_REG_RESET()			do{(RCC->RCC_AHB1RSTR |= (1 << 0)); (RCC->RCC_AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()			do{(RCC->RCC_AHB1RSTR |= (1 << 1)); (RCC->RCC_AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()			do{(RCC->RCC_AHB1RSTR |= (1 << 2)); (RCC->RCC_AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()			do{(RCC->RCC_AHB1RSTR |= (1 << 3)); (RCC->RCC_AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()			do{(RCC->RCC_AHB1RSTR |= (1 << 4)); (RCC->RCC_AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()			do{(RCC->RCC_AHB1RSTR |= (1 << 5)); (RCC->RCC_AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET()			do{(RCC->RCC_AHB1RSTR |= (1 << 6)); (RCC->RCC_AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET()			do{(RCC->RCC_AHB1RSTR |= (1 << 7)); (RCC->RCC_AHB1RSTR &= ~(1 << 7));}while(0)
#define GPIOI_REG_RESET()			do{(RCC->RCC_AHB1RSTR |= (1 << 8)); (RCC->RCC_AHB1RSTR &= ~(1 << 8));}while(0)

/*
 * generic defined macros
 */
#define ENABLE 				1
#define DISABLE				0
#define SET					ENABLE
#define RESER				DISABLE
#define GPIO_PIN_SET 		SET
#define GPIO_PIN_RESET		RESET

#endif /* INC_STM32F407XX_H_ */
