/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Feb 17, 2025
 *      Author: gk422
 */

#include "stm32f407xx_gpio_driver.h"



/*
 * peripheral clock setup
 */
/**************************************************************
 * @fn				- GPIO_PeriClockControl
 *
 * @berif			- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]		- base address of the GPIO peripheral
 * @param[in]		- ENABLE or DISABLE Macros
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none

 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD){
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}else if(pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}
	}else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD){
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}else if (pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}else if(pGPIOx == GPIOI){
			GPIOI_PCLK_DI();
		}
	}
}

/*
 * Init and De-Init
 */
/**************************************************************
 * @fn				- GPIO_Init
 *
 * @berif			- This function Initializes GPIO port
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none

 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	//1. Configure the mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		//the non interrupt mode
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle ->pGPIOx->MODER =temp;
		temp = 0;
	}else{
		//the interrupt mode
	}
	//2. Configure the speed

	//3. configure the pull-up/pull-down mode

	//4.configure the op type

	//5. configure the alt function
}

/**************************************************************
 * @fn				- GPIO_DeInti
 *
 * @berif			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none

 */
void GPIO_DeInti(GPIO_RegDef_t *pGPIOx){

}

/*
 * Data read and write
 */
/**************************************************************
 * @fn				- GPIO_ReadFromInputPin
 *
 * @berif			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-

 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

}

/**************************************************************
 * @fn				- GPIO_ReadFromInputPort
 *
 * @berif			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			- none

 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

}

/**************************************************************
 * @fn				- GPIO_WriteToOutputPin
 *
 * @berif			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-

 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

}

/**************************************************************
 * @fn				- GPIO_WriteToOutputPort
 *
 * @berif			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-

 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value){

}

/**************************************************************
 * @fn				- GPIO_ToggleOutputPin
 *
 * @berif			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-

 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

}

/*
 *IRQ Configuration and ISR handling
 */

/**************************************************************
 * @fn				- GPIO_IRQConfig
 *
 * @berif			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-

 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi){

}

/**************************************************************
 * @fn				- GPIO_IRQHandling
 *
 * @berif			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-

 */
void GPIO_IRQHandling(uint8_t PinNumber){

}
