/********************************************************************************
	@file  stm32f407xx.c 
	@Desciprition 
		-  
********************************************************************************/
/*================================================================================
								 INCLUDE FILES
================================================================================*/
#include"stm32f407xx.h"

/*================================================================================
								 FUNCTION DEFINITIONS
================================================================================*/

__weak void SystemInit(void)
{
  	/* FPU settings ------------------------------------------------------------*/
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
  	/* Reset the RCC clock configuration to the default reset state ------------*/
#if 0  
	/* Set HSION bit */
  	RCC->CR |= (uint32_t)0x00000001;

  	/* Reset CFGR register */
  	RCC->CFGR = 0x00000000;

  	/* Reset HSEON, CSSON and PLLON bits */
  	RCC->CR &= (uint32_t)0xFEF6FFFF;

  	/* Reset PLLCFGR register */
  	RCC->PLLCFGR = 0x24003010;

  	/* Reset HSEBYP bit */
  	RCC->CR &= (uint32_t)0xFFFBFFFF;

  	/* Disable all interrupts */
  	RCC->CIR = 0x00000000;

  	/* Configure the Vector Table location add offset address ------------------*/

  	SCB->VTOR = FLASH_BASEADDR; /* Vector Table Relocation in Internal FLASH */
#endif
}
