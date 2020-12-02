#ifndef	INC_STM32F4_H_
#define INC_STM32F4_H_
/********************************************************************************
	@file  stm32f407xx.h 
	@Desciprition 
		- This Header file contains STM32F4 Microcontroller specific details
		- This header file can e used in both application level and driver level
		- It Contains
			- The Base addresses of memory devices present in the Microcontroller (Ex, Flash, SRAM1, SRAM2, ROM, etc)
			- Peripheral register definition structures
			- Base addresss of various peripherals present in diffrent bus domains of the Microcontroller
			- Base addresses of various clock domains such as (AHBx domain, APBx domain)
			- Clock management macros (ie, Clock enable and clock disable macros)
			- IRQ definitions
		 	- Peripheral register bit definitions
********************************************************************************/

/*================================================================================
								 INCLUDE FILES
================================================================================*/
#include<stdint.h>

/*================================================================================
								 MACRO DEFINITIONS
================================================================================*/

#define __vo							volatile

/*--------------------------------------------------------------------------------
						   Base addresses of Memories
--------------------------------------------------------------------------------*/

#define	FLASH_BASEADDR					0x08000000U		/*!< Base address of the Flash memory with the size of 1MB */		
#define	SRAM1_BASEADDR					0X20000000U		/*!< Base address of the SRAM1 memory with the size of 112KB */
#define	SRAM2_BASEADDR					0x2001C000U		/*!< Base address of the SRAM2 memory with the size of 16KB */
#define	ROM_BASEADDR					0x1FFF0000U		/*!< Base address of the System memory (ROM) with the size of 30KB */
#define	SRAM							SRAM1_BASEADDR

/*--------------------------------------------------------------------------------
					 Base addresses of BUS and Peripherals
--------------------------------------------------------------------------------*/

/* AHBx and APBx Bus Peripheral base addresses */
#define	PERIPH_BASE 					0x40000000U
#define APB1PERIPH_BASEADDR				PERIPH_BASE
#define	APB2PERIPH_BASEADDR				(PERIPH_BASE + 0x10000U)
#define AHB1PERIPH_BASEADDR				(PERIPH_BASE + 0x20000U)				
#define	AHB2PERIPH_BASEADDR				(PERIPH_BASE + 0x10000000)

/* Base addresses of peripherals which are hanging on AHB1 BUS */
#define	GPIOA_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0000U)
#define	GPIOB_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0400U)
#define	GPIOC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0800U)
#define	GPIOD_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0C00U)
#define	GPIOE_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1000U)
#define	GPIOF_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1400U)
#define	GPIOG_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1800U)
#define	GPIOH_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1C00U)
#define	GPIOI_BASEADDR					(AHB1PERIPH_BASEADDR + 0x2000U)
#define	GPIOJ_BASEADDR					(AHB1PERIPH_BASEADDR + 0x2400U)
#define	GPIOK_BASEADDR					(AHB1PERIPH_BASEADDR + 0x2800U)
#define	CRC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x3000U)
#define	RCC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x3800U)

/* Base addresses of peripherals which are hanging on APB1 BUS */
#define	TIM2_BASEADDR					(APB1PERIPH_BASEADDR + 0x0000U)
#define	TIM3_BASEADDR					(APB1PERIPH_BASEADDR + 0x0400U)
#define	TIM4_BASEADDR					(APB1PERIPH_BASEADDR + 0x0400U)
#define	TIM5_BASEADDR					(APB1PERIPH_BASEADDR + 0x0C00U)
#define	TIM6_BASEADDR					(APB1PERIPH_BASEADDR + 0x1000U)
#define	TIM7_BASEADDR					(APB1PERIPH_BASEADDR + 0x1400U)
#define	TIM12_BASEADDR					(APB1PERIPH_BASEADDR + 0x1800U)
#define	TIM13_BASEADDR					(APB1PERIPH_BASEADDR + 0x1C00U)
#define	TIM14_BASEADDR					(APB1PERIPH_BASEADDR + 0x2000U)
#define	RTC_AND_BKP_BASEADDR			(APB1PERIPH_BASEADDR + 0x2800U)
#define	WWDG_BASEADDR					(APB1PERIPH_BASEADDR + 0x2C00U)
#define	IWDG_BASEADDR					(APB1PERIPH_BASEADDR + 0x3000U)
#define	I2S2ext_BASEADDR				(APB1PERIPH_BASEADDR + 0x3400U)
#define	SPI2_I2S2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800U)
#define	SPI3_I2S3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00U)
#define	I2S3ext_BASEADDR				(APB1PERIPH_BASEADDR + 0x4000U)
#define	USART2_BASEADDR					(APB1PERIPH_BASEADDR + 0x4400U)
#define	USART3_BASEADDR					(APB1PERIPH_BASEADDR + 0x4800U)
#define	UART4_BASEADDR					(APB1PERIPH_BASEADDR + 0x4C00U)
#define	UART5_BASEADDR					(APB1PERIPH_BASEADDR + 0x5000U)
#define	I2C1_BASEADDR					(APB1PERIPH_BASEADDR + 0x5400U)
#define	I2C2_BASEADDR					(APB1PERIPH_BASEADDR + 0x5800U)
#define	I2C3_BASEADDR					(APB1PERIPH_BASEADDR + 0x5C00U)
#define	CAN1_BASEADDR					(APB1PERIPH_BASEADDR + 0x6400U)
#define	CAN2_BASEADDR					(APB1PERIPH_BASEADDR + 0x6800U)
#define	PWR_BASEADDR					(APB1PERIPH_BASEADDR + 0x7000U)
#define	DAC_BASEADDR					(APB1PERIPH_BASEADDR + 0x7400U)
#define	UART7_BASEADDR					(APB1PERIPH_BASEADDR + 0x7800U)
#define	UART8_BASEADDR					(APB1PERIPH_BASEADDR + 0x7C00U)

/* Base addresses of peripherals which are hanging on APB2 BUS */
#define	TIM1_BASEADDR					(APB2PERIPH_BASEADDR + 0x0000U)
#define	TIM8_BASEADDR					(APB2PERIPH_BASEADDR + 0x0400U)
#define	USART1_BASEADDR					(APB2PERIPH_BASEADDR + 0x1000U)
#define	USART6_BASEADDR					(APB2PERIPH_BASEADDR + 0x1400U)
#define	ADC1_ADC2_ADC3_BASEADDR			(APB2PERIPH_BASEADDR + 0x2000U)
#define	SDIO_BASEADDR					(APB2PERIPH_BASEADDR + 0x2C00U)
#define	SPI1_BASEADDR					(APB2PERIPH_BASEADDR + 0x3000U)
#define	SPI4_BASEADDR					(APB2PERIPH_BASEADDR + 0x3400U)
#define	SYSCFG_BASEADDR					(APB2PERIPH_BASEADDR + 0x3800U)
#define	EXTI_BASEADDR					(APB2PERIPH_BASEADDR + 0x3C00U)
#define	TIM9_BASEADDR					(APB2PERIPH_BASEADDR + 0x4000U)
#define	TIM10_BASEADDR					(APB2PERIPH_BASEADDR + 0x4400U)
#define	TIM11_BASEADDR					(APB2PERIPH_BASEADDR + 0x4800U)
#define	SPI5_BASEADDR					(APB2PERIPH_BASEADDR + 0x5000U)
#define	SPI6_BASEADDR					(APB2PERIPH_BASEADDR + 0x5400U)
#define	SAI1_BASEADDR					(APB2PERIPH_BASEADDR + 0x5800U)
#define	LCD_TFT_BASEADDR				(APB2PERIPH_BASEADDR + 0x6800U)

/* SCB */
#define SCS_BASE            			(0xE000E000UL)
#define SCB_BASE            			(SCS_BASE +  0x0D00UL)

/*--------------------------------------------------------------------------------
 Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
--------------------------------------------------------------------------------*/

/* GPIO */
#define GPIOA 							((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 							((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 							((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 							((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 							((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 							((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 							((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 							((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI 							((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ 							((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK 							((GPIO_RegDef_t*)GPIOK_BASEADDR)

/* RCC */
#define RCC 							((RCC_RegDef_t*)RCC_BASEADDR)

/* SCB */
#define SCB								((SCB_Type*)SCB_BASE)
/*--------------------------------------------------------------------------------
					  Clock Enable macros for Peripherals
--------------------------------------------------------------------------------*/

/* Clock enable Macros for GPIOx peripherals */
#define GPIOA_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 0 ))
#define GPIOB_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 1 ))
#define GPIOC_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 2 ))
#define GPIOD_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 3 ))
#define GPIOE_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 4 ))
#define GPIOF_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 5 ))
#define GPIOG_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 6 ))
#define GPIOH_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 7 ))
#define GPIOI_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 8 ))
#define GPIOJ_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 9 ))
#define GPIOK_PCLK_EN()					( RCC->AHB1ENR |= ( 1 << 10 ))

/* Clock enable Macros for I2Cx peripherals */ 
#define I2C1_PCLK_EN()					( RCC->APB1ENR |= ( 1 << 21 ))
#define I2C2_PCLK_EN()					( RCC->APB1ENR |= ( 1 << 22 ))
#define I2C3_PCLK_EN()					( RCC->APB1ENR |= ( 1 << 23 ))

/* Clock enable Macros for SPIx peripherals */
#define SPI1_PCLK_EN()					( RCC->APB2ENR |= ( 1 << 12 )) 
#define SPI2_PCLK_EN()					( RCC->APB1ENR |= ( 1 << 14 ))
#define SPI3_PCLK_EN()					( RCC->APB1ENR |= ( 1 << 15 ))

/* Clock enable Macros for UARTx/USARTX peripherals */
#define USART1_PCLK_EN()				( RCC->APB2ENR |= ( 1 << 4 ))
#define USART2_PCLK_EN()				( RCC->APB1ENR |= ( 1 << 17 ))
#define USART3_PCLK_EN()				( RCC->APB1ENR |= ( 1 << 18 ))
#define UART4_PCLK_EN()					( RCC->APB1ENR |= ( 1 << 19 ))
#define UART5_PCLK_EN()					( RCC->APB1ENR |= ( 1 << 20 ))
#define USART6_PCLK_EN()				( RCC->APB2ENR |= ( 1 << 5 ))

/* Clock enable Macros for SYSCFG peripherals */
#define SYSCFG_PCLK_EN()				( RCC->APB2ENR |= ( 1 << 14 ))

/*--------------------------------------------------------------------------------
Clock Disable macros for Peripherals
--------------------------------------------------------------------------------*/

/* Clock disable Macros for GPIOx peripherals */ 
#define GPIOA_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 0 ))
#define GPIOB_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 1 ))
#define GPIOC_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 2 ))
#define GPIOD_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 3 ))
#define GPIOE_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 4 ))
#define GPIOF_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 5 ))
#define GPIOG_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 6 ))
#define GPIOH_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 7 ))
#define GPIOI_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 8 ))
#define GPIOJ_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 9 ))
#define GPIOK_PCLK_DI()					( RCC->AHB1ENR &= ~( 1 << 10 ))

/* Clock disable Macros for I2Cx peripherals */ 
#define I2C1_PCLK_DI()					( RCC->APB1ENR &= ~( 1 << 21 ))
#define I2C2_PCLK_DI()					( RCC->APB1ENR &= ~( 1 << 22 ))
#define I2C3_PCLK_DI()					( RCC->APB1ENR &= ~( 1 << 23 ))

/* Clock disable Macros for SPIx peripherals */
#define SPI1_PCLK_DI()					( RCC->APB2ENR &= ~( 1 << 12 )) 
#define SPI2_PCLK_DI()					( RCC->APB1ENR &= ~( 1 << 14 ))
#define SPI3_PCLK_DI()					( RCC->APB1ENR &= ~( 1 << 15 ))

/* Clock disable Macros for UARTx/USARTX peripherals */
#define USART1_PCLK_DI()				( RCC->APB2ENR &= ~( 1 << 4 ))
#define USART2_PCLK_DI()				( RCC->APB1ENR &= ~( 1 << 17 ))
#define USART3_PCLK_DI()				( RCC->APB1ENR &= ~( 1 << 18 ))
#define UART4_PCLK_DI()					( RCC->APB1ENR &= ~( 1 << 19 ))
#define UART5_PCLK_DI()					( RCC->APB1ENR &= ~( 1 << 20 ))
#define USART6_PCLK_DI()				( RCC->APB2ENR &= ~( 1 << 5 ))

/* Clock disable Macros for SYSCFG peripherals */
#define SYSCFG_PCLK_DI()				( RCC->APB2ENR &= ~( 1 << 14 ))

/*================================================================================
								 STRUCTURES
================================================================================*/

typedef struct
{
	__vo uint32_t MODER;		/*!< GPIO port mode register										Address offset : 0x00 */
	__vo uint32_t OTYPER;		/*!< GPIO port output type register									Address offset : 0x04 */
	__vo uint32_t OSPEEDR;		/*!< GPIO port output speed register								Address offset : 0x08 */
	__vo uint32_t PUPDR;		/*!< GPIO port pull-up/pull-down register							Address offset : 0x0C */
	__vo uint32_t IDR;			/*!< GPIO port input data register									Address offset : 0x10 */
	__vo uint32_t ODR;			/*!< GPIO port output data register									Address offset : 0x14 */
	__vo uint32_t BSRR;			/*!< GPIO port bit set/reset register								Address offset : 0x18 */
	__vo uint32_t LCKR;			/*!< GPIO port configuration lock register							Address offset : 0x1C */
	__vo uint32_t AFR[2];		/*!< GPIO alternate function register (Low and High)				Address offset : 0x20 */
}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;			/*!< RCC clock control register										Address offset : 0x00 */
	__vo uint32_t PLLCFGR;		/*!< RCC PLL configuration register									Address offset : 0x04 */
	__vo uint32_t CFGR;			/*!< RCC clock configuration register				 				Address offset : 0x08 */
	__vo uint32_t CIR;			/*!< RCC clock interrupt register			 						Address offset : 0x0C */
	__vo uint32_t AHB1RSTR;		/*!< RCC AHB1 peripheral reset register								Address offset : 0x10 */
	__vo uint32_t AHB2RSTR;		/*!< RCC AHB2 peripheral reset register								Address offset : 0x14 */
	__vo uint32_t AHB3RSTR;		/*!< RCC AHB3 peripheral reset register								Address offset : 0x18 */
	uint32_t RESERVED0;			/*!< Reserved 														Address offset : 0x1C */
	__vo uint32_t APB1RSTR;		/*!< RCC APB1 peripheral reset register								Address offset : 0x20 */
	__vo uint32_t APB2RSTR;		/*!< RCC APB2 peripheral reset register			 					Address offset : 0x24 */
	uint32_t RESERVED1;			/*!< Reserved 														Address offset : 0x28 */
	uint32_t RESERVED2;			/*!< Reserved 														Address offset : 0x2C */
	__vo uint32_t AHB1ENR;		/*!< RCC AHB1 peripheral clock enable register 						Address offset : 0x30 */
	__vo uint32_t AHB2ENR;		/*!< RCC AHB2 peripheral clock enable register 						Address offset : 0x34 */
	__vo uint32_t AHB3ENR;		/*!< RCC AHB3 peripheral clock enable register 						Address offset : 0x38 */
	uint32_t RESERVED3;			/*!< Reserved 														Address offset : 0x3C */
	__vo uint32_t APB1ENR;		/*!< RCC APB1 peripheral clock enable register 						Address offset : 0x40 */
	__vo uint32_t APB2ENR;		/*!< RCC APB2 peripheral clock enable register 						Address offset : 0x44 */
	uint32_t RESERVED4;			/*!< Reserved 														Address offset : 0x48 */
	uint32_t RESERVED5;			/*!< Reserved 														Address offset : 0x4C */
	__vo uint32_t AHB1LPENR;	/*!< RCC AHB1 peripheral clock enable in low power mode register 	Address offset : 0x50 */
	__vo uint32_t AHB2LPENR;	/*!< RCC AHB2 peripheral clock enable in low power mode register 	Address offset : 0x54 */
	__vo uint32_t AHB3LPENR;	/*!< RCC AHB3 peripheral clock enable in low power mode register 	Address offset : 0x58 */
	uint32_t RESERVED6;			/*!< Reserved 														Address offset : 0x5C */
	__vo uint32_t APB1LPENR;	/*!< RCC APB1 peripheral clock enable in low power mode register 	Address offset : 0x60 */
	__vo uint32_t APB2LPENR;	/*!< RCC APB2 peripheral clock enable in low power mode register 	Address offset : 0x64 */
	uint32_t RESERVED7; 		/*!< Reserved 														Address offset : 0x68 */
	uint32_t RESERVED8;			/*!< Reserved 														Address offset : 0x6C */
	__vo uint32_t BDCR;			/*!< RCC Backup domain control register 							Address offset : 0x70 */
	__vo uint32_t CSR;			/*!< RCC clock control & status register 							Address offset : 0x74 */
	uint32_t RESERVED9;			/*!< Reserved 														Address offset : 0x78 */
	uint32_t RESERVED10;		/*!< Reserved 														Address offset : 0x7C */
	__vo uint32_t SSCGR;		/*!< RCC spread spectrum clock generation register 					Address offset : 0x80 */
	__vo uint32_t PLLI2SCFGR;	/*!< RCC PLLI2S configuration register 								Address offset : 0x84 */
}RCC_RegDef_t;

typedef struct
{
  __vo const  uint32_t CPUID;                  /*!< Offset: 0x000 (R/ )  CPUID Base Register */
  __vo uint32_t ICSR;                   /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
  __vo uint32_t VTOR;                   /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
  __vo uint32_t AIRCR;                  /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
  __vo uint32_t SCR;                    /*!< Offset: 0x010 (R/W)  System Control Register */
  __vo uint32_t CCR;                    /*!< Offset: 0x014 (R/W)  Configuration Control Register */
  __vo uint8_t  SHP[12U];               /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
  __vo uint32_t SHCSR;                  /*!< Offset: 0x024 (R/W)  System Handler Control and State Register */
  __vo uint32_t CFSR;                   /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register */
  __vo uint32_t HFSR;                   /*!< Offset: 0x02C (R/W)  HardFault Status Register */
  __vo uint32_t DFSR;                   /*!< Offset: 0x030 (R/W)  Debug Fault Status Register */
  __vo uint32_t MMFAR;                  /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register */
  __vo uint32_t BFAR;                   /*!< Offset: 0x038 (R/W)  BusFault Address Register */
  __vo uint32_t AFSR;                   /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
  __vo const  uint32_t PFR[2U];                /*!< Offset: 0x040 (R/ )  Processor Feature Register */
  __vo const  uint32_t DFR;                    /*!< Offset: 0x048 (R/ )  Debug Feature Register */
  __vo const  uint32_t ADR;                    /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register */
  __vo const  uint32_t MMFR[4U];               /*!< Offset: 0x050 (R/ )  Memory Model Feature Register */
  __vo const  uint32_t ISAR[5U];               /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register */
        uint32_t RESERVED0[5U];
  __vo uint32_t CPACR;                  /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register */
} SCB_Type;



#endif  /* INC_STM32F4_H_ */

