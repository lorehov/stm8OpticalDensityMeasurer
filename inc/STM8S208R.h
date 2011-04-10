/* STM8S208R.h */

/* Copyright (c) 2003-2010 STMicroelectronics */

#ifndef __STM8S208R__
#define __STM8S208R__

/* STM8S208R */

	/* Check MCU name */
	#ifdef MCU_NAME
		#define STM8S208R 1
		#if (MCU_NAME != STM8S208R)
		#error "wrong include file STM8S208R.h for chosen MCU!"
		#endif
	#endif

	#ifdef __CSMC__
		#define DEF_8BIT_REG_AT(NAME,ADDRESS) volatile unsigned char NAME @ADDRESS
		#define DEF_16BIT_REG_AT(NAME,ADDRESS) volatile unsigned int NAME @ADDRESS
	#endif

	#ifdef __RAISONANCE__
		#define DEF_8BIT_REG_AT(NAME,ADDRESS) at ADDRESS hreg NAME
		#define DEF_16BIT_REG_AT(NAME,ADDRESS) at ADDRESS hreg16 NAME
	#endif

/* Port A */
/*****************************************************************/

/* Port A data output latch register */
DEF_8BIT_REG_AT(PA_ODR,0x5000);

/* Port A input pin value register */
DEF_8BIT_REG_AT(PA_IDR,0x5001);

/* Port A data direction register */
DEF_8BIT_REG_AT(PA_DDR,0x5002);

/* Port A control register 1 */
DEF_8BIT_REG_AT(PA_CR1,0x5003);

/* Port A control register 2 */
DEF_8BIT_REG_AT(PA_CR2,0x5004);

/* Port B */
/*****************************************************************/

/* Port B data output latch register */
DEF_8BIT_REG_AT(PB_ODR,0x5005);

/* Port B input pin value register */
DEF_8BIT_REG_AT(PB_IDR,0x5006);

/* Port B data direction register */
DEF_8BIT_REG_AT(PB_DDR,0x5007);

/* Port B control register 1 */
DEF_8BIT_REG_AT(PB_CR1,0x5008);

/* Port B control register 2 */
DEF_8BIT_REG_AT(PB_CR2,0x5009);

/* Port C */
/*****************************************************************/

/* Port C data output latch register */
DEF_8BIT_REG_AT(PC_ODR,0x500a);

/* Port C input pin value register */
DEF_8BIT_REG_AT(PC_IDR,0x500b);

/* Port C data direction register */
DEF_8BIT_REG_AT(PC_DDR,0x500c);

/* Port C control register 1 */
DEF_8BIT_REG_AT(PC_CR1,0x500d);

/* Port C control register 2 */
DEF_8BIT_REG_AT(PC_CR2,0x500e);

/* Port D */
/*****************************************************************/

/* Port D data output latch register */
DEF_8BIT_REG_AT(PD_ODR,0x500f);

/* Port D input pin value register */
DEF_8BIT_REG_AT(PD_IDR,0x5010);

/* Port D data direction register */
DEF_8BIT_REG_AT(PD_DDR,0x5011);

/* Port D control register 1 */
DEF_8BIT_REG_AT(PD_CR1,0x5012);

/* Port D control register 2 */
DEF_8BIT_REG_AT(PD_CR2,0x5013);

/* Port E */
/*****************************************************************/

/* Port E data output latch register */
DEF_8BIT_REG_AT(PE_ODR,0x5014);

/* Port E input pin value register */
DEF_8BIT_REG_AT(PE_IDR,0x5015);

/* Port E data direction register */
DEF_8BIT_REG_AT(PE_DDR,0x5016);

/* Port E control register 1 */
DEF_8BIT_REG_AT(PE_CR1,0x5017);

/* Port E control register 2 */
DEF_8BIT_REG_AT(PE_CR2,0x5018);

/* Port F */
/*****************************************************************/

/* Port F data output latch register */
DEF_8BIT_REG_AT(PF_ODR,0x5019);

/* Port F input pin value register */
DEF_8BIT_REG_AT(PF_IDR,0x501a);

/* Port F data direction register */
DEF_8BIT_REG_AT(PF_DDR,0x501b);

/* Port F control register 1 */
DEF_8BIT_REG_AT(PF_CR1,0x501c);

/* Port F control register 2 */
DEF_8BIT_REG_AT(PF_CR2,0x501d);

/* Port G */
/*****************************************************************/

/* Port G data output latch register */
DEF_8BIT_REG_AT(PG_ODR,0x501e);

/* Port G input pin value register */
DEF_8BIT_REG_AT(PG_IDR,0x501f);

/* Port G data direction register */
DEF_8BIT_REG_AT(PG_DDR,0x5020);

/* Port G control register 1 */
DEF_8BIT_REG_AT(PG_CR1,0x5021);

/* Port G control register 2 */
DEF_8BIT_REG_AT(PG_CR2,0x5022);

/* Port I */
/*****************************************************************/

/* Port I data output latch register */
DEF_8BIT_REG_AT(PI_ODR,0x5028);

/* Port I input pin value register */
DEF_8BIT_REG_AT(PI_IDR,0x5029);

/* Port I data direction register */
DEF_8BIT_REG_AT(PI_DDR,0x502a);

/* Port I control register 1 */
DEF_8BIT_REG_AT(PI_CR1,0x502b);

/* Port I control register 2 */
DEF_8BIT_REG_AT(PI_CR2,0x502c);

/* Flash */
/*****************************************************************/

/* Flash control register 1 */
DEF_8BIT_REG_AT(FLASH_CR1,0x505a);

/* Flash control register 2 */
DEF_8BIT_REG_AT(FLASH_CR2,0x505b);

/* Flash complementary control register 2 */
DEF_8BIT_REG_AT(FLASH_NCR2,0x505c);

/* Flash protection register */
DEF_8BIT_REG_AT(FLASH_FPR,0x505d);

/* Flash complementary protection register */
DEF_8BIT_REG_AT(FLASH_NFPR,0x505e);

/* Flash in-application programming status register */
DEF_8BIT_REG_AT(FLASH_IAPSR,0x505f);

/* Flash Program memory unprotection register */
DEF_8BIT_REG_AT(FLASH_PUKR,0x5062);

/* Data EEPROM unprotection register */
DEF_8BIT_REG_AT(FLASH_DUKR,0x5064);

/* External Interrupt Control Register (ITC) */
/*****************************************************************/

/* External interrupt control register 1 */
DEF_8BIT_REG_AT(EXTI_CR1,0x50a0);

/* External interrupt control register 2 */
DEF_8BIT_REG_AT(EXTI_CR2,0x50a1);

/* Reset (RST) */
/*****************************************************************/

/* Reset status register 1 */
DEF_8BIT_REG_AT(RST_SR,0x50b3);

/* Clock Control (CLK) */
/*****************************************************************/

/* Internal clock control register */
DEF_8BIT_REG_AT(CLK_ICKR,0x50c0);

/* External clock control register */
DEF_8BIT_REG_AT(CLK_ECKR,0x50c1);

/* Clock master status register */
DEF_8BIT_REG_AT(CLK_CMSR,0x50c3);

/* Clock master switch register */
DEF_8BIT_REG_AT(CLK_SWR,0x50c4);

/* Clock switch control register */
DEF_8BIT_REG_AT(CLK_SWCR,0x50c5);

/* Clock divider register */
DEF_8BIT_REG_AT(CLK_CKDIVR,0x50c6);

/* Peripheral clock gating register 1 */
DEF_8BIT_REG_AT(CLK_PCKENR1,0x50c7);

/* Clock security system register */
DEF_8BIT_REG_AT(CLK_CSSR,0x50c8);

/* Configurable clock control register */
DEF_8BIT_REG_AT(CLK_CCOR,0x50c9);

/* Peripheral clock gating register 2 */
DEF_8BIT_REG_AT(CLK_PCKENR2,0x50ca);

/* CAN clock control register */
DEF_8BIT_REG_AT(CLK_CANCCR,0x50cb);

/* HSI clock calibration trimming register */
DEF_8BIT_REG_AT(CLK_HSITRIMR,0x50cc);

/* SWIM clock control register */
DEF_8BIT_REG_AT(CLK_SWIMCCR,0x50cd);

/* Window Watchdog (WWDG) */
/*****************************************************************/

/* WWDG Control Register */
DEF_8BIT_REG_AT(WWDG_CR,0x50d1);

/* WWDR Window Register */
DEF_8BIT_REG_AT(WWDG_WR,0x50d2);

/* Independent Watchdog (IWDG) */
/*****************************************************************/

/* IWDG Key Register */
DEF_8BIT_REG_AT(IWDG_KR,0x50e0);

/* IWDG Prescaler Register */
DEF_8BIT_REG_AT(IWDG_PR,0x50e1);

/* IWDG Reload Register */
DEF_8BIT_REG_AT(IWDG_RLR,0x50e2);

/* Auto Wake-Up (AWU) */
/*****************************************************************/

/* AWU Control/Status Register */
DEF_8BIT_REG_AT(AWU_CSR,0x50f0);

/* AWU asynchronous prescaler buffer register */
DEF_8BIT_REG_AT(AWU_APR,0x50f1);

/* AWU Timebase selection register */
DEF_8BIT_REG_AT(AWU_TBR,0x50f2);

/* Beeper (BEEP) */
/*****************************************************************/

/* BEEP Control/Status Register */
DEF_8BIT_REG_AT(BEEP_CSR,0x50f3);

/* Serial Peripheral Interface (SPI) */
/*****************************************************************/

/* SPI Control Register 1 */
DEF_8BIT_REG_AT(SPI_CR1,0x5200);

/* SPI Control Register 2 */
DEF_8BIT_REG_AT(SPI_CR2,0x5201);

/* SPI Interrupt Control Register */
DEF_8BIT_REG_AT(SPI_ICR,0x5202);

/* SPI Status Register */
DEF_8BIT_REG_AT(SPI_SR,0x5203);

/* SPI Data Register */
DEF_8BIT_REG_AT(SPI_DR,0x5204);

/* SPI CRC Polynomial Register */
DEF_8BIT_REG_AT(SPI_CRCPR,0x5205);

/* SPI Rx CRC Register */
DEF_8BIT_REG_AT(SPI_RXCRCR,0x5206);

/* SPI Tx CRC Register */
DEF_8BIT_REG_AT(SPI_TXCRCR,0x5207);

/* I2C Bus Interface (I2C) */
/*****************************************************************/

/* I2C control register 1 */
DEF_8BIT_REG_AT(I2C_CR1,0x5210);

/* I2C control register 2 */
DEF_8BIT_REG_AT(I2C_CR2,0x5211);

/* I2C frequency register */
DEF_8BIT_REG_AT(I2C_FREQR,0x5212);

/* I2C Own address register low */
DEF_8BIT_REG_AT(I2C_OARL,0x5213);

/* I2C Own address register high */
DEF_8BIT_REG_AT(I2C_OARH,0x5214);

/* I2C data register */
DEF_8BIT_REG_AT(I2C_DR,0x5216);

/* I2C status register 1 */
DEF_8BIT_REG_AT(I2C_SR1,0x5217);

/* I2C status register 2 */
DEF_8BIT_REG_AT(I2C_SR2,0x5218);

/* I2C status register 3 */
DEF_8BIT_REG_AT(I2C_SR3,0x5219);

/* I2C interrupt control register */
DEF_8BIT_REG_AT(I2C_ITR,0x521a);

/* I2C Clock control register low */
DEF_8BIT_REG_AT(I2C_CCRL,0x521b);

/* I2C Clock control register high */
DEF_8BIT_REG_AT(I2C_CCRH,0x521c);

/* I2C TRISE register */
DEF_8BIT_REG_AT(I2C_TRISER,0x521d);

/* I2C packet error checking register */
DEF_8BIT_REG_AT(I2C_PECR,0x521e);

/* Universal synch/asynch receiver transmitter (UART1) */
/*****************************************************************/

/* UART1 Status Register */
DEF_8BIT_REG_AT(UART1_SR,0x5230);

/* UART1 Data Register */
DEF_8BIT_REG_AT(UART1_DR,0x5231);

/* UART1 Baud Rate Register 1 */
DEF_8BIT_REG_AT(UART1_BRR1,0x5232);

/* UART1 Baud Rate Register 2 */
DEF_8BIT_REG_AT(UART1_BRR2,0x5233);

/* UART1 Control Register 1 */
DEF_8BIT_REG_AT(UART1_CR1,0x5234);

/* UART1 Control Register 2 */
DEF_8BIT_REG_AT(UART1_CR2,0x5235);

/* UART1 Control Register 3 */
DEF_8BIT_REG_AT(UART1_CR3,0x5236);

/* UART1 Control Register 4 */
DEF_8BIT_REG_AT(UART1_CR4,0x5237);

/* UART1 Control Register 5 */
DEF_8BIT_REG_AT(UART1_CR5,0x5238);

/* UART1 Guard time Register */
DEF_8BIT_REG_AT(UART1_GTR,0x5239);

/* UART1 Prescaler Register */
DEF_8BIT_REG_AT(UART1_PSCR,0x523a);

/* LIN Universal asynch. receiver transmitter (UART3) */
/*****************************************************************/

/* UART3 Status Register */
DEF_8BIT_REG_AT(UART3_SR,0x5240);

/* UART3 Data Register */
DEF_8BIT_REG_AT(UART3_DR,0x5241);

/* UART3 Baud Rate Register 1 */
DEF_8BIT_REG_AT(UART3_BRR1,0x5242);

/* UART3 Baud Rate Register 2 */
DEF_8BIT_REG_AT(UART3_BRR2,0x5243);

/* UART3 Control Register 1 */
DEF_8BIT_REG_AT(UART3_CR1,0x5244);

/* UART3 Control Register 2 */
DEF_8BIT_REG_AT(UART3_CR2,0x5245);

/* UART3 Control Register 3 */
DEF_8BIT_REG_AT(UART3_CR3,0x5246);

/* UART3 Control Register 4 */
DEF_8BIT_REG_AT(UART3_CR4,0x5247);

/* UART3 Control Register 6 */
DEF_8BIT_REG_AT(UART3_CR6,0x5249);

/* 16-Bit Timer 1 (TIM1) */
/*****************************************************************/

/* TIM1 Control register 1 */
DEF_8BIT_REG_AT(TIM1_CR1,0x5250);

/* TIM1 Control register 2 */
DEF_8BIT_REG_AT(TIM1_CR2,0x5251);

/* TIM1 Slave Mode Control register */
DEF_8BIT_REG_AT(TIM1_SMCR,0x5252);

/* TIM1 external trigger register */
DEF_8BIT_REG_AT(TIM1_ETR,0x5253);

/* TIM1 Interrupt enable register */
DEF_8BIT_REG_AT(TIM1_IER,0x5254);

/* TIM1 Status register 1 */
DEF_8BIT_REG_AT(TIM1_SR1,0x5255);

/* TIM1 Status register 2 */
DEF_8BIT_REG_AT(TIM1_SR2,0x5256);

/* TIM1 Event Generation register */
DEF_8BIT_REG_AT(TIM1_EGR,0x5257);

/* TIM1 Capture/Compare mode register 1 */
DEF_8BIT_REG_AT(TIM1_CCMR1,0x5258);

/* TIM1 Capture/Compare mode register 2 */
DEF_8BIT_REG_AT(TIM1_CCMR2,0x5259);

/* TIM1 Capture/Compare mode register 3 */
DEF_8BIT_REG_AT(TIM1_CCMR3,0x525a);

/* TIM1 Capture/Compare mode register 4 */
DEF_8BIT_REG_AT(TIM1_CCMR4,0x525b);

/* TIM1 Capture/Compare enable register 1 */
DEF_8BIT_REG_AT(TIM1_CCER1,0x525c);

/* TIM1 Capture/Compare enable register 2 */
DEF_8BIT_REG_AT(TIM1_CCER2,0x525d);

/* TIM1 Counter */
DEF_16BIT_REG_AT(TIM1_CNTR,0x525e);
/* Data bits High */
DEF_8BIT_REG_AT(TIM1_CNTRH,0x525e);
/* Data bits Low */
DEF_8BIT_REG_AT(TIM1_CNTRL,0x525f);

/* TIM1 Prescaler register */
DEF_16BIT_REG_AT(TIM1_PSCR,0x5260);
/* Data bits High */
DEF_8BIT_REG_AT(TIM1_PSCRH,0x5260);
/* Data bits Low */
DEF_8BIT_REG_AT(TIM1_PSCRL,0x5261);

/* TIM1 Auto-reload register */
DEF_16BIT_REG_AT(TIM1_ARR,0x5262);
/* Data bits High */
DEF_8BIT_REG_AT(TIM1_ARRH,0x5262);
/* Data bits Low */
DEF_8BIT_REG_AT(TIM1_ARRL,0x5263);

/* TIM1 Repetition counter register */
DEF_8BIT_REG_AT(TIM1_RCR,0x5264);

/* TIM1 Capture/Compare register 1 */
DEF_16BIT_REG_AT(TIM1_CCR1,0x5265);
/* Data bits High */
DEF_8BIT_REG_AT(TIM1_CCR1H,0x5265);
/* Data bits Low */
DEF_8BIT_REG_AT(TIM1_CCR1L,0x5266);

/* TIM1 Capture/Compare register 2 */
DEF_16BIT_REG_AT(TIM1_CCR2,0x5267);
/* Data bits High */
DEF_8BIT_REG_AT(TIM1_CCR2H,0x5267);
/* Data bits Low */
DEF_8BIT_REG_AT(TIM1_CCR2L,0x5268);

/* TIM1 Capture/Compare register 3 */
DEF_16BIT_REG_AT(TIM1_CCR3,0x5269);
/* Data bits High */
DEF_8BIT_REG_AT(TIM1_CCR3H,0x5269);
/* Data bits Low */
DEF_8BIT_REG_AT(TIM1_CCR3L,0x526a);

/* TIM1 Capture/Compare register 4 */
DEF_16BIT_REG_AT(TIM1_CCR4,0x526b);
/* Data bits High */
DEF_8BIT_REG_AT(TIM1_CCR4H,0x526b);
/* Data bits Low */
DEF_8BIT_REG_AT(TIM1_CCR4L,0x526c);

/* TIM1 Break register */
DEF_8BIT_REG_AT(TIM1_BKR,0x526d);

/* TIM1 Dead-time register */
DEF_8BIT_REG_AT(TIM1_DTR,0x526e);

/* TIM1 Output idle state register */
DEF_8BIT_REG_AT(TIM1_OISR,0x526f);

/* 16-Bit Timer 2 (TIM2) */
/*****************************************************************/

/* TIM2 Control register 1 */
DEF_8BIT_REG_AT(TIM2_CR1,0x5300);

/* TIM2 Interrupt enable register */
DEF_8BIT_REG_AT(TIM2_IER,0x5301);

/* TIM2 Status register 1 */
DEF_8BIT_REG_AT(TIM2_SR1,0x5302);

/* TIM2 Status register 2 */
DEF_8BIT_REG_AT(TIM2_SR2,0x5303);

/* TIM2 Event Generation register */
DEF_8BIT_REG_AT(TIM2_EGR,0x5304);

/* TIM2 Capture/Compare mode register 1 */
DEF_8BIT_REG_AT(TIM2_CCMR1,0x5305);

/* TIM2 Capture/Compare mode register 2 */
DEF_8BIT_REG_AT(TIM2_CCMR2,0x5306);

/* TIM2 Capture/Compare mode register 3 */
DEF_8BIT_REG_AT(TIM2_CCMR3,0x5307);

/* TIM2 Capture/Compare enable register 1 */
DEF_8BIT_REG_AT(TIM2_CCER1,0x5308);

/* TIM2 Capture/Compare enable register 2 */
DEF_8BIT_REG_AT(TIM2_CCER2,0x5309);

/* TIM2 Counter */
DEF_16BIT_REG_AT(TIM2_CNTR,0x530a);
/* Data bits High */
DEF_8BIT_REG_AT(TIM2_CNTRH,0x530a);
/* Data bits Low */
DEF_8BIT_REG_AT(TIM2_CNTRL,0x530b);

/* TIM2 Prescaler register */
DEF_8BIT_REG_AT(TIM2_PSCR,0x530c);

/* TIM2 Auto-reload register */
DEF_16BIT_REG_AT(TIM2_ARR,0x530d);
/* Data bits High */
DEF_8BIT_REG_AT(TIM2_ARRH,0x530d);
/* Data bits Low */
DEF_8BIT_REG_AT(TIM2_ARRL,0x530e);

/* TIM2 Capture/Compare register 1 */
DEF_16BIT_REG_AT(TIM2_CCR1,0x530f);
/* Data bits High */
DEF_8BIT_REG_AT(TIM2_CCR1H,0x530f);
/* Data bits Low */
DEF_8BIT_REG_AT(TIM2_CCR1L,0x5310);

/* TIM2 Capture/Compare register 2 */
DEF_16BIT_REG_AT(TIM2_CCR2,0x5311);
/* Data bits High */
DEF_8BIT_REG_AT(TIM2_CCR2H,0x5311);
/* Data bits Low */
DEF_8BIT_REG_AT(TIM2_CCR2L,0x5312);

/* TIM2 Capture/Compare register 3 */
DEF_16BIT_REG_AT(TIM2_CCR3,0x5313);
/* Data bits High */
DEF_8BIT_REG_AT(TIM2_CCR3H,0x5313);
/* Data bits Low */
DEF_8BIT_REG_AT(TIM2_CCR3L,0x5314);

/* 16-Bit Timer 3 (TIM3) */
/*****************************************************************/

/* TIM3 Control register 1 */
DEF_8BIT_REG_AT(TIM3_CR1,0x5320);

/* TIM3 Interrupt enable register */
DEF_8BIT_REG_AT(TIM3_IER,0x5321);

/* TIM3 Status register 1 */
DEF_8BIT_REG_AT(TIM3_SR1,0x5322);

/* TIM3 Status register 2 */
DEF_8BIT_REG_AT(TIM3_SR2,0x5323);

/* TIM3 Event Generation register */
DEF_8BIT_REG_AT(TIM3_EGR,0x5324);

/* TIM3 Capture/Compare mode register 1 */
DEF_8BIT_REG_AT(TIM3_CCMR1,0x5325);

/* TIM3 Capture/Compare mode register 2 */
DEF_8BIT_REG_AT(TIM3_CCMR2,0x5326);

/* TIM3 Capture/Compare enable register 1 */
DEF_8BIT_REG_AT(TIM3_CCER1,0x5327);

/* TIM3 Counter */
DEF_16BIT_REG_AT(TIM3_CNTR,0x5328);
/* Data bits High */
DEF_8BIT_REG_AT(TIM3_CNTRH,0x5328);
/* Data bits Low */
DEF_8BIT_REG_AT(TIM3_CNTRL,0x5329);

/* TIM3 Prescaler register */
DEF_8BIT_REG_AT(TIM3_PSCR,0x532a);

/* TIM3 Auto-reload register */
DEF_16BIT_REG_AT(TIM3_ARR,0x532b);
/* Data bits High */
DEF_8BIT_REG_AT(TIM3_ARRH,0x532b);
/* Data bits Low */
DEF_8BIT_REG_AT(TIM3_ARRL,0x532c);

/* TIM3 Capture/Compare register */
DEF_16BIT_REG_AT(TIM3_CCR1,0x532d);
/* Data bits High */
DEF_8BIT_REG_AT(TIM3_CCR1H,0x532d);
/* Data bits Low */
DEF_8BIT_REG_AT(TIM3_CCR1L,0x532e);

/* TIM3 Capture/Compare register */
DEF_16BIT_REG_AT(TIM3_CCR2,0x532f);
/* Data bits High */
DEF_8BIT_REG_AT(TIM3_CCR2H,0x532f);
/* Data bits Low */
DEF_8BIT_REG_AT(TIM3_CCR2L,0x5330);

/* 8-Bit  Timer 4 (TIM4) */
/*****************************************************************/

/* TIM4 Control register 1 */
DEF_8BIT_REG_AT(TIM4_CR1,0x5340);

/* TIM4 Interrupt enable register */
DEF_8BIT_REG_AT(TIM4_IER,0x5341);

/* TIM4 Status register */
DEF_8BIT_REG_AT(TIM4_SR,0x5342);

/* TIM4 Event Generation register */
DEF_8BIT_REG_AT(TIM4_EGR,0x5343);

/* TIM4 Counter */
DEF_8BIT_REG_AT(TIM4_CNTR,0x5344);

/* TIM4 Prescaler register */
DEF_8BIT_REG_AT(TIM4_PSCR,0x5345);

/* TIM4 Auto-reload register */
DEF_8BIT_REG_AT(TIM4_ARR,0x5346);

/* 10-Bit A/D Converter (ADC) */
/*****************************************************************/

/* ADC Control/Status Register */
DEF_8BIT_REG_AT(ADC_CSR,0x5400);

/* ADC Configuration Register 1 */
DEF_8BIT_REG_AT(ADC_CR1,0x5401);

/* ADC Configuration Register 2 */
DEF_8BIT_REG_AT(ADC_CR2,0x5402);

/* ADC Data Register */
DEF_16BIT_REG_AT(ADC_DR,0x5404);
/* Data bits High */
DEF_8BIT_REG_AT(ADC_DRH,0x5404);
/* Data bits Low */
DEF_8BIT_REG_AT(ADC_DRL,0x5405);

/* ADC Schmitt Trigger Disable Register */
DEF_16BIT_REG_AT(ADC_TDR,0x5406);
/* Schmitt trigger disable High */
DEF_8BIT_REG_AT(ADC_TDRH,0x5406);
/* Schmitt trigger disable Low */
DEF_8BIT_REG_AT(ADC_TDRL,0x5407);

/*  Controller Area Network (CAN) */
/*****************************************************************/

/* Master Control Register */
DEF_8BIT_REG_AT(CAN_MCR,0x5420);

/* Master Status Register */
DEF_8BIT_REG_AT(CAN_MSR,0x5421);

/* Transmit Status Register */
DEF_8BIT_REG_AT(CAN_TSR,0x5422);

/* Transmit Priority Register */
DEF_8BIT_REG_AT(CAN_TPR,0x5423);

/* Receive FIFO Register */
DEF_8BIT_REG_AT(CAN_RFR,0x5424);

/* Interrupt Enable Register */
DEF_8BIT_REG_AT(CAN_IER,0x5425);

/* Diagnosis Register */
DEF_8BIT_REG_AT(CAN_DGR,0x5426);

/* Filter Page Select Register */
DEF_8BIT_REG_AT(CAN_FPSR,0x5427);

/* MCSR, MCSR, F0R1,  F2R1, F4R1, MCSR, ESR, MFMI - CAN Paged Register 0 */
DEF_8BIT_REG_AT(CAN_P0,0x5428);

/* MDLCR, MDLCR, F0R2, F2R2, F4R2, MDLCR, EIER, MDLCR - CAN Paged  Register 1 */
DEF_8BIT_REG_AT(CAN_P1,0x5429);

/* MIDR1, MIDR1, F0R3, F2R3, F4R3, MIDR1, TECR,  MIDR1 - CAN Paged Register 2 */
DEF_8BIT_REG_AT(CAN_P2,0x542a);

/* MIDR2, MIDR2, F0R4, F2R4, F4R4, MIDR2, RECR,  MIDR2 - CAN Paged Register 3 */
DEF_8BIT_REG_AT(CAN_P3,0x542b);

/* MIDR3, MIDR3, F0R5, F2R5, F4R5, MIDR3, BTCR1, MIDR3 - CAN Paged Register 4 */
DEF_8BIT_REG_AT(CAN_P4,0x542c);

/* MIDR4, MIDR4, F0R6, F2R6, F4R6, MIDR4, BTCR2, MIDR4 - CAN Paged Register 5 */
DEF_8BIT_REG_AT(CAN_P5,0x542d);

/* MDAR1, MDAR1, F0R7, F2R7, F4R7, MDAR1, Reserved, MDAR1 - CAN Paged Register 6 */
DEF_8BIT_REG_AT(CAN_P6,0x542e);

/* MDAR2, MDAR2, F0R8, F2R8, F4R8, MDAR2, Reserved, MDAR2 - CAN Paged Register 7 */
DEF_8BIT_REG_AT(CAN_P7,0x542f);

/* MDAR3, MDAR3, F1R1, F3R1, F5R1, MDAR3, FMR1, MDAR3 - CAN Paged  Register 8 */
DEF_8BIT_REG_AT(CAN_P8,0x5430);

/* MDAR4, MDAR4, F1R2, F3R2, F5R2, MDAR4, FMR1, MDAR4 - CAN Paged  Register 9 */
DEF_8BIT_REG_AT(CAN_P9,0x5431);

/* MDAR5, MDAR5, F1R3, F3R3, F5R3, MDAR5, FCR2, MDAR5 - CAN Paged  Register A */
DEF_8BIT_REG_AT(CAN_PA,0x5432);

/* MDAR6, MDAR6, F1R4, F3R4, F5R4, MDAR6, FCR3, MDAR6 - CAN Paged  Register B */
DEF_8BIT_REG_AT(CAN_PB,0x5433);

/* MDAR7, MDAR7, F1R5, F3R5, F5R5, MDAR7, FCR4, MDAR7 - CAN Paged  Register C */
DEF_8BIT_REG_AT(CAN_PC,0x5434);

/* MDAR8, MDAR8, F1R6, F3R6, F5R6, MDAR8, Reserved, MDAR8 - CAN Paged Register D */
DEF_8BIT_REG_AT(CAN_PD,0x5435);

/* MTSRL, MTSRL, F1R7, F3R7, F5R7, MTSLR, Reserved, MTSRL - CAN Paged Register E */
DEF_8BIT_REG_AT(CAN_PE,0x5436);

/* MTSRH, MTSRH, F1R8, F3R8, F5R8, MTSRH, Reserved, MTSRH - CAN Paged Register F */
DEF_8BIT_REG_AT(CAN_PF,0x5437);

/*  Global configuration register (CFG) */
/*****************************************************************/

/* CFG Global configuration register */
DEF_8BIT_REG_AT(CFG_GCR,0x7f60);

/* Interrupt Software Priority Register (ITC) */
/*****************************************************************/

/* Interrupt Software priority register 1 */
DEF_8BIT_REG_AT(ITC_SPR1,0x7f70);

/* Interrupt Software priority register 2 */
DEF_8BIT_REG_AT(ITC_SPR2,0x7f71);

/* Interrupt Software priority register 3 */
DEF_8BIT_REG_AT(ITC_SPR3,0x7f72);

/* Interrupt Software priority register 4 */
DEF_8BIT_REG_AT(ITC_SPR4,0x7f73);

/* Interrupt Software priority register 5 */
DEF_8BIT_REG_AT(ITC_SPR5,0x7f74);

/* Interrupt Software priority register 6 */
DEF_8BIT_REG_AT(ITC_SPR6,0x7f75);

/* Interrupt Software priority register 7 */
DEF_8BIT_REG_AT(ITC_SPR7,0x7f76);

#endif /* __STM8S208R__ */
