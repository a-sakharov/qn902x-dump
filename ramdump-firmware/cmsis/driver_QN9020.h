/**
 ****************************************************************************************
 *
 * @file driver_QN9020.h
 *
 * @brief QN9020 Device Driver Header File.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef QN9020_DRIVER_H
#define QN9020_DRIVER_H

///@cond
/** @defgroup DRIVERS QN9020 Drivers
 *   This file defines all QN9020 Driver functions for the following modules:
 *   - GPIO
 *   - UART
 *   - SPI
 *   - I2C
 *   - Timer
 *   - PWM
 *   - RTC
 *   - WDT
 *   - DMA
 *   - ADC
 *   - Analog
 *   - SerialFlash
 *   - Sleep
 *   - SystemController
 */
///@endcond


#ifdef __cplusplus
extern "C" {
#endif

#include "QN9020.h"
#include "app_config.h"

/*----------------------------------------------------------------------------
  DEFINES
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/
/* ToDo: add here your necessary defines for device initialization
         following is an example for different system frequencies             */


#define outp8(addr, value)        (*((volatile  uint8_t *)(addr)) = value)
#define inp8(addr)                (*((volatile  uint8_t *)(addr)))
#define outp16(addr, value)       (*((volatile uint16_t *)(addr)) = value)
#define inp16(addr)               (*((volatile uint16_t *)(addr)))
#define outp32(addr, value)       (*((volatile uint32_t *)(addr)) = value)
#define inp32(addr)               (*((volatile uint32_t *)(addr)))


#define BYPASS                    0x1
#define DIV(x)                    (2 * ((x) + 1))


/// Enable Mask
#define MASK_ENABLE               (0xFFFFFFFF)
/// Disable Mask
#define MASK_DISABLE              (0x00000000)

#ifndef FALSE
#define FALSE                     (0)
#endif

#ifndef TRUE
#define TRUE                      (1)
#endif

// external clock value
#define XTAL_16MHz                (16000000UL)        /*!< XTAL is 16MHz */
#define XTAL_32MHz                (32000000UL)        /*!< XTAL is 32MHz */

// system clock, select by CLK_MUX @ CLK_MUX_DIV_CTRL[31:30], (M0 core/ADC)
#define SYS_EXT_XTAL              (__XTAL)            /*!< System clock is external XTAL */
#define SYS_INT_20M               (20000000UL)        /*!< System clock is internal 20MHz */
#define SYS_PLL_32M               (32000000UL)        /*!< System clock is PLL 32MHz */
#define SYS_LOW_32K               (32000UL)           /*!< System clock is low speed 32KHz */

// M0 clock
#define CLK_1M                    (1000000UL)         /*!< Clock is 1MHz */
#define CLK_2M                    (2000000UL)         /*!< Clock is 2MHz */
#define CLK_4M                    (4000000UL)         /*!< Clock is 4MHz */
#define CLK_8M                    (8000000UL)         /*!< Clock is 8MHz */
#define CLK_16M                   (16000000UL)        /*!< Cock is 16MHz */
#define CLK_32M                   (32000000UL)        /*!< Clock is 32MHz */

/**
 ****************************************************************************************
 * @brief  delay cycle number.
 * @param[in] dly   number
 *
 * delay duration = dly * 9 + 28 cycles
 *****************************************************************************************
 */
__STATIC_INLINE void delay(uint32_t dly)
{
    int i;
    for (i=0;i<dly;i++){
        __ISB();
    }
    return;
}

/// @cond

/**
 ****************************************************************************************
 * @defgroup QN_Driver QN9020 Inline Driver Definitions
 * @ingroup DRIVERS
 * @brief QN9020 Inline Driver Definitions
 * @{
 ****************************************************************************************
 */

//=================================== driver =========================================//

#if defined(QN_9020_B2)
extern uint32_t __rd_reg(uint32_t addr);
extern void __wr_reg(uint32_t addr, uint32_t val);
extern void __wr_reg_with_msk(uint32_t addr, uint32_t msk, uint32_t val);
#elif defined(QN_9020_B4)
#define __rd_reg(addr)                      (*(volatile uint32_t *)(addr))
#define __wr_reg(addr, val)                 (*(volatile uint32_t *)(addr)) = (val)
#define __wr_reg_with_msk(addr, msk, val)   (*(volatile uint32_t *)(addr)) = ((*(volatile uint32_t *)(addr)) & (~msk) | (msk & val))
#endif


/*SYSCON driver functions*/

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @param value The value to set to CRSS register
 *
 * @brief  outputs specified value to CRSS register
 *
 *  |      31 : GATING_TIMER3
 *  |      30 : GATING_TIMER2
 *  |      29 : GATING_TIMER1
 *  |      28 : GATING_TIMER0
 *  |      27 : GATING_UART1
 *  |      26 : GATING_UART0
 *  |      25 : GATING_SPI1
 *  |      24 : GATING_SPI0
 *  |      23 : GATING_32K_CLK
 *  |      22 : GATING_SPI_AHB
 *  |      21 : GATING_GPIO
 *  |      20 : GATING_ADC
 *  |      19 : GATING_DMA
 *  |      18 : GATING_BLE_AHB
 *  |      17 : RSVD
 *  |      16 : REBOOT_SYS
 *  |      15 : LOCKUP_RST
 *  |      14 : BLE_RST
 *  |      13 : DP_RST
 *  |      12 : DPREG_RST
 *  |      11 : RTC_RST
 *  |      10 : I2C_RST
 *  |       9 : GPIO_RST
 *  |       8 : WDOG_RST
 *  |       7 : TIMER3_RST
 *  |       6 : TIMER2_RST
 *  |       5 : TIMER1_RST
 *  |       4 : TIMER0_RST
 *  |       3 : USART1_RST
 *  |       2 : USART0_RST
 *  |       1 : DMA_RST
 *  |       0 : CPU_RST
 */
 __STATIC_INLINE void syscon_SetCRSS(QN_SYSCON_TypeDef *SYSCON, uint32_t value)
 {
     __wr_reg((uint32_t)&SYSCON->CRSS, value);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @return CRSS register value
 *
 * @brief  returns a uint32_t which read from CRSS register
 */
 __STATIC_INLINE uint32_t syscon_GetCRSS(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->CRSS);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @param value The value to set to CRSC register
 *
 * @brief  outputs specified value to CRSC register
 *
 *  |      31 : GATING_TIMER3
 *  |      30 : GATING_TIMER2
 *  |      29 : GATING_TIMER1
 *  |      28 : GATING_TIMER0
 *  |      27 : GATING_UART1
 *  |      26 : GATING_UART0
 *  |      25 : GATING_SPI1
 *  |      24 : GATING_SPI0
 *  |      23 : GATING_32K_CLK
 *  |      22 : GATING_SPI_AHB
 *  |      21 : GATING_GPIO
 *  |      20 : GATING_ADC
 *  |      19 : GATING_DMA
 *  |      18 : GATING_BLE_AHB
 *  |      17 : RSVD
 *  |      16 : RSVD
 *  |      15 : LOCKUP_RST
 *  |      14 : BLE_RST
 *  |      13 : DP_RST
 *  |      12 : DPREG_RST
 *  |      11 : 32K_RST
 *  |      10 : I2C_RST
 *  |       9 : GPIO_RST
 *  |       8 : WDOG_RST
 *  |       7 : TIMER3_RST
 *  |       6 : TIMER2_RST
 *  |       5 : TIMER1_RST
 *  |       4 : TIMER0_RST
 *  |       3 : USART1_RST
 *  |       2 : USART0_RST
 *  |       1 : DMA_RST
 *  |       0 : CPU_RST
 */
 __STATIC_INLINE void syscon_SetCRSC(QN_SYSCON_TypeDef *SYSCON, uint32_t value)
 {
     __wr_reg((uint32_t)&SYSCON->CRSC, value);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @return CRSC register value
 *
 * @brief  returns a uint32_t which read from CRSC register
 */
 __STATIC_INLINE uint32_t syscon_GetCRSC(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->CRSC);
 }


/**
 *
 * @param *SYSCON SYSCON Pointer
 * @param value The value to set to CMDCR register
 *
 * @brief  outputs specified value to CMDCR register
 *
 *  | 31 - 12 : CLK_MUX
 *  |      11 : SEL_CLK_32K
 *  |      10 : BLE_FRQ_SEL
 *  |       9 : BLE_DIV_BYPASS
 *  |       8 : BLE_DIVIDER
 *  |      25 : AHB_DIV_BYPASS
 *  | 24 - 16 : AHB_DIVIDER
 *  |      15 : USART1_DIV_BYPASS
 *  | 14 - 12 : USART1_DIVIDER
 *  |      11 : USART0_DIV_BYPASS
 *  | 10 -  8 : USART0_DIVIDER
 *  |       7 : RSVD
 *  |       6 : APB_DIV_BYPASS
 *  |  5 -  4 : APB_DIVIDER
 *  |       3 : TIMER_DIV_BYPASS
 *  |       0 : TIMER_DIVIDER
 */
 __STATIC_INLINE void syscon_SetCMDCR(QN_SYSCON_TypeDef *SYSCON, uint32_t value)
 {
     __wr_reg((uint32_t)&SYSCON->CMDCR, value);
 }

 __STATIC_INLINE void syscon_SetCMDCRWithMask(QN_SYSCON_TypeDef *SYSCON, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&SYSCON->CMDCR, mask, value);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @return CMDCR register value
 *
 * @brief  returns a uint32_t which read from CMDCR register
 */
 __STATIC_INLINE uint32_t syscon_GetCMDCR(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->CMDCR);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @param value The value to set to STCR register
 *
 * @brief  outputs specified value to STCR register
 *
 *  |       31 : EN_STCLKEN
 *  |  30 - 26 : RSVD
 *  |  25 -  0 : STCALIB
 */
 __STATIC_INLINE void syscon_SetSTCR(QN_SYSCON_TypeDef *SYSCON, uint32_t value)
 {
     __wr_reg((uint32_t)&SYSCON->STCR, value);
 }

 __STATIC_INLINE void syscon_SetSTCRWithMask(QN_SYSCON_TypeDef *SYSCON, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&SYSCON->STCR, mask, value);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @return STCR register value
 *
 * @brief  returns a uint32_t which read from STCR register
 */
 __STATIC_INLINE uint32_t syscon_GetSTCR(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->STCR);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @param value The value to set to SOCR register
 *
 * @brief  outputs specified value to SOCR register
 *
 *  |  30 - 14 : RSVD
 *  |  13 -  0 : EM_BASE_ADDR
 */
 __STATIC_INLINE void syscon_SetSOCR(QN_SYSCON_TypeDef *SYSCON, uint32_t value)
 {
     __wr_reg((uint32_t)&SYSCON->SOCR, value);
 }

 __STATIC_INLINE void syscon_SetSOCRWithMask(QN_SYSCON_TypeDef *SYSCON, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&SYSCON->SOCR, mask, value);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @return SOCR register value
 *
 * @brief  returns a uint32_t which read from SOCR register
 */
 __STATIC_INLINE uint32_t syscon_GetSOCR(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->SOCR);
 }


/**
 *
 * @param *SYSCON SYSCON Pointer
 * @param value The value to set to PMCR0 register
 *
 * @brief  outputs specified value to PMCR0 register
 *
-            00(Default)     01              10              11
- [1:0]      GPIO[0](I/O)    UART0_TXD(O)    SPI0_DAT(I/O)   RTCI(I)           P0_0
- [3:2]      GPIO[1](I/O)    NC              SPI0_CS0(I/O)   UART0_CTSn(I)     P0_1
- [5:4]      GPIO[2](I/O)    I2C_SDA(I/O)    SPI0_CLK(I/O)   UART0_RTSn(O)     P0_2
- [7:6]      GPIO[3](I/O)    RADIO_EN(O)     CLKOUT0(O)      TIMER0_eclk(I/O)  P0_3
- [9:8]      GPIO[4](I/O)    NC              CLKOUT1(O)      RTCI(I)           P0_4
- [11:10]    GPIO[5](I/O)    I2C_SCL(I/O)    ADCT(I)         ACMP1_O(O)        P0_5
- [13:12]    SW_DAT(I/O)     GPIO[6](I/O)    AIN2(AI)        ACMP1-(AI)        P0_6
- [15:14]    SW_CLK(I)       GPIO[7](I/O)    AIN3(AI)        ACMP1+(AI)        P0_7
- [17:16]    GPIO[8](I/O)    SPI1_DIN(I)     UART1_RXD(I)    TIMER2_eclk(I/O)  P1_0
- [19:18]    GPIO[9](I/O)    SPI1_DAT(I/O)   UART1_TXD(O)    TIMER1_0(I/O)     P1_1
- [21:20]    GPIO[10](I/O)   SPI1_CS0(I/O)   UART1_CTSn(I)   ADCT(I)           P1_2
- [23:22]    GPIO[11](I/O)   SPI1_CLK(I/O)   UART1_RTSn(O)   CLKOUT1(O)        P1_3
- [25:24]    GPIO[12](I/O)   RDYN(O)         NC              TIMER1_3(I/O)     P1_4
- [27:26]    GPIO[13](I/O)   RADIO_EN(O)     PWM1(O)         TIMER1_2(I/O)     P1_5
- [29:28]    GPIO[14](I/O)   SPI0_CS1_O(O)   PWM0(O)         TIMER0_3(I/O)     P1_6
- [31:30]    GPIO[15](I/O)   UART0_RXD(I)    SPI0_DIN(I)     TIMER0_o(O)       P1_7
*/
 __STATIC_INLINE void syscon_SetPMCR0(QN_SYSCON_TypeDef *SYSCON, uint32_t value)
 {
     __wr_reg((uint32_t)&SYSCON->PMCR0, value);
 }

 __STATIC_INLINE void syscon_SetPMCR0WithMask(QN_SYSCON_TypeDef *SYSCON, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&SYSCON->PMCR0, mask, value);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @return PMCR0 register value
 *
 * @brief  returns a uint32_t which read from PMCR0 register
 */
 __STATIC_INLINE uint32_t syscon_GetPMCR0(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->PMCR0);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @param value The value to set to PMCR1 register
 *
 * @brief  outputs specified value to PMCR1 register
 *
-            00(Default)     01              10              11
- [1:0]      GPIO[16](I/O)   SPI1_DIN(I)     UART1_RXD(I)    TIMER3_2(I/O)     P2_0
- [3:2]      GPIO[17](I/O)   SPI1_DAT(I/O)   UART1_TXD(O)    TIMER3_1(I/O)     P2_1
- [5:4]      GPIO[18](I/O)   SPI1_CLK(I/O)   UART1_RTSn(O)   TIMER2_3(I/O)     P2_2
- [7:6]      GPIO[19](I/O)   I2C_SDA(I/O)    ACMP0_O(O)      TIMER3_0(I/O)     P2_3
- [9:8]      GPIO[20](I/O)   I2C_SCL(I/O)    PWM1(O)         TIMER3_eclk(I/O)  P2_4
- [11:10]    GPIO[21](I/O)   SPI1_CS1_O(O)   NC              TIMER2_2(I/O)     P2_5
- [13:12]    GPIO[22](I/O)   Antenna_O(O)    PWM1(O)         TIMER2_0(I/O)     P2_6
- [15:14]    GPIO[23](I/O)   ACMP1_O(O)      PWM0(O)         TIMER1_eclk(I/O)  P2_7
- [17:16]    GPIO[24](I/O)   TIMER2_1(I/O)   AIN0(AI)        ACMP0-(AI)        P3_0
- [19:18]    GPIO[25](I/O)   TIMER0_2(I/O)   AIN1(AI)        ACMP0+(AI)        P3_1
- [21:20]    GPIO[26](I/O)   SPI0_DIN(I)     NC              ACMP0_O(O)        P3_2
- [23:22]    GPIO[27](I/O)   SPI0_DAT(I/O)   CLKOUT0(O)      NC                P3_3
- [24]       GPIO[28](I/O)   SPI0_CLK(I/O)   NC              NC                P3_4
- [26:25]    GPIO[29](I/O)   SPI0_CS0(I/O)   INT_FM(I)       TIMER0_0(I/O)     P3_5
- [28:27]    GPIO[30](I/O)   SPI1_CS0(I/O)   UART1_CTSn(I)   NC                P3_6
- [29]       TEST_ENABLE[0]
- [30]       TEST_ENABLE[1]
- [31]       FLASH_PIN_CTRL
 */
 __STATIC_INLINE void syscon_SetPMCR1(QN_SYSCON_TypeDef *SYSCON, uint32_t value)
 {
     __wr_reg((uint32_t)&SYSCON->PMCR1, value);
 }

 __STATIC_INLINE void syscon_SetPMCR1WithMask(QN_SYSCON_TypeDef *SYSCON, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&SYSCON->PMCR1, mask, value);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @return PMCR1 register value
 *
 * @brief  returns a uint32_t which read from PMCR1 register
 */
 __STATIC_INLINE uint32_t syscon_GetPMCR1(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->PMCR1);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @param value The value to set to PMCR2 register
 *
 * @brief  outputs specified value to PMCR2 register
 *
 *  | 31 - 27 : TEST_CTRL[4 :0]
 *  | 27 -  8 : RSVD
 *  |       7 : CLK_OUT_SEL[1]
 *  |       6 : CLK_OUT_SEL[0]
 *  |       5 : UART1_PIN_SEL
 *  |       4 : I2C_PIN_SEL
 *  |       3 : ADCT_PIN_SEL
 *  |       2 : RSVD
 *  |       1 : SPI0_PIN_SEL
 *  |       0 : SPI1_PIN_SEL
 */
 __STATIC_INLINE void syscon_SetPMCR2(QN_SYSCON_TypeDef *SYSCON, uint32_t value)
 {
     __wr_reg((uint32_t)&SYSCON->PMCR2, value);
 }

 __STATIC_INLINE void syscon_SetPMCR2WithMask(QN_SYSCON_TypeDef *SYSCON, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&SYSCON->PMCR2, mask, value);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @return PMCR2 register value
 *
 * @brief  returns a uint32_t which read from PMCR2 register
 */
 __STATIC_INLINE uint32_t syscon_GetPMCR2(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->PMCR2);
 }


/**
 *
 * @param *SYSCON SYSCON Pointer
 * @param value The value to set to PDCR register
 *
 * @brief  outputs specified value to PDCR register
 *
 *  |      31 : RSVD
 *  | 30 -  0 : PAD_DRV_CTRL
 */
 __STATIC_INLINE void syscon_SetPDCR(QN_SYSCON_TypeDef *SYSCON, uint32_t value)
 {
     __wr_reg((uint32_t)&SYSCON->PDCR, value);
 }

 __STATIC_INLINE void syscon_SetPDCRWithMask(QN_SYSCON_TypeDef *SYSCON, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&SYSCON->PDCR, mask, value);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @return PDCR register value
 *
 * @brief  returns a uint32_t which read from PDCR register
 */
 __STATIC_INLINE uint32_t syscon_GetPDCR(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->PDCR);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @param value The value to set to PPCR0 register
 *
 * @brief  outputs specified value to PPCR0 register
 *
 *  | 31 -  0 : PAD_PULL_CTRL[31:0]
 */
 __STATIC_INLINE void syscon_SetPPCR0(QN_SYSCON_TypeDef *SYSCON, uint32_t value)
 {
     __wr_reg((uint32_t)&SYSCON->PPCR0, value);
 }

 __STATIC_INLINE void syscon_SetPPCR0WithMask(QN_SYSCON_TypeDef *SYSCON, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&SYSCON->PPCR0, mask, value);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @return PPCR0 register value
 *
 * @brief  returns a uint32_t which read from PPCR0 register
 */
 __STATIC_INLINE uint32_t syscon_GetPPCR0(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->PPCR0);
 }


/**
 *
 * @param *SYSCON SYSCON Pointer
 * @param value The value to set to PPCR1 register
 *
 * @brief  outputs specified value to PPCR1 register
 *
 *  | 31 - 30 : RSVD
 *  | 29 -  0 : PAD_PULL_CTRL[61:32]
 */
 __STATIC_INLINE void syscon_SetPPCR1(QN_SYSCON_TypeDef *SYSCON, uint32_t value)
 {
     __wr_reg((uint32_t)&SYSCON->PPCR1, value);
 }

 __STATIC_INLINE void syscon_SetPPCR1WithMask(QN_SYSCON_TypeDef *SYSCON, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&SYSCON->PPCR1, mask, value);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @return PPCR1 register value
 *
 * @brief  returns a uint32_t which read from PPCR1 register
 */
 __STATIC_INLINE uint32_t syscon_GetPPCR1(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->PPCR1);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @param value The value to set to RCS register
 *
 * @brief  outputs specified value to RCS register
 *
 *  |      31 : RST_CAUSE_CLR
 *  | 30 -  8 : RSVD
 *  |  7 -  0 : RESET_CAUSE
 */
 __STATIC_INLINE void syscon_SetRCS(QN_SYSCON_TypeDef *SYSCON, uint32_t value)
 {
     __wr_reg((uint32_t)&SYSCON->RCS, value);
 }

 __STATIC_INLINE void syscon_SetRCSWithMask(QN_SYSCON_TypeDef *SYSCON, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&SYSCON->RCS, mask, value);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @return RCS register value
 *
 * @brief  returns a uint32_t which read from RCS register
 */
 __STATIC_INLINE uint32_t syscon_GetRCS(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->RCS);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @param value The value to set to IOWCR register
 *
 * @brief  outputs specified value to IOWCR register
 *
 *  | 31 - 16 : IO_VALUE
 *  | 15 -  0 : IO_WAKEUP_EN
 */
 __STATIC_INLINE void syscon_SetIOWCR(QN_SYSCON_TypeDef *SYSCON, uint32_t value)
 {
     __wr_reg((uint32_t)&SYSCON->IOWCR, value);
 }

 __STATIC_INLINE void syscon_SetIOWCRWithMask(QN_SYSCON_TypeDef *SYSCON, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&SYSCON->IOWCR, mask, value);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @return IOWCR register value
 *
 * @brief  returns a uint32_t which read from IOWCR register
 */
 __STATIC_INLINE uint32_t syscon_GetIOWCR(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->IOWCR);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @return BLESR register value
 *
 * @brief  returns a uint32_t which read from BLESR register
 *
 *  | 31 - 18 : RSVD
 *  |      17 : CLK_RDY
 *  |      16 : CLK_XTAL32_RDY
 *  |      15 : REF_PLL_RDY
 *  |      14 : BG_RDY
 *  |      13 : BUCK_RDY
 *  |      12 : TX_EN
 *  |      11 : RX_EN
 *  |      10 : OSC_EN
 *  |       9 : CLK_STATUS
 *  |       8 : RADIO_EN
 *  |  7 -  0 : FREQ_WORD
 */
 __STATIC_INLINE uint32_t syscon_GetBLESR(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->BLESR);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @param value The value to set to SMR register
 *
 * @brief  outputs specified value to SMR register
 *
 *  |      31 : BOOT_MODE
 *  |      30 : EN_SW_MAP
 *  | 29 -  6 : RSVD
 *  |       5 : RAM_BIST_FAIL
 *  |       4 : RAM_BIST_END
 *  |       3 : ROM_BIST_FAIL
 *  |       2 : ROM_BIST_END
 *  |       1 : BIST_START
 *  |       0 : TEST_MOD
 */
 __STATIC_INLINE void syscon_SetSMR(QN_SYSCON_TypeDef *SYSCON, uint32_t value)
 {
     __wr_reg((uint32_t)&SYSCON->SMR, value);
 }
 __STATIC_INLINE void syscon_SetSMRWithMask(QN_SYSCON_TypeDef *SYSCON, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&SYSCON->SMR, mask, value);
 }

 __STATIC_INLINE uint32_t syscon_GetSMR(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->SMR);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @return CHIP_ID register value
 *
 * @brief  returns a uint32_t which read from CHIP_ID register
 *
 *  | 31 - 16 : RSVD
 *  | 15 -  0 : CHIP_ID
 */
 __STATIC_INLINE uint32_t syscon_GetChipID(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->CHIP_ID);
 }


/**
 *
 * @param *SYSCON SYSCON Pointer
 * @param value The value to set to PGCR0 register
 *
 * @brief  outputs specified value to PGCR0 register
 *
 *  |      31 : SEL_PD
 *  |      30 : PD_OSC
 *  |      29 : PD_BG
 *  |      28 : PD_V2I
 *  |      27 : PD_BUCK
 *  |      26 : PD_VREG_A
 *  |      25 : PD_VREG_D
 *  |      24 : PD_XTAL
 *  |      23 : PD_XTAL32
 *  |      22 : PD_REF_PLL_B0
 *  |      22 : DIV_RST_SYNC_B1
 *  |      21 : PD_LO_VCO
 *  |      20 : PD_LO_PLL
 *  |      19 : PD_PA
 *  |      18 : PD_LNA
 *  |      17 : PD_LNA_PKDET
 *  |      16 : PD_MIXER
 *  |      15 : PD_PPF_PKDET
 *  |      14 : PD_PPF
 *  |      13 : PD_RX_PKDET
 *  |      12 : PD_RX_ADC
 *  |      11 : PD_SAR_ADC
 *  |      10 : PD_RCO
 *  |       9 : BOND_EN
 *  |       8 : RSVD
 *  |       7 : PD_MEM7
 *  |       6 : PD_MEM6
 *  |       5 : PD_MEM5
 *  |       4 : PD_MEM4
 *  |       3 : PD_MEM3
 *  |       2 : PD_MEM2
 *  |       1 : PD_MEM1
 *  |       0 : PL_VERG_D
 */
 __STATIC_INLINE void syscon_SetPGCR0(QN_SYSCON_TypeDef *SYSCON, uint32_t value)
 {
     __wr_reg((uint32_t)&SYSCON->PGCR0, value);
 }

 __STATIC_INLINE void syscon_SetPGCR0WithMask(QN_SYSCON_TypeDef *SYSCON, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&SYSCON->PGCR0, mask, value);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @return PGCR0 register value
 *
 * @brief  returns a uint32_t which read from PGCR0 register
 */
 __STATIC_INLINE uint32_t syscon_GetPGCR0(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->PGCR0);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @param value The value to set to PGCR1 register
 *
 * @brief  outputs specified value to PGCR1 register
 *
 *  |      31 : VDD_RCO_SET
 *  |      30 : DIS_OSC
 *  |      29 : DIS_BG
 *  |      28 : DIS_V2I
 *  |      27 : DIS_BUCK
 *  |      26 : DIS_VREG_A
 *  |      25 : DIS_VREG_D
 *  |      24 : DIS_XTAL
 *  |      23 : DIS_XTAL32
 *  |      22 : DIS_REF_PLL
 *  |      21 : DIS_LO_VCO
 *  |      20 : DIS_LO_PLL
 *  |      19 : DIS_PA
 *  |      18 : DIS_LNA
 *  |      17 : DIS_LNA_PKDET
 *  |      16 : DIS_MIXER
 *  |      15 : DIS_PPF_PKDET
 *  |      14 : DIS_PPF
 *  |      13 : DIS_RX_PKDET
 *  |      12 : DIS_RX_ADC
 *  |      11 : DIS_SAR_ADC
 *  |      10 : DIS_RCO
 *  |       9 : RSVD
 *  |       8 : RSVD
 *  |       7 : DIS_MEM7
 *  |       6 : DIS_MEM6
 *  |       5 : DIS_MEM5
 *  |       4 : DIS_MEM4
 *  |       3 : DIS_MEM3
 *  |       2 : DIS_MEM2
 *  |       1 : DIS_MEM1
 *  |       0 : DIS_SAR_BUF
 */
 __STATIC_INLINE void syscon_SetPGCR1(QN_SYSCON_TypeDef *SYSCON, uint32_t value)
 {
     __wr_reg((uint32_t)&SYSCON->PGCR1, value);
 }

 __STATIC_INLINE void syscon_SetPGCR1WithMask(QN_SYSCON_TypeDef *SYSCON, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&SYSCON->PGCR1, mask, value);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @return PGCR1 register value
 *
 * @brief  returns a uint32_t which read from PGCR1 register
 */
 __STATIC_INLINE uint32_t syscon_GetPGCR1(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->PGCR1);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @param value The value to set to PGCR2 register
 *
 * @brief  outputs specified value to PGCR2 register
 *
 *  | 31 - 18 : RSVD
 *  | 77 - 16 : VREG12_A_BAK
 *  | 12 -  9 : RSVD
 *  |       8 : OSC_WAKEUP_EN
 *  |       7 : RTCI_PIN_SEL
 *  |       6 : PD_STATE
 *  |       5 : BD_AMP_EN
 *  |       4 : DVDD12_PMU_SET
 *  |       3 : RX_EN_SEL
 *  |       2 : FLASH_VCC_EN
 *  |       1 : PMUENABLE
 *  |       0 : DBGPMUENABLE
 */
 __STATIC_INLINE void syscon_SetPGCR2(QN_SYSCON_TypeDef *SYSCON, uint32_t value)
 {
     __wr_reg((uint32_t)&SYSCON->PGCR2, value);
 }

 __STATIC_INLINE void syscon_SetPGCR2WithMask(QN_SYSCON_TypeDef *SYSCON, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&SYSCON->PGCR2, mask, value);
 }

 __STATIC_INLINE uint32_t syscon_GetPGCR2(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->PGCR2);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @param value The value to set to GCR register
 *
 * @brief  outputs specified value to GCR register
 *
 *  |      31 : TX_PWR_SEL
 *  |      30 : PA_GAIN_BOOST
 *  | 29 - 28 : BM_PA
 *  | 27 - 24 : PA_GAIN
 *  | 23 - 22 : LNA_GAIN1
 *  | 21 - 20 : LNA_GAIN2
 *  | 19 - 16 : PPF_GAIN
 *  | 15 - 14 : RSVD
 *  |      13 : LNA_GAIN_WEN
 *  |      12 : PPF_GAIN_WEN
 *  | 11 -  9 : VT_PKDET1_HG
 *  |  8 -  6 : VT_PKDET1_MG
 *  |  5 -  3 : VT_PKDET1_LG
 *  |       2 : RSVD
 *  |       1 : VT_PKDET2
 *  |       0 : VT_PKDET3
 */
 __STATIC_INLINE void syscon_SetGCR(QN_SYSCON_TypeDef *SYSCON, uint32_t value)
 {
     __wr_reg((uint32_t)&SYSCON->GCR, value);
 }

 __STATIC_INLINE void syscon_SetGCRWithMask(QN_SYSCON_TypeDef *SYSCON, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&SYSCON->GCR, mask, value);
 }

 __STATIC_INLINE uint32_t syscon_GetGCR(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->GCR);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @param value The value to set to IVREF_X32 register
 *
 * @brief  outputs specified value to IVREF_X32 register
 *
 *  | 31 - 28 : BGSEL
 *  | 27 - 26 : VREG15
 *  | 25 - 24 : VREG12_A
 *  |      23 : TR_SWITCH
 *  | 22 - 21 : BM_PKDET3
 *  | 20 - 19 : VREG12_D
 *  |      18 : DVDD12_SW_EN
 *  |      17 : BUCK_BYPASS
 *  |      16 : BUCK_DPD
 *  | 15 - 14 : BUCK_ERR_ISEL
 *  | 13 - 12 : BUCK_VBG
 *  |      11 : X32SMT_EN
 *  |      10 : X32BP_RES
 *  |  9 -  8 : BM_X32BUF
 *  |  7 -  6 : X32INJ
 *  |  5 -  0 : X32ICTRL
 */
 __STATIC_INLINE void syscon_SetIvrefX32(QN_SYSCON_TypeDef *SYSCON, uint32_t value)
 {
     __wr_reg((uint32_t)&SYSCON->IVREF_X32, value);
 }

 __STATIC_INLINE void syscon_SetIvrefX32WithMask(QN_SYSCON_TypeDef *SYSCON, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&SYSCON->IVREF_X32, mask, value);
 }

 __STATIC_INLINE uint32_t syscon_GetIvrefX32(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->IVREF_X32);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @param value The value to set to XTAL_BUCK register
 *
 * @brief  outputs specified value to XTAL_BUCK register
 *
 *  |      31 : XSP_CSEL_B0
 *  | 30 - 29 : XTAL_INJ
 *  | 28 - 23 : XICTRL
 *  | 22 - 17 : XCSEL
 *  |      16 : XSMT_EN
 *  | 15 - 14 : BUCK_VTHL
 *  | 13 - 12 : BUCK_VTHH
 *  | 11 -  9 : BUCK_TMOS
 *  |       8 : BUCK_FC
 *  |       7 : BUCK_AGAIN
 *  |       6 : BUCK_ADRES
 *  |  5 -  4 : BUCK_BM
 *  |  3 -  0 : TST_CPREF
 */
 __STATIC_INLINE void syscon_SetXtalBuck(QN_SYSCON_TypeDef *SYSCON, uint32_t value)
 {
     __wr_reg((uint32_t)&SYSCON->XTAL_BUCK, value);
 }

 __STATIC_INLINE void syscon_SetXtalBuckWithMask(QN_SYSCON_TypeDef *SYSCON, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&SYSCON->XTAL_BUCK, mask, value);
 }

 __STATIC_INLINE uint32_t syscon_GetXtalBuck(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->XTAL_BUCK);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @param value The value to set to LO1 register
 *
 * @brief  outputs specified value to LO1 register
 *
 *  |      31 : XDIV
 *  |      30 : LO_TEST_INT
 *  | 29 - 26 : LO_TST_CP
 *  |      25 : LO_ICPH
 *  | 24 - 23 : LO_BM_FIL
 *  | 22 - 21 : LO_BM_DAC
 *  | 20 - 19 : LO_BM_CML_C
 *  | 18 - 17 : LO_BM_CML_D
 *  | 16 - 15 : LO_BM_BVCO
 *  | 14 - 12 : LO_VCO_AMP
 *  |      11 : PMUX_EN
 *  | 10 -  9 : PA_PHASE
 *  |       8 : LO_DAC_TEST_EN
 *  |  7 -  0 : LO_DAC_TEST
 */
 __STATIC_INLINE void syscon_SetLO1(QN_SYSCON_TypeDef *SYSCON, uint32_t value)
 {
     __wr_reg((uint32_t)&SYSCON->LO1, value);
 }

 __STATIC_INLINE void syscon_SetLO1WithMask(QN_SYSCON_TypeDef *SYSCON, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&SYSCON->LO1, mask, value);
 }

 __STATIC_INLINE uint32_t syscon_GetLO1(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->LO1);
 }


/**
 *
 * @param *SYSCON SYSCON Pointer
 * @param value The value to set to LO2 register
 *
 * @brief  outputs specified value to LO2 register
 *
 *  |      31 : LO_SEL
 *  |      30 : EN_DATA_VLD
 *  | 29 -  8 : LO_FRAC
 *  |       7 : LO_CHANGE
 *  |       6 : LO_REG
 *  |       5 : LO_TXBW
 *  |  4 -  0 : LO_INT
 */
 __STATIC_INLINE void syscon_SetLO2(QN_SYSCON_TypeDef *SYSCON, uint32_t value)
 {
     __wr_reg((uint32_t)&SYSCON->LO2, value);
 }

 __STATIC_INLINE void syscon_SetLO2WithMask(QN_SYSCON_TypeDef *SYSCON, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&SYSCON->LO2, mask, value);
 }

 __STATIC_INLINE uint32_t syscon_GetLO2(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->LO2);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @param value The value to set to RXCR register
 *
 * @brief  outputs specified value to RXCR register
 *
 *  | 31 - 28 : LNA_LOAD_CAP
 *  | 27 - 26 : BM_LNA_GM
 *  | 25 - 24 : BM_LNA
 *  | 23 - 22 : BM_LNA_PK
 *  |      21 : IQSWAP
 *  | 20 - 19 : BM_PPF
 *  |      18 : EN_PDF_DIS
 *  | 17 - 16 : VT_ADC_RST
 *  |      15 : IMR
 *  |      14 : RSTN_DWA
 *  | 13 - 12 : ADC_DAC2I
 *  | 11 - 10 : ADC_DAC3I
 *  |       9 : ADC_RCTEMP
 *  |       8 : ADC_CAP_SEL
 *  |  7 -  5 : ADC_DCON
 *  |  4 -  3 : ADC_OPA12I
 *  |  2 -  1 : ADC_OPA4I
 *  |       0 : STF_PK_EN
 */
 __STATIC_INLINE void syscon_SetRXCR(QN_SYSCON_TypeDef *SYSCON, uint32_t value)
 {
     __wr_reg((uint32_t)&SYSCON->RXCR, value);
 }

 __STATIC_INLINE void syscon_SetRXCRWithMask(QN_SYSCON_TypeDef *SYSCON, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&SYSCON->RXCR, mask, value);
 }

 __STATIC_INLINE uint32_t syscon_GetRXCR(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->RXCR);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @param value The value to set to ADCCR register
 *
 * @brief  outputs specified value to ADCCR register
 *
 *  | 31 - 28 : RSVD
 *  | 27 - 20 : SPEED_UP_TIME
 *  |      19 : RSVD
 *  |      18 : CK_DAC_DLY
 *  |      17 : BYPASS_TESTBUF
 *  |      16 : SEL_TEST_EN
 *  | 15 -  8 : TESTREG
 *  |       6 : ADC_DIG_RST
 *  |       5 : ADC_CLK_SEL
 *  |       4 : ADC_DIV_BYPASS
 *  |  3 -  0 : ADC_DIV
 */
 __STATIC_INLINE void syscon_SetADCCR(QN_SYSCON_TypeDef *SYSCON, uint32_t value)
 {
     __wr_reg((uint32_t)&SYSCON->ADCCR, value);
 }

 __STATIC_INLINE void syscon_SetADCCRWithMask(QN_SYSCON_TypeDef *SYSCON, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&SYSCON->ADCCR, mask, value);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @return ADCCR register value
 *
 * @brief  returns a uint32_t which read from ADCCR register
 */
 __STATIC_INLINE uint32_t syscon_GetADCCR(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->ADCCR);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @param value The value to set to ANALOGCR register
 *
 * @brief  outputs specified value to ANALOGCR register
 *
 *  | 31 - 28 : ACMP0_REF
 *  | 27 - 24 : ACMP1_REF
 *  | 23 - 22 : BD_TH
 *  |      21 : ACMP0_EN
 *  |      20 : ACMP1_EN
 *  |      19 : BT_EN
 *  |      18 : BD_EN
 *  |      17 : TS_EN
 *  |      16 : ACMP1_VALUE
 *  |      15 : ACMP0_VALUE
 *  |      14 : ACMP0_HYST_EN
 *  |      13 : ACMP1_HYST_EN
 *  | 12 -  9 : AINX_EN
 *  |       8 : BUCK_PMDR_B1
 *  |       7 : BUCK_NMDR_B1
 *  |       6 : PA_GAIN_BIT4_B1
 *  |       5 : XSP_CSEL_B1
 *  |  4 -  0 : RSVD
 */
 __STATIC_INLINE void syscon_SetAnalogCR(QN_SYSCON_TypeDef *SYSCON, uint32_t value)
 {
     __wr_reg((uint32_t)&SYSCON->ANALOGCR, value);
 }

 __STATIC_INLINE void syscon_SetAnalogCRWithMask(QN_SYSCON_TypeDef *SYSCON, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&SYSCON->ANALOGCR, mask, value);
 }

 __STATIC_INLINE uint32_t syscon_GetAnalogCR(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->ANALOGCR);
 }

/**
 *
 * @param *SYSCON SYSCON Pointer
 * @param value The value to set to ADDITIONCR register
 *
 * @brief  outputs specified value to ADDITIONCR register
 *
 *  | 31 - 15 : RSVD
 *  | 14 - 12 : BUCK_TMOS_BAK
 *  | 11 - 10 : BUCK_BM_BAK
 *  |       9 : HALF_LO_OPCUR
 *  |       8 : EN_RXDAC
 *  |       7 : TX_PLL_PFD_DIS
 *  |       6 : RX_PLL_PFD_DIS
 *  |       5 : CALI_REDUCE
 *  |       4 : REF_REDUCE_I
 *  |       3 : XADD_C
 *  |       2 : DIS_XPD_DLY
 *  |       1 : PA_CKEN_SEL
 *  |       0 : DC_CAL_MODE
 */
 __STATIC_INLINE void syscon_SetAdditionCR(QN_SYSCON_TypeDef *SYSCON, uint32_t value)
 {
     __wr_reg((uint32_t)&SYSCON->ADDITIONCR, value);
 }

 __STATIC_INLINE void syscon_SetAdditionCRWithMask(QN_SYSCON_TypeDef *SYSCON, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&SYSCON->ADDITIONCR, mask, value);
 }

 __STATIC_INLINE uint32_t syscon_GetAdditionCR(QN_SYSCON_TypeDef *SYSCON)
 {
     return __rd_reg((uint32_t)&SYSCON->ADDITIONCR);
 }


/*GPIO driver functions*/
/**
 *
 * @param *GPIO GPIO Pointer
 * @param outenableset pattern to be used to set output enable register
 * @return none
 *
 * @brief  Sets pins on a port as an output. Set the bit corresponding to the pin number to 1 for output i.e. Set bit 1 of outenable to 1 to set pin 1 as an output.
 */

 __STATIC_INLINE void gpio_gpio_SetOutEnable(QN_GPIO_TypeDef *GPIO, uint32_t outenableset)
 {
       GPIO->OUTENABLESET = outenableset;
 }

/**
 *
 * @param *GPIO GPIO Pointer
 * @param outenableclr Bit pattern to be used to set output enable register
 * @return none
 *
 * @brief  Sets pins on a port as an input. Set the bit corresponding to the pin number to 1 for input i.e. Set bit 1 of outenable to 1 to set pin 1 as an input.
 */

 __STATIC_INLINE void gpio_gpio_ClrOutEnable(QN_GPIO_TypeDef *GPIO, uint32_t outenableclr)
 {
       GPIO->OUTENABLECLR = outenableclr;
 }

/**
 *
 * @param *GPIO GPIO Pointer
 * @return outputstatus
 *
 * @brief  returns a uint32_t which defines the whether pins on a port are set as inputs or outputs i.e. if bit 1 of the returned uint32_t is set to 1 then this means that pin 1 is an output.
 */

 __STATIC_INLINE uint32_t gpio_gpio_GetOutEnable(QN_GPIO_TypeDef *GPIO)
 {
       return GPIO->OUTENABLESET;
 }

/**
 *
 * @param *GPIO GPIO Pointer
 * @param AltFuncset To specify whether the alternate function for the pins on the port is enabled
 * @return none
 *
 * @brief  enables the alternative function for pins. Set the bit corresponding to the pin number to 1 for alternate function i.e. Set bit 1 of ALtFunc to 1 to set pin 1 to its alternative function.
 */

 __STATIC_INLINE void gpio_gpio_SetAltFunc(QN_GPIO_TypeDef *GPIO, uint32_t AltFuncset)
 {
       GPIO->ALTFUNCSET = AltFuncset;
 }

/**
 *
 * @param *GPIO GPIO Pointer
 * @param AltFuncclr To specify whether the alternate function for the pins on the port is enabled
 * @return none
 *
 * @brief  disables the alternative function for pins. Set the bit corresponding to the pin number to 1 to disable alternate function i.e. Set bit 1 of ALtFunc to 1 to set pin 1 to the orignal output function.
 */

 __STATIC_INLINE void gpio_gpio_ClrAltFunc(QN_GPIO_TypeDef *GPIO, uint32_t AltFuncclr)
 {
       GPIO->ALTFUNCCLR = AltFuncclr;
 }

/**
 *
 * @param *GPIO GPIO Pointer
 * @return AltFuncStatus
 *
 * @brief  returns a uint32_t which defines the whether pins on a port are set to their alternative or their original output functionality i.e. if bit 1 of the returned uint32_t is set to 1 then this means that pin 1 is set to its alternative function.
 */

 __STATIC_INLINE uint32_t gpio_gpio_GetAltFunc(QN_GPIO_TypeDef *GPIO)
 {
       return GPIO->ALTFUNCSET;
 }


/**
 *
 * @param *GPIO GPIO Pointer
 * @return NewIntStatus
 *
 * @brief  Clears the interrupt flag for the specified pin and then returns the new interrupt status of the pin.
 */

 __STATIC_INLINE uint32_t gpio_gpio_IntStatus(QN_GPIO_TypeDef *GPIO)
 {
       return GPIO->INTSTATUS;
 }

/**
 *
 * @param *GPIO GPIO Pointer
 * @param Mask The pin mask for which to clear the interrupt
 *
 * @brief  Clears the interrupt flag for the specified pin and then returns the new interrupt status of the pin.
 */

 __STATIC_INLINE void gpio_gpio_IntClear(QN_GPIO_TypeDef *GPIO, uint32_t Mask)
 {
       GPIO->INTCLEAR = Mask;
 }

/**
 *
 * @param *GPIO GPIO Pointer
 * @param Mask The pin mask for which to enable the interrupt
 *
 * @brief  Enables interrupts for the specified pin and then returns the new interrupt enable status of the pin.
 */

 __STATIC_INLINE void gpio_gpio_SetIntEnable(QN_GPIO_TypeDef *GPIO, uint32_t Mask)
 {
       GPIO->INTENSET = Mask;
 }

/**
 *
 * @param *GPIO GPIO Pointer
 * @param Mask The pin mask for which to disable the interrupt
 *
 * @brief  Disables interrupts for the specified pin and then returns the new interrupt enable status of the pin.
 */

 __STATIC_INLINE  void gpio_gpio_ClrIntEnable(QN_GPIO_TypeDef *GPIO, uint32_t Mask)
 {
       GPIO->INTENCLR = Mask;
 }

/**
 *
 * @param *GPIO GPIO Pointer
 * @param Mask The pin mask for which to set the interrupt type
 * @return none
 *
 * @brief  Changes the interrupt type for the specified pin to high level interrupt.
 */

 __STATIC_INLINE void gpio_gpio_SetIntHighLevel(QN_GPIO_TypeDef *GPIO, uint32_t Mask)
 {
       GPIO->INTTYPECLR = Mask; /* Clear INT TYPE bit */
       GPIO->INTPOLSET = Mask;  /* Set INT POLarity bit */
 }

/**
 *
 * @param *GPIO GPIO Pointer
 * @param Mask The pin mask for which to set the interrupt type
 * @return none
 *
 * @brief  Changes the interrupt type for the specified pin to rising edge interrupt.
 */

 __STATIC_INLINE void gpio_gpio_SetIntRisingEdge(QN_GPIO_TypeDef *GPIO, uint32_t Mask)
 {
       GPIO->INTTYPESET = Mask; /* Set INT TYPE bit */
       GPIO->INTPOLSET = Mask;  /* Set INT POLarity bit */
 }

/**
 *
 * @param *GPIO GPIO Pointer
 * @param Mask The pin mask for which to set the interrupt type
 * @return none
 *
 * @brief  Changes the interrupt type for the specified pin to low level interrupt.
 */

 __STATIC_INLINE void gpio_gpio_SetIntLowLevel(QN_GPIO_TypeDef *GPIO, uint32_t Mask)
 {
       GPIO->INTTYPECLR = Mask;  /* Clear INT TYPE bit */
       GPIO->INTPOLCLR = Mask;   /* Clear INT POLarity bit */
 }

/**
 *
 * @param *GPIO GPIO Pointer
 * @param Mask The pin mask for which to set the interrupt type
 * @return none
 *
 * @brief  Changes the interrupt type for the specified pin to falling edge interrupt.
 */

 __STATIC_INLINE void gpio_gpio_SetIntFallingEdge(QN_GPIO_TypeDef *GPIO, uint32_t Mask)
 {
       GPIO->INTTYPESET = Mask;  /* Set INT TYPE bit */
       GPIO->INTPOLCLR = Mask;   /* Clear INT POLarity bit */
 }

/**
 *
 * @param *GPIO GPIO Pointer
 * @param mask The output port mask
 * @param value The value to output to the specified port
 * @return none
 *
 * @brief Outputs the specified value on the desired port using the user defined mask to perform Masked access.
 */

 __STATIC_INLINE void gpio_gpio_MaskedWrite(QN_GPIO_TypeDef *GPIO, uint32_t mask, uint32_t value)
 {
       GPIO->LB_LW_MASKED[0x000000FF & mask] = value;
       GPIO->UB_LW_MASKED[(0x0000FF00 & mask) >> 8] = value;
       GPIO->LB_UW_MASKED[(0x00FF0000 & mask) >> 16] = value;
       GPIO->UB_UW_MASKED[(0xFF000000 & mask) >> 24] = value;
 }

/**
 *
 * @param *GPIO GPIO Pointer
 * @return DATA
 *
 * @brief  returns a uint32_t which read from pin
 */
 __STATIC_INLINE uint32_t gpio_gpio_GetInputData(QN_GPIO_TypeDef *GPIO)
 {
       return GPIO->DATA;
 }

/**
 *
 * @param *GPIO GPIO Pointer
 * @return DATAOUT
 *
 * @brief  returns a uint32_t which read from output register
 */
 __STATIC_INLINE uint32_t gpio_gpio_GetOutputData(QN_GPIO_TypeDef *GPIO)
 {
       return GPIO->DATAOUT;
 }

/**
 *
 * @param *GPIO GPIO Pointer
 * @param data The data to output to the specified port
 *
 * @brief  returns a uint32_t which read from output register
 */
 __STATIC_INLINE void gpio_gpio_SetOutputData(QN_GPIO_TypeDef *GPIO, uint32_t data)
 {
       __wr_reg((uint32_t)&GPIO->DATAOUT, data);
 }


/*SPI driver functions*/
/**
 *
 * @param *SPI SPI Pointer
 * @param value The value to set to CR0 register
 *
 * @brief  outputs specified value to CR0 register
 *
 *  | 31 - 22 : RSVD
 *  | 21 - 16 : CLK_DIV_MASTER
 *  |      15 : MSTR_SS1
 *  |      14 : MSTR_SS0
 *  | 13 - 12 : RSVD
 *  |      11 : SPI_INT_EN
 *  |      10 : RX_FIFO_OVR_IE
 *  |       9 : RX_FIFO_NEMT_IE
 *  |       8 : TX_FIFO_NFUL_IE
 *  |       7 : DIO_MODE
 *  |       6 : BYTE_ENDIAN
 *  |       5 : BUF_WIDTH
 *  |       4 : BIT_ORDER
 *  |   3 - 2 : SPI_MODE
 *  |       1 : CPHA
 *  |       0 : CPOL
 */
 __STATIC_INLINE void spi_spi_SetCR0(QN_SPI_TypeDef *SPI, uint32_t value)
 {
       __wr_reg((uint32_t)&SPI->CR0, value);
 }

 __STATIC_INLINE void spi_spi_SetCR0WithMask(QN_SPI_TypeDef *SPI, uint32_t mask, uint32_t value)
 {
       __wr_reg_with_msk((uint32_t)&SPI->CR0, mask, value);
 }

/**
 *
 * @param *SPI SPI Pointer
 * @return CR0 register value
 *
 * @brief  returns a uint32_t which read from CR0 register
 */
 __STATIC_INLINE uint32_t spi_spi_GetCR0(QN_SPI_TypeDef *SPI)
 {
       return __rd_reg((uint32_t)&SPI->CR0);
 }

/**
 *
 * @param *SPI SPI Pointer
 * @param value The value to set to CR1 register
 *
 * @brief  outputs specified value to CR1 register
 *
 *  | 31 - 18 : RSVD
 *  |      17 : RX_FIFO_CLR
 *  |      16 : TX_FIFO_CLR
 *  | 15 - 10 : RSVD
 *  |       9 : S_SDIO_EN
 *  |       8 : M_SDIO_EN
 *  |   7 - 0 : RSVD
 */
 __STATIC_INLINE void spi_spi_SetCR1(QN_SPI_TypeDef *SPI, uint32_t value)
 {
       __wr_reg((uint32_t)&SPI->CR1, value);
 }

 __STATIC_INLINE void spi_spi_SetCR1WithMask(QN_SPI_TypeDef *SPI, uint32_t mask, uint32_t value)
 {
       __wr_reg_with_msk((uint32_t)&SPI->CR1, mask, value);
 }

/**
 *
 * @param *SPI SPI Pointer
 * @return CR1 register value
 *
 * @brief  returns a uint32_t which read from CR1 register
 */
 __STATIC_INLINE uint32_t spi_spi_GetCR1(QN_SPI_TypeDef *SPI)
 {
       return __rd_reg((uint32_t)&SPI->CR1);
 }

/**
 *
 * @param *SPI SPI Pointer
 * @return FPR register value
 *
 * @brief  returns a uint32_t which read from FIFO pointer register
 *
 *  | 31 - 26 : RSVD
 *  | 25 - 24 : RX_FIFO_RD_PTR
 *  | 23 - 18 : RSVD
 *  | 17 - 16 : RX_FIFO_WR_PTR
 *  | 15 - 10 : RSVD
 *  |   9 - 8 : TX_FIFO_RD_PTR
 *  |   7 - 2 : RSVD
 *  |   1 - 0 : TX_FIFO_WR_PTR
 */
 __STATIC_INLINE uint32_t spi_spi_GetFPR(QN_SPI_TypeDef *SPI)
 {
       return __rd_reg((uint32_t)&SPI->FPR);
 }

/**
 *
 * @param *SPI SPI Pointer
 * @return FSR register value
 *
 * @brief  returns a uint32_t which read from FIFO status register
 *
 *  | 31 - 27 : RSVD
 *  | 26 - 24 : RX_FIFO_LEFT_CNT
 *  | 23 - 19 : RSVD
 *  | 18 - 16 : RX_FIFO_FILL_CNT
 *  | 15 - 11 : RSVD
 *  |  10 - 8 : TX_FIFO_LEFT_CNT
 *  |   7 - 3 : RSVD
 *  |   2 - 0 : TX_FIFO_FILL_CNT
 */
 __STATIC_INLINE uint32_t spi_spi_GetFSR(QN_SPI_TypeDef *SPI)
 {
       return __rd_reg((uint32_t)&SPI->FSR);
 }

/**
 *
 * @param *SPI SPI Pointer
 * @param data The data to write to TXD register
 *
 * @brief  outputs specified value to flash command register
 */
 __STATIC_INLINE void spi_spi_SetTXD(QN_SPI_TypeDef *SPI, uint32_t data)
 {
       __wr_reg((uint32_t)&SPI->TXD, data);
 }

/**
 *
 * @param *SPI SPI Pointer
 * @return RX data
 *
 * @brief  returns a uint32_t which read from RXD register
 */
 __STATIC_INLINE uint32_t spi_spi_GetRXD(QN_SPI_TypeDef *SPI)
 {
       return __rd_reg((uint32_t)&SPI->RXD);
 }


/**
 *
 * @param *SPI SPI Pointer
 * @param value The value to set to SR register
 *
 * @brief  outputs specified value to SR register
 *
 *  | 31 - 25 : RSVD
 *  |      24 : BUSY
 *  | 23 - 17 : RSVD
 *  |      16 : SPI_INT
 *  | 15 -  5 : RSVD
 *  |       4 : RX_FIFO_FULL
 *  |       3 : TX_FIFO_EMPT
 *  |       2 : RX_FIFO_OVR_IF
 *  |       1 : RX_FIFO_NEMT_IF
 *  |       0 : TX_FIFO_NFUL_IF
 */
 __STATIC_INLINE void spi_spi_ClrSR(QN_SPI_TypeDef *SPI, uint32_t value)
 {
       __wr_reg((uint32_t)&SPI->SR, value);
 }

/**
 *
 * @param *SPI SPI Pointer
 * @return SR register value
 *
 * @brief  returns a uint32_t which read from status register
 *
 */
 __STATIC_INLINE uint32_t spi_spi_GetSR(QN_SPI_TypeDef *SPI)
 {
       return __rd_reg((uint32_t)&SPI->SR);
 }


/*UART driver functions*/
/**
 *
 * @param *UART UART Pointer
 * @param txchar Character to be sent
 * @return none
 *
 * @brief  Sends a character to the TX buffer for transmission.
 */
 __STATIC_INLINE void uart_uart_SetTXD(QN_UART_TypeDef *UART, char txchar)
 {
       __wr_reg((uint32_t)&UART->TXD, (uint32_t)txchar);
 }

/**
 *
 * @param *UART UART Pointer
 * @return rxchar
 *
 * @brief  returns the character from the RX buffer which has been received.
 */
 __STATIC_INLINE uint8_t uart_uart_GetRXD(QN_UART_TypeDef *UART)
 {
       return (uint8_t)__rd_reg((uint32_t)&UART->RXD);
 }

/**
 *
 * @param *UART UART Pointer
 * @param divider
 *
 * @brief Set UART Baud rate divider. Note that the Baud rate divider is the difference between the clock frequency and the Baud frequency.
 */
 __STATIC_INLINE void uart_uart_SetBaudDivider(QN_UART_TypeDef *UART, uint32_t divider)
 {
       __wr_reg((uint32_t)&UART->BAUD, divider);
 }

/**
 *
 * @param *UART UART Pointer
 * @return BaudDiv
 *
 * @brief  Returns the current UART Baud rate divider. Note that the Baud rate divider is the difference between the clock frequency and the Baud frequency.
 */
 __STATIC_INLINE uint32_t uart_uart_GetBaudDivider(QN_UART_TypeDef *UART)
 {
       return __rd_reg((uint32_t)&UART->BAUD);
 }

/**
 *
 * @param *UART UART Pointer
 * @param value The value to set to CR register
 *
 * @brief  outputs specified value to CR register
 *
 *  | 31 - 23 : RSVD
 *  |      22 : UART_IE
 *  |      21 : BE_IE
 *  |      20 : PE_IE
 *  |      19 : FE_IE
 *  |      18 : OE_IE
 *  |      17 : TX_IE
 *  |      16 : RX_IE
 *  | 15 - 13 : RSVD
 *  |      11 : OVS
 *  |      10 : CTS_EN
 *  |      9  : RTS_EN
 *  |      8  : BREAK
 *  |      7  : LEVEL_INV
 *  |      6  : STP2_EN
 *  |      5  : BIT_ORDER
 *  |      4  : PEN
 *  |      3  : EPS
 *  |      2  : RX_EN
 *  |      1  : TX_EN
 *  |      0  : UART_EN
 */
 __STATIC_INLINE void uart_uart_SetCR(QN_UART_TypeDef *UART, uint32_t value)
 {
       __wr_reg((uint32_t)&UART->CR, value);
 }

 __STATIC_INLINE void uart_uart_SetCRWithMask(QN_UART_TypeDef *UART, uint32_t mask, uint32_t value)
 {
       __wr_reg_with_msk((uint32_t)&UART->CR, mask, value);
 }

/**
 *
 * @param *UART UART Pointer
 * @return CR register value
 *
 * @brief  returns a uint32_t which read from CR0 register
 */
 __STATIC_INLINE uint32_t uart_uart_GetCR(QN_UART_TypeDef *UART)
 {
       return __rd_reg((uint32_t)&UART->CR);
 }

/**
 *
 * @param *UART UART Pointer
 * @param value The value to set to FLAG register
 *
 * @brief  outputs specified value to FLAG register
 *
 *  | 31 - 9 : RSVD
 *  |      8 : RX_BUSY
 *  |      7 : TX_BUSY
 *  |      6 : UART_IF
 *  |      5 : BE_IF
 *  |      4 : PE_IF
 *  |      3 : FE_IF
 *  |      2 : OE_IF
 *  |      1 : TX_IF
 *  |      0 : RX_IF
 */
 __STATIC_INLINE void uart_uart_ClrIntFlag(QN_UART_TypeDef *UART, uint32_t value)
 {
       __wr_reg((uint32_t)&UART->FLAG, value);
 }

 __STATIC_INLINE void uart_uart_ClrIntFlagWithMask(QN_UART_TypeDef *UART, uint32_t mask, uint32_t value)
 {
       __wr_reg_with_msk((uint32_t)&UART->FLAG, mask, value);
 }

 /**
 *
 * @param *UART UART Pointer
 * @return FLAG status
 *
 * @brief  Returns the interrupt flag.
 */
 __STATIC_INLINE uint32_t uart_uart_GetIntFlag(QN_UART_TypeDef *UART)
 {
       return __rd_reg((uint32_t)&UART->FLAG);
 }


/*I2C driver functions*/
/**
 *
 * @param *I2C I2C Pointer
 * @param value The value to set to CR register
 *
 * @brief  outputs specified value to CR register
 *
 *  | 31 - 30 : RSVD
 *  | 29 - 24 : SCL_RATIO
 *  |      23 : RSVD
 *  | 22 - 16 : SLAVE_ADDR
 *  | 15 - 10 : RSVD
 *  |       9 : SLAVE_EN
 *  |       8 : MASTR_EN
 *  |   7 - 6 : RSVD
 *  |       5 : STP_INT_EN
 *  |       4 : SAM_INT_EN
 *  |       3 : GC_INT_EN
 *  |       2 : AL_INT_EN
 *  |       1 : RX_INT_EN
 *  |       0 : TX_INT_EN
 */
 __STATIC_INLINE void i2c_i2c_SetCR(QN_I2C_TypeDef *I2C, uint32_t value)
 {
       __wr_reg((uint32_t)&I2C->CR, value);
 }

 __STATIC_INLINE void i2c_i2c_SetCRWithMask(QN_I2C_TypeDef *I2C, uint32_t mask, uint32_t value)
 {
       __wr_reg_with_msk((uint32_t)&I2C->CR, mask, value);
 }

/**
 *
 * @param *I2C I2C Pointer
 * @return CR register value
 *
 * @brief  returns a uint32_t which read from CR register
 */
 __STATIC_INLINE uint32_t i2c_i2c_GetCR(QN_I2C_TypeDef *I2C)
 {
       return __rd_reg((uint32_t)&I2C->CR);
 }

/**
 *
 * @param *I2C I2C Pointer
 * @return SR register value
 *
 * @brief  returns a uint32_t which read from status register
 *
 *  |  31 - 3 : RSVD
 *  |       1 : BUSY
 *  |       0 : ACK_RECEIVED
 */
 __STATIC_INLINE uint32_t i2c_i2c_GetSR(QN_I2C_TypeDef *I2C)
 {
       return __rd_reg((uint32_t)&I2C->SR);
 }

/**
 *
 * @param *I2C I2C Pointer
 * @param data The data to write to TXD register
 *
 *  | 31 - 21 : RSVD
 *  |      20 : ACK_SEND
 *  |      19 : RD_EN
 *  |      18 : WR_EN
 *  |      17 : STOP
 *  |      16 : START
 *  |  15 - 8 : RSVD
 *  |   7 - 0 : TXD

 * @brief  outputs specified value to flash command register
 */
 __STATIC_INLINE void i2c_i2c_SetTXD(QN_I2C_TypeDef *I2C, uint32_t data)
 {
       __wr_reg((uint32_t)&I2C->TXD, data);
 }

/**
 *
 * @param *I2C I2C Pointer
 * @return RX data
 *
 * @brief  returns a uint8_t which read from RXD register
 */
 __STATIC_INLINE uint8_t i2c_i2c_GetRXD(QN_I2C_TypeDef *I2C)
 {
       return (uint8_t)__rd_reg((uint32_t)&I2C->RXD);
 }

/**
 *
 * @param *I2C I2C Pointer
 * @param value The value to set to INT register
 *
 * @brief  outputs specified value to INT register
 *
 *  | 31 - 6 : RSVD
 *  |      5 : STP_INT
 *  |      4 : SAM_INT
 *  |      3 : GC_INT
 *  |      2 : AL_INT
 *  |      1 : RX_INT
 *  |      0 : TX_INT
 */
 __STATIC_INLINE void i2c_i2c_ClrIntStatus(QN_I2C_TypeDef *I2C, uint32_t value)
 {
       __wr_reg((uint32_t)&I2C->INT, value);
 }

 /**
 *
 * @param *I2C I2C Pointer
 * @return IntStatus
 *
 * @brief  Returns the interrupt status.
 */
 __STATIC_INLINE uint32_t i2c_i2c_GetIntStatus(QN_I2C_TypeDef *I2C)
 {
       return __rd_reg((uint32_t)&I2C->INT);
 }


/*TIMER driver functions*/
/**
 *
 * @param *TIMER TIMER Pointer
 * @param value The value to set to CR register
 *
 * @brief  outputs specified value to CR register
 *
 *  | 31 - 30 : RSVD
 *  | 29 - 28 : CSS
 *  | 27 - 26 : RSVD
 *  | 25 - 16 : PSCL
 *  |      15 : PWM_OE
 *  |      14 : POL
 *  |      13 : ICNCE
 *  |      12 : ICSS
 *  | 11 - 10 : ICPS
 *  |  9 -  8 : ICES
 *  |       7 : CMP_EN
 *  |       6 : CHAIN_EN
 *  |  5 -  4 : OMS
 *  |       3 : ICIE
 *  |       2 : OCIE
 *  |       1 : TOVIE
 *  |       0 : TEN
 */
 __STATIC_INLINE void timer_timer_SetCR(QN_TIMER_TypeDef *TIMER, uint32_t value)
 {
       __wr_reg((uint32_t)&TIMER->CR, value);
 }

 __STATIC_INLINE void timer_timer_SetCRWithMask(QN_TIMER_TypeDef *TIMER, uint32_t mask, uint32_t value)
 {
       __wr_reg_with_msk((uint32_t)&TIMER->CR, mask, value);
 }

/**
 *
 * @param *TIMER TIMER Pointer
 * @return CR register value
 *
 * @brief  returns a uint32_t which read from CR register
 */
 __STATIC_INLINE uint32_t timer_timer_GetCR(QN_TIMER_TypeDef *TIMER)
 {
       return __rd_reg((uint32_t)&TIMER->CR);
 }

/**
 *
 * @param *TIMER TIMER Pointer
 * @param value The value to set to IFR register
 *
 * @brief  outputs specified value to IFR register
 *
 *  | 31 - 3 : RSVD
 *  |      2 : ICF
 *  |      1 : OCF
 *  |      0 : TOVF
 */
 __STATIC_INLINE void timer_timer_ClrIntFlag(QN_TIMER_TypeDef *TIMER, uint32_t value)
 {
       __wr_reg((uint32_t)&TIMER->IFR,value);
 }

 /**
 *
 * @param *TIMER TIMER Pointer
 * @return IFR register value
 *
 * @brief  Returns the interrupt flag.
 */
 __STATIC_INLINE uint32_t timer_timer_GetIntFlag(QN_TIMER_TypeDef *TIMER)
 {
       return __rd_reg((uint32_t)&TIMER->IFR);
 }

/**
 *
 * @param *TIMER TIMER Pointer
 * @param value The value to set to TOPR register
 *
 * @brief  outputs specified value to TOPR register
 *
 *  | 31 - 0 : TOPR
 */
 __STATIC_INLINE void timer_timer_SetTOPR(QN_TIMER_TypeDef *TIMER, uint32_t value)
 {
       __wr_reg((uint32_t)&TIMER->TOPR,value);
 }

/**
 *
 * @param *TIMER TIMER Pointer
 * @return TOPR register value
 *
 * @brief  returns a uint32_t which read from TOPR register
 */
 __STATIC_INLINE uint32_t timer_timer_GetTOPR(QN_TIMER_TypeDef *TIMER)
 {
       return __rd_reg((uint32_t)&TIMER->TOPR);
 }

/**
 *
 * @param *TIMER TIMER Pointer
 * @return ICER register value
 *
 * @brief  returns a uint32_t which read from ICER register
 */
 __STATIC_INLINE uint32_t timer_timer_GetICER(QN_TIMER_TypeDef *TIMER)
 {
      return __rd_reg((uint32_t)&TIMER->ICER);
 }

/**
 *
 * @param *TIMER TIMER Pointer
 * @param value The value to set to CCR register
 *
 * @brief  outputs specified value to CCR register
 *
 *  | 31 - 0 : CCR
 */
 __STATIC_INLINE void timer_timer_SetCCR(QN_TIMER_TypeDef *TIMER, uint32_t value)
 {
      __wr_reg((uint32_t)&TIMER->CCR,value);
 }

/**
 *
 * @param *TIMER TIMER Pointer
 * @return CCR register value
 *
 * @brief  returns a uint32_t which read from CCR register
 */
 __STATIC_INLINE uint32_t timer_timer_GetCCR(QN_TIMER_TypeDef *TIMER)
 {
      return __rd_reg((uint32_t)&TIMER->CCR);
 }

/**
 *
 * @param *TIMER TIMER Pointer
 * @return CNT register value
 *
 * @brief  returns a uint32_t which read from CNT register
 *
 *  | 31 - 0 : CNT
 */
 __STATIC_INLINE uint32_t timer_timer_GetCNT(QN_TIMER_TypeDef *TIMER)
 {
      return __rd_reg((uint32_t)&TIMER->CNT);
 }

/*PWM driver functions*/
/**
 *
 * @param *PWM PWM Pointer
 * @param value The value to set to CR register
 *
 * @brief  outputs specified value to CR register
 *
 *  | 31 - 11 : RSVD
 *  |      10 : CH1_POL
 *  |       9 : CH1_IE
 *  |       8 : CH1_EN
 *  |  7 -  3 : RSVD
 *  |       2 : CH0_POL
 *  |       1 : CH0_IE
 *  |       0 : CH0_EN
 */
 __STATIC_INLINE void pwm_pwm_SetCR(QN_PWM_TypeDef *PWM, uint32_t value)
 {
       __wr_reg((uint32_t)&PWM->CR,value);
 }

 __STATIC_INLINE void pwm_pwm_SetCRWithMask(QN_PWM_TypeDef *PWM, uint32_t mask, uint32_t value)
 {
       __wr_reg_with_msk((uint32_t)&PWM->CR,mask,value);
 }

/**
 *
 * @param *PWM PWM Pointer
 * @return CR register value
 *
 * @brief  returns a uint32_t which read from CR register
 */
 __STATIC_INLINE uint32_t pwm_pwm_GetCR(QN_PWM_TypeDef *PWM)
 {
       return __rd_reg((uint32_t)&PWM->CR);
 }

/**
 *
 * @param *PWM PWM Pointer
 * @param value The value to set to PSCL register
 *
 * @brief  outputs specified value to PSCL register
 *
 *  | 31 - 26 : RSVD
 *  | 25 - 16 : CH1_PSCL
 *  | 15 - 10 : RSVD
 *  |  9 -  0 : CH0_PSCL
 */
 __STATIC_INLINE void pwm_pwm_SetPSCL(QN_PWM_TypeDef *PWM, uint32_t value)
 {
       __wr_reg((uint32_t)&PWM->PSCL,value);
 }

 __STATIC_INLINE void pwm_pwm_SetPSCLWithMask(QN_PWM_TypeDef *PWM, uint32_t mask, uint32_t value)
 {
       __wr_reg_with_msk((uint32_t)&PWM->PSCL,mask,value);
 }

/**
 *
 * @param *PWM PWM Pointer
 * @return PSCL register value
 *
 * @brief  returns a uint32_t which read from PSCL register
 */
 __STATIC_INLINE uint32_t pwm_pwm_GetPSCL(QN_PWM_TypeDef *PWM)
 {
       return __rd_reg((uint32_t)&PWM->PSCL);
 }

/**
 *
 * @param *PWM PWM Pointer
 * @param value The value to set to PCP register
 *
 * @brief  outputs specified value to PCP register
 *
 *  | 31 - 24 : CH1_CMP
 *  | 23 - 16 : CH1_PERIOD
 *  | 15 -  8 : CH0_CMP
 *  |  7 -  0 : CH0_PERIOD
 */
 __STATIC_INLINE void pwm_pwm_SetPCP(QN_PWM_TypeDef *PWM, uint32_t value)
 {
       __wr_reg((uint32_t)&PWM->PCP,value);
 }

 __STATIC_INLINE void pwm_pwm_SetPCPWithMask(QN_PWM_TypeDef *PWM, uint32_t mask, uint32_t value)
 {
       __wr_reg_with_msk((uint32_t)&PWM->PCP,mask,value);
 }

/**
 *
 * @param *PWM PWM Pointer
 * @return PCP register value
 *
 * @brief  returns a uint32_t which read from PCP register
 */
 __STATIC_INLINE uint32_t pwm_pwm_GetPCP(QN_PWM_TypeDef *PWM)
 {
       return __rd_reg((uint32_t)&PWM->PCP);
 }

/**
 *
 * @param *PWM PWM Pointer
 * @param value The value to set to status register
 *
 * @brief  outputs specified value to status register
 *
 *  |  31 - 9 : RSVD
 *  |       8 : CH1_IF
 *  |   7 - 1 : RSVD
 *  |       0 : CH0_IF
 */
 __STATIC_INLINE void pwm_pwm_ClrIntStatus(QN_PWM_TypeDef *PWM, uint32_t value)
 {
       __wr_reg((uint32_t)&PWM->SR,value);
 }

/**
 *
 * @param *PWM PWM Pointer
 * @return SR register value
 *
 * @brief  returns a uint32_t which read from status register
 *
 */
 __STATIC_INLINE uint32_t pwm_pwm_GetIntStatus(QN_PWM_TypeDef *PWM)
 {
       return __rd_reg((uint32_t)&PWM->SR);
 }

/*RTC driver functions*/
/**
 *
 * @param *RTC RTC Pointer
 * @param value The value to set to CR register
 *
 * @brief  outputs specified value to CR register
 *
 *  | 31 -  9 : RSVD
 *  |       8 : CAL_EN
 *  |       7 : RSVD
 *  |       6 : CAP_EDGE_SEL
 *  |       5 : CAP_IE
 *  |       4 : CAP_EN
 *  |       3 : RSVD
 *  |       2 : CFG
 *  |       1 : CORR_EN
 *  |       0 : SEC_IE
 */
 __STATIC_INLINE void rtc_rtc_SetCR(QN_RTC_TypeDef *RTC, uint32_t value)
 {
       __wr_reg((uint32_t)&RTC->CR,value);
 }

 __STATIC_INLINE void rtc_rtc_SetCRWithMask(QN_RTC_TypeDef *RTC, uint32_t mask, uint32_t value)
 {
       __wr_reg_with_msk((uint32_t)&RTC->CR,mask,value);
 }

/**
 *
 * @param *RTC RTC Pointer
 * @return CR register value
 *
 * @brief  returns a uint32_t which read from CR register
 */
 __STATIC_INLINE uint32_t rtc_rtc_GetCR(QN_RTC_TypeDef *RTC)
 {
       return __rd_reg((uint32_t)&RTC->CR);
 }

/**
 *
 * @param *RTC RTC Pointer
 * @param value The value to set to SR register
 *
 * @brief  outputs specified value to SR register
 *
 *  | 31 - 13 : RSVD
 *  |      12 : CALIB_SYNC_BUSY
 *  |      11 : CORR_SYNC_BUSY
 *  |      10 : SEC_SYNC_BUSY
 *  |       9 : SR_SYNC_BUSY
 *  |       8 : CR_SYNC_BUSY
 *  |  7 -  5 : RSVD
 *  |       4 : CAP_IF
 *  |  3 -  1 : RSVD
 *  |       0 : SEC_IF
 */
 __STATIC_INLINE void rtc_rtc_ClrSR(QN_RTC_TypeDef *RTC, uint32_t value)
 {
       __wr_reg((uint32_t)&RTC->SR,value);
 }

/**
 *
 * @param *RTC RTC Pointer
 * @return SR register value
 *
 * @brief  returns a uint32_t which read from SR register
 *
 */
 __STATIC_INLINE uint32_t rtc_rtc_GetSR(QN_RTC_TypeDef *RTC)
 {
       return __rd_reg((uint32_t)&RTC->SR);
 }


/**
 *
 * @param *RTC RTC Pointer
 * @param value The value to set to RTC_SEC register
 *
 * @brief  outputs specified value to RTC_SEC register
 *
 *  | 31 -  0 : SEC
 */
 __STATIC_INLINE void rtc_rtc_SetSecVal(QN_RTC_TypeDef *RTC, uint32_t value)
 {
       __wr_reg((uint32_t)&RTC->SEC,value);
 }

 __STATIC_INLINE uint32_t rtc_rtc_GetSecVal(QN_RTC_TypeDef *RTC)
 {
       return __rd_reg((uint32_t)&RTC->SEC);
 }
/**
 *
 * @param *RTC RTC Pointer
 * @param value The value to set to RTC_CORR register
 *
 * @brief  outputs specified value to RTC_CORR register
 *
 *  | 31 - 15 : SEC_CORR
 *  | 14 -  0 : CNT_CORR

 */
 __STATIC_INLINE void rtc_rtc_SetCORR(QN_RTC_TypeDef *RTC, uint32_t value)
 {
       __wr_reg((uint32_t)&RTC->CORR,value);
 }

/**
 *
 * @param *RTC RTC Pointer
 * @param value The value to set to RTC_CALIB register
 *
 * @brief  outputs specified value to RTC_CALIB register
 *
 *  | 31 - 17 : RSVD
 *  |      16 : CAL_DIR
 *  | 15 -  0 : PPM_VAL

 */
 __STATIC_INLINE void rtc_rtc_SetCalVal(QN_RTC_TypeDef *RTC, uint32_t value)
 {
       __wr_reg((uint32_t)&RTC->CALIB,value);
 }

/**
 *
 * @param *RTC RTC Pointer
 * @return CNT register value
 *
 * @brief  returns a uint32_t which read from CNT register
 *
 *  | 31 - 15 : RSVD
 *  | 14 -  0 : CNT_VAL
 */
 __STATIC_INLINE uint32_t rtc_rtc_GetCNT(QN_RTC_TypeDef *RTC)
 {
       return __rd_reg((uint32_t)&RTC->CNT);
 }

/*WDT driver functions*/
/**
 *
 * @param *WDT WDT Pointer
 * @param value The value to set to CTRL register
 *
 * @brief  outputs specified value to CTRL register
 *
 *  | 31 -  2 : RSVD
 *  |       1 : RESET_EN
 *  |       0 : INT_EN
 */
 __STATIC_INLINE void wdt_wdt_SetCR(QN_WDT_TypeDef *WDT, uint32_t value)
 {
     __wr_reg((uint32_t)&WDT->CTRL, value);
 }

 __STATIC_INLINE void wdt_wdt_SetCRWithMask(QN_WDT_TypeDef *WDT, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&WDT->CTRL,mask,value);
 }

 /**
 *
 * @param *WDT WDT Pointer
 * @return CTRL register value
 *
 * @brief  returns a uint32_t which read from CTRL register
 *
 *  | 31 -  0 : VAL
 *  |       1 : RESET_EN
 *  |       0 : INT_EN
 */
 __STATIC_INLINE uint32_t wdt_wdt_GetCR(QN_WDT_TypeDef *WDT)
 {
     return __rd_reg((uint32_t)&WDT->CTRL);
 }

 /**
 *
 * @param *WDT WDT Pointer
 * @param value The value to set to LOAD register
 *
 * @brief  outputs specified value to LOAD register
 *
 *  | 31 -  0 : VAL
 */
 __STATIC_INLINE void wdt_wdt_SetLDR(QN_WDT_TypeDef *WDT, uint32_t value)
 {
     __wr_reg((uint32_t)&WDT->LOAD, value);
 }

 /**
 *
 * @param *WDT WDT Pointer
 * @return LOAD register value
 *
 * @brief  returns a uint32_t which read from LOAD register
 *
 *  | 31 -  0 : VAL
 */
 __STATIC_INLINE uint32_t wdt_wdt_GetLDR(QN_WDT_TypeDef *WDT)
 {
     return __rd_reg((uint32_t)&WDT->LOAD);
 }

 /**
 *
 * @param *WDT WDT Pointer
 * @param value The value to set to VALUE register
 *
 * @brief  outputs specified value to VALUE register
 *
 *  | 31 -  0 : VAL
 */
 __STATIC_INLINE void wdt_wdt_SetVALR(QN_WDT_TypeDef *WDT, uint32_t value)
 {
     __wr_reg((uint32_t)&WDT->VALUE, value);
 }

 /**
 *
 * @param *WDT WDT Pointer
 * @return VALUE register value
 *
 * @brief  returns a uint32_t which read from VALUE register
 *
 *  | 31 -  0 : VAL
 */
 __STATIC_INLINE uint32_t wdt_wdt_GetVALR(QN_WDT_TypeDef *WDT)
 {
     return __rd_reg((uint32_t)&WDT->VALUE);
 }

 /**
 *
 * @param *WDT WDT Pointer
 * @param value The value to set to LOCK register
 *
 * @brief  outputs specified value to LOCK register
 *
 *  | 31 -  0 : VAL
 */
  __STATIC_INLINE void wdt_wdt_SetLKR(QN_WDT_TypeDef *WDT, uint32_t value)
 {
     __wr_reg((uint32_t)&WDT->LOCK, value);
 }

 /**
 *
 * @param *WDT WDT Pointer
 * @return LOCK register value
 *
 * @brief  returns a uint32_t which read from LOCK register
 *
 *  | 31 -  0 : VAL
 */
 __STATIC_INLINE uint32_t wdt_wdt_GetLKR(QN_WDT_TypeDef *WDT)
 {
     return __rd_reg((uint32_t)&WDT->LOCK);
 }

 /**
 *
 * @param *WDT WDT Pointer
 * @param value The value to set to INTCLR register
 *
 * @brief  outputs specified value to INTCLR register
 *
 *  | 31 -  1 : RSVD
 *  |       0 : INT_CLR
 */
 __STATIC_INLINE void wdt_wdt_ClrIntStatus(QN_WDT_TypeDef *WDT, uint32_t value)
 {
     __wr_reg((uint32_t)&WDT->INTCLR, value);
 }

  /**
 *
 * @param *WDT WDT Pointer
 * @return INTCLR register value
 *
 * @brief  returns a uint32_t which read from INTCLR register
 *
 *  | 31 -  1 : RSVD
 *  |       0 : INT_CLR
 */
 __STATIC_INLINE uint32_t wdt_wdt_GetIntStatus(QN_WDT_TypeDef *WDT)
 {
     return __rd_reg((uint32_t)&WDT->INTCLR);
 }


/*DMA driver functions*/
/**
 *
 * @param *DMA DMA Pointer
 * @param value The value to set to SRC register
 *
 * @brief  outputs specified value to SRC register
 *
 *  | 31 - 0 : SRC
 */
 __STATIC_INLINE void dma_dma_SetSRC(QN_DMA_TypeDef *DMA, uint32_t value)
 {
       __wr_reg((uint32_t)&DMA->SRC, value);
 }

 /**
 *
 * @param *DMA DMA Pointer
 * @param value The value to set to DST register
 *
 * @brief  outputs specified value to DST register
 *
 *  | 31 - 0 : DST
 */
 __STATIC_INLINE void dma_dma_SetDST(QN_DMA_TypeDef *DMA, uint32_t value)
 {
       __wr_reg((uint32_t)&DMA->DST, value);
 }

/**
 *
 * @param *DMA DMA Pointer
 * @param value The value to set to CR register
 *
 * @brief  outputs specified value to CR register
 *
 *  | 31      : DONE_IE
 *  | 30      : ERROR_IE
 *  | 29      : INT_EN
 *  | 25 - 16 : TRANS_SIZE
 *  | 15 - 12 : DSR_MUX
 *  | 11 - 8  : SRC_MUX
 *  | 7 - 6   : TRANS_MODE
 *  | 5       : DST_REQ_EN
 *  | 4       : SRC_REQ_EN
 *  | 3       : DST_ADDR_FIX
 *  | 2       : SRC_ADDR_FIX
 *  | 1       : SRC_UDLEN
 *  | 0       : START
 */
 __STATIC_INLINE void dma_dma_SetCR(QN_DMA_TypeDef *DMA, uint32_t value)
 {
       __wr_reg((uint32_t)&DMA->CR, value);
 }

 __STATIC_INLINE void dma_dma_SetCRWithMask(QN_DMA_TypeDef *DMA, uint32_t mask, uint32_t value)
 {
       __wr_reg_with_msk((uint32_t)&DMA->CR, mask, value);
 }

/**
 *
 * @param *DMA DMA Pointer
 * @return CR register value
 *
 * @brief  returns a uint32_t which read from CR register
 */
 __STATIC_INLINE uint32_t dma_dma_GetCR(QN_DMA_TypeDef *DMA)
 {
       return __rd_reg((uint32_t)&DMA->CR);
 }

/**
 *
 * @param *DMA DMA Pointer
 *
 * @brief  ABORT is a virtual register which only occupy an APB address
 *         writing 32'h1 to ABORT address to send an abort signal
 *
 *  | 31 - 1 : RSVD
 *  |      0 : ABORT
 */
 __STATIC_INLINE void dma_dma_SetAbort(QN_DMA_TypeDef *DMA)
 {
       __wr_reg((uint32_t)&DMA->ABORT, (uint32_t)0x01);
 }

/**
 *
 * @param *DMA DMA Pointer
 * @param value The value to set to status register
 *
 * @brief  outputs specified value to status register
 *
 *  | 31 - 3 : RSVD
 *  |      2 : BUSY
 *  |      1 : ERROR
 *  |      0 : DONE
 */
 __STATIC_INLINE void dma_dma_ClrIntStatus(QN_DMA_TypeDef *DMA, uint32_t value)
 {
       __wr_reg((uint32_t)&DMA->SR, value);
 }

/**
 *
 * @param *DMA DMA Pointer
 * @return SR register value
 *
 * @brief  returns a uint32_t which read from status register
 *
 *  | 31 - 1 : RSVD
 *  |      2 : BUSY
 *  |      1 : ERROR
 *  |      0 : DONE
 */
 __STATIC_INLINE uint32_t dma_dma_GetIntStatus(QN_DMA_TypeDef *DMA)
 {
       return __rd_reg((uint32_t)&DMA->SR);
 }


/*SPI FLASH CONTROLLER driver functions*/
/**
 *
 * @param *SF_CTRL SF_CTRL Pointer
 * @param value The value to set to CTRL_STAT register
 *
 * @brief  outputs specified value to CTRL_STAT register
 *
 *  |      31 : BOOT_ERR
 *  |      30 : BOOT_DONE
 *  | 29 - 16 : RSVD
 *  | 15 -  5 : RD_STAT_CMD
 *  |  7 -  6 : RSVD
 *  |  5 -  0 : CLK_DIV
 */
 __STATIC_INLINE void sf_ctrl_SetCR(QN_SF_CTRL_TypeDef *SF_CTRL, uint32_t value)
 {
     __wr_reg((uint32_t)&SF_CTRL->CTRL_STAT, value);
 }

 __STATIC_INLINE void sf_ctrl_SetCRWithMask(QN_SF_CTRL_TypeDef *SF_CTRL, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&SF_CTRL->CTRL_STAT, mask, value);
 }

 __STATIC_INLINE uint32_t sf_ctrl_GetSR(QN_SF_CTRL_TypeDef *SF_CTRL)
 {
     return __rd_reg((uint32_t)&SF_CTRL->CTRL_STAT);
 }

/**
 *
 * @param *SF_CTRL SF_CTRL Pointer
 * @param value The value to set to DATA_LEN register
 *
 * @brief  outputs specified value to DATA_LEN register
 *
 *  |  31 - 17 : RSVD
 *  |  16 -  0 : DATA_LEN
 */
 __STATIC_INLINE void sf_ctrl_SetDataLen(QN_SF_CTRL_TypeDef *SF_CTRL, uint32_t value)
 {
     __wr_reg((uint32_t)&SF_CTRL->DATA_LEN, value);
 }

 __STATIC_INLINE uint32_t sf_ctrl_GetDataLen(QN_SF_CTRL_TypeDef *SF_CTRL)
 {
     return __rd_reg((uint32_t)&SF_CTRL->DATA_LEN);
 }


/**
 *
 * @param *SF_CTRL SF_CTRL Pointer
 * @param value The value to set to CMD register
 *
 * @brief  outputs specified value to CMD register
 *
 *  |  31 - 8 : RSVD
 *  |   7 - 0 : CMD
 */
 __STATIC_INLINE void sf_ctrl_SetCMD(QN_SF_CTRL_TypeDef *SF_CTRL, uint32_t value)
 {
     __wr_reg((uint32_t)&SF_CTRL->CMD1, value);
 }

/**
 *
 * @param *SF_CTRL SF_CTRL Pointer
 * @param value The value to set to CMD1ADDR3 register
 *
 * @brief  outputs specified value to CMD1ADDR3 register
 *
 *  | 31 - 24 : ADDR2
 *  | 23 - 16 : ADDR1
 *  |  15 - 8 : ADDR0
 *  |   7 - 0 : CMD
 */
 __STATIC_INLINE void sf_ctrl_SetCmdAddr(QN_SF_CTRL_TypeDef *SF_CTRL, uint32_t value)
 {
     __wr_reg((uint32_t)&SF_CTRL->CMD1ADDR3, value);
 }

 /**
 *
 * @param *SF_CTRL SF_CTRL Pointer
 * @return FLASH_SR register value
 *
 * @brief  returns a uint32_t which read from FLASH_SR register
 *
 *  | 31 - 16 : RSVD
 *  |   7 - 0 : STAT
 */
 __STATIC_INLINE uint32_t sf_ctrl_GetFlashSR(QN_SF_CTRL_TypeDef *SF_CTRL)
 {
     return __rd_reg((uint32_t)&SF_CTRL->FLASH_SR);
 }

 /**
 *
 * @param *SF_CTRL SF_CTRL Pointer
 * @return FLASH_ID register value
 *
 * @brief  returns a uint32_t which read from FLASH_ID register
 *
 *  | 31 - 0 : FLASH_ID
 */
 __STATIC_INLINE uint32_t sf_ctrl_GetFlashID(QN_SF_CTRL_TypeDef *SF_CTRL)
 {
     return __rd_reg((uint32_t)&SF_CTRL->FLASH_ID);
 }

/*DP driver functions*/
/**
 *
 * @param offset  register address offset: 0x00, 0x04, 0x08...
 * @param value  The value to set to REG_XX register
 *
 * @brief  outputs specified value to REG_XX register
 *
 */
 __STATIC_INLINE void dp_dp_SetReg(uint32_t offset, uint32_t value)
 {
     __wr_reg(QN_DP_BASE + offset, value);
 }

 __STATIC_INLINE void dp_dp_SetRegWithMask(uint32_t offset, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk(QN_DP_BASE + offset, mask, value);
 }

 __STATIC_INLINE uint32_t dp_dp_GetReg(uint32_t offset)
 {
     return __rd_reg(QN_DP_BASE + offset);
 }


/*ADC driver functions*/
/**
 *
 * @param *ADC ADC Pointer
 * @param value The value to set to ADC0 register
 *
 * @brief  outputs specified value to ADC0 register
 *
 *  | 31 - 28 : SCAN_CH_END
 *  | 27 - 24 : SCAN_CH_START
 *  | 23 - 19 : RSVD
 *  | 18 - 17 : SCAN_INTV
 *  |      16 : SCAN_EN
 *  |      15 : SINGLE_EN
 *  | 14 - 12 : START_SEL
 *  | 11 - 10 : RSVD
 *  |       9 : ADC_EN
 *  |       8 : SFT_START
 *  |  7 -  2 : POW_UP_DLY
 *  |       1 : POW_DN_CTRL
 *  |       0 : RSVD
 */
 __STATIC_INLINE void adc_adc_SetADC0(QN_ADC_TypeDef *ADC, uint32_t value)
 {
     __wr_reg((uint32_t)&ADC->ADC0, value);
 }

 __STATIC_INLINE void adc_adc_SetADC0WithMask(QN_ADC_TypeDef *ADC, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&ADC->ADC0, mask, value);
 }

 __STATIC_INLINE uint32_t adc_adc_GetADC0(QN_ADC_TypeDef *ADC)
 {
       return ADC->ADC0;
 }


/**
 *
 * @param *ADC ADC Pointer
 * @param value The value to set to ADC1 register
 *
 * @brief  outputs specified value to ADC1 register
 *
 *  |      31 : INT_MASK
 *  |      30 : DAT_RDY_IE
 *  |      29 : WCMP_IE
 *  |      28 : FIFO_OF_IE
 *  |      27 : TIF_EN
 *  | 26 - 24 : TIF_SEL
 *  | 23 - 22 : RSVD
 *  | 21 - 20 : VREF_SEL
 *  |      19 : INBUF_BP
 *  |      18 : BUF_GAIN_BP
 *  | 17 - 16 : BUF_GAIN
 *  | 15 - 14 : BUF_BM_DRV
 *  | 13 - 12 : BUF_BM_GAIN
 *  | 11 - 10 : BUF_IN_P
 *  |  9 -  8 : BUF_IN_N
 *  |  7 -  6 : SAR_RES_SEL
 *  |       5 : WCMP_SEL
 *  |       4 : WCMP_EN
 *  |       3 : RSVD
 *  |  2 -  1 : DECI_DIV
 *  |       0 : DECI_EN
 */
 __STATIC_INLINE void adc_adc_SetADC1(QN_ADC_TypeDef *ADC, uint32_t value)
 {
     __wr_reg((uint32_t)&ADC->ADC1, value);
 }

 __STATIC_INLINE void adc_adc_SetADC1WithMask(QN_ADC_TypeDef *ADC, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&ADC->ADC1, mask, value);
 }

 __STATIC_INLINE uint32_t adc_adc_GetADC1(QN_ADC_TypeDef *ADC)
 {
       return ADC->ADC1;
 }


/**
 *
 * @param *ADC ADC Pointer
 * @param value The value to set to ADC2 register
 *
 * @brief  outputs specified value to ADC2 register
 *
 *  | 31 - 16 : WCMP_TH_HI
 *  | 15 -  0 : WCMP_TH_LO
 */
 __STATIC_INLINE void adc_adc_SetADC2(QN_ADC_TypeDef *ADC, uint32_t value)
 {
     __wr_reg((uint32_t)&ADC->ADC2, value);
 }

 __STATIC_INLINE void adc_adc_SetADC2WithMask(QN_ADC_TypeDef *ADC, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&ADC->ADC2, mask, value);
 }

 __STATIC_INLINE uint32_t adc_adc_GetADC2(QN_ADC_TypeDef *ADC)
 {
       return ADC->ADC2;
 }


/**
 *
 * @param *ADC ADC Pointer
 * @param value The value to set to SR register
 *
 * @brief  outputs specified value to SR register
 *
 *  | 31 -  3 : RSVD
 *  |       2 : FIFO_OF_IF
 *  |       1 : WCMP_IF
 *  |       0 : DAT_RDY_IF
 */
 __STATIC_INLINE void adc_adc_ClrSR(QN_ADC_TypeDef *ADC, uint32_t value)
 {
       ADC->SR = value;
 }

 __STATIC_INLINE void adc_adc_ClrSRWithMask(QN_ADC_TypeDef *ADC, uint32_t mask, uint32_t value)
 {
       uint32_t reg;

       reg = ADC->SR;
       reg &= ~mask;
       reg |= (mask&value);
       ADC->SR = reg;
 }

 __STATIC_INLINE uint32_t adc_adc_GetSR(QN_ADC_TypeDef *ADC)
 {
       return ADC->SR;
 }

/**
 *
 * @param *ADC ADC Pointer
 * @return DATA register value
 *
 * @brief  returns a uint32_t which read from DATA register
 */
 __STATIC_INLINE uint32_t adc_adc_GetDATA(QN_ADC_TypeDef *ADC)
 {
     return __rd_reg((uint32_t)&ADC->DATA);
 }

/*Calibration driver functions*/
/**
 *
 * @param *CALIB CALIB Pointer
 * @param value The value to set to CAL0 register
 *
 * @brief  outputs specified value to CAL0 register
 *
 *  |      31 : SEQ_CAL_REQ
 *  |      30 : RSVD
 *  |      29 : CH_CHG_CAL_EN
 *  |      28 : CH_CHG_CAL_REQ
 *  | 27 - 26 : RSVD
 *  |      25 : LO_CAL_SKIP
 *  |      24 : LO_KCAL_SKIP
 *  |      23 : CAL_DONE_DIS
 *  | 22 - 16 : RSVD
 *  |      15 : REF_CAL_REQ
 *  |      14 : REF_CAL_DIS
 *  | 13 - 12 : RSVD
 *  | 11 -  8 : REF_CAL
 *  |       7 : RC_CAL_REQ
 *  |       6 : RC_CAL_DIS
 *  |  5 -  4 : RC_CAL_DLY
 *  |  3 -  0 : RC_CAL
 */
 __STATIC_INLINE void cal_cal_SetCAL0(QN_CALIB_TypeDef *CALIB, uint32_t value)
 {
     __wr_reg((uint32_t)&CALIB->CAL0, value);
 }

 __STATIC_INLINE void cal_cal_SetCAL0WithMask(QN_CALIB_TypeDef *CALIB, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&CALIB->CAL0, mask, value);
 }

 __STATIC_INLINE uint32_t cal_cal_GetCAL0(QN_CALIB_TypeDef *CALIB)
 {
     return __rd_reg((uint32_t)&CALIB->CAL0);
 }


/**
 *
 * @param *CALIB CALIB Pointer
 * @param value The value to set to CAL1 register
 *
 * @brief  outputs specified value to CAL1 register
 *
 *  |      31 : LO_CAL_REQ
 *  |      30 : LO_ACAL_DIS
 *  |      29 : LO_ACAL_E
 *  | 28 - 24 : LO_ACAL
 *  |      23 : LO_SPEED_UP
 *  |      22 : LO_FCAL_DIS
 *  |      21 : RSVD
 *  | 20 - 16 : LO_FCAL
 *  |      15 : LO_KCAL_REQ
 *  |      14 : LO_KCAL_DIS
 *  | 13 -  3 : LO_KCAL
 *  |       2 : RSVD
 *  |       1 : EN_KCAL_SD
 *  |       0 : DS_SEL
 */
 __STATIC_INLINE void cal_cal_SetCAL1(QN_CALIB_TypeDef *CALIB, uint32_t value)
 {
     __wr_reg((uint32_t)&CALIB->CAL1, value);
 }

 __STATIC_INLINE void cal_cal_SetCAL1WithMask(QN_CALIB_TypeDef *CALIB, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&CALIB->CAL1, mask, value);
 }

 __STATIC_INLINE uint32_t cal_cal_GetCAL1(QN_CALIB_TypeDef *CALIB)
 {
     return __rd_reg((uint32_t)&CALIB->CAL1);
 }


/**
 *
 * @param *CALIB CALIB Pointer
 * @param value The value to set to CAL2 register
 *
 * @brief  outputs specified value to CAL2 register
 *
 *  | 31 - 26 : RSVD
 *  | 25 - 16 : SD_GAIN_INV
 *  | 15 - 11 : RSVD
 *  | 10 -  8 : LO_KDAC_E
 *  |       7 : RSVD
 *  |  6 -  4 : TX_DLY2
 *  |       3 : RSVD
 *  |  2 -  0 : TX_DLY1
 */
 __STATIC_INLINE void cal_cal_SetCAL2(QN_CALIB_TypeDef *CALIB, uint32_t value)
 {
     __wr_reg((uint32_t)&CALIB->CAL2, value);
 }

 __STATIC_INLINE void cal_cal_SetCAL2WithMask(QN_CALIB_TypeDef *CALIB, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&CALIB->CAL2, mask, value);
 }

 __STATIC_INLINE uint32_t cal_cal_GetCAL2(QN_CALIB_TypeDef *CALIB)
 {
     return __rd_reg((uint32_t)&CALIB->CAL2);
 }


/**
 *
 * @param *CALIB CALIB Pointer
 * @param value The value to set to CAL3 register
 *
 * @brief  outputs specified value to CAL3 register
 *
 *  |      31 : PA_CAL_REQ
 *  |      30 : PA_CAL_DIS
 *  | 29 - 28 : RSVD
 *  | 27 - 24 : PA_CAL
 *  |      23 : R_CAL_REQ
 *  |      22 : R_CAL_DIS
 *  | 21 - 20 : RSVD
 *  | 19 - 16 : R_CAL
 *  |      15 : ROS_CAL_REQ
 *  |      14 : ROS_CAL_I_DIS
 *  | 13 -  8 : ROS_CAL_I
 *  |       7 : RSVD
 *  |       6 : ROS_CAL_Q_DIS
 *  |  5 -  0 : ROS_CAL_Q
 */
 __STATIC_INLINE void cal_cal_SetCAL3(QN_CALIB_TypeDef *CALIB, uint32_t value)
 {
     __wr_reg((uint32_t)&CALIB->CAL3, value);
 }

 __STATIC_INLINE void cal_cal_SetCAL3WithMask(QN_CALIB_TypeDef *CALIB, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&CALIB->CAL3, mask, value);
 }

 __STATIC_INLINE uint32_t cal_cal_GetCAL3(QN_CALIB_TypeDef *CALIB)
 {
     return __rd_reg((uint32_t)&CALIB->CAL3);
 }

/**
 *
 * @param *CALIB CALIB Pointer
 * @param value The value to set to CAL4 register
 *
 * @brief  outputs specified value to CAL4 register
 *
 *  |      31 : RCO_CAL_REQ
 *  |      30 : RCO_CAL_DIS
 *  | 29 - 28 : RSVD
 *  | 27 - 24 : RCO_CAL
 *  | 23 - 20 : PA_CODE_TX
 *  | 19 - 16 : PA_CODE_RX
 *  | 15 - 13 : RSVD
 *  | 12 -  8 : PLL_RDY_DLY
 *  |  7 -  0 : RSVD
 *  |  4 -  1 : LO_SU_DLY
 *  |       0 : PA_CAL_EN
 */
 __STATIC_INLINE void cal_cal_SetCAL4(QN_CALIB_TypeDef *CALIB, uint32_t value)
 {
     __wr_reg((uint32_t)&CALIB->CAL4, value);
 }

 __STATIC_INLINE void cal_cal_SetCAL4WithMask(QN_CALIB_TypeDef *CALIB, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&CALIB->CAL4, mask, value);
 }

 __STATIC_INLINE uint32_t cal_cal_GetCAL4(QN_CALIB_TypeDef *CALIB)
 {
     return __rd_reg((uint32_t)&CALIB->CAL4);
 }

/**
 *
 * @param *CALIB CALIB Pointer
 * @param value The value to set to CR register
 *
 * @brief  outputs specified value to CR register
 *
 *  |      31 : 32M_GATE_EN
 *  |      30 : 16M_GATE_EN
 *  | 29 -  9 : RSVD
 *  |       8 : CAL_IE
 *  |       7 : REF_DONE_IE
 *  |       6 : RC_DONE_IE
 *  |       5 : LO_DONE_IE
 *  |       4 : KVCO_DONE_IE
 *  |       3 : PA_DONE_IE
 *  |       2 : R_DONE_IE
 *  |       1 : ROS_DONE_IE
 *  |       0 : RCO_DONE_IE
 */
 __STATIC_INLINE void cal_cal_SetCR(QN_CALIB_TypeDef *CALIB, uint32_t value)
 {
     __wr_reg((uint32_t)&CALIB->CR, value);
 }

 __STATIC_INLINE void cal_cal_SetCRWithMask(QN_CALIB_TypeDef *CALIB, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&CALIB->CR, mask, value);
 }

 __STATIC_INLINE uint32_t cal_cal_GetCR(QN_CALIB_TypeDef *CALIB)
 {
     return __rd_reg((uint32_t)&CALIB->CR);
 }

/**
 *
 * @param *CALIB CALIB Pointer
 * @param value The value to set to SR register
 *
 * @brief  outputs specified value to SR register
 *
 *  | 31 -  8 : RSVD
 *  |       7 : REF_DONE_IF
 *  |       6 : RC_DONE_IF
 *  |       5 : LO_DONE_IF
 *  |       4 : KVCO_DONE_IF
 *  |       3 : PA_DONE_IF
 *  |       2 : R_DONE_IF
 *  |       1 : ROS_DONE_IF
 *  |       0 : RCO_DONE_IF
 */
 __STATIC_INLINE void cal_cal_ClrSR(QN_CALIB_TypeDef *CALIB, uint32_t value)
 {
     __wr_reg((uint32_t)&CALIB->SR, value);
 }

 __STATIC_INLINE void cal_cal_ClrSRWithMask(QN_CALIB_TypeDef *CALIB, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&CALIB->SR, mask, value);
 }

 __STATIC_INLINE uint32_t cal_cal_GetSR(QN_CALIB_TypeDef *CALIB)
 {
     return __rd_reg((uint32_t)&CALIB->SR);
 }

/*PROP driver functions*/
/**
 *
 * @param *PROP PROPIETARY MODE pointer
 * @param value The value to set to CR register
 *
 * @brief  outputs specified value to CR register
 *
 *  | 31 -  7 : RSVD
 *  |       6 : TX_BUSY
 *  |       5 : RX_BUSY
 *  |       4 : TX_INT
 *  |       3 : RX_INT
 *  |       2 : RX_INT_MASK
 *  |       1 : TX_INT_MASK
 *  |       0 : BIT_ORDER
 */
 __STATIC_INLINE void prop_prop_SetCr(QN_PROP_TypeDef *PROP, uint32_t value)
 {
     __wr_reg((uint32_t)&PROP->CR, value);
 }

 __STATIC_INLINE void prop_prop_SetCrWithMask(QN_PROP_TypeDef *PROP, uint32_t mask, uint32_t value)
 {
     __wr_reg_with_msk((uint32_t)&PROP->CR, mask, value);
 }

 __STATIC_INLINE uint32_t prop_prop_GetCr(QN_PROP_TypeDef *PROP)
 {
     return __rd_reg((uint32_t)&PROP->CR);
 }
/*@}*/ /* end of group QN_Driver QN9020 Inline Driver Definitions */

/// @endcond


#ifdef __cplusplus

}
#endif


#endif /* QN9020_DRIVER_H */

