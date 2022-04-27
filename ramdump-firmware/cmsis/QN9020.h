/**
 ****************************************************************************************
 *
 * @file QN9020.h
 *
 * @brief CMSIS compatible Cortex-M0 Core Peripheral Access Layer Header File for QN9020.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef QN9020_H
#define QN9020_H

#ifdef __cplusplus
 extern "C" {
#endif

/** @defgroup QN9020_Definitions QN9020 Definitions
  This file defines all structures and symbols for QN9020:
    - registers and bitfields
    - pin mux mask definitions
    - peripheral base address
    - Peripheral definitions
*/


/******************************************************************************/
/*                Processor and Core Peripherals                              */
/******************************************************************************/
/** @defgroup QN9020_Processor QN9020 Processor
    @ingroup QN9020_Definitions
    @brief Configuration of the Cortex-M0 Processor and Core Peripherals
  @{
*/

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */

typedef enum IRQn
{
    /******  Cortex-M0 Processor Exceptions Numbers ***************************************************/

    /* ToDo: use this Cortex interrupt numbers if your device is a CORTEX-M0 device           */
    NonMaskableInt_IRQn           = -14,      /*!<  2 Non Maskable Interrupt                  */
    HardFault_IRQn                = -13,      /*!<  3 Hard Fault Interrupt                    */
    SVCall_IRQn                   = -5,       /*!< 11 SV Call Interrupt                       */
    PendSV_IRQn                   = -2,       /*!< 14 Pend SV Interrupt                       */
    SysTick_IRQn                  = -1,       /*!< 15 System Tick Interrupt                   */

    /******  Device Specific Interrupt Numbers ********************************************************/
    /* ToDo: add here your device specific external interrupt numbers
         according the interrupt handlers defined in startup_Device.s
         eg.: Interrupt for Timer#1       TIMER1_IRQHandler  ->  TIM1_IRQn                    */
    GPIO_IRQn                     = 0,        /*!< 0  GPIO Event                              */
    ACMP0_IRQn                    = 1,        /*!< 1  ACMP0                                   */
    ACMP1_IRQn                    = 2,        /*!< 2  ACMP1                                   */
    BLE_IRQn                      = 3,        /*!< 3  BLE                                     */
    RTC_CAP_IRQn                  = 4,        /*!< 4  RTC capture                             */
    OSC_EN_IRQn                   = 5,        /*!< 5  BLE IP OSC_EN output                    */
    RTC_IRQn                      = 6,        /*!< 6  RTC                                     */
    ADC_IRQn                      = 7,        /*!< 7  ADC                                     */
    DMA_IRQn                      = 8,        /*!< 8  DMA                                     */
    UART0_TX_IRQn                 = 10,       /*!< 10 UART0 TX                                */
    UART0_RX_IRQn                 = 11,       /*!< 11 UART0 RX                                */
    SPI0_TX_IRQn                  = 12,       /*!< 12 SPI0 TX                                 */
    SPI0_RX_IRQn                  = 13,       /*!< 13 SPI1 RX                                 */
    UART1_TX_IRQn                 = 14,       /*!< 14 UART1 TX                                */
    UART1_RX_IRQn                 = 15,       /*!< 15 UART1 RX                                */
    SPI1_TX_IRQn                  = 16,       /*!< 16 SPI1 TX                                 */
    SPI1_RX_IRQn                  = 17,       /*!< 17 SPI1 RX                                 */
    I2C_IRQn                      = 18,       /*!< 18 I2C                                     */
    TIMER0_IRQn                   = 19,       /*!< 19 Timer 0                                 */
    TIMER1_IRQn                   = 20,       /*!< 20 Timer 1                                 */
    TIMER2_IRQn                   = 21,       /*!< 21 Timer 2                                 */
    TIMER3_IRQn                   = 22,       /*!< 22 Timer 3                                 */
    WDT_IRQn                      = 23,       /*!< 23 Watch Dog                               */
    PWM0_IRQn                     = 24,       /*!< 24 PWM CH0                                 */
    PWM1_IRQn                     = 25,       /*!< 25 PWM CH1                                 */
    CALIB_IRQn                    = 26,       /*!< 26 Calibration                             */
    TUNER_RX_IRQn                 = 29,       /*!< 29 Tuner Rx                                */
    TUNER_TX_IRQn                 = 30,       /*!< 30 Tuner Tx                                */
    TUNER_SETTING_IRQn            = 31,       /*!< 31 Tuner Setting                           */
} IRQn_Type;


/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M0 Processor and Core Peripherals */
/* ToDo: set the defines according your Device                                                    */
/* ToDo: define the correct core revision
         __CM0_REV if your device is a CORTEX-M0 device
         __CM3_REV if your device is a CORTEX-M3 device
         __CM4_REV if your device is a CORTEX-M4 device                                           */
#define __CM0_REV                 0x0201    /*!< Core Revision r2p1                               */
#define __NVIC_PRIO_BITS          2         /*!< Number of Bits used for Priority Levels          */
#define __Vendor_SysTickConfig    0         /*!< Set to 1 if different SysTick Config is used     */
#define __MPU_PRESENT             0         /*!< MPU present or not                               */
/* ToDo: define __FPU_PRESENT if your devise is a CORTEX-M4                                       */
#define __FPU_PRESENT             0         /*!< FPU present or not                               */

/*@}*/ /* end of group QN9020_IRQn */


/* ToDo: include the correct core_cm0.h file
         core_cm0.h if your device is a CORTEX-M0 device
         core_cm3.h if your device is a CORTEX-M3 device
         core_cm4.h if your device is a CORTEX-M4 device                                          */
#include "core_cm0.h"                       /* Cortex-M0 processor and core peripherals           */
/* ToDo: include your system_QN9020.h file
         replace 'QN9020' with your device name                                                   */

/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/
/** @defgroup QN9020_Peripherals QN9020 Peripherals
    @ingroup QN9020_Definitions
    @brief QN9020 Device Specific Peripheral registers structures
*/

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

/* ToDo: add here your device specific peripheral access structure typedefs
         following is an example for a timer                                  */

/*------------- System Controller (SYSCON) -----------------------------------*/
/** @defgroup QN_SYSCON QN_SYSCON
    @ingroup QN9020_Peripherals
    @brief QN9020 System Controller Registers Structure
  @{
*/
typedef struct
{
  __IO uint32_t CRSS;                       /*!< Offset: 0x000 Enable clock gating and set block reset */
  __IO uint32_t CRSC;                       /*!< Offset: 0x004 Disable clock gating and clear block reset */
  __IO uint32_t CMDCR;                      /*!< Offset: 0x008 Set clock switch and clock divider */
  __IO uint32_t STCR;                       /*!< Offset: 0x00C Set systick timer STCALIB and STCLKEN */
       uint32_t RESERVED0;
  __IO uint32_t SOCR;                       /*!< Offset: 0x014 Set exchange memory base address */
       uint32_t RESERVED1[2];
  __IO uint32_t PMCR0;                      /*!< Offset: 0x020 PIN mux control register0 */
  __IO uint32_t PMCR1;                      /*!< Offset: 0x024 PIN mux control register1 */
  __IO uint32_t PMCR2;                      /*!< Offset: 0x028 PIN mux control register2 */
  __IO uint32_t PDCR;                       /*!< Offset: 0x02C PAD driver control register */
  __IO uint32_t PPCR0;                      /*!< Offset: 0x030 PAD pull-up and pull-down control register0 */
  __IO uint32_t PPCR1;                      /*!< Offset: 0x034 PAD pull-up and pull-down control register1 */
  __IO uint32_t RCS;                        /*!< Offset: 0x038 Reset cause source register */
  __IO uint32_t IOWCR;                      /*!< Offset: 0x03C GPIO wakeup source control register */
  __I  uint32_t BLESR;                      /*!< Offset: 0x040 BLE status register */
       uint32_t RESERVED2[15];
  __IO uint32_t SMR;                        /*!< Offset: 0x080 Enter Scan or test mode */
       uint32_t RESERVED3;
  __I  uint32_t CHIP_ID;                    /*!< Offset: 0x088 chip id */
       uint32_t RESERVED4;
  __IO uint32_t PGCR0;                      /*!< Offset: 0x090 Power gating control register0 */
  __IO uint32_t PGCR1;                      /*!< Offset: 0x094 Power gating control register1 */
  __IO uint32_t PGCR2;                      /*!< Offset: 0x098 Power gating control register2 */
  __IO uint32_t GCR;                        /*!< Offset: 0x09C Gain Control register */
  __IO uint32_t IVREF_X32;                  /*!< Offset: 0x0A0 Control XTAL32 and IVREF */
  __IO uint32_t XTAL_BUCK;                  /*!< Offset: 0x0A4 Control XTAL and BUCK */
  __IO uint32_t LO1;                        /*!< Offset: 0x0A8 Control analog LO */
  __IO uint32_t LO2;                        /*!< Offset: 0x0AC Control frequency word */
  __IO uint32_t RXCR;                       /*!< Offset: 0x0B0 Control analog Rx */
  __IO uint32_t ADCCR;                      /*!< Offset: 0x0B4 Control SAR ADC clock source */
  __IO uint32_t ANALOGCR;                   /*!< Offset: 0x0B8 Control analog peripherals */
  __IO uint32_t ADDITIONCR;                 /*!< Offset: 0x0BC Addition control register */

} QN_SYSCON_TypeDef;

/* SYSCON Register bits mask and position */
// CLK_RST
#define SYSCON_MASK_GATING_TIMER3           0x80000000   /* 31 */
#define SYSCON_MASK_GATING_TIMER2           0x40000000   /* 30 */
#define SYSCON_MASK_GATING_TIMER1           0x20000000   /* 29 */
#define SYSCON_MASK_GATING_TIMER0           0x10000000   /* 28 */
#define SYSCON_MASK_GATING_UART1            0x08000000   /* 27 */
#define SYSCON_MASK_GATING_UART0            0x04000000   /* 26 */
#define SYSCON_MASK_GATING_SPI1             0x02000000   /* 25 */
#define SYSCON_MASK_GATING_SPI0             0x01000000   /* 24 */
#define SYSCON_MASK_GATING_32K_CLK          0x00800000   /* 23 */
#define SYSCON_MASK_GATING_SPI_AHB          0x00400000   /* 22 */
#define SYSCON_MASK_GATING_GPIO             0x00200000   /* 21 */
#define SYSCON_MASK_GATING_ADC              0x00100000   /* 20 */
#define SYSCON_MASK_GATING_DMA              0x00080000   /* 19 */
#define SYSCON_MASK_GATING_BLE_AHB          0x00040000   /* 18 */
#define SYSCON_MASK_GATING_PWM              0x00020000   /* 17 */
#define SYSCON_MASK_REBOOT_SYS              0x00010000   /* 16 */
#define SYSCON_MASK_LOCKUP_RST              0x00008000   /* 15 */
#define SYSCON_MASK_BLE_RST                 0x00004000   /* 14 */
#define SYSCON_MASK_DP_RST                  0x00002000   /* 13 */
#define SYSCON_MASK_DPREG_RST               0x00001000   /* 12 */
#define SYSCON_MASK_32K_RST                 0x00000800   /* 11 */
#define SYSCON_MASK_I2C_RST                 0x00000400   /* 10 */
#define SYSCON_MASK_GPIO_RST                0x00000200   /* 9 */
#define SYSCON_MASK_WDOG_RST                0x00000100   /* 8 */
#define SYSCON_MASK_TIMER3_RST              0x00000080   /* 7 */
#define SYSCON_MASK_TIMER2_RST              0x00000040   /* 6 */
#define SYSCON_MASK_TIMER1_RST              0x00000020   /* 5 */
#define SYSCON_MASK_TIMER0_RST              0x00000010   /* 4 */
#define SYSCON_MASK_USART1_RST              0x00000008   /* 3 */
#define SYSCON_MASK_USART0_RST              0x00000004   /* 2 */
#define SYSCON_MASK_DMA_RST                 0x00000002   /* 1 */
#define SYSCON_MASK_CPU_RST                 0x00000001   /* 0 */
// CLK_MUX_DIV_CTRL
#define SYSCON_MASK_CLK_MUX                 0xC0000000   /* 31 - 30 */
#define SYSCON_MASK_SEL_CLK_32K             0x20000000   /* 29 */
#define SYSCON_MASK_BLE_FRQ_SEL             0x10000000   /* 28 */
#define SYSCON_MASK_BLE_DIV_BYPASS          0x08000000   /* 27 */
#define SYSCON_MASK_BLE_DIVIDER             0x04000000   /* 26 */
#define SYSCON_MASK_AHB_DIV_BYPASS          0x02000000   /* 25 */
#define SYSCON_MASK_AHB_DIVIDER             0x01FF0000   /* 24 - 16 */
#define SYSCON_MASK_USART1_DIV_BYPASS       0x00008000   /* 15 */
#define SYSCON_MASK_USART1_DIVIDER          0x00007000   /* 14 - 12 */
#define SYSCON_MASK_USART0_DIV_BYPASS       0x00000800   /* 11 */
#define SYSCON_MASK_USART0_DIVIDER          0x00000700   /* 10 - 8 */
#define SYSCON_MASK_APB_DIV_BYPASS          0x00000040   /* 6 */
#define SYSCON_MASK_APB_DIVIDER             0x00000030   /* 5 - 4 */
#define SYSCON_MASK_TIMER_DIV_BYPASS        0x00000008   /* 3 */
#define SYSCON_MASK_TIMER_DIVIDER           0x00000007   /* 2 - 0 */
#define SYSCON_POS_CLK_MUX                          30
#define SYSCON_POS_SEL_CLK_32K                      29
#define SYSCON_POS_BLE_FRQ_SEL                      28
#define SYSCON_POS_BLE_DIV_BYPASS                   27
#define SYSCON_POS_BLE_DIVIDER                      26
#define SYSCON_POS_AHB_DIV_BYPASS                   25
#define SYSCON_POS_AHB_DIVIDER                      16
#define SYSCON_POS_USART1_DIV_BYPASS                15
#define SYSCON_POS_USART1_DIVIDER                   12
#define SYSCON_POS_USART0_DIV_BYPASS                11
#define SYSCON_POS_USART0_DIVIDER                    8
#define SYSCON_POS_APB_DIV_BYPASS                    6
#define SYSCON_POS_APB_DIVIDER                       4
#define SYSCON_POS_TIMER_DIV_BYPASS                  3
#define SYSCON_POS_TIMER_DIVIDER                     0
// SYS_TICK_CTRL
#define SYSCON_MASK_EN_STCLKEN              0x80000000   /* 31 */
#define SYSCON_MASK_STCALIB                 0x03FFFFFF   /* 25 - 0 */
// SRAM_OTP_CTRL
#define SYSCON_MASK_EM_BASE_ADDR            0x00003FFF   /* 13 - 0 */
// PIN_MUX_CTRL0
#define SYSCON_MASK_PIN_CTRL0               0xFFFFFFFF   /* 31 - 0 */
// PIN_MUX_CTRL1
#define SYSCON_MASK_FLASH_CTRL_PIN          0x80000000   /* 31 */
#define SYSCON_MASK_TEST_ENABLE1            0x40000000   /* 30 */
#define SYSCON_MASK_TEST_ENABLE0            0x20000000   /* 29 */
#define SYSCON_MASK_PIN_CTRL1               0x1FFFFFFF   /* 28 - 0 */
// PIN_MUX_CTRL2
#define SYSCON_MASK_TEST_CTRL               0xF8000000   /* 31 - 27 */
#define SYSCON_MASK_CLKOUT1_PIN_SEL         0x00000080   /* 7 */
#define SYSCON_MASK_CLKOUT0_PIN_SEL         0x00000040   /* 6 */
#define SYSCON_MASK_UART1_PIN_SEL           0x00000020   /* 5 */
#define SYSCON_MASK_I2C_PIN_SEL             0x00000010   /* 4 */
#define SYSCON_MASK_ADCT_PIN_SEL            0x00000008   /* 3 */
#define SYSCON_MASK_SPI0_PIN_SEL            0x00000002   /* 1 */
#define SYSCON_MASK_SPI1_PIN_SEL            0x00000001   /* 0 */
#define SYSCON_POS_TEST_CTRL                        27
// PAD_DRV_CTRL
#define SYSCON_MASK_PAD_DRV_CTRL            0x7FFFFFFF   /* 30 - 0 */
// PAD_PULL_CTRL0
#define SYSCON_MASK_PAD_PULL_CTRL0          0xFFFFFFFF   /* 31 - 0 */
// PAD_PULL_CTRL1
#define SYSCON_MASK_PAD_PULL_CTRL1          0x3FFFFFFF   /* 29 - 0 */
// RST_CAUSE_SRC
#define SYSCON_MASK_RST_CAUSE_CLR           0x80000000   /* 31 */
#define SYSCON_MASK_RST_CAUSE               0x000000FF   /* 7 - 0 */
// IO_WAKEUP_CTRL
#define SYSCON_MASK_IO_VALUE                0xFFFF0000   /* 31 - 16 */
#define SYSCON_MASK_IO_WAKEUP_EN            0x0000FFFF   /* 15 - 0 */
#define SYSCON_POS_IO_VALUE                         16
// BLE_STATUS
#define SYSCON_MASK_CLK_RDY                 0x00020000  /* 17 */
#define SYSCON_MASK_CLK_XTAL32_RDY          0x00010000  /* 16 */
#define SYSCON_MASK_REF_PLL_RDY             0x00008000  /* 15 */
#define SYSCON_MASK_BG_RDY                  0x00004000  /* 14 */
#define SYSCON_MASK_BUCK_RDY                0x00002000  /* 13 */
#define SYSCON_MASK_TX_EN                   0x00001000  /* 12 */
#define SYSCON_MASK_RX_EN                   0x00000800  /* 11 */
#define SYSCON_MASK_OSC_EN                  0x00000400  /* 10 */
#define SYSCON_MASK_CLK_STATUS              0x00000200  /* 9 */
#define SYSCON_MASK_RADIO_EN                0x00000100  /* 8 */
#define SYSCON_MASK_FREQ_WORD               0x000000FF  /* 7 - 0 */
// SYS_MODE_REG
#define SYSCON_MASK_BOOT_MODE               0x80000000   /* 31 */
#define SYSCON_MASK_EN_SW_MAP               0x40000000   /* 30 */
#define SYSCON_MASK_RAM_BIST_FAIL           0x00000020   /* 5 */
#define SYSCON_MASK_RAM_BIST_END            0x00000010   /* 4 */
#define SYSCON_MASK_ROM_BIST_FAIL           0x00000008   /* 3 */
#define SYSCON_MASK_ROM_BIST_END            0x00000004   /* 2 */
#define SYSCON_MASK_BIST_START              0x00000002   /* 1 */
#define SYSCON_MASK_TEST_MOD                0x00000001   /* 0 */
// CHIP_ID
#define SYSCON_MASK_CHIP_ID                 0x0000FFFF  /* 15 - 0 */
// POWER_GATING_CTRL0
#define SYSCON_MASK_SEL_PD                  0x80000000   /* 31 */
#define SYSCON_MASK_PD_OSC                  0x40000000   /* 30 */
#define SYSCON_MASK_PD_BG                   0x20000000   /* 29 */
#define SYSCON_MASK_PD_V2I                  0x10000000   /* 28 */
#define SYSCON_MASK_PD_BUCK                 0x08000000   /* 27 */
#define SYSCON_MASK_PD_VREG_A               0x04000000   /* 26 */
#define SYSCON_MASK_PD_VREG_D               0x02000000   /* 25 */
#define SYSCON_MASK_PD_XTAL                 0x01000000   /* 24 */
#define SYSCON_MASK_PD_XTAL32               0x00800000   /* 23 */
#define SYSCON_MASK_PD_REF_PLL_B0           0x00400000   /* 22 */
#define SYSCON_MASK_DIV_RST_SYNC_B1         0x00400000   /* 22 */
#define SYSCON_MASK_PD_LO_VCO               0x00200000   /* 21 */
#define SYSCON_MASK_PD_LO_PLL               0x00100000   /* 20 */
#define SYSCON_MASK_PD_PA                   0x00080000   /* 19 */
#define SYSCON_MASK_PD_LNA                  0x00040000   /* 18 */
#define SYSCON_MASK_PD_LNA_PKDET            0x00020000   /* 17 */
#define SYSCON_MASK_PD_MIXER                0x00010000   /* 16 */
#define SYSCON_MASK_PD_PPF_PKDET            0x00008000   /* 15 */
#define SYSCON_MASK_PD_PPF                  0x00004010   /* 14 */
#define SYSCON_MASK_PD_RX_PKDET             0x00002000   /* 13 */
#define SYSCON_MASK_PD_RX_ADC               0x00001000   /* 12 */
#define SYSCON_MASK_PD_SAR_ADC              0x00000800   /* 11 */
#define SYSCON_MASK_PD_RCO                  0x00000400   /* 10 */
#define SYSCON_MASK_BOND_EN                 0x00000200   /* 9 */
#define SYSCON_MASK_PD_MEM7                 0x00000080   /* 7 */
#define SYSCON_MASK_PD_MEM6                 0x00000040   /* 6 */
#define SYSCON_MASK_PD_MEM5                 0x00000020   /* 5 */
#define SYSCON_MASK_PD_MEM4                 0x00000010   /* 4 */
#define SYSCON_MASK_PD_MEM3                 0x00000008   /* 3 */
#define SYSCON_MASK_PD_MEM2                 0x00000004   /* 2 */
#define SYSCON_MASK_PD_MEM1                 0x00000002   /* 1 */
#define SYSCON_MASK_PL_VREG_D               0x00000001   /* 0 */
// POWER_GATING_CTRL1
#define SYSCON_MASK_VDD_RCO_SET             0x80000000   /* 31 */
#define SYSCON_MASK_DIS_OSC                 0x40000000   /* 30 */
#define SYSCON_MASK_DIS_BG                  0x20000000   /* 29 */
#define SYSCON_MASK_DIS_V2I                 0x10000000   /* 28 */
#define SYSCON_MASK_DIS_BUCK                0x08000000   /* 27 */
#define SYSCON_MASK_DIS_VREG_A              0x04000000   /* 26 */
#define SYSCON_MASK_DIS_VREG_D              0x02000000   /* 25 */
#define SYSCON_MASK_DIS_XTAL                0x01000000   /* 24 */
#define SYSCON_MASK_DIS_XTAL32              0x00800000   /* 23 */
#define SYSCON_MASK_DIS_REF_PLL             0x00400000   /* 22 */
#define SYSCON_MASK_DIS_LO_VCO              0x00200000   /* 21 */
#define SYSCON_MASK_DIS_LO_PLL              0x00100000   /* 20 */
#define SYSCON_MASK_DIS_PA                  0x00080000   /* 19 */
#define SYSCON_MASK_DIS_LNA                 0x00040000   /* 18 */
#define SYSCON_MASK_DIS_LNA_PKDET           0x00020000   /* 17 */
#define SYSCON_MASK_DIS_MIXER               0x00010000   /* 16 */
#define SYSCON_MASK_DIS_PPF_PKDET           0x00008000   /* 15 */
#define SYSCON_MASK_DIS_PPF                 0x00004000   /* 14 */
#define SYSCON_MASK_DIS_RX_PKDET            0x00002000   /* 13 */
#define SYSCON_MASK_DIS_RX_ADC              0x00001000   /* 12 */
#define SYSCON_MASK_DIS_SAR_ADC             0x00000800   /* 11 */
#define SYSCON_MASK_DIS_RCO                 0x00000400   /* 10 */
#define SYSCON_MASK_DIS_MEM7                0x00000080   /* 7 */
#define SYSCON_MASK_DIS_MEM6                0x00000040   /* 6 */
#define SYSCON_MASK_DIS_MEM5                0x00000020   /* 5 */
#define SYSCON_MASK_DIS_MEM4                0x00000010   /* 4 */
#define SYSCON_MASK_DIS_MEM3                0x00000008   /* 3 */
#define SYSCON_MASK_DIS_MEM2                0x00000004   /* 2 */
#define SYSCON_MASK_DIS_MEM1                0x00000002   /* 1 */
#define SYSCON_MASK_DIS_SAR_BUF             0x00000001   /* 0 */
// POWER_GATING_CTRL2
#define SYSCON_MASK_VREG12_A_BAK            0x00030000  /* 17 - 16 */
#define SYSCON_MASK_OSC_WAKEUP_EN           0x00000100  /* 8 */
#define SYSCON_MASK_RTCI_PIN_SEL            0x00000080  /* 7 */
#define SYSCON_MASK_PD_STATE                0x00000040  /* 6 */
#define SYSCON_MASK_BD_AMP_EN               0x00000020  /* 5 */
#define SYSCON_MASK_DVDD12_PMU_SET          0x00000010  /* 4 */
#define SYSCON_MASK_RX_EN_SEL               0x00000008  /* 3 */
#define SYSCON_MASK_FLASH_VCC_EN            0x00000004  /* 2 */
#define SYSCON_MASK_PMUENABLE               0x00000002  /* 1 */
#define SYSCON_MASK_DBGPMUENABLE            0x00000001  /* 0 */
#define SYSCON_POS_VREG12_A_BAK                     16
// GAIN_CTRL
#define SYSCON_MASK_TX_PWR_SEL              0x80000000  /* 31 */
#define SYSCON_MASK_PA_GAIN_BOOST           0x40000000  /* 30 */
#define SYSCON_MASK_BM_PA                   0x30000000  /* 29 - 28 */
#define SYSCON_MASK_PA_GAIN                 0x0F000000  /* 27 - 24 */
#define SYSCON_MASK_LNA_GAIN1               0x00C00000  /* 23 - 22 */
#define SYSCON_MASK_LNA_GAIN2               0x00300000  /* 21 - 20 */
#define SYSCON_MASK_PPF_GAIN                0x000F0000  /* 19 - 16 */
#define SYSCON_MASK_LNA_GAIN_WEN            0x00002000  /* 13 */
#define SYSCON_MASK_PPF_GAIN_WEN            0x00001000  /* 12 */
#define SYSCON_MASK_VT_PKDET1_HG            0x00000E00  /* 11 - 9 */
#define SYSCON_MASK_VT_PKDET1_MG            0x000001C0  /* 8 - 6 */
#define SYSCON_MASK_VT_PKDET1_LG            0x00000038  /* 5 - 3 */
#define SYSCON_MASK_VT_PKDET2               0x00000002  /* 1 */
#define SYSCON_MASK_VT_PKDET3               0x00000001  /* 0 */
#define SYSCON_POS_PA_GAIN_BOOST                    30
#define SYSCON_POS_BM_PA                            28
#define SYSCON_POS_PA_GAIN                          24
#define SYSCON_POS_LNA_GAIN1                        22
#define SYSCON_POS_LNA_GAIN2                        20
#define SYSCON_POS_PPF_GAIN                         16
#define SYSCON_POS_VT_PKDET1_HG                      9
#define SYSCON_POS_VT_PKDET1_MG                      6
#define SYSCON_POS_VT_PKDET1_LG                      3
// IVREF_X32
#define SYSCON_MASK_BGSEL                   0xF0000000  /* 31 - 28 */
#define SYSCON_MASK_VREG15                  0x0C000000  /* 27 - 26 */
#define SYSCON_MASK_VREG12_A                0x03000000  /* 25 - 24 */
#define SYSCON_MASK_TR_SWITCH               0x00800000  /* 23 */
#define SYSCON_MASK_BM_PKDET3               0x00600000  /* 22 - 21 */
#define SYSCON_MASK_VREG12_D                0x00180000  /* 20 - 19 */
#define SYSCON_MASK_DVDD12_SW_EN            0x00040000  /* 18 */
#define SYSCON_MASK_BUCK_BYPASS             0x00020000  /* 17 */
#define SYSCON_MASK_BUCK_DPD                0x00010000  /* 16 */
#define SYSCON_MASK_BUCK_ERR_ISEL           0x0000C000  /* 15 - 14 */
#define SYSCON_MASK_BUCK_VBG                0x00003000  /* 13 - 12 */
#define SYSCON_MASK_X32SMT_EN               0x00000800  /* 11 */
#define SYSCON_MASK_X32BP_RES               0x00000400  /* 10 */
#define SYSCON_MASK_BM_X32BUF               0x00000300  /* 9 - 8 */
#define SYSCON_MASK_X32INJ                  0x000000C0  /* 7 - 6 */
#define SYSCON_MASK_X32ICTRL                0x0000003F  /* 5 - 0 */
#define SYSCON_POS_BGSEL                            28
#define SYSCON_POS_VREG15                           26
#define SYSCON_POS_VREG12_A                         24
#define SYSCON_POS_BM_PKDET3                        21
#define SYSCON_POS_VREG12_D                         19
#define SYSCON_POS_BUCK_ERR_ISEL                    14
#define SYSCON_POS_BUCK_VBG                         12
#define SYSCON_POS_BM_X32BUF                         8
#define SYSCON_POS_X32INJ                            6
#define SYSCON_POS_X32ICTRL                          0
// XTAL_BUCK
#define SYSCON_MASK_XSP_CSEL_B0             0x80000000  /* 31 */
#define SYSCON_MASK_XTAL_INJ                0x60000000  /* 30 - 29 */
#define SYSCON_MASK_XICTRL                  0x1F800000  /* 28 - 23 */
#define SYSCON_MASK_XCSEL                   0x007E0000  /* 22 - 17 */
#define SYSCON_MASK_XSMT_EN                 0x00010000  /* 16 */
#define SYSCON_MASK_BUCK_VTHL               0x0000C000  /* 15 - 14 */
#define SYSCON_MASK_BUCK_VTHH               0x00003000  /* 13 - 12 */
#define SYSCON_MASK_BUCK_TMOS               0x00000E00  /* 11 - 9 */
#define SYSCON_MASK_BUCK_FC                 0x00000100  /* 8 */
#define SYSCON_MASK_BUCK_AGAIN              0x00000080  /* 7 */
#define SYSCON_MASK_BUCK_ADRES              0x00000040  /* 6 */
#define SYSCON_MASK_BUCK_BM                 0x00000030  /* 5 - 4 */
#define SYSCON_MASK_TST_CPREF               0x0000000F  /* 3 - 0 */
#define SYSCON_POS_XTAL_INJ                         29
#define SYSCON_POS_XICTRL                           23
#define SYSCON_POS_XCSEL                            17
#define SYSCON_POS_BUCK_VTHL_SEL                    14
#define SYSCON_POS_BUCK_VTHH_SEL                    12
#define SYSCON_POS_BUCK_TMOS                         9
#define SYSCON_POS_BUCK_BM                           4
// LO1
#define SYSCON_MASK_XDIV                    0x80000000  /* 31 */
#define SYSCON_MASK_LO_TEST_INT             0x40000000  /* 30 */
#define SYSCON_MASK_LO_TST_CP               0x3C000000  /* 29 - 26 */
#define SYSCON_MASK_LO_ICPH                 0x02000000  /* 25 */
#define SYSCON_MASK_LO_BM_FIL               0x01800000  /* 24 - 23 */
#define SYSCON_MASK_LO_BM_DAC               0x00600000  /* 22 - 21 */
#define SYSCON_MASK_LO_BM_CML_C             0x00180000  /* 20 - 19 */
#define SYSCON_MASK_LO_BM_CML_D             0x00060000  /* 18 - 17 */
#define SYSCON_MASK_LO_BM_BVCO              0x00018000  /* 16 - 15 */
#define SYSCON_MASK_LO_VCO_AMP              0x00007000  /* 14 - 12 */
#define SYSCON_MASK_PMUX_EN                 0x00000800  /* 11 */
#define SYSCON_MASK_PA_PHASE                0x00000600  /* 10 - 9 */
#define SYSCON_MASK_LO_DAC_TEST_EN          0x00000100  /* 8 */
#define SYSCON_MASK_LO_DAC_TEST             0x000000FF  /* 7 - 0 */
#define SYSCON_POS_LO_TST_CP                        26
#define SYSCON_POS_LO_BM_FIL                        23
#define SYSCON_POS_LO_BM_DAC                        21
#define SYSCON_POS_LO_BM_CML_C                      19
#define SYSCON_POS_LO_BM_CML_D                      17
#define SYSCON_POS_LO_BM_BVCO                       15
#define SYSCON_POS_LO_VCO_AMP                       12
#define SYSCON_POS_PA_PHASE                          9
// LO2
#define SYSCON_MASK_LO_SEL                  0x80000000  /* 31 */
#define SYSCON_MASK_EN_DATA_VLD             0x40000000  /* 30 */
#define SYSCON_MASK_LO_FRAC                 0x3FFFFF00  /* 29 - 8 */
#define SYSCON_MASK_LO_CHANGE               0x00000080  /* 7 */
#define SYSCON_MASK_LO_REG                  0x00000040  /* 6 */
#define SYSCON_MASK_LO_TXBW                 0x00000020  /* 5 */
#define SYSCON_MASK_LO_INT                  0x0000001F  /* 4 - 0 */
#define SYSCON_POS_LO_FRAC                           8
#define SYSCON_POS_LO_INT                            0
// RX_CTRL
#define SYSCON_MASK_LNA_LOAD_CAP            0xF0000000  /* 31 - 28 */
#define SYSCON_MASK_BM_LNA_GM               0x0C000000  /* 27 - 26 */
#define SYSCON_MASK_BM_LNA                  0x03000000  /* 25 - 24 */
#define SYSCON_MASK_BM_LNA_PK               0x00C00000  /* 23 - 22 */
#define SYSCON_MASK_IQSWAP                  0x00200000  /* 21 */
#define SYSCON_MASK_BM_PPF                  0x00180000  /* 20 - 19 */
#define SYSCON_MASK_EN_DPF_DIS              0x00040000  /* 18 */
#define SYSCON_MASK_VT_ADC_RST              0x00030000  /* 17 - 16 */
#define SYSCON_MASK_IMR                     0x00008000  /* 15 */
#define SYSCON_MASK_RSTN_DWA                0x00004000  /* 14 */
#define SYSCON_MASK_ADC_DAC2I               0x00003000  /* 13 - 12 */
#define SYSCON_MASK_ADC_DAC3I               0x00000C00  /* 11 - 10 */
#define SYSCON_MASK_ADC_RCTEMP              0x00000200  /* 9 */
#define SYSCON_MASK_ADC_CAP_SEL             0x00000100  /* 8 */
#define SYSCON_MASK_ADC_DCON                0x000000E0  /* 7 - 5 */
#define SYSCON_MASK_ADC_OPA12I              0x00000018  /* 4 - 3 */
#define SYSCON_MASK_ADC_OPA4I               0x00000006  /* 2 - 1 */
#define SYSCON_MASK_STF_PK_EN               0x00000001  /* 0 */
#define SYSCON_POS_LNA_LOAD_CAP                     28
#define SYSCON_POS_BM_LNA_GM                        26
#define SYSCON_POS_BM_LNA                           24
#define SYSCON_POS_BM_LNA_PK                        22
#define SYSCON_POS_BM_PPF                           19
#define SYSCON_POS_VT_ADC_RST                       16
#define SYSCON_POS_ADC_DAC2I                        12
#define SYSCON_POS_ADC_DAC3I                        10
#define SYSCON_POS_ADC_DCON                          5
#define SYSCON_POS_ADC_OPA12I                        3
#define SYSCON_POS_ADC_OPA4I                         1
// SAR_ADC_CTRL
#define SYSCON_MASK_SPEED_UP_TIME           0x0FF00000  /* 27 - 20 */
#define SYSCON_MASK_CK_DAC_DLY              0x00040000  /* 18 */
#define SYSCON_MASK_BYPASS_TESTBUF          0x00020000  /* 17 */
#define SYSCON_MASK_SEL_TEST_EN             0x00010000  /* 16 */
#define SYSCON_MASK_TESTREG                 0x0000FF00  /* 15 - 8 */
#define SYSCON_MASK_ADC_DIG_RST             0x00000040  /* 6 */
#define SYSCON_MASK_ADC_CLK_SEL             0x00000020  /* 5 */
#define SYSCON_MASK_ADC_DIV_BYPASS          0x00000010  /* 4 */
#define SYSCON_MASK_ADC_DIV                 0x0000000F  /* 3 - 0 */
#define SYSCON_POS_TESTREG                          20
#define SYSCON_POS_SPEED_UP_TIME                     8
#define SYSCON_POS_ADC_CLK_SEL                       5
// ANALOG_CTRL
#define SYSCON_MASK_ACMP0_REF               0xF0000000  /* 31 - 28 */
#define SYSCON_MASK_ACMP1_REF               0x0F000000  /* 27 - 24 */
#define SYSCON_MASK_BD_TH                   0x00C00000  /* 23 - 22 */
#define SYSCON_MASK_ACMP0_EN                0x00200000  /* 21 */
#define SYSCON_MASK_ACMP1_EN                0x00100000  /* 20 */
#define SYSCON_MASK_BT_EN                   0x00080000  /* 19 */
#define SYSCON_MASK_BD_EN                   0x00040000  /* 18 */
#define SYSCON_MASK_TS_EN                   0x00020000  /* 17 */
#define SYSCON_MASK_ACMP1_VALUE             0x00010000  /* 16 */
#define SYSCON_MASK_ACMP0_VALUE             0x00008000  /* 15 */
#define SYSCON_MASK_ACMP0_HYST_EN           0x00004000  /* 14 */
#define SYSCON_MASK_ACMP1_HYST_EN           0x00002000  /* 13 */
#define SYSCON_MASK_AINX_EN                 0x00001E00  /* 12 - 9 */
#define SYSCON_MASK_BUCK_PMDR_B1            0x00000100  /* 8 */
#define SYSCON_MASK_BUCK_NMDR_B1            0x00000080  /* 7 */
#define SYSCON_MASK_PA_GAIN_BIT4_B1         0x00000040  /* 6 */
#define SYSCON_MASK_XSP_CSEL_B1             0x00000020  /* 5 */
#define SYSCON_POS_ACMP0_REF                        28
#define SYSCON_POS_ACMP1_REF                        24
#define SYSCON_POS_BD_TH                            22
#define SYSCON_POS_ACMP1_VALUE                      16
#define SYSCON_POS_ACMP0_VALUE                      15
#define SYSCON_POS_ACMP0_HYST                       14
#define SYSCON_POS_ACMP1_HYST                       13
#define SYSCON_POS_AINX_EN                           9
// ADDITION_CTRL
#define SYSCON_MASK_BUCK_TMOS_BAK           0x00007000   /* 14 - 12 */
#define SYSCON_MASK_BUCK_BM_BAK             0x00000C00   /* 11 - 10 */
#define SYSCON_MASK_HALF_LO_OPCUR           0x00000200   /* 9 */
#define SYSCON_MASK_EN_RXDAC                0x00000100   /* 8 */
#define SYSCON_MASK_TX_PLL_PFD_DIS          0x00000080   /* 7 */
#define SYSCON_MASK_RX_PLL_PFD_DIS          0x00000040   /* 6 */
#define SYSCON_MASK_CALI_REDUCE             0x00000020   /* 5 */
#define SYSCON_MASK_REF_REDUCE_I            0x00000010   /* 4 */
#define SYSCON_MASK_XADD_C                  0x00000008   /* 3 */
#define SYSCON_MASK_DIS_XPD_DLY             0x00000004   /* 2 */
#define SYSCON_MASK_PA_CKEN_SEL             0x00000002   /* 1 */
#define SYSCON_MASK_DC_CAL_MODE             0x00000001   /* 0 */
#define SYSCON_POS_BUCK_TMOS_BAK                    12
#define SYSCON_POS_BUCK_BM_BAK                      10
/*@}*/ /* end of group QN_SYSCON */


/*------------- General Purpose Input/Output (GPIO) --------------------------*/
/** @defgroup QN_GPIO QN_GPIO
    @ingroup QN9020_Peripherals
    @brief QN9020 General Purpose Input/Output Register Structure
  @{
*/
typedef struct
{
  __IO   uint32_t  DATA;                    /*!< Offset: 0x000 DATA Register (R/W) */
  __IO   uint32_t  DATAOUT;                 /*!< Offset: 0x004 Data Output Latch Register (R/W) */
         uint32_t  RESERVED0[2];
  __IO   uint32_t  OUTENABLESET;            /*!< Offset: 0x010 Output Enable Set Register  (R/W) */
  __IO   uint32_t  OUTENABLECLR;            /*!< Offset: 0x014 Output Enable Clear Register  (R/W) */
  __IO   uint32_t  ALTFUNCSET;              /*!< Offset: 0x018 Alternate Function Set Register  (R/W) */
  __IO   uint32_t  ALTFUNCCLR;              /*!< Offset: 0x01C Alternate Function Clear Register  (R/W) */
  __IO   uint32_t  INTENSET;                /*!< Offset: 0x020 Interrupt Enable Set Register  (R/W) */
  __IO   uint32_t  INTENCLR;                /*!< Offset: 0x024 Interrupt Enable Clear Register  (R/W) */
  __IO   uint32_t  INTTYPESET;              /*!< Offset: 0x028 Interrupt Type Set Register  (R/W) */
  __IO   uint32_t  INTTYPECLR;              /*!< Offset: 0x02C Interrupt Type Clear Register  (R/W) */
  __IO   uint32_t  INTPOLSET;               /*!< Offset: 0x030 Interrupt Polarity Set Register  (R/W) */
  __IO   uint32_t  INTPOLCLR;               /*!< Offset: 0x034 Interrupt Polarity Clear Register  (R/W) */
  union {
  __I    uint32_t  INTSTATUS;               /*!< Offset: 0x038 Interrupt Status Register (R/ ) */
  __O    uint32_t  INTCLEAR;                /*!< Offset: 0x038 Interrupt Clear Register ( /W) */
  };
         uint32_t RESERVED1[241];
  __IO   uint32_t LB_LW_MASKED[256];           /*!< Offset: 0x400 - 0x7FC Lower byte of lower word Masked Access Register (R/W) */
  __IO   uint32_t UB_LW_MASKED[256];           /*!< Offset: 0x800 - 0xBFC Upper byte of lower word Masked Access Register (R/W) */
         uint32_t RESERVED2[256];
  __IO   uint32_t LB_UW_MASKED[256];           /*!< Offset: 0x1000 - 0x13FC Lower byte of upper word Masked Access Register (R/W) */
  __IO   uint32_t UB_UW_MASKED[256];           /*!< Offset: 0x1400 - 0x17FC Upper byte of upper word Masked Access Register (R/W) */

} QN_GPIO_TypeDef;
/*@}*/ /* end of group QN_GPIO */


/*------------------- WDT ----------------------------------------------*/
/** @defgroup QN_WDT QN_WDT
    @ingroup QN9020_Peripherals
    @brief QN9020 Watch Dog Timer Register Structure
  @{
*/
typedef struct
{
  __IO    uint32_t  LOAD;                   /*!< Offset: 0x000 WDT Load Register (R/W) */
  __I     uint32_t  VALUE;                  /*!< Offset: 0x004 WDT Value Register (R/ ) */
  __IO    uint32_t  CTRL;                   /*!< Offset: 0x008 WDT Control Register
                                                                           bit1 RESEN: Reset enable
                                                                           bit0 INTEN: Interrupt enable  (R/W) */
  __O     uint32_t  INTCLR;                 /*!< Offset: 0x00C WDT Clear Interrupt Register (R/W) */
  __I     uint32_t  RAWINTSTAT;             /*!< Offset: 0x010 WDT Raw Interrupt Status Register (R/W) */
  __I     uint32_t  MASKINTSTAT;            /*!< Offset: 0x014 WDT Interrupt Status Register (R/W) */
          uint32_t  RESERVED0[762];
  __IO    uint32_t  LOCK;                   /*!< Offset: 0xC00 WDT Lock Register (R/W) */
          uint32_t  RESERVED1[191];
  __IO    uint32_t  ITCR;                   /*!< Offset: 0xF00 WDT Integration Test Control Register (R/W) */
  __O     uint32_t  ITOP;                   /*!< Offset: 0xF04 WDT Integration Test Output Set Register (R/W) */
}QN_WDT_TypeDef;

/* WDT REGISTER MASK */
#define WDT_MASK_LOAD                       0xFFFFFFFF      /*!< WDT LOAD: LOAD Mask */
#define WDT_MASK_VALUE                      0xFFFFFFFF      /*!< WDT VALUE: VALUE Mask */
#define WDT_POS_CTRL_RESEN                  1               /*!< WDT CTRL_RESEN: Enable Reset Output Position */
#define WDT_MASK_CTRL_RESEN                 0x00000002      /*!< WDT CTRL_RESEN: Enable Reset Output Mask */
#define WDT_MASK_CTRL_INTEN                 0x00000001      /*!< WDT CTRL_INTEN: Int Enable Mask */
#define WDT_MASK_INTCLR                     0x00000001      /*!< WDT INTCLR: Int Clear Mask */
#define WDT_MASK_RAWINTSTAT                 0x00000001      /*!< WDT RAWINTSTAT: Raw Int Status Mask */
#define WDT_MASK_MASKINTSTAT                0x00000001      /*!< WDT MASKINTSTAT: Mask Int Status Mask */
#define WDT_MASK_LOCK                       0x00000001      /*!< WDT LOCK: LOCK Mask */
#define WDT_MASK_INTEGTESTEN                0x00000001      /*!< WDT INTEGTESTEN: Integration Test Enable Mask */
#define WDT_MASK_INTEGTESTOUTSET            0x00000001      /*!< WDT INTEGTESTOUTSET: Integration Test Output Set Mask */
/*@}*/ /* end of group  QN_WDT */



/*------------- Serial Peripheral interface (SPI) -----------------------*/
/** @defgroup QN_SPI QN_SPI
    @ingroup QN9020_Peripherals
    @brief QN9020 Synchronous Serial Port Register Structure
  @{
*/
typedef struct
{
  __IO uint32_t CR0;                        /*!< Offset: 0x000 Control Register 0 (R/W) */
  __IO uint32_t CR1;                        /*!< Offset: 0x004 Control Register 1 (R/W) */
  __I  uint32_t FPR;                        /*!< Offset: 0x008 FIFO pointer Register (R/ ) */
  __I  uint32_t FSR;                        /*!< Offset: 0x00C FIFO status Register (R/ ) */
  __O  uint32_t TXD;                        /*!< Offset: 0x010 Tx data Register (/W) */
  __I  uint32_t RXD;                        /*!< Offset: 0x014 Rx data Register (R/) */
  __IO uint32_t SR;                         /*!< Offset: 0x018 status Register (R/) */
} QN_SPI_TypeDef;

/* SPI REGISTER MASK */
// CR0
#define SPI_MASK_BITRATE                    0x003F0000   /* 21 - 16 */
#define SPI_MASK_MSTR_SS1                   0x00008000   /* 15 */
#define SPI_MASK_MSTR_SS0                   0x00004000   /* 14 */
#define SPI_MASK_SPI_IE                     0x00000800   /* 11 */
#define SPI_MASK_RX_FIFO_OVR_IE             0x00000400   /* 10 */
#define SPI_MASK_RX_FIFO_NEMT_IE            0x00000200   /* 9 */
#define SPI_MASK_TX_FIFO_NFUL_IE            0x00000100   /* 8 */
#define SPI_MASK_DATA_IO_MODE               0x00000080   /* 7 */
#define SPI_MASK_BYTE_ENDIAN                0x00000040   /* 6 */
#define SPI_MASK_BUF_WIDTH                  0x00000020   /* 5 */
#define SPI_MASK_BIT_ORDER                  0x00000010   /* 4 */
#define SPI_MASK_SPI_MODE                   0x0000000C   /* 3 - 2 */
#define SPI_MASK_CPHA                       0x00000002   /* 1 */
#define SPI_MASK_CPOL                       0x00000001   /* 0 */
#define SPI_POS_CLK_DIV_FLASH                       24
#define SPI_POS_CLK_DIV_MASTER                      16
#define SPI_POS_MSTR_SS1                            15
#define SPI_POS_MSTR_SS0                            14
#define SPI_POS_INT_EN                              11
#define SPI_POS_RX_OVR_IE                           10
#define SPI_POS_RX_NEMT_IE                           9
#define SPI_POS_TX_NFUL_IE                           8
#define SPI_POS_DATA_IO_MODE                         7
#define SPI_POS_BYTE_ENDIAN                          6
#define SPI_POS_BUFFER_WIDTH                         5
#define SPI_POS_BIT_ORDERING                         4
#define SPI_POS_MODE                                 2
#define SPI_POS_PHASE                                1
// CR1
#define SPI_MASK_RX_FIFO_CLR                0x00020000   /* 17 */
#define SPI_MASK_TX_FIFO_CLR                0x00010000   /* 16 */
#define SPI_MASK_S_SDIO_EN                  0x00000200   /* 9 */
#define SPI_MASK_M_SDIO_EN                  0x00000100   /* 8 */
#define SPI_POS_RX_FIFO_CLR                         17
#define SPI_POS_TX_FIFO_CLR                         16
#define SPI_POS_S_SDIO_EN                            9
#define SPI_POS_M_SDIO_EN                            8
// FPR
#define SPI_MASK_RX_FIFO_RD_PTR             0x03000000   /* 25 - 24 */
#define SPI_MASK_RX_FIFO_WR_PTR             0x00030000   /* 17 - 16 */
#define SPI_MASK_TX_FIFO_RD_PTR             0x00000300   /* 9 - 8 */
#define SPI_MASK_TX_FIFO_WR_PTR             0x00000003   /* 1 - 0 */
#define SPI_POS_RX_FIFO_RD_PTR                      24
#define SPI_POS_RX_FIFO_WR_PTR                      16
#define SPI_POS_TX_FIFO_RD_PTR                       8
#define SPI_POS_TX_FIFO_WR_PTR                       0
// FSR
#define SPI_MASK_RX_FIFO_LEFT_CNT           0x03000000   /* 26 - 24 */
#define SPI_MASK_RX_FIFO_FILL_CNT           0x00030000   /* 18 - 16 */
#define SPI_MASK_TX_FIFO_LEFT_CNT           0x00000300   /* 10 - 8 */
#define SPI_MASK_TX_FIFO_FILL_CNT           0x00000003   /* 2 - 0 */
#define SPI_POS_RX_FIFO_LEFT_CNT                    24
#define SPI_POS_RX_FIFO_FILL_CNT                    16
#define SPI_POS_TX_FIFO_LEFT_CNT                     8
#define SPI_POS_TX_FIFO_FILL_CNT                     0
// SR
#define SPI_MASK_BUSY                       0x01000000   /* 24 */
#define SPI_MASK_SPI_IF                     0x00010000   /* 16 */
#define SPI_MASK_RX_FIFO_FULL               0x00000010   /* 4 */
#define SPI_MASK_TX_FIFO_EMPT               0x00000008   /* 3 */
#define SPI_MASK_RX_FIFO_OVR_IF             0x00000004   /* 2 */
#define SPI_MASK_RX_FIFO_NEMT_IF            0x00000002   /* 1 */
#define SPI_MASK_TX_FIFO_NFUL_IF            0x00000001   /* 0 */
/*@}*/ /* end of group QN_SPI */



/*------------- Universal Asynchronous Receiver Transmitter (UART) -----------*/
/** @defgroup QN_UART QN_UART
    @ingroup QN9020_Peripherals
    @brief QN9020 Universal Asynchronous Receiver/Transmitter Register Structure
  @{
*/
typedef struct
{
  __O   uint32_t  TXD;                      /*!< Offset: 0x000 TX Data Register    ( /W) */
  __I   uint32_t  RXD;                      /*!< Offset: 0x004 RX Data Register  (R/ ) */
  __IO  uint32_t  BAUD;                     /*!< Offset: 0x008 Baudrate Divider Register (R/W) */
  __IO  uint32_t  CR;                       /*!< Offset: 0x00C Control Register (R/W) */
  __IO  uint32_t  FLAG;                     /*!< Offset: 0x010 Status Register (R/W) */

} QN_UART_TypeDef;

/* UART DATA Register Mask Definitions */
// TXD
#define UART_MASK_TXD                       0x000000FF      /* 7 - 0 */
// RXD
#define UART_MASK_RXD                       0x000000FF      /* 7 - 0 */
// BAUD
#define UART_MASK_BAUD_DIV_INT              0x00FFFF00      /* 23 - 8 */
#define UART_MASK_BAUD_DIV_FRC              0x0000003F      /* 5 - 0 */
#define UART_POS_DIVIDER_INT                         8
#define UART_POS_DIVIDER_FRC                         0
// CR
#define UART_MASK_UART_IE                   0x00400000      /* 22 */
#define UART_MASK_BE_IE                     0x00200000      /* 21 */
#define UART_MASK_PE_IE                     0x00100000      /* 20 */
#define UART_MASK_FE_IE                     0x00080000      /* 19 */
#define UART_MASK_OE_IE                     0x00040000      /* 18 */
#define UART_MASK_TX_IE                     0x00020000      /* 17 */
#define UART_MASK_RX_IE                     0x00010000      /* 16 */
#define UART_MASK_OVS                       0x00000800      /* 11 */
#define UART_MASK_CTS_EN                    0x00000400      /* 10 */
#define UART_MASK_RTS_EN                    0x00000200      /* 9 */
#define UART_MASK_BREAK                     0x00000100      /* 8 */
#define UART_MASK_LEVEL_INV                 0x00000080      /* 7 */
#define UART_MASK_STP2_EN                   0x00000040      /* 6 */
#define UART_MASK_BIT_ORDER                 0x00000020      /* 5 */
#define UART_MASK_PEN                       0x00000010      /* 4 */
#define UART_MASK_EPS                       0x00000008      /* 3 */
#define UART_MASK_RX_EN                     0x00000004      /* 2 */
#define UART_MASK_TX_EN                     0x00000002      /* 1 */
#define UART_MASK_UART_EN                   0x00000001      /* 0 */
// FLAG
#define UART_MASK_RX_BUSY                   0x00000100      /* 8 */
#define UART_MASK_TX_BUSY                   0x00000080      /* 7 */
#define UART_MASK_UART_IF                   0x00000040      /* 6 */
#define UART_MASK_BE_IF                     0x00000020      /* 5 */
#define UART_MASK_PE_IF                     0x00000010      /* 4 */
#define UART_MASK_FE_IF                     0x00000008      /* 3 */
#define UART_MASK_OE_IF                     0x00000004      /* 2 */
#define UART_MASK_TX_IF                     0x00000002      /* 1 */
#define UART_MASK_RX_IF                     0x00000001      /* 0 */
/*@}*/ /* end of group QN_UART */


/*------------- Inter-Integrated Circuit (I2C) -------------------------------*/
/** @defgroup QN_I2C QN_I2C
    @ingroup QN9020_Peripherals
    @brief QN9020 I2C-Bus Interface Register Structure
  @{
*/
typedef struct
{
  __IO uint32_t CR;                         /*!< Offset: 0x000 I2C Control Set Register (R/W) */
  __I  uint32_t SR;                         /*!< Offset: 0x004 I2C Status Register (R/ ) */
  __O  uint32_t TXD;                        /*!< Offset: 0x008 I2C TX Data Register ( /W) */
  __I  uint32_t RXD;                        /*!< Offset: 0x00C I2C RX Data Register (R/ ) */
  __IO uint32_t INT;                        /*!< Offset: 0x010 I2C interrupt status Register (R/W) */
} QN_I2C_TypeDef;

/* I2C REGISTER MASK & POSITION */
// CR
#define I2C_MASK_SCL_RATIO                  0x3F000000      /* 29 - 24 */
#define I2C_MASK_SLAVE_ADDR                 0x007F0000      /* 22 - 16 */
#define I2C_MASK_SLAVE_EN                   0x00000200      /* 9 */
#define I2C_MASK_MASTR_EN                   0x00000100      /* 8 */
#define I2C_MASK_STP_INT_EN                 0x00000020      /* 5 */
#define I2C_MASK_SAM_INT_EN                 0x00000010      /* 4 */
#define I2C_MASK_GC_INT_EN                  0x00000008      /* 3 */
#define I2C_MASK_AL_INT_EN                  0x00000004      /* 2 */
#define I2C_MASK_RX_INT_EN                  0x00000002      /* 1 */
#define I2C_MASK_TX_INT_EN                  0x00000001      /* 0 */
#define I2C_POS_SCL_RATIO                           24
#define I2C_POS_SLAVE_ADDR                          16
#define I2C_POS_SLAVE_EN                             9
#define I2C_POS_MASTR_EN                             8
#define I2C_POS_STP_INT_EN                           5
#define I2C_POS_SAM_INT_EN                           4
#define I2C_POS_GC_INT_EN                            3
#define I2C_POS_AL_INT_EN                            2
#define I2C_POS_RX_INT_EN                            1
#define I2C_POS_TX_INT_EN                            0
// SR
#define I2C_MASK_BUSY                       0x00000002      /* 1 */
#define I2C_MASK_ACK_RECEIVED               0x00000001      /* 0 */
#define I2C_POS_BUSY                                 1
#define I2C_POS_ACK_RECEIVED                         0
// TXD
#define I2C_MASK_NACK_SEND                  0x00100000      /* 20 */
#define I2C_MASK_ACK_SEND                   0x00000000      /* 20 */
#define I2C_MASK_RD_EN                      0x00080000      /* 19 */
#define I2C_MASK_WR_EN                      0x00040000      /* 18 */
#define I2C_MASK_STOP                       0x00020000      /* 17 */
#define I2C_MASK_START                      0x00010000      /* 16 */
#define I2C_MASK_TXD                        0x000000FF      /* 7 - 0 */
#define I2C_POS_ACK_SEND                            20
#define I2C_POS_RD_EN                               19
#define I2C_POS_WR_EN                               18
#define I2C_POS_STOP                                17
#define I2C_POS_START                               16
// RXD
#define I2C_MASK_RXD                        0x000000FF      /* 7 - 0 */
// INT
#define I2C_MASK_STP_INT                    0x00000020      /* 5 */
#define I2C_MASK_SAM_INT                    0x00000010      /* 4 */
#define I2C_MASK_GC_INT                     0x00000008      /* 3 */
#define I2C_MASK_AL_INT                     0x00000004      /* 2 */
#define I2C_MASK_RX_INT                     0x00000002      /* 1 */
#define I2C_MASK_TX_INT                     0x00000001      /* 0 */
#define I2C_POS_STP_INT                              5
#define I2C_POS_SAM_INT                              4
#define I2C_POS_GC_INT                               3
#define I2C_POS_AL_INT                               2
#define I2C_POS_RX_INT                               1
#define I2C_POS_TX_INT                               0
/*@}*/ /* end of group QN_I2C */


/*------------- 32/16-bit Timer/Event Counter (TIMER) -----------------------------*/
/** @defgroup QN_TIMER QN_TIMER
    @ingroup QN9020_Peripherals
    @brief QN9020 32/16-bit Timer/Event Counter Register Structure
  @{
*/
typedef struct
{
  __IO uint32_t CR;                         /*!< Offset: 0x0000 Timer control Register             */
  __IO uint32_t IFR;                        /*!< Offset: 0x0004 Timer interrupt flag Register      */
  __IO uint32_t TOPR;                       /*!< Offset: 0x0008 Timer TOP count Register           */
  __I  uint32_t ICER;                       /*!< Offset: 0x000C Timer input capture event Register */
  __IO uint32_t CCR;                        /*!< Offset: 0x0010 Timer input capture/compare Register     */
  __I  uint32_t CNT;                        /*!< Offset: 0x0014 Timer counter current value Register     */
} QN_TIMER_TypeDef;


/* TIMER REGISTER MASK & POSITION */
// CR
#define TIMER_MASK_CSS                      0x30000000      /* 29 - 28 */
#define TIMER_MASK_PSCL                     0x03FF0000      /* 25 - 16 */
#define TIMER_MASK_PWM_OE                   0x00008000      /* 15 */
#define TIMER_MASK_POL                      0x00004000      /* 14 */
#define TIMER_MASK_ICNCE                    0x00002000      /* 13 */
#define TIMER_MASK_ICSS                     0x00001000      /* 12 */
#define TIMER_MASK_ICPS                     0x00000C00      /* 11 - 10 */
#define TIMER_MASK_ICES                     0x00000300      /* 9 - 8 */
#define TIMER_MASK_CMP_EN                   0x00000080      /* 7 */
#define TIMER_MASK_CHAIN_EN                 0x00000040      /* 6 */
#define TIMER_MASK_OMS                      0x00000030      /* 5 - 4 */
#define TIMER_MASK_ICIE                     0x00000008      /* 3 */
#define TIMER_MASK_OCIE                     0x00000004      /* 2 */
#define TIMER_MASK_TOVIE                    0x00000002      /* 1 */
#define TIMER_MASK_TEN                      0x00000001      /* 0 */
#define TIMER_POS_CSS                               28
#define TIMER_POS_PSCL                              16
#define TIMER_POS_ICPS                              10
#define TIMER_POS_ICES                               8
#define TIMER_POS_OMS                                4
// IFR
#define TIMER_MASK_ICF                      0x00000004      /* 2 */
#define TIMER_MASK_OCF                      0x00000002      /* 1 */
#define TIMER_MASK_TOVF                     0x00000001      /* 0 */
// TOPR
#define TIMER_MASK_TOPR                     0xFFFFFFFF      /* 31 - 0 */
// ICER
#define TIMER_MASK_ICER                     0x0000FFFF      /* 15 - 0 */
// CCR
#define TIMER_MASK_CCR                      0xFFFFFFFF      /* 31 - 0 */
// CNT
#define TIMER_MASK_CNT                      0xFFFFFFFF      /* 31 - 0 */
/*@}*/ /* end of group QN_TIMER */


/*-------------------- PWM (PWM) -----------------------------------------------*/
/** @defgroup QN_PWM QN_PWM
    @ingroup QN9020_Peripherals
    @brief QN9020 PWM Register Structure
  @{
*/
typedef struct
{
  __IO uint32_t CR;                         /*!< Offset: 0x0000 PWM control Register             */
  __IO uint32_t PSCL;                       /*!< Offset: 0x0004 PWM prescaler Register      */
  __IO uint32_t PCP;                        /*!< Offset: 0x0008 PWM period&compare Register           */
  __IO  uint32_t SR;                        /*!< Offset: 0x000C PWM status Register */
} QN_PWM_TypeDef;


/* PWM REGISTER MASK & POSITION */
// CR
#define PWM_MASK_CH1_POL                    0x00000400      /* 10 */
#define PWM_MASK_CH1_IE                     0x00000200      /* 9 */
#define PWM_MASK_CH1_EN                     0x00000100      /* 8 */
#define PWM_MASK_CH0_POL                    0x00000004      /* 2 */
#define PWM_MASK_CH0_IE                     0x00000002      /* 1 */
#define PWM_MASK_CH0_EN                     0x00000001      /* 0 */
// PSCL
#define PWM_MASK_CH1_PSCL                   0x03FF0000      /* 25 - 16 */
#define PWM_MASK_CH0_PSCL                   0x000003FF      /* 9 - 0 */
#define PWM_POS_CH1_PSCL                            16
// PCP
#define PWM_MASK_CH1_CMP                    0xFF000000      /* 31 - 24 */
#define PWM_MASK_CH1_PERIOD                 0x00FF0000      /* 23 - 16 */
#define PWM_MASK_CH0_CMP                    0x0000FF00      /* 15 - 8 */
#define PWM_MASK_CH0_PERIOD                 0x000000FF      /* 7 - 0 */
#define PWM_POS_CH1_CMP                             24
#define PWM_POS_CH1_PERIOD                          16
#define PWM_POS_CH0_CMP                              8
// SR
#define PWM_MASK_CH1_IF                     0x00000100      /* 8 */
#define PWM_MASK_CH0_IF                     0x00000001      /* 0 */
/*@}*/ /* end of group QN_PWM */


/*------------- Real Time Counter (RTC) -----------------------------*/
/** @defgroup QN_RTC QN_RTC
    @ingroup QN9020_Peripherals
    @brief QN9020 Real Time Counter Register Structure
  @{
*/
typedef struct
{
  __IO uint32_t CR;                         /*!< Offset: 0x0000 RTC control Register   */
  __IO uint32_t SR;                         /*!< Offset: 0x0004 RTC status Register   */
  __IO uint32_t SEC;                        /*!< Offset: 0x0008 RTC Second count Register   */
  __O  uint32_t CORR;                       /*!< Offset: 0x000C RTC correction Register  */
  __O  uint32_t CALIB;                      /*!< Offset: 0x0010 RTC calibration Register   */
  __I  uint32_t CNT;                        /*!< Offset: 0x0014 RTC counter current value Register   */
} QN_RTC_TypeDef;

/* RTC REGISTER MASK & POSITION */
// CR
#define RTC_MASK_CAL_EN                     0x00000100      /* 8 */
#define RTC_MASK_CAP_EDGE_SEL               0x00000040      /* 6 */
#define RTC_MASK_CAP_IE                     0x00000020      /* 5 */
#define RTC_MASK_CAP_EN                     0x00000010      /* 4 */
#define RTC_MASK_CFG                        0x00000004      /* 2 */
#define RTC_MASK_CORR_EN                    0x00000002      /* 1 */
#define RTC_MASK_SEC_IE                     0x00000001      /* 0 */
// SR
#define RTC_MASK_CALIB_SYNC_BUSY            0x00001000      /* 12 */
#define RTC_MASK_CORR_SYNC_BUSY             0x00000800      /* 11 */
#define RTC_MASK_SEC_SYNC_BUSY              0x00000400      /* 10 */
#define RTC_MASK_SR_SYNC_BUSY               0x00000200      /* 9 */
#define RTC_MASK_CR_SYNC_BUSY               0x00000100      /* 8 */
#define RTC_MASK_CAP_IF                     0x00000010      /* 4 */
#define RTC_MASK_SEC_IF                     0x00000001      /* 0 */
// SEC
#define RTC_MASK_SEC_VAL                    0xFFFFFFFF      /* 31 - 0 */
// COEE
#define RTC_MASK_SEC_CORR                   0xFFFF8000      /* 31 - 15 */
#define RTC_MASK_CNT_CORR                   0x00007FFF      /* 14 - 0 */
#define RTC_POS_SEC_CORR                            15
// CALIB
#define RTC_MASK_CAL_DIR                    0x00010000      /* 16 */
#define RTC_MASK_PPM_VAL                    0x0000FFFF      /* 15 - 0 */
// CNT
#define RTC_MASK_CNT_VAL                    0x00007FFF      /* 14 - 0 */
/*@}*/ /* end of group QN_RTC */


/*------------- DMA controller (DMA) -----------------------------*/
/** @defgroup QN_DMA QN_DMA
    @ingroup QN9020_Peripherals
    @brief QN9020 DMA Controller Register Structure
  @{
*/
typedef struct
{
  __IO uint32_t SRC;                         /*!< Offset: 0x0000 DMA source data start address Register   */
  __IO uint32_t DST;                         /*!< Offset: 0x0004 DMA destination data start address Register   */
  __IO uint32_t CR;                          /*!< Offset: 0x0008 DMA control Register   */
  __O  uint32_t ABORT;                       /*!< Offset: 0x000C DMA abort Register  */
  __IO uint32_t SR;                          /*!< Offset: 0x0010 DMA status Register  */
} QN_DMA_TypeDef;

/* DMA REGISTER MASK & POSITION */
// CR
#define DMA_MASK_DONE_IE                    0x80000000      /* 31 */
#define DMA_MASK_ERROR_IE                   0x40000000      /* 30 */
#define DMA_MASK_INT_EN                     0x20000000      /* 29 */
#define DMA_MASK_TRANS_SIZE                 0x07FF0000      /* 26 - 16 */
#define DMA_MASK_DSR_MUX                    0x0000F000      /* 15 - 12 */
#define DMA_MASK_SRC_MUX                    0x00000F00      /* 11 - 8 */
#define DMA_MASK_TRANS_MODE                 0x000000C0      /* 7 - 6 */
#define DMA_MASK_DST_REQ_EN                 0x00000020      /* 5 */
#define DMA_MASK_SRC_REQ_EN                 0x00000010      /* 4 */
#define DMA_MASK_DST_ADDR_FIX               0x00000008      /* 3 */
#define DMA_MASK_SRC_ADDR_FIX               0x00000004      /* 2 */
#define DMA_MASK_SRC_UDLEN                  0x00000002      /* 1 */
#define DMA_MASK_START                      0x00000001      /* 0 */
#define DMA_POS_TRANS_SIZE                          16
#define DMA_POS_DST_MUX                             12
#define DMA_POS_SRC_MUX                              8
#define DMA_POS_TRANS_MODE                           6
// ABORT
#define DMA_MASK_ABORT                      0x00000001      /* 0 */
// SR
#define DMA_MASK_BUSY                       0x00000004      /* 2 */
#define DMA_MASK_ERRO                       0x00000002      /* 1 */
#define DMA_MASK_DONE                       0x00000001      /* 0 */
/*@}*/ /* end of group QN_DMA */


/*------------- Serial Flash Controller (SF_CTRL) -----------------------------*/
/** @defgroup QN_SF_CTRL QN_SF_CTRL
    @ingroup QN9020_Peripherals
    @brief QN9020 Serial Flash Controller Register Structure
  @{
*/
typedef struct
{
  __I  uint32_t FLASH_ID;                   /*!< Offset: 0xFFFFFE8 Serial FLASH ID Register   */
  __I  uint32_t FLASH_SR;                   /*!< Offset: 0xFFFFFEC Serial FLASH status Register   */
  __O  uint32_t CMD1ADDR3;                  /*!< Offset: 0xFFFFFF0 SF_CTRL command with address Register   */
  __O  uint32_t CMD1;                       /*!< Offset: 0xFFFFFF4 SF_CTRL command Register   */
  __IO uint32_t DATA_LEN;                   /*!< Offset: 0xFFFFFF8 SF_CTRL flash data length Register   */
  __IO uint32_t CTRL_STAT;                  /*!< Offset: 0xFFFFFFC SF_CTRL control and status Register   */
} QN_SF_CTRL_TypeDef;

/* SF_CTRL REGISTER MASK & POSITION */
// FLASH_ID
#define SF_CTRL_MASK_FLASH_ID               0xFFFFFFFF      /* 31 - 0 */
// FLASH_SR
#define SF_CTRL_MASK_STAT                   0x000000FF      /* 7 - 0 */
// CMD1ADDR3
#define SF_CTRL_MASK_CMDADDR2               0xFF000000      /* 31 - 24 */
#define SF_CTRL_MASK_CMDADDR1               0x00FF0000      /* 23 - 16 */
#define SF_CTRL_MASK_CMDADDR0               0x0000FF00      /* 15 - 8 */
#define SF_CTRL_POS_CMDADDR2                        24
#define SF_CTRL_POS_CMDADDR1                        16
#define SF_CTRL_POS_CMDADDR0                         8
// CMD1
#define SF_CTRL_MASK_CMD                    0x000000FF      /* 7 - 0 */
// DATA_LEN
#define SF_CTRL_MASK_DATA_LEN               0x0001FFFF      /* 16 - 0 */
// CTRL_STAT
#define SF_CTRL_MASK_BOOT_DONE              0x80000000      /* 31 */
#define SF_CTRL_MASK_RD_STAT_CMD            0x0000FF00      /* 15 - 8 */
#define SF_CTRL_MASK_CLK_DIV                0x0000003F      /* 5 - 0 */
#define SF_CTRL_POS_RD_STAT_CMD                      8
/*@}*/ /* end of group QN_SF_CTRL */


/*------------- BLE Data Path (BLE_DP) -----------------------------*/
/** @defgroup QN_DP QN_DP
    @ingroup QN9020_Peripherals
    @brief QN9020 BLE Data Path Register Structure
  @{
*/
typedef struct
{
  __IO uint32_t REG_00;                     /*!< Offset: 0x0000 BLE DP Register 0x00   */
  __IO uint32_t REG_04;                     /*!< Offset: 0x0004 BLE DP Register 0x04   */
  __IO uint32_t REG_08;                     /*!< Offset: 0x0008 BLE DP Register 0x08   */
  __IO uint32_t REG_0C;                     /*!< Offset: 0x000C BLE DP Register 0x0C   */
  __IO uint32_t REG_10;                     /*!< Offset: 0x0010 BLE DP Register 0x10   */
  __IO uint32_t REG_14;                     /*!< Offset: 0x0014 BLE DP Register 0x14   */
  __IO uint32_t REG_18;                     /*!< Offset: 0x0018 BLE DP Register 0x18   */
  __IO uint32_t REG_1C;                     /*!< Offset: 0x001C BLE DP Register 0x1C   */
  __IO uint32_t REG_20;                     /*!< Offset: 0x0020 BLE DP Register 0x20   */
  __IO uint32_t REG_24;                     /*!< Offset: 0x0024 BLE DP Register 0x24   */
  __IO uint32_t REG_28;                     /*!< Offset: 0x0028 BLE DP Register 0x28   */
  __IO uint32_t REG_2C;                     /*!< Offset: 0x002C BLE DP Register 0x2C   */
  __IO uint32_t REG_30;                     /*!< Offset: 0x0030 BLE DP Register 0x30   */
  __IO uint32_t REG_34;                     /*!< Offset: 0x0034 BLE DP Register 0x34   */
  __IO uint32_t REG_38;                     /*!< Offset: 0x0038 BLE DP Register 0x38   */
  __IO uint32_t REG_3C;                     /*!< Offset: 0x003C BLE DP Register 0x3C   */
  __IO uint32_t REG_40;                     /*!< Offset: 0x0040 BLE DP Register 0x40   */
  __IO uint32_t REG_44;                     /*!< Offset: 0x0044 BLE DP Register 0x44   */
  __IO uint32_t REG_48;                     /*!< Offset: 0x0048 BLE DP Register 0x48   */
  __IO uint32_t REG_4C;                     /*!< Offset: 0x004C BLE DP Register 0x4C   */
  __IO uint32_t REG_50;                     /*!< Offset: 0x0050 BLE DP Register 0x50   */
  __IO uint32_t REG_54;                     /*!< Offset: 0x0054 BLE DP Register 0x54   */
  __IO uint32_t REG_58;                     /*!< Offset: 0x0058 BLE DP Register 0x58   */
  __IO uint32_t REG_5C;                     /*!< Offset: 0x005C BLE DP Register 0x5C   */
  __IO uint32_t REG_60;                     /*!< Offset: 0x0060 BLE DP Register 0x60   */
  __IO uint32_t REG_64;                     /*!< Offset: 0x0064 BLE DP Register 0x64   */
  __IO uint32_t REG_68;                     /*!< Offset: 0x0068 BLE DP Register 0x68   */
  __IO uint32_t REG_6C;                     /*!< Offset: 0x006C BLE DP Register 0x6C   */
  __IO uint32_t REG_70;                     /*!< Offset: 0x0070 BLE DP Register 0x70   */
  __IO uint32_t REG_74;                     /*!< Offset: 0x0074 BLE DP Register 0x74   */
  __IO uint32_t REG_78;                     /*!< Offset: 0x0078 BLE DP Register 0x78   */
  __IO uint32_t REG_7C;                     /*!< Offset: 0x007C BLE DP Register 0x7C   */
} QN_DP_TypeDef;

/* BLE_DP REGISTER MASK & POSITION */
// REG_00
#define DP_MASK_DET_MODE                    0x80000000      /* 31 */
#define DP_MASK_ANT_DATA_START              0x40000000      /* 30 */
#define DP_MASK_RX_MODE                     0x30000000      /* 29 - 28 */
#define DP_MASK_TX_REQ                      0x08000000      /* 27 */
#define DP_MASK_RX_REQ                      0x04000000      /* 26 */
#define DP_MASK_TX_EN_SEL                   0x02000000      /* 25 */
#define DP_MASK_RX_EN_SEL                   0x01000000      /* 24 */
#define DP_MASK_H_IDX                       0x00FF0000      /* 23 -16 */
#define DP_MASK_PDU_LEN_SEL                 0x00008000      /* 15 */
#define DP_MASK_AA_SEL                      0x00004000      /* 14 */
#define DP_MASK_RX_PDU_LEN_IN               0x00003FFF      /* 13 - 0 */
#define DP_POS_DET_MODE                             31
#define DP_POS_ANT_DATA_START                       30
#define DP_POS_RX_MODE                              28
#define DP_POS_TX_REQ                               27
#define DP_POS_RX_REQ                               26
#define DP_POS_TX_EN_SEL                            25
#define DP_POS_RX_EN_SEL                            24
#define DP_POS_RX_H_IDX                             16
#define DP_POS_PDU_LEN_SEL                          15
#define DP_POS_AA_SEL                               14
#define DP_POS_RX_PDU_LEN                            0
// REG_04
#define DP_MASK_TX_POWER_DN_TIME            0xF8000000      /* 31 - 27 */
#define DP_MASK_PROP_DIRECTION_MODE         0x01000000      /* 24 */
#define DP_MASK_PROP_DIRECTION_RATE         0x00C00000      /* 23 - 22 */
#define DP_MASK_PROP_DATA_RATE              0x00300000      /* 21 - 20 */
#define DP_MASK_PROP_PRE_NUM                0x00070000      /* 18 - 16 */
#define DP_MASK_PROP_AA_NUM                 0x00003000      /* 13 - 12 */
#define DP_MASK_PROP_CRC_NUM                0x00000300      /* 9 - 8 */
#define DP_MASK_PROP_AA_ADDR_IN             0x000000FF      /* 7 - 0 */
#define DP_POS_TX_POWER_DN_TIME                     27
#define DP_POS_PROP_DIRECTION_MODE                  24
#define DP_POS_PROP_DIRECTION_RATE                  22
#define DP_POS_PROP_DATA_RATE                       20
#define DP_POS_PROP_PRE_NUM                         16
#define DP_POS_PROP_AA_NUM                          12
#define DP_POS_PROP_CRC_NUM                          8
#define DP_POS_PROP_AA_ADDR_IN                       0
// REG_08
#define DP_MASK_AA_ADDR_IN                  0xFFFFFFFF      /* 31 - 0 */
#define DP_POS_AA_ADDR_IN                            0
// REG_0C
#define DP_MASK_PROP_PREAMBLE               0xFF000000
#define DP_MASK_TEST_PATTERN_EN             0x00100000
#define DP_MASK_PATTERN_SEL                 0x000F0000
#define DP_MASK_PDU_DATA0                   0x0000FFFF
// REG_10
#define DP_MASK_PDU_DATA1                   0xFFFFFFFF      /* 31 - 0 */
// REG_14
#define DP_MASK_PDU_DATA2                   0xFFFFFFFF      /* 31 - 0 */
// REG_18
#define DP_MASK_PDU_DATA3                   0xFFFFFFFF      /* 31 - 0 */
// REG_1C
#define DP_MASK_PDU_DATA4                   0xFFFFFFFF      /* 31 - 0 */
// REG_20
#define DP_MASK_PDU_DATA5                   0xFFFFFFFF      /* 31 - 0 */
// REG_24
#define DP_MASK_PDU_DATA6                   0xFFFFFFFF      /* 31 - 0 */
// REG_28
#define DP_MASK_PDU_DATA7                   0xFFFFFFFF      /* 31 - 0 */
// REG_2C
#define DP_MASK_CRC_SEED_WEN                0x01000000
#define DP_MASK_CRC_SEED_IN                 0x00FFFFFF
// REG_30
#define DP_MASK_RX_EN_MODE                  0x80000000      /* 31 */
#define DP_MASK_TX_EN_MODE                  0x40000000      /* 30 */
#define DP_MASK_ADC_IN_FLIP                 0x20000000      /* 29 */
#define DP_MASK_CFO_INI_EN                  0x10000000      /* 28 */
#define DP_MASK_CFO_TRACK_EN                0x08000000      /* 27 */
#define DP_MASK_HP_CFO_EN                   0x04000000      /* 26 */
#define DP_MASK_FAGC_WEN                    0x02000000      /* 25 */
#define DP_MASK_FAGC_WIN_LEN                0x01000000      /* 24 */
#define DP_MASK_RESAMPLER_BP                0x00800000      /* 23 */
#define DP_MASK_RESAMPLER_TAP               0x00600000      /* 22 - 21 */
#define DP_MASK_XCORR_WIN_AUTO_EN           0x00100000      /* 20 */
#define DP_MASK_XCORR_AA_LEN                0x000C0000      /* 19 - 18 */
#define DP_MASK_XCORR_FULLWIN_EN            0x00020000      /* 17 */
#define DP_MASK_XCORR_FILT_EN               0x00010000      /* 16 */
#define DP_MASK_RFAGC_TRACK_DLY             0x00008000      /* 15 */
#define DP_MASK_TRACK_LEN                   0x00007000      /* 14 - 12 */
#define DP_MASK_FIX_DELAY_EN                0x00000800      /* 11 */
#define DP_MASK_DC_AVE_EN                   0x00000400      /* 10 */
#define DP_MASK_FR_OFFSET_EN                0x00000200      /* 9 */
#define DP_MASK_LP_ADJ_MODE                 0x00000100      /* 8 */
#define DP_MASK_DOUT_ADJ_DIS                0x00000080      /* 7 */
#define DP_MASK_LP_SNR_LEN_AUTO             0x00000040      /* 6 */
#define DP_MASK_CHF_COEF_IDX                0x00000030      /* 5 - 4 */
#define DP_MASK_CHF_COEF_WEN                0x00000008      /* 3 */
#define DP_MASK_DP_INT_SEL                  0x00000007      /* 2 - 0 */
#define DP_POS_RX_EN_MODE                           31
#define DP_POS_TX_EN_MODE                           30
#define DP_POS_ADC_IN_FLIP                          29
#define DP_POS_CFO_INI_EN                           28
#define DP_POS_CFO_TRACK_EN                         27
#define DP_POS_HP_CFO_EN                            26
#define DP_POS_FAGC_WEN                             25
#define DP_POS_FAGC_WIN_LEN                         24
#define DP_POS_RESAMPLER_BP                         23
#define DP_POS_RESAMPLER_TAP                        21
#define DP_POS_XCORR_WIN_AUTO_EN                    20
#define DP_POS_XCORR_AA_LEN                         18
#define DP_POS_XCORR_FULLWIN_EN                     17
#define DP_POS_FILT_EN                              16
#define DP_POS_TRACK_DLY                            15
#define DP_POS_TRACK_LEN                            12
#define DP_POS_FIX_DELAY_EN                         11
#define DP_POS_DC_AVE_EN                            10
#define DP_POS_FR_OFFSET_EN                          9
#define DP_POS_LP_ADJ_MODE                           8
#define DP_POS_DOUT_ADJ_DIS                          7
#define DP_POS_LP_SNR_LEN_AUTO                       6
#define DP_POS_CHF_COEF_IDX                          4
#define DP_POS_CHF_COEF_WEN                          3
#define DP_POS_DP_INT_SEL                            0
// REG_34
#define DP_MASK_CLK_RFE_GATE_DIS            0x00100000      /* 20 */
#define DP_MASK_CLK_HPDET_GATE_DIS          0x00080000      /* 19 */
#define DP_MASK_CLK_LPDET_GATE_DIS          0x00040000      /* 18 */
#define DP_MASK_CLK_RX_GATE_DIS             0x00020000      /* 17 */
#define DP_MASK_CLK_BUST_GATE_DIS           0x00010000      /* 16 */
#define DP_MASK_BUF_FULL_OFFRF_DIS          0x00008000      /* 15 */
#define DP_MASK_CLK_TX_GATE_DIS             0x00004000      /* 14 */
#define DP_MASK_TIF_EN                      0x00001000      /* 12 */
#define DP_MASK_TIF_CLK_SEL                 0x00000300      /* 9 - 8 */
#define DP_MASK_TIF_SEL                     0x000000FF      /* 7 - 0 */
#define DP_POS_TIF_CLK_SEL                           8
// REG_38
#define DP_MASK_TX_BUSY                     0x08000000      /* 27 */
#define DP_MASK_CNR_VLD                     0x04000000      /* 26 */
#define DP_MASK_SNR_VLD                     0x02000000      /* 25 */
#define DP_MASK_AGC_RSSI                    0x01FF0000      /* 24 - 16 */
#define DP_MASK_CNR_EST                     0x00003F00      /* 13 - 8 */
#define DP_MASK_SNR_EST                     0x000000FF      /* 7 - 0 */
#define DP_POS_AGC_RSSI                             16
#define DP_POS_CNR_EST                               8
// REG_3C
#define DP_MASK_DP_STATUS_VLD_0             0x80000000      /* 31 */
#define DP_MASK_BURST_DET                   0x40000000      /* 30 */
#define DP_MASK_CRC_ERROR                   0x20000000      /* 29 */
#define DP_MASK_AA_ERR_NUM                  0x003F0000      /* 21 - 16 */
#define DP_MASK_VALID_PCK_NUM               0x0000FFFF      /* 15 - 0 */
#define DP_POS_DP_STATUS_VLD_0                      31
#define DP_POS_BURST_DET                            30
#define DP_POS_CRC_ERR                              29
#define DP_POS_AA_ERR_NUM                           16
// REG_40
#define DP_MASK_CFO_EST_FD                  0x07FF0000      /* 26 - 16 */
#define DP_MASK_FD_CFO_TRACK                0x000007FF      /* 10 - 0 */
#define DP_POS_CFO_EST_FD                           16
// REG_44
#define DP_MASK_HP_CFO_VLD                  0x80000000      /* 31 */
#define DP_MASK_HP_CFO                      0x0FFF0000      /* 27 - 16 */
#define DP_MASK_RESAMPLER_PH                0x000003FF      /* 9 - 0 */
#define DP_POS_HP_CFO                               16
// REG_48
#define DP_MASK_DCNOTCH_GIN                 0x00030000      /* 17 - 16 */
#define DP_MASK_CFO_COMP                    0x00007FFF      /* 14 - 0 */
#define DP_POS_DCNOTCH_GIN                          16
// REG_4C
#define DP_MASK_FREQ_TRADE_EN               0x10000000      /* 28 */
#define DP_MASK_CORDIC_MIN_VIN_TH           0x0F000000      /* 27 - 24 */
#define DP_MASK_FAGC_REF                    0x00FF0000      /* 23 - 16 */
#define DP_MASK_FAGC_GAIN                   0x000007FF      /* 10 - 0 */
#define DP_POS_CORDIC_MIN_VIN_TH                    24
#define DP_POS_FAGC_REF                             16
// REG_50
#define DP_MASK_XCORR_PAR_TH0               0x3F000000      /* 29 - 24 */
#define DP_MASK_XCORR_PAR_TH1               0x003F0000      /* 21 - 16 */
#define DP_MASK_XCORR_PAR_TH2               0x00003F00      /* 13 - 8 */
#define DP_MASK_XCORR_PAR_TH3               0x0000003F      /* 5 - 0 */
#define DP_POS_XCORR_PAR_TH0                        24
#define DP_POS_XCORR_PAR_TH1                        16
#define DP_POS_XCORR_PAR_TH2                         8
// REG_54
#define DP_MASK_XCORR_POW_TH0               0x3F000000      /* 29 - 24 */
#define DP_MASK_XCORR_POW_TH1               0x003F0000      /* 21 - 16 */
#define DP_MASK_XCORR_POW_TH2               0x00003F00      /* 12 - 8 */
#define DP_MASK_XCORR_POW_TH3               0x0000003F      /* 5 - 0 */
#define DP_POS_XCORR_POW_TH0                        24
#define DP_POS_XCORR_POW_TH1                        16
#define DP_POS_XCORR_POW_TH2                         8
// REG_58
#define DP_MASK_XCORR_RSSI_TH0              0xF0000000      /* 31 - 28 */
#define DP_MASK_XCORR_RSSI_TH1              0x0F000000      /* 27 - 24 */
#define DP_MASK_XCORR_RSSI_TH2              0x00F00000      /* 23 - 20 */
#define DP_MASK_XCORR_RSSI_TH3              0x000F0000      /* 19 - 16 */
#define DP_MASK_TRIG_XCORR_CNT              0x0000F000      /* 15 - 12 */
#define DP_MASK_CNT_SETTLE_IDX              0x00000700      /* 10 - 8 */
#define DP_MASK_SYNC_DIN_SAT_EN             0x00000080      /* 7 */
#define DP_MASK_SYNC_DIN_SAT_VALUE          0x00000070      /* 6 - 4 */
#define DP_MASK_GAIN_TED                    0x00000003      /* 1 - 0 */
#define DP_POS_XCORR_RSSI_TH0                       28
#define DP_POS_XCORR_RSSI_TH1                       24
#define DP_POS_XCORR_RSSI_TH2                       20
#define DP_POS_XCORR_RSSI_TH3                       16
#define DP_POS_TRIG_XCORR_CNT                       12
#define DP_POS_CNT_SETTLE_IDX                        8
// REG_5C
#define DP_MASK_CFO_FR_IDX                  0x30000000      /* 29 - 28 */
#define DP_MASK_DET_FR_IDX                  0x03000000      /* 25 - 24 */
#define DP_MASK_H_REF_GAIN                  0x003F0000      /* 21 - 16 */
#define DP_MASK_HP_HIDX_GAIN                0x0000FF00      /* 15 - 8 */
#define DP_MASK_HP_TRAIN_SIZ                0x0000001F      /* 4 - 0 */
#define DP_POS_CFO_FR_IDX                           28
#define DP_POS_DET_FR_IDX                           24
#define DP_POS_H_REF_GAIN                           16
#define DP_POS_HP_HIDX_GAIN                          8
// REG_60
#define DP_MASK_WMF2_DSAMP_IDX              0xE0000000      /* 31 - 29 */
#define DP_MASK_HP_BMC_CZ1                  0x003F0000      /* 21 - 16 */
#define DP_MASK_HP_BMC_P_TRAIN              0x00003F00      /* 13 - 8 */
#define DP_MASK_HP_BMC_P_TRACK              0x0000003F      /* 5 - 0 */
#define DP_POS_WMF2_DSAMP_IDX                       29
#define DP_POS_HP_BMC_CZ1                           16
#define DP_POS_HP_BMC_P_TRAIN                        8
// REG_64
#define DP_MASK_HP_BMC_Q_TRAIN              0xFF000000      /* 31 - 24 */
#define DP_MASK_HP_BMC_Q_TRACK              0x00FF0000      /* 23 - 16 */
#define DP_MASK_SNR_EST_EN                  0x00001000      /* 12 */
#define DP_MASK_SNR_EST_LEN                 0x00000300      /* 9 - 8 */
#define DP_MASK_SNR_EST_REF                 0x000000FF      /* 7 - 0 */
#define DP_POS_HP_BMC_Q_TRAIN                       24
#define DP_POS_HP_BMC_Q_TRACK                       16
#define DP_POS_SNR_EST_LEN                           8
// REG_68
#define DP_MASK_PKT_OFFSET_COM              0x1FF00000      /* 28 - 20 */
#define DP_MASK_NIDX                        0x0000F000      /* 15 - 12 */
#define DP_MASK_MAX_XCORR                   0x000003FF      /* 9 - 0 */
#define DP_POS_PKT_OFFSET_COM                       20
#define DP_POS_NIDX                                 12
// REG_6C
#define DP_MASK_MAX_PAR_XCORR               0x03FF0000      /* 25 - 16 */
#define DP_MASK_MAX_PAR_SPWR                0x000003FF      /* 9 - 0 */
#define DP_POS_MAX_PAR_XCORR                        16
// REG_70
#define DP_MASK_RST_ADC_REG                 0x80000000      /* 31 */
#define DP_MASK_ADC_RST_EN                  0x40000000      /* 30 */
#define DP_MASK_RFAGC_LNA_12DB_RES          0x20000000      /* 29 */
#define DP_MASK_RFAGC_GF_9DB_RES            0x10000000      /* 28 */
#define DP_MASK_PD1_CLR_EN                  0x08000000      /* 27 */
#define DP_MASK_PD2_CLR_EN                  0x04000000      /* 26 */
#define DP_MASK_PD3_CLR_EN                  0x02000000      /* 25 */
#define DP_MASK_RFAGC_ALWAYS_ON             0x01000000      /* 24 */
#define DP_MASK_RFAGC_GLNA_BACKOFF_EN       0x00800000      /* 23 */
#define DP_MASK_RST_PLS_LEN                 0x00400000      /* 22 */
#define DP_MASK_RFAGC_PAREA5_IGNORE_MODE    0x00200000      /* 21 */
#define DP_MASK_RFAGC_FSYNC_DET_DIS         0x00100000      /* 20 */
#define DP_MASK_RFAGC_MM0_PDCLR_EN          0x00080000      /* 19 */
#define DP_MASK_RFAGC_TG_DOWN_MODE1         0x00040000      /* 18 */
#define DP_MASK_RFAGC_ADCRST_MODE           0x00020000      /* 17 */
#define DP_MASK_RFAGC_GAIN_SEL              0x00010000      /* 16 */
#define DP_MASK_RFAGC_DIRECTION_FREEZE      0x00008000      /* 15 */
#define DP_MASK_RFAGC_ADCRST_DLY            0x00003000      /* 13 - 12 */
#define DP_POS_RFAGC_ADCRST_DLY                     12
// REG_74
#define DP_MASK_PD1_SETL_CNT_TH             0x1F000000      /* 28 - 24 */
#define DP_MASK_PD2_SETL_CNT_TH             0x001F0000      /* 20 - 16 */
#define DP_MASK_PD3_SETL_CNT_TH             0x00001F00      /* 12 - 8 */
#define DP_MASK_EN_DET1_CNT_TH              0x0000001F      /* 4 - 0 */
#define DP_POS_PD1_SETL_CNT_TH                      24
#define DP_POS_PD2_SETL_CNT_TH                      16
#define DP_POS_PD3_SETL_CNT_TH                       8
// REG_78      /* 3 */
#define DP_MASK_EN_DET2_CNT_TH              0x1F000000      /* 28 - 24 */
#define DP_MASK_EN_DET3_CNT_TH              0x001F0000      /* 20 - 16 */
#define DP_MASK_EN_DET0_CNT_TH1             0x00001F00      /* 12 - 8 */
#define DP_MASK_EN_DET0_CNT_TH2             0x0000001F      /* 4 - 0 */
// REG_7C
#define DP_MASK_LNA_GAIN1                   0xC0000000      /* 31 - 30 */
#define DP_MASK_LNA_GAIN2                   0x30000000      /* 29 - 28 */
#define DP_MASK_PPF_GAIN                    0x0F000000      /* 27 - 24 */
#define DP_MASK_RF_GAIN                     0x007F0000      /* 22 - 16 */
#define DP_MASK_RFAGC_TRIGGER_O             0x00008000      /* 15 */
#define DP_POS_LNA_GAIN1                            30
#define DP_POS_LNA_GAIN2                            28
#define DP_POS_PPF_GAIN                             24
#define DP_POS_RF_GAIN                              16
/*@}*/ /* end of group QN_DP */


/*------------ ADC -------------------------------------*/
/** @defgroup QN_ADC QN_ADC
    @ingroup QN9020_Peripherals
    @brief QN9020 A/D Controller Register Structure
  @{
*/
typedef struct
{
    __IO uint32_t ADC0;                     /*!< Offset: 0x0000 ADC Control Register0  */
    __IO uint32_t ADC1;                     /*!< Offset: 0x0004 ADC Control Register1  */
    __IO uint32_t ADC2;                     /*!< Offset: 0x0008 ADC Control Register2  */
    __IO uint32_t SR;                       /*!< Offset: 0x000C ADC Status Register    */
    __I  uint32_t DATA;                     /*!< Offset: 0x0010 ADC Data Register      */
} QN_ADC_TypeDef;

/* CALIBRATION REGISTER MASK & POSITION */
// ADC0
#define ADC_MASK_SCAN_CH_END                0xF0000000      /* 31 - 28 */
#define ADC_MASK_SCAN_CH_START              0x0F000000      /* 27 - 24 */
#define ADC_MASK_SCAN_INTV                  0x00060000      /* 18 - 17 */
#define ADC_MASK_SCAN_EN                    0x00010000      /* 16 */
#define ADC_MASK_SINGLE_EN                  0x00008000      /* 15 */
#define ADC_MASK_START_SEL                  0x00007000      /* 14 - 12 */
#define ADC_MASK_ADC_EN                     0x00000200      /* 9 */
#define ADC_MASK_SFT_START                  0x00000100      /* 8 */
#define ADC_MASK_POW_UP_DLY                 0x000000FC      /* 7 -  2 */
#define ADC_MASK_POW_DN_CTRL                0x00000002      /* 1 */
#define ADC_POS_SCAN_CH_END                         28
#define ADC_POS_SCAN_CH_START                       24
#define ADC_POS_SCAN_INTV                           17
#define ADC_POS_SCAN_EN                             16
#define ADC_POS_SINGLE_EN                           15
#define ADC_POS_START_SEL                           12
#define ADC_POS_ADC_EN                               9
#define ADC_POS_SFT_START                            8
#define ADC_POS_POW_UP_DLY                           2
#define ADC_POS_POW_DN_CTRL                          1
// ADC1
#define ADC_MASK_INT_MASK                   0x80000000      /* 31 */
#define ADC_MASK_DAT_RDY_IE                 0x40000000      /* 30 */
#define ADC_MASK_WCMP_IE                    0x20000000      /* 29 */
#define ADC_MASK_FIFO_OF_IE                 0x10000000      /* 28 */
#define ADC_MASK_TIF_EN                     0x08000000      /* 27 */
#define ADC_MASK_TIF_SEL                    0x07000000      /* 26 - 24 */
#define ADC_MASK_VREF_SEL                   0x00300000      /* 21 - 20 */
#define ADC_MASK_INBUF_BP                   0x00080000      /* 19 */
#define ADC_MASK_BUF_GAIN_BP                0x00040000      /* 18 */
#define ADC_MASK_BUF_GAIN                   0x00030000      /* 17 - 16 */
#define ADC_MASK_BUF_BM_DRV                 0x0000C000      /* 15 - 14 */
#define ADC_MASK_BUF_BM_GAIN                0x00003000      /* 13 - 12 */
#define ADC_MASK_BUF_IN_P                   0x00000C00      /* 11 - 10 */
#define ADC_MASK_BUF_IN_N                   0x00000300      /* 9 - 8 */
#define ADC_MASK_RES_SEL                    0x000000C0      /* 7 - 6 */
#define ADC_MASK_WCMP_SEL                   0x00000020      /* 5 */
#define ADC_MASK_WCMP_EN                    0x00000010      /* 4 */
#define ADC_MASK_DECI_DIV                   0x00000006      /* 2 -  1 */
#define ADC_MASK_DECI_EN                    0x00000001      /* 0 */
#define ADC_POS_TIF_SEL                             24
#define ADC_POS_VREF_SEL                            20
#define ADC_POS_BUF_GAIN                            16
#define ADC_POS_BUF_BM_DRV                          14
#define ADC_POS_BUF_BM_GAIN                         12
#define ADC_POS_BUF_IN_P                            10
#define ADC_POS_BUF_IN_N                             8
#define ADC_POS_RES_SEL                              6
#define ADC_POS_DECI_DIV                             1
// ADC2
#define ADC_MASK_WCMP_TH_HI                 0xFFFF0000      /* 31 - 16 */
#define ADC_MASK_WCMP_TH_LO                 0x0000FFFF      /* 15 - 0 */
#define ADC_POS_WCMP_TH_HI                          16
#define ADC_POS_WCMP_TH_LO                           0
// SR
#define ADC_MASK_FIFO_OF_IF                 0x00000004      /* 2 */
#define ADC_MASK_WCMP_IF                    0x00000002      /* 1 */
#define ADC_MASK_DAT_RDY_IF                 0x00000001      /* 0 */
// DATA
#define ADC_MASK_ADC_DATA                   0x0000FFFF      /* 15 - 0 */
/*@}*/ /* end of group QN_ADC */

/*------------ CALIBRATION -------------------------------------*/
/** @defgroup QN_CALIB QN_CALIB
    @ingroup QN9020_Peripherals
    @brief QN9020 Calibration Register Structure
  @{
*/
typedef struct
{
    __IO uint32_t CAL0;                     /*!< Offset: 0x0000 Calibration Control Register0  */
    __IO uint32_t CAL1;                     /*!< Offset: 0x0004 Calibration Control Register1  */
    __IO uint32_t CAL2;                     /*!< Offset: 0x0008 Calibration Control Register2  */
    __IO uint32_t CAL3;                     /*!< Offset: 0x000C Calibration Control Register3  */
    __IO uint32_t CAL4;                     /*!< Offset: 0x0010 Calibration Control Register4  */
    __IO uint32_t CR;                       /*!< Offset: 0x0014 Calibration Control Register   */
    __IO uint32_t SR;                       /*!< Offset: 0x0018 Calibration Status Register    */
} QN_CALIB_TypeDef;

/* CALIBRATION REGISTER MASK & POSITION */
// CAL0
#define CALIB_MASK_SEQ_CAL_REQ              0x80000000      /* 31 */
#define CALIB_MASK_CH_CHG_CAL_EN            0x20000000      /* 29 */
#define CALIB_MASK_CH_CHG_CAL_REQ           0x10000000      /* 28 */
#define CALIB_MASK_LO_CAL_SKIP              0x02000000      /* 25 */
#define CALIB_MASK_LO_KCAL_SKIP             0x01000000      /* 24 */
#define CALIB_MASK_CAL_DONE_DIS             0x00800000      /* 23 */
#define CALIB_MASK_REF_CAL_REQ              0x00008000      /* 15 */
#define CALIB_MASK_REF_CAL_DIS              0x00004000      /* 14 */
#define CALIB_MASK_REF_CAL                  0x00000F00      /* 11 - 8 */
#define CALIB_MASK_RC_CAL_REQ               0x00000080      /* 7 */
#define CALIB_MASK_RC_CAL_DIS               0x00000040      /* 6 */
#define CALIB_MASK_RC_CAL_DLY               0x00000030      /* 5 - 4 */
#define CALIB_MASK_RC_CAL                   0x0000000F      /* 3 - 0 */
#define CALIB_POS_LO_CAL_SKIP                       25
#define CALIB_POS_LO_KCAL_SKIP                      24
#define CALIB_POS_REF_CAL                            8
#define CALIB_POS_RC_CAL_DLY                         4
// CAL1
#define CALIB_MASK_LO_CAL_REQ               0x80000000      /* 31 */
#define CALIB_MASK_LO_ACAL_DIS              0x40000000      /* 30 */
#define CALIB_MASK_LO_ACAL_E                0x20000000      /* 29 */
#define CALIB_MASK_LO_ACAL                  0x1F000000      /* 28 - 24 */
#define CALIB_MASK_LO_SPEED_UP              0x00800000      /* 23 */
#define CALIB_MASK_LO_FCAL_DIS              0x00400000      /* 22 */
#define CALIB_MASK_LO_FCAL                  0x001F0000      /* 20 - 16 */
#define CALIB_MASK_LO_KCAL_REQ              0x00008000      /* 15 */
#define CALIB_MASK_LO_KCAL_DIS              0x00004000      /* 14 */
#define CALIB_MASK_LO_KCAL                  0x00003FF8      /* 13 - 3 */
#define CALIB_MASK_EN_KCAL_SD               0x00000002      /* 1 */
#define CALIB_MASK_DS_SEL                   0x00000001      /* 0 */
#define CALIB_POS_LO_ACAL                           24
#define CALIB_POS_LO_FCAL                           16
#define CALIB_POS_LO_KCAL                            3
// CAL2
#define CALIB_MASK_SD_GAIN_INV              0x03FF0000      /* 25 - 16 */
#define CALIB_MASK_LO_KDAC_E                0x00000700      /* 10 - 8 */
#define CALIB_MASK_TX_DLY2                  0x00000070      /*  6 - 4 */
#define CALIB_MASK_TX_DLY1                  0x00000007      /*  2 - 0 */
#define CALIB_POS_SD_GAIN_INV                       16
#define CALIB_POS_LO_KDAC_E                          8
#define CALIB_POS_TX_DLY2                            4
// CAL3
#define CALIB_MASK_PA_CAL_REQ               0x80000000      /* 31 */
#define CALIB_MASK_PA_CAL_DIS               0x40000000      /* 30 */
#define CALIB_MASK_PA_CAL                   0x0F000000      /* 27 - 24 */
#define CALIB_MASK_R_CAL_REQ                0x00800000      /* 23 */
#define CALIB_MASK_R_CAL_DIS                0x00400000      /* 22 */
#define CALIB_MASK_R_CAL                    0x000F0000      /* 19 - 16 */
#define CALIB_MASK_ROS_CAL_REQ              0x00008000      /* 15 */
#define CALIB_MASK_ROS_CAL_I_DIS            0x00004000      /* 14 */
#define CALIB_MASK_ROS_CAL_I                0x00003F00      /* 13 - 8 */
#define CALIB_MASK_ROS_CAL_Q_DIS            0x00000040      /* 6 */
#define CALIB_MASK_ROS_CAL_Q                0x0000003F      /* 5 - 0 */
#define CALIB_POS_PA_CAL                            24
#define CALIB_POS_R_CAL                             16
#define CALIB_POS_ROS_CAL_I                          8
// CAL4
#define CALIB_MASK_RCO_CAL_REQ              0x80000000      /* 31 */
#define CALIB_MASK_RCO_CAL_DIS              0x40000000      /* 30 */
#define CALIB_MASK_RCO_CAL                  0x0F000000      /* 27 - 24 */
#define CALIB_MASK_PA_CODE_TX               0x00F00000      /* 23 - 20 */
#define CALIB_MASK_PA_CODE_RX               0x000F0000      /* 19 - 16 */
#define CALIB_MASK_PLL_RDY_DLY              0x00001F00      /* 12 -  8 */
#define CALIB_MASK_LO_SU_DLY                0x0000001E      /* 4 - 1 */
#define CALIB_MASK_PA_CAL_EN                0x00000001      /* 0 */
#define CALIB_POS_RCO_CAL                           24
#define CALIB_POS_PA_CODE_TX                        20
#define CALIB_POS_PA_CODE_RX                        16
#define CALIB_POS_PLL_RDY_DLY                        8
#define CALIB_POS_LO_SU_DLY                          1
// CR
#define CALIB_MASK_32M_GATE_EN              0x80000000      /* 31 */
#define CALIB_MASK_16M_GATE_EN              0x40000000      /* 30 */
#define CALIB_MASK_CAL_IE                   0x00000100      /* 8 */
#define CALIB_MASK_REF_DONE_IE              0x00000080      /* 7 */
#define CALIB_MASK_RC_DONE_IE               0x00000040      /* 6 */
#define CALIB_MASK_LO_DONE_IE               0x00000020      /* 5 */
#define CALIB_MASK_KVCO_DONE_IE             0x00000010      /* 4 */
#define CALIB_MASK_PA_DONE_IE               0x00000008      /* 3 */
#define CALIB_MASK_R_DONE_IE                0x00000004      /* 2 */
#define CALIB_MASK_ROS_DONE_IE              0x00000002      /* 1 */
#define CALIB_MASK_RCO_DONE_IE              0x00000001      /* 0 */
// SR
#define CALIB_MASK_REF_DONE_IF              0x00000080      /* 7 */
#define CALIB_MASK_RC_DONE_IF               0x00000040      /* 6 */
#define CALIB_MASK_LO_DONE_IF               0x00000020      /* 5 */
#define CALIB_MASK_KVCO_DONE_IF             0x00000010      /* 4 */
#define CALIB_MASK_PA_DONE_IF               0x00000008      /* 3 */
#define CALIB_MASK_R_DONE_IF                0x00000004      /* 2 */
#define CALIB_MASK_ROS_DONE_IF              0x00000002      /* 1 */
#define CALIB_MASK_RCO_DONE_IF              0x00000001      /* 0 */
/*@}*/ /* end of group QN_CALIB */


/*------------- PROPRIETARY Interface ------------------------------------------------*/
/** @defgroup QN_PROP QN_PROP
    @ingroup QN9020_Peripherals
    @brief QN9020 Proprietary Interface Register Structure
  @{
*/
typedef struct
{
    __O uint32_t TXD;                       /*!< Offset: 0x0000 Proprietary TX Data Register    */
    __I uint32_t RXD;                       /*!< Offset: 0x0004 Calibration RX Data Register    */
    __IO uint32_t CR;                       /*!< Offset: 0x0008 Calibration Control Register    */
} QN_PROP_TypeDef;

/* PROPRIETARY Interface REGISTER MASK & POSITION */
// CR
#define PROP_MASK_PROP_CLR                  0x00000080      /* 7 */
#define PROP_MASK_TX_BUSY                   0x00000040      /* 6 */
#define PROP_MASK_RX_BUSY                   0x00000020      /* 5 */
#define PROP_MASK_TX_INT                    0x00000010      /* 4 */
#define PROP_MASK_RX_INT                    0x00000008      /* 3 */
#define PROP_MASK_RX_INT_MASK               0x00000004      /* 2 */
#define PROP_MASK_TX_INT_MASK               0x00000002      /* 1 */
#define PROP_MASK_BIT_ORDER                 0x00000001      /* 0 */
#define PROP_POS_PROP_CLR                            7
#define PROP_POS_TX_BUSY                             6
#define PROP_POS_RX_BUSY                             5
#define PROP_POS_TX_INT                              4
#define PROP_POS_RX_INT                              3
#define PROP_POS_RX_INT_MASK                         2
#define PROP_POS_TX_INT_MASK                         1
#define PROP_POS_BIT_ORDER                           0
/*@}*/ /* end of group QN_PROP */

#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif


/*------------- Pin mask ------------------------------------------------*/
/** @defgroup QN_PIN_MUX QN9020 Pin Mask
    @ingroup QN9020_Definitions
  @{
*/
// PIN_CTRL0[1:0]
#define P00_GPIO_0_PIN_CTRL                     (0<<0)      /*!< GPIO */
#define P00_UART0_TXD_PIN_CTRL                  (1<<0)      /*!< UART0_TXD */
#define P00_SPI0_DAT_PIN_CTRL                   (2<<0)      /*!< SPI0_DAT */
#define P00_RTCI_PIN_CTRL                       (3<<0)      /*!< RTCI */
#define P00_MASK_PIN_CTRL                       (3<<0)      /*!< Mask of Pin */
// PIN_CTRL0[3:2]
#define P01_GPIO_1_PIN_CTRL                     (0<<2)      /*!< GPIO */
#define P01_SPI0_CS0_PIN_CTRL                   (2<<2)      /*!< SPI0_CS0 */
#define P01_UART0_CTS_PIN_CTRL                  (3<<2)      /*!< UART0_CTS */
#define P01_MASK_PIN_CTRL                       (3<<2)      /*!< Mask of Pin */
// PIN_CTRL0[5:4]
#define P02_GPIO_2_PIN_CTRL                     (0<<4)      /*!< GPIO */
#define P02_I2C_SDA_PIN_CTRL                    (1<<4)      /*!< I2C_SDA */
#define P02_SPI0_CLK_PIN_CTRL                   (2<<4)      /*!< SPI0_CLK */
#define P02_UART0_RTS_PIN_CTRL                  (3<<4)      /*!< UART0_RTS */
#define P02_MASK_PIN_CTRL                       (3<<4)      /*!< Mask of Pin */
// PIN_CTRL0[7:6]
#define P03_GPIO_3_PIN_CTRL                     (0<<6)      /*!< GPIO */
#define P03_RADIO_EN_PIN_CTRL                   (1<<6)      /*!< RADIO_EN */
#define P03_CLKOUT0_PIN_CTRL                    (2<<6)      /*!< CLKOUT0 */
#define P03_TIMER0_ECLK_PIN_CTRL                (3<<6)      /*!< TIMER */
#define P03_MASK_PIN_CTRL                       (3<<6)      /*!< Mask of Pin */
// PIN_CTRL0[9:8]
#define P04_GPIO_4_PIN_CTRL                     (0<<8)      /*!< GPIO */
#define P04_CLKOUT1_PIN_CTRL                    (2<<8)      /*!< CLKOUT1 */
#define P04_RTCI_PIN_CTRL                       (3<<8)      /*!< RTCI */
#define P04_MASK_PIN_CTRL                       (3<<8)      /*!< Mask of Pin */
// PIN_CTRL0[11:10]
#define P05_GPIO_5_PIN_CTRL                     (0<<10)      /*!< GPIO */
#define P05_I2C_SCL_PIN_CTRL                    (1<<10)      /*!< I2C_SCL */
#define P05_ADCT_PIN_CTRL                       (2<<10)      /*!< ADCT */
#define P05_ACMP1_O_PIN_CTRL                    (3<<10)      /*!< ACMP1_O */
#define P05_MASK_PIN_CTRL                       (3<<10)      /*!< Mask of Pin */
// PIN_CTRL0[13:12]
#define P06_SW_DAT_PIN_CTRL                     (0<<12)      /*!< SW data */
#define P06_GPIO_6_PIN_CTRL                     (1<<12)      /*!< GPIO */
#define P06_AIN2_PIN_CTRL                       (2<<12)      /*!< AIN2 */
#define P06_ACMP1_P_PIN_CTRL                    (3<<12)      /*!< ACMP1+ */
#define P06_MASK_PIN_CTRL                       (3<<12)      /*!< Mask of Pin */
// PIN_CTRL0[15:14]
#define P07_SW_CLK_PIN_CTRL                     (0<<14)      /*!< SW clk */
#define P07_GPIO_7_PIN_CTRL                     (1<<14)      /*!< GPIO */
#define P07_AIN3_PIN_CTRL                       (2<<14)      /*!< AIN3 */
#define P07_ACMP1_N_PIN_CTRL                    (3<<14)      /*!< ACMP1- */
#define P07_MASK_PIN_CTRL                       (3<<14)      /*!< Mask of Pin */
// PIN_CTRL0[17:16]
#define P10_GPIO_8_PIN_CTRL                     (0<<16)      /*!< GPIO */
#define P10_SPI1_DIN_PIN_CTRL                   (1<<16)      /*!< SPI1_DIN */
#define P10_UART1_RXD_PIN_CTRL                  (2<<16)      /*!< UART1_RXD */
#define P10_TIMER2_ECLK_PIN_CTRL                (3<<16)      /*!< TIMER */
#define P10_MASK_PIN_CTRL                       (3<<16)      /*!< Mask of Pin */
// PIN_CTRL0[19:18]
#define P11_GPIO_9_PIN_CTRL                     (0<<18)      /*!< GPIO */
#define P11_SPI1_DAT_PIN_CTRL                   (1<<18)      /*!< SPI1_DAT */
#define P11_UART1_TXD_PIN_CTRL                  (2<<18)      /*!< UART1_TXD */
#define P11_TIMER1_ICP0_PIN_CTRL                (3<<18)      /*!< TIMER */
#define P11_MASK_PIN_CTRL                       (3<<18)      /*!< Mask of Pin */
// PIN_CTRL0[21:20]
#define P12_GPIO_10_PIN_CTRL                    (0<<20)      /*!< GPIO */
#define P12_SPI1_CS0_PIN_CTRL                   (1<<20)      /*!< SPI1_CS0 */
#define P12_UART1_CTS_PIN_CTRL                  (2<<20)      /*!< UART1_CTS */
#define P12_ADCT_PIN_CTRL                       (3<<20)      /*!< ADCT */
#define P12_MASK_PIN_CTRL                       (3<<20)      /*!< Mask of Pin */
// PIN_CTRL0[23:22]
#define P13_GPIO_11_PIN_CTRL                    (0<<22)      /*!< GPIO */
#define P13_SPI1_CLK_PIN_CTRL                   (1<<22)      /*!< SPI1_CLK */
#define P13_UART1_RTS_PIN_CTRL                  (2<<22)      /*!< UART1_RTS */
#define P13_CLKOUT1_PIN_CTRL                    (3<<22)      /*!< CLKOUT1 */
#define P13_MASK_PIN_CTRL                       (3<<22)      /*!< Mask of Pin */
// PIN_CTRL0[25:24]
#define P14_GPIO_12_PIN_CTRL                    (0<<24)      /*!< GPIO */
#define P14_RDYN_PIN_CTRL                       (1<<24)      /*!< RDYN */
#define P14_TIMER1_ICP3_PIN_CTRL                (3<<24)      /*!< TIMER */
#define P14_MASK_PIN_CTRL                       (3<<24)      /*!< Mask of Pin */
// PIN_CTRL0[27:26]
#define P15_GPIO_13_PIN_CTRL                    (0<<26)      /*!< GPIO */
#define P15_RADIO_EN_PIN_CTRL                   (1<<26)      /*!< RADIO_EN */
#define P15_PWM1_PIN_CTRL                       (2<<26)      /*!< PWM1 */
#define P15_TIMER1_ICP2_PIN_CTRL                (3<<26)      /*!< TIMER */
#define P15_MASK_PIN_CTRL                       (3<<26)      /*!< Mask of Pin */
// PIN_CTRL0[29:28]
#define P16_GPIO_14_PIN_CTRL                    (0<<28)      /*!< GPIO */
#define P16_SPI0_CS1_PIN_CTRL                   (1<<28)      /*!< SPI0_CS1 */
#define P16_PWM0_PIN_CTRL                       (2<<28)      /*!< PWM0 */
#define P16_TIMER0_ICP3_PIN_CTRL                (3<<28)      /*!< TIMER0 */
#define P16_MASK_PIN_CTRL                       (3<<28)      /*!< Mask of Pin */
// PIN_CTRL0[31:30]
#define P17_GPIO_15_PIN_CTRL                    (0<<30)      /*!< GPIO */
#define P17_UART0_RXD_PIN_CTRL                  (1UL<<30)    /*!< UART0_RXD */
#define P17_SPI0_DIN_PIN_CTRL                   (2UL<<30)    /*!< SPI0_DIN */
#define P17_TIMER0_OUT_PIN_CTRL                 (3UL<<30)    /*!< TIMER */
#define P17_MASK_PIN_CTRL                       (3UL<<30)    /*!< Mask of Pin */
// PIN_CTRL1[1:0]
#define P20_GPIO_16_PIN_CTRL                    (0<<0)      /*!< GPIO */
#define P20_SPI1_DIN_PIN_CTRL                   (1<<0)      /*!< SPI_DAT */
#define P20_UART1_RXD_PIN_CTRL                  (2<<0)      /*!< UART1_RXD */
#define P20_TIMER3_ICP2_PIN_CTRL                (3<<0)      /*!< TIMER */
#define P20_MASK_PIN_CTRL                       (3<<0)      /*!< Mask of Pin */
// PIN_CTRL1[3:2]
#define P21_GPIO_17_PIN_CTRL                    (0<<2)      /*!< GPIO */
#define P21_SPI1_DAT_PIN_CTRL                   (1<<2)      /*!< SPI1_DAT */
#define P21_UART1_TXD_PIN_CTRL                  (2<<2)      /*!< UART1_TXD */
#define P21_TIMER3_ICP1_PIN_CTRL                (3<<2)      /*!< TIMER */
#define P21_MASK_PIN_CTRL                       (3<<2)      /*!< Mask of Pin */
// PIN_CTRL1[5:4]
#define P22_GPIO_18_PIN_CTRL                    (0<<4)      /*!< GPIO */
#define P22_SPI1_CLK_PIN_CTRL                   (1<<4)      /*!< SPI1_CLK */
#define P22_UART1_RTS_PIN_CTRL                  (2<<4)      /*!< UART1_RTS */
#define P22_TIMER2_ICP3_PIN_CTRL                (3<<4)      /*!< TIMER */
#define P22_MASK_PIN_CTRL                       (3<<4)      /*!< Mask of Pin */
// PIN_CTRL1[7:6]
#define P23_GPIO_19_PIN_CTRL                    (0<<6)      /*!< GPIO */
#define P23_I2C_SDA_PIN_CTRL                    (1<<6)      /*!< I2C_SDA */
#define P23_ACMP0_O_PIN_CTRL                    (2<<6)      /*!< ACMP0_O */
#define P23_TIMER3_ICP0_PIN_CTRL                (3<<6)      /*!< TIMER */
#define P23_MASK_PIN_CTRL                       (3<<6)      /*!< Mask of Pin */
// PIN_CTRL1[9:8]
#define P24_GPIO_20_PIN_CTRL                    (0<<8)      /*!< GPIO */
#define P24_I2C_SCL_PIN_CTRL                    (1<<8)      /*!< I2C_SCL */
#define P24_PWM1_PIN_CTRL                       (2<<8)      /*!< PWM1 */
#define P24_TIMER3_ECLK_PIN_CTRL                (3<<8)      /*!< TIMER */
#define P24_MASK_PIN_CTRL                       (3<<8)      /*!< Mask of Pin */
// PIN_CTRL1[11:10]
#define P25_GPIO_21_PIN_CTRL                    (0<<10)      /*!< GPIO */
#define P25_SPI1_CS1_PIN_CTRL                   (1<<10)      /*!< SPI1_CS1 */
#define P25_TIMER2_ICP2_PIN_CTRL                (3<<10)      /*!< TIMER */
#define P25_MASK_PIN_CTRL                       (3<<10)      /*!< Mask of Pin */
// PIN_CTRL1[13:12]
#define P26_GPIO_22_PIN_CTRL                    (0<<12)      /*!< GPIO */
#define P26_RDYN_PIN_CTRL                       (1<<12)      /*!< RDYN */
#define P26_PWM1_PIN_CTRL                       (2<<12)      /*!< PWM1 */
#define P26_TIMER2_ICP0_PIN_CTRL                (3<<12)      /*!< TIMER */
#define P26_MASK_PIN_CTRL                       (3<<12)      /*!< Mask of Pin */
// PIN_CTRL1[15:14]
#define P27_GPIO_23_PIN_CTRL                    (0<<14)      /*!< GPIO */
#define P27_ACMP1_O_PIN_CTRL                    (1<<14)      /*!< ACMP1_O */
#define P27_PWM0_PIN_CTRL                       (2<<14)      /*!< PWM0 */
#define P27_TIMER1_ECLK_PIN_CTRL                (3<<14)      /*!< TIMER */
#define P27_MASK_PIN_CTRL                       (3<<14)      /*!< Mask of Pin */
// PIN_CTRL1[17:16]
#define P30_GPIO_24_PIN_CTRL                    (0<<16)      /*!< GPIO */
#define P30_TIMER2_ICP1_PIN_CTRL                (1<<16)      /*!< TIMER */
#define P30_AIN0_PIN_CTRL                       (2<<16)      /*!< AIN0 */
#define P30_ACMP0_P_PIN_CTRL                    (3<<16)      /*!< ACMP0+ */
#define P30_MASK_PIN_CTRL                       (3<<16)      /*!< Mask of Pin */
// PIN_CTRL1[19:18]
#define P31_GPIO_25_PIN_CTRL                    (0<<18)      /*!< GPIO */
#define P31_TIMER0_ICP2_PIN_CTRL                (1<<18)      /*!< TIMER */
#define P31_AIN1_PIN_CTRL                       (2<<18)      /*!< AIN1 */
#define P31_ACMP0_N_PIN_CTRL                    (3<<18)      /*!< ACMP1- */
#define P31_MASK_PIN_CTRL                       (3<<18)      /*!< Mask of Pin */
// PIN_CTRL1[21:20]
#define P32_GPIO_26_PIN_CTRL                    (0<<20)      /*!< GPIO */
#define P32_SPI0_DIN_PIN_CTRL                   (1<<20)      /*!< SPI0_DIN */
#define P32_ACMP0_O_PIN_CTRL                    (3<<20)      /*!< ACMP0_O */
#define P32_MASK_PIN_CTRL                       (3<<20)      /*!< Mask of Pin */
// PIN_CTRL1[23:22]
#define P33_GPIO_27_PIN_CTRL                    (0<<22)      /*!< GPIO */
#define P33_SPI0_DAT_PIN_CTRL                   (1<<22)      /*!< SPI0_DAT */
#define P33_CLKOUT0_PIN_CTRL                    (2<<22)      /*!< CLKOUT0 */
#define P33_MASK_PIN_CTRL                       (3<<22)      /*!< Mask of Pin */
// PIN_CTRL1[24]
#define P34_GPIO_28_PIN_CTRL                    (0<<24)      /*!< GPIO */
#define P34_SPI0_CLK_PIN_CTRL                   (1<<24)      /*!< SPI0_CLK */
#define P34_MASK_PIN_CTRL                       (1<<24)      /*!< Mask of Pin */
// PIN_CTRL1[26:25]
#define P35_GPIO_29_PIN_CTRL                    (0<<25)      /*!< GPIO */
#define P35_SPI0_CS0_PIN_CTRL                   (1<<25)      /*!< SPI0_CS0 */
#define P35_INT_FM_PIN_CTRL                     (2<<25)      /*!< INT_FM */
#define P35_TIMER0_ICP0_PIN_CTRL                (3<<25)      /*!< TIMER */
#define P35_MASK_PIN_CTRL                       (3<<25)      /*!< Mask of Pin */
// PIN_CTRL1[28:27]
#define P36_GPIO_30_PIN_CTRL                    (0<<27)      /*!< GPIO */
#define P36_SPI1_CS0_PIN_CTRL                   (1<<27)      /*!< SPI1_CS0 */
#define P36_UART1_CTS_PIN_CTRL                  (2<<27)      /*!< UART1_CTS */
#define P36_MASK_PIN_CTRL                       (3<<27)      /*!< Mask of Pin */

/*------------- Peripheral memory map-----------------------------------------*/
/* ToDo: add here your device peripherals base addresses
         following is an example for timer                                    */
/** @defgroup QN9020_MemoryMap QN9020 Memory Mapping
    @ingroup QN9020_Definitions
  @{
*/

/* Base address */
#define QN_ROM_BASE                         (0x00000000UL)            /*!< (ROM       ) Base Address */
#define QN_SRAM_BASE                        (0x20000000UL)            /*!< (SRAM      ) Base Address */
#define QN_BLE_BASE                         (0x2F000000UL)            /*!< (BLE       ) Base Address */
#define QN_FLASH_BASE                       (0x30000000UL)            /*!< (Serial Flash) Base Address */
#define QN_APB_BASE                         (0x40000000UL)            /*!< (Peripheral) Base Address */
#define QN_AHB_BASE                         (0x50000000UL)            /*!< (AHB       ) Base Address */

/* APB Peripheral memory map */
#define QN_SYSCON_BASE                      (QN_APB_BASE + 0x0000)    /*!< (System Controller) Base Address */
#define QN_WDT_BASE                         (QN_APB_BASE + 0x1000)    /*!< (Watchdog Timer) Base Address */
#define QN_TIMER0_BASE                      (QN_APB_BASE + 0x2000)    /*!< (Timer0    ) Base Address */
#define QN_TIMER1_BASE                      (QN_APB_BASE + 0x3000)    /*!< (Timer1    ) Base Address */
#define QN_TIMER2_BASE                      (QN_APB_BASE + 0x4000)    /*!< (Timer2    ) Base Address */
#define QN_TIMER3_BASE                      (QN_APB_BASE + 0x5000)    /*!< (Timer3    ) Base Address */
#define QN_RTC_BASE                         (QN_APB_BASE + 0x6000)    /*!< (RTC       ) Base Address */
#define QN_UART0_BASE                       (QN_APB_BASE + 0x7000)    /*!< (UART0     ) Base Address */
#define QN_SPI0_BASE                        (QN_APB_BASE + 0x7800)    /*!< (SPI0      ) Base Address */
#define QN_I2C_BASE                         (QN_APB_BASE + 0x8000)    /*!< (I2C       ) Base Address */
#define QN_DMA_BASE                         (QN_APB_BASE + 0x9000)    /*!< (DMA       ) Base Address */
#define QN_UART1_BASE                       (QN_APB_BASE + 0xA000)    /*!< (UART1     ) Base Address */
#define QN_SPI1_BASE                        (QN_APB_BASE + 0xA800)    /*!< (SPI1      ) Base Address */
#define QN_DP_BASE                          (QN_APB_BASE + 0xB000)    /*!< (BLE DataPath) Base Address */
#define QN_CALIB_BASE                       (QN_APB_BASE + 0xC000)    /*!< (Calibration) Base Address */
#define QN_PROP_BASE                        (QN_APB_BASE + 0xD000)    /*!< (Proprietary) Base Address */
#define QN_PWM_BASE                         (QN_APB_BASE + 0xE000)    /*!< (PWM      ) Base Address */
#define QN_SF_CTRL_BASE                     (QN_FLASH_BASE + 0xFFFFFE8) /*!< (Serial Flash Controller) Base Address */

/* AHB Peripheral */
#define QN_GPIO_BASE                        (QN_AHB_BASE + 0x00000)     /*!< (GPIO    ) Base Address */
#define QN_ADC_BASE                         (QN_AHB_BASE + 0x10000)     /*!< (ADC     ) Base Address */
/*@}*/ /* end of group QN9020_MemoryMap */


/*------------- Peripheral declaration ---------------------------------------*/
/* ToDo: add here your device peripherals pointer definitions
         following is an example for timer                                    */


/** @addtogroup QN9020_PeripheralDecl QN9020 Peripheral Declaration
    @ingroup QN9020_Definitions
  @{
*/
#define QN_SYSCON                           ((QN_SYSCON_TypeDef *)  QN_SYSCON_BASE)
#define QN_GPIO                             ((QN_GPIO_TypeDef *)    QN_GPIO_BASE)
#define QN_WDT                              ((QN_WDT_TypeDef *)     QN_WDT_BASE)
#define QN_TIMER0                           ((QN_TIMER_TypeDef *)   QN_TIMER0_BASE)
#define QN_TIMER1                           ((QN_TIMER_TypeDef *)   QN_TIMER1_BASE)
#define QN_TIMER2                           ((QN_TIMER_TypeDef *)   QN_TIMER2_BASE)
#define QN_TIMER3                           ((QN_TIMER_TypeDef *)   QN_TIMER3_BASE)
#define QN_RTC                              ((QN_RTC_TypeDef *)     QN_RTC_BASE)
#define QN_UART0                            ((QN_UART_TypeDef *)    QN_UART0_BASE)
#define QN_UART1                            ((QN_UART_TypeDef *)    QN_UART1_BASE)
#define QN_SPI0                             ((QN_SPI_TypeDef *)     QN_SPI0_BASE)
#define QN_SPI1                             ((QN_SPI_TypeDef *)     QN_SPI1_BASE)
#define QN_I2C                              ((QN_I2C_TypeDef *)     QN_I2C_BASE)
#define QN_DMA                              ((QN_DMA_TypeDef *)     QN_DMA_BASE)
#define QN_DP                               ((QN_DP_TypeDef *)      QN_DP_BASE)
#define QN_CALIB                            ((QN_CALIB_TypeDef *)   QN_CALIB_BASE)
#define QN_PROP                             ((QN_PROP_TypeDef *)    QN_PROP_BASE)
#define QN_PWM                              ((QN_PWM_TypeDef *)     QN_PWM_BASE)
#define QN_SF_CTRL                          ((QN_SF_CTRL_TypeDef *) QN_SF_CTRL_BASE)
#define QN_ADC                              ((QN_ADC_TypeDef *)     QN_ADC_BASE)
/*@}*/ /* end of group QN9020_PeripheralDecl */


#ifdef __cplusplus
}
#endif


#endif  /* QN9020_H */
