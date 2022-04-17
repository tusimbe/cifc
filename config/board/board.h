/*
    ChibiOS - Copyright (C) 2006..2020 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 * This file has been automatically generated using ChibiStudio board
 * generator plugin. Do not edit manually.
 */

#ifndef BOARD_H
#define BOARD_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*
 * Setup for STMicroelectronics STM32F429I-Discovery board.
 */

/*
 * Board identifier.
 */
#define BOARD_CHASING_INNOVATION_M2
#define BOARD_NAME                  "CHASING INNOVATION M2"

/*
 * Board oscillators-related settings.
 * NOTE: LSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                32768U
#endif

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                24000000U
#endif

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   300U

/*
 * MCU type as defined in the ST header.
 */
#define STM32F427xx

/*
 * IO pins assignments.
 */
#define GPIOA_UART4_TX              0U
#define GPIOA_UART4_RX              1U
#define GPIOA_PIN2                  2U
#define GPIOA_PIN3                  3U
#define GPIOA_PIN4                  4U
#define GPIOA_SPI1_CLK              5U
#define GPIOA_SPI1_MISO             6U
#define GPIOA_SPI1_MOSI             7U
#define GPIOA_PIN8                  8U
#define GPIOA_UART1_TX              9U
#define GPIOA_UART1_RX              10U
#define GPIOA_OTG_FS_DM             11U
#define GPIOA_OTG_FS_DP             12U
#define GPIOA_SWDIO                 13U
#define GPIOA_SWCLK                 14U
#define GPIOA_PIN15                 15U

#define GPIOB_LED_PWM1              0U
#define GPIOB_PIN1                  1U
#define GPIOB_BOOT1                 2U
#define GPIOB_SWO                   3U
#define GPIOB_PIN4                  4U
#define GPIOB_PIN5                  5U
#define GPIOB_I2C1_SCL              6U
#define GPIOB_I2C1_SDA              7U
#define GPIOB_PIN8                  8U
#define GPIOB_PIN9                  9U
#define GPIOB_UART3_TX              10U
#define GPIOB_UART3_RX              11U
#define GPIOB_FRAM_CS               12U
#define GPIOB_SPI2_FRAM_CLK         13U
#define GPIOB_SPI2_FRAM_MISO        14U
#define GPIOB_SPI2_FRAM_MOSI        15U

#define GPIOC_PIN0                  0U
#define GPIOC_PIN1                  1U
#define GPIOC_PIN2                  2U
#define GPIOC_PIN3                  3U
#define GPIOC_MPU6000_CS            4U
#define GPIOC_PIN5                  5U
#define GPIOC_PIN6                  6U
#define GPIOC_PIN7                  7U
#define GPIOC_SDIO_D0               8U
#define GPIOC_SDIO_D1               9U
#define GPIOC_SDIO_D2               10U
#define GPIOC_SDIO_D3               11U
#define GPIOC_SDIO_CK               12U
#define GPIOC_CONF4                 13U
#define GPIOC_OSC32_IN              14U
#define GPIOC_OSC32_OUT             15U

#define GPIOD_CAN1_RX               0U
#define GPIOD_CAN1_TX               1U
#define GPIOD_SDIO_CMD              2U
#define GPIOD_PIN3                  3U
#define GPIOD_PIN4                  4U
#define GPIOD_UART2_TX              5U
#define GPIOD_UART2_RX              6U
#define GPIOD_PIN7                  7U
#define GPIOD_PIN8                  8U
#define GPIOD_PIN9                  9U
#define GPIOD_PIN10                 10U
#define GPIOD_PIN11                 11U
#define GPIOD_PWM_CH5               12U
#define GPIOD_PWM_CH6               13U
#define GPIOD_PWM_CH7               14U
#define GPIOD_PWM_CH8               15U

#define GPIOE_UART8_RX              0U
#define GPIOE_UART8_TX              1U
#define GPIOE_PIN2                  2U
#define GPIOE_PIN3                  3U
#define GPIOE_PIN4                  4U
#define GPIOE_PIN5                  5U
#define GPIOE_PIN6                  6U
#define GPIOE_UART7_RX              7U
#define GPIOE_UART7_TX              8U
#define GPIOE_PWM_CH1               9U
#define GPIOE_PIN10                 10U
#define GPIOE_PWM_CH2               11U
#define GPIOE_PIN12                 12U
#define GPIOE_PWM_CH3               13U
#define GPIOE_PWM_CH4               14U
#define GPIOE_PIN15                 15U

#define GPIOF_I2C2_SDA              0U
#define GPIOF_I2C2_CLK              1U
#define GPIOF_UART1_RTSN            2U
#define GPIOF_CONF1                 3U
#define GPIOF_CONF2                 4U
#define GPIOF_CONF3                 5U
#define GPIOF_PIN6                  6U
#define GPIOF_PIN7                  7U
#define GPIOF_PIN8                  8U
#define GPIOF_PIN9                  9U
#define GPIOF_PIN10                 10U
#define GPIOF_PIN11                 11U
#define GPIOF_CONF5                 12U
#define GPIOF_POWER_DELAY           13U
#define GPIOF_POWER_DETECT          14U
#define GPIOF_LED_GREEN             15U

#define GPIOG_LED_AMBER             0U
#define GPIOG_MPU_DRDY              1U
#define GPIOG_LED_SAFETY            2U
#define GPIOG_PIN3                  3U
#define GPIOG_T_SEN                 4U
#define GPIOG_LED_BLUE              5U
#define GPIOG_PIN6                  6U
#define GPIOG_PIN7                  7U
#define GPIOG_PIN8                  8U
#define GPIOG_UART6_RX              9U
#define GPIOG_PIN10                 10U
#define GPIOG_PIN11                 11U
#define GPIOG_PIN12                 12U
#define GPIOG_PIN13                 13U
#define GPIOG_UART6_TX              14U
#define GPIOG_PIN15                 15U

#define GPIOH_OSC_IN                0U
#define GPIOH_OSC_OUT               1U
#define GPIOH_PIN2                  2U
#define GPIOH_PIN3                  3U
#define GPIOH_PIN4                  4U
#define GPIOH_PIN5                  5U
#define GPIOH_PIN6                  6U
#define GPIOH_PIN7                  7U
#define GPIOH_PIN8                  8U
#define GPIOH_PIN9                  9U
#define GPIOH_PIN10                 10U
#define GPIOH_PIN11                 11U
#define GPIOH_PIN12                 12U
#define GPIOH_PIN13                 13U
#define GPIOH_PIN14                 14U
#define GPIOH_PIN15                 15U

#define GPIOI_PIN0                  0U
#define GPIOI_PIN1                  1U
#define GPIOI_PIN2                  2U
#define GPIOI_PIN3                  3U
#define GPIOI_PIN4                  4U
#define GPIOI_PIN5                  5U
#define GPIOI_PIN6                  6U
#define GPIOI_PIN7                  7U
#define GPIOI_PIN8                  8U
#define GPIOI_PIN9                  9U
#define GPIOI_PIN10                 10U
#define GPIOI_PIN11                 11U
#define GPIOI_PIN12                 12U
#define GPIOI_PIN13                 13U
#define GPIOI_PIN14                 14U
#define GPIOI_PIN15                 15U

/*
 * IO lines assignments.
 */
#define LINE_BUTTON                 PAL_LINE(GPIOA, 0U)
#define LINE_MEMS_INT1              PAL_LINE(GPIOA, 1U)
#define LINE_MEMS_INT2              PAL_LINE(GPIOA, 2U)
#define LINE_LCD_B5                 PAL_LINE(GPIOA, 3U)
#define LINE_LCD_VSYNC              PAL_LINE(GPIOA, 4U)
#define LINE_LCD_G2                 PAL_LINE(GPIOA, 6U)
#define LINE_ACP_RST                PAL_LINE(GPIOA, 7U)
#define LINE_I2C3_SCL               PAL_LINE(GPIOA, 8U)
#define LINE_UART_TX                PAL_LINE(GPIOA, 9U)
#define LINE_UART_RX                PAL_LINE(GPIOA, 10U)
#define LINE_OTG_FS_DM              PAL_LINE(GPIOA, 11U)
#define LINE_OTG_FS_DP              PAL_LINE(GPIOA, 12U)
#define LINE_SWDIO                  PAL_LINE(GPIOA, 13U)
#define LINE_SWCLK                  PAL_LINE(GPIOA, 14U)
#define LINE_TP_INT                 PAL_LINE(GPIOA, 15U)
#define LINE_LCD_R3                 PAL_LINE(GPIOB, 0U)
#define LINE_LCD_R6                 PAL_LINE(GPIOB, 1U)
#define LINE_BOOT1                  PAL_LINE(GPIOB, 2U)
#define LINE_SWO                    PAL_LINE(GPIOB, 3U)
#define LINE_FMC_SDCKE1             PAL_LINE(GPIOB, 5U)
#define LINE_FMC_SDNE1              PAL_LINE(GPIOB, 6U)
#define LINE_LCD_B6                 PAL_LINE(GPIOB, 8U)
#define LINE_LCD_B7                 PAL_LINE(GPIOB, 9U)
#define LINE_LCD_G4                 PAL_LINE(GPIOB, 10U)
#define LINE_LCD_G5                 PAL_LINE(GPIOB, 11U)
#define LINE_FRAM_CS                PAL_LINE(GPIOB, GPIOB_FRAM_CS)
#define LINE_OTG_HS_VBUS            PAL_LINE(GPIOB, 13U)
#define LINE_OTG_HS_DM              PAL_LINE(GPIOB, 14U)
#define LINE_OTG_HS_DP              PAL_LINE(GPIOB, 15U)
#define LINE_FMC_SDNWE              PAL_LINE(GPIOC, 0U)
#define LINE_SPI5_MEMS_CS           PAL_LINE(GPIOC, 1U)
#define LINE_SPI5_LCD_CS            PAL_LINE(GPIOC, 2U)
#define LINE_MPU6000_CS             PAL_LINE(GPIOC, GPIOC_MPU6000_CS)
#define LINE_OTG_HS_OC              PAL_LINE(GPIOC, 5U)
#define LINE_LCD_HSYNC              PAL_LINE(GPIOC, 6U)
#define LINE_LCD_G6                 PAL_LINE(GPIOC, 7U)
#define LINE_I2C3_SDA               PAL_LINE(GPIOC, 9U)
#define LINE_LCD_R2                 PAL_LINE(GPIOC, 10U)
#define LINE_OSC32_IN               PAL_LINE(GPIOC, 14U)
#define LINE_OSC32_OUT              PAL_LINE(GPIOC, 15U)
#define LINE_FMC_D2                 PAL_LINE(GPIOD, 0U)
#define LINE_FMC_D3                 PAL_LINE(GPIOD, 1U)
#define LINE_LCD_G7                 PAL_LINE(GPIOD, 3U)
#define LINE_LCD_B2                 PAL_LINE(GPIOD, 6U)
#define LINE_FMC_D13                PAL_LINE(GPIOD, 8U)
#define LINE_FMC_D14                PAL_LINE(GPIOD, 9U)
#define LINE_FMC_D15                PAL_LINE(GPIOD, 10U)
#define LINE_LCD_TE                 PAL_LINE(GPIOD, 11U)
#define LINE_LCD_RDX                PAL_LINE(GPIOD, 12U)
#define LINE_LCD_WRX                PAL_LINE(GPIOD, 13U)
#define LINE_FMC_D0                 PAL_LINE(GPIOD, 14U)
#define LINE_FMC_D1                 PAL_LINE(GPIOD, 15U)
#define LINE_FMC_NBL0               PAL_LINE(GPIOE, 0U)
#define LINE_FMC_NBL1               PAL_LINE(GPIOE, 1U)
#define LINE_FMC_D4                 PAL_LINE(GPIOE, 7U)
#define LINE_FMC_D5                 PAL_LINE(GPIOE, 8U)
#define LINE_FMC_D6                 PAL_LINE(GPIOE, 9U)
#define LINE_FMC_D7                 PAL_LINE(GPIOE, 10U)
#define LINE_FMC_D8                 PAL_LINE(GPIOE, 11U)
#define LINE_FMC_D9                 PAL_LINE(GPIOE, 12U)
#define LINE_FMC_D10                PAL_LINE(GPIOE, 13U)
#define LINE_FMC_D11                PAL_LINE(GPIOE, 14U)
#define LINE_FMC_D12                PAL_LINE(GPIOE, 15U)
#define LINE_FMC_A0                 PAL_LINE(GPIOF, 0U)
#define LINE_FMC_A1                 PAL_LINE(GPIOF, 1U)
#define LINE_FMC_A2                 PAL_LINE(GPIOF, 2U)
#define LINE_FMC_A3                 PAL_LINE(GPIOF, 3U)
#define LINE_FMC_A4                 PAL_LINE(GPIOF, 4U)
#define LINE_FMC_A5                 PAL_LINE(GPIOF, 5U)
#define LINE_LCD_DCX                PAL_LINE(GPIOF, 7U)
#define LINE_SPI5_MISO              PAL_LINE(GPIOF, 8U)
#define LINE_SPI5_MOSI              PAL_LINE(GPIOF, 9U)
#define LINE_LCD_DE                 PAL_LINE(GPIOF, 10U)
#define LINE_FMC_SDNRAS             PAL_LINE(GPIOF, 11U)
#define LINE_FMC_A6                 PAL_LINE(GPIOF, 12U)
#define LINE_FMC_A7                 PAL_LINE(GPIOF, 13U)
#define LINE_FMC_A8                 PAL_LINE(GPIOF, 14U)
#define LINE_LED_GREEN              PAL_LINE(GPIOF, 15U)
#define LINE_FMC_A10                PAL_LINE(GPIOG, 0U)
#define LINE_FMC_A11                PAL_LINE(GPIOG, 1U)
#define LINE_FMC_BA0                PAL_LINE(GPIOG, 4U)
#define LINE_LED_BLUE               PAL_LINE(GPIOG, GPIOG_LED_BLUE)
#define LINE_LCD_R7                 PAL_LINE(GPIOG, 6U)
#define LINE_LCD_CLK                PAL_LINE(GPIOG, 7U)
#define LINE_FMC_SDCLK              PAL_LINE(GPIOG, 8U)
#define LINE_LCD_G3                 PAL_LINE(GPIOG, 10U)
#define LINE_LCD_B3                 PAL_LINE(GPIOG, 11U)
#define LINE_LCD_B4                 PAL_LINE(GPIOG, 12U)
#define LINE_LED3_GREEN             PAL_LINE(GPIOG, 13U)
#define LINE_LED4_RED               PAL_LINE(GPIOG, 14U)
#define LINE_FMC_SDNCAS             PAL_LINE(GPIOG, 15U)
#define LINE_OSC_IN                 PAL_LINE(GPIOH, 0U)
#define LINE_OSC_OUT                PAL_LINE(GPIOH, 1U)

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

/*
 * GPIOA setup:
 *
 * PA0  - BUTTON                    (input floating).
 * PA1  - MEMS_INT1                 (input floating).
 * PA2  - MEMS_INT2                 (input floating).
 * PA3  - LCD_B5                    (alternate 14).
 * PA4  - LCD_VSYNC                 (alternate 14).
 * PA5  - PIN5                      (input pullup).
 * PA6  - LCD_G2                    (alternate 14).
 * PA7  - ACP_RST                   (input pullup).
 * PA8  - I2C3_SCL                  (alternate 4).
 * PA9  - UART_TX                   (alternate 7).
 * PA10 - UART_RX                   (alternate 7).
 * PA11 - LCD_R4                    (alternate 14).
 * PA12 - LCD_R5                    (alternate 14).
 * PA13 - SWDIO                     (alternate 0).
 * PA14 - SWCLK                     (alternate 0).
 * PA15 - TP_INT                    (input floating).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_UART4_TX) |       \
                                     PIN_MODE_INPUT(GPIOA_UART4_RX) |       \
                                     PIN_MODE_INPUT(GPIOA_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOA_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOA_PIN4) |           \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_CLK) |   \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_MISO) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_SPI1_MOSI) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_PIN8) |       \
                                     PIN_MODE_INPUT(GPIOA_UART1_TX) |       \
                                     PIN_MODE_INPUT(GPIOA_UART1_RX) |       \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DM) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DP) |  \
                                     PIN_MODE_INPUT(GPIOA_SWDIO) |      \
                                     PIN_MODE_INPUT(GPIOA_SWCLK) |      \
                                     PIN_MODE_INPUT(GPIOA_PIN15))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_UART4_TX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART4_RX) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN2) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN4) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_CLK) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_MISO) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SPI1_MOSI) |    \
                                     PIN_OTYPE_OPENDRAIN(GPIOA_PIN8) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART1_TX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART1_RX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DM) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DP) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PIN15))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_MEDIUM(GPIOA_UART4_TX) |     \
                                     PIN_OSPEED_MEDIUM(GPIOA_UART4_RX) |  \
                                     PIN_OSPEED_MEDIUM(GPIOA_PIN2) |  \
                                     PIN_OSPEED_MEDIUM(GPIOA_PIN3) |        \
                                     PIN_OSPEED_MEDIUM(GPIOA_PIN4) |     \
                                     PIN_OSPEED_MEDIUM(GPIOA_SPI1_CLK) |       \
                                     PIN_OSPEED_MEDIUM(GPIOA_SPI1_MISO) |        \
                                     PIN_OSPEED_MEDIUM(GPIOA_SPI1_MOSI) |    \
                                     PIN_OSPEED_MEDIUM(GPIOA_PIN8) |      \
                                     PIN_OSPEED_MEDIUM(GPIOA_UART1_TX) |    \
                                     PIN_OSPEED_MEDIUM(GPIOA_UART1_RX) |    \
                                     PIN_OSPEED_MEDIUM(GPIOA_OTG_FS_DM) |        \
                                     PIN_OSPEED_MEDIUM(GPIOA_OTG_FS_DP) |        \
                                     PIN_OSPEED_MEDIUM(GPIOA_SWDIO) |         \
                                     PIN_OSPEED_MEDIUM(GPIOA_SWCLK) |         \
                                     PIN_OSPEED_MEDIUM(GPIOA_PIN15))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_UART4_TX) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_UART4_RX) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN2) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN3) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN4) |  \
                                     PIN_PUPDR_PULLUP(GPIOA_SPI1_CLK) |         \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI1_MISO) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_SPI1_MOSI) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN8) |   \
                                     PIN_PUPDR_FLOATING(GPIOA_UART1_TX) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_UART1_RX) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DM) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DP) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_SWDIO) |        \
                                     PIN_PUPDR_FLOATING(GPIOA_SWCLK) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_PIN15))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_UART4_TX) |           \
                                     PIN_ODR_HIGH(GPIOA_UART4_RX) |        \
                                     PIN_ODR_HIGH(GPIOA_PIN2) |        \
                                     PIN_ODR_HIGH(GPIOA_PIN3) |           \
                                     PIN_ODR_HIGH(GPIOA_PIN4) |        \
                                     PIN_ODR_HIGH(GPIOA_SPI1_CLK) |             \
                                     PIN_ODR_HIGH(GPIOA_SPI1_MISO) |           \
                                     PIN_ODR_HIGH(GPIOA_SPI1_MOSI) |          \
                                     PIN_ODR_HIGH(GPIOA_PIN8) |         \
                                     PIN_ODR_HIGH(GPIOA_UART1_TX) |          \
                                     PIN_ODR_HIGH(GPIOA_UART1_RX) |          \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DM) |           \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DP) |           \
                                     PIN_ODR_HIGH(GPIOA_SWDIO) |            \
                                     PIN_ODR_HIGH(GPIOA_SWCLK) |            \
                                     PIN_ODR_HIGH(GPIOA_PIN15))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_UART4_TX,  0U) |        \
                                     PIN_AFIO_AF(GPIOA_UART4_RX,  0U) |     \
                                     PIN_AFIO_AF(GPIOA_PIN2,      0U) |     \
                                     PIN_AFIO_AF(GPIOA_PIN3,      0U) |       \
                                     PIN_AFIO_AF(GPIOA_PIN4,      0U) |    \
                                     PIN_AFIO_AF(GPIOA_SPI1_CLK,  5U) |          \
                                     PIN_AFIO_AF(GPIOA_SPI1_MISO, 5U) |       \
                                     PIN_AFIO_AF(GPIOA_SPI1_MOSI, 5U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_PIN8,      4U) |      \
                                     PIN_AFIO_AF(GPIOA_UART1_TX,  0U) |       \
                                     PIN_AFIO_AF(GPIOA_UART1_RX,  0U) |       \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DM, 10U) |       \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DP, 10U) |       \
                                     PIN_AFIO_AF(GPIOA_SWDIO,     0U) |         \
                                     PIN_AFIO_AF(GPIOA_SWCLK,     0U) |         \
                                     PIN_AFIO_AF(GPIOA_PIN15,    0U))

/*
 * GPIOB setup:
 *
 * PB0  - LCD_R3                    (alternate 14).
 * PB1  - LCD_R6                    (alternate 14).
 * PB2  - BOOT1                     (input pullup).
 * PB3  - SWO                       (alternate 0).
 * PB4  - PIN4                      (input pullup).
 * PB5  - FMC_SDCKE1                (alternate 12).
 * PB6  - FMC_SDNE1                 (alternate 12).
 * PB7  - PIN7                      (input pullup).
 * PB8  - LCD_B6                    (alternate 14).
 * PB9  - LCD_B7                    (alternate 14).
 * PB10 - LCD_G4                    (alternate 14).
 * PB11 - LCD_G5                    (alternate 14).
 * PB12 - OTG_HS_ID                 (alternate 12).
 * PB13 - OTG_HS_VBUS               (input pulldown).
 * PB14 - OTG_HS_DM                 (alternate 12).
 * PB15 - OTG_HS_DP                 (alternate 12).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_OUTPUT(GPIOB_LED_PWM1) |     \
                                     PIN_MODE_INPUT(GPIOB_PIN1) |     \
                                     PIN_MODE_INPUT(GPIOB_BOOT1) |          \
                                     PIN_MODE_INPUT(GPIOB_SWO) |        \
                                     PIN_MODE_INPUT(GPIOB_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOB_PIN5) | \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SCL) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SDA) |           \
                                     PIN_MODE_INPUT(GPIOB_PIN8) |     \
                                     PIN_MODE_INPUT(GPIOB_PIN9) |     \
                                     PIN_MODE_INPUT(GPIOB_UART3_TX) |     \
                                     PIN_MODE_INPUT(GPIOB_UART3_RX) |     \
                                     PIN_MODE_OUTPUT(GPIOB_FRAM_CS) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI2_FRAM_CLK) |    \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI2_FRAM_MISO) |  \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI2_FRAM_MOSI))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_LED_PWM1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_BOOT1) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SWO) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN5) | \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SCL) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOB_I2C1_SDA) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN8) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN9) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_UART3_TX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_UART3_RX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_FRAM_CS) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI2_FRAM_CLK) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI2_FRAM_MISO) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI2_FRAM_MOSI))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_MEDIUM(GPIOB_LED_PWM1) |        \
                                     PIN_OSPEED_MEDIUM(GPIOB_PIN1) |        \
                                     PIN_OSPEED_MEDIUM(GPIOB_BOOT1) |         \
                                     PIN_OSPEED_MEDIUM(GPIOB_SWO) |           \
                                     PIN_OSPEED_MEDIUM(GPIOB_PIN4) |       \
                                     PIN_OSPEED_MEDIUM(GPIOB_PIN5) |    \
                                     PIN_OSPEED_MEDIUM(GPIOB_I2C1_SCL) |     \
                                     PIN_OSPEED_MEDIUM(GPIOB_I2C1_SDA) |       \
                                     PIN_OSPEED_MEDIUM(GPIOB_PIN8) |        \
                                     PIN_OSPEED_MEDIUM(GPIOB_PIN9) |        \
                                     PIN_OSPEED_MEDIUM(GPIOB_UART3_TX) |        \
                                     PIN_OSPEED_MEDIUM(GPIOB_UART3_RX) |        \
                                     PIN_OSPEED_MEDIUM(GPIOB_FRAM_CS) |     \
                                     PIN_OSPEED_MEDIUM(GPIOB_SPI2_FRAM_CLK) |\
                                     PIN_OSPEED_MEDIUM(GPIOB_SPI2_FRAM_MISO) |     \
                                     PIN_OSPEED_MEDIUM(GPIOB_SPI2_FRAM_MOSI))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_LED_PWM1) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN1) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_BOOT1) |        \
                                     PIN_PUPDR_FLOATING(GPIOB_SWO) |        \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN4) |         \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN5) | \
                                     PIN_PUPDR_FLOATING(GPIOB_I2C1_SCL) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_I2C1_SDA) |         \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN8) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_PIN9) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_UART3_TX) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_UART3_RX) |     \
                                     PIN_PUPDR_PULLUP(GPIOB_FRAM_CS) |  \
                                     PIN_PUPDR_PULLUP(GPIOB_SPI2_FRAM_CLK) |\
                                     PIN_PUPDR_FLOATING(GPIOB_SPI2_FRAM_MISO) |  \
                                     PIN_PUPDR_FLOATING(GPIOB_SPI2_FRAM_MOSI))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_LED_PWM1) |           \
                                     PIN_ODR_HIGH(GPIOB_PIN1) |           \
                                     PIN_ODR_HIGH(GPIOB_BOOT1) |            \
                                     PIN_ODR_HIGH(GPIOB_SWO) |              \
                                     PIN_ODR_HIGH(GPIOB_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOB_PIN5) |       \
                                     PIN_ODR_HIGH(GPIOB_I2C1_SCL) |        \
                                     PIN_ODR_HIGH(GPIOB_I2C1_SDA) |             \
                                     PIN_ODR_HIGH(GPIOB_PIN8) |           \
                                     PIN_ODR_HIGH(GPIOB_PIN9) |           \
                                     PIN_ODR_HIGH(GPIOB_UART3_TX) |           \
                                     PIN_ODR_HIGH(GPIOB_UART3_RX) |           \
                                     PIN_ODR_HIGH(GPIOB_FRAM_CS) |        \
                                     PIN_ODR_HIGH(GPIOB_SPI2_FRAM_CLK) |      \
                                     PIN_ODR_HIGH(GPIOB_SPI2_FRAM_MISO) |        \
                                     PIN_ODR_HIGH(GPIOB_SPI2_FRAM_MOSI))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_LED_PWM1,  0U) |       \
                                     PIN_AFIO_AF(GPIOB_PIN1,      0U) |       \
                                     PIN_AFIO_AF(GPIOB_BOOT1,     0U) |         \
                                     PIN_AFIO_AF(GPIOB_SWO,       0U) |           \
                                     PIN_AFIO_AF(GPIOB_PIN4,      0U) |          \
                                     PIN_AFIO_AF(GPIOB_PIN5,      0U) |   \
                                     PIN_AFIO_AF(GPIOB_I2C1_SCL,  4U) |    \
                                     PIN_AFIO_AF(GPIOB_I2C1_SDA,  4U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_PIN8,      0U) |       \
                                     PIN_AFIO_AF(GPIOB_PIN9,      0U) |       \
                                     PIN_AFIO_AF(GPIOB_UART3_TX,  0U) |       \
                                     PIN_AFIO_AF(GPIOB_UART3_RX,  0U) |       \
                                     PIN_AFIO_AF(GPIOB_FRAM_CS,   0U) |    \
                                     PIN_AFIO_AF(GPIOB_SPI2_FRAM_CLK,  5U) |   \
                                     PIN_AFIO_AF(GPIOB_SPI2_FRAM_MISO, 5U) |    \
                                     PIN_AFIO_AF(GPIOB_SPI2_FRAM_MOSI, 5U))

/*
 * GPIOC setup:
 *
 * PC0  - FMC_SDNWE                 (alternate 12).
 * PC1  - SPI5_MEMS_CS              (output pushpull maximum).
 * PC2  - SPI5_LCD_CS               (output pushpull maximum).
 * PC3  - PIN3                      (input pullup).
 * PC4  - OTG_HS_PSO                (output pushpull maximum).
 * PC5  - OTG_HS_OC                 (input floating).
 * PC6  - LCD_HSYNC                 (alternate 14).
 * PC7  - LCD_G6                    (alternate 14).
 * PC8  - PIN8                      (input pullup).
 * PC9  - I2C3_SDA                  (alternate 4).
 * PC10 - LCD_R2                    (alternate 14).
 * PC11 - PIN11                     (input pullup).
 * PC12 - PIN12                     (input pullup).
 * PC13 - PIN13                     (input pullup).
 * PC14 - OSC32_IN                  (input floating).
 * PC15 - OSC32_OUT                 (input floating).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_INPUT(0U) |  \
                                     PIN_MODE_INPUT(1U) |  \
                                     PIN_MODE_INPUT(2U) |   \
                                     PIN_MODE_INPUT(GPIOC_PIN3) |           \
                                     PIN_MODE_OUTPUT(GPIOC_MPU6000_CS) |    \
                                     PIN_MODE_INPUT(5U) |      \
                                     PIN_MODE_INPUT(6U) |  \
                                     PIN_MODE_INPUT(7U) |     \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_D0) |           \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_D1) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_D2) |     \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_D3) |          \
                                     PIN_MODE_ALTERNATE(GPIOC_SDIO_CK) |          \
                                     PIN_MODE_INPUT(GPIOC_CONF4) |          \
                                     PIN_MODE_INPUT(GPIOC_OSC32_IN) |       \
                                     PIN_MODE_INPUT(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(0U) |  \
                                     PIN_OTYPE_PUSHPULL(1U) |\
                                     PIN_OTYPE_PUSHPULL(2U) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_MPU6000_CS) | \
                                     PIN_OTYPE_PUSHPULL(5U) |  \
                                     PIN_OTYPE_PUSHPULL(6U) |  \
                                     PIN_OTYPE_PUSHPULL(7U) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D1) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_D3) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SDIO_CK) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_CONF4) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_IN) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_MEDIUM(0U) |     \
                                     PIN_OSPEED_MEDIUM(1U) |  \
                                     PIN_OSPEED_MEDIUM(2U) |   \
                                     PIN_OSPEED_MEDIUM(GPIOC_PIN3) |       \
                                     PIN_OSPEED_MEDIUM(GPIOC_MPU6000_CS) |    \
                                     PIN_OSPEED_MEDIUM(5U) |     \
                                     PIN_OSPEED_MEDIUM(6U) |     \
                                     PIN_OSPEED_MEDIUM(7U) |        \
                                     PIN_OSPEED_MEDIUM(8U) |       \
                                     PIN_OSPEED_MEDIUM(9U) |      \
                                     PIN_OSPEED_MEDIUM(10U) |        \
                                     PIN_OSPEED_MEDIUM(11U) |      \
                                     PIN_OSPEED_MEDIUM(12U) |      \
                                     PIN_OSPEED_MEDIUM(13U) |      \
                                     PIN_OSPEED_MEDIUM(GPIOC_OSC32_IN) |      \
                                     PIN_OSPEED_MEDIUM(GPIOC_OSC32_OUT))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(0U) |  \
                                     PIN_PUPDR_FLOATING(1U) |\
                                     PIN_PUPDR_FLOATING(2U) |\
                                     PIN_PUPDR_FLOATING(GPIOC_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOC_MPU6000_CS) | \
                                     PIN_PUPDR_FLOATING(5U) |  \
                                     PIN_PUPDR_FLOATING(6U) |  \
                                     PIN_PUPDR_FLOATING(7U) |     \
                                     PIN_PUPDR_FLOATING(8U) |         \
                                     PIN_PUPDR_FLOATING(9U) |   \
                                     PIN_PUPDR_FLOATING(10U) |     \
                                     PIN_PUPDR_FLOATING(11U) |        \
                                     PIN_PUPDR_FLOATING(12U) |        \
                                     PIN_PUPDR_FLOATING(13U) |        \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_IN) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_OUT))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(0U) |        \
                                     PIN_ODR_HIGH(1U) |     \
                                     PIN_ODR_HIGH(2U) |      \
                                     PIN_ODR_HIGH(GPIOC_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOC_MPU6000_CS) |       \
                                     PIN_ODR_HIGH(5U) |        \
                                     PIN_ODR_HIGH(6U) |        \
                                     PIN_ODR_HIGH(7U) |           \
                                     PIN_ODR_HIGH(8U) |             \
                                     PIN_ODR_HIGH(9U) |         \
                                     PIN_ODR_HIGH(10U) |           \
                                     PIN_ODR_HIGH(11U) |            \
                                     PIN_ODR_HIGH(12U) |            \
                                     PIN_ODR_HIGH(13U) |            \
                                     PIN_ODR_HIGH(GPIOC_OSC32_IN) |         \
                                     PIN_ODR_HIGH(GPIOC_OSC32_OUT))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(0U, 0U) |    \
                                     PIN_AFIO_AF(1U, 0U) |  \
                                     PIN_AFIO_AF(2U, 0U) |   \
                                     PIN_AFIO_AF(GPIOC_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_MPU6000_CS, 0U) |    \
                                     PIN_AFIO_AF(5U, 0U) |     \
                                     PIN_AFIO_AF(6U, 0U) |    \
                                     PIN_AFIO_AF(7U, 0U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(8U, 0U) |          \
                                     PIN_AFIO_AF(9U, 4U) |      \
                                     PIN_AFIO_AF(10U, 0U) |       \
                                     PIN_AFIO_AF(11U, 0U) |         \
                                     PIN_AFIO_AF(12U, 0U) |         \
                                     PIN_AFIO_AF(13U, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_OSC32_IN, 0U) |      \
                                     PIN_AFIO_AF(GPIOC_OSC32_OUT, 0U))

/*
 * GPIOD setup:
 *
 * PD0  - FMC_D2                    (alternate 12).
 * PD1  - FMC_D3                    (alternate 12).
 * PD2  - PIN2                      (input pullup).
 * PD3  - LCD_G7                    (alternate 14).
 * PD4  - PIN4                      (input pullup).
 * PD5  - PIN5                      (input pullup).
 * PD6  - LCD_B2                    (alternate 14).
 * PD7  - PIN7                      (input pullup).
 * PD8  - FMC_D13                   (alternate 12).
 * PD9  - FMC_D14                   (alternate 12).
 * PD10 - FMC_D15                   (alternate 12).
 * PD11 - LCD_TE                    (input floating).
 * PD12 - LCD_RDX                   (output pushpull maximum).
 * PD13 - LCD_WRX                   (output pushpull maximum).
 * PD14 - FMC_D0                    (alternate 12).
 * PD15 - FMC_D1                    (alternate 12).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_ALTERNATE(0U) |     \
                                     PIN_MODE_ALTERNATE(1U) |     \
                                     PIN_MODE_INPUT(2U) |           \
                                     PIN_MODE_INPUT(3U) |     \
                                     PIN_MODE_INPUT(GPIOD_PIN4) |           \
                                     PIN_MODE_ALTERNATE(5U) |           \
                                     PIN_MODE_ALTERNATE(6U) |     \
                                     PIN_MODE_INPUT(GPIOD_PIN7) |           \
                                     PIN_MODE_INPUT(8U) |    \
                                     PIN_MODE_INPUT(9U) |    \
                                     PIN_MODE_INPUT(10U) |    \
                                     PIN_MODE_INPUT(11U) |         \
                                     PIN_MODE_ALTERNATE(12U) |       \
                                     PIN_MODE_ALTERNATE(13U) |       \
                                     PIN_MODE_ALTERNATE(14U) |     \
                                     PIN_MODE_ALTERNATE(15U))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(0U) |     \
                                     PIN_OTYPE_PUSHPULL(1U) |     \
                                     PIN_OTYPE_PUSHPULL(2U) |       \
                                     PIN_OTYPE_PUSHPULL(3U) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(5U) |       \
                                     PIN_OTYPE_PUSHPULL(6U) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(8U) |    \
                                     PIN_OTYPE_PUSHPULL(9U) |    \
                                     PIN_OTYPE_PUSHPULL(10U) |    \
                                     PIN_OTYPE_PUSHPULL(11U) |     \
                                     PIN_OTYPE_PUSHPULL(12U) |    \
                                     PIN_OTYPE_PUSHPULL(13U) |    \
                                     PIN_OTYPE_PUSHPULL(14U) |     \
                                     PIN_OTYPE_PUSHPULL(15U))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_MEDIUM(0U) |        \
                                     PIN_OSPEED_MEDIUM(1U) |        \
                                     PIN_OSPEED_MEDIUM(GPIOD_SDIO_CMD) |       \
                                     PIN_OSPEED_MEDIUM(GPIOD_PIN3) |        \
                                     PIN_OSPEED_MEDIUM(GPIOD_PIN4) |       \
                                     PIN_OSPEED_MEDIUM(GPIOD_UART2_TX) |       \
                                     PIN_OSPEED_MEDIUM(GPIOD_UART2_RX) |        \
                                     PIN_OSPEED_MEDIUM(GPIOD_PIN7) |       \
                                     PIN_OSPEED_MEDIUM(GPIOD_PIN8) |       \
                                     PIN_OSPEED_MEDIUM(GPIOD_PIN9) |       \
                                     PIN_OSPEED_MEDIUM(GPIOD_PIN10) |       \
                                     PIN_OSPEED_MEDIUM(GPIOD_PIN11) |        \
                                     PIN_OSPEED_MEDIUM(GPIOD_PWM_CH5) |       \
                                     PIN_OSPEED_MEDIUM(GPIOD_PWM_CH6) |       \
                                     PIN_OSPEED_MEDIUM(GPIOD_PWM_CH7) |        \
                                     PIN_OSPEED_MEDIUM(GPIOD_PWM_CH8))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_FLOATING(0U) |     \
                                     PIN_PUPDR_FLOATING(1U) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_SDIO_CMD) |         \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN3) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_UART2_TX) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_UART2_RX) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN7) |         \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN8) |    \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN9) |    \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN10) |    \
                                     PIN_PUPDR_FLOATING(GPIOD_PIN11) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_PWM_CH5) |    \
                                     PIN_PUPDR_FLOATING(GPIOD_PWM_CH6) |    \
                                     PIN_PUPDR_FLOATING(GPIOD_PWM_CH7) |     \
                                     PIN_PUPDR_FLOATING(GPIOD_PWM_CH8))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(0U) |           \
                                     PIN_ODR_HIGH(1U) |           \
                                     PIN_ODR_HIGH(GPIOD_SDIO_CMD) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN3) |           \
                                     PIN_ODR_HIGH(GPIOD_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOD_UART2_TX) |             \
                                     PIN_ODR_HIGH(GPIOD_UART2_RX) |           \
                                     PIN_ODR_HIGH(GPIOD_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN8) |          \
                                     PIN_ODR_HIGH(GPIOD_PIN9) |          \
                                     PIN_ODR_HIGH(GPIOD_PIN10) |          \
                                     PIN_ODR_HIGH(GPIOD_PIN11) |           \
                                     PIN_ODR_HIGH(GPIOD_PWM_CH5) |          \
                                     PIN_ODR_HIGH(GPIOD_PWM_CH6) |          \
                                     PIN_ODR_HIGH(GPIOD_PWM_CH7) |           \
                                     PIN_ODR_HIGH(GPIOD_PWM_CH8))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(0U, 9U) |       \
                                     PIN_AFIO_AF(1U, 9U) |       \
                                     PIN_AFIO_AF(GPIOD_SDIO_CMD, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN3, 0U) |       \
                                     PIN_AFIO_AF(GPIOD_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_UART2_TX, 7U) |          \
                                     PIN_AFIO_AF(GPIOD_UART2_RX, 7U) |       \
                                     PIN_AFIO_AF(GPIOD_PIN7, 0U))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_PIN8, 0U) |      \
                                     PIN_AFIO_AF(GPIOD_PIN9, 0U) |      \
                                     PIN_AFIO_AF(GPIOD_PIN10, 0U) |      \
                                     PIN_AFIO_AF(GPIOD_PIN11, 0U) |        \
                                     PIN_AFIO_AF(GPIOD_PWM_CH5, 2U) |       \
                                     PIN_AFIO_AF(GPIOD_PWM_CH6, 2U) |       \
                                     PIN_AFIO_AF(GPIOD_PWM_CH7, 2U) |       \
                                     PIN_AFIO_AF(GPIOD_PWM_CH8, 2U))

/*
 * GPIOE setup:
 *
 * PE0  - FMC_NBL0                  (alternate 12).
 * PE1  - FMC_NBL1                  (alternate 12).
 * PE2  - PIN2                      (input pullup).
 * PE3  - PIN3                      (input pullup).
 * PE4  - PIN4                      (input pullup).
 * PE5  - PIN5                      (input pullup).
 * PE6  - PIN6                      (input pullup).
 * PE7  - FMC_D4                    (alternate 12).
 * PE8  - FMC_D5                    (alternate 12).
 * PE9  - FMC_D6                    (alternate 12).
 * PE10 - FMC_D7                    (alternate 12).
 * PE11 - FMC_D8                    (alternate 12).
 * PE12 - FMC_D9                    (alternate 12).
 * PE13 - FMC_D10                   (alternate 12).
 * PE14 - FMC_D11                   (alternate 12).
 * PE15 - FMC_D12                   (alternate 12).
 */
#define VAL_GPIOE_MODER             (PIN_MODE_ALTERNATE(GPIOE_UART8_RX) |   \
                                     PIN_MODE_ALTERNATE(GPIOE_UART8_TX) |   \
                                     PIN_MODE_INPUT(GPIOE_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOE_UART7_RX) |     \
                                     PIN_MODE_INPUT(GPIOE_UART7_TX) |     \
                                     PIN_MODE_ALTERNATE(GPIOE_PWM_CH1) |     \
                                     PIN_MODE_INPUT(GPIOE_PIN10) |     \
                                     PIN_MODE_ALTERNATE(GPIOE_PWM_CH2) |     \
                                     PIN_MODE_INPUT(GPIOE_PIN12) |     \
                                     PIN_MODE_ALTERNATE(GPIOE_PWM_CH3) |    \
                                     PIN_MODE_ALTERNATE(GPIOE_PWM_CH4) |    \
                                     PIN_MODE_INPUT(GPIOE_PIN15))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_UART8_RX) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_UART8_TX) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_UART7_RX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_UART7_TX) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PWM_CH1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN10) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PWM_CH2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN12) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PWM_CH3) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PWM_CH4) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN15))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_MEDIUM(GPIOE_UART8_RX) |      \
                                     PIN_OSPEED_MEDIUM(GPIOE_UART8_TX) |      \
                                     PIN_OSPEED_MEDIUM(GPIOE_PIN2) |       \
                                     PIN_OSPEED_MEDIUM(GPIOE_PIN3) |       \
                                     PIN_OSPEED_MEDIUM(GPIOE_PIN4) |       \
                                     PIN_OSPEED_MEDIUM(GPIOE_PIN5) |       \
                                     PIN_OSPEED_MEDIUM(GPIOE_PIN6) |       \
                                     PIN_OSPEED_MEDIUM(GPIOE_UART7_RX) |        \
                                     PIN_OSPEED_MEDIUM(GPIOE_UART7_TX) |        \
                                     PIN_OSPEED_MEDIUM(GPIOE_PWM_CH1) |        \
                                     PIN_OSPEED_MEDIUM(GPIOE_PIN10) |        \
                                     PIN_OSPEED_MEDIUM(GPIOE_PWM_CH2) |        \
                                     PIN_OSPEED_MEDIUM(GPIOE_PIN12) |        \
                                     PIN_OSPEED_MEDIUM(GPIOE_PWM_CH3) |       \
                                     PIN_OSPEED_MEDIUM(GPIOE_PWM_CH4) |       \
                                     PIN_OSPEED_MEDIUM(GPIOE_PIN15))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_PULLUP(GPIOE_UART8_RX) |   \
                                     PIN_PUPDR_PULLUP(GPIOE_UART8_TX) |   \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN2) |         \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN3) |         \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN4) |         \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN5) |         \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN6) |         \
                                     PIN_PUPDR_FLOATING(GPIOE_UART7_RX) |     \
                                     PIN_PUPDR_FLOATING(GPIOE_UART7_TX) |     \
                                     PIN_PUPDR_FLOATING(GPIOE_PWM_CH1) |     \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN10) |     \
                                     PIN_PUPDR_FLOATING(GPIOE_PWM_CH2) |     \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN12) |     \
                                     PIN_PUPDR_FLOATING(GPIOE_PWM_CH3) |    \
                                     PIN_PUPDR_FLOATING(GPIOE_PWM_CH4) |    \
                                     PIN_PUPDR_FLOATING(GPIOE_PIN15))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_UART8_RX) |         \
                                     PIN_ODR_HIGH(GPIOE_UART8_TX) |         \
                                     PIN_ODR_HIGH(GPIOE_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOE_UART7_RX) |           \
                                     PIN_ODR_HIGH(GPIOE_UART7_TX) |           \
                                     PIN_ODR_HIGH(GPIOE_PWM_CH1) |           \
                                     PIN_ODR_HIGH(GPIOE_PIN10) |           \
                                     PIN_ODR_HIGH(GPIOE_PWM_CH2) |           \
                                     PIN_ODR_HIGH(GPIOE_PIN12) |           \
                                     PIN_ODR_HIGH(GPIOE_PWM_CH3) |          \
                                     PIN_ODR_HIGH(GPIOE_PWM_CH4) |          \
                                     PIN_ODR_HIGH(GPIOE_PIN15))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_UART8_RX, 8U) |     \
                                     PIN_AFIO_AF(GPIOE_UART8_TX, 8U) |     \
                                     PIN_AFIO_AF(GPIOE_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_UART7_RX, 0U))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_UART7_TX, 0U) |       \
                                     PIN_AFIO_AF(GPIOE_PWM_CH1, 1U) |       \
                                     PIN_AFIO_AF(GPIOE_PIN10, 0U) |       \
                                     PIN_AFIO_AF(GPIOE_PWM_CH2, 1U) |       \
                                     PIN_AFIO_AF(GPIOE_PIN12, 0U) |       \
                                     PIN_AFIO_AF(GPIOE_PWM_CH3, 1U) |      \
                                     PIN_AFIO_AF(GPIOE_PWM_CH4, 1U) |      \
                                     PIN_AFIO_AF(GPIOE_PIN15, 0U))

/*
 * GPIOF setup:
 *
 * PF0  - FMC_A0                    (alternate 12).
 * PF1  - FMC_A1                    (alternate 12).
 * PF2  - FMC_A2                    (alternate 12).
 * PF3  - FMC_A3                    (alternate 12).
 * PF4  - FMC_A4                    (alternate 12).
 * PF5  - FMC_A5                    (alternate 12).
 * PF6  - PIN6                      (input pullup).
 * PF7  - LCD_DCX                   (alternate 5).
 * PF8  - SPI5_MISO                 (alternate 5).
 * PF9  - SPI5_MOSI                 (alternate 5).
 * PF10 - LCD_DE                    (alternate 14).
 * PF11 - FMC_SDNRAS                (alternate 12).
 * PF12 - FMC_A6                    (alternate 12).
 * PF13 - FMC_A7                    (alternate 12).
 * PF14 - FMC_A8                    (alternate 12).
 * PF15 - FMC_A9                    (alternate 12).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_ALTERNATE(GPIOF_I2C2_SDA) |     \
                                     PIN_MODE_ALTERNATE(GPIOF_I2C2_CLK) |     \
                                     PIN_MODE_INPUT(GPIOF_UART1_RTSN) |     \
                                     PIN_MODE_INPUT(GPIOF_CONF1) |     \
                                     PIN_MODE_INPUT(GPIOF_CONF2) |     \
                                     PIN_MODE_INPUT(GPIOF_CONF3) |     \
                                     PIN_MODE_INPUT(GPIOF_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN7) |    \
                                     PIN_MODE_INPUT(GPIOF_PIN8) |  \
                                     PIN_MODE_INPUT(GPIOF_PIN9) |  \
                                     PIN_MODE_INPUT(GPIOF_PIN10) |     \
                                     PIN_MODE_INPUT(GPIOF_PIN11) | \
                                     PIN_MODE_INPUT(GPIOF_CONF5) |     \
                                     PIN_MODE_INPUT(GPIOF_POWER_DELAY) |     \
                                     PIN_MODE_INPUT(GPIOF_POWER_DETECT) |     \
                                     PIN_MODE_OUTPUT(GPIOF_LED_GREEN))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOF_I2C2_SDA) |     \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_I2C2_CLK) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_UART1_RTSN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_CONF1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_CONF2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_CONF3) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN7) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN8) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN9) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN10) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN11) | \
                                     PIN_OTYPE_PUSHPULL(GPIOF_CONF5) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_POWER_DELAY) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_POWER_DETECT) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOF_LED_GREEN))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_MEDIUM(GPIOF_I2C2_SDA) |        \
                                     PIN_OSPEED_MEDIUM(GPIOF_I2C2_CLK) |        \
                                     PIN_OSPEED_MEDIUM(GPIOF_UART1_RTSN) |        \
                                     PIN_OSPEED_MEDIUM(GPIOF_CONF1) |        \
                                     PIN_OSPEED_MEDIUM(GPIOF_CONF2) |        \
                                     PIN_OSPEED_MEDIUM(GPIOF_CONF3) |        \
                                     PIN_OSPEED_MEDIUM(GPIOF_PIN6) |       \
                                     PIN_OSPEED_MEDIUM(GPIOF_PIN7) |       \
                                     PIN_OSPEED_MEDIUM(GPIOF_PIN8) |     \
                                     PIN_OSPEED_MEDIUM(GPIOF_PIN9) |     \
                                     PIN_OSPEED_MEDIUM(GPIOF_PIN10) |        \
                                     PIN_OSPEED_MEDIUM(GPIOF_PIN11) |    \
                                     PIN_OSPEED_MEDIUM(GPIOF_CONF5) |        \
                                     PIN_OSPEED_MEDIUM(GPIOF_POWER_DELAY) |        \
                                     PIN_OSPEED_MEDIUM(GPIOF_POWER_DETECT) |        \
                                     PIN_OSPEED_MEDIUM(GPIOF_LED_GREEN))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_I2C2_SDA) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_I2C2_CLK) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_UART1_RTSN) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_CONF1) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_CONF2) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_CONF3) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN6) |         \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN7) |    \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN8) |  \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN9) |  \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN10) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN11) | \
                                     PIN_PUPDR_FLOATING(GPIOF_CONF5) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_POWER_DELAY) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_POWER_DETECT) |     \
                                     PIN_PUPDR_FLOATING(GPIOF_LED_GREEN))
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(GPIOF_I2C2_SDA) |           \
                                     PIN_ODR_HIGH(GPIOF_I2C2_CLK) |           \
                                     PIN_ODR_HIGH(GPIOF_UART1_RTSN) |           \
                                     PIN_ODR_HIGH(GPIOF_CONF1) |           \
                                     PIN_ODR_HIGH(GPIOF_CONF2) |           \
                                     PIN_ODR_HIGH(GPIOF_CONF3) |           \
                                     PIN_ODR_HIGH(GPIOF_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN7) |          \
                                     PIN_ODR_HIGH(GPIOF_PIN8) |        \
                                     PIN_ODR_HIGH(GPIOF_PIN9) |        \
                                     PIN_ODR_HIGH(GPIOF_PIN10) |           \
                                     PIN_ODR_HIGH(GPIOF_PIN11) |       \
                                     PIN_ODR_HIGH(GPIOF_CONF5) |           \
                                     PIN_ODR_HIGH(GPIOF_POWER_DELAY) |           \
                                     PIN_ODR_HIGH(GPIOF_POWER_DETECT) |           \
                                     PIN_ODR_HIGH(GPIOF_LED_GREEN))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_I2C2_SDA, 4U) |       \
                                     PIN_AFIO_AF(GPIOF_I2C2_CLK, 4U) |       \
                                     PIN_AFIO_AF(GPIOF_UART1_RTSN, 0U) |       \
                                     PIN_AFIO_AF(GPIOF_CONF1, 0U) |       \
                                     PIN_AFIO_AF(GPIOF_CONF2, 0U) |       \
                                     PIN_AFIO_AF(GPIOF_CONF3, 0U) |       \
                                     PIN_AFIO_AF(GPIOF_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN7, 0U))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_PIN8, 0U) |     \
                                     PIN_AFIO_AF(GPIOF_PIN9, 0U) |     \
                                     PIN_AFIO_AF(GPIOF_PIN10, 0U) |       \
                                     PIN_AFIO_AF(GPIOF_PIN11, 0U) |   \
                                     PIN_AFIO_AF(GPIOF_CONF5, 0U) |       \
                                     PIN_AFIO_AF(GPIOF_POWER_DELAY,  0U) |       \
                                     PIN_AFIO_AF(GPIOF_POWER_DETECT, 0U) |       \
                                     PIN_AFIO_AF(GPIOF_LED_GREEN, 0U))

/*
 * GPIOG setup:
 *
 * PG0  - FMC_A10                   (alternate 12).
 * PG1  - FMC_A11                   (alternate 12).
 * PG2  - PIN2                      (input pullup).
 * PG3  - PIN3                      (input pullup).
 * PG4  - FMC_BA0                   (alternate 12).
 * PG5  - FMC_BA1                   (alternate 12).
 * PG6  - LCD_R7                    (alternate 14).
 * PG7  - LCD_CLK                   (alternate 14).
 * PG8  - FMC_SDCLK                 (alternate 12).
 * PG9  - PIN9                      (input pullup).
 * PG10 - LCD_G3                    (alternate 14).
 * PG11 - LCD_B3                    (alternate 14).
 * PG12 - LCD_B4                    (alternate 14).
 * PG13 - LED3_GREEN                (output pushpull maximum).
 * PG14 - LED4_RED                  (output pushpull maximum).
 * PG15 - FMC_SDNCAS                (alternate 12).
 */
#define VAL_GPIOG_MODER             (PIN_MODE_OUTPUT(GPIOG_LED_AMBER) |    \
                                     PIN_MODE_INPUT(GPIOG_MPU_DRDY) |    \
                                     PIN_MODE_OUTPUT(GPIOG_LED_SAFETY) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOG_T_SEN) |    \
                                     PIN_MODE_OUTPUT(GPIOG_LED_BLUE) |    \
                                     PIN_MODE_INPUT(GPIOG_PIN6) |     \
                                     PIN_MODE_INPUT(GPIOG_PIN7) |    \
                                     PIN_MODE_INPUT(GPIOG_PIN8) |  \
                                     PIN_MODE_ALTERNATE(GPIOG_UART6_RX) |           \
                                     PIN_MODE_INPUT(GPIOG_PIN10) |     \
                                     PIN_MODE_INPUT(GPIOG_PIN11) |     \
                                     PIN_MODE_INPUT(GPIOG_PIN12) |     \
                                     PIN_MODE_INPUT(GPIOG_PIN13) |    \
                                     PIN_MODE_ALTERNATE(GPIOG_UART6_TX) |      \
                                     PIN_MODE_INPUT(GPIOG_PIN15))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOG_LED_AMBER) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_MPU_DRDY) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_LED_SAFETY) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_T_SEN) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_LED_BLUE) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN6) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN7) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN8) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOG_UART6_RX) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN10) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN11) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN12) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN13) | \
                                     PIN_OTYPE_PUSHPULL(GPIOG_UART6_TX) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN15))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_MEDIUM(GPIOG_LED_AMBER) |       \
                                     PIN_OSPEED_MEDIUM(GPIOG_MPU_DRDY) |       \
                                     PIN_OSPEED_MEDIUM(GPIOG_LED_SAFETY) |       \
                                     PIN_OSPEED_MEDIUM(GPIOG_PIN3) |       \
                                     PIN_OSPEED_MEDIUM(GPIOG_T_SEN) |       \
                                     PIN_OSPEED_MEDIUM(GPIOG_LED_BLUE) |       \
                                     PIN_OSPEED_MEDIUM(GPIOG_PIN6) |        \
                                     PIN_OSPEED_MEDIUM(GPIOG_PIN7) |       \
                                     PIN_OSPEED_MEDIUM(GPIOG_PIN8) |     \
                                     PIN_OSPEED_MEDIUM(GPIOG_UART6_RX) |       \
                                     PIN_OSPEED_MEDIUM(GPIOG_PIN10) |        \
                                     PIN_OSPEED_MEDIUM(GPIOG_PIN11) |        \
                                     PIN_OSPEED_MEDIUM(GPIOG_PIN12) |        \
                                     PIN_OSPEED_MEDIUM(GPIOG_PIN13) |    \
                                     PIN_OSPEED_MEDIUM(GPIOG_UART6_TX) |      \
                                     PIN_OSPEED_MEDIUM(GPIOG_PIN15))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_FLOATING(GPIOG_LED_AMBER) |    \
                                     PIN_PUPDR_FLOATING(GPIOG_MPU_DRDY) |    \
                                     PIN_PUPDR_FLOATING(GPIOG_LED_SAFETY) |         \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN3) |         \
                                     PIN_PUPDR_FLOATING(GPIOG_T_SEN) |    \
                                     PIN_PUPDR_FLOATING(GPIOG_LED_BLUE) |    \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN6) |     \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN7) |    \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN8) |  \
                                     PIN_PUPDR_PULLUP(GPIOG_UART6_RX) |         \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN10) |     \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN11) |     \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN12) |     \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN13) | \
                                     PIN_PUPDR_PULLUP(GPIOG_UART6_TX) |   \
                                     PIN_PUPDR_FLOATING(GPIOG_PIN15))
#define VAL_GPIOG_ODR               (PIN_ODR_HIGH(GPIOG_LED_AMBER) |          \
                                     PIN_ODR_HIGH(GPIOG_MPU_DRDY) |          \
                                     PIN_ODR_HIGH(GPIOG_LED_SAFETY) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOG_T_SEN) |          \
                                     PIN_ODR_HIGH(GPIOG_LED_BLUE) |          \
                                     PIN_ODR_HIGH(GPIOG_PIN6) |           \
                                     PIN_ODR_HIGH(GPIOG_PIN7) |          \
                                     PIN_ODR_HIGH(GPIOG_PIN8) |        \
                                     PIN_ODR_HIGH(GPIOG_UART6_RX) |             \
                                     PIN_ODR_HIGH(GPIOG_PIN10) |           \
                                     PIN_ODR_HIGH(GPIOG_PIN11) |           \
                                     PIN_ODR_HIGH(GPIOG_PIN12) |           \
                                     PIN_ODR_HIGH(GPIOG_PIN13) |        \
                                     PIN_ODR_HIGH(GPIOG_UART6_TX) |          \
                                     PIN_ODR_HIGH(GPIOG_PIN15))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_LED_AMBER, 0U) |      \
                                     PIN_AFIO_AF(GPIOG_MPU_DRDY, 0U) |      \
                                     PIN_AFIO_AF(GPIOG_LED_SAFETY, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_T_SEN, 0U) |      \
                                     PIN_AFIO_AF(GPIOG_LED_BLUE, 0) |      \
                                     PIN_AFIO_AF(GPIOG_PIN6, 0U) |       \
                                     PIN_AFIO_AF(GPIOG_PIN7, 0U))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_PIN8, 0U) |    \
                                     PIN_AFIO_AF(GPIOG_UART6_RX, 8U) |          \
                                     PIN_AFIO_AF(GPIOG_PIN10, 0U) |       \
                                     PIN_AFIO_AF(GPIOG_PIN11, 0U) |       \
                                     PIN_AFIO_AF(GPIOG_PIN12, 0U) |       \
                                     PIN_AFIO_AF(GPIOG_PIN13, 0U) |    \
                                     PIN_AFIO_AF(GPIOG_UART6_TX, 8U) |      \
                                     PIN_AFIO_AF(GPIOG_PIN15, 0U))

/*
 * GPIOH setup:
 *
 * PH0  - OSC_IN                    (input floating).
 * PH1  - OSC_OUT                   (input floating).
 * PH2  - PIN2                      (input pullup).
 * PH3  - PIN3                      (input pullup).
 * PH4  - PIN4                      (input pullup).
 * PH5  - PIN5                      (input pullup).
 * PH6  - PIN6                      (input pullup).
 * PH7  - PIN7                      (input pullup).
 * PH8  - PIN8                      (input pullup).
 * PH9  - PIN9                      (input pullup).
 * PH10 - PIN10                     (input pullup).
 * PH11 - PIN11                     (input pullup).
 * PH12 - PIN12                     (input pullup).
 * PH13 - PIN13                     (input pullup).
 * PH14 - PIN14                     (input pullup).
 * PH15 - PIN15                     (input pullup).
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN) |         \
                                     PIN_MODE_INPUT(GPIOH_OSC_OUT) |        \
                                     PIN_MODE_INPUT(GPIOH_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN15))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_OSC_IN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOH_OSC_OUT) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN15))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_HIGH(GPIOH_OSC_IN) |        \
                                     PIN_OSPEED_HIGH(GPIOH_OSC_OUT) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN15))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_FLOATING(GPIOH_OSC_IN) |     \
                                     PIN_PUPDR_FLOATING(GPIOH_OSC_OUT) |    \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN15))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_OSC_IN) |           \
                                     PIN_ODR_HIGH(GPIOH_OSC_OUT) |          \
                                     PIN_ODR_HIGH(GPIOH_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN15))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_OSC_IN, 0U) |        \
                                     PIN_AFIO_AF(GPIOH_OSC_OUT, 0U) |       \
                                     PIN_AFIO_AF(GPIOH_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN7, 0U))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(GPIOH_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN15, 0U))

/*
 * GPIOI setup:
 *
 * PI0  - PIN0                      (input pullup).
 * PI1  - PIN1                      (input pullup).
 * PI2  - PIN2                      (input pullup).
 * PI3  - PIN3                      (input pullup).
 * PI4  - PIN4                      (input pullup).
 * PI5  - PIN5                      (input pullup).
 * PI6  - PIN6                      (input pullup).
 * PI7  - PIN7                      (input pullup).
 * PI8  - PIN8                      (input pullup).
 * PI9  - PIN9                      (input pullup).
 * PI10 - PIN10                     (input pullup).
 * PI11 - PIN11                     (input pullup).
 * PI12 - PIN12                     (input pullup).
 * PI13 - PIN13                     (input pullup).
 * PI14 - PIN14                     (input pullup).
 * PI15 - PIN15                     (input pullup).
 */
#define VAL_GPIOI_MODER             (PIN_MODE_INPUT(GPIOI_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN15))
#define VAL_GPIOI_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOI_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN15))
#define VAL_GPIOI_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOI_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN15))
#define VAL_GPIOI_PUPDR             (PIN_PUPDR_PULLUP(GPIOI_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN15))
#define VAL_GPIOI_ODR               (PIN_ODR_HIGH(GPIOI_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN15))
#define VAL_GPIOI_AFRL              (PIN_AFIO_AF(GPIOI_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN7, 0U))
#define VAL_GPIOI_AFRH              (PIN_AFIO_AF(GPIOI_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN15, 0U))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* BOARD_H */
