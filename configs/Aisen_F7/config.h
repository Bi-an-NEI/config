/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define FC_TARGET_MCU     STM32F7X2

#define BOARD_NAME        Aisen_F7
#define MANUFACTURER_ID   Aisen

#define LED0_PIN             PC13

#define BEEPER               PC3
#define BEEPER_INVERTED

/************************ACC GYRO SPI1***************************/
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN         PA5
#define SPI1_SDI_PIN         PA6
#define SPI1_SDO_PIN         PA7

#define USE_ACC
#define USE_ACC_SPI_ICM42688P

#define USE_GYRO
#define USE_GYRO_SPI_ICM42688P

#define GYRO_1_SPI_INSTANCE SPI1
#define GYRO_1_EXTI_PIN      PA4
#define GYRO_1_CS_PIN        PC4

//#define USE_SPI_GYRO
#define GYRO_1_ALIGN CW270_DEG


/************************FLASH SPI2******************************/
#define SPI2_SCK_PIN         PB13
#define SPI2_SDI_PIN         PB14
#define SPI2_SDO_PIN         PB15

#define USE_FLASH
#define USE_FLASH_W25Q128FV
#define USE_FLASH_M25P16
#define USE_FLASH_W25N01G
#define USE_FLASH_SPI
#define DEFAULT_BLACKBOX_DEVICE     BLACKBOX_DEVICE_FLASH

#define FLASH_CS_PIN         PB12
#define FLASH_SPI_INSTANCE   SPI2


/************************BARO MAX7456 SPI3***********************/
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PB3
#define SPI3_SDI_PIN            PB4
#define SPI3_SDO_PIN            PB5

#define USE_DMA
#define SPI3_TX_DMA_OPT         1
#define USE_BARO
#define USE_BARO_SPI_BMP280
#define BARO_CS_PIN             PB7
#define BARO_SPI_INSTANCE       SPI3

#define USE_MAX7456
#define MAX7456_SPI_CS_PIN      PA8
#define MAX7456_SPI_INSTANCE    SPI3
/********************************MOTOR***************************/
#define MOTOR1_PIN           PB11
#define MOTOR2_PIN           PB10
#define MOTOR3_PIN           PB1
#define MOTOR4_PIN           PB0
#define MOTOR5_PIN           PC9
#define MOTOR6_PIN           PC8
#define MOTOR7_PIN           PC7
#define MOTOR8_PIN           PC6


/****************************UART*********************************/
#define USE_UART1
#define UART1_TX_PIN         PA9
#define UART1_RX_PIN         PA10

#define USE_UART2
#define UART2_TX_PIN         PA2
#define UART2_RX_PIN         PA3

#define USE_UART3
#define UART3_TX_PIN         PC10
#define UART3_RX_PIN         PC11

#define USE_UART4
#define UART4_TX_PIN         PA0
#define UART4_RX_PIN         PA1

#define USE_UART5
#define UART5_TX_PIN         PA12
#define UART5_RX_PIN         PD2

/****************************I2C MAG*******************************/
#define USE_I2C
#define I2C1_SCL_PIN         PB8
#define I2C1_SDA_PIN         PB9
#define DASHBOARD_I2C_INSTANCE  (I2CDEV_1)

#define USE_MAG
#define MAG_I2C_INSTANCE        (I2CDEV_1)

/******************************************************************/
//#define ADC_INSTANCE         ADC1
#define USE_ADC
#define ADC_VBAT_PIN         PC0
#define ADC_CURR_PIN         PC1

#define LED_STRIP_PIN        PA15

#define ADC1_DMA_OPT        1

#define SYSTEM_HSE_MHZ       8

#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC

#define USE_GPS
#define USE_GPS_NMEA
#define USE_GPS_UBLOX
#define USE_GPS_RESCUE

#define DEFAULT_DSHOT_BURST DSHOT_DMAR_ON

/***************************TIMER_MAP******************************/
#define TIMER_PIN_MAPPING \
    TIMER_PIN_MAP( 0, PA15, 2, 0) \
    TIMER_PIN_MAP( 1, PB11, 1, 1) \
    TIMER_PIN_MAP( 2, PB10 , 1,  0) \
    TIMER_PIN_MAP( 3, PB1 , 2,  1) \
    TIMER_PIN_MAP( 4, PB0 , 2,  0) \
    TIMER_PIN_MAP( 5, PC9 , 2,  0) \
    TIMER_PIN_MAP( 6, PC8 , 2,  0) \
    TIMER_PIN_MAP( 7, PC7 , 2,  0) \
    TIMER_PIN_MAP( 8, PC6 , 2,  0)