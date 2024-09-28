#pragma once

#include <Arduino.h>

#define SPI_MOSI    GPIO_NUM_35
#define SPI_MISO    GPIO_NUM_37
#define SPI_SCK     GPIO_NUM_36

#define I2C_SDA     GPIO_NUM_17
#define I2C_SCL     GPIO_NUM_18

#define LED_CLK     GPIO_NUM_10
#define LED_DATA    GPIO_NUM_9

#define BUZZER      GPIO_NUM_8

#define RADIO_CS    GPIO_NUM_11
#define RADIO_D0    GPIO_NUM_12
#define RADIO_RST   GPIO_NUM_13

#define HIG_CS      GPIO_NUM_34 /* 23 is schem label, but actually 34 */
#define IMU_CSB1    GPIO_NUM_21 
#define IMU_CSB2    GPIO_NUM_33 /* 22 in schem label, but actually 33 */

#define VCHAN1      GPIO_NUM_1
#define QCHAN1      GPIO_NUM_5
#define VCHAN2      GPIO_NUM_2
#define QCHAN2      GPIO_NUM_6
#define VCHAN3      GPIO_NUM_3
#define QCHAN3      GPIO_NUM_7

#define VDC         GPIO_NUM_4

#define CAN_RX      GPIO_NUM_38
#define CAN_TX      GPIO_NUM_45
#define CAN_SHDN    GPIO_NUM_14


