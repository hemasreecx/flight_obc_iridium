
#pragma once
#include "hardware/i2c.h"
#include "hardware/uart.h"

// ── I2C buses ────────────────────────────────────────
#define KX134_I2C_BUS      i2c0
#define KX134_SDA_PIN      12
#define KX134_SCL_PIN      13

#define MAG_I2C_BUS      i2c1
#define MAG_SDA_PIN      10
#define MAG_SCL_PIN      11

#define LS_I2C_BUS      i2c1
#define LS_SDA_PIN       10
#define LS_SCL_PIN       11

#define INA260_I2C_BUS      i2c1
#define INA260_SDA_PIN      10
#define INA260_SCL_PIN      11

// ── UART (Iridium) ───────────────────────────────────
#define IRIDIUM_UART     uart1
#define IRIDIUM_TX_PIN   4
#define IRIDIUM_RX_PIN   5

// ── Iridium optional control/status pins ─────────────
// Set to 0xFF when not wired (TX/RX + power only setup).
#define IRIDIUM_ONOFF_PIN   0xFF // 14
#define IRIDIUM_NETAVB_PIN  0xFF //2
#define IRIDIUM_RI_PIN      0xFF // 3
#define IRIDIUM_CTS_PIN     0xFF // 6
#define IRIDIUM_RTS_PIN     0xFF // 7

#define LED_PIN           25


#define GNSS_TX_PIN         0
#define GNSS_RX_PIN         1
#define GNSS_RST_PIN        16

// thermocouple _adc_en gp 29, addc2_interrupt gp24, adc_1_interrupt gp23, adc2_cs - 22, spi1,  adc_mosi - 27, miso- 28, sck - 26, cs-21
// buzzer pin - 9
//watchdog_ in - gp15, rst - 26
// lets make this way - when the transmission happens -> the buzzer will sound