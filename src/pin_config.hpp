
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
#define LS_SDA_PIN       26
#define LS_SCL_PIN       27

#define INA260_I2C_BUS      i2c1
#define INA260_SDA_PIN      14
#define INA260_SCL_PIN      15
#define INA260_EN           22


// ── UART (Iridium) ───────────────────────────────────
#define IRIDIUM_UART     uart1
#define IRIDIUM_TX_PIN   4
#define IRIDIUM_RX_PIN   5

// ── Iridium control pins ─────────────────────────────
#define IRIDIUM_ONOFF_PIN   14
#define IRIDIUM_NETAVB_PIN   2    // network available input
#define IRIDIUM_RI_PIN       3    // ring indicator input
#define IRIDIUM_CTS_PIN      6
#define IRIDIUM_RTS_PIN      7 

#define LED_PIN           25