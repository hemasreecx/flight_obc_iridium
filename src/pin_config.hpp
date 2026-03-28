// src/pin_config.hpp
#pragma once
#include "hardware/i2c.h"
#include "hardware/uart.h"

// ── I2C buses ────────────────────────────────────────
#define KX134_I2C_BUS      i2c1
#define KX134_SDA_PIN      10
#define KX134_SCL_PIN      11

#define MAG_I2C_BUS      i2c0
#define MAG_SDA_PIN      12
#define MAG_SCL_PIN      13

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