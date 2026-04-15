#include "kx134.hpp"
#include "pico/stdlib.h"
#include "pico/timeout_helper.h"
#include <cstdio>

#if KX134_DEBUG
#include <stdio.h>
#define KX134_LOG(...) printf(__VA_ARGS__)
#else
#define KX134_LOG(...)
#endif


constexpr int      KX134_MAX_RETRIES = 3;
constexpr uint32_t KX134_TIMEOUT_US  = 1000;   // 1ms per I2C operation



KX134::KX134(i2c_inst_t* i2c, uint8_t address)
    : _i2c(i2c),
      _address(address)
{
    
}



int16_t KX134::combine(uint8_t low, uint8_t high)
{
    return static_cast<int16_t>(
        static_cast<uint16_t>(high) << 8 | static_cast<uint16_t>(low)
    );
}

/* ============================================================
   WRITE REGISTER
   Sends [reg_addr, value] over I2C with timeout protection.
   Retries up to KX134_MAX_RETRIES times on failure.
   ============================================================ */

bool KX134::writeRegister(KX134_Register reg, uint8_t value)
{
    uint8_t buffer[2];
    buffer[0] = static_cast<uint8_t>(reg);
    buffer[1] = value;

    for (int attempt = 0; attempt < KX134_MAX_RETRIES; attempt++)
    {
        int ret = i2c_write_blocking_until(
            _i2c, _address, buffer, 2, false,
            make_timeout_time_us(KX134_TIMEOUT_US)
        );

        if (ret == 2)
            return true;

        sleep_us(100);
    }

    return false;
}


/* ============================================================
   READ REGISTER
   Uses repeated-start (nostop=true) to write reg address then
   read 1 byte. Both operations are timeout-protected.
   On write failure, sends a STOP to clean up the bus before
   retrying.
   ============================================================ */

bool KX134::readRegister(KX134_Register reg, uint8_t& value)
{
    uint8_t reg_addr = static_cast<uint8_t>(reg);

    for (int attempt = 0; attempt < KX134_MAX_RETRIES; attempt++)
    {
        // Write register address — nostop=true (repeated start)
        int ret = i2c_write_blocking_until(
            _i2c, _address, &reg_addr, 1, true,
            make_timeout_time_us(KX134_TIMEOUT_US)
        );

        if (ret != 1)
        {
            // Write failed — bus left without STOP, force cleanup
            i2c_write_blocking_until(
                _i2c, _address, &reg_addr, 1, false,
                make_timeout_time_us(KX134_TIMEOUT_US)
            );
            sleep_us(100);
            continue;
        }

        // Read 1 byte — nostop=false (send STOP after)
        ret = i2c_read_blocking_until(
            _i2c, _address, &value, 1, false,
            make_timeout_time_us(KX134_TIMEOUT_US)
        );

        if (ret == 1)
            return true;

        sleep_us(100);
    }

    return false;
}


/* ============================================================
   READ MULTIPLE REGISTERS
   Burst read of `length` bytes starting from `reg`.
   Both write and read are timeout-protected.
   ============================================================ */

bool KX134::readMulti(KX134_Register reg, uint8_t* buffer, uint16_t length)
{
    uint8_t reg_addr = static_cast<uint8_t>(reg);

    for (int attempt = 0; attempt < KX134_MAX_RETRIES; attempt++)
    {
        // Write register address — nostop=true (repeated start)
        int ret = i2c_write_blocking_until(
            _i2c, _address, &reg_addr, 1, true,
            make_timeout_time_us(KX134_TIMEOUT_US)
        );

        if (ret != 1)
        {
            // Force STOP to clean up bus
            i2c_write_blocking_until(
                _i2c, _address, &reg_addr, 1, false,
                make_timeout_time_us(KX134_TIMEOUT_US)
            );
            sleep_us(100);
            continue;
        }

        // Burst read — nostop=false (send STOP after)
        ret = i2c_read_blocking_until(
            _i2c, _address, buffer, length, false,
            make_timeout_time_us(KX134_TIMEOUT_US)
        );

        if (ret == static_cast<int>(length))
            return true;

        sleep_us(100);
    }

    return false;
}


/* ============================================================
   CHECK WHO_AM_I
   ============================================================ */

KX134_Status KX134::checkID()
{
    uint8_t id;

    if (!readRegister(KX134_Register::WHO_AM_I, id))
        return KX134_Status::ERR_COMM;

    if (id != KX134_WHO_AM_I_VALUE)
        return KX134_Status::ERR_INVALID_ID;

    return KX134_Status::OK;
}


/* ============================================================
   SOFT RESET
   Writes to CNTL2 reset bit, waits for boot, checks ID.
   ============================================================ */

KX134_Status KX134::reset()
{
    if (!writeRegister(KX134_Register::CNTL2, 0x80))
        return KX134_Status::ERR_COMM;

    sleep_ms(10);

    return checkID();
}


/* ============================================================
   DISABLE / ENABLE (PC1 bit in CNTL1) - we are just putting it in standby mode
   Must disable before changing config registers.
   ============================================================ */

KX134_Status KX134::disable()
{
    uint8_t reg;

    if (!readRegister(KX134_Register::CNTL1, reg))
        return KX134_Status::ERR_COMM;

    reg &= ~(0x80);   // clear PC1

    if (!writeRegister(KX134_Register::CNTL1, reg))
        return KX134_Status::ERR_COMM;

    return KX134_Status::OK;
}

KX134_Status KX134::enable()
{
    uint8_t reg;

    if (!readRegister(KX134_Register::CNTL1, reg))
        return KX134_Status::ERR_COMM;

    reg |= 0x80;   // set PC1

    if (!writeRegister(KX134_Register::CNTL1, reg))
        return KX134_Status::ERR_COMM;

    return KX134_Status::OK;
}


/* ============================================================
   SET RANGE
   ============================================================ */

KX134_Status KX134::setRange(KX134_Range range)
{
    if (disable() != KX134_Status::OK)
        return KX134_Status::ERR_COMM;

    uint8_t reg;

    if (!readRegister(KX134_Register::CNTL1, reg))
        return KX134_Status::ERR_COMM;

    reg &= ~(0x03 << 3);
    reg |= (static_cast<uint8_t>(range) << 3);

    if (!writeRegister(KX134_Register::CNTL1, reg))
        return KX134_Status::ERR_COMM;

    return enable();
}


/* ============================================================
   GET RANGE
   Reads directly from the CNTL1 hardware register over I2C.
   Range bits are at bits 4:3 of CNTL1.
   ============================================================ */

KX134_Status KX134::getRange(KX134_Range& range)
{
    uint8_t reg;

    if (!readRegister(KX134_Register::CNTL1, reg))
        return KX134_Status::ERR_COMM;

    range = static_cast<KX134_Range>((reg >> 3) & 0x03);

    return KX134_Status::OK;
}


/* ============================================================
   READ RAW ACCELEROMETER DATA (burst read of 6 bytes)
   ============================================================ */

KX134_Status KX134::readRaw(KX134_Raw& raw)
{
    uint8_t buffer[6];

    if (!readMulti(KX134_Register::XOUT_L, buffer, 6))
        return KX134_Status::ERR_COMM;

    raw.x = combine(buffer[0], buffer[1]);
    raw.y = combine(buffer[2], buffer[3]);
    raw.z = combine(buffer[4], buffer[5]);

    return KX134_Status::OK;
}


/* ============================================================
   DATA READY CHECK
   ============================================================ */

bool KX134::dataReady()
{
    uint8_t status;

    if (!readRegister(KX134_Register::INS2, status))
        return false;

    return (status & 0x10) != 0;
}
void KX134::TrimValues()
{
    // Read XADP, YADP, ZADP — these are the OTP offset trim registers
    uint8_t xadp_l, xadp_h = 0;
    uint8_t yadp_l, yadp_h = 0;
    uint8_t zadp_l, zadp_h = 0;

    readRegister(KX134_Register::XADP_L, xadp_l);
    readRegister(KX134_Register::XADP_H, xadp_h);
    readRegister(KX134_Register::YADP_L, yadp_l);
    readRegister(KX134_Register::YADP_H, yadp_h);
    readRegister(KX134_Register::ZADP_L, zadp_l);
    readRegister(KX134_Register::ZADP_H, zadp_h);

    int16_t xadp = (int16_t)((xadp_h << 8) | xadp_l);
    int16_t yadp = (int16_t)((yadp_h << 8) | yadp_l);
    int16_t zadp = (int16_t)((zadp_h << 8) | zadp_l);

    // printf("=== KX134 Factory Trim Values ===\n");
    KX134_LOG("X trim: %d\n", xadp);
    KX134_LOG("Y trim: %d\n", yadp);
    KX134_LOG("Z trim: %d\n", zadp);
    // printf("=================================\n");
    fflush(stdout);
}

/* ============================================================
   FIFO READ
   ============================================================ */

uint16_t KX134::readFIFO(KX134_Raw* buffer, uint16_t max_samples)
{
    uint8_t fifoCount;

    if (!readRegister(KX134_Register::BUF_STATUS_1, fifoCount))
        return 0;

    uint16_t samples = (fifoCount > max_samples) ? max_samples : fifoCount;

    for (uint16_t i = 0; i < samples; i++)
    {
        uint8_t data[6];

        if (!readMulti(KX134_Register::BUF_READ, data, 6))
            return i;

        buffer[i].x = combine(data[0], data[1]);
        buffer[i].y = combine(data[2], data[3]);
        buffer[i].z = combine(data[4], data[5]);
    }

    return samples;
}

bool KX134::fifoOverflow()
{
    uint8_t status;

    if (!readRegister(KX134_Register::BUF_STATUS_2, status))
        return false;

    return (status & 0x80) != 0;
}


/* ============================================================
   SELF TEST
   Writes 0xCA to SELF_TEST register, waits, reads COTR.
   Expected COTR value = 0x55 on pass.
   ============================================================ */

KX134_Status KX134::selfTest()
{
    // KX134_LOG("Starting self-test\n");

    if (!writeRegister(KX134_Register::SELF_TEST, 0xCA))
        return KX134_Status::ERR_COMM;

    sleep_ms(2);

    uint8_t cotr;

    if (!readRegister(KX134_Register::COTR, cotr))
        return KX134_Status::ERR_COMM;

    writeRegister(KX134_Register::SELF_TEST, 0x00);  // disable self-test

    if (cotr != 0x55)
    {
        // KX134_LOG("Self-test FAILED (cotr=0x%02X)\n", cotr);
        return KX134_Status::ERR_CONFIG;
    }

    // KX134_LOG("Self-test PASSED\n");
    return KX134_Status::OK;
}


/* ============================================================
   INIT
   Resets chip, applies full config, stores config in cache,
   then enables the device.
   ============================================================ */

KX134_Status KX134::init(const KX134_Config& config)
{
    if (reset() != KX134_Status::OK)
        return KX134_Status::ERR_COMM;

    if (disable() != KX134_Status::OK)
        return KX134_Status::ERR_COMM;

    uint8_t cntl1 = 0;

    // Bit 6: Performance mode
    cntl1 |= ((static_cast<uint8_t>(config.performance_mode) & 0x01) << 6);

    // Bit 5: Data Ready Interrupt
    if (config.enable_data_ready_interrupt)
        cntl1 |= (1 << 5);

    // Bits 4:3: Range (GSEL)
    cntl1 |= ((static_cast<uint8_t>(config.range) & 0x03) << 3);

    if (!writeRegister(KX134_Register::CNTL1, cntl1))
        return KX134_Status::ERR_CONFIG;

    uint8_t odcntl = 0;

    // Bit 5: FSTUP = 1
    odcntl |= (1 << 5);

    // Bits 3:0: ODR
    odcntl |= (static_cast<uint8_t>(config.odr) & 0x0F);

    if (!writeRegister(KX134_Register::ODCNTL, odcntl))
        return KX134_Status::ERR_CONFIG;

    uint8_t cntl6 = 0x07;   // 0b00000111
    if (!writeRegister(KX134_Register::CNTL6, cntl6))
        return KX134_Status::ERR_CONFIG;

    return enable();
}