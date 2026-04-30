/*
 * platform.cpp
 * Hybrid RobotiX — hybx_vl53l5cx
 * Dale Weber <hybotix@hybridrobotix.io>
 *
 * ST ULD platform adaptation — Zephyr native I2C implementation.
 *
 * WHY ZEPHYR NATIVE I2C INSTEAD OF WIRE1?
 * ----------------------------------------
 * Wire1.endTransmission() wraps i2c_write() which holds the I2C device
 * mutex for the entire transfer. When called from a Zephyr thread doing
 * heavy I2C work (firmware upload), the Bridge UART thread times out.
 *
 * Zephyr native i2c_transfer() uses a completion semaphore — it yields
 * to other threads (including the Bridge update thread) between transfers.
 * This allows the Bridge to respond while firmware upload is in progress.
 *
 * CRITICAL: Wire1.begin() must still be called BEFORE Bridge.begin()
 * to initialize the I2C peripheral. After that, we use i2c_transfer()
 * directly for all sensor communication.
 *
 * CHUNK SIZE: Wire1 TX buffer is 256 bytes. With 2-byte address prefix,
 * maximum data per chunk is 254 bytes. i2c_transfer() has no size limit
 * but we chunk to match the VL53L5CX page structure.
 *
 * License: MIT (our code) — ST ULD files carry BSD 3-clause.
 */

#include "platform.h"
#include <Arduino.h>
#include <Wire.h>
#include <zephyr/drivers/i2c.h>
#include <string.h>

/* Maximum bytes per WrMulti chunk — 254 bytes data + 2 bytes address = 256 */
#define HYBX_I2C_WR_CHUNK  254U

/* Maximum bytes per RdMulti chunk — Wire1 requestFrom() limit */
#define HYBX_I2C_RD_CHUNK  255U

/* Get the Zephyr I2C device from Wire1 */
static const struct device *get_i2c_dev() {
    /* Wire1 is a ZephyrI2C object — its i2c_dev field is the Zephyr device.
     * We access it via the DT label directly instead. */
    return DEVICE_DT_GET(DT_NODELABEL(i2c4));
}

/* -------------------------------------------------------------------------
 * WrByte — write one byte to RegisterAddress.
 * -------------------------------------------------------------------------*/
extern "C" uint8_t WrByte(VL53L5CX_Platform *p_platform,
                           uint16_t RegisterAddress, uint8_t value)
{
    uint8_t buf[3] = {
        (uint8_t)(RegisterAddress >> 8),
        (uint8_t)(RegisterAddress & 0xFF),
        value
    };
    return i2c_write(get_i2c_dev(), buf, 3, (uint16_t)p_platform->address) == 0 ? 0 : 1;
}

/* -------------------------------------------------------------------------
 * RdByte — read one byte from RegisterAddress.
 * -------------------------------------------------------------------------*/
extern "C" uint8_t RdByte(VL53L5CX_Platform *p_platform,
                           uint16_t RegisterAddress, uint8_t *p_value)
{
    uint8_t reg[2] = {
        (uint8_t)(RegisterAddress >> 8),
        (uint8_t)(RegisterAddress & 0xFF)
    };
    return i2c_write_read(get_i2c_dev(), (uint16_t)p_platform->address,
                          reg, 2, p_value, 1) == 0 ? 0 : 1;
}

/* -------------------------------------------------------------------------
 * WrMulti — write size bytes to RegisterAddress in 254-byte chunks.
 *
 * Firmware pages (RegisterAddress==0, size>1024): streaming mode,
 * address stays 0x0000 for each chunk.
 * Normal registers: address increments per chunk.
 * -------------------------------------------------------------------------*/
extern "C" uint8_t WrMulti(VL53L5CX_Platform *p_platform,
                            uint16_t RegisterAddress,
                            uint8_t *p_values, uint32_t size)
{
    bool     streaming = (RegisterAddress == 0 && size > 1024U);
    uint32_t offset    = 0;
    static uint8_t txbuf[HYBX_I2C_WR_CHUNK + 2];

    while (offset < size) {
        uint32_t chunk = size - offset;
        if (chunk > HYBX_I2C_WR_CHUNK) chunk = HYBX_I2C_WR_CHUNK;

        uint16_t chunkAddr = streaming ? 0 : (RegisterAddress + (uint16_t)offset);

        txbuf[0] = (uint8_t)(chunkAddr >> 8);
        txbuf[1] = (uint8_t)(chunkAddr & 0xFF);
        memcpy(&txbuf[2], p_values + offset, chunk);

        if (i2c_write(get_i2c_dev(), txbuf, chunk + 2,
                      (uint16_t)p_platform->address) != 0) return 1;

        offset += chunk;
    }
    return 0;
}

/* -------------------------------------------------------------------------
 * RdMulti — read size bytes from RegisterAddress in 255-byte chunks.
 * -------------------------------------------------------------------------*/
extern "C" uint8_t RdMulti(VL53L5CX_Platform *p_platform,
                            uint16_t RegisterAddress,
                            uint8_t *p_values, uint32_t size)
{
    uint32_t offset = 0;

    while (offset < size) {
        uint32_t chunk = size - offset;
        if (chunk > HYBX_I2C_RD_CHUNK) chunk = HYBX_I2C_RD_CHUNK;

        uint16_t chunkAddr = RegisterAddress + (uint16_t)offset;
        uint8_t reg[2] = {
            (uint8_t)(chunkAddr >> 8),
            (uint8_t)(chunkAddr & 0xFF)
        };

        if (i2c_write_read(get_i2c_dev(), (uint16_t)p_platform->address,
                           reg, 2, p_values + offset, chunk) != 0) {
            memset(p_values + offset, 0xFF, chunk);
            return 1;
        }
        offset += chunk;
    }
    return 0;
}

/* -------------------------------------------------------------------------
 * SwapBuffer — big-endian <-> little-endian word swap in place.
 * -------------------------------------------------------------------------*/
extern "C" void SwapBuffer(uint8_t *buffer, uint16_t size)
{
    uint32_t tmp;
    for (uint16_t i = 0; i < size; i += 4) {
        tmp = ((uint32_t)buffer[i    ] << 24)
            | ((uint32_t)buffer[i + 1] << 16)
            | ((uint32_t)buffer[i + 2] <<  8)
            | ((uint32_t)buffer[i + 3]);
        memcpy(&buffer[i], &tmp, 4);
    }
}

/* -------------------------------------------------------------------------
 * WaitMs — blocking delay.
 * -------------------------------------------------------------------------*/
extern "C" uint8_t WaitMs(VL53L5CX_Platform *p_platform, uint32_t TimeMs)
{
    (void)p_platform;
    delay(TimeMs);
    return 0;
}
