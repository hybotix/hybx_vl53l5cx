/*
 * platform.cpp
 * Hybrid RobotiX — hybx_vl53l5cx
 * Dale Weber <hybotix@hybridrobotix.io>
 *
 * ST ULD platform adaptation — Arduino Wire1 implementation.
 *
 * WHY WIRE1?
 * ----------
 * Zephyr native i2c_transfer() hangs indefinitely during the VL53L5CX
 * firmware upload when Arduino RouterBridge is running. Wire1 works
 * correctly with the Bridge — confirmed by vl53-diag.
 *
 * CRITICAL: Wire1.begin() must be called BEFORE Bridge.begin() in setup().
 * Calling Wire1.begin() after Bridge.begin() hangs the MCU permanently.
 *
 * WIRE1 LIMITS:
 * - Write: Wire1.write(buf, len) streams bytes with no size limit.
 * - Read:  Wire1.requestFrom() is limited to 256 bytes per call.
 *          RdMulti chunks reads at HYBX_I2C_RD_CHUNK bytes.
 *
 * I2C TRANSACTION DESIGN:
 * WrByte:  beginTransmission + write(reg_hi, reg_lo, val) + endTransmission
 * WrMulti: beginTransmission + write(reg_hi, reg_lo) + write(buf, len) + endTransmission
 *          (chunked for firmware pages — address stays 0x0000 per chunk)
 * RdByte:  beginTransmission + write(reg) + endTransmission(false) + requestFrom(1)
 * RdMulti: same pattern, chunked at 256 bytes with incrementing address
 *
 * License: MIT (our code) — ST ULD files carry BSD 3-clause.
 */

#include "platform.h"
#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include <zephyr/kernel.h>   /* k_yield() */

/* Maximum bytes per RdMulti chunk — Wire1.requestFrom() limit is 255 */
#define HYBX_I2C_RD_CHUNK  255U

/* -------------------------------------------------------------------------
 * WrByte — write one byte to RegisterAddress via Wire1.
 * -------------------------------------------------------------------------*/
extern "C" uint8_t WrByte(VL53L5CX_Platform *p_platform,
                           uint16_t RegisterAddress, uint8_t value)
{
    Wire1.beginTransmission((uint8_t)p_platform->address);
    Wire1.write((uint8_t)(RegisterAddress >> 8));
    Wire1.write((uint8_t)(RegisterAddress & 0xFF));
    Wire1.write(value);
    return Wire1.endTransmission() == 0 ? 0 : 1;
}

/* -------------------------------------------------------------------------
 * RdByte — read one byte from RegisterAddress via Wire1.
 * -------------------------------------------------------------------------*/
extern "C" uint8_t RdByte(VL53L5CX_Platform *p_platform,
                           uint16_t RegisterAddress, uint8_t *p_value)
{
    Wire1.beginTransmission((uint8_t)p_platform->address);
    Wire1.write((uint8_t)(RegisterAddress >> 8));
    Wire1.write((uint8_t)(RegisterAddress & 0xFF));
    if (Wire1.endTransmission(false) != 0) return 1;
    Wire1.requestFrom((uint8_t)p_platform->address, (uint8_t)1);
    if (!Wire1.available()) return 1;
    *p_value = Wire1.read();
    return 0;
}

/* -------------------------------------------------------------------------
 * WrMulti — write size bytes from p_values to RegisterAddress via Wire1.
 *
 * Firmware page writes (RegisterAddress == 0, size > 1024):
 *   The VL53L5CX page memory is a streaming buffer. The write pointer
 *   auto-advances — resending address 0x0000 does NOT reset it. Each
 *   chunk sends address 0x0000 so the sensor continues streaming.
 *
 * Normal register writes (all other calls):
 *   Address increments by chunk size across chunks.
 * Wire1's internal TX buffer is 256 bytes. With 2-byte address sent as a
 * separate transaction, the data transaction can be up to 254 bytes.
 * -------------------------------------------------------------------------*/
#define HYBX_I2C_WR_CHUNK  254U

extern "C" uint8_t WrMulti(VL53L5CX_Platform *p_platform,
                            uint16_t RegisterAddress,
                            uint8_t *p_values, uint32_t size)
{
    /* Wire1 TX ring buffer is 256 bytes total.
     * Each transaction: 2 bytes address + up to 254 bytes data.
     * Address and data MUST be in the same beginTransmission/endTransmission
     * cycle — endTransmission() ignores the stopBit parameter and always
     * issues a STOP, so two-transaction approach doesn't work. */
    bool     streaming = (RegisterAddress == 0 && size > 1024U);
    uint32_t offset    = 0;

    while (offset < size) {
        uint32_t chunk = size - offset;
        if (chunk > HYBX_I2C_WR_CHUNK) chunk = HYBX_I2C_WR_CHUNK;

        uint16_t chunkAddr = streaming ? 0 : (RegisterAddress + (uint16_t)offset);

        Wire1.beginTransmission((uint8_t)p_platform->address);
        Wire1.write((uint8_t)(chunkAddr >> 8));
        Wire1.write((uint8_t)(chunkAddr & 0xFF));
        Wire1.write(p_values + offset, (size_t)chunk);
        if (Wire1.endTransmission() != 0) return 1;

        offset += chunk;
        k_msleep(1);
    }
    return 0;
}

/* -------------------------------------------------------------------------
 * RdMulti — read size bytes from RegisterAddress via Wire1.
 *
 * Chunked at HYBX_I2C_RD_CHUNK (255) bytes — Wire1.requestFrom() limit.
 * Address increments across chunks.
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

        Wire1.beginTransmission((uint8_t)p_platform->address);
        Wire1.write((uint8_t)(chunkAddr >> 8));
        Wire1.write((uint8_t)(chunkAddr & 0xFF));
        if (Wire1.endTransmission(false) != 0) {
            memset(p_values + offset, 0xFF, chunk);
            return 1;
        }

        Wire1.requestFrom((uint8_t)p_platform->address, (uint8_t)chunk);
        for (uint32_t i = 0; i < chunk; i++) {
            if (!Wire1.available()) {
                memset(p_values + offset + i, 0xFF, chunk - i);
                return 1;
            }
            p_values[offset + i] = Wire1.read();
        }

        offset += chunk;
    }
    return 0;
}

/* -------------------------------------------------------------------------
 * SwapBuffer — big-endian <-> little-endian word swap in place.
 * Size is always a multiple of 4.
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
    k_yield();
    return 0;
}
