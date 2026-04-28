/*
 * platform.cpp
 * Hybrid RobotiX — hybx_vl53l5cx
 * Dale Weber <hybotix@hybridrobotix.io>
 *
 * ST ULD platform adaptation — Zephyr native I2C implementation.
 *
 * NOTE: Function names (RdByte, WrByte, RdMulti, WrMulti, SwapBuffer,
 * WaitMs) are MANDATED by the ST Ultra Lite Driver API. They cannot be
 * renamed. These are our implementations of the required platform
 * interface, using Zephyr's native i2c_transfer() kernel API directly.
 *
 * WHY NOT ARDUINO WIRE?
 * ---------------------
 * The ST ULD requires single I2C transactions up to 32,800 bytes write
 * and 3,100 bytes read (UM2887, Table 2). The Arduino ZephyrI2C Wire
 * implementation has a 256-byte ring buffer — it cannot handle these
 * sizes. Chunking into multiple 32-byte transactions with intermediate
 * STOP conditions causes the VL53L5CX boot poll (register 0x06) to
 * never complete, hanging vl53l5cx_init() indefinitely.
 *
 * i2c_transfer() uses struct i2c_msg[] with direct buffer pointers —
 * no ring buffer, no size limit beyond available RAM. Each function
 * issues a single atomic I2C transaction.
 *
 * I2C TRANSACTION DESIGN
 * ----------------------
 * WrByte:  [addr_hi, addr_lo, value]           WRITE | STOP
 * WrMulti: [addr_hi, addr_lo] WRITE +
 *          [data...N bytes]   WRITE | STOP      — single transaction
 * RdByte:  [addr_hi, addr_lo] WRITE +
 *          [value]            RESTART | READ | STOP
 * RdMulti: [addr_hi, addr_lo] WRITE +
 *          [data...N bytes]   RESTART | READ | STOP
 *
 * License: MIT (our code) — ST ULD files carry BSD 3-clause.
 */

#include "platform.h"
#include <Arduino.h>
#include <zephyr/drivers/i2c.h>
/* I2C_MSG_STM32_USE_RELOAD_MODE is a private STM32 I2C V2 driver flag
 * defined in <zephyr/drivers/i2c/i2c_ll_stm32.h> (llext-edk, not in
 * the standard sketch include path). We define the value directly —
 * it is BIT(7) = 0x80, verified against the installed header at:
 * ~/.arduino15/packages/arduino/hardware/zephyr/0.54.1/variants/
 *   arduino_uno_q_stm32u585xx/llext-edk/include/zephyr/drivers/i2c/
 *   i2c_ll_stm32.h
 * This flag instructs the STM32 I2C V2 driver to use hardware RELOAD
 * mode for transfers > 255 bytes instead of generating new START
 * conditions at each 255-byte boundary. */
#ifndef I2C_MSG_STM32_USE_RELOAD_MODE
#define I2C_MSG_STM32_USE_RELOAD_MODE   (1U << 7U)
#endif

/* -------------------------------------------------------------------------
 * RdByte — read one byte from RegisterAddress.
 *
 * Issues: START + ADDR + reg_hi + reg_lo + RESTART + ADDR(R) + byte + STOP
 * -------------------------------------------------------------------------*/
extern "C" uint8_t RdByte(VL53L5CX_Platform *p_platform,
                           uint16_t RegisterAddress, uint8_t *p_value)
{
    uint8_t reg[2] = {
        (uint8_t)(RegisterAddress >> 8),
        (uint8_t)(RegisterAddress & 0xFF)
    };

    int ret = i2c_write_read(p_platform->i2c_dev,
                             (uint16_t)p_platform->address,
                             reg, sizeof(reg),
                             p_value, 1U);
    return (ret == 0) ? 0 : 1;
}

/* -------------------------------------------------------------------------
 * WrByte — write one byte to RegisterAddress.
 *
 * Issues: START + ADDR + reg_hi + reg_lo + value + STOP
 * -------------------------------------------------------------------------*/
extern "C" uint8_t WrByte(VL53L5CX_Platform *p_platform,
                           uint16_t RegisterAddress, uint8_t value)
{
    uint8_t buf[3] = {
        (uint8_t)(RegisterAddress >> 8),
        (uint8_t)(RegisterAddress & 0xFF),
        value
    };

    int ret = i2c_write(p_platform->i2c_dev,
                        buf, sizeof(buf),
                        (uint16_t)p_platform->address);
    return (ret == 0) ? 0 : 1;
}

/* -------------------------------------------------------------------------
 * RdMulti — read size bytes from RegisterAddress.
 *
 * Uses i2c_write_read() for a single atomic transaction:
 *   msg[0]: 2-byte register address  (WRITE, no stop)
 *   msg[1]: size-byte read buffer    (RESTART | READ | STOP)
 *
 * No chunking — the full payload is transferred in one transaction.
 * -------------------------------------------------------------------------*/
extern "C" uint8_t RdMulti(VL53L5CX_Platform *p_platform,
                            uint16_t RegisterAddress,
                            uint8_t *p_values, uint32_t size)
{
    uint8_t reg[2] = {
        (uint8_t)(RegisterAddress >> 8),
        (uint8_t)(RegisterAddress & 0xFF)
    };

    int ret = i2c_write_read(p_platform->i2c_dev,
                             (uint16_t)p_platform->address,
                             reg, sizeof(reg),
                             p_values, size);

    if (ret != 0) {
        /* Fill with 0xFF so ULD poll_for_answer timeout fires correctly */
        for (uint32_t i = 0; i < size; i++) {
            p_values[i] = 0xFF;
        }
        return 1;
    }
    return 0;
}

/* -------------------------------------------------------------------------
 * WrMulti — write size bytes from p_values to RegisterAddress.
 *
 * The STM32U5 I2C driver has a kernel-level transfer timeout of 500ms
 * (CONFIG_I2C_STM32_TRANSFER_TIMEOUT_MSEC). At 400kHz, a single 32KB
 * transfer takes ~0.7 seconds, exceeding this limit and causing the
 * kernel semaphore to timeout, aborting the transfer.
 *
 * Solution: chunk at HYBX_I2C_WR_CHUNK bytes per i2c_transfer() call.
 * At 400kHz, 16KB takes ~0.37 seconds — safely within 500ms.
 *
 * The VL53L5CX requires a 16-bit register address with EVERY I2C
 * transaction — there is no auto-increment across separate transactions.
 * Each chunk sends its own address (RegisterAddress + offset).
 *
 * Each chunk uses i2c_transfer() with two message segments:
 *   msg[0]: 2-byte register address (WRITE, no STOP)
 *   msg[1]: chunk data              (WRITE | STOP)
 *
 * I2C_MSG_STM32_USE_RELOAD_MODE on msg[1] enables hardware RELOAD for
 * the internal 255-byte boundaries within each chunk, keeping each
 * chunk as a single continuous I2C transaction on the wire.
 * -------------------------------------------------------------------------*/

/* Maximum bytes per WrMulti chunk — must complete within 500ms at 400kHz.
 * 16384 bytes × 9 bits / 400000 bps ≈ 0.37s — well within 500ms limit. */
#define HYBX_I2C_WR_CHUNK  4096U

extern "C" uint8_t WrMulti(VL53L5CX_Platform *p_platform,
                            uint16_t RegisterAddress,
                            uint8_t *p_values, uint32_t size)
{
    uint32_t offset = 0;
    while (offset < size) {
        uint32_t chunk = size - offset;
        if (chunk > HYBX_I2C_WR_CHUNK) {
            chunk = HYBX_I2C_WR_CHUNK;
        }

        /* The VL53L5CX page memory (selected via WrByte(0x7fff, page))
         * is a streaming buffer. Writes always begin at RegisterAddress
         * (typically 0x0000). The address does NOT increment across
         * separate I2C transactions — the sensor advances its internal
         * write pointer sequentially as data arrives. Each chunk must
         * present the SAME base address (RegisterAddress), not an
         * incrementing offset. */
        uint8_t reg[2] = {
            (uint8_t)(RegisterAddress >> 8),
            (uint8_t)(RegisterAddress & 0xFF)
        };

        struct i2c_msg msgs[2];

        msgs[0].buf   = reg;
        msgs[0].len   = sizeof(reg);
        msgs[0].flags = I2C_MSG_WRITE;
        msgs[1].buf   = p_values + offset;
        msgs[1].len   = chunk;
        msgs[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP
                        | I2C_MSG_STM32_USE_RELOAD_MODE;

        int ret = i2c_transfer(p_platform->i2c_dev,
                               msgs, 2,
                               (uint16_t)p_platform->address);
        if (ret != 0) {
            return 1;
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
    return 0;
}
