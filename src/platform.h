/*
 * platform.h
 * Hybrid RobotiX — hybx_vl53l5cx
 * Dale Weber <hybotix@hybridrobotix.io>
 *
 * VL53L5CX ULD platform layer for Arduino UNO Q — Wire1 implementation.
 *
 * WHY WIRE1 INSTEAD OF ZEPHYR NATIVE I2C?
 * ----------------------------------------
 * Zephyr native i2c_transfer() hangs indefinitely during the VL53L5CX
 * firmware upload when Arduino RouterBridge is running. The Bridge
 * interferes with the Zephyr STM32 I2C kernel driver.
 *
 * Wire1 (Arduino QWIIC bus) works correctly with the Bridge — confirmed
 * by vl53-diag: probe, page select, device ID (0xF0), revision ID (0x02)
 * all pass with Bridge running.
 *
 * CRITICAL: Wire1.begin() must NEVER be called. Calling it after
 * Bridge.begin() hangs the MCU. Wire1 works without explicit
 * initialization on the UNO Q.
 *
 * Wire1 write: unlimited streaming via write(buf, len) — no size limit.
 * Wire1 read:  256 bytes max per requestFrom() call — RdMulti chunks.
 *
 * VL53L5CX_Platform holds only the 7-bit I2C address — Wire1 is global.
 *
 * NOTE: Function names (RdByte, WrByte, RdMulti, WrMulti, SwapBuffer,
 * WaitMs) are MANDATED by the ST Ultra Lite Driver API. They cannot be
 * renamed.
 *
 * License: MIT (our code) — ST ULD files carry BSD 3-clause.
 */

#ifndef HYBX_PLATFORM_H
#define HYBX_PLATFORM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * VL53L5CX_Platform — opaque handle passed through every ULD call.
 * Wire1 is a global on Arduino UNO Q — no device pointer needed.
 * -------------------------------------------------------------------------*/
typedef struct {
    uint16_t address;   /* 7-bit I2C address (default 0x29) */
} VL53L5CX_Platform;

/* -------------------------------------------------------------------------
 * ULD tuning — 1 target per zone, only distance + status output.
 * -------------------------------------------------------------------------*/
#define VL53L5CX_NB_TARGET_PER_ZONE    1U

#define VL53L5CX_DISABLE_AMBIENT_PER_SPAD
#define VL53L5CX_DISABLE_NB_SPADS_ENABLED
#define VL53L5CX_DISABLE_NB_TARGET_DETECTED
#define VL53L5CX_DISABLE_SIGNAL_PER_SPAD
#define VL53L5CX_DISABLE_RANGE_SIGMA_MM
/* VL53L5CX_DISABLE_DISTANCE_MM      — keep */
#define VL53L5CX_DISABLE_REFLECTANCE_PERCENT
/* VL53L5CX_DISABLE_TARGET_STATUS    — keep */
#define VL53L5CX_DISABLE_MOTION_INDICATOR

/* -------------------------------------------------------------------------
 * Mandatory ST ULD platform functions — implemented in platform.cpp.
 * -------------------------------------------------------------------------*/
uint8_t RdByte(VL53L5CX_Platform *p_platform,
               uint16_t RegisterAddress, uint8_t *p_value);

uint8_t WrByte(VL53L5CX_Platform *p_platform,
               uint16_t RegisterAddress, uint8_t value);

uint8_t RdMulti(VL53L5CX_Platform *p_platform,
                uint16_t RegisterAddress,
                uint8_t *p_values, uint32_t size);

uint8_t WrMulti(VL53L5CX_Platform *p_platform,
                uint16_t RegisterAddress,
                uint8_t *p_values, uint32_t size);

void    SwapBuffer(uint8_t *buffer, uint16_t size);

uint8_t WaitMs(VL53L5CX_Platform *p_platform, uint32_t TimeMs);

#ifdef __cplusplus
}
#endif

#endif /* HYBX_PLATFORM_H */
