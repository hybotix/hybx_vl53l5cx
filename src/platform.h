/*
 * platform.h
 * Hybrid RobotiX — hybx_vl53l5cx
 * Dale Weber <hybotix@hybridrobotix.io>
 *
 * VL53L5CX ULD platform layer for Arduino UNO Q / Zephyr RTOS.
 *
 * It defines VL53L5CX_Platform and declares the six mandatory platform
 * functions that vl53l5cx_api.cpp calls. All implementations are in
 * platform.cpp and use the Zephyr native I2C API (i2c_transfer) directly
 * — bypassing the Arduino Wire layer entirely.
 *
 * WHY ZEPHYR NATIVE I2C?
 * ----------------------
 * The ST ULD requires single I2C transactions of up to 32,800 bytes write
 * and 3,100 bytes read (UM2887 Table 2). The Arduino ZephyrI2C Wire
 * implementation has a 256-byte ring buffer and cannot handle these sizes.
 *
 * The Zephyr native i2c_transfer() API uses struct i2c_msg[] with direct
 * buffer pointers — no ring buffer, no size limit beyond available RAM.
 * WrMulti and RdMulti use i2c_transfer() with two message segments:
 *   msg[0]: 2-byte register address (I2C_MSG_WRITE, no stop)
 *   msg[1]: data payload (I2C_MSG_WRITE|STOP or I2C_MSG_RESTART|READ|STOP)
 * This issues a single I2C transaction with no intermediate STOP conditions,
 * exactly as the VL53L5CX firmware upload requires.
 *
 * Configuration macros
 * --------------------
 * VL53L5CX_NB_TARGET_PER_ZONE   — targets reported per zone (1-4). We
 *                                  use 1: minimises I2C payload and keeps
 *                                  result arrays at exactly [64] elements.
 *
 * All other output fields (ambient_per_spad, nb_spads_enabled, etc.) are
 * disabled via the DISABLE macros below, shrinking temp_buffer and the
 * over-the-wire I2C payload to the minimum needed for distance + status.
 *
 * NOTE: Function names (RdByte, WrByte, RdMulti, WrMulti, SwapBuffer,
 * WaitMs) are MANDATED by the ST Ultra Lite Driver API. They cannot be
 * renamed. These are our implementations of the required platform interface.
 *
 * License: MIT (our code) — ST ULD files carry BSD 3-clause.
 */

#ifndef HYBX_PLATFORM_H
#define HYBX_PLATFORM_H

#include <stdint.h>
#include <zephyr/drivers/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * VL53L5CX_Platform — opaque handle passed through every ULD call.
 *
 * i2c_dev : Zephyr I2C device pointer (e.g. DEVICE_DT_GET(DT_NODELABEL(i2c4)))
 * address : 7-bit I2C address of the sensor (default 0x29)
 * -------------------------------------------------------------------------*/
typedef struct {
    const struct device *i2c_dev;  /* Zephyr I2C device — bypasses Wire */
    uint16_t             address;  /* 7-bit I2C address (default 0x29)  */
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
 *
 * These function names are MANDATED by the ST Ultra Lite Driver API.
 * They cannot be renamed — vl53l5cx_api.cpp calls them by these exact
 * names. They must have C linkage so the C-compiled ULD can call them
 * from C++.
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
