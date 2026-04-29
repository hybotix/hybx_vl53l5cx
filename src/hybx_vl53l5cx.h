/*
 * hybx_vl53l5cx.h
 * Hybrid RobotiX — hybx_vl53l5cx
 * Dale Weber <hybotix@hybridrobotix.io>
 *
 * Minimal, heap-free VL53L5CX driver for the Arduino UNO Q / Zephyr RTOS.
 *
 * DESIGN CONSTRAINTS
 * ------------------
 * The Arduino RouterBridge on Zephyr RTOS is incompatible with operator new.
 * This library uses ZERO heap — all buffers including the ST ULD's temp_buffer
 * are static globals placed in BSS by the linker.
 *
 * Only two result arrays are allocated:
 *   hybx_distance_mm[64]     int16_t  millimetres, zones 0-63
 *   hybx_target_status[64]   uint8_t  ST status codes (5 or 9 = valid)
 *
 * ERROR REPORTING
 * ---------------
 * No silent failures. Every ULD call result is checked and stored in
 * hybx_last_error (ULD status code) and hybx_last_error_step (which
 * step failed). Both are exposed as public globals for the sketch to
 * report via Bridge functions.
 *
 * I2C
 * ---
 * Default: i2c4 (UNO Q QWIIC bus / Wire1), address 0x29.
 * Uses Zephyr native i2c_transfer() — no Wire.begin() needed.
 * The i2c4 device is enabled and configured at 400kHz in the DTS overlay.
 *
 * License: MIT (our code) — ST ULD files carry BSD 3-clause.
 */

#pragma once

#include <Arduino.h>
#include <zephyr/drivers/i2c.h>
#include "platform.h"
#include "uld/vl53l5cx_api.h"

/* -------------------------------------------------------------------------
 * Error step codes — identify which ULD call failed
 * -------------------------------------------------------------------------*/
#define HYBX_ERR_NONE             0
#define HYBX_ERR_INIT             1   /* vl53l5cx_init failed */
#define HYBX_ERR_SET_RESOLUTION   2   /* vl53l5cx_set_resolution failed */
#define HYBX_ERR_SET_FREQUENCY    3   /* vl53l5cx_set_ranging_frequency_hz failed */
#define HYBX_ERR_START_RANGING    4   /* vl53l5cx_start_ranging failed */
#define HYBX_ERR_STOP_RANGING     5   /* vl53l5cx_stop_ranging failed */
#define HYBX_ERR_CHECK_READY      6   /* vl53l5cx_check_data_ready failed */
#define HYBX_ERR_GET_DATA         7   /* vl53l5cx_get_ranging_data failed */
#define HYBX_ERR_BAD_RESOLUTION   8   /* invalid resolution value passed */
#define HYBX_ERR_NOT_INITIALIZED  9   /* method called before begin() */

/* -------------------------------------------------------------------------
 * Public result and error globals — BSS allocation, defined in .cpp
 * -------------------------------------------------------------------------*/
extern int16_t  hybx_distance_mm[64];
extern uint8_t  hybx_target_status[64];
extern bool     hybx_sensor_ready;
extern uint8_t  hybx_last_error;       /* ULD status code of last failure */
extern uint8_t  hybx_last_error_step;  /* HYBX_ERR_* step that failed */
extern uint8_t  hybx_init_step;        /* diagnostic: poll checkpoint reached in vl53l5cx_init */

/* -------------------------------------------------------------------------
 * hybx_vl53l5cx — thin, heap-free driver class
 * -------------------------------------------------------------------------*/
class hybx_vl53l5cx {
public:
    /*
     * Constructor.
     * resolution : 16 (4x4) or 64 (8x8). Default 64.
     * address    : 7-bit I2C address. Default 0x29.
     * i2c_dev    : Zephyr I2C device pointer.
     *              Default: DEVICE_DT_GET(DT_NODELABEL(i2c4)) (Wire1/QWIIC on UNO Q).
     *              Use Zephyr native API — bypasses Arduino Wire ring buffer
     *              limit which cannot handle the 32KB firmware upload.
     */
    hybx_vl53l5cx(uint8_t resolution = 64,
                  uint8_t address    = 0x29,
                  const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c4)));

    /*
     * begin() — upload firmware and start ranging.
     * Call from setup() AFTER Bridge.begin() and Bridge.provide() calls.
     * Wire1.begin() must already have been called.
     * Blocks up to ~10 s during firmware upload.
     * Returns true on success. On failure, hybx_last_error and
     * hybx_last_error_step contain the ULD status and step that failed.
     */
    bool begin();

    /*
     * poll() — call from loop(). Non-blocking.
     * On failure, hybx_last_error and hybx_last_error_step are updated.
     */
    void poll();

    /*
     * setResolution() — change resolution at runtime.
     * resolution: 16 (4x4) or 64 (8x8).
     * Returns true on success.
     */
    bool setResolution(uint8_t resolution);

    uint8_t getResolution() const { return _resolution; }

private:
    /* Static: lives in BSS, never touches the heap. */
    static VL53L5CX_Configuration _dev;

    uint8_t _resolution;
    bool    _initialized;

    void _readFrame();

    /* Record a failure — sets both error globals and returns false. */
    bool _fail(uint8_t step, uint8_t uld_status);
};
