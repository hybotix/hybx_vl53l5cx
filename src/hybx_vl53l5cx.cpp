/*
 * hybx_vl53l5cx.cpp
 * Hybrid RobotiX — hybx_vl53l5cx
 * Dale Weber <hybotix@hybridrobotix.io>
 *
 * No silent failures. Every ULD call result is checked.
 * Failures are stored in hybx_last_error and hybx_last_error_step
 * for the sketch to report via Bridge functions.
 *
 * License: MIT (our code) — ST ULD files carry BSD 3-clause.
 */

#include "hybx_vl53l5cx.h"
#include <zephyr/drivers/i2c.h>

/* -------------------------------------------------------------------------
 * Static definitions — all in BSS, zero heap.
 * -------------------------------------------------------------------------*/
VL53L5CX_Configuration hybx_vl53l5cx::_dev;

int16_t hybx_distance_mm[64];
uint8_t hybx_target_status[64];
bool    hybx_sensor_ready    = false;
uint8_t hybx_last_error      = HYBX_ERR_NONE;
uint8_t hybx_last_error_step = HYBX_ERR_NONE;
uint8_t hybx_init_step       = 0;

/* -------------------------------------------------------------------------
 * Constructor
 * -------------------------------------------------------------------------*/
hybx_vl53l5cx::hybx_vl53l5cx(uint8_t resolution, uint8_t address)
    : _resolution(resolution), _initialized(false)
{
    _dev.platform.address = (uint16_t)address;
}

/* -------------------------------------------------------------------------
 * _fail() — record error and return false
 * -------------------------------------------------------------------------*/
bool hybx_vl53l5cx::_fail(uint8_t step, uint8_t uld_status)
{
    hybx_last_error_step = step;
    hybx_last_error      = uld_status;
    return false;
}

/* -------------------------------------------------------------------------
 * begin()
 * -------------------------------------------------------------------------*/
bool hybx_vl53l5cx::begin()
{
    uint8_t status;

    status = vl53l5cx_init(&_dev);
    if (status != VL53L5CX_STATUS_OK) {
        return _fail(HYBX_ERR_INIT, status);
    }

    status = vl53l5cx_set_resolution(&_dev, _resolution);
    if (status != VL53L5CX_STATUS_OK) {
        return _fail(HYBX_ERR_SET_RESOLUTION, status);
    }

    uint8_t freq = (_resolution == VL53L5CX_RESOLUTION_8X8) ? 15 : 30;
    status = vl53l5cx_set_ranging_frequency_hz(&_dev, freq);
    if (status != VL53L5CX_STATUS_OK) {
        return _fail(HYBX_ERR_SET_FREQUENCY, status);
    }

    status = vl53l5cx_start_ranging(&_dev);
    if (status != VL53L5CX_STATUS_OK) {
        return _fail(HYBX_ERR_START_RANGING, status);
    }

    /* Allow sensor MCU to stabilize after starting ranging before
     * any further commands (e.g. set_resolution) are accepted. */
    delay(500);

    _initialized = true;
    return true;
}

/* -------------------------------------------------------------------------
 * setResolution()
 * -------------------------------------------------------------------------*/
bool hybx_vl53l5cx::setResolution(uint8_t resolution)
{
    if (!_initialized) {
        return _fail(HYBX_ERR_NOT_INITIALIZED, 0);
    }
    if (resolution != VL53L5CX_RESOLUTION_4X4 &&
        resolution != VL53L5CX_RESOLUTION_8X8) {
        return _fail(HYBX_ERR_BAD_RESOLUTION, resolution);
    }

    uint8_t status;

    status = vl53l5cx_stop_ranging(&_dev);
    if (status != VL53L5CX_STATUS_OK) {
        return _fail(HYBX_ERR_STOP_RANGING, status);
    }

    _resolution = resolution;

    status = vl53l5cx_set_resolution(&_dev, _resolution);
    if (status != VL53L5CX_STATUS_OK) {
        return _fail(HYBX_ERR_SET_RESOLUTION, status);
    }

    uint8_t freq = (_resolution == VL53L5CX_RESOLUTION_8X8) ? 15 : 30;
    status = vl53l5cx_set_ranging_frequency_hz(&_dev, freq);
    if (status != VL53L5CX_STATUS_OK) {
        return _fail(HYBX_ERR_SET_FREQUENCY, status);
    }

    status = vl53l5cx_start_ranging(&_dev);
    if (status != VL53L5CX_STATUS_OK) {
        return _fail(HYBX_ERR_START_RANGING, status);
    }

    return true;
}

/* -------------------------------------------------------------------------
 * poll()
 * -------------------------------------------------------------------------*/
void hybx_vl53l5cx::poll()
{
    if (!_initialized) {
        /* Not initialized — record as error so sketch can report it */
        _fail(HYBX_ERR_NOT_INITIALIZED, 0);
        return;
    }

    uint8_t isReady = 0;
    uint8_t status  = vl53l5cx_check_data_ready(&_dev, &isReady);
    if (status != VL53L5CX_STATUS_OK) {
        _fail(HYBX_ERR_CHECK_READY, status);
        return;
    }

    if (isReady) {
        _readFrame();
    }
}

/* -------------------------------------------------------------------------
 * _readFrame()
 * -------------------------------------------------------------------------*/
void hybx_vl53l5cx::_readFrame()
{
    static VL53L5CX_ResultsData results;

    uint8_t status = vl53l5cx_get_ranging_data(&_dev, &results);
    if (status != VL53L5CX_STATUS_OK) {
        _fail(HYBX_ERR_GET_DATA, status);
        return;
    }

    uint8_t zones = _resolution;
    for (uint8_t i = 0; i < zones; i++) {
        hybx_distance_mm[i]   = results.distance_mm[i];
        hybx_target_status[i] = results.target_status[i];
    }

    /* Clear any previous error once a frame is successfully read */
    hybx_last_error      = HYBX_ERR_NONE;
    hybx_last_error_step = HYBX_ERR_NONE;
    hybx_sensor_ready    = true;
}
