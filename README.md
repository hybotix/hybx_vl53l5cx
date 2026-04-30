# hybx_vl53l5cx
Minimal, Heap-Free Library for the ST VL53L5CX 8x8 ToF Distance Sensor
**Hybrid RobotiX — Dale Weber <hybotix@hybridrobotix.io>**

---

## Overview

`hybx_vl53l5cx` is a minimal Arduino library for the ST VL53L5CX 8x8 time-of-flight
distance sensor, purpose-built for the **Arduino UNO Q / Zephyr RTOS** environment.

It bundles the ST Ultra Lite Driver (ULD) source directly under its original BSD
3-clause license, and replaces the SparkFun platform layer with a clean Wire
implementation — **no SparkFun library dependency**.

### ⚠️ v1.x Platform Limitation — Arduino UNO Q

`hybx_vl53l5cx` **cannot initialize the VL53L5CX on the Arduino UNO Q under
the HybX Development System v1.x.** This is a known platform limitation that
will be resolved in v2.0 (HybX Build System).

**Root cause:** The VL53L5CX firmware upload requires a single continuous I2C
transaction of up to 32,800 bytes (ST UM2887 Table 2). The Zephyr STM32 I2C
driver as shipped in the Arduino Zephyr board package enforces a 500ms kernel
timeout (`CONFIG_I2C_STM32_TRANSFER_TIMEOUT_MSEC`) per `i2c_transfer()` call.
At 400kHz, a 32KB transfer takes ~0.7 seconds — exceeding this limit. The
transfer is aborted before the firmware upload completes.

**Attempted solutions rejected:**
- Chunking the upload: fails because the VL53L5CX page memory resets on each
  new I2C START condition — only the last chunk survives
- Board config override: `CONFIG_I2C_STM32_TRANSFER_TIMEOUT_MSEC=5000` or
  `CONFIG_I2C_STM32_V2_DMA=y` are overwritten on Arduino package updates

**v2.0 resolution:** The HybX Build System owns the board Kconfig. DMA will be
enabled for i2c4 (`CONFIG_I2C_STM32_V2_DMA=y`) — the correct, permanent fix.

The library architecture, platform layer, ULD integration, and Bridge interface
are all correct and complete. Only the I2C transfer size constraint blocks
initialization on v1.x.

---

## Why this exists

The Arduino RouterBridge on Zephyr RTOS is incompatible with `operator new`. Any heap
allocation at or after `Bridge.begin()` / `Bridge.provide()` corrupts the Bridge's
internal String parameter registration table.

The SparkFun VL53L5CX wrapper allocates `VL53L5CX_Configuration` and
`VL53L5CX_ResultsData` on the heap. This library eliminates all heap use:

- `VL53L5CX_Configuration` (including its 4096-byte `temp_buffer`) is a `static`
  member of the driver class — linker places it in BSS.
- `VL53L5CX_ResultsData` is a `static` local inside `_readFrame()` — also BSS.
- Only two result arrays are exposed as public static globals:
  - `int16_t hybx_distance_mm[64]`
  - `uint8_t hybx_target_status[64]`

All other ULD output fields (ambient, signal, sigma, reflectance, motion) are
disabled via DISABLE macros in `platform.h`, minimising both the I2C payload and
`temp_buffer` size.

---

## Library Structure

```
hybx_vl53l5cx/
  library.properties
  src/
    platform.h            VL53L5CX_Platform struct + platform function declarations
    platform.cpp          Wire I2C: RdByte/WrByte/RdMulti/WrMulti/SwapBuffer/WaitMs
    hybx_vl53l5cx.h       Arduino driver class + public result globals
    hybx_vl53l5cx.cpp     Driver implementation
    uld/
      vl53l5cx_api.h      ST ULD API header (BSD 3-clause)
      vl53l5cx_api.cpp    ST ULD implementation (BSD 3-clause)
      vl53l5cx_buffers.h  ST firmware blob + default config/xtalk (BSD 3-clause)
```

---

## Installation

Install to `~/Arduino/libraries/hybx_vl53l5cx/` on the Arduino UNO Q.

```bash
cd ~/Arduino/libraries
git clone https://github.com/hybotix/hybx_vl53l5cx.git
```

---

## Usage

```cpp
#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <hybx_vl53l5cx.h>

hybx_vl53l5cx sensor;   // 8x8, address 0x29, Wire1

String get_distance_data() {
    if (!hybx_sensor_ready) return "0";
    // read hybx_distance_mm[0..63]
    ...
}

void setup() {
    Bridge.begin();
    Bridge.provide("get_distance_data", get_distance_data);
    Wire1.begin();
    sensor.begin();    // blocks up to 10 s — firmware upload
}

void loop() {
    sensor.poll();     // non-blocking; updates result globals when data ready
}
```

### Result globals

| Global | Type | Description |
|---|---|---|
| `hybx_distance_mm[64]` | `int16_t` | Distance per zone in mm |
| `hybx_target_status[64]` | `uint8_t` | Status per zone (5 or 9 = valid) |
| `hybx_sensor_ready` | `bool` | True once first frame received |

### Constructor

```cpp
hybx_vl53l5cx sensor(resolution, address, wire);
```

| Parameter | Default | Description |
|---|---|---|
| `resolution` | `64` | `16` = 4x4, `64` = 8x8 |
| `address` | `0x29` | 7-bit I2C address |
| `wire` | `Wire1` | Wire instance (UNO Q QWIIC = Wire1) |

### Methods

| Method | Description |
|---|---|
| `begin()` | Upload firmware and start ranging. Call after `Bridge.provide()` and `Wire1.begin()`. Returns `bool`. |
| `poll()` | Non-blocking. Call from `loop()`. Updates result globals when data ready. |
| `setResolution(res)` | Change resolution at runtime (`16` or `64`). Returns `bool`. |
| `getResolution()` | Returns active resolution (`16` or `64`). |

---

## sketch.yaml

```yaml
profiles:
  default:
    platforms:
      - platform: arduino:zephyr
    libraries:
      - Arduino_RouterBridge (0.3.0)
      - dependency: Arduino_RPClite (0.2.1)
      - dependency: ArxContainer (0.7.0)
      - dependency: ArxTypeTraits (0.3.2)
      - dependency: DebugLog (0.8.4)
      - dependency: MsgPack (0.4.2)
      - hybx_vl53l5cx
default_profile: default
```

---

## Error Reporting

No silent failures. Every ULD call result is checked and reported.

### Error globals

| Global | Type | Description |
|---|---|---|
| `hybx_last_error` | `uint8_t` | ULD status code of the last failure (0 = no error) |
| `hybx_last_error_step` | `uint8_t` | Which step failed (see step codes below) |

### Error step codes

| Constant | Value | ULD Function |
|---|---|---|
| `HYBX_ERR_NONE` | 0 | No error |
| `HYBX_ERR_INIT` | 1 | `vl53l5cx_init` |
| `HYBX_ERR_SET_RESOLUTION` | 2 | `vl53l5cx_set_resolution` |
| `HYBX_ERR_SET_FREQUENCY` | 3 | `vl53l5cx_set_ranging_frequency_hz` |
| `HYBX_ERR_START_RANGING` | 4 | `vl53l5cx_start_ranging` |
| `HYBX_ERR_STOP_RANGING` | 5 | `vl53l5cx_stop_ranging` |
| `HYBX_ERR_CHECK_READY` | 6 | `vl53l5cx_check_data_ready` |
| `HYBX_ERR_GET_DATA` | 7 | `vl53l5cx_get_ranging_data` |
| `HYBX_ERR_BAD_RESOLUTION` | 8 | Invalid resolution value passed to `setResolution()` |
| `HYBX_ERR_NOT_INITIALIZED` | 9 | Method called before `begin()` succeeded |

### Failure coverage

| Layer | Failure Point | Handling |
|---|---|---|
| `platform.cpp` | Every I2C endTransmission/requestFrom | Returns 1 → ULD → `_fail()` |
| `begin()` | init, set_resolution, set_frequency, start_ranging | `_fail()` with step + ULD code |
| `setResolution()` | not_init, bad_res, stop, set_res, set_freq, start | `_fail()` with step + ULD code |
| `poll()` | not_initialized, check_data_ready | `_fail()` with step + ULD code |
| `_readFrame()` | get_ranging_data | `_fail()` with step + ULD code |

Error globals are cleared automatically when a frame is successfully read.

### Exposing errors via Bridge

```cpp
String get_sensor_status() {
    if (initFailed) {
        return "init_failed:" + String(hybx_last_error_step) +
               ":" + String(hybx_last_error);
    }
    if (hybx_sensor_ready) {
        if (hybx_last_error_step != 0) {
            return "error:" + String(hybx_last_error_step) +
                   ":" + String(hybx_last_error);
        }
        return "ready";
    }
    return "initializing";
}
```

---

## Licensing

| Component | License |
|---|---|
| `platform.h`, `platform.cpp`, `hybx_vl53l5cx.h`, `hybx_vl53l5cx.cpp` | MIT (c) 2026 Dale Weber |
| `src/uld/vl53l5cx_api.h`, `src/uld/vl53l5cx_api.cpp`, `src/uld/vl53l5cx_buffers.h` | BSD 3-clause (c) 2020 STMicroelectronics |

---

*Hybrid RobotiX — San Diego*
*"I. WILL. NEVER. GIVE. UP. OR. SURRENDER."*

---

## Platform Layer — Wire1 Implementation

### Why Wire1, not Zephyr native I2C

The original platform layer used Zephyr's native `i2c_transfer()` API directly. This caused **indefinite hangs** during the VL53L5CX firmware upload (~96KB across three 32KB pages) when Arduino RouterBridge was running. The Zephyr STM32 I2C kernel driver and the RouterBridge are incompatible — root cause not fully understood but consistently reproduced.

Wire1 (the Arduino QWIIC bus on the UNO Q) works correctly with the Bridge running — confirmed by the `vl53-diag` diagnostic app:
- I2C probe at 0x29 ✅
- Page select write (reg 0x7FFF) ✅  
- Device ID read (expect 0xF0) ✅
- Revision ID read (expect 0x02) ✅

### CRITICAL: Wire1.begin() must be called BEFORE Bridge.begin()

Calling `Wire1.begin()` AFTER `Bridge.begin()` **hangs the MCU permanently**. Also, `#include <Wire.h>` must be in the **sketch**, not this library — including it in a library auto-initializes Wire1 before `setup()` runs, also hanging the MCU.

The correct order in `setup()`:
```cpp
#include <Wire.h>          // In the SKETCH — not the library

void setup() {
    Wire1.begin();         // MUST be before Bridge.begin()
    Bridge.begin();
    Bridge.provide(...);
    sensor.begin();
}
```

### Wire1 size limits

- **Write**: `Wire1.write(buf, len)` streams bytes with no practical size limit — the full 32KB firmware page is written in 4KB chunks per `beginTransmission`/`endTransmission` cycle.
- **Read**: `Wire1.requestFrom()` is limited to 256 bytes per call. `RdMulti` chunks reads at 256 bytes with incrementing address.

### When the library changes, force a recompile

The HybX sketch hash only tracks `.ino` file changes. If the library (`hybx_vl53l5cx`) changes, the sketch hash won't detect it and `start` will skip recompile. Always use `clean <app>` after a library update — it passes `--compile` to `start` which forces a full recompile regardless of hash.

```bash
update
board sync --force
clean monitor-vl53l5cx
mon
```
