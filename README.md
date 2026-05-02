# hybx_vl53l5cx_unoq
Minimal, Heap-Free Library for the ST VL53L5CX 8x8 ToF Distance Sensor
**Hybrid RobotiX — Dale Weber <hybotix@hybridrobotix.io>**

---

## Overview

`hybx_vl53l5cx_unoq` is a minimal Arduino library for the ST VL53L5CX 8x8
time-of-flight distance sensor, purpose-built for the **Arduino UNO Q /
Zephyr RTOS** environment with the Arduino **RouterBridge**.

It bundles the ST Ultra Lite Driver (ULD) source directly under its original
BSD 3-clause license, and replaces the SparkFun platform layer with a clean
Wire1 implementation — **no SparkFun library dependency, no heap allocation**.

---

## Why This Library Exists

### No heap allocation
The Arduino RouterBridge on Zephyr RTOS is incompatible with `operator new`.
Any heap allocation at or after `Bridge.begin()` / `Bridge.provide()` corrupts
the Bridge's internal String parameter registration table.

The SparkFun VL53L5CX wrapper allocates `VL53L5CX_Configuration` and
`VL53L5CX_ResultsData` on the heap. This library eliminates all heap use:

- `VL53L5CX_Configuration` (including its 4096-byte `temp_buffer`) is a
  `static` member of the driver class — placed in BSS by the linker.
- `VL53L5CX_ResultsData` is a `static` local inside `_readFrame()` — also BSS.
- Result arrays are public static globals, directly readable by the sketch.

### Arduino Build System replacement
`arduino-app-cli` (the Arduino tool that compiles, flashes, and manages Docker
containers for UNO Q apps) has a critical flaw: it caches compiled binaries by
**sketch hash only**. Library changes are completely invisible — it silently
reuses the old binary. This made library development impossible.

**HybX Development System v2.0** replaces `arduino-app-cli` entirely:

| Arduino Component | HybX Replacement | Why |
|---|---|---|
| `arduino-cli` (compile) | `HybXCompiler` | Always compiles fresh — no caching |
| `arduino-cli` (flash) | `HybXFlasher` | Always flashes — no stale binaries |
| `arduino-app-cli` (container) | `HybXRunner` | Direct Docker control via Python API |
| `arduino-app-cli app logs` | `docker logs -f` | Direct, no intermediary |

The HybX commands on the UNO Q:

```bash
update                         # Pull latest repos, deploy lib/bin updates
board sync <app> --force       # Sync specific app from repo to board
clean <app>                    # Stop container, compile fresh, flash, start
mon                            # Follow app output (docker logs -f)
build <app>                    # Compile and flash only (no container)
stop <app>                     # Stop app container
```

---

## Platform Layer — Wire1 Implementation

### Why Wire1 instead of Zephyr native i2c_transfer()

The original platform layer called Zephyr's native `i2c_transfer()` directly.
This caused **indefinite hangs** during the VL53L5CX firmware upload when
RouterBridge was running. The Zephyr STM32 I2C kernel driver and the Bridge
UART transport conflict at the interrupt level — root cause not fully resolved,
but consistently reproduced and consistently fixed by using Wire1.

Wire1 (the Arduino QWIIC bus on the UNO Q) works correctly with the Bridge —
confirmed by the `vl53-diag` diagnostic app:

- I2C probe at 0x29 ✅
- Page select write (reg 0x7FFF) ✅
- Device ID read (expect 0xF0) ✅
- Revision ID read (expect 0x02) ✅
- 4096-byte write ✅
- 255-byte read ✅
- Full 86KB firmware upload ✅
- Ranging data reads ✅

### Wire1 size limits

The Zephyr Wire1 implementation uses a **256-byte ring buffer** for both TX
and RX.

- **Write**: address (2 bytes) + data share the 256-byte TX buffer.
  Maximum data payload per `endTransmission()` = **254 bytes**.
- **Read**: `requestFrom()` limit = **255 bytes** per call.

`WrMulti` chunks writes at 254 bytes with **incrementing register address**.
`RdMulti` chunks reads at 255 bytes with incrementing register address.

> ⚠️ The VL53L5CX firmware page memory does NOT auto-advance the write
> pointer on repeated transactions at address 0x0000. The register address
> MUST increment across chunks for ALL writes, including firmware pages.

### CRITICAL: Initialization order

```cpp
#include <Arduino_RouterBridge.h>
#include <Wire.h>              // Must be in the SKETCH — not in this library
#include <hybx_vl53l5cx_unoq.h>

hybx_vl53l5cx_unoq sensor;

String get_sensor_status() { ... }
String begin_sensor() {
    sensor.begin();            // Blocks ~30s during firmware upload
    ...
}

void setup() {
    Wire1.begin();             // 1. MUST be before Bridge.begin()
    Bridge.begin();            // 2. Start Bridge transport
    Bridge.provide("begin_sensor",      begin_sensor);
    Bridge.provide("get_sensor_status", get_sensor_status);
    // ... other provides ...
}                              // 3. Do NOT call sensor.begin() here

void loop() {
    sensor.poll();             // Non-blocking ranging updates
}
```

**Why `begin_sensor()` is triggered from Linux, not called in `setup()`:**
The VL53L5CX firmware upload takes ~30 seconds. Blocking `setup()` for that
duration starves the Bridge UART transport — Python times out waiting for a
response. Instead, `begin_sensor()` is a Bridge function that Python calls
explicitly. The Bridge call blocks on the **Linux side** while the MCU
uploads firmware transparently. The Bridge transport remains idle but intact.

**Why `Wire1.begin()` MUST come before `Bridge.begin()`:**
Calling `Wire1.begin()` after `Bridge.begin()` hangs the MCU permanently
with no error output. Root cause: a Zephyr RTOS resource conflict between
the RouterBridge serial driver and the Wire1/i2c4 peripheral initialization.

**Why `#include <Wire.h>` must be in the sketch, not this library:**
Including `Wire.h` inside a library causes the Arduino build system to
auto-initialize Wire1 before `setup()` runs — also hanging the MCU.

---

## Library Structure

```
hybx_vl53l5cx_unoq/
  library.properties
  src/
    platform.h            VL53L5CX_Platform struct + platform declarations
    platform.cpp          Wire1: RdByte/WrByte/RdMulti/WrMulti/SwapBuffer/WaitMs
    hybx_vl53l5cx_unoq.h       Driver class + public result globals
    hybx_vl53l5cx_unoq.cpp     Driver implementation
    uld/
      vl53l5cx_api.h      ST ULD API header (BSD 3-clause)
      vl53l5cx_api.cpp    ST ULD implementation (BSD 3-clause)
      vl53l5cx_buffers.h  ST firmware blob + default config/xtalk (BSD 3-clause)
```

---

## Installation

```bash
cd ~/Arduino/libraries
git clone https://github.com/hybotix/hybx_vl53l5cx_unoq.git
```

Add to `sketch.yaml`:

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
      - dir: /home/arduino/Arduino/libraries/hybx_vl53l5cx_unoq
default_profile: default
```

---

## Public Globals

| Global | Type | Description |
|---|---|---|
| `hybx_distance_mm[64]` | `int16_t` | Distance per zone in mm |
| `hybx_target_status[64]` | `uint8_t` | ST status code per zone (5 or 9 = valid) |
| `hybx_signal_per_spad[64]` | `uint32_t` | Signal strength per zone (kcps/SPAD) |
| `hybx_range_sigma_mm[64]` | `uint16_t` | Ranging sigma per zone (mm) |
| `hybx_sensor_ready` | `bool` | True once first valid frame received |
| `hybx_last_error` | `uint8_t` | ULD status code of last failure (0 = none) |
| `hybx_last_error_step` | `uint8_t` | Step that failed (see error codes) |

Zones are ordered row-major, top-left to bottom-right (zone 0 = top-left,
zone 63 = bottom-right).

---

## Constructor

```cpp
hybx_vl53l5cx_unoq sensor;                        // 8x8, address 0x29
hybx_vl53l5cx_unoq sensor(16);                    // 4x4, address 0x29
hybx_vl53l5cx_unoq sensor(64, 0x29);              // 8x8, custom address
```

| Parameter | Default | Description |
|---|---|---|
| `resolution` | `64` | `16` = 4x4, `64` = 8x8 |
| `address` | `0x29` | 7-bit I2C address |

---

## Methods

| Method | Description |
|---|---|
| `begin()` | Upload firmware and start ranging. Triggered from Linux via Bridge. Returns `bool`. |
| `poll()` | Non-blocking. Call from `loop()`. Updates result globals when data ready. |
| `setResolution(res)` | Change resolution at runtime (`16` or `64`). Returns `bool`. |
| `getResolution()` | Returns active resolution (`16` or `64`). |

---

## Confidence Values (0.00–99.99%)

Per-zone confidence is computed from `signal_per_spad` and `range_sigma_mm`:

```python
signal_score = min(signal_per_spad / 8000.0, 1.0)
sigma_score  = max(0.0, 1.0 - range_sigma_mm / 30.0)
confidence   = (signal_score * 0.6 + sigma_score * 0.4) * 99.99
```

If `target_status` is not 5 or 9, confidence is 0.00% regardless.

| Confidence | Meaning |
|---|---|
| 0.00% | Invalid zone — no valid return |
| 1–40% | Long range or weak return — use with caution |
| 40–80% | Medium confidence |
| 80–99.99% | High confidence — strong, clean return |

**Observed values:**
- Close target (~90mm): 91–99% center zones, decreasing toward edges
- Wall at ~1.6–2.0m: 20–33% — physically correct (weaker signal at range)
- Invalid/edge zones: exactly 0.00%

Confidence drops naturally with distance as signal weakens and sigma
increases. This is the expected physical behavior.

---

## Error Reporting

| Step Constant | Value | ULD Function |
|---|---|---|
| `HYBX_ERR_NONE` | 0 | No error |
| `HYBX_ERR_INIT` | 1 | `vl53l5cx_init` |
| `HYBX_ERR_SET_RESOLUTION` | 2 | `vl53l5cx_set_resolution` |
| `HYBX_ERR_SET_FREQUENCY` | 3 | `vl53l5cx_set_ranging_frequency_hz` |
| `HYBX_ERR_START_RANGING` | 4 | `vl53l5cx_start_ranging` |
| `HYBX_ERR_STOP_RANGING` | 5 | `vl53l5cx_stop_ranging` |
| `HYBX_ERR_CHECK_READY` | 6 | `vl53l5cx_check_data_ready` |
| `HYBX_ERR_GET_DATA` | 7 | `vl53l5cx_get_ranging_data` |
| `HYBX_ERR_BAD_RESOLUTION` | 8 | Invalid resolution |
| `HYBX_ERR_NOT_INITIALIZED` | 9 | Called before `begin()` succeeded |

---

## Licensing

| Component | License |
|---|---|
| `platform.h/cpp`, `hybx_vl53l5cx_unoq.h/cpp` | MIT © 2026 Dale Weber / Hybrid RobotiX |
| `src/uld/` | BSD 3-clause © 2020 STMicroelectronics |

---

*Hybrid RobotiX — San Diego*
*Empowering developers to go beyond what standard development systems allow —
taking full control without being constrained by vendor tools that cannot,
will not, or refuse to provide what professional development demands.*
*"I. WILL. NEVER. GIVE. UP. OR. SURRENDER."*
