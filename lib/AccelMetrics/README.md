# AccelMetrics

IMU-agnostic accelerometer metrics library for embedded wildlife tracking devices.

Accepts raw x/y/z acceleration samples in **g** from any sensor and computes
biologging and motion-analysis metrics over a fixed-size circular buffer.
No dynamic memory allocation — the caller supplies the sample arrays.

---

## Design principles

- **IMU-agnostic** — the library never talks to hardware. The caller reads
  from whatever IMU is present and passes `float x, y, z` values in g.
- **No heap allocation** — the caller declares static arrays and passes them
  to the constructor. Zero `malloc` / `new` inside the library.
- **No dependencies** — only `<Arduino.h>` and `<math.h>`.
- **Circular buffer** — oldest samples are overwritten automatically once
  the buffer is full. All windowed metrics always reflect the most recent
  `capacity` samples.

---

## Quick start

```cpp
#include <AccelMetrics.h>

// 1. Declare static sample buffers (sized to your window)
float bx[48], by[48], bz[48];

// 2. Create the AccelMetrics instance
AccelMetrics accel(bx, by, bz, 48);

// 3. Feed samples from any IMU
float x, y, z;
myImu.read(x, y, z);          // whatever your IMU's read function is
accel.addSample(x, y, z);

// 4. Query metrics
float  v    = accel.vedba();
bool   dead = accel.isMortality();
```

---

## Buffer sizing

The buffer size determines the **evidence window** for all windowed metrics.
Choose it based on how often you add samples and how long a window you need.

| Sampling strategy | Buffer size | Evidence window |
|---|---|---|
| Once per GPS fix (3 min interval) | 48 | ~2.4 hours |
| Once per GPS fix (3 min interval) | 10 | ~30 minutes |
| Every 10 s (continuous) | 180 | 30 minutes |
| Every 1 s (continuous) | 300 | 5 minutes |

Memory cost: `capacity × 3 axes × 4 bytes`.
At 48 samples: **576 bytes**.
At 180 samples: **2 160 bytes**.

---

## API reference

### Constructor

```cpp
AccelMetrics(float *bufX, float *bufY, float *bufZ, uint16_t capacity);
```

| Parameter  | Description |
|---|---|
| `bufX/Y/Z` | Caller-owned float arrays, each of length `capacity`. Must stay valid for the object's lifetime. |
| `capacity`  | Maximum number of samples. Range: 1–65 535. |

---

### Sample management

#### `addSample(float x, float y, float z)`
Push one acceleration sample into the circular buffer.
Units must be **g** (1 g ≈ 9.81 m/s²). When the buffer is full the oldest
sample is overwritten.

#### `reset()`
Clear the buffer and reset all counters. Use this to start a fresh evidence
window (e.g. after a confirmed mortality event is reported).

#### `count()` → `uint16_t`
Number of valid samples currently held. Increases from 0 up to `capacity`,
then stays at `capacity`.

#### `isFull()` → `bool`
`true` when `count() == capacity`. Most windowed metrics give more reliable
results once the buffer is full.

#### `capacity()` → `uint16_t`
The maximum number of samples declared at construction.

---

### Per-sample utilities  *(static — no buffer required)*

These can be called without an instance:

#### `magnitude(float x, float y, float z)` → `float`
```
√(x² + y² + z²)
```
Returns the total acceleration vector length in g.
A stationary device reads approximately **1.0 g** (gravity only).

```cpp
float m = AccelMetrics::magnitude(x, y, z);
```

#### `pitch_deg(float x, float y, float z)` → `float`
Forward/back tilt angle in degrees, computed from the gravity vector.
Valid only when the device is approximately stationary.

```
atan2(x, √(y² + z²)) × (180 / π)
```

#### `roll_deg(float x, float y, float z)` → `float`
Left/right tilt angle in degrees, computed from the gravity vector.
Valid only when the device is approximately stationary.

```
atan2(y, √(x² + z²)) × (180 / π)
```

---

### Windowed metrics

All methods below operate on the samples currently in the buffer.
Results improve as the buffer fills up — check `isFull()` or `count()`
if you need a minimum number of samples before trusting a result.

#### `staticX()` / `staticY()` / `staticZ()` → `float`
Per-axis mean over all buffered samples.

Approximates the **static gravity vector** (low-frequency component of
acceleration). When the device is stationary this equals the projection
of 1 g onto each axis depending on device orientation.

Used internally by `odba()` and `vedba()` to separate dynamic from static
acceleration.

#### `meanMagnitude()` → `float`
Mean of per-sample `magnitude()` values over the buffer.
Near 1.0 g when the device is mostly stationary.

#### `varianceMagnitude()` → `float`
Variance of per-sample `magnitude()` values over the buffer (population
variance, not sample variance).

| Typical value | Interpretation |
|---|---|
| < 0.001 g² | Completely stationary (on a bench) |
| < 0.01 g²  | Resting animal, slow postural shifts |
| 0.01–0.1 g² | Slow walking |
| > 0.1 g²   | Running / vigorous activity |

This is the primary signal used by `isStationary()` and `isMortality()`.

#### `odba()` → `float`
**Overall Dynamic Body Acceleration** — a standard biologging proxy for
energy expenditure.

```
ODBA = mean(|x − staticX|) + mean(|y − staticY|) + mean(|z − staticZ|)
```

Returns a value in g. Zero when the device is completely stationary,
higher during more energetic movement.

Reference: Wilson et al. (2006). *Moving towards acceleration for estimates
of activity-specific metabolic rate in free-living animals.*

#### `vedba()` → `float`
**Vectorial Dynamic Body Acceleration** — the direction-independent variant
of ODBA, preferred for most applications because it is independent of
device orientation on the animal.

```
VeDBA = mean( √((x−staticX)² + (y−staticY)² + (z−staticZ)²) )
```

Returns a value in g.

#### `activityIndex(float maxVedba = 1.0f)` → `float`
Normalised activity score in the range **0.0–1.0**.

```
activityIndex = clamp(vedba() / maxVedba, 0, 1)
```

`maxVedba` is the VeDBA value that maps to 1.0 (maximum expected activity
for your animal and attachment site). Calibrate this against known
high-activity events for your specific deployment.

#### `isStationary(float threshold = 0.01f)` → `bool`
Returns `true` if `varianceMagnitude() < threshold`.

`threshold` is in g². The default of **0.01 g²** reliably separates a
completely stationary device from slow walking on the LSM6DSL at ±2 g full
scale. Adjust if using a different IMU or full-scale range.

#### `isMortality(float threshold = 0.01f)` → `bool`
Returns `true` when all three conditions hold simultaneously:

1. **Buffer is full** — ensures a complete evidence window before triggering.
2. **`isStationary(threshold)` is true** — no meaningful movement detected
   across the entire window.
3. **`meanMagnitude()` is in [0.8, 1.2] g** — device reads gravity normally,
   ruling out a disconnected sensor (which would read ~0 g on all axes).

```
Recommended firmware usage:
  - Call once per ping interval.
  - Set the `mortality` flag in the long ping struct.
  - Let the base station confirm true mortality by observing the flag
    set across multiple consecutive pings (e.g. 3+ pings = definite).
```

The buffer should be sized to the **minimum evidence period** you require
before raising a mortality alert. Examples:

| GPS interval | Buffer size | Evidence period |
|---|---|---|
| 3 min | 10 | 30 min (early warning) |
| 3 min | 48 | ~2.4 hours (conservative) |
| 10 min | 24 | 4 hours |

---

## Burst-average pattern (recommended for low-power devices)

Instead of continuous sampling, take N rapid samples at an existing
wake-up event (e.g. GPS acquisition) and average them before calling
`addSample()`. This gives a high-quality single data point per interval
with minimal power overhead.

```cpp
// Take 50 samples at the IMU's ODR and average
float sumX = 0, sumY = 0, sumZ = 0;
for (uint8_t i = 0; i < 50; i++) {
    float x, y, z;
    myImu.read(x, y, z);
    sumX += x; sumY += y; sumZ += z;
    delay(20);   // match your IMU's ODR period
}
accel.addSample(sumX / 50.0f, sumY / 50.0f, sumZ / 50.0f);
```

`isMortality()` works correctly with this pattern: the variance it tests
is the variance **across burst-averaged snapshots**, which is near zero
for a dead device and non-zero for a resting-but-alive animal that shifts
posture between GPS fixes.

---

## Notes

- All metrics assume inputs in **g**. If your IMU returns m/s² divide by
  9.80665 before calling `addSample()`.
- `varianceMagnitude()` uses population variance (÷ N). For small buffers
  (< 10 samples) results may not be statistically robust.
- `isMortality()` is conservative by design — it requires the buffer to be
  **completely full** before returning `true`. This prevents false positives
  at startup when only a few samples have been collected.
