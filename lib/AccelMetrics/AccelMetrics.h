/**
 * AccelMetrics.h
 * ─────────────────────────────────────────────────────────────────────────────
 * IMU-agnostic accelerometer metrics library.
 *
 * Accepts raw x/y/z acceleration samples in g from any sensor and computes
 * a set of biologging and motion-analysis metrics over a fixed-size circular
 * buffer.  No dynamic allocation — the caller supplies the sample buffers.
 *
 * Usage:
 *   float bx[180], by[180], bz[180];
 *   AccelMetrics accel(bx, by, bz, 180);
 *
 *   // Feed samples from any IMU:
 *   accel.addSample(x, y, z);
 *
 *   // Query metrics:
 *   float v    = accel.vedba();
 *   bool  dead = accel.isMortality();
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * Metric reference
 * ─────────────────────────────────────────────────────────────────────────────
 *
 *  Static component  — per-axis mean over the window, approximates the gravity
 *                      vector (low-frequency component of acceleration).
 *
 *  Dynamic component — sample minus static component; pure movement signal
 *                      with gravity removed.
 *
 *  ODBA  (Overall Dynamic Body Acceleration)
 *        mean(|dx|) + mean(|dy|) + mean(|dz|)
 *        Standard biologging proxy for energy expenditure.
 *
 *  VeDBA (Vectorial Dynamic Body Acceleration)
 *        mean( sqrt(dx²+dy²+dz²) )
 *        Direction-independent; preferred over ODBA for most applications.
 *
 *  Variance of magnitude — low when stationary, high when active.
 *  Used as the primary signal for isStationary() and isMortality().
 * ─────────────────────────────────────────────────────────────────────────────
 */

#ifndef ACCELMETRICS_H
#define ACCELMETRICS_H

#include <Arduino.h>
#include <math.h>

class AccelMetrics {
public:
    // ── Construction ─────────────────────────────────────────────────────────
    //
    // bufX/bufY/bufZ  Caller-owned arrays of length `capacity`.
    //                 Must remain valid for the lifetime of this object.
    // capacity        Maximum number of samples the buffer can hold.
    //                 When full, oldest sample is overwritten (circular).
    AccelMetrics(float *bufX, float *bufY, float *bufZ, uint16_t capacity);

    // ── Sample management ─────────────────────────────────────────────────────
    void     addSample(float x, float y, float z);  // push one sample
    void     reset();                               // clear buffer and counters
    uint16_t count()    const { return _count;              }
    bool     isFull()   const { return _count == _capacity; }
    uint16_t capacity() const { return _capacity;           }

    // ── Per-sample utilities (static — no buffer required) ───────────────────

    // Acceleration vector magnitude in g.  Equals ~1.0 g when stationary.
    static float magnitude(float x, float y, float z);

    // Pitch angle in degrees (rotation about the Y axis, forward/back tilt).
    // Valid only when the device is approximately stationary.
    static float pitch_deg(float x, float y, float z);

    // Roll angle in degrees (rotation about the X axis, left/right tilt).
    // Valid only when the device is approximately stationary.
    static float roll_deg(float x, float y, float z);

    // ── Windowed metrics (require at least one buffered sample) ──────────────

    // Per-axis mean over the buffer — approximates the gravity vector.
    float staticX() const;
    float staticY() const;
    float staticZ() const;

    // Mean of per-sample magnitudes over the buffer.
    float meanMagnitude() const;

    // Variance of per-sample magnitudes.
    // Near zero when stationary; grows with movement intensity.
    float varianceMagnitude() const;

    // ODBA — mean absolute dynamic acceleration summed across axes.
    float odba() const;

    // VeDBA — mean vectorial dynamic acceleration.
    float vedba() const;

    // Normalised activity index 0.0–1.0.
    // Pass the maximum VeDBA expected for the animal/use case as `maxVedba`.
    float activityIndex(float maxVedba = 1.0f) const;

    // Returns true if varianceMagnitude() < threshold.
    // threshold  Variance of magnitude in g² below which motion is negligible.
    //            Default 0.01 g² separates stationary from slow walking
    //            reliably for the LSM6DSL at ±2 g.
    bool isStationary(float threshold = 0.01f) const;

    // Returns true when ALL of the following hold:
    //   1. Buffer is full (complete evidence window).
    //   2. varianceMagnitude() < threshold (no movement).
    //   3. meanMagnitude() is within 0.8–1.2 g (gravity-dominated; rules
    //      out a disconnected or broken sensor reading 0 g on all axes).
    //
    // Firmware usage: call once per ping interval.  Flag the mortality field
    // in the long ping.  The base station confirms true mortality by observing
    // the flag set across multiple consecutive pings.
    bool isMortality(float threshold = 0.01f) const;

private:
    float    *_bx;
    float    *_by;
    float    *_bz;
    uint16_t  _capacity;
    uint16_t  _head;    // index of next write slot (circular)
    uint16_t  _count;   // number of valid samples currently held
};

#endif // ACCELMETRICS_H
