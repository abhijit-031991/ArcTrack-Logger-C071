#include "AccelMetrics.h"

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────

AccelMetrics::AccelMetrics(float *bufX, float *bufY, float *bufZ, uint16_t capacity)
    : _bx(bufX), _by(bufY), _bz(bufZ),
      _capacity(capacity), _head(0), _count(0)
{}

// ─────────────────────────────────────────────────────────────────────────────
// Sample management
// ─────────────────────────────────────────────────────────────────────────────

void AccelMetrics::addSample(float x, float y, float z) {
    _bx[_head] = x;
    _by[_head] = y;
    _bz[_head] = z;
    _head = (_head + 1) % _capacity;
    if (_count < _capacity) _count++;
}

void AccelMetrics::reset() {
    _head  = 0;
    _count = 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Per-sample utilities (static)
// ─────────────────────────────────────────────────────────────────────────────

float AccelMetrics::magnitude(float x, float y, float z) {
    return sqrtf(x * x + y * y + z * z);
}

float AccelMetrics::pitch_deg(float x, float y, float z) {
    // Rotation about Y axis (forward/back tilt)
    return atan2f(x, sqrtf(y * y + z * z)) * 57.29577951f;
}

float AccelMetrics::roll_deg(float x, float y, float z) {
    // Rotation about X axis (left/right tilt)
    return atan2f(y, sqrtf(x * x + z * z)) * 57.29577951f;
}

// ─────────────────────────────────────────────────────────────────────────────
// Internal iteration note
// ─────────────────────────────────────────────────────────────────────────────
// All windowed metrics iterate indices 0 … (_count - 1).
//
// When the buffer is not yet full (_count < _capacity):
//   Valid entries occupy exactly slots 0 … (_count - 1) in insertion order.
//
// When the buffer is full (_count == _capacity):
//   All slots 0 … (_capacity - 1) are valid.  The circular ordering means
//   slot _head holds the oldest entry, but since every metric here is
//   order-independent (mean, variance, sum) the iteration is correct without
//   reordering.

// ─────────────────────────────────────────────────────────────────────────────
// Static component — per-axis mean (gravity estimate)
// ─────────────────────────────────────────────────────────────────────────────

float AccelMetrics::staticX() const {
    if (_count == 0) return 0.0f;
    float sum = 0.0f;
    for (uint16_t i = 0; i < _count; i++) sum += _bx[i];
    return sum / (float)_count;
}

float AccelMetrics::staticY() const {
    if (_count == 0) return 0.0f;
    float sum = 0.0f;
    for (uint16_t i = 0; i < _count; i++) sum += _by[i];
    return sum / (float)_count;
}

float AccelMetrics::staticZ() const {
    if (_count == 0) return 0.0f;
    float sum = 0.0f;
    for (uint16_t i = 0; i < _count; i++) sum += _bz[i];
    return sum / (float)_count;
}

// ─────────────────────────────────────────────────────────────────────────────
// Mean magnitude
// ─────────────────────────────────────────────────────────────────────────────

float AccelMetrics::meanMagnitude() const {
    if (_count == 0) return 0.0f;
    float sum = 0.0f;
    for (uint16_t i = 0; i < _count; i++)
        sum += magnitude(_bx[i], _by[i], _bz[i]);
    return sum / (float)_count;
}

// ─────────────────────────────────────────────────────────────────────────────
// Variance of magnitude
// Uses the two-pass algorithm: mean first, then sum of squared deviations.
// ─────────────────────────────────────────────────────────────────────────────

float AccelMetrics::varianceMagnitude() const {
    if (_count < 2) return 0.0f;
    float mean = meanMagnitude();
    float sumSq = 0.0f;
    for (uint16_t i = 0; i < _count; i++) {
        float diff = magnitude(_bx[i], _by[i], _bz[i]) - mean;
        sumSq += diff * diff;
    }
    return sumSq / (float)_count;
}

// ─────────────────────────────────────────────────────────────────────────────
// ODBA — Overall Dynamic Body Acceleration
// Static component (per-axis mean) is subtracted from each sample.
// ─────────────────────────────────────────────────────────────────────────────

float AccelMetrics::odba() const {
    if (_count == 0) return 0.0f;
    float sx = staticX();
    float sy = staticY();
    float sz = staticZ();
    float sum = 0.0f;
    for (uint16_t i = 0; i < _count; i++) {
        sum += fabsf(_bx[i] - sx)
             + fabsf(_by[i] - sy)
             + fabsf(_bz[i] - sz);
    }
    return sum / (float)_count;
}

// ─────────────────────────────────────────────────────────────────────────────
// VeDBA — Vectorial Dynamic Body Acceleration
// Mean of per-sample vectorial dynamic acceleration magnitudes.
// ─────────────────────────────────────────────────────────────────────────────

float AccelMetrics::vedba() const {
    if (_count == 0) return 0.0f;
    float sx = staticX();
    float sy = staticY();
    float sz = staticZ();
    float sum = 0.0f;
    for (uint16_t i = 0; i < _count; i++) {
        float dx = _bx[i] - sx;
        float dy = _by[i] - sy;
        float dz = _bz[i] - sz;
        sum += sqrtf(dx * dx + dy * dy + dz * dz);
    }
    return sum / (float)_count;
}

// ─────────────────────────────────────────────────────────────────────────────
// Activity index — normalised VeDBA clamped to 0.0–1.0
// ─────────────────────────────────────────────────────────────────────────────

float AccelMetrics::activityIndex(float maxVedba) const {
    if (maxVedba <= 0.0f) return 0.0f;
    float v = vedba() / maxVedba;
    return (v > 1.0f) ? 1.0f : v;
}

// ─────────────────────────────────────────────────────────────────────────────
// isStationary
// ─────────────────────────────────────────────────────────────────────────────

bool AccelMetrics::isStationary(float threshold) const {
    if (_count == 0) return true;   // empty buffer — assume stationary
    return varianceMagnitude() < threshold;
}

// ─────────────────────────────────────────────────────────────────────────────
// isMortality
// Three conditions must all be true:
//   1. Buffer is full — complete evidence window, no premature triggering.
//   2. isStationary() — no movement detected over the entire window.
//   3. meanMagnitude() within [0.8, 1.2] g — device is gravity-dominated,
//      ruling out a disconnected or zero-reading broken sensor.
// ─────────────────────────────────────────────────────────────────────────────

bool AccelMetrics::isMortality(float threshold) const {
    if (!isFull())             return false;
    if (!isStationary(threshold)) return false;
    float mm = meanMagnitude();
    return (mm >= 0.8f && mm <= 1.2f);
}
