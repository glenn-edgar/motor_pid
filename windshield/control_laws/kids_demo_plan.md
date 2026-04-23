# Windshield Algorithms — M0+ and M4 Implementations

**Status:** Draft v0.1
**Companion to:** `windshield.md` (architecture), `windshield_hardware.md` (BOM), `windshield_demo_pinout.md` (pin assignments)
**Scope:** The three smart-window algorithms — endpoint calibration, current-per-position baseline learning, and encoder plausibility checking — implemented at two fidelity levels for the SAMD21 (Cortex-M0+) and RA4M1 (Cortex-M4F) platforms.

---

## 1. Purpose

Two tiers, two implementations of the same algorithmic family. The basic tier (M0+) uses fixed-point math and threshold-based checks; the premium tier (M4F) uses single-precision float and statistically-proper checks. Same architecture, same state machine, same shim protocol — the algorithms themselves diverge at the points where FPU availability and cycle budget meaningfully change what's practical.

This document specifies:

- Shared state-machine and data-flow skeleton
- M0+ algorithms in enough detail to implement
- M4F algorithms in enough detail to implement
- What each algorithm cannot do and when that matters
- Experiment guide: how to make the differences visible on the desktop lever-arm demo

---

## 2. Shared Skeleton

The state machine and data flow are identical across tiers. Platform-specific code is localized behind clean interfaces.

### 2.1 State machine

```
       ┌──────────────┐
       │   UNCAL      │  ← power-up default, no travel known
       └──────┬───────┘
              │ CALIBRATE command
              ▼
       ┌──────────────┐
       │  CALIBRATING │  ← calibration sequence running
       └──────┬───────┘
              │ success
              ▼
       ┌──────────────┐      MOVE command      ┌──────────────┐
       │    IDLE      ├───────────────────────►│   MOVING     │
       │              │◄───────────────────────┤              │
       └──────┬───────┘    move complete       └───────┬──────┘
              │                                         │
              │                                         │
              │                ┌────────────────────────┤
              │                │ anti-pinch detected    │
              │                ▼                        │
              │         ┌──────────────┐                │
              │         │  REVERSING   │                │
              │         └──────┬───────┘                │
              │                │ retract complete       │
              │◄───────────────┘                        │
              │                                         │
              │                ┌────────────────────────┤
              │                │ plausibility/thermal/  │
              │                │ stall-mid-travel fault │
              │                ▼                        │
              │         ┌──────────────┐                │
              └────────►│    FAULT     │                │
                        └──────────────┘                │
                               ▲                        │
                               └────────────────────────┘
                                heartbeat timeout →
                                SAFE_IDLE (subset of FAULT)
```

### 2.2 Data flow per 1 kHz tick during MOVING

```
  (from ISR, updated at 10-20 kHz)
  current_ma, v_bus_mv, encoder_pos, velocity
      │
      ▼
  ┌──────────────────────────────┐
  │  profile executor            │  ← trapezoidal (M0+) or S-curve (M4)
  │  output: pos_ref, v_ref,     │
  │          a_ref (M4 only)     │
  └─────────────┬────────────────┘
                │
                ▼
  ┌──────────────────────────────┐
  │  velocity PI + optional FF   │
  │  output: i_cmd               │  ← to current loop ISR
  └──────────────────────────────┘
                │
                ▼
  ┌──────────────────────────────┐
  │  plausibility checks         │
  │  - direction consistent?     │
  │  - expected velocity?        │
  │  - travel time bounds?       │
  └──────────────┬───────────────┘
                 │ pass
                 ▼
  ┌──────────────────────────────┐
  │  anti-pinch detector         │
  │  - baseline(pos) lookup      │
  │  - Δi check                  │
  │  - dv/dt check               │
  │  - DOB residual (M4 only)    │
  │  - voting                    │
  └──────────────┬───────────────┘
                 │ no pinch
                 ▼
  ┌──────────────────────────────┐
  │  baseline update             │  ← only on good ticks
  │  - running mean (M0+)        │
  │  - full Welford (M4)         │
  └──────────────────────────────┘
```

### 2.3 File and module layout

```
  src/
    core/                            ← shared across tiers
      state_machine.{c,h}            ← state transitions, fault logic
      shim_protocol.{c,h}            ← USB CDC command/telemetry
      calibration_flow.{c,h}         ← orchestration, not detection
      bucket_index.{c,h}             ← position → bucket mapping
      profile_executor.h             ← interface

    platform/
      samd21/                        ← M0+ implementations
        profile_trapezoid.c
        baseline_mean.c
        anti_pinch_fixed.c
        plausibility_basic.c
        hal_samd21.c                 ← TCC, ADC, EIC, EVSYS
      ra4m1/                         ← M4F implementations
        profile_scurve.c
        baseline_welford.c
        anti_pinch_statistical.c
        plausibility_extended.c
        dob.c
        hal_ra4m1.c                  ← GPT, ADC, ELC, POEG
```

Build system selects the platform directory. Core code is identical.

---

## 3. M0+ Algorithm (SAMD21 Basic Tier)

Design principle: **fixed-point only, no square roots, no per-tick divisions, no trigonometry**. Everything fits in Q16.16 signed or int32 counts. PI updates are ~15 cycles, baseline updates ~40 cycles.

### 3.1 Arithmetic conventions

| quantity | format | range | resolution |
|---|---|---|---|
| current (amps) | Q16.16 signed | ±32 A | 15 μA |
| velocity (rad/s, motor shaft) | Q16.16 signed | ±32768 rad/s | very fine |
| position | int32 encoder counts | full encoder range | 1 count |
| PI gains | Q8.24 signed | small integer | many fractional bits |
| PWM duty | Q0.15 signed | ±1.0 | 1/32768 |
| temperature (°C × 10) | int16 | ±3276.7 °C | 0.1 °C |

Key: **Q16.16 multiplies produce Q32.32 intermediates** which are 64-bit on M0+. Use `int64_t` with care, shift back to Q16.16 with `>> 16`. The M0+ has no hardware 64×64 multiply, but 32×32→64 is a single instruction (`SMULL`/`UMULL`).

### 3.2 Calibration — single-pass per direction

```c
typedef enum { CLOSE = +1, OPEN = -1 } direction_t;

#define CAL_VELOCITY_Q           TO_Q16_16(0.1f * V_CRUISE)  // 10% cruise
#define CAL_STALL_V_THRESHOLD_Q  TO_Q16_16(2.0f)             // rad/s
#define CAL_STALL_I_THRESHOLD_MA 200
#define CAL_STALL_CONFIRM_MS     200
#define CAL_TIMEOUT_MS           15000

int32_t pos_max;  // closed endpoint, in encoder counts
int32_t pos_min;  // open endpoint

typedef enum { CAL_OK, CAL_TIMEOUT, CAL_INCONSISTENT } cal_result_t;

cal_result_t calibrate_direction(direction_t dir) {
    set_velocity_setpoint_q(CAL_VELOCITY_Q * dir);
    enable_bridge();

    uint32_t stall_start_ms = 0;
    uint32_t t0 = now_ms();

    while (now_ms() - t0 < CAL_TIMEOUT_MS) {
        delay_ms(20);  // 50 Hz polling adequate for calibration

        int32_t velocity_q = read_velocity_q();
        int16_t current_ma = read_current_averaged_ma(5);

        // stall = low velocity AND high current, held for confirm window
        if (abs_q(velocity_q) < CAL_STALL_V_THRESHOLD_Q &&
            current_ma > CAL_STALL_I_THRESHOLD_MA) {
            if (stall_start_ms == 0) {
                stall_start_ms = now_ms();
            } else if (now_ms() - stall_start_ms > CAL_STALL_CONFIRM_MS) {
                // stall confirmed
                int32_t enc = encoder_position();
                if (dir == CLOSE) pos_max = enc;
                else              pos_min = enc;
                disable_bridge();
                return CAL_OK;
            }
        } else {
            stall_start_ms = 0;  // motion resumed, reset timer
        }
    }

    disable_bridge();
    return CAL_TIMEOUT;
}

cal_result_t calibrate_all(void) {
    // drive a little off each endpoint first, in case we're parked on one
    move_relative_blocking(-100 /* counts */);

    cal_result_t r = calibrate_direction(CLOSE);
    if (r != CAL_OK) return r;

    move_relative_blocking(-100);

    r = calibrate_direction(OPEN);
    if (r != CAL_OK) return r;

    // sanity check: total travel must be within expected bounds
    int32_t travel = pos_max - pos_min;
    if (travel < TRAVEL_MIN_COUNTS || travel > TRAVEL_MAX_COUNTS) {
        return CAL_INCONSISTENT;
    }

    // persist to non-volatile storage
    save_endpoints(pos_min, pos_max);
    return CAL_OK;
}
```

Stall detection is single-signal AND single-pass. Good enough for most cases; occasionally a transient mechanical bind during calibration will bake in a bad endpoint, and the firmware will need a recalibrate command to recover.

### 3.3 Baseline learning — 32-bucket running mean

```c
#define BASELINE_BUCKETS 32

int16_t baseline_i_ma[BASELINE_BUCKETS];  // 64 bytes
uint16_t baseline_n[BASELINE_BUCKETS];    // 64 bytes
#define BASELINE_N_SATURATE 60000

static inline uint8_t pos_to_bucket(int32_t pos_enc) {
    int32_t travel = pos_max - pos_min;
    int32_t rel = pos_enc - pos_min;
    // divide carefully: (rel * BUCKETS) / travel
    // can overflow if not 64-bit
    int64_t num = (int64_t)rel * (int64_t)BASELINE_BUCKETS;
    int32_t bucket = (int32_t)(num / travel);
    if (bucket < 0) return 0;
    if (bucket >= BASELINE_BUCKETS) return BASELINE_BUCKETS - 1;
    return (uint8_t)bucket;
}

void baseline_update(int32_t pos_enc, int16_t current_ma) {
    uint8_t bucket = pos_to_bucket(pos_enc);
    uint16_t n = baseline_n[bucket];
    if (n >= BASELINE_N_SATURATE) return;

    // running mean: new_mean = old_mean + (x - old_mean) / (n+1)
    int32_t diff = (int32_t)current_ma - (int32_t)baseline_i_ma[bucket];
    int32_t delta = diff / (int32_t)(n + 1);
    baseline_i_ma[bucket] += (int16_t)delta;
    baseline_n[bucket] = n + 1;
}
```

Division by (n+1) is the expensive part; M0+ has no hardware divide. Takes ~70 cycles for a general signed division. Called at most 1 kHz during MOVING, so ~70k cycles/sec = 0.15% CPU. Acceptable.

**Critically**: only call `baseline_update()` during *successful, non-anomalous* ticks. Don't call it during REVERSING, during STALL confirmation, or after any fault during the move. Learning the obstacle would be a disaster.

### 3.4 Anti-pinch detection — fixed threshold, two-signal AND

```c
#define PINCH_I_DELTA_MA      300   // how far above baseline before flag
#define PINCH_DV_THRESHOLD_Q  TO_Q16_16(-5.0f)  // rad/s² — velocity dropping fast
#define BASELINE_MIN_N        10    // don't trust baseline below this

bool anti_pinch_check(int32_t pos_enc, int16_t current_ma,
                      int32_t dv_dt_q) {
    uint8_t bucket = pos_to_bucket(pos_enc);

    if (baseline_n[bucket] < BASELINE_MIN_N) {
        // too few samples in this bucket — use conservative absolute threshold
        return current_ma > ABSOLUTE_OVERCURRENT_MA;
    }

    int16_t delta_ma = current_ma - baseline_i_ma[bucket];
    bool vote_i = (delta_ma > PINCH_I_DELTA_MA);
    bool vote_v = (dv_dt_q < PINCH_DV_THRESHOLD_Q);

    return vote_i && vote_v;  // two-signal AND
}
```

Both signals must agree. Current alone false-triggers on cold grease; velocity-drop alone is noisy. The AND is what keeps the false-trigger rate acceptable with fixed thresholds.

**Cold-grease handling fallback**: if a bucket's baseline has saturated (n > N_SATURATE) but current is consistently high across many recent runs, the thresholds could be widened via a slow secondary adaptation. M0+ implementation: skipped. Either recalibrate, or accept that winter mornings need a forced retract-reverse if the door-panel binding is bad. Production units typically handle this by temperature-binned baselines (§3.6).

### 3.5 Temperature compensation — binned tables

```c
#define TEMP_BINS 4   // <0°C, 0-15°C, 15-30°C, >30°C
int16_t baseline_i_ma_t[TEMP_BINS][BASELINE_BUCKETS];  // 256 bytes
uint16_t baseline_n_t[TEMP_BINS][BASELINE_BUCKETS];    // 256 bytes

static inline uint8_t temp_to_bin(int16_t temp_c_x10) {
    if (temp_c_x10 < 0)    return 0;
    if (temp_c_x10 < 150)  return 1;
    if (temp_c_x10 < 300)  return 2;
    return 3;
}

void baseline_update_t(int32_t pos, int16_t current_ma, int16_t temp_c_x10) {
    uint8_t tbin = temp_to_bin(temp_c_x10);
    uint8_t bucket = pos_to_bucket(pos);
    uint16_t n = baseline_n_t[tbin][bucket];
    if (n >= BASELINE_N_SATURATE) return;

    int32_t diff = (int32_t)current_ma - (int32_t)baseline_i_ma_t[tbin][bucket];
    baseline_i_ma_t[tbin][bucket] += (int16_t)(diff / (int32_t)(n + 1));
    baseline_n_t[tbin][bucket] = n + 1;
}

int16_t baseline_lookup_t(int32_t pos, int16_t temp_c_x10) {
    uint8_t bucket = pos_to_bucket(pos);
    uint8_t tbin_lo = temp_to_bin(temp_c_x10);

    // simple: just return the bin's value, no interpolation
    return baseline_i_ma_t[tbin_lo][bucket];
}
```

Four bins, no interpolation. If it matters, linear interpolation between adjacent bins in Q0.8 fixed-point is ~20 cycles. For M0+ default, skip it — the discretization is acceptable.

Temperature source: SAMD21 has no internal temp sensor in the usable range. Add an external NTC thermistor on one of the spare ADC pins (D8 or D9 on the revised pinout), linearize via a small lookup table.

### 3.6 Encoder plausibility — two checks

```c
int32_t last_enc_1khz = 0;
uint16_t ticks_wrong_dir = 0;
uint16_t stall_ticks = 0;

#define DUTY_MIN_FOR_MOTION_Q   TO_Q0_15(0.05f)  // 5% duty
#define WRONG_DIR_TICKS_FAULT   10       // 10 ms
#define STALL_NO_MOTION_TICKS   200      // 200 ms without motion

void plausibility_1khz(void) {
    int32_t enc = encoder_position();
    int32_t enc_delta = enc - last_enc_1khz;
    int16_t duty_q = pwm_duty_q();
    direction_t cmd = commanded_direction;

    // (1) direction mismatch
    if (abs(duty_q) > DUTY_MIN_FOR_MOTION_Q && enc_delta != 0) {
        bool delta_positive = (enc_delta > 0);
        bool cmd_positive = (cmd == CLOSE);  // assumption: CLOSE = +enc
        if (delta_positive != cmd_positive) {
            ticks_wrong_dir++;
            if (ticks_wrong_dir > WRONG_DIR_TICKS_FAULT) {
                fault(ENCODER_DIRECTION_FAULT);
                return;
            }
        } else {
            ticks_wrong_dir = 0;
        }
    } else {
        ticks_wrong_dir = 0;
    }

    // (2) stall-but-commanded
    if (abs(duty_q) > DUTY_MIN_FOR_MOTION_Q && enc_delta == 0) {
        stall_ticks++;
        if (stall_ticks > STALL_NO_MOTION_TICKS) {
            // distinguish "stall at expected endpoint" from "bad stall"
            int32_t pos = encoder_position();
            bool near_endpoint = (pos > pos_max - ENDPOINT_MARGIN) ||
                                  (pos < pos_min + ENDPOINT_MARGIN);
            if (near_endpoint) {
                // legitimate seal stall, let normal move-end logic handle
                // (don't fault, the supervisor will see the move complete)
            } else {
                fault(STALL_MID_TRAVEL);
                return;
            }
        }
    } else {
        stall_ticks = 0;
    }

    last_enc_1khz = enc;
}
```

Two checks. Catches: encoder unplugged mid-move, encoder A/B swapped, motor wires swapped, mechanism seized mid-travel, external force opposing motion. Doesn't catch: slow mechanical wear, gradual sensor drift, commutation-ripple cross-check failures.

### 3.7 What M0+ gives up

Documented explicitly so nobody is surprised:

- **No confidence-aware thresholds.** A bucket with 5 samples uses the same threshold as a bucket with 5000.
- **No per-bucket adaptive threshold.** Quiet regions and noisy regions have the same fixed delta.
- **No V_bus normalization.** Current measurements at 11.5 V and 13.5 V are treated identically, even though the same mechanical load produces different currents.
- **No disturbance observer.** Slow load changes (dust in seal, grease hardening over seasons) get absorbed into baseline as "normal" rather than flagged.
- **No commutation-ripple backup.** If the encoder fails silently (electrically present but logically frozen), plausibility catches it via the stall check, but not quickly and not with a diagnostic that says "encoder" specifically.
- **Coarse fault codes.** 4-bit field: OVERCURRENT, ENCODER, THERMAL, SHIM, STALL_MID, CAL_FAIL, UNKNOWN, NOFAULT. Enough for safe-state decisions, not enough for field forensics without bench tools.

Flash: ~40 KB total for the M0+ variant. RAM: ~1 KB for baselines (with temp bins) + ~2 KB for state machine, buffers, shim. Comfortable in 32 KB.

---

## 4. M4F Algorithm (RA4M1 Premium Tier)

Design principle: **exploit the FPU and extra cycles to replace fixed thresholds with statistical tests and add a disturbance observer**. Same shape of algorithm, better at each step.

### 4.1 Arithmetic conventions

All quantities in `float` (IEEE 754 single-precision). FPU handles add/sub/mul in 1 cycle, divide in ~14, sqrt in ~14. Budget comfortably allows these in the 1 kHz task.

### 4.2 Calibration — multi-pass with statistical confirmation

```c
#define CAL_PASSES           3
#define CAL_BACKOFF_FRAC     0.1f
#define CAL_STDDEV_MAX_MM    1.5f    // endpoints must agree within 1.5 mm

typedef enum { CAL_OK_M4, CAL_TIMEOUT_M4, CAL_INCONSISTENT_M4 } cal_result_m4_t;

cal_result_m4_t calibrate_direction_m4(direction_t dir) {
    float positions[CAL_PASSES];

    for (int i = 0; i < CAL_PASSES; i++) {
        cal_result_m4_t r = find_stall_once_m4(dir, &positions[i]);
        if (r != CAL_OK_M4) return r;

        // back off before the next pass to clear any transient bind
        float backoff = -CAL_BACKOFF_FRAC * expected_travel_mm * dir_sign(dir);
        move_relative_blocking_m4(backoff);
    }

    // statistical check: all three stall positions must cluster tightly
    float mean = 0.0f;
    for (int i = 0; i < CAL_PASSES; i++) mean += positions[i];
    mean /= (float)CAL_PASSES;

    float sum_sq = 0.0f;
    for (int i = 0; i < CAL_PASSES; i++) {
        float d = positions[i] - mean;
        sum_sq += d * d;
    }
    float stddev = sqrtf(sum_sq / (float)(CAL_PASSES - 1));

    if (stddev > CAL_STDDEV_MAX_MM) {
        return CAL_INCONSISTENT_M4;
    }

    if (dir == CLOSE) pos_max_mm = mean;
    else              pos_min_mm = mean;

    return CAL_OK_M4;
}
```

Three passes. If they disagree by more than 1.5 mm (on a ~450 mm travel), something was inconsistent — grit in the mechanism, a transient voltage sag, a loose cable. Refuse to complete calibration rather than baking the bad endpoint in.

Stall detection within `find_stall_once_m4()` uses **three signals**: velocity droop, current rise, AND disturbance-observer residual. The DOB residual at a true seal/foam endpoint shows a characteristic smooth ramp as material compresses; at a mid-travel obstacle it's sharper. The signatures can be distinguished in the calibration logic to reject "calibrated on an accidental obstacle" cases.

### 4.3 Baseline learning — full Welford per bucket

```c
#define BASELINE_BUCKETS_M4 64
#define V_BUS_NOMINAL_V     12.0f

typedef struct {
    uint32_t n;
    float mean_i;        // mean current, V_bus-normalized
    float M2_i;          // for current variance
    float mean_tau;      // mean DOB residual (for mechanical-health tracking)
    float M2_tau;        // for DOB variance
    float temp_slope;    // A/°C, learned
    uint32_t last_temp_update_n;
} welford_bucket_t;

welford_bucket_t baseline[BASELINE_BUCKETS_M4];  // 64 * ~32 = 2048 bytes

static inline uint8_t pos_to_bucket_m4(float pos_mm) {
    float travel = pos_max_mm - pos_min_mm;
    float rel = pos_mm - pos_min_mm;
    int32_t b = (int32_t)(rel * (float)BASELINE_BUCKETS_M4 / travel);
    if (b < 0) return 0;
    if (b >= BASELINE_BUCKETS_M4) return BASELINE_BUCKETS_M4 - 1;
    return (uint8_t)b;
}

void baseline_update_m4(float pos_mm, float i_a, float v_bus_v,
                        float tau_l_hat, float temp_c) {
    // V_bus normalization: same mechanical load at different bus voltages
    // produces different currents. Normalize to V_NOMINAL before learning.
    float i_norm = i_a * (V_BUS_NOMINAL_V / v_bus_v);

    uint8_t bucket = pos_to_bucket_m4(pos_mm);
    welford_bucket_t *w = &baseline[bucket];

    w->n += 1;

    // Welford for current
    float delta = i_norm - w->mean_i;
    w->mean_i += delta / (float)w->n;
    float delta2 = i_norm - w->mean_i;
    w->M2_i += delta * delta2;

    // Welford for DOB residual
    float delta_t = tau_l_hat - w->mean_tau;
    w->mean_tau += delta_t / (float)w->n;
    float delta2_t = tau_l_hat - w->mean_tau;
    w->M2_tau += delta_t * delta2_t;

    // temperature slope update (slow, 1-in-10 ticks)
    if ((w->n - w->last_temp_update_n) >= 10) {
        float expected_at_nom = w->mean_i;
        float residual = i_norm - expected_at_nom;
        float temp_delta = temp_c - TEMP_NOMINAL_C;
        if (fabsf(temp_delta) > 2.0f) {
            // rough linear-regression update, low gain
            w->temp_slope += 0.01f * (residual / temp_delta - w->temp_slope);
        }
        w->last_temp_update_n = w->n;
    }
}

static inline float baseline_stddev_i(uint8_t bucket) {
    welford_bucket_t *w = &baseline[bucket];
    if (w->n < 2) return INFINITY;
    return sqrtf(w->M2_i / (float)(w->n - 1));
}

static inline float baseline_stddev_tau(uint8_t bucket) {
    welford_bucket_t *w = &baseline[bucket];
    if (w->n < 2) return INFINITY;
    return sqrtf(w->M2_tau / (float)(w->n - 1));
}

float baseline_lookup_i(uint8_t bucket, float temp_c) {
    welford_bucket_t *w = &baseline[bucket];
    return w->mean_i + w->temp_slope * (temp_c - TEMP_NOMINAL_C);
}
```

About 32 bytes per bucket × 64 buckets = 2 KB. Easily fits.

V_bus normalization is the critical move: it removes bus voltage as a nuisance variable so the baseline reflects mechanical condition only. Without it, a 1 V battery sag looks like increased mechanical load.

Welford + DOB cost: ~30 FPU operations per update, ~50 cycles on M4F. At 1 kHz during MOVING = 50k cycles/sec = 0.1% CPU. Cheap.

### 4.4 Anti-pinch — statistical z-score + 2-of-3 voting

```c
#define PINCH_Z_THRESHOLD_I    4.0f    // 4 sigma on current
#define PINCH_DV_THRESHOLD     -5.0f   // rad/s² on velocity derivative
#define PINCH_K_TAU            3.0f    // sigma multiplier for DOB
#define BASELINE_MIN_N_M4      30      // need 30 samples before trusting

bool anti_pinch_check_m4(float pos_mm, float i_a, float v_bus_v,
                         float dv_dt, float tau_l_hat, float temp_c) {
    uint8_t bucket = pos_to_bucket_m4(pos_mm);
    welford_bucket_t *w = &baseline[bucket];

    if (w->n < BASELINE_MIN_N_M4) {
        // immature baseline: fall back to absolute threshold
        return i_a > ABSOLUTE_OVERCURRENT_A;
    }

    // V_bus-normalize current before comparing to baseline
    float i_norm = i_a * (V_BUS_NOMINAL_V / v_bus_v);
    float expected_i = baseline_lookup_i(bucket, temp_c);
    float sigma_i = baseline_stddev_i(bucket);
    float z_i = (i_norm - expected_i) / sigma_i;

    // DOB residual against its own baseline
    float expected_tau = w->mean_tau;
    float sigma_tau = baseline_stddev_tau(bucket);
    float delta_tau = tau_l_hat - expected_tau;

    bool vote_i   = (z_i > PINCH_Z_THRESHOLD_I);
    bool vote_v   = (dv_dt < PINCH_DV_THRESHOLD);
    bool vote_tau = (delta_tau > PINCH_K_TAU * sigma_tau);

    int votes = (int)vote_i + (int)vote_v + (int)vote_tau;
    return votes >= 2;  // any two agree → pinch
}
```

Three differences from the M0+ version:

1. **Per-bucket adaptive threshold via z-score.** A quiet mid-travel bucket (low σ) trips on a small absolute deviation; a noisy near-endpoint bucket (high σ) requires a larger deviation. Same sigma-threshold everywhere, translated to different amp-thresholds by the bucket's learned variance.
2. **Third independent signal from DOB.** The disturbance observer measures load torque via motor dynamics; the measurement failure mode of Kt·î (current sensor noise) doesn't affect it. 2-of-3 voting makes the detector robust against single-signal anomalies.
3. **Confidence-aware fallback.** Baselines with n < 30 aren't trusted for statistical reasoning; the detector uses an absolute threshold until enough data accumulates. M0+ has a similar guard (§3.4) but the M4 version can be more nuanced — the threshold in the fallback can itself be tuned per temperature bin, for instance.

### 4.5 Disturbance observer

Runs in the 1 kHz task. First-order low-pass of the torque-balance residual:

```c
float tau_l_hat = 0.0f;
float omega_prev = 0.0f;

#define DOB_CUTOFF_HZ  20.0f
#define DT_1KHZ        0.001f

void dob_update_m4(float i_a, float omega_rad_s) {
    // instantaneous torque-balance residual
    float domega_dt = (omega_rad_s - omega_prev) / DT_1KHZ;
    float tau_l_raw = KT * i_a - J_MOTOR * domega_dt - B_MOTOR * omega_rad_s;

    // first-order LPF
    float alpha = 2.0f * (float)M_PI * DOB_CUTOFF_HZ * DT_1KHZ;
    if (alpha > 1.0f) alpha = 1.0f;
    tau_l_hat += alpha * (tau_l_raw - tau_l_hat);

    omega_prev = omega_rad_s;
}
```

20 Hz cutoff is low enough to reject commutation ripple and high-frequency measurement noise, high enough to track realistic load changes (cold-grease transitions, foam compression).

The DOB output has three roles:

1. **Feedforward into velocity loop** — accelerates disturbance rejection beyond PI bandwidth.
2. **Third anti-pinch vote** — independent of current-sensor measurement chain.
3. **Mechanical-health tracking** — slow drift in `tau_l_hat` statistics across runs = bearing wear, belt slippage, seal grit buildup. Welford statistics on τ̂_L feed into ChainTree maintenance signals.

### 4.6 Encoder plausibility — extended

Same two checks as M0+, plus three more:

```c
// check 3: velocity/duty plausibility
// At steady state, omega ≈ (V_bus * duty - i*R) / Ke
// If measured omega is very far from predicted, something's decoupled
void plausibility_velocity_duty(float duty, float omega, float i_a, float v_bus) {
    float predicted_omega = (v_bus * duty - i_a * MOTOR_R) / MOTOR_KE;
    float ratio = fabsf(omega) / fabsf(predicted_omega + 0.01f);

    if (fabsf(duty) > 0.2f && fabsf(omega) > 0.5f) {  // only when meaningful
        if (ratio > 2.5f || ratio < 0.3f) {
            plausibility_velocity_mismatch_ticks++;
            if (plausibility_velocity_mismatch_ticks > 50) {
                fault(VELOCITY_DUTY_IMPLAUSIBLE);
            }
        } else {
            plausibility_velocity_mismatch_ticks = 0;
        }
    }
}

// check 4: travel-time bounds
// A commanded move must complete within T_max, and must not complete
// faster than T_min.  Learned from successful calibration moves.
void plausibility_travel_time(void) {
    if (move_in_progress) {
        uint32_t elapsed = now_ms() - move_start_ms;
        if (elapsed > travel_time_max_ms) {
            fault(TRAVEL_TOO_SLOW);
        }
    }
    // travel-too-fast check happens at move completion
}

// check 5: (optional) commutation-ripple cross-check
// Brushed motors produce a current AC component at
// (commutator_segments * rpm / 60) Hz.  Counting this gives a position
// estimate independent of the encoder.
// Implementation: bandpass filter on current signal, zero-crossing counter,
// compare accumulated count to encoder delta over 100 ms window.
// Heavy — only do this when the ADC has headroom (20 kHz sampling on M4F).
void plausibility_commutation_ripple(void) {
    // sketch only — full implementation is nontrivial
    // maintained for completeness; may be deferred
}
```

Checks 3 and 4 are almost free. Check 5 is expensive but makes the fault detection true diverse-sensor: encoder failure is caught by an independent position estimator, not just by stall heuristics.

### 4.7 Fault forensics

```c
typedef struct {
    uint32_t uptime_ms;
    int32_t  pos_counts;
    float    velocity_rad_s;
    float    current_a;
    float    v_bus_v;
    float    tau_l_hat;
    uint8_t  state;
    uint8_t  flags;
} telemetry_frame_t;

#define RING_SIZE 128
telemetry_frame_t ring[RING_SIZE];  // ~5 KB
uint32_t ring_head = 0;

void telemetry_ring_push(void) {
    telemetry_frame_t *f = &ring[ring_head];
    f->uptime_ms = now_ms();
    f->pos_counts = encoder_position();
    f->velocity_rad_s = read_velocity_float();
    // ... populate rest
    ring_head = (ring_head + 1) % RING_SIZE;
}

// 32-bit structured fault code
// [31:24] layer (hw, current, encoder, thermal, shim, cal, baseline)
// [23:16] subsystem (which check fired)
// [15:0]  specific code / context
void fault_log(uint32_t code) {
    // snapshot the ring buffer
    save_ring_to_nvm(ring, ring_head);
    save_fault_code(code);
    transition_to_fault_state();
}
```

On fault, the preceding 128 ms of full state gets persisted with the fault code. Field diagnostic: pull the log over USB CDC, see exactly what was happening 100 ms before the failure. Without bench instrumentation. This is what separates "something went wrong" from "the current spiked at bucket 47 during a close move at 11.8 V bus, 0.3 rad/s velocity, τ̂_L = 0.08 N·m above bucket baseline, which is 6.2 sigmas high."

### 4.8 What M4F buys

Compared to M0+:

- **Confidence-aware thresholds** — rare buckets are treated as rare, common ones get tight thresholds.
- **Per-bucket adaptive thresholds** — quiet regions catch soft obstacles, noisy regions don't false-trigger.
- **V_bus-normalized baselines** — battery voltage doesn't corrupt learning.
- **Disturbance observer** — third independent signal, plus mechanical-health tracking.
- **Extended plausibility checks** — catch more failure modes, sooner.
- **Structured fault codes + ring buffer** — field-diagnosable failures.
- **Continuous temperature compensation** via learned slope per bucket — smoother than binned tables.

Flash: ~90 KB. RAM: ~8 KB (baseline table + ring buffer + state machine + buffers). Comfortable in 32 KB.

---

## 5. Comparison Summary

| concern | M0+ (SAMD21) | M4F (RA4M1) |
|---|---|---|
| calibration passes | 1 | 3 with stddev gate |
| stall detection signals | 2 (v, i) | 3 (v, i, DOB) |
| baseline buckets | 32 | 64 |
| baseline statistics | mean only | mean + variance (Welford) |
| anti-pinch threshold | fixed delta | per-bucket z-score |
| anti-pinch signals | 2 (i, dv/dt) AND | 3 (i, dv/dt, τ̂) 2-of-3 voting |
| V_bus normalization | no | yes |
| temperature compensation | 4 discrete bins | continuous learned slope per bucket |
| disturbance observer | — | yes |
| encoder plausibility checks | 2 | 5 |
| commutation-ripple backup | — | optional |
| fault forensics | 4-bit code | 32-bit + 128 ms ring buffer |
| flash | ~40 KB | ~90 KB |
| RAM | ~1 KB baselines | ~8 KB all state |
| CPU during MOVING | ~5% | ~15% |

Both are production-viable. The M0+ version is appropriate for budget-tier vehicles, appliances, and cost-sensitive industrial actuators. The M4F version is appropriate for comfort/luxury vehicles, safety-critical applications, and anywhere fleet diagnostics or mechanical-health prediction is valuable.

---

## 6. Experiment Guide — Making the Differences Visible

On the desktop lever-arm demo, these experiments show the algorithmic differences concretely:

### Experiment 1 — The soft obstacle

Setup: build a very soft foam obstacle (upholstery foam, thin). It adds only a small current load — maybe 40 mA above baseline — when the lever touches it.

- **M0+**: with `PINCH_I_DELTA_MA = 300`, a 40 mA obstacle is invisible. Lever crushes it.
- **M4F**: in a quiet mid-arc bucket where σ ≈ 8 mA, a 40 mA obstacle is 5 σ above baseline. Detected cleanly. Lever reverses.

Teaching point: fixed thresholds trade sensitivity for false-alarm rate; statistical thresholds adapt to what each position actually looks like.

### Experiment 2 — The "fresh calibration" problem

Setup: factory-reset the device. Immediately command 3 moves. Insert the soft obstacle on the 4th move.

- **M0+**: baselines after 3 moves have n=3 per bucket. Fixed threshold still applies. If the obstacle is small enough to slip under `PINCH_I_DELTA_MA`, it's missed.
- **M4F**: baselines after 3 moves have n=3. M4F sees n < 30 and falls back to absolute threshold. Obstacle below absolute threshold but above fresh-baseline z-score gets missed — same behavior as M0+ until maturity. After 30+ moves, the M4F starts catching it cleanly while M0+ still cannot.

Teaching point: learning takes time. Both algorithms are weakest immediately after calibration; M4F just recovers faster and knows when it's recovered.

### Experiment 3 — The "bus voltage drop" scenario

Setup: run normal moves at 12 V. Drop bus to 10 V (simulated weak battery via the bench supply). Command a move.

- **M0+**: current at 10 V is lower than baseline (learned at 12 V) at every position. Looks like "lower than normal" → not a pinch, so no false trigger. Good. But if an obstacle now appears, it still has to exceed the fixed delta above the *12 V baseline*, which is a higher bar — missed obstacles.
- **M4F**: V_bus normalization converts the 10 V current to a 12 V-equivalent number before comparing. Baseline comparison is stable across bus voltage. Obstacle is detected at its true magnitude.

Teaching point: nuisance variables must be normalized out, or they burn threshold budget.

### Experiment 4 — The "gradual wear" scenario

Setup: slowly tighten a friction clip on the lever rig over 20+ runs, so the current slowly rises at all positions.

- **M0+**: baseline adapts to the rising current slowly. No fault, no warning. The system just silently gets worse at detecting obstacles because the baseline is rising toward the obstacle level.
- **M4F**: baseline *mean* adapts similarly. But the DOB-residual variance per bucket starts rising (mechanical transient signatures change), and long-term drift in τ̂_L mean is flagged as a maintenance-predictive signal in the ChainTree ltree KB.

Teaching point: the system that tells you about wear is different from the system that handles it silently. You want both.

### Experiment 5 — The unplugged encoder

Setup: mid-move, pull the encoder cable.

- **M0+**: stall-ticks counter increments, faults at 200 ms with code STALL_MID_TRAVEL. Lever stops safely. Diagnosis: "something stopped." Bench work required to isolate.
- **M4F**: velocity-duty plausibility check fires within ~50 ms (PWM at 50% but velocity = 0 is a massive ratio mismatch). Faults with code VELOCITY_DUTY_IMPLAUSIBLE and a snapshot showing last 128 ms of state. Diagnosis: "encoder disconnected during motion at bucket 28, velocity went from 2.3 rad/s to 0 in one tick." No bench work needed.

Teaching point: diversity of checks catches faults sooner and with more context.

Each experiment is a 5–15 minute bench exercise. The concepts are generalizable far beyond window control.

---

## 7. Open Items

1. **NTC thermistor selection** for SAMD21 temperature compensation. Curve linearization table in firmware.
2. **Commutation-ripple detector** for M4F — implement as an optional module, deferred from initial demo.
3. **Non-volatile storage mechanism** for baselines. SAMD21: built-in NVM controller, needs wear-leveling plan. RA4M1: 8 KB EEPROM onboard, simpler.
4. **Factory-vs-learned mode toggle** — some applications want a factory-set baseline shipped, with learning refining it over time. Specify the merging policy.
5. **Cross-tier baseline migration** — if a vehicle upgrades from M0+ to M4F controllers, can learned data transfer? In principle yes (means transfer, Welford n resets conservatively). Scope for later.

---

## Appendix A — Cycle Budget Verification

### SAMD21 @ 48 MHz, 1 kHz motion task

| operation | cycles | notes |
|---|---|---|
| read encoder + compute velocity | 80 | hardware counter read + M/T logic |
| profile executor (trapezoid) | 200 | three-state machine, Q16.16 math |
| velocity PI | 80 | Q16.16 multiply + saturate |
| plausibility checks (2) | 120 | integer comparisons |
| anti-pinch check | 100 | bucket lookup + AND |
| baseline update | 150 | running mean with divide |
| telemetry push (every 10 ticks) | 50 avg | structured copy |
| **total avg** | **780** | 1.6% of 48000 tick budget |
| **peak** | **~1200** | when all checks fire |

Plenty of margin for shim handling, NVM flushing, and future additions.

### RA4M1 @ 48 MHz, 1 kHz motion task

| operation | cycles | notes |
|---|---|---|
| read encoder (GPT reg) | 20 | single register |
| compute velocity | 40 | float division |
| S-curve executor | 250 | polynomial eval in current segment |
| velocity PI + FF | 150 | FPU ops |
| disturbance observer | 60 | 1 divide, LPF |
| plausibility checks (5) | 300 | includes floats |
| anti-pinch (z-score + voting) | 250 | 2 sqrts + compares |
| baseline Welford update | 200 | 2 Welford updates |
| temperature slope update (1/10) | 50 avg | amortized |
| telemetry ring push | 80 | 32-byte struct copy |
| **total avg** | **1400** | 2.9% of 48000 tick budget |
| **peak** | **~2500** | with commutation-ripple enabled |

Also comfortable margin.

## Appendix B — Reference

- Welford, B. P. (1962). "Note on a method for calculating corrected sums of squares and products."
- Knuth, D. E. (1998). *The Art of Computer Programming, Vol. 2*, §4.2.2.
- Astrom & Murray, *Feedback Systems* (2010), Ch. 7 (state estimation, disturbance observers)
- Ohnishi, K. (1987). "A new servo method in mechatronics." (classic DOB paper)
- FMVSS 118 S5 — US power-operated window safety standard
- UNECE R21 — UN regulation on interior fittings, §5.8 anti-pinch
