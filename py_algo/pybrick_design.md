# Pybrick-Style Motion Control Design

A design document for a Pybricks-class wheeled-robot control stack, consisting of the existing `pbio` motion-control core augmented with a richer sensor architecture (fast 6-DOF IMU + Bosch chip as a calibrated magnetometer peripheral), a firmware-owned orientation filter, and a world-pose layer.

---

## 1. Scope

- Wheeled differential-drive (two driven wheels) robot at LEGO scale.
- Geared brushed-DC motors with shaft encoders, no current sensing.
- STM32-class MCU hub running a cooperative scheduler.
- Goal: accurate motion execution, drift-bounded heading, usable world-pose estimate, and a path for following reference trajectories beyond simple arcs.

## 2. Existing Motion Control Core (Kept)

The existing `pbio` stack is structurally sound and is retained unchanged.

### 2.1 Pipeline

```
reference trajectory ──► observer ──► feedback law ──► inverse model ──► motor
```

### 2.2 Trajectory Generation

Three-phase trapezoidal profiles (accelerate → cruise → decelerate), precomputed at command time from `(target, max_speed, accel, decel)`. Sampled analytically each control tick for `(θ_ref, ω_ref, α_ref)`.

Primitives: `drive_straight`, `drive_turn`, `drive_arc_*`, `drive_forever`. All finite primitives funnel through `pbio_drivebase_drive_relative` as simultaneous distance + heading trajectories, time-synchronized by `pbio_trajectory_stretch` so distance and heading progress stay locked.

### 2.3 Luenberger Observer (per motor)

Discrete state-space `x(k+1) = Ax(k) + Bu(k)` with state `[θ̂, ω̂, î]` (angle, speed, current). Measurement update on encoder angle only; current is instrumental (improves speed estimate through the DC-motor coupling). Coulomb friction modeled with linear transition through zero. Stall detection via feedback-voltage residual.

### 2.4 Feedback Law

Gain-scheduled PID per motor:

- `Kp(position_error, commanded_speed)` boosts gain near the endpoint to overcome stiction.
- `Kd` multiplies speed error against the observer's `ω̂` (model-filtered, not numerically differentiated).
- `Ki` integrates position error (or position error of a speed-integral for timed commands).
- Torque clamped to `actuation_max_temporary`.

Anti-windup: pause the trajectory clock and the integrator when proportional torque saturates in a direction the reference isn't trying to reverse.

### 2.5 Feedforward

Inverse motor model:

```
τ_ff = ½ · τ_friction · sign(ω_ref) + (ω_ref / Kω) + (α_ref · J)
```

Added to the PID torque before conversion to PWM voltage via the static motor constant `d_voltage_d_torque`. No inner current loop — the voltage command is open-loop on torque.

### 2.6 Drivebase Decoupling

Two independent controllers in orthogonal coordinates:

- **distance** = (θ_L + θ_R) / 2 → base-speed controller
- **heading** = (θ_L − θ_R) → heading controller

Per-wheel voltage mix:

```
V_L = V(τ_dist + τ_head) + V_ff_L
V_R = V(τ_dist − τ_head) + V_ff_R
```

Heading gets actuation priority on saturation so the robot steers straight even when torque-limited.

## 3. Sensor Architecture

### 3.1 Components

| Component | Part (example) | Role |
|---|---|---|
| Fast 6-DOF IMU | BMI270 or LSM6DSO | Body rate (gyro), linear accel, tilt/gravity |
| Bosch 9-DOF fused chip | BNO085 (preferred) or BNO055 | Calibrated magnetic field vector |
| Wheel encoders | existing | Per-motor shaft angle; drivebase distance/heading |

### 3.2 Role of the Bosch Chip

**The Bosch chip is used as a calibrated-magnetometer peripheral, not as a fusion source.**

The chip runs its on-die fusion internally (for the purpose of calibration) but only its *calibrated magnetic field output* is consumed. The chip's continuous auto-calibration maintains:

- **Hard-iron offset** `(x₀, y₀, z₀)` — subtracted from raw mag readings to correct fixed ferromagnetic bias rigidly attached to the robot.
- **Soft-iron correction** — 3×3 matrix reshaping the reading ellipsoid back to a sphere.
- **Gyro bias estimate** `(ωx₀, ωy₀, ωz₀)` — available via calibrated gyro output if needed.

Together these eliminate rotation-dependent heading errors. The chip's calibration-status byte (0–3 per sensor) is exposed as a quality gate.

**Why this split:** continuous auto-calibration in a changing magnetic environment is genuinely hard to reimplement and is exactly what the Bosch chip does well. The orientation filter itself — where the sensor choices matter and where tuning is needed — stays in firmware where it is visible and tunable.

### 3.3 Configuration

**BNO085 (recommended):**
- SPI at 3 MHz (preferred) or I²C at 400 kHz
- Subscribe via SH-2 to `magnetic_field_calibrated` at 100 Hz
- Optionally subscribe to `gyroscope_calibrated` as a cross-check
- Data-ready interrupt triggers DMA reads

**BNO055 (alternative):**
- I²C at 400 kHz
- Operation mode `NDOF` (so the internal fusion runs and maintains calibration)
- Read the calibrated mag register; ignore the quaternion
- Watch for SCL clock-stretching quirks in the I²C master

### 3.4 Fast 6-DOF IMU Configuration

- SPI at ≥ 3 MHz
- Gyro output 1 kHz, accelerometer 200 Hz
- Internal FIFO enabled; drain per control tick
- Data-ready interrupt for ISR-driven DMA

### 3.5 Placement

The Bosch chip is PCB-soldered with the main hub; the fast IMU can be co-located. Magnetometer placement cannot be relocated without moving to a standalone magnetometer (see §9.1). Mitigation: calibration-status gating in firmware; fallback to gyro-only dead reckoning when mag is untrustworthy.

## 4. Orientation Fusion Layer (New)

### 4.1 Filter

**Mahony complementary filter** at 200 Hz (locked to the control loop). Preferred over Madgwick for its explicit gyro-bias integrator `Ki` and cleaner tuning surface.

Inputs per update:
- Gyro rate (rad/s) — fast IMU, every tick
- Accelerometer (m/s²) — fast IMU, every tick (provides roll/pitch reference via gravity)
- Calibrated magnetic field vector — Bosch chip, applied asynchronously when a new sample arrives (100 Hz)
- Calibration-status byte — gates mag correction

Output: unit quaternion `q` expressing body orientation in a gravity-aligned, magnetic-north-referenced frame. Yaw extracted as `ψ = atan2(2(q.w·q.z + q.x·q.y), 1 − 2(q.y² + q.z²))`.

### 4.2 Disturbance Handling

Mag correction is gated on:
- Chip calibration-status byte ≥ 2
- `|M| − |M_reference|` within ±20% of the locally-learned field magnitude (rejects near-field disturbances)

When either gate fails, the filter falls back to gyro integration with accelerometer-only correction. Heading becomes relative (drifts) but tilt stays correct.

### 4.3 Fit to Existing Abstraction

The `pbio_imu.h` interface is the stable boundary. The filter sits inside the backend; consumers call `pbio_imu_get_heading_scaled` unchanged. New consumer: `pbio_imu_get_magnetic_field` for diagnostics and research use.

Persistent settings extended with mag hard-iron / soft-iron storage (optional — mostly tracked by the chip itself).

### 4.4 Cost

- Filter compute: ~100 μs per update on Cortex-M4 with FPU
- Bus time: ~500 μs total for all sensor reads at 400 kHz / 3 MHz SPI, overlapped with compute via DMA
- Total per-tick overhead: ~200 μs of CPU, comfortably within the 5 ms budget

## 5. World-Pose Layer (New)

### 5.1 Strategy

Encoders contribute to **pose**, not to orientation. The orientation filter owns `ψ`; the pose module integrates encoder-derived forward velocity through that heading:

```
ẋ = ḋ · cos ψ
ẏ = ḋ · sin ψ

x(t) = x₀ + ∫ ḋ · cos ψ  dt
y(t) = y₀ + ∫ ḋ · sin ψ  dt
```

Where `ḋ` is `(v_L + v_R) / 2` from the drivebase state.

### 5.2 API

New module `pbio_pose.c`:

```c
void pbio_pose_update(void);
void pbio_pose_get(int32_t *x_mm, int32_t *y_mm, float *theta_rad);
void pbio_pose_reset(int32_t x_mm, int32_t y_mm, float theta_rad);
```

Called once per control tick after the drivebase update. No control-loop involvement — pose is observable state for navigation, not feedback.

### 5.3 Expected Accuracy

With wheel encoders + Bosch-cal-assisted heading:
- Heading drift: bounded (magnetometer correction) when mag is trustworthy; ~1°/min gyro drift during mag-disabled windows
- Position drift: dominated by residual heading error + wheel slip; typically a few percent of distance traveled on flat ground with clean mag environment

Sufficient for waypoint navigation, return-to-base, and programmatic path execution over several meters. Not sufficient for indefinite autonomous operation without an external reference.

## 6. Path Following

### 6.1 Primitive Layer (Kept)

Existing drivebase primitives: `drive_straight`, `drive_turn`, `drive_arc_angle`, `drive_arc_distance`, `drive_forever`. Users compose paths by chaining these. This remains the canonical interface.

### 6.2 Spline Path Support (Optional, Tiered)

A continuous reference trajectory (Bézier, Catmull-Rom, clothoid) cannot use the current trajectory module directly — trapezoidal profiles assume a single closed-form endpoint. Three implementation tiers:

**Tier A — Piecewise arc approximation (zero firmware changes):**
Sample the spline at N points, compute (arc_length, turn_angle) for each segment, chain `drive_arc_distance` calls with `PBIO_CONTROL_ON_COMPLETION_CONTINUE`. Segment boundaries introduce curvature discontinuities; acceptable for moderate-curvature splines.

**Tier B — Streaming reference generator (moderate firmware work):**
New command type in `pbio_trajectory_*` that installs a callback `(t) → (pos, speed, accel)`. `pbio_trajectory_get_reference` calls it instead of evaluating trapezoidal vertices. Preserves continuous curvature and variable speed along the path. Still open-loop in the plane — closes on `(distance, heading)`, not on pose error.

**Tier C — Pure-pursuit / Stanley path follower (significant new code):**
Use the world-pose from §5 to compute cross-track error against the spline. Pure pursuit: pick a look-ahead point, command `(v, ω)` to steer toward it. Output into `drive_forever`, re-commanded each tick. Closes the loop on pose, which is the architecturally correct answer.

**Recommendation:** start with Tier A. Move to Tier C only if a localization source richer than wheel+IMU odometry becomes available.

### 6.3 Reactive Behaviors (Line/Wall Following)

Reactive behaviors are **not** in firmware. They are user Python code that reads sensors and continuously rewrites `drive(speed, turn_rate)`. The outer loop runs at Python's scheduling rate (50–200 Hz), the inner kinematic loop runs at 200 Hz firmware.

Cascaded structure:

```
sensor → Python P/PID → drive(speed, turn_rate)
                              ↓
                        firmware drivebase control
                              ↓
                              motors
```

No change proposed — the abstraction split is clean.

## 7. Control Loop Timing

### 7.1 Rates

- **Outer control loop** (position/velocity PID + observer + trajectory + pose): **200 Hz** (5 ms) — unchanged.
- **Orientation filter** (Mahony): **200 Hz** synchronized with the control loop.
- **Magnetometer update**: 100 Hz asynchronous (applied to filter when new sample arrives).
- **Fast IMU gyro**: 1 kHz native, consumed at 200 Hz (one sample per tick, or FIFO-batched).

### 7.2 Why Not Faster

- Mechanical time constant of geared LEGO motor: 50–200 ms. Mechanical bandwidth ~ 1–3 Hz. Achievable position-loop bandwidth at 200 Hz ≈ 20 Hz — 10× mechanical dynamics, comfortable.
- Faster control rates do not buy tracking performance the mechanical plant can use.
- Budget for other firmware (BLE, USB, VM, UI) is the practical cap.

### 7.3 Why Not Slower

- Electrical time constant (observer-relevant, for current state): 0.1–0.5 ms. Already undersampled at 200 Hz; slowing would worsen observer transient accuracy.
- Practical floor for this motor class: ~100 Hz. Below that, tracking stiffness and observer numerics degrade visibly.

### 7.4 No Inner Current Loop

Motor current is **estimated** by the observer but not measured. The LEGO motor-driver hardware exposes only PWM duty cycle, no current sense. Adding a hardware current sensor and inner torque loop would require 10–20 kHz sampling — 50–100× faster than the outer loop — and would primarily help with mechanical shocks and explicit force control, not steady-state tracking. Not proposed for LEGO-scale robots.

## 8. Implementation Roadmap

**Phase 1 — Sensor layer**
1. Port/write fast 6-DOF IMU driver (BMI270 or LSM6DSO); fill `pbio_imu_get_angular_velocity` / `pbio_imu_get_acceleration`.
2. Write Bosch chip driver in calibrated-mag-only consumption pattern; add `pbio_imu_get_magnetic_field` to the IMU abstraction.
3. Surface calibration-status byte; define gating thresholds.

**Phase 2 — Orientation fusion**
1. Implement Mahony filter behind `pbio_imu_get_heading_scaled`.
2. Tune `Kp`/`Ki` gains on bench; verify against a ground-truth reference (a turntable or a phone IMU).
3. Implement mag-gating logic using calibration status and field-magnitude plausibility.

**Phase 3 — Pose layer**
1. New `pbio_pose.c` integrating drivebase `(ḋ)` + filtered `ψ` into `(x, y, θ)`.
2. Expose `pbio_pose_get/reset` to user-space API.

**Phase 4 — Path following (optional)**
1. Tier A first — user-space spline sampler calling `drive_arc_distance` in sequence.
2. Evaluate whether pose-closed path following is warranted. If yes, Tier C with pure-pursuit or Stanley.

**Phase 5 — Navigation layer (optional)**
1. `go_to(x, y)` primitive built on Phase 3 pose + Tier A / C.
2. Waypoint sequence execution with look-ahead.

## 9. Open Questions and Trade-offs

### 9.1 Magnetometer Placement

The Bosch chip is stuck on the main hub PCB, close to motor drivers. If mag disturbances in the target environment dominate heading quality, the alternative is a **standalone Bosch BMM150 / BMM350** (or STMicro LIS3MDL) on a short flex cable 10–20 cm from the motor driver. This improves input-signal quality dramatically but transfers the calibration responsibility to firmware. Decision deferred to measured mag behavior in the real environment.

### 9.2 Filter Choice

Mahony is the recommended starting point. An EKF (state `[q, ω_bias]`) offers better disturbance rejection at ~10× the compute cost and significantly more tuning surface. Revisit if Mahony proves inadequate under realistic motor noise.

### 9.3 Encoders as Fusion Input

Standard Mahony/Madgwick does not consume encoders. An advanced extension treats wheel-rate-difference-derived yaw rate `ψ̇_wheels = (v_R − v_L) / axle_track` as a second gyro input for cross-checking and slip detection. Not in baseline design; a candidate enhancement if wheel slip becomes a measurable issue.

### 9.4 Inner Torque Loop

Rejected for LEGO-scale operation. Revisit only for impact-heavy or explicit-force-control applications with hardware current sensing.

### 9.5 Path-Follower Tier

Tier A (piecewise arcs) is the default. Tier C (pose-closed pursuit) is justified only if external localization upgrades the pose estimate beyond wheel+IMU dead-reckoning.

## 10. Summary

A minimal-delta upgrade to the existing `pbio` stack:

- **Keep** the 200 Hz PID + Luenberger observer + feedforward + trapezoidal trajectory core. It is well-suited to the plant.
- **Add** a fast 6-DOF IMU for high-rate body dynamics and a Bosch chip used as a calibrated magnetometer peripheral.
- **Add** a firmware Mahony filter for drift-bounded, magnetometer-corrected orientation.
- **Add** a pose integrator that combines encoder distance with filtered heading for world `(x, y, θ)`.
- **Preserve** the existing configuration-space primitive API; extend it later if spline path following or waypoint navigation is required.

The architecture is cleanly layered: sensor hardware → IMU abstraction (`pbio_imu.h`) → orientation filter → pose module → existing motion controllers → motors. No rate increase is needed; the Bosch chip adds drift-bounded heading and the pose layer adds world coordinates without perturbing the well-tuned inner loops.
