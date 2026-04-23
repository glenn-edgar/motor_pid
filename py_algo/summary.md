# Conversation Summary

Analysis of the `pbio` motion-control source tree (`/home/gedgar/py_algo/`) and a progressive discussion of architectural extensions for sensing, path following, and world-coordinate operation on a Pybricks-class wheeled robot.

---

## 1. Control Strategy of the Existing Stack

**Identified architecture:** model-based PID servo with a Luenberger state observer, trapezoidal trajectory generation, and inverse-model feedforward.

Pipeline:
```
reference trajectory → observer → PID feedback → inverse motor model → motor
```

Key findings:
- **Trajectory (`src/trajectory.c`)**: three-phase trapezoidal profiles (accel/cruise/decel), precomputed at command time.
- **Observer (`src/observer.c`)**: discrete-time state-space model with state `[θ̂, ω̂, î]` (angle, speed, current). Only encoder angle is measured; current is instrumental.
- **PID (`src/control.c:280–287`)**: gain-scheduled `Kp` (boosts near endpoint for stiction), `Kd` on observer speed, `Ki` on integrated position error. Anti-windup by trajectory-pause when proportional torque saturates.
- **Feedforward (`observer.c:250–258`)**: friction + back-EMF + inertial, converted to voltage via a static motor constant.
- **Drivebase (`src/drivebase.c`)**: dual independent controllers on decoupled `(distance = (θ_L+θ_R)/2, heading = (θ_L−θ_R))` coordinates. Trajectories time-synchronized via `pbio_trajectory_stretch`.

## 2. Path Following

**Conclusion:** the robot does not follow continuous paths — it executes a sequence of constant-curvature primitives (straight, turn, arc) chained by user code. `pbio_drivebase_drive_relative` is the core function; all primitives funnel through it.

The "path follower" is a segment sequencer on top of a time-synchronized dual-axis trapezoidal trajectory generator. There is no lookahead, no pure-pursuit, no spline follower.

## 3. Spline Path Feasibility

**Conclusion:** not natively supported. Three obstacles:
1. `pbio_trajectory_t` is a closed-form trapezoid, not a streamable reference.
2. No 2D pose state anywhere — only scalar `(distance, heading)`.
3. `pbio_trajectory_stretch` enforces constant `distance/heading` ratio, incompatible with varying curvature.

Three implementation tiers identified:
- **Tier A**: piecewise-arc approximation using existing primitives (zero firmware changes).
- **Tier B**: streaming reference generator (replace trapezoidal trajectory with spline evaluator).
- **Tier C**: pure-pursuit / Stanley controller with 2D pose closed-loop (full pose state required).

## 4. Bosch IMU Compatibility

**Conclusion:** compatible and well-abstracted via `include/pbio/imu.h`. The stack consumes raw gyro/accel and scalar heading only — no quaternion or fusion consumer exists in the control path.

Three integration modes:
- **Raw 6-DOF use**: valid but wastes the fusion capability.
- **Fused yaw as heading source**: highest-value minimum-change option. Yaw becomes drift-bounded, magnetometer-anchored, recoverable across reboots.
- **Full orientation consumption**: requires new consumers for tilt-corrected operation; not needed for baseline features.

Terminology note: BNO055 uses Bosch BSX fusion; BNO085 uses CEVA SH-2. Magnetometer is the only external yaw reference — unreliable near motors and steel, mitigated by chip's calibration-status byte.

## 5. Line and Wall Following

**Conclusion:** not implemented in firmware. There is no sensor-in-the-loop abstraction. The stack's feedback chain ends at encoders + IMU.

Both behaviors are done in **user Python** using a P or PID controller that polls a sensor and repeatedly calls `drive(speed, turn_rate)`. The firmware primitive is `drive_forever`, which is rewritten on each outer-loop tick.

Cascaded structure:
```
sensor → Python P-loop (outer) → drive(speed, turn_rate)
                                       ↓
                         firmware drivebase (inner, 200 Hz)
                                       ↓
                                      motors
```

Outer-loop rate capped by Python VM (~50–200 Hz). Architecturally clean but limits high-speed reactive control.

## 6. World Coordinates

**Conclusion:** not used. All state is in configuration space (joint-angle space).

- Servo state: motor shaft angle (1D).
- Drivebase state: `(distance, heading)` — both scalar accumulators, not Cartesian.
- IMU 3D heading: gravity-aligned horizontal yaw relative to an arbitrary zero (power-on or last reset).

`pbio/geometry.h` exists (xyz, 3×3, quaternions) but is used only inside the IMU module for mapping sensor-frame ↔ body-frame orientation. No anchor to any external reference.

Design intent: no external localization sensor assumed; any (x, y) estimate would drift unboundedly; converting world goals to configuration-space goals is a planning concern outside `pbio`.

## 7. Bosch IMU and World Coordinates

**Conclusion:** the chip significantly helps orientation; does not directly help position — but on a wheeled robot that's most of the battle.

- **Orientation**: major win. Fused yaw is drift-bounded, world-anchored, recoverable. Existing `pbio_imu_get_heading_scaled` consumes it unchanged.
- **Position**: no direct help. Double-integrating accelerometer output is infeasible on any MEMS IMU — bias causes quadratic unbounded drift in seconds.
- **Practical impact**: wheel encoders provide arc length; combining with IMU heading via `(ẋ, ẏ) = ḋ · (cos ψ, sin ψ)` is the standard pattern. Kills the dominant heading-error source on a wheeled robot.

Limitations: mag-reliability near motors/steel; mag calibration required; still need a `(x, y)` integrator module — the chip fixes the sensor problem, not the software-architecture problem.

## 8. Motor Current

**Conclusion:** estimated but never measured and never fed back.

- `pbdrv/motor_driver.h` exposes only `coast` and `set_duty_cycle` — pure voltage/PWM H-bridge, no current sense.
- Current `î` is part of the observer state to propagate speed/angle correctly through DC-motor dynamics, but it is not a control target.
- PID sees position error only; no current term. Feedforward is kinematic.
- Stall detection is **model-residual based** (observer feedback voltage vs. command voltage), substituting for a current sensor.

## 9. Control Loop Rate

**Current:** 200 Hz (`PBIO_CONFIG_CONTROL_LOOP_TIME_MS = 5` in `include/pbio/config.h:16`). Single-rate — trajectory, observer, PID, mix/unmix all run at this rate.

**Inner torque/current loop (hypothetical):**
- Required rate: 10–20 kHz (100–50 μs period), 50–100× faster than outer loop.
- Driven by electrical time constant τₑ = L/R ≈ 0.1–0.5 ms.
- PWM frequency must exceed current loop rate by 2–4× (20–40 kHz).
- Must live in a timer ISR, not the cooperative scheduler.

**Conclusion:** inner torque loop is not worth the effort on LEGO-class geared brushed DC motors — mechanical time constants dominate, voltage-command open-loop on torque is adequate.

## 10. Could the Bosch IMU Let the Loop Run Slower?

**Conclusion:** no, not meaningfully. The 200 Hz rate is set by the motor mechanical regulation problem, not by IMU processing.

- IMU work is ~10–20% of per-tick CPU. Moving it onto the chip saves that fraction but doesn't change the required tick rate.
- Practical floor for this motor class: ~100 Hz, driven by mechanical bandwidth target and observer electrical-state numerics. Independent of IMU choice.
- Multi-rate architecture (heading loop slower than motor loop) is possible but adds complexity for modest gain.

## 11. I²C Speed Adequacy

**Conclusion:** bus is not a constraint at 200 Hz — comfortable margin of ~10×.

| Mode | Wire rate | 9-DOF read | % of 5 ms tick |
|---|---|---|---|
| Standard | 100 kHz | ~2.0 ms | 40% — tight |
| Fast | 400 kHz | ~500 μs | 10% — comfortable |
| Fast Mode Plus | 1 MHz | ~200 μs | 4% — trivial |

Real constraint is chip output rate (e.g., BNO055 fusion at 100 Hz), not bus bandwidth. Standard architecture uses DMA + data-ready interrupts + internal FIFO — making bus time invisible to the control loop.

## 12. Two IMUs (Fast + Bosch)

**Conclusion:** the partition "one for motor control, one for heading" doesn't map — motor control doesn't consume IMU data.

A more defensible partition: **fast 6-DOF for body dynamics + Bosch fused for orientation**. Benefits: high-rate heading D-term, low-latency impact detection, tilt slope detection, magnetometer-disturbance cross-check.

But a single BNO085 on SPI provides both fused and raw outputs at 400 Hz — covers 95% of the use cases at lower complexity. **Recommendation**: single BNO085 by default; dual-chip only if magnetic environment is genuinely hostile or impact detection below 2.5 ms latency is required.

## 13. Standalone Magnetometer + Fast IMU

**Conclusion:** the real win is **magnetometer placement**, not fusion quality.

The Bosch chip's magnetometer is soldered next to the motor driver — it sees all the brushed-DC noise. A discrete mag on a flex cable 15 cm away can be an order of magnitude quieter.

Costs: write Mahony/Madgwick filter (~100–200 LOC), implement hard/soft-iron calibration (weeks of UX work), handle asynchronous sample rates, time alignment.

Worth it when: mag-hostile environment, PCB flexibility allows remote mag, team can own fusion.
Not worth it when: hobby/education scale, fixed PCB layout, short time-to-ship.

## 14. Bosch Standalone Magnetometers

**Options:**
- **BMM150** (Bosch) — the mag that's inside every Bosch 9-DOF chip, available standalone (~$1.50). 13-bit, 30 Hz normal.
- **BMM350** (Bosch, newer) — 14-bit, 200 Hz, better tempco (~$2).
- Alternatives: LIS3MDL, MMC5983MA, AK09916.

Buying a full BNO085 only for its mag is a bad cost decision (~10× overpayment). Three sensible patterns:
- **BMM150/BMM350** as discrete mag in a full discrete-sensor build.
- **BNO085** alone, consuming fused outputs (no separate mag).
- **BNO085** subscribing to both fused quaternion AND raw mag reports — production reliability + observability.

## 15. Using the Bosch Chip as a Calibrated-Magnetometer Peripheral

**Final architecture decided on:** Bosch chip runs its on-die fusion internally for the purpose of continuous auto-calibration, but only the **calibrated magnetic field vector** is consumed. The orientation filter itself stays in firmware.

Chip provides:
- Hard-iron offset `(x, y, z)` subtracted from mag readings — eliminates rotation-dependent heading errors from fixed ferromagnetic bias.
- Soft-iron correction matrix — reshapes ellipsoid back to sphere.
- Continuous gyro bias estimation (available via `gyroscope_calibrated`).
- Calibration-status byte as a quality gate.

Firmware owns:
- Mahony complementary filter at 200 Hz consuming (gyro + accel + calibrated mag).
- Mag-correction gating on calibration status and field-magnitude plausibility.
- Fall-back to gyro-only dead reckoning when mag is untrustworthy.
- Pose integrator combining encoder `ḋ` with filtered `ψ` → `(x, y)`.

Terminology correction: the rotation-dependent heading error that the mag offsets fix is **hard-iron calibration**, not Coriolis. MEMS gyros themselves *use* Coriolis force to measure rate. Gyro bias (also maintained by the chip) is a separate effect that reduces turning-induced heading drift.

**Filter architecture note:** Mahony/Madgwick fuses gyro + accel + mag for *orientation*. Encoders do not enter the orientation filter directly — they combine with the output yaw to produce *pose* in a separate cascaded step. An advanced extension could use wheel-rate-difference yaw as a second gyro input for slip detection, but that's beyond the baseline.

---

## Overall Takeaway

The `pbio` motion-control core (200 Hz PID + observer + feedforward + trapezoidal trajectories) is well-designed for the LEGO-class plant it regulates and should be kept as-is.

The productive extensions are **sensor-side**:
1. Fast 6-DOF IMU for body dynamics and filter driver.
2. Bosch chip as calibrated-magnetometer peripheral (not as fusion source).
3. Firmware Mahony filter for drift-bounded orientation.
4. Pose integrator combining encoder distance with filtered heading.
5. Optional spline/waypoint navigation layer above the existing primitives.

The **control-side** is complete — no inner current loop, no higher tick rate, no fundamental restructuring is warranted. The stack's configuration-space scope is intentional, and world-coordinate support belongs in a layer above it.
