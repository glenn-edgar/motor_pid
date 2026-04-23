# Automotive Window & Seat Actuator Control — Design Document

**Status:** Draft v0.1
**Scope:** Two-tier product family (basic / premium) for Pittman GM8000-class brushed DC motor actuators in automotive window-lift and power-seat applications, using Seeed Studio XIAO SAMD21 and XIAO RA4M1 as the real-time control platforms. Non-real-time supervision handled by ChainTree / s-expression layer on a separate processor.

---

## 1. Purpose & Scope

This document specifies the real-time motor control architecture for a Pittman GM8000-class brushed DC motor driving a worm-gearboxed actuator (window lift or power seat). Two variants are defined:

- **Basic tier (XIAO SAMD21):** trapezoidal motion profile, fixed-point arithmetic, single-path safety with hardware overcurrent.
- **Premium tier (XIAO RA4M1):** jerk-limited S-curve profile, single-precision float, disturbance observer, voting anti-pinch, richer diagnostics.

Both tiers ship the same firmware core and the same command/telemetry API to a higher-level ChainTree supervisor. Platform differences are confined to the HAL, math layer, and a small number of feature flags.

### 1.1 Non-goals

- ISO 26262 process certification. The architecture is ASIL-B-shaped (independent diagnostic channels, documented safe state, hardware-latched cutoff), but no formal safety case is produced.
- Sensorless operation. A quadrature encoder is required.
- Brushless motor support.
- Direct CAN bus participation (handled by the ChainTree layer if present).

---

## 2. Mission Profile

The actuator performs bounded, commanded moves under driver control with hard safety requirements around human contact.

| parameter | window lift | power seat (slide / recline / height) |
|---|---|---|
| travel | ~450 mm linear | 200–250 mm slide, ~60° recline |
| cruise velocity | ~100 mm/s | ~30 mm/s slide, ~10°/s recline |
| typical run time | 3–5 s | 4–8 s |
| duty cycle | intermittent, ~10 cycles/day | intermittent, ~5 cycles/day |
| end stops | seal stall (top) + mechanical (bottom) | mechanical both ends |
| pinch hazard | finger/arm in aperture | finger under rail, hand in recline mechanism |
| applicable standard | FMVSS 118 S5, UNECE R21 | UNECE R21 §5.8 (power devices, anti-pinch) |
| pinch reversal requirement | ≤100 N force, reverse within 100 ms | same posture |

Mechanically, both applications are the same control problem: position command of a non-backdrivable worm-driven linear or angular axis with force limiting. Parameters differ; architecture does not.

---

## 3. System Architecture

Two processors, two tiers of concern:

```
┌──────────────────────────────────────────────────────────────┐
│ NON-REAL-TIME TIER — ChainTree / s-expression                │
│   ∙ supervisor behaviors (auto mode, memory positions)       │
│   ∙ baseline-current learning (Welford per position bucket)  │
│   ∙ calibration orchestration, fault policy, user prefs      │
│   ∙ OTA config push, logging, inter-ECU messaging            │
│   ∙ tolerates tens-of-ms jitter                              │
└────────────────┬─────────────────────────────────────────────┘
                 │ command / telemetry shim
                 │ framed UART or SPI, CRC-protected,
                 │ bounded rate, heartbeat required
                 │
┌────────────────▼─────────────────────────────────────────────┐
│ REAL-TIME TIER — XIAO (SAMD21 or RA4M1)                      │
│                                                              │
│  100 Hz supervisor task                                      │
│    state machine, thermal model, baseline lookup, shim IO    │
│                              │ pos_ref, v_limit, i_limit     │
│                              ▼                               │
│  1 kHz motion task                                           │
│    profile execution, velocity PI + FF, DOB (RA4M1),         │
│    anti-pinch detector, safe-state enforcement               │
│                              │ i_cmd                         │
│                              ▼                               │
│  10 kHz (SAMD21) or 20 kHz (RA4M1) current-loop ISR          │
│    ADC read, encoder read, velocity estimate (M/T),          │
│    current PI, back-EMF FF, PWM duty write                   │
│                                                              │
│  HARDWARE SAFETY (independent of firmware)                   │
│    analog comparator → PWM fault latch                       │
│    watchdog (must be fed by 1 kHz task)                      │
│    gate-driver fault flag → safe-state ISR                   │
└──────────────────────────────────────────────────────────────┘
```

### 3.1 Design principles

1. **The real-time tier is self-sufficient for safety.** Loss of the ChainTree link, a missing heartbeat, or corrupted commands must not produce unsafe behavior. On heartbeat timeout the actuator transitions to SAFE_IDLE (PWM disabled, H-bridge in coast or brake per policy).
2. **Safety is layered and independent.** Hardware overcurrent cuts PWM without firmware involvement. Software overcurrent, stall detection, anti-pinch detection, and thermal model are independent checks; any one can command safe state.
3. **The command API is declarative.** The high-level tier requests *outcomes* (`MOVE_TO(pos)`), not actuation details. The real-time tier owns profile generation, current limits, and termination conditions.
4. **Parameterization is externalized.** Motor constants, travel limits, gains, and safety thresholds live in non-volatile config, pushed from ChainTree at init. The firmware core is portable across variants.

---

## 4. Target Platforms

### 4.1 XIAO SAMD21 — basic tier

- **MCU:** Microchip ATSAMD21G18, Cortex-M0+ @ 48 MHz, no FPU, no DSP extensions.
- **Memory:** 32 KB SRAM, 256 KB flash. Budget allocation: ~12 KB SRAM, ~80 KB flash for this module.
- **Relevant peripherals:**
  - TCC0 for center-aligned PWM with dead-time insertion, fault input
  - TC3/TC4 paired for hardware quadrature decode (32-bit counter)
  - ADC0 with EVSYS trigger from TCC, 12-bit, 350 ksps effective
  - AC (analog comparator) — drives TCC fault input for hardware overcurrent
  - WDT in window mode
  - EVSYS for deterministic ADC triggering
- **Control-loop rates:** 10 kHz current, 1 kHz motion, 100 Hz supervisor.
- **Arithmetic:** Q16.16 fixed-point throughout the ISR and motion task. No float in time-critical paths.

### 4.2 XIAO RA4M1 — premium tier

- **MCU:** Renesas R7FA4M1AB, Cortex-M4 @ 48 MHz, single-precision FPU, DSP extensions.
- **Memory:** 32 KB SRAM, 256 KB flash. Comparable budget to SAMD21.
- **Relevant peripherals:**
  - GPT3 for center-aligned PWM, complementary outputs, HW dead-time
  - GPT channels paired for quadrature decode
  - ADC14, 14-bit, 0.6 μs conversion, triggered by GPT via ELC
  - ACMPHS (high-speed analog comparator) driving POEG (Port Output Enable for GPT) for hardware PWM latch
  - IWDT + WDT, used together for window/time-bounded feeding
  - ELC (Event Link Controller) eliminates software trigger jitter on ADC conversion
- **Control-loop rates:** 20 kHz current, 1 kHz motion, 100 Hz supervisor.
- **Arithmetic:** single-precision float throughout. Hard-float ABI.

### 4.3 Shared choices

Both platforms use hardware quadrature decode (zero CPU cost for position), PWM-triggered ADC sampling (zero software jitter), hardware-latched overcurrent cutoff, and center-aligned complementary PWM with dead-time insertion. The firmware core is identical; the HAL differs.

---

## 5. Motor Model

Pittman GM8000-class PMDC motor with quadrature encoder on the rotor shaft, driving a worm gearbox (~60:1, η ≈ 0.5, non-backdrivable) connected to a window regulator or seat screw-drive.

### 5.1 Canonical third-order model

```
electrical:   L · di/dt   = V − R·i − Ke·ω_m
mechanical:   J · dω_m/dt = Kt·i − b·ω_m − τ_L/N − τ_fric(ω_m)
kinematic:    dθ_out/dt   = ω_m / N
```

where `ω_m` is motor-shaft velocity, `N` is gear ratio, `θ_out` is output position, `τ_L` is load torque at the output, and `τ_fric(ω_m)` is a Coulomb + viscous model.

### 5.2 Parameters to identify

| symbol | meaning | identification method |
|---|---|---|
| R | terminal resistance (Ω) | LCR meter cold; datasheet warm |
| L | terminal inductance (H) | LCR meter @ 1 kHz |
| Kt | torque constant (N·m/A) | stall test: Kt = τ_stall / i_stall |
| Ke | back-EMF constant (V·s/rad) | no-load: Ke = V / ω_no-load; Kt ≡ Ke in SI |
| J | rotor inertia (kg·m²) | coast-down test, fit exponential |
| b | viscous damping (N·m·s/rad) | coast-down steady-state |
| Fc | Coulomb friction (N·m) | minimum current to initiate rotation |
| N | gear ratio | datasheet; verify via encoder counts per output rev |
| η | gearbox efficiency | load test; worm ≈ 0.4–0.6 |

Parameters are stored in non-volatile config and pushed from the ChainTree layer. Tier firmware treats them as read-only after init except for slow thermal-drift corrections.

### 5.3 Non-backdrivability as a feature

The worm gearbox cannot be driven in reverse by output-side load. Consequences exploited:

- No holding current required at stopped positions. PWM can be fully disabled after a move completes, saving thermal budget.
- Seal stall (window at top, fully closed) is a *legitimate* end-of-travel reference, not a fault. The motor stalls, current rises, velocity goes to zero — the control logic recognizes this signature and treats it as a successful move completion.
- Any backward motion without commanded reverse is a mechanism failure and latches a fault.

---

## 6. Current Measurement

Current sensing is the single most critical measurement in this design. It is simultaneously the control feedback for the innermost loop, the primary signal for anti-pinch detection, the input to the thermal model, and the substrate for the hardware overcurrent safety layer. Getting it right is not optional.

### 6.1 Topology choice: in-line shunt

Three shunt placements are possible:

- **Low-side shunt** (between H-bridge low legs and ground): cheap, ground-referenced, no level-shifting amplifier needed. But: current is only measurable during PWM-on for one polarity and requires synchronous switching between polarities for bidirectional sensing. Freewheel current (when both low-side MOSFETs are on for regenerative braking or coasting) is invisible. Unacceptable for anti-pinch, which must work during dynamic braking.

- **High-side shunt** (between V_bus and bridge): measures total bus current. Requires common-mode range to V_bus (13.5 V automotive, up to ~18 V with alternator transients). Tells you bridge input current but not motor direction without extra logic. Usable but not ideal.

- **In-line shunt** (between a bridge half and the motor): measures true motor current, bidirectional, present in every PWM state including freewheeling. The signal lives on a node that swings between 0 V and V_bus at the PWM rate, so the differential amplifier must have high common-mode rejection at the switching edges. This is exactly the problem the INA240 and similar parts were designed to solve.

**Choice: in-line shunt + INA240-class amplifier.** This is also the automotive standard for this class of actuator. Two shunts (one per bridge output) allow per-phase current measurement; for a simple H-bridge driving a single brushed motor, one shunt on the motor return is sufficient.

### 6.2 Shunt sizing

Design targets:
- Peak current during normal operation: ~5 A (stall at seal)
- Overcurrent trip threshold: 15 A
- Shunt power dissipation at continuous cruise (1 A typical): should be <100 mW
- Signal voltage at trip threshold: within amplifier input range

With a 10 mΩ shunt:
- Signal at 5 A: 50 mV
- Signal at 15 A: 150 mV
- Power at 1 A continuous: 10 mW
- Power at 5 A peak: 250 mW (transient, acceptable)
- Amplifier with gain of 20× gives 3.0 V at 15 A, well within ADC range

10 mΩ, 1% metal-film, 1 W (derated) is the target part. Kelvin connections to the amplifier are mandatory — any IR drop in the sense traces from current in the power loop corrupts the reading.

### 6.3 Amplifier

**INA240A2** (×50) or **INA240A1** (×20) depending on shunt value. Key specs:
- Common-mode range: −4 V to +80 V
- PWM-rejection-optimized input stage (enhanced CMRR at switching edges): this is the feature that makes in-line sensing work reliably
- Bandwidth: 400 kHz — well above our 20 kHz PWM
- Supply: 2.7–5.5 V, single-supply
- Output is ground-referenced with a 2.5 V (or configurable) offset for bidirectional signaling

The 2.5 V zero-current offset is what enables bidirectional measurement: positive motor current moves the output above 2.5 V, negative below. The ADC sees a unipolar signal that encodes both polarities.

Alternative amplifiers: INA241, MAX40056 (automotive-qualified), AD8418. For automotive qualification, the AEC-Q100-graded variant of whichever part is chosen.

### 6.4 Filtering

A small RC filter between shunt and amplifier input rejects ringing from MOSFET switching without slowing the signal meaningfully:
- R_filt = 10 Ω (each line, for differential symmetry)
- C_diff = 1 nF (across diff inputs)
- C_cm = 100 pF (each line to ground)

This gives a differential-mode cutoff of ~8 MHz, which removes only the ringing and leaves the PWM fundamental and all control-relevant bandwidth intact. No filtering at the amplifier output — we want the full signal fidelity for the ADC.

### 6.5 ADC sampling strategy

The critical design choice: **the ADC sample must be synchronized to the PWM cycle, not asynchronous.** Random sampling of a PWM-modulated signal gives noisy nonsense. With center-aligned PWM, the on-time is symmetric around the midpoint of the PWM period, and sampling at the exact midpoint of the on-time gives the best estimate of average motor current with minimum ripple error.

On **SAMD21**:
- TCC0 configured for center-aligned PWM at 20 kHz (period = 50 μs)
- TCC0 generates an EVSYS event at the counter midpoint (compare match on a dedicated channel)
- EVSYS routes the event directly to ADC0's START_SINGLE input
- ADC converts without CPU involvement; EOC IRQ fires at 10 kHz (every other PWM period, adequate for L/R ≈ 1 ms)

On **RA4M1**:
- GPT3 configured for center-aligned PWM at 20 kHz
- GPT compare-match event linked via ELC to ADC14 trigger
- ADC14 converts in 0.6 μs, EOC IRQ fires at 20 kHz
- ELC routing is fully programmable; no software involved in triggering

The payoff of hardware triggering: **zero jitter** between the PWM edge and the ADC sample point. Software-triggered sampling can easily have 1–5 μs of jitter, which at 20 kHz PWM is several percent of the cycle — enough to inject duty-cycle-correlated noise into the measurement.

### 6.6 Offset calibration

The INA240's output at zero current is nominally 2.5 V but has ±2 mV initial offset and ~10 μV/°C drift. The ADC has its own offset. Combined, uncalibrated zero error can be 10–20 mA equivalent, which corrupts the baseline current table and the thermal integrator.

**Calibration sequence at every power-up:**
1. H-bridge fully disabled (all MOSFETs off, motor electrically floating).
2. Wait 10 ms for any residual current to decay.
3. Take 256 ADC samples at 10 kHz, average.
4. Store as `i_offset_counts` in RAM.
5. Subtract from every subsequent sample before scaling.

Additionally, every transition from STOPPED to MOVING can re-sample offset during the ≤1 ms bridge-enable dead time. This compensates for thermal drift during operation.

### 6.7 Scaling

Signal path, per platform:

**SAMD21 (12-bit ADC, V_ref = 3.3 V):**
- Shunt voltage at 1 A: 10 mV
- Amplifier output change at 1 A: 200 mV (gain 20)
- ADC LSB: 3.3 V / 4096 = 0.806 mV
- Counts per amp: 248
- Resolution: ~4 mA/LSB — far finer than any control need
- Full-scale current: ±(3.3 V − 2.5 V offset) / 20 × 1/10 mΩ = ±4 A useful range, ±8 A if centered at midrange

For a 15 A overcurrent system, the in-line sense is used only for control-band current (up to ~8 A); the hardware comparator uses a separate tap with a lower gain or direct shunt input for the full 0–20 A trip range. This is standard: the ADC sees a scaled, precision signal for control; the comparator sees a raw, fast signal for trip.

**RA4M1 (14-bit ADC, V_ref = 3.3 V):**
- Same signal path
- ADC LSB: 3.3 V / 16384 = 0.201 mV
- Counts per amp: 996
- Resolution: ~1 mA/LSB
- Same full-scale considerations

The extra 2 bits of ADC resolution on the RA4M1 directly improves current-loop quiet operation and DOB fidelity at low currents.

### 6.8 Two independent paths: control vs. safety

A principle worth making explicit: the ADC-based current measurement is the **control-band** path (precision, filtered, synchronized, ≤8 A useful). The analog comparator driving the PWM fault latch is the **safety-band** path (fast, unfiltered, latching, 0–20 A trip). They share the shunt but not the amplifier, not the filtering, not the MCU state.

A firmware bug that corrupts the control-band reading cannot defeat the safety-band trip. This is the cheapest useful diversity in a single-MCU design and is explicitly what "safety if possible" buys you.

### 6.9 Summary

| item | spec |
|---|---|
| topology | in-line shunt, motor-return side |
| shunt | 10 mΩ, 1%, metal film, 1 W, Kelvin-connected |
| amplifier | INA240A1 (×20), PWM-rejection-optimized |
| filter | differential RC, ~8 MHz cutoff |
| ADC trigger | hardware event from PWM timer midpoint |
| offset cal | at every power-up with bridge disabled, re-checked per move |
| control-path full scale | ±8 A, 1–4 mA/LSB resolution |
| safety-path | separate analog comparator, 15 A trip, hardware-latched |

---

## 7. Control Architecture

Cascaded loops with bandwidth separation of ~decade per loop:

1. **Current loop (innermost):** 10 kHz (SAMD21) or 20 kHz (RA4M1). Closes i to i_cmd. Plant is electrical L/R. Closed-loop BW ~1 kHz. Back-EMF feedforward `V_ff = Ke · ω̂` linearizes the plant.

2. **Velocity loop (middle):** 1 kHz. Closes ω to ω_ref from the profiler. Plant is mechanical J/b (seen through the closed current loop, which looks like "torque in, torque out"). Closed-loop BW ~100 Hz.

3. **Profile executor (outer):** runs in the 1 kHz task. Trapezoidal (SAMD21) or S-curve (RA4M1). Produces (θ_ref, ω_ref, α_ref) each tick; the velocity loop tracks ω_ref and the α_ref term feeds forward.

4. **Supervisor (slowest):** 100 Hz state machine. Commands profile start/stop, monitors anti-pinch, stall, thermal, handles shim I/O.

### 7.1 Current PI

Analytic tuning via pole-zero cancellation:

```
Kp_i = L · ω_cl
Ki_i = R · ω_cl
```

where `ω_cl` is desired closed-loop current-loop bandwidth in rad/s. Target `ω_cl = 2π · 1000` rad/s (1 kHz) as a starting point, adjusted down if switching noise couples into the loop.

Back-EMF feedforward: `V_cmd = PI_output + Ke · ω̂`. This makes the current loop see a pure L/R plant; without it, ω-dependent back-EMF acts as a slow disturbance that the PI has to chase.

### 7.2 Velocity estimation (M/T method)

Naive velocity estimation by Δposition/Δtime at fixed Δt fails at low speeds: with a 500 CPR encoder and 1 ms tick, the minimum non-zero velocity is one count per tick = 12.5 rad/s at the motor shaft, quantized in ugly steps.

The M/T method switches estimator based on edge rate:

- **High speed (M method):** count encoder edges during fixed Δt. Accurate when many edges per tick.
- **Low speed (T method):** measure time between consecutive edges. Accurate when few edges, high time resolution (hardware timer capture).

Switchover threshold: ~10 edges per tick. Both platforms have enough timer peripherals to implement this without CPU cost.

### 7.3 Velocity loop

Basic tier: PI on ω error, output is i_cmd (torque command divided by Kt).

Premium tier: PI + feedforward:

```
τ_cmd = J_refl · α_ref + b · ω_ref + Fc · sign(ω_ref) + τ̂_L + PI(ω_ref − ω̂)
i_cmd = τ_cmd / Kt
```

Where `J_refl = J_motor + J_load/N²`, and `τ̂_L` is the disturbance observer output. This is a 2-DOF controller: feedforward handles tracking, PI + DOB handles rejection.

### 7.4 Disturbance observer (premium tier only)

Estimates load torque in real time:

```
τ̂_L_raw = Kt · î − J · (dω̂/dt) − b · ω̂
τ̂_L = LPF(τ̂_L_raw, fc = 20 Hz)
```

The LPF cutoff is chosen below the mechanical resonance of the arm/regulator and above the slowest expected load change (cold grease, seal friction gradient). 20 Hz is a typical starting value.

Feeding τ̂_L forward into i_cmd closes a second-order disturbance rejection loop at ~20 Hz, dramatically improving behavior under changing load (temperature, wear, wet vs dry seal).

### 7.5 Profile execution

**SAMD21 (trapezoidal):** three states — accel, cruise, decel. One-time computation per move:

```
t_accel  = v_cruise / a_max
d_accel  = 0.5 · a_max · t_accel²
d_cruise = |d_target| − 2·d_accel     (clip to zero → triangle profile if negative)
t_cruise = d_cruise / v_cruise
```

The 1 kHz task increments time through each segment and computes (θ_ref, ω_ref) analytically from segment start conditions. No square roots per tick. Fixed-point throughout.

**RA4M1 (S-curve, 7-segment jerk-limited):** jerk-up → constant-accel → jerk-down → cruise → jerk-down (negative) → constant-decel → jerk-up (to zero). Segment durations (t₁…t₄) computed at move start from (Δθ, v_max, a_max, j_max). Per-tick evaluation within each segment:

```
α(t) = α_start + j · t
ω(t) = ω_start + α_start · t + ½ · j · t²
θ(t) = θ_start + ω_start · t + ½ · α_start · t² + ⅙ · j · t³
```

FPU evaluates this in ~5 μs. The α_ref output is the critical coupling into the velocity-loop feedforward.

---

## 8. Safety Architecture

"Safety if possible" translates to: engineer toward ASIL-B-shaped architecture without formal certification.

### 8.1 Independent diagnostic channels

Any of the following can command SAFE_IDLE:

| channel | signal | action | latency |
|---|---|---|---|
| HW overcurrent | comparator on shunt | PWM latch off | <1 μs |
| HW undervoltage | BOR / V_bus comparator | PWM latch off | <1 μs |
| SW overcurrent | ADC reading > limit | current loop saturates; supervisor faults | <1 ms |
| Anti-pinch | Δi AND dω/dt thresholds | reverse then stop | ≤10 ms detect, ≤50 ms reverse |
| Stall timeout | i_above_limit AND ω ≈ 0 for > t_stall | fault latch | t_stall (config) |
| Thermal | ∫i²·dt model > limit | derate, then fault | seconds |
| Watchdog | task timing miss | MCU reset → safe boot | <10 ms |
| Encoder plausibility | direction sign mismatch with command | fault | <10 ms |

### 8.2 Anti-pinch detection

Core algorithm (both tiers):

```
Δi = i_meas − i_baseline(pos, V_bus, T_est)
pinch_detected = (Δi > i_pinch_threshold) AND (dω/dt < ω̇_pinch_threshold)
```

Two-signal AND is essential. Current alone false-triggers on cold grease; velocity derivative alone is noisy. Both together is robust.

Premium tier adds a third vote: DOB residual τ̂_L exceeds baseline_τ_L(pos) + threshold. 2-of-3 voting across (Δi, dω/dt, Δτ̂_L) reduces both false triggers and missed detections.

Baseline `i_baseline(pos)` is a 32-point (SAMD21) or 64-point (RA4M1) piecewise-linear table across the travel range, updated slowly via Welford statistics in the supervisor task during successful non-anomalous runs. Temperature and V_bus compensation are applied at lookup time, not stored per-bucket.

### 8.3 Safe state definition

SAFE_IDLE:
- All PWM outputs disabled (MOSFETs off → motor coasts, or low-side MOSFETs on → dynamic brake per policy)
- Bridge enable line deasserted
- Current command internally zeroed
- State machine blocks all move commands until CLEAR_FAULT received
- Fault cause logged with timestamp and last ~100 ms of telemetry (ring buffer)

Coast vs. brake choice is application-dependent: window lift with non-backdrivable worm → coast is safe (load can't move pane); seat with high static load → brake may be preferred during faults to prevent slow creep.

### 8.4 What we do not have

- Dual MCU lockstep
- Redundant current-sense paths (we have *diverse* paths — control vs. safety — but not redundant)
- ASIL-rated gate driver with integrated safety monitoring
- Certified software process

These are the cost line. If the product requires them later, the architecture is prepared (parameterized, channels already independent) but not the process.

---

## 9. Real-Time Partitioning

### 9.1 SAMD21 timing budget

Core at 48 MHz = 48 cycles/μs. M0+ typical IPC ≈ 0.7–0.9 for representative mixed code.

**Current-loop ISR (10 kHz, period 100 μs, budget 4800 cycles):**

| step | cycles (approx) |
|---|---|
| IRQ entry/exit | 40 |
| ADC read, offset, Q16.16 scale | 60 |
| encoder Δ, M/T velocity update | 160 |
| back-EMF FF computation | 40 |
| current PI, saturation | 80 |
| PWM duty write | 20 |
| **total** | **400** |
| **margin** | **4400 (92%)** |

Generous margin. The constraint is RAM, not cycles.

**Motion task (1 kHz, budget ~40,000 cycles):**

Profile update (trapezoidal, 3 states): ~200 cycles.
Velocity PI: ~100 cycles.
Anti-pinch check: ~150 cycles.
Total ~500, well under budget.

### 9.2 RA4M1 timing budget

Core at 48 MHz with FPU. Single-precision float ops are 1–3 cycles each.

**Current-loop ISR (20 kHz, period 50 μs, budget 2400 cycles):**

| step | cycles (approx) |
|---|---|
| IRQ entry/exit with FP context | 80 |
| ADC read, scale to float | 40 |
| encoder Δ, M/T velocity | 100 |
| back-EMF FF | 20 |
| current PI (float) | 40 |
| PWM duty write | 20 |
| **total** | **300** |
| **margin** | **2100 (87%)** |

**Motion task (1 kHz, budget ~48,000 cycles):**

S-curve evaluation: ~250 cycles. Velocity PI + FF: ~150. DOB (1st-order filter + computation): ~200. Anti-pinch with 2-of-3 voting: ~150. Total ~750, under 2% of budget.

### 9.3 Priority order (both platforms)

1. Current-loop ISR — highest priority, nothing preempts.
2. Motion task — preempted only by current ISR.
3. Shim RX ISR — preempted by above, runs briefly for byte reception.
4. Supervisor task (100 Hz) — lower than motion.
5. Main loop — housekeeping, logging, debug CLI.

No mutexes in the ISR/motion path. Shared state uses atomic reads/writes on 32-bit words (natural on both platforms) and double-buffering for multi-word structures (profile segment data, for example).

---

## 10. Non-Real-Time Interface

### 10.1 Command set

```
MOVE_ABSOLUTE(pos_target_mm_x100, v_limit, i_limit, flags)
MOVE_INCREMENTAL(delta_mm_x100, v_limit, i_limit, flags)
STOP(mode = BRAKE | COAST)
CALIBRATE(direction)
SET_PARAM(id, value)          -- runtime-safe subset only
GET_PARAM(id)
GET_STATUS
CLEAR_FAULT
HEARTBEAT
```

### 10.2 Telemetry (100 Hz)

```
pos_mm_x100
v_mmps_x100
i_mA
V_bus_mV
state (enum)
fault_flags (bitfield)
baseline_i_at_pos_mA
temp_estimate_c_x10
uptime_ms
crc16
```

### 10.3 Framing and safety

- Byte-oriented frame with start byte, length, payload, CRC-16.
- Heartbeat required every 250 ms; absence for 500 ms triggers SAFE_IDLE.
- `SET_PARAM` is restricted to a whitelist of runtime-safe parameters (thresholds, limits). Motor constants require reboot to change.
- All received commands are plausibility-checked (range, rate) before acceptance.

This is the boundary between "ChainTree world" and "firmware world". The real-time tier trusts nothing it receives; the ChainTree tier expresses policy in s-expressions and emits the command stream.

---

## 11. Parameter Specification

Parameters are grouped by scope and persistence:

**Motor identity (const after flash):** R, L, Kt, Ke, J_motor, b, Fc, N, encoder_cpr, max_rpm.

**Application (config, push from ChainTree at init):** pos_min, pos_max, v_cruise, a_max, j_max (premium), pos_soft_limit_margin.

**Safety (config, runtime-updatable within ranges):** i_max_op, i_pinch_delta, dω_pinch_threshold, t_stall_ms, thermal_i2t_limit, thermal_tau_s.

**Tuning (config, derived from motor identity but overridable):** Kp_i, Ki_i, Kp_v, Ki_v, DOB_fc (premium).

**Learned state (non-volatile, written by firmware):** baseline_i_table[32 or 64], last_cal_timestamp, run_counter, i_offset_counts (volatile, re-measured at power-up).

---

## 12. Open Items

1. **Shim transport:** UART or SPI. UART is simpler; SPI is faster and more deterministic. Decision depends on ChainTree-side hardware and distance.
2. **Gate driver part selection:** integrated vs. discrete half-bridge drivers. Automotive-qualified options (e.g., DRV8705-Q1, or IFX007T for integrated). Affects BOM and safety architecture.
3. **Encoder CPR selection:** trade-off between velocity-estimate fidelity at low speed and M/T switchover complexity. 500 CPR is a reasonable default for this motor class.
4. **Exact Pittman part number:** identification tests confirm R, L, Kt, J, b; final gains are computed from those values.
5. **Thermal model fidelity:** single-pole lumped I²t vs. two-pole (winding + housing). Single-pole sufficient for intermittent duty; two-pole if duty-cycle limits matter.
6. **Calibration sequence specification:** order of seal-stall find, travel measurement, baseline table population, persistence.

---

## Appendix A — Glossary

- **ASIL:** Automotive Safety Integrity Level (ISO 26262). A/B/C/D in increasing rigor.
- **Back-EMF:** voltage generated by a spinning motor, proportional to velocity via Ke.
- **CMRR:** common-mode rejection ratio (amplifier's ability to ignore voltages common to both inputs).
- **CPR:** counts per revolution (quadrature encoder resolution after ×4 decoding).
- **DOB:** disturbance observer.
- **FMVSS 118:** US federal standard for power-operated window systems.
- **FPU:** floating-point unit.
- **IWDT / WDT:** independent / standard watchdog timer.
- **M/T method:** velocity estimation that switches between edge-counting and edge-timing.
- **POEG:** Port Output Enable for GPT (Renesas hardware PWM safety fault).
- **UNECE R21:** UN regulation on interior fittings, includes anti-pinch requirements.

## Appendix B — References (to populate)

- Pittman GM8000-series datasheet
- TI INA240 datasheet and reference design (TIDA-00909 or similar)
- Microchip SAMD21 family datasheet, §33 (ADC), §36 (TCC)
- Renesas RA4M1 user manual, Ch. 33 (GPT), Ch. 51 (ADC14), Ch. 52 (ELC)
- FMVSS 118 §S5
- UNECE Regulation 21 §5.8
