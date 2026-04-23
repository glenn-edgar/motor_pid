# Conversation: TI Smart Driver Integration & `pbio` Call Graph

Continuation of the `pbio` motion-control discussion. Covers TI I²C
dual H-bridge integration, stall handling, breakout availability, and a
structural review of the existing control architecture (two-level
organization, block diagram, src/ call trees).

---

## 1. TI I²C Dual H-Bridge — Effect on Control Strategy

**Question:** TI has an I²C dual H-bridge chip with PWM, closed-loop
speed control, and encoder inputs (DRV8214 / DRV8234 class). How would
it change the control strategy, and would it be effective?

### What would change

**1. Cascaded position → velocity.** Today the MCU computes a voltage.
With the smart chip, MCU emits a speed setpoint `ω_cmd` and the chip
closes the velocity loop internally. The outer position PID becomes
`ω_cmd = ω_ref + Kp·(θ_ref − θ_meas) + Ki·∫…`.

**2. The Luenberger observer becomes partially redundant.**
- `ω̂` — chip supplies a filtered speed from its own loop
- `î` — chip measures current directly
- `θ̂` — encoder counts come from the chip over I²C
- Stall detection moves from model-residual (voltage-based) to
  current-based — an **upgrade**

**3. Feedforward collapses.** Back-EMF and static friction
compensation move into the chip. What you lose is **`α_ref·J` preview**
— the chip does not know your planned acceleration and will lag the
trajectory by its own loop bandwidth.

**4. Drivebase heading-priority mix is the real casualty.** With
independent chip-internal loops, MCU cannot redistribute torque
headroom between wheels the way it does with voltage mixing. Straight-
line tracking under torque limit degrades. **Architecturally
significant regression.**

**5. I²C bus becomes the hot path.** Per 5 ms tick, for two motors:
write speed setpoint + read encoders + read status. At 400 kHz:
~400–600 μs, ~10% of tick budget. At 1 MHz FM+: negligible. Do not run
this chip at standard-mode 100 kHz.

**6. Jitter immunity on the velocity loop.** Chip's loop runs
asynchronously; MCU scheduler jitter no longer couples into velocity
tracking.

### Verdict for existing `pbio` drivebase

**Net neutral to mildly negative.** The stack's distinguishing features
— observer-filtered speed, `α_ref` feedforward, gain-scheduled `Kp`,
time-synchronized `(distance, heading)` trajectories, heading-priority
voltage mix — are either replicated worse by the chip or flat-out lost.
The plant is a geared LEGO motor with ~50–200 ms mechanical time
constant; 200 Hz outer loop already has ample bandwidth.

### Where the chip *would* be effective

- Greenfield apps without an existing servo stack
- Independent-axis robots (arms, single-motor mechanisms)
- MCU compute-starved designs
- When you need built-in current sense/stall and the board lacks sense
  resistors
- Safety-critical velocity limiting (chip-enforced, survives firmware
  bugs)

### Pragmatic hybrid for `pbio`

Use the chip in **PWM passthrough mode** (disable internal speed loop)
and consume only:
- Current sense → replaces observer `î` with a measurement
- Encoder counter → offloads one interrupt source
- Stall/fault pins

Keeps the entire `pbio` architecture and drivebase mix, gains one real
measurement (current). This is the `pbio`-compatible win.

---

## 2. TI Smart Driver — Stall Handling

**Question:** How does the chip handle stall?

### Three parallel mechanisms

**1. ITRIP hardware current regulation (µs-scale, always on).**
Configurable current threshold (`ITRIP_LVL`) chops PWM cycle-by-cycle
when current exceeds the limit. Not stall detection per se — a cycle-
level current cap protecting the bridge regardless of firmware. Response
time ~tens of µs, no MCU involvement.

**2. Stall *flag*.** Asserted from one of:
- Current above `STALL_TH` for a configurable window
- Ripple-count speed drop (sensorless tach via back-EMF ripple)
- Encoder-delta compared to commanded direction

Readable via I²C, routed to `nFAULT` pin (interrupt-capable). Integration
window typically 10–100 ms, tunable.

**3. Stall *action*.** Configurable:

| Mode | Behavior |
|---|---|
| Report-only | Set flag, assert nFAULT, keep driving |
| Auto-brake | Short both low-side FETs |
| Auto-coast | HiZ the bridge |
| Auto-retry | Disable for T_retry, re-enable, repeat |
| Latch | Disable until CLR_FLT is written |

### Comparison to `pbio` model-residual stall

`pbio` declares stall when the PID's proportional torque saturates in a
direction the reference is not reversing — "control law is asking for
more voltage than I can give." **Slow** (50–200 ms) but **context-
aware**: knows a wall-push at the end of `drive_straight` is expected
and pauses the trajectory integrator.

The chip's stall is **fast and blind**: sees current spike or speed
drop, not the trajectory. Flags aggressive braking (regen current) or
commanded hold-against-load as stall. Firmware must interpret.

### Recommended `pbio` integration

1. **ITRIP always on**, set conservatively above max FF torque.
   Silicon protection only; MCU sees nothing unless it trips.
2. **nFAULT → MCU EXTI** → reads status over I²C, propagates
   overcurrent/UVLO/thermal as hardware faults (new path — `pbio` has
   no driver-fault consumer today).
3. **Chip stall flag as input to `pbio` stall logic**, not
   replacement. Both agree → pause trajectory. Hardware-protection
   class (thermal, UVLO, ITRIP trip) → immediate stop.
4. **Disable chip auto-retry/auto-brake.** `pbio` owns policy. Stock
   chip behaviors assume it owns the control loop; it doesn't.

---

## 3. Breakout Board Availability

**Question:** Does any vendor make breakout boards for this unit?

Honest answer — availability not verified without a web search.

**Definitely exists:** TI EVMs (`DRV8214EVM`, `DRV8234EVM` and
variants). $50–100, bench-oriented, not for embedding. Canonical
breakout.

**Usually exists, needs checking:**
- **Pololu** — carries many TI driver breakouts (DRV8833, DRV8871,
  DRV8876, DRV8256). Tends to lag new parts (~1–2 years). Worth
  searching for DRV8214/8234 specifically.
- **MIKROE click boards** — often cover newer TI parts faster than
  hobby vendors.
- **Adafruit / SparkFun** — historically focus on older/simpler TI
  drivers. Less likely for this family.
- **AliExpress / LCSC modules** — generic breakouts, variable quality.

**Package matters:** DRV8214 is HTSSOP-16 with thermal pad —
hand-solderable with hot air. DRV8234 variants are often QFN/HTQFP,
need reflow. On HTSSOP, a custom 2-layer board with ~4 external
components is a weekend project and cheaper than anything off the
shelf.

---

## 4. Two-Level Structure in `pbio`

**Question:** Is there two levels — one for motors and one for heading
estimation based on encoders and IMU?

**Yes, two levels — but heading is *selected*, not *fused*.**

### Level 1: Per-motor servo (`servo.c`, `control.c`, `observer.c`)
200 Hz. Observer estimates `[θ̂, ω̂, î]` from **encoder only**. PID +
feedforward → voltage. One instance per motor.

### Level 2: Drivebase (`drivebase.c`)
200 Hz, sits above the two servos. Defines two virtual axes
`(distance, heading)`, each with its own `pbio_control_t` (PID). Not
fused state — decoupled coordinates. Outputs mix back to per-wheel
voltages (`src/drivebase.c:510–524`).

### The IMU hook — `src/drivebase.c:180–188`

```c
// Optionally use gyro to override the heading source for more accuracy.
if (db->gyro_heading_type != PBIO_IMU_HEADING_TYPE_NONE) {
    pbio_imu_get_heading_scaled(db->gyro_heading_type,
                                &state_heading->position,
                                &state_heading->speed,
                                db->control_heading.settings.ctl_steps_per_app_step);
}
```

Modes via `pbio_drivebase_set_use_gyro()`:
- `PBIO_IMU_HEADING_TYPE_NONE` — encoder difference `(θ_L − θ_R)` only
- `PBIO_IMU_HEADING_TYPE_1D` — integrated gyro on one fixed axis
- `PBIO_IMU_HEADING_TYPE_3D` — projected gyro onto horizontal plane

**This is a selector, not an estimator.** The function overwrites
`state_heading->position` and `state_heading->speed` with IMU values.
No complementary filter, no Kalman update, no residual check. Heading
PID cannot tell which source it is consuming.

### Consequences

- Gyro mode on → heading drifts at gyro bias rate (~1°/min)
- Encoder mode on → heading drifts with every wheel slip event
- The two sources never validate each other — stuck wheel / tumbled
  gyro both go undetected

The `pybrick_design.md` Mahony/pose proposal fills this gap: the
existing architecture has the hook (single `state_heading` input) but
doesn't populate it with a fused estimate. `pbio_imu_get_heading_scaled`
lives outside `src/` (only header present); based on the signature and
summary evidence, implementation is raw gyro integration — no accel,
no mag, no filter.

---

## 5. Architecture Block Diagram

In drivebase mode, per-motor servo PIDs are dormant — drivebase calls
`pbio_servo_actuate(... TORQUE, ...)` which converts torque → voltage
directly (`src/drivebase.c:500–525`). Per-motor observers still run for
state.

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         USER API  (MicroPython)                         │
│     drive_straight   drive_turn   drive_arc   drive(speed, turn_rate)   │
└────────────────────────────────┬────────────────────────────────────────┘
                                 ▼
╔═════════════════════════════════════════════════════════════════════════╗
║  LEVEL 2 — DRIVEBASE   (drivebase.c, 200 Hz)                            ║
║                                                                         ║
║   ┌──────────────────┐         ┌──────────────────┐                     ║
║   │ distance         │◄───────►│ heading          │  trajectory_stretch ║
║   │ trajectory       │  sync   │ trajectory       │  (time-locks both)  ║
║   └─────────┬────────┘         └─────────┬────────┘                     ║
║             │ θ_ref,ω_ref,α_ref          │ θ_ref,ω_ref,α_ref            ║
║             ▼                            ▼                              ║
║   ┌──────────────────┐         ┌──────────────────┐                     ║
║   │ distance PID     │         │ heading PID      │   (control.c)       ║
║   │  control_distance│         │  control_heading │  gain-scheduled Kp  ║
║   └─────────┬────────┘         └─────────┬────────┘                     ║
║             │ τ_dist                     │ τ_head                       ║
║             └──────────────┬─────────────┘                              ║
║                            ▼                                            ║
║                ┌───────────────────────────┐                            ║
║                │  MIX  +  per-wheel FF     │                            ║
║                │  τ_L = τ_d + τ_h + FF_L   │                            ║
║                │  τ_R = τ_d − τ_h + FF_R   │ (heading has priority      ║
║                └─────────┬───────┬─────────┘  on saturation)            ║
║                          │       │                                      ║
║   state_distance ▲       │       │      ▲ state_heading (pos + speed)   ║
║                  │       │       │      │                               ║
║   ┌──────────────┴───────┴───────┴──────┴───────────────────────────┐   ║
║   │   get_state_control()                                           │   ║
║   │     dist = (θ_L + θ_R)/2              ◄── always encoders       │   ║
║   │                                                                 │   ║
║   │     head = (θ_L − θ_R) ────┐                                    │   ║
║   │                            │   ┌─ SELECTOR ─┐                   │   ║
║   │     if gyro_heading≠NONE: ─┴──►│    mux     │◄── IMU heading    │   ║
║   │       head := IMU.heading      └─────┬──────┘                   │   ║
║   │                                      ▼                          │   ║
║   │                                   head_out                      │   ║
║   └─────────────────────────────────────┬───────────────────────────┘   ║
╚════════════════════════╤══════════════╤═╪═══════════════════════════════╝
                         │ τ_L          │ │                              ▲
                         ▼              ▼ │                              │
 ╔═══════════════════════════╗  ╔═════════╪═════════════╗      ┌─────────┴─┐
 ║ LEVEL 1 — LEFT SERVO      ║  ║ LEVEL 1 ▼  RIGHT SERVO║      │ IMU module│
 ║   (servo.c)               ║  ║  (servo.c)            ║      │ pbio_imu  │
 ║                           ║  ║                       ║      │           │
 ║  actuate(TORQUE, τ_L)     ║  ║  actuate(TORQUE, τ_R) ║      │ ∫ gyro dt │
 ║    └── τ→V via motor k    ║  ║    └── τ→V via motor k║      │ (1D or 3D)│
 ║                           ║  ║                       ║      │  NO accel │
 ║   ┌───────────────────┐   ║  ║   ┌───────────────────║      │  NO mag   │
 ║   │ observer          │   ║  ║   │ observer          │      │  NO fuse  │
 ║   │ [θ̂_L, ω̂_L, î_L]  │◄──╫──╫───┤ [θ̂_R, ω̂_R, î_R] │      └─────▲─────┘
 ║   │  (state_space)    │ θ_L ║   │  (state_space)    │            │
 ║   └────────▲──────────┘   ║  ║   └────────▲──────────┘            │
 ║            │              ║  ║            │         θ_R           │ raw ω
 ║   ┌────────┴──────────┐   ║  ║   ┌────────┴──────────┐            │
 ║   │ dcmotor (PWM)     │   ║  ║   │ dcmotor (PWM)     │       ┌────┴────┐
 ║   └────────┬──────────┘   ║  ║   └────────┬──────────┘       │ MEMS    │
 ╚════════════╪══════════════╝  ╚════════════╪══════════════════╡ gyro    │
              ▼                               ▼                  │ +accel  │
         ┌─────────┐                    ┌─────────┐              └─────────┘
         │ Motor L │                    │ Motor R │
         │+encoder │                    │+encoder │
         └────┬────┘                    └────┬────┘
              │ θ_L                          │ θ_R
              └──────────────────┬───────────┘
                                 │
                                 └──► feeds back into get_state_control
```

### Observations

1. Level 1 observers still run in drivebase mode (state only, PID
   dormant).
2. FF lives at Level 2, per-wheel, computed from drivebase references.
3. SELECTOR is the *only* point IMU data enters the control loop —
   a mux, not a fuser.
4. IMU module is a leaf, not a level. Mahony filter would replace its
   internals without changing any upstream wiring.
5. No pose layer. Nothing consumes `(x, y)`.

---

## 6. Call Tree of `src/` Files

### Top-level entry points

| Entry | File | Type |
|---|---|---|
| `pbio_servo_update_all` | servo.c | scheduler tick |
| `pbio_drivebase_update_all` | drivebase.c | scheduler tick |
| `pbio_servo_run_*`, `pbio_servo_track_target`, `pbio_servo_stop` | servo.c | user command |
| `pbio_drivebase_drive_*`, `pbio_drivebase_stop`, `pbio_drivebase_reset` | drivebase.c | user command |
| `pbio_servo_setup`, `pbio_drivebase_get_drivebase[_spike]`, `pbio_dcmotor_init_instance` | — | init |

### Servo tick (per-motor servo mode)

```
pbio_servo_update_all → pbio_servo_update
  ├─ pbio_servo_get_state_control
  │   ├─ pbio_tacho_get_angle             (encoder read)
  │   └─ pbio_observer_get_estimated_state
  ├─ pbio_control_update                    ← PID, gain-scheduled Kp
  │   ├─ pbio_trajectory_get_reference
  │   ├─ pbio_position_integrator_* / pbio_speed_integrator_*
  │   └─ [math]
  ├─ pbio_observer_get_feedforward_torque
  ├─ pbio_servo_actuate
  │   └─ pbio_dcmotor_set_voltage | coast
  ├─ pbio_observer_update                   ← model propagation
  │   └─ pbio_differentiator_update_and_get_speed
  ├─ pbio_observer_get_feedback_voltage     ← stall residual
  └─ pbio_logger_add_row
```

### Drivebase tick (supersedes per-motor PID)

```
pbio_drivebase_update_all → pbio_drivebase_update
  ├─ pbio_drivebase_get_state_control
  │   ├─ pbio_drivebase_get_state_via_motors
  │   │   ├─ pbio_servo_get_state_control     (L, R)
  │   │   └─ pbio_angle_{avg,diff}
  │   └─ pbio_imu_get_heading_scaled   ◄── SELECTOR (external, imu backend)
  ├─ pbio_control_update × 2                   (distance, heading)
  │   └─ pbio_trajectory_get_reference + integrators
  ├─ pbio_observer_get_feedforward_torque × 2  (L, R per-wheel FF)
  └─ pbio_servo_actuate × 2
      └─ pbio_dcmotor_set_voltage               (TORQUE → V, skips servo PID)
```

### User command (example: `run_target`)

```
pbio_servo_run_target
  ├─ pbio_parent_stop                 ← detaches any active drivebase
  ├─ pbio_servo_get_state_control
  └─ pbio_control_start_position_control
      └─ pbio_trajectory_new_angle_command
```

Drivebase commands mirror this but also call `pbio_trajectory_stretch`
to time-sync the two trajectories.

### Module responsibility

| Module | Owns | Called by |
|---|---|---|
| servo.c | per-motor servo state, user run-* API | scheduler, user, drivebase |
| drivebase.c | dual-axis coordination, heading selector, arc trajectories | scheduler, user |
| control.c | PID + completion logic | servo, drivebase (×2) |
| observer.c | state-space model, FF torque, feedback-voltage residual (stall) | servo, drivebase |
| trajectory.c | trapezoidal profile math, stretch/sync | control, drivebase |
| integrator.c | position/speed integrators, windup logic | control |
| dcmotor.c | voltage actuation, coast/brake | servo, drivebase |
| tacho.c | encoder angle, offset | servo |
| differentiator.c | speed from Δθ samples | observer |
| parent.c | parent/child link (servo ↔ drivebase) | servo, drivebase, dcmotor |
| control_settings.c | app↔ctl unit conversion, gain math | control, servo, drivebase |
| angle.c, int_math.c, util.c | 32-bit angle wrap, fixed-point math, time compare | everywhere |
| logger.c | ring-buffer diagnostics | servo, control |
| servo_settings.c | default gains / limits loader | servo |

### Cross-module call matrix

```
caller \ callee   servo  drive  ctrl  obs  traj  intg  dc  tach  diff
servo              ·      ·      ✓     ✓    ·     ·     ✓   ✓     ·
drivebase          ✓      ·      ✓     ✓    ✓     ·     ✓   ·     ·
control            ·      ·      ·     ·    ✓     ✓     ·   ·     ·
observer           ·      ·      ·     ·    ·     ·     ·   ·     ✓
trajectory         ·      ·      ·     ·    ·     ·     ·   ·     ·
integrator         ·      ·      ·     ·    ·     ·     ·   ·     ·
dcmotor            ·      ·      ·     ·    ·     ·     ·   ·     ·
tacho              ·      ·      ·     ·    ·     ·     ·   ·     ·
```

(Math/util modules omitted — called by all, call nobody.)

### Shape of the graph

- **Strict DAG, shallow.** No cycles. Longest control-path chain is
  4 deep: `drivebase_update → control_update → trajectory_get_reference
  → [math]`.
- **control.c is the central choke point.** Both servo and drivebase
  funnel through it. A new outer loop (pose-closed pursuit) plugs in
  above `control_update`, not inside it.
- **observer.c is a pure leaf** (calls only differentiator + math).
  Swappable without touching callers — a Kalman replacement is
  feasible.
- **IMU is external**, attached at exactly one point
  (`drivebase_get_state_control`). A Mahony filter only has to honor
  the `pbio_imu_get_heading_scaled` signature.
- **trajectory.c is a leaf** from control.c's perspective — replacing
  the trapezoidal with a spline evaluator is a one-module swap, as
  anticipated by Tier B in the design doc.
