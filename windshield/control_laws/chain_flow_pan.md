# Windshield Execution Architecture

**Status:** Draft v0.1
**Supersedes relevant sections of:** `windshield.md` §3 (system architecture), `windshield_algorithms.md` §2 (shared skeleton), `windshield_demo_pinout.md` §3.3 and §4.3 (peripheral configuration)
**Scope:** How execution is organized across contexts, how the policy engine (ChainTree or s-expression) fits in, and how the single ADC on each XIAO is multiplexed to serve the critical current-loop path alongside supervisory signals.

---

## 1. Purpose

Earlier docs in the windshield series specified *what* the controller does and *on what hardware*. This document specifies *how execution is structured* — the set of contexts that run, their priorities, what work belongs in each, how shared state crosses between them, and how the ADC hardware is multiplexed to serve multiple signals without compromising the current loop.

Three architectural commitments consolidate here:

1. **The policy engine is swappable.** The supervisor tier runs a policy engine — either the ChainTree C engine or the S-expression C engine — hosting the orchestration, learning, and decision logic that isn't time-critical. Both engines are benchmark candidates for "better embedded fit on 32 KB RAM systems." The real-time tier is engine-agnostic.

2. **Five well-defined execution contexts.** Current-loop ISR, hardware fault ISR, motion task, supervisor task, main loop. USB CDC runs as a lower-priority ISR feeding queues. No ambiguity, no accidentally-mixed responsibilities.

3. **ADC scan-and-dispatch.** Both XIAOs have a single ADC peripheral. Both support hardware-triggered channel scanning with DMA/DTC result delivery. All control-relevant analog signals are sampled every PWM cycle, with I_SENSE first in the sequence for minimum latency.

---

## 2. Policy Engine Hosting — ChainTree vs S-Expression

The supervisor tier runs a policy engine that interprets authored policy (behavior trees or s-expressions) against the live telemetry stream and emits commands back to the motion task. The engine is the bulk of the RAM-resident logic; the firmware around it is small and deterministic.

### 2.1 What the engine owns

All orchestration, learning, and non-real-time decision making:

- **Calibration sequencing** — multi-pass stall-find, stddev gating, accept/reject, persistence
- **Baseline learning** — Welford updates on current and DOB residual per position bucket
- **Temperature compensation** — binned tables (basic) or learned slope per bucket (premium)
- **Anti-pinch response** — what to do when the motion task raises the trip flag (reverse distance, stop mode, user notification)
- **Plausibility policy** — which checks are active in which mode, fault thresholds, retry counts
- **Mode sequencing** — auto-up with pinch-reverse, memory positions, calibrate-on-first-use
- **Fault response orchestration** — state transitions, recovery attempts, safe-state entry
- **Fleet-facing behaviors** — OTA policy push, maintenance predictions from baseline statistics

### 2.2 What the engine does *not* own

The current loop and anti-pinch detection *math* live in the motion task on the MCU:

- Current PI, velocity PI, profile executor — all in firmware, all deterministic
- Per-tick threshold *comparison* (is current above the cached threshold for this bucket?) — motion task
- Raw detection flag generation — motion task
- Hardware safety latches — wired, not software

The engine computes thresholds *via Welford statistics* and pushes them down to the motion task's cached table. The motion task never does statistical math; it just compares.

### 2.3 The A/B benchmark

Two candidate engines, same policy ported to both, same firmware beneath:

**Engine A — ChainTree C engine**
- Policy expressed as a behavior tree with parameterized nodes
- Nodes dispatched via FNV-1a hash → function pointer table
- State lives in node instances plus a shared blackboard
- Wheelhouse: sequences, fallbacks, decorators, orchestration

**Engine B — S-expression C engine**
- Policy compiled to `.sexb` binary modules
- Quad-instruction interpretation with stack frames, function dictionary
- State lives in the stack and a dictionary-backed environment
- Wheelhouse: computation, statistical operations, recursive logic

Neither is obviously "better" across the whole policy workload. The windshield policy divides into orchestration-heavy pieces (calibration, mode sequencing, fault response) where ChainTree shines, and computation-heavy pieces (Welford, z-scores, voting, temperature math) where s-expressions shine. A hybrid deployment is possible but outside the initial benchmark scope — we run each engine against the full policy and measure what falls out.

### 2.4 Measurement axes

For each engine, measured on the same XIAO hardware and same policy workload:

| metric | method |
|---|---|
| Static RAM footprint | linker map + runtime query at init |
| Runtime RAM peak | instrumented high-water-mark during calibration + MOVING |
| Flash footprint | engine binary + loaded policy module size |
| CPU per supervisor tick | DWT cycle counter measurement from tick entry to exit |
| Worst-case tick jitter | max over N minutes of operation |
| Policy source size | lines of authored policy |
| Compiled policy size | `.ctb` or `.sexb` bytes |
| Push-and-activate time | USB-CDC push start to new-policy-active latching |
| Fault diagnostic quality | qualitative — what a fault dump tells you |

Run the same scenarios against both engines, log the numbers, compare. The windshield workload is well-defined enough that "did both engines produce equivalent control decisions?" is trivially verifiable.

### 2.5 What both engines require from the firmware

The firmware/engine interface is the **shim protocol over USB CDC** (detailed in the forthcoming `windshield_shim_protocol.md`). Both engines consume the same telemetry stream and emit the same command set. Neither engine leaks concepts into the firmware. This is what makes the benchmark apples-to-apples and what makes the firmware verifiable in isolation.

The firmware exposes:

**Commands accepted (from engine):**
- `move_to(pos, v_max, i_max, flags)`
- `move_rel(delta, v_max, i_max, flags)`
- `stop(mode: brake | coast)`
- `cal_find_stall(direction)`
- `set_threshold(bucket, value)` — pushes one entry of the anti-pinch threshold table
- `set_threshold_block(start_bucket, values[N])` — bulk push
- `set_pi_gains(kp_i, ki_i, kp_v, ki_v)` — rare, typically init-only
- `set_profile_params(v_cruise, a_max, j_max)` — per-mode tuning
- `get_status` — query current state

**Events/telemetry emitted (to engine):**
- Periodic telemetry at 100 Hz: `pos, velocity, current, v_bus, temp, state`
- Burst telemetry at 1 kHz for ~128 ms after anomaly: full state ring
- Asynchronous events: `stall_detected`, `trip_flag_raised`, `move_complete`, `move_aborted`, `fault(code)`
- Heartbeat response

---

## 3. Execution Contexts

Five contexts, fixed priority, explicit preemption rules.

### 3.1 Context inventory

| # | context | trigger | rate | priority | preempts |
|---|---|---|---|---|---|
| 1 | Current-loop ISR | ADC end-of-conversion (PWM-triggered) | 10 kHz (SAMD21) / 20 kHz (RA4M1) | highest | everything |
| 2 | Hardware fault ISR | nFAULT edge or comparator trip | asynchronous, rare | high | motion, supervisor, main |
| 3 | Motion task | timer ISR | 1 kHz | medium-high | supervisor, main |
| 4 | Supervisor task | timer wake or cooperative dispatch | 20 Hz | medium-low | main only |
| 5 | Main loop / idle | default | whenever nothing else runs | lowest | nothing |
| (aux) | USB CDC ISR | USB controller event | asynchronous | low | main only |

### 3.2 Context 1 — Current-Loop ISR

**Trigger:** ADC end-of-sequence interrupt. The ADC was started by an EVSYS (SAMD21) or ELC (RA4M1) event from the PWM timer compare-match at the midpoint of the PWM on-time.

**Work:**
- Read ADC results for the full channel sequence from the DMA/DTC ring
- Scale raw counts to physical units (current in amps, V_bus in volts, etc.)
- Apply offset correction (from power-up calibration)
- Run current PI: `V_cmd = Kp_i * (i_cmd - i_meas) + Ki_i * integrator + Ke * omega`
- Saturate V_cmd, update PWM duty register
- Publish `i_measured`, `v_bus`, `ipropi`, `temp` to shared state for other contexts to read

**Budget:** 50 μs period at 20 kHz (RA4M1), 100 μs at 10 kHz (SAMD21). Actual work 10-15 μs.

**Shared state written:** `i_measured`, `v_bus_latest`, `temp_latest`, `ipropi_latest`, `pwm_duty`. Single-word atomic writes.
**Shared state read:** `i_cmd`, `omega_estimate` (from motion task).

**Preemption:** nothing. Highest priority.

### 3.3 Context 2 — Hardware Fault ISR

**Trigger:** external interrupt on the nFAULT input pin. The pin itself is also wired through EVSYS/ELC to the PWM fault-input peripheral (TCC FAULT_A on SAMD21, POEG on RA4M1), which disables the PWM outputs in hardware *immediately* — this ISR is for software notification, not for the actual cutoff.

**Work:**
- Latch the fault cause bits (which source asserted nFAULT — overcurrent, bridge internal, thermal, undervoltage)
- Snapshot the telemetry ring buffer for post-mortem
- Set a flag visible to the motion task and supervisor
- Enqueue a fault-log NVM write task for main loop

**Budget:** happens rarely, a few hundred cycles of work.

**Preemption:** preempts motion task, supervisor, main. Does *not* preempt the current-loop ISR — but it doesn't need to, because the hardware has already killed the PWM before this ISR runs.

### 3.4 Context 3 — Motion Task

**Trigger:** dedicated timer interrupt at 1 kHz.

**Work:**

*Every tick:*
- Read encoder position (register read on RA4M1 hardware QDEC; accumulated counter on SAMD21 EIC-decoded)
- Compute velocity via M/T estimator
- Advance profile executor (trapezoidal on SAMD21, S-curve on RA4M1)
- Run velocity PI, optionally with feedforward (M4F)
- Compute dv/dt
- Compute τ̂_L (RA4M1 only, via disturbance observer)
- Look up `threshold[bucket]` from the cached threshold table; compare `i_measured` against it
- If `(current > threshold) AND (dv/dt < dv_threshold)`: set `trip_flag`
- Run plausibility checks: direction consistency, stall-without-motion, velocity-duty ratio (M4F)
- Push updated `i_cmd` to shared state for current-loop ISR
- Push one telemetry frame to the ring buffer

*Less frequently:*
- Every 10 ticks (100 Hz): emit telemetry frame to USB CDC TX queue
- Every 1 ms during anomaly: burst telemetry at full 1 kHz rate for ~128 ms

**Budget:** 1 ms period. Actual work 30-100 μs. Plenty of headroom.

**Shared state written:** `i_cmd`, `omega_estimate`, `trip_flag`, `telemetry_ring` (head advances), `plausibility_fault_code`.
**Shared state read:** `threshold_table[]` (via atomic pointer), `move_command` (current in-progress), `i_measured`, `v_bus_latest`.

**Preemption:** preempted by current-loop ISR and hardware fault ISR. Preempts supervisor and main.

### 3.5 Context 4 — Supervisor Task

**Trigger:** timer wake at 20 Hz.

**Work:**

*Every tick (50 ms period):*
- Drain telemetry ring buffer — typically 50 frames since last supervisor tick (50 ms × 1 kHz)
- Drain USB CDC RX queue, parse commands from ChainTree fleet
- Check heartbeat freshness → if expired, command SAFE_IDLE
- Invoke the policy engine tick (ChainTree behavior tree or s-expression policy)
  - Engine consumes telemetry, updates Welford baselines, emits commands
  - Engine state transitions: IDLE → MOVING → REVERSING → FAULT, etc.
- If engine computed new thresholds, push via `set_threshold_block` to motion task (updates the shadow table, atomic pointer swap)
- Enqueue TX telemetry (summary, 1 Hz) to USB CDC TX queue
- Enqueue NVM writes (baseline persistence, fault logs) to main-loop queue
- Check for fault flags from motion task or hardware fault ISR; orchestrate response

**Budget:** 50 ms period. Actual engine work 1-10 ms. Very comfortable.

**Shared state written:** `threshold_table_shadow[]`, `move_command`, `profile_params`, policy-engine internal state.
**Shared state read:** `telemetry_ring` (tail advances), `trip_flag`, `plausibility_fault_code`.

**Preemption:** preempted by everything above. Preempts only main loop.

### 3.6 Context 5 — Main Loop / Idle

**Trigger:** default execution when nothing else runs.

**Work:**
- Drain NVM write queue — write baseline snapshots, fault logs, calibration persistence
- Pet the watchdog (alternatively from motion task; doing it here also catches "supervisor dead" since watchdog-petting depends on the task queue being serviced)
- Slow housekeeping — UART debug (if enabled), LED status, sleep-mode entry logic
- Anything that can tolerate jitter and blocking

**Budget:** unbounded, runs only when the CPU has nothing else to do.

**Preemption:** preempted by everything.

**Critical rule:** NVM writes *must* happen here. Flash sector erases take 10-50 ms on SAMD21, during which the CPU is actually stalled. Doing this in any periodic context would miss ticks. Main loop is the only safe place.

### 3.7 USB CDC ISR (aux)

**Trigger:** USB controller event (Start-of-Frame, OUT endpoint received data, IN endpoint ready for more).

**Work:**
- On OUT (host → device): copy received bytes into RX ring buffer, signal ready flag
- On IN (device → host): check TX ring buffer, transmit available data
- Minimal parsing — frame assembly and CRC validation happen in the supervisor task, not here

**Budget:** events happen at up to 1 kHz (one USB frame per ms); ISR is brief (~50 μs).

**Priority:** lower than motion task. The supervisor reads out the RX queue each 50 ms tick; bytes are buffered in the meantime.

### 3.8 Priority stack visualization

```
           ↑ Preempts
  ┌────────────────────────────────────┐
  │ Current-loop ISR       (20 kHz)    │ ← nothing preempts this
  ├────────────────────────────────────┤
  │ Hardware fault ISR     (async)     │
  ├────────────────────────────────────┤
  │ Motion task            (1 kHz)     │
  ├────────────────────────────────────┤
  │ USB CDC ISR            (async)     │ ← lower than motion by design
  ├────────────────────────────────────┤
  │ Supervisor task        (20 Hz)     │
  ├────────────────────────────────────┤
  │ Main loop / idle       (background)│
  └────────────────────────────────────┘
           ↓ Preempted by
```

### 3.9 Shared state and cross-context discipline

Every shared variable crosses exactly two contexts. Single-producer-single-consumer patterns dominate.

**Current ISR ↔ Motion task:**
| variable | writer | reader | mechanism |
|---|---|---|---|
| `i_cmd` | motion | ISR | atomic 32-bit word |
| `i_measured` | ISR | motion | atomic 32-bit word |
| `v_bus_latest` | ISR | motion | atomic 32-bit word |
| `omega_estimate` | motion | ISR | atomic 32-bit word |

**Motion task ↔ Supervisor:**
| variable | writer | reader | mechanism |
|---|---|---|---|
| `threshold_table[]` | supervisor | motion | shadow buffer + atomic pointer swap |
| `telemetry_ring[]` | motion | supervisor | SPSC ring, atomic head/tail |
| `trip_flag` | motion | supervisor | atomic bit, supervisor clears |
| `move_command` | supervisor | motion | double-buffered struct + atomic pointer |
| `plausibility_fault_code` | motion | supervisor | atomic word |

**USB ISR ↔ Supervisor:**
| variable | writer | reader | mechanism |
|---|---|---|---|
| `usb_rx_ring[]` | USB ISR | supervisor | SPSC ring |
| `usb_tx_ring[]` | supervisor | USB ISR | SPSC ring |

**Main loop ↔ Supervisor:**
| variable | writer | reader | mechanism |
|---|---|---|---|
| `nvm_write_queue` | supervisor | main | SPSC queue |

No mutexes, no semaphores, no priority inversion. Every crossing is either a single atomic word or a SPSC ring buffer. Verifiable by inspection.

### 3.10 What's deliberately *not* a separate context

Worth stating explicitly:

- **No separate safety context.** Safety is distributed across the hardware latch, the current-loop ISR, the motion task, the hardware fault ISR, and the supervisor. Adding a fifth abstraction would add confusion, not safety.
- **No separate calibration context.** Calibration is a *mode* the supervisor enters. It commands the motion task through the normal shim and waits for events. No new execution context needed.
- **No separate thermal context.** Thermal model updates run inside the supervisor tick, divided down (1 Hz typical).
- **No separate fleet-comm context.** USB CDC handles everything; fleet-level telemetry is just another command/telemetry pattern over the same channel.
- **No separate "slow supervisor" context.** The 20 Hz tick can divide down internally for slower work (thermal @ 1 Hz, fleet telemetry @ 0.1 Hz) via counters.

---

## 4. ADC Multiplexing

Both XIAOs have exactly one ADC peripheral. Both can multiplex cleanly using hardware-scan modes. The workload (4 channels sampled per PWM cycle) fits comfortably on both.

### 4.1 Signals to convert

Four analog channels in this design:

| signal | source | consumer | required fresh-rate |
|---|---|---|---|
| I_SENSE | INA240 breakout VIOUT | current loop (ISR) | every PWM cycle — critical |
| V_BUS | divider output | motion task, thermal model | 1 kHz is plenty |
| TEMP | NTC thermistor (SAMD21) or internal (RA4M1) | thermal model | 10 Hz is plenty |
| IPROPI | DRV8873H breakout current monitor | supervisor cross-check | 100 Hz is plenty |

The strategy: **sample all four every PWM cycle** via hardware scan, publish to shared state, let each consumer pick up what it needs at the rate it needs. Simpler than conditional scanning, cost is negligible on both chips.

### 4.2 Channel order

I_SENSE is first in every scan. Its result is available fastest after the PWM midpoint trigger, minimizing latency into the current loop.

```
  PWM midpoint event
        │
        ▼
  Slot 0: I_SENSE     ← available ~1-3 μs after trigger
        │
        ▼
  Slot 1: V_BUS
        │
        ▼
  Slot 2: TEMP
        │
        ▼
  Slot 3: IPROPI
        │
        ▼
  End-of-scan interrupt
```

### 4.3 SAMD21 configuration

**Hardware:** ADC0, single 12-bit SAR, ~350 ksps throughput, one sample-and-hold.

**Trigger path:**
```
TCC0 compare-match (mid-PWM event)
  → EVSYS channel 0: user = ADC_START_SINGLE
    → ADC starts channel 0 (I_SENSE)
      → ADC result-ready event
        → DMA (channel 0): copies result to ring, increments pointer
          → Next ADC sample triggered via INPUTSCAN sequencing
            → ... repeats through 4 channels ...
              → Sequence complete interrupt
```

**Key registers:**
- `ADC->INPUTCTRL.bit.INPUTSCAN = 3` — scan 4 channels starting from MUXPOS
- `ADC->INPUTCTRL.bit.MUXPOS = AIN_I_SENSE` — starting channel
- `ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val`
- `ADC->SAMPCTRL.reg = appropriate value` — sampling time; INA240 output is low-impedance so minimum sampling is fine, but NTC thermistor input (if in the scan) needs more
- `ADC->EVCTRL.bit.STARTEI = 1` — enable start-on-event
- EVSYS: channel 0 generator = TCC0 MC0 (compare match), user = ADC_START
- DMA: one channel, source = `ADC->RESULT.reg`, destination = result buffer, transfer on ADC RESRDY event

**Timing:**
- ADC clock max for 12-bit: 2.1 MHz
- Per-conversion: ~3 μs (including sample time for all our sources)
- Full 4-channel scan: ~12 μs
- 10 kHz scan rate: 12% duty cycle on ADC
- Fits inside the PWM on-time at typical duties

**Gotchas:**
- All channels in a scan share gain, reference, and resolution settings
- NTC has higher source impedance than INA240 → use longer sampling time, or buffer the NTC through an op-amp
- Reference: VDDANA = 3.3 V — stable, shared across all channels, no settling delays

### 4.4 RA4M1 configuration

**Hardware:** ADC14 (one unit), 14-bit SAR, ~1 μs per conversion, sample-and-hold capable of multiple simultaneous captures, two independent conversion groups (A and B).

**Group A — PWM-synchronized, critical signals (every PWM cycle, 20 kHz):**
- I_SENSE (AN09)
- V_BUS (AN00)
- Triggered by ELC from GPT3 compare-match at mid-PWM
- Result delivery via DTC to ring buffer
- Completion interrupt drives the current loop

**Group B — Software-triggered or low-rate, supervisory signals (100 Hz):**
- IPROPI (via DRV8873H monitor output, on a spare ADC pin)
- TEMP (MCU internal temperature sensor, or external NTC if more accuracy needed)
- Triggered by software from motion task every 10 ticks, or from supervisor when needed
- Result delivery via DTC or direct register read

**Key configuration:**
- `ADC14.ADCSR.ADCS[1:0] = 01b` — single-scan mode (per group)
- `ADC14.ADANSA0/1` — channel selection for Group A and B respectively
- `ADC14.ADCER.ACE = 1` — automatic clearing of data register
- ELC: link GPT3 compare-match event → `ADC_CHA` trigger (Group A start)
- DTC: transfer block for Group A results into motion-task ring buffer

**Timing:**
- Per-conversion: ~1 μs at 14-bit
- Group A (2 channels): ~2 μs per scan
- Group B (2 channels): ~2 μs per scan, but runs at 100 Hz not 20 kHz
- Group A 20 kHz × 2 μs = 4% duty cycle on ADC
- Very comfortable

**The Group A/B separation matters.** Group A is inviolate — it always runs at the PWM rate, never gets held up by anything. Group B runs when Group A isn't converting. If you tried to cram everything into one group scan, a 4-channel sequence at 20 kHz would still fit but you'd lose the flexibility to change IPROPI sampling rate independently of V_BUS.

### 4.5 Offset calibration (both platforms)

Independent of which platform, the INA240's output at zero current isn't exactly mid-rail — there's a few mV of initial offset plus ~10 μV/°C drift. Procedure:

1. At every power-up, with H-bridge disabled (all FETs off, motor electrically floating):
2. Wait 10 ms for any residual current to decay
3. Trigger 256 ADC conversions of I_SENSE at the normal scan rate (ignore other channels' results)
4. Average → `i_offset_counts`
5. Store in RAM
6. Subtract from every subsequent I_SENSE reading before scaling to amps

On RA4M1, additionally re-sample offset during every move's pre-enable dead time (~1 ms before bridge energizes) to track thermal drift during operation. SAMD21 can do the same but it's less critical because 12-bit resolution is coarser than the drift.

### 4.6 Scaling reference

For the current breakout actually in use (Adafruit INA240A2, 2 mΩ shunt, gain 50 → 100 mV/A):

**SAMD21 (12-bit, Vref = 3.3 V):**
- ADC LSB = 0.806 mV
- Counts per amp = 124
- Resolution = 8 mA/LSB
- Useful full-scale = ±13 A (headroom before rail-clip)

**RA4M1 (14-bit, Vref = 3.3 V):**
- ADC LSB = 0.201 mV
- Counts per amp = 497
- Resolution = 2 mA/LSB
- Useful full-scale = ±13 A

For V_BUS divider (100 kΩ / 10 kΩ, ratio 0.0909):
- At 12 V bus → 1.09 V ADC → 1351 counts (SAMD21) / 5405 counts (RA4M1)
- At 36 V bus → 3.27 V ADC → 4056 counts (SAMD21) / 16220 counts (RA4M1) — near rail

### 4.7 Sampling latency into the current loop

The interesting question for the current-loop designer: **how old is the I_SENSE value when the PI runs?**

Timeline for RA4M1:
```
t = 0 μs    PWM compare-match at midpoint → ELC event → ADC starts
t = +1 μs   I_SENSE conversion complete → DTC writes result
t = +2 μs   V_BUS conversion complete
t = +2.5 μs end-of-group-A interrupt fires
t = +3 μs   ISR begins executing, reads result ring
t = +3.5 μs current PI math begins, i_meas is ~3 μs old
t = +10 μs  PWM duty register updated
```

3 μs from physical event to PI execution. This is essentially the best achievable on Cortex-M — any further reduction would require sampling at some other clock phase, which trades latency for noise.

SAMD21 timeline is similar but conversion times are ~3 μs per channel, so I_SENSE result arrives at ~3 μs, PI runs at ~7 μs. Still well under the 100 μs PWM period.

Both fast enough that current-loop bandwidth is limited by the motor L/R (~1 ms time constant), not by sampling latency.

### 4.8 Published shared state

After each scan completes:

```c
// Updated by current-loop ISR
typedef struct {
    float i_amps;           // calibrated, offset-corrected
    float v_bus_volts;
    float temp_celsius;     // low-pass filtered, slow-varying
    float ipropi_amps;      // redundant current sense
    uint32_t sequence_count; // monotonic, for consumer to detect new data
} adc_state_t;

adc_state_t adc_state;  // written by ISR, read by motion task
```

Motion task reads this every 1 ms; supervisor reads via motion's ring buffer at 20 Hz. Temperature is heavily filtered in the ISR because the underlying thermal mass changes slowly but the ADC sample can be noisy; filtering in the ISR means all consumers see the same smoothed value.

---

## 5. Putting It Together — A Trace of One Cycle

Walk through what happens in a single PWM cycle during normal MOVING state:

```
t = 0 μs       PWM timer compare-match fires at mid-on-time
               ├─ Hardware: ELC event routes to ADC Group A start
               └─ Hardware: No CPU involvement yet

t = +1 μs      ADC converts I_SENSE (channel 0)
               └─ DTC writes result to adc_ring[0]

t = +2 μs      ADC converts V_BUS (channel 1)
               └─ DTC writes result to adc_ring[1]

t = +2.5 μs    End-of-scan interrupt fires
               Current-loop ISR begins:
               ├─ Read adc_ring[0], convert to amps (i_meas)
               ├─ Read adc_ring[1], convert to volts (v_bus)
               ├─ Compute error = i_cmd - i_meas
               ├─ Update integrator (with anti-windup)
               ├─ Compute V_cmd = Kp*error + Ki*integrator + Ke*omega
               ├─ Saturate V_cmd, write PWM duty register
               └─ Publish i_meas, v_bus to adc_state

t = +12 μs     Current-loop ISR exits
               (CPU resumes whatever was running — usually idle)

... wait for 1 ms timer tick ...

t = 1000 μs    Motion task timer fires
               Motion task runs (preempts main/supervisor):
               ├─ Read encoder → compute velocity
               ├─ Advance profile executor by 1 ms
               ├─ Compute v_ref, pos_ref
               ├─ Compute dv/dt, tau_l_hat (M4F)
               ├─ Velocity PI: i_cmd = f(v_ref - v_meas, feedforward)
               ├─ Write i_cmd to shared state
               ├─ Compute bucket from current position
               ├─ Look up threshold[bucket]
               ├─ if (i_meas > threshold AND dv/dt < dv_thresh):
               │      set trip_flag
               ├─ Run plausibility checks (direction, stall, velocity/duty)
               ├─ Push telemetry frame to ring buffer
               └─ (every 10 ticks) enqueue USB CDC TX frame

t = 1050 μs    Motion task exits (~50 μs of work)

... 19 more motion ticks pass (19 ms) ...

t = 20 ms      Supervisor timer fires
               Supervisor runs (preempts main):
               ├─ Drain telemetry ring (50 frames accumulated)
               ├─ Drain USB CDC RX queue → parse commands
               ├─ Check heartbeat freshness
               ├─ Invoke engine.tick() — ChainTree or s-expression
               │    ├─ Engine consumes telemetry
               │    ├─ For each telemetry frame during MOVING:
               │    │    ├─ Welford update baseline[bucket_i]
               │    │    └─ Welford update baseline[bucket_tau] (M4F)
               │    ├─ If trip_flag was raised: orchestrate REVERSING
               │    │    ├─ Compute retract distance
               │    │    ├─ emit set_move_cmd(...)
               │    │    └─ Transition state
               │    ├─ Compute fresh threshold for any bucket whose
               │    │   Welford stats changed materially
               │    └─ Emit set_threshold commands for updated buckets
               ├─ Enqueue NVM write tasks for main loop if baselines
               │   haven't been persisted in > 10 s
               └─ Enqueue fleet telemetry (1 Hz divided down)

t = 20 ms + 5ms  Supervisor exits (~5 ms of engine work)

... main loop runs in the gaps, NVM write happens if queued ...
```

Three contexts, working together, with clean timing separation at each rate boundary. The engine is a participant in this flow but never blocks real-time work.

---

## 6. Open Items

1. **Confirm ELC event routing to ADC Group A on RA4M1** — datasheet says this works but bring-up should verify.
2. **SAMD21 EVSYS channel allocation** — ensure no conflicts between current-loop path and any other peripheral that wants an EVSYS channel.
3. **DMA/DTC channel allocation** — both MCUs have limited channels; budget them: one for ADC results, reserved.
4. **Threshold table update mechanism** — atomic pointer swap requires alignment; verify the table is 4-byte aligned and the pointer is atomic on both MCUs.
5. **USB CDC throughput under anomaly burst** — 128 ms of 1 kHz × ~40-byte frames = ~5 KB/s. USB Full-Speed has no problem with this; supervisor-side consumer (ChainTree) must keep up.
6. **Watchdog strategy** — which context pets the watchdog? Main loop is natural but adds "all the others starved main" as an undetectable failure. Motion task + main loop with independent windows, both required → kicks in if *either* context is starved.
7. **Shared timer resources** — current-loop ISR uses a PWM timer. Motion task and supervisor need a 1 kHz and 20 Hz tick each. Budget the available timer instances on each MCU and assign.

---

## Appendix A — Quick Reference

### Contexts
| context | rate | what it does |
|---|---|---|
| Current-loop ISR | 10/20 kHz | current PI, ADC read, PWM write |
| HW fault ISR | async | snapshot, flag, safe state |
| Motion task | 1 kHz | velocity PI, profile, detection, plausibility |
| Supervisor | 20 Hz | engine tick, Welford, orchestration |
| Main loop | idle | NVM, housekeeping |
| USB CDC ISR | async | bytes in/out of rings |

### ADC sequence
1. I_SENSE (always first, critical path)
2. V_BUS
3. TEMP (SAMD21 only in scan, RA4M1 uses Group B)
4. IPROPI (SAMD21 in scan, RA4M1 in Group B)

### Priorities (highest to lowest)
Current-loop ISR → HW fault ISR → Motion task → USB CDC ISR → Supervisor → Main loop

### Engine candidates
- ChainTree C engine (behavior tree, FNV-1a hash dispatch, `.ctb` images)
- S-expression C engine (quad instructions, `.sexb` images, function dictionary)

### Rates summary
- PWM / current-loop ISR: 20 kHz (RA4M1), 10 kHz (SAMD21)
- Motion task: 1 kHz
- Supervisor / engine: 20 Hz
- USB CDC telemetry: 100 Hz periodic, 1 kHz burst during anomaly
- Heartbeat: 2-4 Hz
- Fleet telemetry: 0.1-1 Hz

### Firmware command/telemetry interface
- Commands: `move_to`, `move_rel`, `stop`, `cal_find_stall`, `set_threshold`, `set_threshold_block`, `set_pi_gains`, `set_profile_params`, `get_status`
- Events: `stall_detected`, `trip_flag_raised`, `move_complete`, `move_aborted`, `fault(code)`, `heartbeat_response`

