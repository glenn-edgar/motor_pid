# Windshield Demo Bench — Pin Assignments and Wiring

**Status:** Draft v0.3, demo bench specification
**Supersedes:** v0.1, v0.2
**Companion to:** `windshield.md` (control architecture), `windshield_hardware.md` (component rationale)
**Scope:** Breakout-board-based demo bench for both XIAO tiers. Proves the control architecture end-to-end. Production variant (custom PCB, SPI diagnostics, dual-path safety) is a separate deliverable.

---

## 1. Demo Scope and Assumptions

This is a **demonstration bench**, not a production unit. The intent is to prove the control architecture, firmware, and ChainTree integration on real hardware, with the understanding that the production PCB will have a different pin budget (more pins exposed) and different safety architecture (independent hardware paths).

Assumptions committed for this demo:

- **Two XIAO platforms**: SAMD21 (basic tier) and RA4M1 (premium tier).
- **Breakout boards** for current sense and H-bridge. No hand-laid analog board, no custom PCB at this stage.
- **ChainTree shim over USB CDC** via the native USB-C connector on the XIAO. Zero GPIO consumed for shim transport.
- **RA4M1 front-header pins only** — no reliance on back-side castellated pads. Production will have access to more pins.
- **DRV8873H-Q1** (no SPI) on the premium tier. SPI register-level bridge diagnostics is a production feature, not demo.
- **Bench power supply** with current limit as motor supply. 12 V, 5 A limit is typical.

What the demo proves:

- Cascaded current → velocity → position control at the specified loop rates (10 kHz / 1 kHz / 100 Hz on SAMD21, 20 kHz / 1 kHz / 100 Hz on RA4M1)
- PWM-synchronous ADC current sampling
- Hardware encoder decode (RA4M1) and software EIC decode (SAMD21)
- Hardware-latched PWM cutoff on fault
- Trapezoidal profile (SAMD21) and S-curve profile (RA4M1)
- Anti-pinch detection with two-signal (Δi AND dω/dt) and three-signal (add DOB residual) voting
- Disturbance observer on RA4M1
- ChainTree shim protocol over USB CDC
- Welford baseline learning for i_nominal(position)

What the demo does **not** prove:

- Production-grade EMC
- Automotive load-dump survival
- Dual-path hardware safety (the comparator path shares the INA240 with the control path on breakouts)
- SPI-based bridge diagnostics
- Final BOM cost

---

## 2. Components

### 2.1 XIAOs

- **XIAO SAMD21** — basic tier, 48 MHz Cortex-M0+, no FPU, 12-bit ADC, software quadrature decode via EIC
- **XIAO RA4M1** — premium tier, 48 MHz Cortex-M4 with FPU, 14-bit ADC, hardware quadrature via GPT

### 2.2 Current-sense breakout

**Adafruit INA240 High or Low Side Voltage Output Current Sensor** (PID 4883).

- INA240A2 onboard (gain 50 V/V)
- 2 mΩ shunt integrated
- 100 mV/A at output
- Bidirectional output centered at Vs/2 (1.65 V with Vs = 3.3 V)
- Vs = 3–5.5 V; drives from XIAO 3V3 rail
- Terminal blocks for in-line shunt connection to motor loop
- Single analog output (VIOUT) pin to MCU ADC

Full-scale math:
- At 0 A: VIOUT = 1.65 V
- At +10 A: VIOUT = 1.65 + 1.0 = 2.65 V
- At −10 A: VIOUT = 1.65 − 1.0 = 0.65 V
- At +15 A: VIOUT = 3.15 V (near rail — approaching clip)
- Useful range: ±13 A with 0.2 V headroom either side

### 2.3 H-bridge breakout

**Pololu DRV8873-Q1 Single Brushed DC Motor Driver Carrier** (specifically the non-SPI / "H" variant).

- DRV8873H-Q1 onboard
- 4.5–38 V motor supply
- 10 A peak, ~6 A continuous (depending on cooling)
- Inputs: IN1, IN2 (dual-PWM interface)
- Outputs: OUT1, OUT2 to motor
- nSLEEP: active-low enable
- nFAULT: open-drain fault output (needs external pull-up or MCU internal)
- IPROPI: analog current-sense output (not used in demo — INA240 is the control-loop path; IPROPI wired to an ADC input if spare pin available for redundant cross-check)
- Onboard: reverse-polarity protection, bus decoupling, logic-level pull-ups

### 2.4 Additional bench parts

| item | purpose | note |
|---|---|---|
| 2× 100 kΩ + 2× 10 kΩ resistors (1%) | V_BUS voltage divider | 11:1 division, divide 36 V → 3.0 V ADC max |
| 1× 100 nF ceramic | V_BUS divider filter cap | at ADC pin |
| 1× 10 μF electrolytic | INA240 Vs bulk decoupling | transient supply for breakout |
| 1× 10 kΩ | nFAULT pull-up | if MCU internal pull-up not used |
| Jumper wires, headers, breadboard | bench wiring | short as possible for analog |
| Motor + encoder | test motor | Pittman GM8000-class + quadrature encoder |
| Bench supply | 12 V, 5 A current-limited | Rigol DP832 or equivalent |
| DMM and oscilloscope | bring-up diagnostics | scope must have >100 MHz BW for PWM edges |

### 2.5 What's intentionally omitted vs production

- **Independent TLV3201 comparator** tapping raw shunt: impossible on breakout, shunt is internal to the INA240 module. OC path is via the firmware comparison in the current-loop ISR (fast, but not hardware-independent of the amp).
- **Load-dump TVS** (SMAJ33A): not needed on bench supply. Required in vehicle harness.
- **Common-mode choke** for EMC: not needed on bench.
- **Galvanic isolation** of any kind: not needed on bench.
- **Any automotive-qualified parts specifically**: demo uses commercial-grade parts.

---

## 3. XIAO SAMD21 — Basic Tier Demo

### 3.1 Pin assignment

| XIAO pin | chip port | function | wire to |
|---|---|---|---|
| 5V | — | power | bench 5 V or USB only |
| 3V3 | — | power out | INA240 breakout Vs |
| GND | — | ground | star-ground node (see §5) |
| D0 | PA02 | spare / LED_STATUS | optional LED to GND via 470 Ω |
| D1 | PA04 | **I_SENSE** (ADC AIN4) | INA240 breakout VIOUT |
| D2 | PA10 | **PWM_IN1** (TCC0/WO[2]) | DRV8873H breakout IN1 |
| D3 | PA11 | **PWM_IN2** (TCC0/WO[3]) | DRV8873H breakout IN2 |
| D4 | PA08 | **V_BUS_SENSE** (ADC AIN16) | junction of V_BUS divider |
| D5 | PA09 | **BRIDGE_nSLEEP** (GPIO) | DRV8873H breakout nSLEEP |
| D6 | PB08 | **ENCODER_A** (EIC EXTINT[8]) | encoder A |
| D7 | PB09 | **ENCODER_B** (EIC EXTINT[9]) | encoder B |
| D8 | PA07 | spare / debug UART TX | |
| D9 | PA05 | spare / debug UART RX | |
| D10 | PA06 | **nFAULT_IN** (EIC EXTINT[6] → TCC0 FAULT_A) | DRV8873H breakout nFAULT |
| USB-C | — | **ChainTree shim (USB CDC)** | USB to supervisor host |

Signal count: 8. Spare: 3 (D0, D8, D9). Headroom for bring-up.

### 3.2 SAMD21 diagram

```
                    ┌────────────────────────────┐
                    │  USB-C ◄───► ChainTree     │
                    │         (USB CDC shim)     │
                    │                            │
         LED/spare ◄┤ D0       XIAO        D10  ├──► nFAULT ← DRV8873H
          I_SENSE ◄─┤ D1      SAMD21        D9  ├──► spare
          PWM_IN1 ◄─┤ D2                    D8  ├──► spare
          PWM_IN2 ◄─┤ D3                    D7  ├──► ENCODER_B
       V_BUS_SENSE ◄┤ D4                    D6  ├──► ENCODER_A
       BRIDGE_nSLP ►┤ D5                   3V3  ├──► INA240 Vs
                    │                      GND  ├──► star ground
                    │                       5V  │
                    └────────────────────────────┘
```

### 3.3 SAMD21 peripheral configuration

- **TCC0**: center-aligned PWM at 20 kHz. Channels 2 and 3 driving D2 and D3. Dual-slope mode. FAULT_A input from EXTINT[6] via EVSYS routing.
- **ADC0**: triggered by TCC0 overflow event via EVSYS. Sequence: AIN4 (I_SENSE) → AIN16 (V_BUS_SENSE). 12-bit. EOC IRQ at 10 kHz (every other PWM period).
- **EIC**: EXTINT[8] and EXTINT[9] on D6/D7, both-edge interrupt for encoder. EXTINT[6] on D10, falling-edge for nFAULT, also routed to EVSYS.
- **EVSYS**: channel 0 = TCC0 overflow → ADC START. Channel 1 = EXTINT[6] → TCC0 FAULT_A (hardware PWM disable).
- **USB**: CDC-ACM device class for shim. Runs on PA24/PA25 (dedicated USB pins, no GPIO impact).
- **NVIC priorities**: ADC EOC IRQ = 0 (highest), TC for 1 kHz tick = 1, EIC encoder = 1, USB = 3, main loop = lowest.

---

## 4. XIAO RA4M1 — Premium Tier Demo

### 4.1 Pin assignment

| XIAO pin | chip port | function | wire to |
|---|---|---|---|
| 5V | — | power | bench 5 V or USB only |
| 3V3 | — | power out | INA240 breakout Vs |
| GND | — | ground | star-ground node |
| D0 | P014 | **I_SENSE** (ADC AN09) | INA240 breakout VIOUT |
| D1 | P000 | **V_BUS_SENSE** (ADC AN00) | junction of V_BUS divider |
| D2 | P001 | **PWM_IN1** (GPT channel A) | DRV8873H breakout IN1 |
| D3 | P002 | **PWM_IN2** (GPT channel B) | DRV8873H breakout IN2 |
| D4 | P206 | **BRIDGE_nSLEEP** (GPIO) | DRV8873H breakout nSLEEP |
| D5 | P100 | **nFAULT_IN** (GPIO IRQ → POEG) | DRV8873H breakout nFAULT |
| D6 | P302 | **ENCODER_A** (GPT encoder A) | encoder A |
| D7 | P301 | **ENCODER_B** (GPT encoder B) | encoder B |
| D8 | P111 | **BRIDGE_IPROPI** (ADC) | DRV8873H breakout IPROPI (redundant I) |
| D9 | P110 | spare / debug UART | |
| D10 | P109 | spare / debug UART | |
| USB-C | — | **ChainTree shim (USB CDC)** | USB to supervisor host |

Signal count: 9. Spare: 2 (D9, D10).

### 4.2 RA4M1 diagram

```
                    ┌────────────────────────────┐
                    │  USB-C ◄───► ChainTree     │
                    │         (USB CDC shim)     │
                    │                            │
          I_SENSE ◄─┤ D0      XIAO         D10  ├──► spare
       V_BUS_SENSE ◄┤ D1     RA4M1          D9  ├──► spare
          PWM_IN1 ◄─┤ D2                    D8  ├──► IPROPI ← DRV8873H
          PWM_IN2 ◄─┤ D3                    D7  ├──► ENCODER_B
       BRIDGE_nSLP ►┤ D4                    D6  ├──► ENCODER_A
       nFAULT_IN ◄──┤ D5                   3V3  ├──► INA240 Vs
                    │                      GND  ├──► star ground
                    │                       5V  │
                    └────────────────────────────┘
```

### 4.3 RA4M1 peripheral configuration

- **GPT3** (or the GPT instance tied to P001/P002): center-aligned PWM at 20 kHz, complementary outputs with ~400 ns dead-time (register-programmable).
- **GPT4 or other instance at P302/P301**: encoder (phase-counting) mode, 16-bit counter with overflow extension to 32-bit in software.
- **ADC14**: sequence AN09 (I_SENSE) → AN00 (V_BUS_SENSE) → IPROPI on D8. Triggered by ELC event from GPT3 compare-match at mid-PWM. 14-bit. EOC IRQ at 20 kHz.
- **ELC**: links GPT3 event → ADC start, links D5 GPIO IRQ → POEG (Port Output Enable for GPT), which disables GPT3 outputs in hardware on fault.
- **POEG**: configured to latch GPT3 outputs to safe state on nFAULT_IN assertion.
- **USB**: CDC-ACM device class for shim.
- **NVIC priorities**: ADC EOC = 0, GPT for 1 kHz = 1, encoder overflow = 2, USB = 3, main = lowest.

### 4.4 Peripheral verification needed before firmware

These need confirmation against the RA4M1 datasheet and the Seeed Arduino core `variant.cpp`:

1. GPT channel available on D2/D3 (P001/P002) with complementary PWM + dead-time.
2. GPT encoder-mode channel available on D6/D7 (P302/P301).
3. ELC event routing from a GPIO IRQ to POEG — confirm sequence works without software intervention.
4. ADC triggering by GPT compare-match event via ELC is straightforward per the RA4M1 manual.

If any of these don't map as assumed, reshuffle within front-header pins. There is flexibility: PWM can go on D4/D5 with some re-wiring, encoder can go on D8/D9 if needed.

---

## 5. Bench Wiring

### 5.1 Power topology

```
  Bench supply (12 V, 5 A limit)
           │
           ├──► DRV8873H breakout VM  ─► motor terminals via OUT1/OUT2
           │
           └──► V_BUS divider (100k/10k) ─► XIAO ADC (V_BUS_SENSE pin)

  XIAO 3V3 rail (from USB)
           │
           ├──► INA240 breakout Vs
           │         │
           │         └─ decoupling: 100 nF + 10 μF at INA240 Vs pin
           │
           └──► DRV8873H breakout logic supply if required
                (check breakout — many derive logic rail from VM)

  Grounds (critical):
     XIAO GND ─┐
     INA240 GND ├──► star node (single junction)
     DRV8873 GND┤
     supply GND ┘
```

**Star ground is not optional.** The INA240 is measuring 100 mV full-scale signals; a daisy-chained ground with 50 mΩ of wire resistance and 3 A of motor current will inject 150 mV of ground offset into the measurement. Run a single fat ground wire from the bench supply to a single binding post or terminal block; connect all three module grounds there individually.

### 5.2 Shunt placement

The INA240 breakout has screw terminals or solder pads for the two motor-side connections. Insert the INA240 in-line with one motor wire:

```
  DRV8873H OUT1 ───► INA240 (IN+) ───► INA240 (IN−) ───► motor terminal 1
  DRV8873H OUT2 ───────────────────────────────────────► motor terminal 2
```

Either output leg works. Convention: put it on OUT1 for consistency across the two boards. The INA240 measures current flowing IN+ to IN− as positive; this corresponds to "forward drive" with OUT1 high, OUT2 low. Sign convention in firmware must match.

### 5.3 V_BUS divider

```
  V_BUS (12 V nominal, max ~36 V including margin)
    │
    ├── R1 (100 kΩ, 1%)
    │
    ├── tap ──► 100 nF cap to GND ──► XIAO ADC (V_BUS_SENSE pin)
    │
    ├── R2 (10 kΩ, 1%)
    │
   GND
```

Division ratio: 10/(10+100) = 0.0909. At 12 V bus: 1.09 V at ADC pin. At 36 V bus: 3.27 V at ADC pin. Stays within 3.3 V ADC range.

**Current through divider**: 12 V / 110 kΩ ≈ 110 μA. Power: 1.3 mW. Negligible.

Tolerance note: 1% resistors give ~2% worst-case voltage error. Calibrate in firmware by measuring a known bus voltage at power-up during bench tests.

### 5.4 nFAULT wiring

DRV8873H nFAULT is open-drain. Connect with a pull-up (10 kΩ to 3.3 V) if the XIAO input doesn't provide internal pull-up in the firmware config. Both XIAOs' GPIOs can provide internal pull-ups — configure in firmware.

Route nFAULT to the MCU pin specified in the tier pinout (D10 on SAMD21, D5 on RA4M1). In firmware, configure as edge-triggered interrupt (falling edge for "fault asserted"), and route the same pin via EVSYS/ELC to the hardware PWM fault latch (TCC FAULT_A on SAMD21, POEG on RA4M1).

### 5.5 Encoder wiring

Typical quadrature encoder on a Pittman-class motor: 4-wire (Vcc, GND, A, B) or 5-wire (adds index). Vcc is usually 5 V for motor-shaft optical encoders.

- If encoder is 5 V open-collector: add pull-ups (10 kΩ) to 3.3 V. 5 V output does not reach the XIAO — the 3.3 V rail pulls the line up, and the encoder pulls it down. Works.
- If encoder is 5 V push-pull: **need a level shifter** (e.g., two-channel 74LVC1T45 or simple resistor divider). Do not connect 5 V push-pull directly to a 3.3 V GPIO.
- If encoder is 3.3 V push-pull: direct connection, possibly with pull-up for noise immunity.

Power the encoder from the bench 5 V rail (bring it out through USB or the XIAO 5V pin if sourced from USB), not from the 3.3 V rail.

Keep encoder wires short and away from motor wires. Twisted pair for A/B if the demo bench extends beyond 30 cm of wiring.

---

## 6. Firmware Current Scaling

Because the INA240 breakout uses a 2 mΩ shunt and gain 50 V/V (Adafruit's A2 variant), the scaling differs from the 10 mΩ × 20 V/V pair originally specified. Updated math:

Output sensitivity: 2 mΩ × 50 V/V = **100 mV/A**.

**SAMD21 (12-bit ADC, Vref = 3.3 V):**
- ADC LSB = 0.806 mV
- Counts per amp = 100 mV/A / 0.806 mV/count = **124 counts/A**
- Resolution = 8 mA/LSB
- Useful full-scale (headroom both sides of Vs/2) = ±13 A

**RA4M1 (14-bit ADC, Vref = 3.3 V):**
- ADC LSB = 0.201 mV
- Counts per amp = **497 counts/A**
- Resolution = 2 mA/LSB
- Same full-scale = ±13 A

Both give adequate resolution for Pittman GM8000-class currents (cruise ~1 A, stall ~5 A). The RA4M1's higher resolution is useful for the disturbance observer but not essential.

Offset calibration procedure unchanged from `windshield.md` §6: at power-up, with bridge disabled, take 256 ADC samples and average. Subtract from all subsequent readings. Expected zero point: 2048 counts (SAMD21) / 8192 counts (RA4M1), ± ADC and amp offset.

---

## 7. Demo Bring-Up Sequence

Order matters. Do not skip ahead.

1. **Unpowered continuity checks.** With no power anywhere, DMM-verify: every GPIO from XIAO header to the correct breakout pin; star-ground continuity; divider resistor values; no shorts between Vs/Vm/GND.

2. **XIAO alone, USB only.** Flash a blinky. Verify USB CDC enumerates on host. No motor supply yet.

3. **INA240 breakout powered, no motor supply.** Verify VIOUT reads ~1.65 V at zero current (INA240 breakout has no current flowing through the shunt because motor supply is off). Capture offset in firmware.

4. **Motor supply energized, bridge nSLEEP held low.** Current should be zero. VIOUT should still read 1.65 V. V_BUS_SENSE should read 12 V × 0.0909 = 1.09 V. Confirm both in firmware telemetry.

5. **Enable bridge, command 0% PWM.** Motor does not spin. Bridge state is "both half-bridges passive." Current ≈ zero, V_BUS_SENSE steady.

6. **Command 10% PWM forward, motor unloaded.** Motor spins slowly. Current reads ~0.2 A (no load). Encoder counts up. If count goes wrong direction, swap A/B in firmware (or swap encoder wires).

7. **Command step changes in PWM duty.** Observe current loop behavior. Tune current PI gains from the analytic starting point (Kp = L × ω_cl, Ki = R × ω_cl).

8. **Close outer velocity loop.** Tune via step response at fixed velocity setpoints.

9. **Close profile executor.** Command trapezoidal moves (SAMD21) or S-curve moves (RA4M1). Verify tracking.

10. **Enable supervisor, anti-pinch detector.** Manually stall the motor by hand; verify fault triggers and reverse or stop executes.

11. **Enable ChainTree shim over USB CDC.** Verify bidirectional protocol works, heartbeat timeout triggers safe-state.

Each step should succeed before moving to the next. If the current reading has visible PWM ripple after step 3, the ground routing is wrong — fix before proceeding.

---

## 8. Known Limitations of Demo vs Production

Recorded explicitly so they're not forgotten during production handoff:

| concern | demo | production |
|---|---|---|
| independent hardware OC path | INA240 output → firmware comparison only | separate comparator on raw shunt |
| bridge forensics | nFAULT signal only | SPI registers (DRV8873S) |
| EMC | not qualified | CISPR 25 qualified |
| load-dump | bench supply only | SMAJ33A TVS |
| reverse polarity | bench assumption | P-FET |
| automotive qualification | none | AEC-Q100 parts throughout |
| shim transport | USB CDC only | USB CDC + fallback UART + optional CAN |
| board | breakouts on breadboard | custom PCB with proper layout |

---

## 9. Open Items

1. **Confirm Adafruit INA240 PID 4883 is the A2 variant (gain 50).** If it's a different gain variant, update the scaling math in §6.
2. **Confirm Pololu DRV8873H breakout logic-level compatibility** with 3.3 V inputs from XIAO. Standard for Pololu, but verify from the product page.
3. **Encoder part number** for the demo motor. Determines CPR, interface level, and wiring.
4. **RA4M1 peripheral mapping verification** per §4.4.
5. **Motor selection from Pittman GM8000 family.** Determines Kt, L, R for initial current-loop gains.

---

## Appendix A — Quick-Reference Summary

| | SAMD21 basic | RA4M1 premium |
|---|---|---|
| I_SENSE pin | D1 | D0 |
| V_BUS_SENSE pin | D4 | D1 |
| PWM_IN1 pin | D2 | D2 |
| PWM_IN2 pin | D3 | D3 |
| nSLEEP pin | D5 | D4 |
| nFAULT pin | D10 | D5 |
| ENCODER_A pin | D6 | D6 |
| ENCODER_B pin | D7 | D7 |
| IPROPI (redundant I) | — | D8 |
| shim | USB CDC | USB CDC |
| current-loop rate | 10 kHz | 20 kHz |
| ADC resolution | 12-bit | 14-bit |
| quadrature decode | EIC software | GPT hardware |
| PWM cutoff mechanism | TCC FAULT_A via EVSYS | POEG via ELC |
| profile | trapezoidal | S-curve |
| bridge | DRV8873H breakout | DRV8873H breakout |
| current sense | INA240 breakout (100 mV/A) | INA240 breakout (100 mV/A) |
| spare pins | D0, D8, D9 | D9, D10 |
