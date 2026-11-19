# TSat-3A reaction wheel: simulation results

Generated 2026-11-17 by the scripts in `sim/` (see [`../README.md`](../README.md) for models and
assumptions). Full Monte Carlo tables: [`yaw_tables.md`](yaw_tables.md). Raw data:
`pcbmotor_sweep.csv`, `pcbmotor_designs.json`, `yaw_summary.csv`, `yaw_requirements.json`,
`tsat1_swing.json`, `video_spin_tsat2b_full.*`, `video_phases_tsat2b_full.*`, `yaw_calibration.json`.

## Conclusions

Updated 2026-11-17 after calibrating the disturbance model to the TSat-2B onboard flight video
(section 2b). The video showed far more spin than the original "medium" assumption, so the
requirements and the recommended payload design changed.

1. **Put a swivel or slip ring at the payload attachment.** TSat-2B, flown without one, spun at
   172 °/s RMS (median 15 rpm, 1 s peaks up to 128 rpm) and the camera was steady (under 2 °/s) only
   1.5% of the time. With the disturbance calibrated to that flight, every candidate (and even an ideal
   actuator) fails without a swivel: holding heading against the line would take 80–340 mN·m of torque
   and far more stored momentum than any wheel has.
2. **A PCB motor can be the main reaction-wheel motor, but it needs more stored momentum than the
   first payload design.**
   - Recommended: the high-momentum payload variant (JLCPCB 4-layer, 2 oz outer / 1 oz inner, 6 coils /
     8 poles, 19.05 × 12.7 × 6.35 mm magnets, 84 mm × 3 mm steel disc). It stores 90 mN·m·s and held
     the camera steady 99–100% of the time behind a swivel or slip ring in the line-driven case,
     and 77–83% in the gust-driven worst case. It was the best real candidate in every case.
   - The first payload design (12 coils / 16 poles, 1.5 mm disc, 37 mN·m·s) reaches 96–99% median but
     saturates in 2–4 of 8 runs. It is the lighter fallback (117 g against 223 g rotating).
   - The 40 mm test stator is too small to fly. It exists to check the model on the bench.
   - The motor model reproduces the one measured PCB reaction wheel we could check (CMU, SmallSat
     2024) within about 10% on torque constant and 11% on resistance (section 1).
3. **Momentum and control bandwidth are the hard requirements, not raw torque.**
   - With a swivel: at least 6 mN·m peak torque and 60 mN·m·s of usable momentum each way
     (line-driven case); 15 mN·m and about 130 mN·m·s if the spin is mostly aerodynamic (gust-driven).
   - The attitude loop must run at 100 Hz with about 20 rad/s bandwidth. At 50 Hz the loop goes
     unstable above about 15 rad/s, and an 8 rad/s loop leaves the camera unsteady about a quarter of
     the time even with a perfect actuator.
   - Smooth sensored control through zero speed. The sensorless Maxon EC32 flat failed in every case.
4. **Use the cheap fab for the payload board.** JLCPCB's 1 oz inner layers, against OSH Park's 0.5 oz,
   cut resistance enough to raise torque per √watt (Km²) by 50–67% over OSH Park for the same copper
   pattern.
5. **Keep the 1U body aerodynamically symmetric about the yaw axis.** If TSat-2B's spin was mostly
   wind torque rather than line twist, a swivel helps less and the momentum demand doubles; a centred
   tether attachment and no one-sided protrusions reduce that torque at the source.

## 1. PCB motor designs

### Recommended designs

| | Test stator | Payload stator (recommended) | Payload, OSH Park 6L (comparison) |
|---|---|---|---|
| Name | `test_4L_6c8p_w0.15_m8.0x3.0` | `payload_jlc4_2oz_out_12c16p_w1.2_h3.18` | `payload_osh6_9c12p_w1.2_h3.18` |
| Fab / stackup | OSH Park 4-layer (published stackup: 1.7 mil plated outer, 0.5 oz inner, 39 mil core) | JLCPCB 4-layer, 2 oz outer / 1 oz inner | OSH Park 6-layer |
| Board | 42 mm round | 90 × 90 mm, winding 42–82 mm diameter | same as payload |
| Coils / poles | 6 / 8 | 12 / 16 | 9 / 12 |
| Trace / space | 0.15 / 0.15 mm, 20 turns per layer per coil | 1.2 / 0.15 mm, 5 turns per layer per coil | 1.2 / 0.15 mm, 7 turns |
| Magnets | 8 × (8 mm dia × 3 mm) N42 discs | 16 × (19.05 × 6.35 × 3.18 mm) N42 blocks | 12 × (19.05 × 9.53 × 3.18 mm) |
| Rotor | 1.5 mm steel back plate, 40 mm | 84 mm × 1.5 mm steel disc (back iron and flywheel) | same as payload |
| Air gap | 0.5 mm | 0.6 mm | 0.6 mm |
| Kt (per A peak, FOC) | 10.5 mN·m/A | 26.7 mN·m/A | 38.5 mN·m/A |
| Phase R / L | 19.3 Ω / 169 µH | 1.14 Ω / 95 µH | 3.99 Ω / 228 µH |
| Km (torque per √W) | 1.95 mN·m/√W | **20.4 mN·m/√W** | 15.7 mN·m/√W |
| Torque at 11.1 V | 3.5 mN·m peak (0.33 A, voltage-limited); **2.06 mN·m continuous** (0.20 A, 1.1 W, 40 K rise) | 26.7 mN·m at the 1 A driver limit, 1.7 W, 11 K rise | 38.5 mN·m at 1 A, 6.0 W, 37 K rise |
| No-load speed at 11.1 V | 8,700 rpm (capped at 6,000 for a hand-balanced rotor) | 3,440 rpm | 2,380 rpm |
| Rotor inertia / mass | 4.4×10⁻⁶ kg·m² / 24 g | 1.02×10⁻⁴ kg·m² / 111 g (+5.6 g copper on the board) | 1.07×10⁻⁴ / 117 g |
| Momentum at max speed | 2.8 mN·m·s | **36.7 mN·m·s** | 26.8 mN·m·s |
| Torque ripple (ideal sinusoidal drive) | 0.7% | 2.4% | 1.6% |
| PWM ripple current p-p at 20 / 50 / 100 kHz | 0.41 / 0.16 / 0.08 A | 0.73 / 0.29 / 0.15 A | 0.31 / 0.12 / 0.06 A |

Notes on the choices:
- **Trace width barely changes Km.** It trades torque per amp against speed: fewer, wider turns mean
  more speed and less Kt at the same copper loss.
- **The payload pick maximizes stored momentum** among cheap-fab designs within 10% of the best Km,
  because torque is never the limit (section 4). The highest-Km cheap option is JLC 6-layer 1 oz
  12/16 (21.5 mN·m/√W), but it reaches only 2,310 rpm (24.7 mN·m·s). The JLC 4-layer board with 2 oz
  outer copper stores 49% more momentum for 5% less Km and fewer layers.
- **Test stator.** Run it with ≥100 kHz PWM or a 5 V bench supply: at 50 kHz and 11.1 V the ripple
  (0.16 A) is comparable to its 0.20 A continuous current. Its 40 K continuous limit with natural
  convection is only 1.1 W, so log board temperature during tests.
- **Payload stator.** 50 kHz PWM is enough. Electrical time constant is 84 µs.

Plots: `pcbmotor_test_layout.png`, `pcbmotor_test_waveforms.png`, `pcbmotor_payload_layout.png`,
`pcbmotor_payload_waveforms.png`, `pcbmotor_payload_osh6_*.png`.

### OSH Park vs JLCPCB at the same copper pattern

12 coils / 16 poles, 1.2 mm traces, 3.18 mm magnets, 42–82 mm winding. Km² is torque² per watt of
copper loss.

| Stackup | Copper (outer/inner) | Kt [mN·m/A] | R phase [Ω] | Km [mN·m/√W] | Km² vs OSH Park 6L | Km² vs OSH Park 4L |
|---|---|---|---|---|---|---|
| OSH Park 4L | 1.24 (plated) / 0.5 oz | 27.1 | 2.11 | 15.2 | 0.83× | 1.00× |
| OSH Park 6L | 1 / 0.5 oz (assumed) | 39.7 | 3.78 | 16.7 | 1.00× | 1.20× |
| JLCPCB 4L | 1 / 1 oz | 26.7 | 1.51 | 17.7 | 1.13× | **1.36×** |
| JLCPCB 6L | 1 / 1 oz | 39.7 | 2.27 | 21.5 | **1.67×** | 2.01× |
| JLCPCB 4L | 2 / 1 oz | 26.7 | 1.14 | 20.4 | **1.50×** | 1.81× |

JLCPCB 4-layer with 1 oz inner copper has 28% less phase resistance than OSH Park 4-layer (1.51 against
2.11 Ω) and 36% more Km² for the same copper pattern, even though OSH Park's plated outer copper is
thicker (1.7 mil, from OSH Park's published stackup, checked 2026-11-18). Kt differs by 1.5% between them
because OSH Park's 39 mil core puts its bottom layers closer to the magnets than the assumed JLC
spacing. The OSH Park 6-layer row still assumes bare 1 oz outer copper and even layer spacing. JLC layer spacing is assumed (JLC doesn't publish its prepreg/core thicknesses),
but it mostly affects Kt, not the resistance gain.

### Sanity checks against real hardware

| | Km [mN·m/√W] | Stall torque | Speed | Mass |
|---|---|---|---|---|
| Faulhaber 2610T012B (datasheet) | 3.3 | 7.45 mN·m at 12 V | 6,400 rpm no-load | 20 g motor |
| Maxon EC32 flat 226006 (datasheet) | 13.2 | 85 mN·m at 24 V | 4,530 rpm at 24 V | 57 g motor |
| CMU PCB reaction wheel (measured, see below) | 7.1 | 8 mN·m (extrapolated; data to 6 mN·m) | 1,600 rpm at 8 V | 140 g, 9 mm thick, 1U |
| This model, 40 mm test stator | 1.95 | 3.5 mN·m (2.06 continuous) | 8,700 rpm no-load | 24 g rotor |
| This model, 1U payload stator (12 coils) | 20.4 | 26.7 mN·m at 1 A | 3,440 rpm | 111 g rotor + board |
| This model, high-momentum payload stator | 16.1 | 21.7 mN·m at 1 A | 4,220 rpm | 223 g rotor + board |

### Model validation against a built PCB motor (CMU wheel)

`validation/cmu_wheel.py` rebuilds the CMU PCB reaction wheel (Tayal, Fisch, Manchester, SmallSat 2024,
doi:10.26077/sr9e-tc61) from its paper and open design files (github.com/SaralTayal123/PCB_Motors:
coil generator notebook, KiCad board, firmware) and runs the same magpylib Lorentz-force method.

- **Their design:** 36 slots and 12 poles (firmware `PP = 6`; the paper's table says "12 pole-pair",
  but the firmware and README say 12 poles). Two PCB stators, one on each side of the rotor, in
  series. Wave winding, 10 turns, each radial conductor stitched across all 4 layers (1 oz, 1.2 mm
  board), coil diameter 53–79 mm, 0.29 mm traces. 12 × (12.7 × 6.35 × 3.18 mm) N42 magnets on a
  3 mm aluminium rotor, so no back iron. PTFE bushings, Hall sensors, ESP32-S3 + TMC6200 + SimpleFOC.
- **Measured:** phase resistance 5.5 Ω. Torque constant from their Fig. 11: for a constant-voltage
  drive the speed–torque slope is 1.5 R / Kt², independent of supply voltage. The best-fit line
  falls from 171 rad/s at 0 mN·m to 13 rad/s at 8 mN·m, giving **Kt ≈ 20.4 mN·m/A and Km ≈ 7.1
  mN·m/√W** (values read by eye from the figure). Their 8 mN·m stall is an extrapolation; measured
  data reach about 6 mN·m.
- **Model:** phase resistance 6.1 Ω (+11%). Kt depends on the air gap, which the paper doesn't give:

| Air gap (magnet face to copper) | 0.3 mm | 0.5 mm | 0.75 mm | 1.0 mm | 1.5 mm |
|---|---|---|---|---|---|
| Model Kt [mN·m/A] | 28.1 | 26.5 | 24.6 | 22.9 | 20.0 |
| Model Km [mN·m/√W] | 9.3 | 8.7 | 8.1 | 7.6 | 6.6 |

For a plausible 1–1.5 mm gap (PTFE bushing flange plus mounting), the model matches the measured
torque constant within about 10% and resistance within 11%. The earlier "about 6× CMU" comparison used
a lower bound (8 mN·m at 5 W total input); the payload designs are about 2.3–2.9× CMU's measured Km,
mostly because they use steel back iron behind the magnets, which CMU did not. The back-iron part of
the model (method of images) is not covered by this check; the test stator still has to confirm it.

**Winding lesson.** CMU and the CIOMP group (Wang et al., Optics and Precision Engineering 2015,
doi:10.3788/OPE.20152304.1004) both use wave windings instead of spiral coils. CIOMP measured 26.2%
lower motor power at 5,000 rpm with the wave winding than with a spiral winding on otherwise identical
flywheel prototypes. The payload designs here use spiral (concentrated) coils; a wave-winding variant
is worth adding to the design study.

### Test stator as laid out (PCBMotorTest v0.1, 2026-11-18)

`PCBMotorTest/gen_stator.py` lays the test design out as real 4-layer spirals (`pcbmotor/layout.py`), and
`pcbmotor/asbuilt.py` runs the same EM model on that copper (`pcbmotor_test_asbuilt.json`). The two
centre vias need room, so the coil has 18 turn rings instead of 20. The turn edges are parallel to the
sector boundary rather than radial, so the outer turns are wider. Net: Kt 11.5 mN·m/A (+9% over the
sweep), phase R 19.6–19.8 Ω including the ring tracks (+2%), Km 2.1 mN·m/√W. Predicted phase back-EMF:
0.80 V peak per 1000 rpm with the steel back plate, 0.61 without (0.5 mm gap). These use OSH Park's
published 4-layer stackup (updated 2026-11-18: the 39 mil core puts the bottom layers 0.17 mm closer to
the magnets than the old 1.2 mm guess, and the 1.7 mil plated outer copper lowers resistance by 7%). That pair is the direct
bench test of the back-iron model. Full table: `PCBMotorTest/README.md`.

### Stacking several stator boards, and dual rotors (2026-11-18)

`pcbmotor/stacking.py` runs the same EM model on boards stacked on one side of the rotor (1.6 mm boards,
0.3 mm apart, phases in series or in parallel), on dual-rotor sandwiches (stator between two magnet
discs), and on the same copper on one board with more layers. Full tables:
[`pcbmotor_stacking.md`](pcbmotor_stacking.md). The study also adds **eddy-current drag in the traces**,
which the design sweep left out: a trace of width w in a changing axial field loses
σ·w²/12·(dB/dt)² per unit volume.

Recommended payload (high-momentum 6c/8p), 11.1 V, 1 A driver limit:

| Config | Km mN·m/√W | No-load rpm | Momentum, no drag | Momentum with eddy drag and 6 mN·m left for control | Eddy loss at top speed | Rotor + boards | Pull between rotors |
|---|---|---|---|---|---|---|---|
| Single board (as designed) | 16.2 | 4,215 | 90 | **80** | 3.4 W | 247 g | – |
| 2 boards, series | 19.7 | 2,444 | 52 | 46 | 1.4 W | 271 g | – |
| 2 boards, parallel | 19.7 | 4,898 | 105 | 82 | 5.8 W + 2.6 W circulating | 271 g | – |
| 3 boards, parallel | 21.0 | 5,627 | 120 | 40 | 8.2 W + 10.2 W circulating | 295 g | – |
| 1 board, 2 rotors | 32.3 | 2,107 | 77 | 71 | 3.0 W | 404 g | 530 N |
| 2 boards parallel, 2 rotors | 42.5 | 2,268 | 83 | 79 | 3.9 W | 428 g | 370 N |
| 6-layer board (JLC 1 oz) | 17.1 | 2,817 | 60 | 53 | 1.4 W | 247 g | – |

(momentum in mN·m·s; requirement 60 line-driven, 130 gust-driven)

1. **Stacking doesn't help the payload.** It raises Km, but torque isn't the limit: the single board already
   needs only 0.86 W of copper loss for the 15 mN·m gust-driven requirement. Stored momentum is the limit, and
   no stacked option stores more usable momentum than the single board.
   - Series boards (and the 6-layer board) raise the back-EMF, so the wheel tops out lower: 46–53 mN·m·s,
     below the 60 needed.
   - Parallel boards keep the speed, but the extra copper adds eddy drag, and the far board's lower back-EMF
     drives a current round the loop between boards. Two boards end level with one (82 against 80); three are
     worse.
   - A dual rotor doubles Km but halves the speed, adds a 157 g second rotor and pulls the rotors together with
     about 530 N (54 kgf). The bearings, the spacer through the stator and the board would carry that, and
     assembly would be hazardous.
2. **Eddy drag is real for the 1.2 mm payload traces.** The single board loses 0.8 W at 2,000 rpm and 3.4 W at
   4,200 rpm, costing about 11% of usable momentum (90 → 80 mN·m·s). It still meets the 60 mN·m·s line-driven
   requirement. Loss scales with trace width squared, so splitting each wide trace into narrower parallel
   strands would cut it sharply. Those strands need transposing between layers, or circulating currents
   replace the eddy currents. Eddy loss is negligible on the 0.15 mm test stator, so the bench won't measure it.
3. **Test stator: two cheap extra measurements.**
   - OSH Park makes 3 boards, so stacking two and measuring each board's back-EMF on its own checks how the
     field falls off with distance: the model gives about 0.47× for the second board.
   - The same v0.1 board between two rotors should show twice the back-EMF (dual rotor). That needs a second
     rotor and a frame that holds about 70 N.

## 2. Old flight data (TSat-1, June 2025)

Source: `~/Documents/tsat1launch/tsatlog3/data.csv` (same files in Drive under TSat-1 / Flight Data).

- **The log has no gyro or magnetometer, so yaw spin is not observable.** Gravity stays on the body
  axis whatever the yaw. The onboard photos are about 10 s apart, so yaw from images aliases above
  ~3 rpm. TSat-2A Drive data is altitude only. The TSat-0/TSat-1 flight videos on Drive (21 MB and
  160 MB) were too large for the Drive connector, so they weren't analysed. Frame-to-frame image
  registration on them is the best remaining source of past spin rates.
- **Sampling.** 20.0 Hz (393,802 rows over 19,704 s, from the flight computer's file timestamps).
- **Flight.** About 296 s from state 2: stepped reel-out to 124 m, about 40 s at the top, then a
  ~115 m drop in about 10 s (≈11 m/s average by baro). The baro's peak derivative of 29 m/s during the
  drop is not reliable.
- **Horizontal specific force.** 1.05 m/s² RMS over the hang, 0.49 m/s² RMS on the quiet top
  plateau. Large spikes come at each reel-out step and in the drop.
- **Tension (|a|) spectrum.** Peaks at 0.29 Hz. If that is a pendulum's 2× tension ripple, it means a
  7.0 s swing period (≈12 m effective pendulum) and up to 26° swing. Each reel-out step jerks the line,
  so treat the amplitude as an upper bound. More peaks near 2.4 Hz and 7–8 Hz are likely structural or
  rocking modes.
- **Used in the sim.** The 0.3 Hz bandwidth of the gust-torque model. Plot: `tsat1_flight_accel.png`.

## 2b. Measured spin: TSat-2B onboard flight video (March 28, 2026)

Source: the full 25.5 min onboard video (`~/KSC/T-Sat2B_Flight.mp4`, from the team's OneDrive) and
the 65 s meeting clip in the Discord export. The camera looks straight down, so rotation of the
ground image is the payload's yaw. `data/video_spin.py` matches ORB features between consecutive
frames and fits a rotation + translation + scale with RANSAC; `data/video_phases.py` cleans
single-frame glitches (0.35% of frames) and computes the statistics. Plots:
`video_spin_tsat2b_full.png`, `video_phases_tsat2b_full.png`.

| Whole flight (25.5 min, 44,511 frames, 99.9% tracked) | Value |
|---|---|
| Yaw rate RMS | 172 °/s |
| Median \|rate\| | 89 °/s (15 rpm) |
| 1 s average: 95th percentile / peak | 47 rpm / 128 rpm |
| Time with \|rate\| < 2 °/s (steady video) | 1.5% |
| Dominant oscillation period | 13 s (about 2 s components near the top) |
| Net rotation / range of cumulative yaw | −0.8 turns / 14.6 turns (+5 to −10) |

- The motion is mostly back-and-forth torsional swinging, not a steady spin.
- On flight day a member noted in Discord that the payload was secured with 2 lines instead of 4,
  which may have contributed. No swivel is mentioned.
- Caveats: TSat-2B was a 2U payload, so its inertia and wind area differ from TSat-3A's 1U. The video
  has a variable frame rate (29.1 fps average); the per-frame time base is approximate. The altitude
  proxy from image scale drifted and is not used.

**Calibration.** Two explanations reproduce the video equally well (uncontrolled, no swivel, 1U
inertia, tether periods 2–15 s; `yaw/calibrate.py`, `yaw_calibration.json`):

| Level | Mechanism | Calibrated value | Sim RMS / 1 s p95 |
|---|---|---|---|
| measured, line-driven | balloon-end yaw jitter through the line | σ = 2.36 rad, 8 s correlation; gusts 1 mN·m RMS | 172 °/s / 55 rpm |
| measured, gust-driven | wind torque on the body | 2.72 mN·m RMS at 0.3 Hz; line as "medium" | 172 °/s / 52 rpm |

A swivel removes the line-driven part but not the gust-driven part, so the two bracket the benefit of
the swivel. 2.7 mN·m RMS is above the ≈1 mN·m estimated for a symmetric 1U face, but plausible for
TSat-2B's larger, asymmetric 2U body.

## 3. Yaw simulation: candidate comparison

Controller: 100 Hz PID, 20 rad/s bandwidth (changed from 50 Hz / 8 rad/s after the video
calibration; see the tuning note below). Stable = |yaw rate| < 2 °/s and |heading error| < 10 °.
Median / worst of 8 runs of 600 s, runs that saturated in brackets. All levels and rigging cases are
in [`yaw_tables.md`](yaw_tables.md).

| Candidate | Stored momentum | Line-driven, no swivel | Line-driven, swivel | Line-driven, slip ring | Gust-driven, swivel | Gust-driven, slip ring |
|---|---|---|---|---|---|---|
| Faulhaber 2610 B + 55 mm steel wheel | 21.6 mN·m·s | 0 / 0 (8/8) | 56 / 18 (7/8) | 64 / 15 (5/8) | 17 / 1 (8/8) | 9 / 0 (8/8) |
| Maxon EC32 flat + 55 mm wheel (sensorless) | 8.3 | 0 / 0 (8/8) | 0 / 0 (8/8) | 0 / 0 (8/8) | 0 / 0 (8/8) | 0 / 0 (8/8) |
| mj5208 + moteus (approx. parameters) | 29 | 0 / 0 (8/8) | 77 / 42 (7/8) | 92 / 37 (5/8) | 59 / 9 (8/8) | 25 / 3 (8/8) |
| PCB test stator, 40 mm | 2.8 | 0 / 0 (8/8) | 0 / 0 (8/8) | 3 / 0 (8/8) | 0 / 0 (8/8) | 0 / 0 (8/8) |
| PCB payload, 12c/16p, 1.5 mm disc | 36.7 | 0 / 0 (8/8) | 99 / 55 (4/8) | 96 / 57 (2/8) | 65 / 13 (8/8) | 34 / 5 (7/8) |
| **PCB payload, high momentum, 6c/8p, 3 mm disc** | **90** | 0 / 0 (8/8) | **100 / 99 (0/8)** | **99 / 97 (0/8)** | **83 / 47 (2/8)** | **77 / 51 (3/8)** |
| Ideal actuator | unlimited | 66 | 100 | 100 | 87 | 85 |

- **Controller tuning.** With the measured disturbance, an ideal actuator behind a swivel was stable
  only 74% of the time at the old 8 rad/s / 50 Hz loop. Raising the bandwidth to 15 rad/s gives 98%,
  but at 50 Hz the loop goes unstable at 25 rad/s. At 100 Hz, 20 rad/s gives 100% (line-driven) with
  margin. The ESP32 can run a 100 Hz loop easily.
- **Power stays small** for the motors that work: 0.2–0.4 W mean, about 0.1–0.2 Wh per 30 min.
  Peaks reach about 10 W for the PCB payload stators in the gust-driven case.
- **Gust step.** After a 3 mN·m, 2 s gust behind a swivel, the Faulhaber, mj5208 and both PCB
  payload stators stay within 1.3 °/s; the Maxon and the test stator do not recover.
- The older low / medium / high levels are still in `yaw_tables.md` for reference; all working
  candidates now hold 100% in the medium case with a swivel.

## 4. Derived motor requirements

Demand comes from ideal-actuator runs that hold heading for 10 minutes with no unloading
(`yaw_requirements.json`). The requirement adds margin and is checked against the real-motor runs.

| Requirement | With swivel or slip ring, line-driven | With swivel, gust-driven (bracket) | Without swivel |
|---|---|---|---|
| Peak torque | demand 4.8–5.3 mN·m → **≥ 6 mN·m** | demand 12.6–14 → ≥ 15 mN·m | 81–338 mN·m: not feasible |
| Stored momentum | demand 59 mN·m·s in 10 min → **≥ 60 mN·m·s usable each way, 90 preferred** | demand 127–132 → about 130 | far beyond any wheel |
| Speed | 60 mN·m·s = 2,800 rpm on the 3 mm-disc PCB rotor (J = 2.0×10⁻⁴), 16,000 rpm on the 55 mm steel wheel | | |
| Control loop | **100 Hz, about 20 rad/s bandwidth** | same | |
| Low-speed control | **sensored commutation, smooth through zero in both directions** | same | |
| Power | < 0.5 W mean, ≈ 10 W peak, < 0.2 Wh per 30 min | < 1 W mean | |

**Rigging recommendation.** Build the swivel or slip ring into the rigging at the payload attachment.
- A ball-bearing fishing/rigging swivel (≈0.05 mN·m) works if no wires cross the joint.
- A capsule slip ring (≈0.5–1 mN·m) is needed if power or signal must pass through. Its friction
  slightly helps unload the wheel in the line-driven case, and slightly hurts in the gust-driven case.
- Avoid brass barrel swivels. Their friction under load can reach several mN·m.
- Use 4 attachment lines (TSat-2B used 2) and measure the real joint's friction on the bench.
- Hang-test the payload with a logging IMU, with and without the swivel, to tell the line-driven and
  gust-driven explanations apart before flight.

## 5. Caveats

- **Satellite.** Inertia is assumed (1 kg, 10×10 cm). Replace it with the CAD value.
- **Tether and disturbance.** The "measured" levels are calibrated to one TSat-2B flight (a 2U
  payload on 2 lines), and two different mechanisms fit it equally well. The older low / medium / high
  levels are assumptions. A hang test of TSat-3A with a rate-logging IMU, with and without the swivel,
  would settle which mechanism dominates.
- **High-momentum PCB variant.** Built from the sweep design by doubling the steel disc to 3 mm; only
  inertia and mass change in the model. Its 223 g rotating mass needs checking against the 1U mass
  budget, and its axial magnetic pull is larger than the 12/16 design's.
- **PCB motor model.**
  - Rotor back iron is ideal (method of images, no saturation; 1.5 mm steel is about 1 T at these
    fluxes, by estimate).
  - No stator iron. No eddy or proximity losses in the copper.
  - 10% allowance for via and link resistance.
  - Inductance is ±30–50%.
  - Thermal limit assumes natural convection on both board faces (rotor airflow ignored, so
    conservative).
  - JLC and OSH Park 6-layer layer spacing is assumed.
  - Magnets are N42 at Br = 1.32 T. Block sizes are stock inch sizes.
- **Mechanics not modelled.** Bearings, rotor balance and axial magnetic pull. The axial pull between
  16 magnets and the steel disc is large, so the bearing must carry it.
- **mj5208.** Parameters (R, rotor inertia) are approximate. Iron-core cogging is ignored.
- **Single-axis model.** Pendulum swing coupling, sensor misalignment and the IMU's own motion are not
  modelled.
- **No measured motor data yet.** The test stator exists to measure Kt, R, back-EMF and heating and
  to correct this model before the payload board is ordered.
