# TSat-3A reaction-wheel simulation

Models used to size the TSat-3A reaction wheel and to judge whether a PCB (printed-stator,
axial-flux) motor can be the main reaction-wheel motor. Results and tables are in
[`results/RESULTS.md`](results/RESULTS.md).

## Setup and running

```bash
cd sim
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt

.venv/bin/python -m pcbmotor.designs        # PCB motor design sweep and plots (~6 min; add --reuse to re-pick from the saved CSV)
.venv/bin/python -m data.tsat1_swing        # TSat-1 flight accelerometer analysis (needs the raw log, see below)
.venv/bin/python -m data.video_spin ~/KSC/T-Sat2B_Flight.mp4 --name tsat2b_full   # yaw from flight video (~15 min)
.venv/bin/python -m data.video_phases results/video_spin_tsat2b_full.csv --name tsat2b_full
.venv/bin/python -m yaw.calibrate           # fit measured disturbance levels (~1 min); copy values into yaw/sim.py
.venv/bin/python -m yaw.sim                 # yaw Monte Carlo for all motor candidates (~2 min)
.venv/bin/python tests/test_em_consistency.py   # torque model cross-check
.venv/bin/python -m validation.cmu_wheel    # model vs. the measured CMU PCB wheel (~2 min)
.venv/bin/python -m pcbmotor.stacking       # stacked boards / dual rotor vs single board (~2 min)
.venv/bin/python -m pcbmotor.asbuilt        # test stator as laid out (after PCBMotorTest/gen_stator.py)
.venv/bin/python tests/test_layout.py       # as-built coil layout checks
```

Run `pcbmotor.designs` before `yaw.sim`: the yaw sim reads the recommended PCB designs from
`results/pcbmotor_designs.json`. Every script is deterministic (fixed seeds).

The TSat-1 raw log is read in place from `~/Documents/tsat1launch/tsatlog3/data.csv` (the same files
are in the shared Drive under TSat-1 / Flight Data / tsatlog3). It is not copied into this repo.

## Layout

| Path | What it is |
|---|---|
| `pcbmotor/geometry.py` | Stator and rotor geometry, fab stackups (OSH Park 2/4/6-layer, JLCPCB 4/6-layer presets). One source of truth that the KiCad footprint generator will also use. |
| `pcbmotor/em.py` | Resistance, inductance, torque and back-EMF of a PCB motor. |
| `pcbmotor/layout.py` | As-built copper: real spirals per layer, layer vias and terminals. Used by the KiCad generator (`PCBMotorTest/gen_stator.py`) and by `em.analyse_layout`. |
| `pcbmotor/asbuilt.py` | Model of the test stator exactly as laid out, and the bench numbers to check it against. |
| `pcbmotor/stacking.py` | Stacked stator boards (series/parallel), dual rotors and more-layer boards against one board, including eddy drag in the traces and the pull between rotors. |
| `pcbmotor/designs.py` | Design sweep for the 40 mm test stator and the 1U payload stator; drive envelope; plots. |
| `data/tsat1_swing.py` | Swing and excitation spectrum from the TSat-1 flight accelerometer log. |
| `yaw/candidates.py` | Motor candidates (datasheet values, approximations flagged). |
| `yaw/sim.py` | Single-axis yaw Monte Carlo: tether, swivel/slip ring, gusts, PID with wheel unloading, motor limits. |
| `data/video_spin.py`, `data/video_phases.py` | Yaw rate from an onboard nadir video (feature-matched image rotation) and per-phase statistics. |
| `yaw/calibrate.py` | Fits the measured disturbance levels to the video statistics. |
| `validation/cmu_wheel.py` | Rebuilds the CMU PCB reaction wheel from its paper and design files and compares the model with its measured resistance and torque constant. |
| `tests/test_em_consistency.py` | Checks Lorentz-force torque against d(flux linkage)/d(angle). |
| `results/` | CSV/JSON outputs, plots and `RESULTS.md`. |

## Models

### PCB motor (`pcbmotor/`)

- **Geometry.** Each coil is a stack of concentric annular-sector turns, inset by one trace pitch per
  turn, repeated on every copper layer and wired in series. That is the usual approximation of a
  spiral; a real spiral differs by under one turn. Coil/pole combinations are assigned to phases with a
  star-of-slots map, which rejects unbalanced windings.
- **Resistance.** Exact turn lengths, the copper weight of each layer (OSH Park 4/6-layer inner layers
  are 0.5 oz, JLCPCB presets are 1 oz or 2 oz) and copper resistivity at 20 °C, plus 10% for vias and
  links.
- **Torque and back-EMF.** [magpylib](https://magpylib.readthedocs.io) computes the analytic field of
  every rotor magnet at points every 0.25 mm along every trace, and the Lorentz force is summed into
  torque. The same curve divided by speed is the back-EMF (energy conservation). The stator has no
  iron. A steel back plate behind the magnets is modelled by the method of images (ideal, unsaturated
  steel ≈ magnet of twice the thickness). One coil is simulated over one electrical period and rotated
  for the others. The torque constant is for ideal sinusoidal (FOC) currents at the best current angle,
  per amp of peak phase current.
- **Check.** `tests/test_em_consistency.py` integrates the magnet flux through every turn and checks
  that d(flux linkage)/d(angle) matches the Lorentz torque within 5%. The check found a real bug (long
  radial trace edges were sampled once), which is now fixed.
- **Inductance.** Neumann integral of the mean turn, with the winding's geometric mean distance for
  self-inductance and the layer z-spacing for mutual terms. Order of magnitude only; it sets the PWM
  frequency, not the torque.
- **Drive envelope.** 11.1 V bus (3S LiPo), SVPWM phase voltage Vbus/√3, a driver current limit (1 A in
  the sweep) and a thermal limit of 40 K board temperature rise with natural convection
  (h = 10 W/m²K) from both faces. Ripple current is Vbus / (8·L·f), a bound for a two-level bridge.

### Yaw simulation (`yaw/`)

- **Plant.** Rigid satellite (yaw axis) plus a reaction wheel. Wheel speed is relative to the body.
  Motor torque is limited by the current limit, the back-EMF/voltage limit and the speed cap. Bearing
  drag is Coulomb plus viscous. Sensorless motors produce no controlled torque below 15% of maximum
  speed and are run at a 50% speed bias.
- **Tether.**
  - **No swivel:** torsional spring and damper to the balloon end. Stored line twist (0.25 to 3 turns)
    is the initial spring deflection, and the balloon end wanders as a random process.
  - **Swivel or slip ring:** only the joint's friction torque couples the payload to the line. A
    ball-bearing swivel is 0.05 mN·m; a capsule slip ring is 1 mN·m. Stored twist unwinds through the
    joint in a few seconds.
- **Gusts.** Aerodynamic yaw torque as filtered noise, with a 0.3 Hz corner set by the TSat-1 tension
  spectrum.
- **Controller.** 100 Hz PID (20 rad/s bandwidth) on gyro rate and heading, one control period of latency, BNO085-class gyro
  noise. **Wheel unloading:** the heading reference walks at no more than 1 °/s in the direction that
  slows the wheel. The satellite gives up absolute heading slowly instead of saturating. The mission
  needs a steady camera, not a fixed compass heading.
- **Monte Carlo.** 3 rigging options × 5 disturbance levels × 8 seeds. Tether torsional period is
  random in 20–120 s for the low / medium / high levels and log-uniform in 2–15 s for the two
  "measured" levels calibrated to the TSat-2B flight video (`yaw/calibrate.py`). Each run is 600 s, and energy is scaled to a 30 min flight. An ideal actuator
  (unlimited torque and speed) is run to measure the demand itself.

## Assumptions

| Quantity | Value | Basis |
|---|---|---|
| Satellite yaw inertia | 1.67×10⁻³ kg·m² | 1 kg, 10×10 cm cross-section (ASSUMED; replace with CAD) |
| Bus voltage | 11.1 V | 3S LiPo proposed for the wheel in Discord |
| Video-stable criterion | \|rate\| < 2 °/s and \|heading error\| < 10 ° | ASSUMED; MCR leaves X/Y/Z as TBD |
| Tether torsional period | 20–120 s (assumed levels); 2–15 s (measured levels), ζ = 0.05 | TSat-2B video shows 2–13 s periods |
| Measured levels | line-driven: wander σ 2.36 rad, 8 s; gust-driven: 2.72 mN·m RMS | calibrated so the uncontrolled 1U sim matches TSat-2B's 172 °/s RMS |
| Stored line twist | 0.25 / 1 / 3 turns (low/medium/high) | ASSUMED |
| Balloon-end yaw wander | σ = 0.5 / 2 / 6 rad, 60 s correlation | ASSUMED |
| Gust yaw torque | 0.1 / 0.3 / 1.0 mN·m RMS, 0.3 Hz | 0.5·ρ·v²·A·Cd at 5 m/s on a 1U face ≈ 0.16 N, times a 2–6 mm centre-of-pressure offset |
| Swivel friction | 0.05 mN·m (ball bearing), 1 mN·m (slip ring) | μ·F·r estimate; capsule slip-ring datasheets ~0.5–1 mN·m |
| Gyro noise | 0.1 °/s RMS | BNO085 class |
| JLCPCB layer spacing | 1.6 mm board, outer prepreg ~0.2 mm | ASSUMED: JLC doesn't publish JLC04161H-7628 / JLC06161H-2116 spacing on the capabilities page |
| OSH Park 6-layer spacing | evenly paired layers | ASSUMED |
| Magnets | N42, Br = 1.32 T | stock K&J-style blocks and discs |
| mj5208 parameters | Kv 304, R ≈ 0.12 Ω, rotor J ≈ 8.5×10⁻⁵ | APPROX, verify |
