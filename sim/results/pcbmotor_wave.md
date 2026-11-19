# Wave winding vs spiral coils: high-momentum payload stator (2026-11-19)

`pcbmotor/wave.py`, checked by `tests/test_wave.py`. Full table: [`pcbmotor_wave_table.md`](pcbmotor_wave_table.md);
raw numbers: `pcbmotor_wave.json`; layout of the recommended design: `pcbmotor_wave_layout.png`.

Everything uses the same rotor as the recommended high-momentum payload: 8 poles, 19.05 × 12.7 × 6.35 mm N42
blocks centred at r = 31 mm (covering 21.5–40.5 mm), 0.6 mm gap, steel back iron, 84 mm × 1.5 mm steel disc.
The stator is a JLC 4-layer board with 2 oz outer and 1 oz inner copper and 0.15 mm space. The drive is the
same 1 A, 11.1 V FOC envelope as `designs.py`.

## Why

Every built comparison in `~/KSC/research` favours wave or radial windings over spiral (concentric) coils:

- **Tokgöz 2020:** with identical boards, a radial wave gives the same torque as a concentric winding with
  20% less phase resistance.
- **Wang (CIOMP) 2015:** a wave winding drew 26.2% less power than a spiral at 5000 rpm.
- **CMU 2024:** chose a wave winding to avoid the spiral's "dead copper".

## Winding modelled

- **Shape:** a full-pitch, two-layer "radial wave" (Tokgöz's "radial" winding). Every conductor on a layer is
  a rotated copy of one template, so nothing on a layer crosses.
- **Conductor path:** each conductor runs radially across the magnet band. Its end sections slant by half a
  pole pitch at each end. The slant goes one way on L1/L3 and the other way on L2/L4.
- **Wave connection:** a top conductor joins the bottom one a pole pitch further round, which joins the next
  top one.
- **Phase bands:** per pole there are three phase bands of q = N/24 conductors, where N is the number of
  conductors per layer.
- **Layers:** L1-L2 and L3-L4 are two wave pairs, all in series. There are no parallel paths, so there is no
  circulating current.

## Results

Loss is the total at 15 mN·m and 2000 rpm: copper, plus eddy, plus circulating current. H is the momentum
at the voltage-limited top speed.

| Design | Kt mN·m/A | R Ω | Km mN·m/√W | Ripple | Top rpm | H mN·m·s | Eddy @2000 rpm | Loss |
|---|---|---|---|---|---|---|---|---|
| **Spiral, current recommendation** (6c/8p, 1.2 mm, r 21–41) | 21.8 | 1.21 | 16.2 | 3.0% | 4215 | 90 | 0.77 W | 1.63 W |
| Wave, same 21–41 envelope, N = 48, width ≤ 1.2 mm | 19.5 | 0.83 | 17.4 | 3.3% | 4713 | 101 | 0.53 W | 1.27 W |
| **Wave, end turns outside the magnets (vias at r 15 / 44), N = 48, width ≤ 1.2 mm** | 23.0 | 1.03 | 18.5 | 4.8% | 3997 | 85 | 0.33 W | **0.99 W** |
| same, width ≤ 1.6 mm | 23.0 | 0.89 | 19.8 | 4.8% | 3997 | 85 | 0.72 W | 1.29 W |
| same, N = 72, width ≤ 0.8 mm | 34.2 | 2.32 | 18.4 | 3.1% | 2681 | 57 | 0.14 W | 0.81 W |
| Wave, full fill (traces up to 5 mm) | 23.0 | 0.73 | 21.9 | 4.8% | 3997 | 85 | 11.3 W | 11.8 W |
| Wave, full fill, radial section slit into 2–4 strands | 23.0 | 0.74 | 21.8 | 4.8% | 3997 | 85 | 0.6–2.6 W | 6.2–7.7 W (4.6–5.2 W circulating) |
| Wave, CMU-style stitched layers (N = 96) | 11.1 | 0.48 | 13.1 | 3.2% | (6000 cap) | – | 2.8 W | 7.7 W (3.6 W circulating) |

## Findings

1. **A wave winding beats the spiral on this rotor.** With the same envelope it is a near drop-in: Km +8%,
   eddy −31%, loss −22% at the operating point, and 12% more momentum.
   - Letting the end turns run outside the magnets (inner vias at r = 15 mm, outer at 44 mm, still inside the
     90 mm board) gives Km +14% and **39% less loss**. Kt and momentum stay within a few percent of the spiral.
   - This matches the literature (Tokgöz: −20% R at equal torque). Some of our gain comes from the wave's
     higher winding factor: 0.955 against 0.866 for the 3-coils-per-4-poles spiral.
2. **Width is the eddy-vs-copper trade.**
   - Filling the slot pitch (traces up to 5 mm) gives the best Km (22) but 11 W of eddy drag at 2000 rpm.
   - Capping at about 1.2 mm is the optimum for this rotor at 2000 rpm. At 1.6 mm the copper saving is eaten
     by eddy loss.
   - This agrees with Chulaee 2024 that the optimum is a narrow trace. Theirs is 0.22 mm at 630 Hz; ours is
     267 Hz at 4000 rpm, so ours comes out wider.
3. **Slit traces (strands merged in the end turns) don't work here.** The strands of a 5 mm conductor sit up to
   about 30 electrical degrees apart. The current circulating between them (4–5 W at 2000 rpm) is several times
   the eddy loss it saves.
   - Running strands through the whole conductor isn't possible either: the slanted end turns are too narrow.
   - This is the circulating-current effect Chulaee 2022/2023 measured (28.3 W against 0.5 W eddy in their
     parallel wave stator).
4. **CMU-style stitched (parallel) layers are the worst option.** The current share differs per layer (2 oz
   outer vs 1 oz inner), and back-EMF differs with depth. Together they give Km 13 and about 3.6 W circulating
   at 2000 rpm. CMU got away with it at low speed and with no back iron.
5. **Higher N** (more, narrower conductors) cuts eddy loss further but raises Kt. That lowers the top speed on
   the 11.1 V bus and loses momentum (N = 72: 57 mN·m·s). N = 48 keeps the spiral's Kt.

## Caveats and layout risks

- **Vias:** only through vias are available. The L1-L2 vias must land between L3/L4 traces and the reverse,
  so the pairs need staggering by half a slot and local necking. This is not modelled beyond a via-pitch check.
  The model counts one 0.3 mm via per conductor, plus a 5% allowance for the lap jogs at the end of each wave
  pass and the phase leads. The spiral model uses 10%.
- **Inner via radius:** r = 15 mm needs the board to be free of copper and hardware from r = 15 to 21 mm, so
  the hub, bearing and standoffs must fit inside r ≈ 14 mm.
- **End turns:** they are linear-in-angle slants, the narrowest at the inner via radius. A real layout would
  curve them to even out the width.
- **Missing loss terms:** the eddy model has only the Bz·w² term. The Bφ·t² term in Chulaee's Eq. 1 is small
  for 2 oz copper and is left out. Eddy paths through vias are not modelled. Rotor eccentricity is not
  modelled; Chulaee 2023 finds series wave windings tolerant of it, unlike paralleled windings.
- **Validation:** the EM method was validated on CMU's wave-wound wheel (Kt within about 10%, R within 11%),
  so the topology itself is covered. The back-iron image model still needs the test stator.
