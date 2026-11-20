# Wave winding vs spiral coils: high-momentum payload stator (2026-11-19)

`pcbmotor/wave.py`, checked by `tests/test_wave.py`. Full table: [`pcbmotor_wave_table.md`](pcbmotor_wave_table.md);
raw numbers: `pcbmotor_wave.json`; layout of the recommended design (centrelines): `pcbmotor_wave_layout.png`.

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
- **Neethu 2017:** a radial wave gives 56 V of back-EMF against 45 V for trapezoidal coils, with half the
  distortion.

## Winding modelled

- **Shape:** a full-pitch, two-layer "radial wave". It is Tokgöz's "radial" winding, and closest to Paul
  2019's "overlapping radial wave", which gives a sinusoidal back-EMF that suits FOC. Every conductor on a layer
  is a rotated copy of one template, so nothing on a layer crosses.
- **Conductor path:** each conductor runs radially across the magnet band. Its end sections slant by half a
  pole pitch at each end, one way on L1/L3 and the other way on L2/L4.
- **Phase bands:** per pole there are three phase bands of q = N/24 conductors, where N is the number of
  conductors per layer.
- **Layers:** L1-L2 and L3-L4 are two wave pairs, all in series.
- **Trace options:**
  - fill: as wide as the slot pitch allows
  - capped: fill, but no wider than a set limit
  - slit: the radial section is split into strands that merge again in the end turns
  - full-length strands: strands that only join at the phase terminals, with the lane order either reversed
    at every layer via ("swap", as in `bundle.py` and François & Dehez's "parallel connection") or fixed

## Results

Loss is the total at 15 mN·m and 2000 rpm: copper, plus eddy, plus circulating current. H is the momentum
at the voltage-limited top speed. "Ends out" means the vias sit at r = 15 and 44 mm, with the active section
at r = 22–40 mm.

| Design | Kt mN·m/A | R Ω | Km mN·m/√W | Ripple | Top rpm | H mN·m·s | Eddy @2000 / top rpm | Loss |
|---|---|---|---|---|---|---|---|---|
| **Spiral, current recommendation** (6c/8p, 1.2 mm, r 21–41) | 21.8 | 1.21 | 16.2 | 3.0% | 4215 | 90 | 0.77 / 3.43 W | 1.63 W |
| Wave, same 21–41 envelope, N = 48, width ≤ 1.2 mm | 19.5 | 0.83 | 17.4 | 3.3% | 4713 | 101 | 0.53 / 2.93 W | 1.27 W |
| Wave, ends out, N = 48, width ≤ 1.2 mm | 23.0 | 1.03 | 18.5 | 4.8% | 3997 | 85 | 0.33 / 1.31 W | 0.99 W |
| **Wave, ends out, N = 48, width ≤ 1.6 mm, 2 strands swapped at every via** | 23.0 | 1.00 | 18.8 | 4.8% | 3997 | 85 | 0.13 / 0.53 W | **0.77 W** |
| same, 2 strands, lanes not swapped | 23.0 | 1.00 | 18.8 | 4.8% | 3997 | 85 | 0.13 / 0.53 W | 0.86 W (0.09 W circulating) |
| same, width ≤ 2.0 mm, 2 strands swapped | 23.0 | 0.93 | 19.5 | 4.8% | 3997 | 85 | 0.27 / 1.06 W | 0.86 W |
| Wave, same 21–41 envelope, width ≤ 2.0 mm, 3 strands swapped | 19.5 | – | 17.4 | 3.3% | 4713 | 101 | 0.08 / 0.46 W | 0.83 W |
| Wave, ends out, N = 72, width ≤ 0.8 mm | 34.2 | 2.32 | 18.4 | 3.1% | 2681 | 57 | 0.14 W | 0.81 W |
| Wave, full fill (traces up to 5 mm) | 23.0 | 0.73 | 21.9 | 4.8% | 3997 | 85 | 11.3 W | 11.8 W |
| Wave, full fill, slit radial section (strands merge in the end turns) | 23.0 | 0.74 | 21.8 | 4.8% | 3997 | 85 | 0.6–2.6 W | 6.2–7.7 W (4.6–5.2 W circulating) |
| Wave, CMU-style stitched layers (N = 96) | 11.1 | 0.48 | 13.1 | 3.2% | (6000 cap) | – | 2.8 W | 7.7 W (3.6 W circulating) |

Three strands in the ends-out envelope fail the via check: 3 vias per conductor don't fit at r = 15 mm with
48 conductors. They fit in the 21–41 mm envelope, but that envelope has lower Km.

## Findings

1. **A wave winding beats the spiral on this rotor.**
   - In the same r = 21–41 mm envelope with 1.2 mm traces: Km +8% and loss −22%.
   - With the end turns outside the magnets (vias at r = 15 and 44 mm, still inside the 90 mm board), the same
     traces give Km +14% and loss −39%, with Kt and momentum within a few percent of the spiral.
   - Some of the gain is the higher winding factor: 0.955 against 0.866 for the 3-coils-per-4-poles spiral.
   - Chulaee 2023/2024 also found the best designs keep magnets off the end turns.
2. **Recommended: 1.6 mm traces split into 2 strands that swap lanes at every layer via.** Total loss is
   0.77 W, 53% below the spiral, and eddy drag at top speed is 0.53 W against 3.4 W.
   - Swapping at the vias (L2 and L4 carry the reversed lane order) balances the two strands' EMFs. Circulating
     current falls from 0.09–0.5 W with fixed lanes to about 0.001 W.
   - This is François & Dehez's "parallel connection", which cut eddy loss about 81% for about 13% more
     resistance, and the same lane reversal `bundle.py` uses on the strand-test board.
3. **Slits that merge in the end turns don't work.** The strands of a wide conductor sit up to about
   30 electrical degrees apart, and joining them at both ends of every conductor lets 4–5 W circulate at
   2000 rpm. That is François & Dehez's "crossed" case and Chulaee 2022's parallel wave stator.
4. **CMU-style stitched (parallel) layers are the worst option**: Km 13, with 3.6 W circulating at 2000 rpm.
   Han 2021 saw the same effect: circulating current of 4.6× the applied current in its paralleled wave stator,
   which capped its speed at 1400 rpm.
5. **N = 48 keeps the spiral's Kt.** Higher N (72) cuts eddy loss but raises Kt, which lowers the top speed on
   the 11.1 V bus and loses momentum (57 mN·m·s).

## Caveats and layout risks

- **Measurement derating:** Paul 2019's built wave motor came in 15.5% below FEA on torque and 5.8% below on
  back-EMF, with R within 5%. Marignetti 2018 matched EMF within 1–2%. Plan on a 5–15% derating until bench data
  exists.
- **Vias:** only through vias are available, so each pair's vias must fall between the other pair's traces.
  That needs a half-slot stagger plus local necking. With 2 strands, each conductor needs 2 vias per end, so
  48 conductors at r = 15 mm leave 1.96 mm of pitch against the 1.5 mm needed. The model counts one 0.3 mm via
  per conductor, plus a 5% allowance for the lap jogs and phase leads.
- **Inner via radius:** r = 15 mm needs the board free from r = 15 to 21 mm, so the hub, bearing and standoffs
  must fit inside r ≈ 14 mm.
- **End turns:** they are linear-in-angle slants. A real layout would curve them to even out the width.
- **Missing loss terms:**
  - The eddy model has only the Bz·w² term. The Bφ·t² term (Chulaee Eq. 1, Marcolini Eq. 14) is small for
    2 oz copper.
  - Eddy paths through vias are not modelled.
  - Rotor eccentricity is not modelled. Chulaee 2023 finds series windings tolerant of it; swapped strands are
    the only parallel paths here.
  - Keep copper pours and conductive supports out of the field. Marcolini lost about 20% to internal copper
    planes, and Salim's steel support disc took input power from 1.5 W to 14.5 W.
- **Validation:** the EM method was validated on CMU's wave-wound wheel (Kt within about 10%, R within 11%),
  so the topology itself is covered. The back-iron image model still needs the test stator.
