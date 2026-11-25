# Spiral coils with the end turns outside the magnets (2026-11-20)

`pcbmotor/spiral_ends_out.py`. Raw numbers: `pcbmotor_spiral_ends_out.json`. As-built layouts: [`spiral_ends_out_w0.6_r15-42`](layouts/spiral_ends_out_w0.6_r15-42.png), [`spiral_ends_out_w0.8_r15-42`](layouts/spiral_ends_out_w0.8_r15-42.png), [`spiral_ends_out_w1_r15-42`](layouts/spiral_ends_out_w1_r15-42.png), [`spiral_ends_out_w0.8_r15-44`](layouts/spiral_ends_out_w0.8_r15-44.png).

Same rotor, drive and operating point as the wave study (5f): 8 × 19.05 × 12.7 × 6.35 mm N42 blocks at r 31 mm (r 21.5–40.5), 0.6 mm gap, 3 mm steel; 1 A, 11.1 V FOC; loss = copper + eddy (+ circulating) at 15 mN·m and 2000 rpm.

## Why

5f compared the wave, with its end turns outside the magnets (vias at r 15 / 44), against a spiral kept inside r 21–41, so the spiral's end turns sat over the magnets. Here the spiral gets the same room.

## Ideal model: Km against envelope (6 coils / 8 poles, 2 oz outer / 1 oz inner)

Trace 0.8 mm. Km (turns per layer):

| r_in \ r_out | 41 | 42 | 43 | 44 |
|---|---|---|---|---|
| 21 | 15.7 (10) | 16.5 (11) | 17.4 (11) | 17.9 (12) |
| 19 | 17.5 (11) | 18.3 (12) | 19.2 (12) | 19.7 (13) |
| 17 | 18.6 (12) | 19.4 (13) | 20.2 (13) | 20.7 (14) |
| 15 | 18.8 (13) | 19.6 (14) | 20.3 (14) | 20.7 (15) |
| 13 | 18.0 (13) | 18.7 (13) | 19.3 (13) | 19.7 (13) |

Trace 1.2 mm. Km (turns per layer):

| r_in \ r_out | 41 | 42 | 43 | 44 |
|---|---|---|---|---|
| 21 | 16.1 (7) | 16.8 (8) | 17.8 (8) | 18.6 (8) |
| 19 | 17.9 (8) | 18.9 (8) | 19.6 (9) | 20.3 (9) |
| 17 | 19.0 (9) | 20.0 (9) | 20.8 (9) | 21.3 (10) |
| 15 | 19.3 (9) | 20.1 (10) | 20.8 (10) | 21.4 (10) |
| 13 | 18.6 (9) | 19.3 (9) | 19.9 (9) | 20.3 (9) |

With the wiring ring outside the winding, a 90 mm board allows r_out up to 42.1 mm (0.8 mm traces) or 42.1 mm (1.2 mm): the L2–L3 via, 3 ring tracks at 0.68 mm pitch and 0.3 mm edge clearance take 2.9 mm.

## Ideal model: best design per width, r_in 15, r_out 42 (fits 90 mm)

| Stackup | Trace | Turns | Wiring | Kt | R Ω | Km | No-load rpm | H mN·m·s | Eddy @2000 | Loss |
|---|---|---|---|---|---|---|---|---|---|---|
| jlc4_2oz_out | 0.5 | 20 | series | 61.1 | 7.11 | 18.7 | 1503 | 32 | 0.12 | 0.77 |
| jlc4_2oz_out | 0.5 | 20 | parallel | 30.5 | 1.78 | 18.7 | 3006 | 64 | 0.12 | 0.77 |
| jlc4_2oz_out | 0.6 | 18 | series | 52.9 | 5.15 | 19.1 | 1734 | 37 | 0.18 | 0.80 |
| jlc4_2oz_out | 0.6 | 18 | parallel | 26.5 | 1.29 | 19.1 | 3467 | 74 | 0.18 | 0.80 |
| jlc4_2oz_out | 0.8 | 14 | series | 41.8 | 3.05 | 19.6 | 2196 | 47 | 0.34 | 0.93 |
| jlc4_2oz_out | 0.8 | 14 | parallel | 20.9 | 0.76 | 19.6 | 4391 | 94 | 0.34 | 0.93 |
| jlc4_2oz_out | 1 | 11 | series | 34.5 | 2.00 | 19.9 | 2659 | 57 | 0.55 | 1.12 |
| jlc4_2oz_out | 1 | 11 | parallel | 17.3 | 0.50 | 19.9 | 5317 | 113 | 0.55 | 1.12 |
| jlc4_2oz_out | 1.2 | 10 | series | 29.4 | 1.43 | 20.1 | 3118 | 67 | 0.82 | 1.37 |
| jlc4_2oz_out | 1.2 | 10 | parallel | 14.7 | 0.36 | 20.1 | 6237 | 128 | 0.82 | 1.37 |
| jlc4_2oz_out | 1.6 | 8 | series | 22.7 | 0.83 | 20.4 | 4039 | 86 | 1.49 | 2.03 |
| jlc4_2oz_out | 1.6 | 8 | parallel | 11.4 | 0.21 | 20.4 | 8077 | 128 | 1.49 | 2.03 |
| jlc4_2oz | 0.5 | 20 | series | 61.1 | 5.06 | 22.2 | 1502 | 32 | 0.14 | 0.60 |
| jlc4_2oz | 0.5 | 20 | parallel | 30.6 | 1.26 | 22.2 | 3004 | 64 | 0.14 | 0.60 |
| jlc4_2oz | 0.6 | 18 | series | 53.0 | 3.66 | 22.6 | 1732 | 37 | 0.21 | 0.65 |
| jlc4_2oz | 0.6 | 18 | parallel | 26.5 | 0.92 | 22.6 | 3465 | 74 | 0.21 | 0.65 |
| jlc4_2oz | 0.8 | 14 | series | 41.8 | 2.17 | 23.2 | 2194 | 47 | 0.39 | 0.81 |
| jlc4_2oz | 0.8 | 14 | parallel | 20.9 | 0.54 | 23.2 | 4388 | 94 | 0.39 | 0.81 |
| jlc4_2oz | 1 | 11 | series | 34.6 | 1.42 | 23.6 | 2656 | 57 | 0.63 | 1.03 |
| jlc4_2oz | 1 | 11 | parallel | 17.3 | 0.36 | 23.6 | 5313 | 113 | 0.63 | 1.03 |
| jlc4_2oz | 1.2 | 10 | series | 29.5 | 1.02 | 23.9 | 3116 | 66 | 0.93 | 1.33 |
| jlc4_2oz | 1.2 | 10 | parallel | 14.7 | 0.25 | 23.9 | 6231 | 128 | 0.93 | 1.33 |
| jlc4_2oz | 1.6 | 8 | series | 22.7 | 0.59 | 24.2 | 4035 | 86 | 1.70 | 2.08 |
| jlc4_2oz | 1.6 | 8 | parallel | 11.4 | 0.15 | 24.2 | 8071 | 128 | 1.70 | 2.08 |

## As built (real spirals, JLC 0.3 / 0.56 mm vias, ring and leads) against the wave

| Design | Stackup | Kt | R Ω | Km | No-load rpm | H mN·m·s | Eddy + circ @2000 | Loss | Ls µH | PWM ripple 50 / 100 kHz | Board |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Spiral w1.2 r21–41, 6 turns, series | jlc4_2oz_out | 19.2 | 1.124 | **14.8** | 4793 | 102 | 0.81 W | **1.85 W** | 19.7 | 0.19 / 0.05 W | 87.8 mm |
| Spiral w0.8 r15–42, 13 turns, parallel | jlc4_2oz_out | 21.8 | 0.792 | **20.0** | 4213 | 90 | 0.40 W | **0.96 W** | 19.4 | 0.14 / 0.04 W | 89.8 mm |
| Spiral w1.2 r21–41, 6 turns, series | jlc4_2oz | 19.2 | 0.789 | **17.6** | 4791 | 102 | 0.92 W | **1.65 W** | 19.7 | 0.13 / 0.03 W | 87.8 mm |
| Spiral w0.6 r15–42, 16 turns, parallel | jlc4_2oz | 27.8 | 0.929 | **23.5** | 3305 | 71 | 0.24 W | **0.65 W** | 29.4 | 0.08 / 0.02 W | 89.8 mm |
| Spiral w0.8 r15–42, 13 turns, parallel | jlc4_2oz | 21.8 | 0.564 | **23.7** | 4209 | 90 | 0.45 W | **0.85 W** | 18.2 | 0.12 / 0.03 W | 89.8 mm |
| Spiral w1 r15–42, 11 turns, parallel | jlc4_2oz | 17.9 | 0.383 | **23.6** | 5128 | 109 | 0.73 W | **1.13 W** | 12.4 | 0.17 / 0.04 W | 89.8 mm |
| Spiral w0.8 r15–44, 14 turns, parallel | jlc4_2oz | 25.0 | 0.628 | **25.8** | 3669 | 78 | 0.46 W | **0.79 W** | 21.7 | 0.10 / 0.03 W | 93.8 mm |
| Wave n48, 2 strands swapped, ideal (5f) | jlc4_2oz_out | 23.0 | – | 18.8 | ~4000 | 85 | 0.14 W | 0.78 W | 8.1 (5h) | 1.05 / 0.26 W (5h) | 90 mm (ideal, no jumpers) |
| Wave n48, 2 strands swapped, as built (through vias) | jlc4_2oz | 22.5 | – | 19.2 | ~4000 | 85 | 0.16 W | 0.77 W | 8.1 (5h) | 1.05 / 0.26 W (5h) | ~129 mm |
| Wave n48, 2 strands swapped, as built (blind vias) | jlc4_2oz | 22.6 | – | 20.6 | ~4000 | 85 | 0.16 W | 0.69 W | 8.1 (5h) | 1.05 / 0.26 W (5h) | ~117 mm |

Circulating current between the two parallel coils of a phase with the rotor tilted (gap ±dg at opposite coils), at 2000 rpm:

| Design | ±0.05 mm | ±0.1 mm |
|---|---|---|
| w0.8 r15–42 jlc4_2oz_out | 1.40 mW | 5.59 mW |
| w0.6 r15–42 jlc4_2oz | 1.93 mW | 7.72 mW |
| w0.8 r15–42 jlc4_2oz | 1.98 mW | 7.90 mW |
| w1 r15–42 jlc4_2oz | 1.97 mW | 7.87 mW |
| w0.8 r15–44 jlc4_2oz | 2.28 mW | 9.12 mW |


## Stranded (strands swap lanes at every layer via, bundle.py), r_in 15, r_out fitted to 90 mm

JLC 4L 2 oz on all layers, JLC 0.3 / 0.56 mm vias, ring and leads as above, strand gap 0.15 mm. The first row is the solid 0.8 mm coil built by the same code. Drag = eddy + circulating between strands, at 2000 rpm. "Untransposed" is the circulating loss if each strand kept its lane on every layer. Rows in **bold** land in the 4000–4300 rpm no-load window (H about 85–90 mN·m·s). bundle.py cannot lay out 3 × 1.0, 3 × 1.8, 4 × 1.6 or 4 × 2.0 mm bundles (via row in the opening), so they are missing.

| Strands × width | Bundle | r_out | Turns | Coils | Kt | R Ω | Km | No-load rpm / H | Eddy | Circulating (untransposed) | Drag @2000 | Loss | Ls µH | PWM ripple 50 / 100 kHz |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| **1 × 0.8 mm** | 0.8 mm | 41.75 | 13 | parallel | 21.4 | 0.559 | 23.4 | **4294 / 92** | 0.450 W | 0.00 mW | 2.15 mN·m | **0.86 W** | 17.9 | 0.13 / 0.03 W |
| 1 × 0.8 mm | 0.8 mm | 41.75 | 13 | series | 42.8 | 2.202 | 23.5 | 2147 / 46 | 0.450 W | 0.00 mW | 2.15 mN·m | 0.86 W | 71.4 | 0.05 / 0.01 W |
| 1 × 1.2 mm | 1.2 mm | 41.35 | 9 | parallel | 14.3 | 0.270 | 22.4 | 6429 / 128 | 1.052 W | 0.00 mW | 5.03 mN·m | 1.50 W | 8.4 | 0.24 / 0.06 W |
| 1 × 1.2 mm | 1.2 mm | 41.35 | 9 | series | 28.6 | 1.050 | 22.8 | 3214 / 69 | 1.052 W | 0.00 mW | 5.03 mN·m | 1.49 W | 33.5 | 0.07 / 0.02 W |
| **2 × 0.3 mm** | 0.75 mm | 41.80 | 13 | parallel | 22.5 | 0.763 | 21.1 | **4074 / 87** | 0.049 W | 0.03 mW (14 mW) | 0.24 mN·m | **0.56 W** | 18.8 | 0.15 / 0.04 W |
| 2 × 0.3 mm | 0.75 mm | 41.80 | 13 | series | 45.1 | 3.019 | 21.2 | 2037 / 43 | 0.049 W | 0.03 mW (14 mW) | 0.24 mN·m | 0.55 W | 75.0 | 0.05 / 0.01 W |
| **2 × 0.325 mm** | 0.8 mm | 41.75 | 13 | parallel | 21.4 | 0.679 | 21.2 | **4295 / 92** | 0.060 W | 0.04 mW (17 mW) | 0.29 mN·m | **0.56 W** | 16.9 | 0.17 / 0.04 W |
| 2 × 0.325 mm | 0.8 mm | 41.75 | 13 | series | 42.7 | 2.683 | 21.3 | 2148 / 46 | 0.060 W | 0.04 mW (17 mW) | 0.29 mN·m | 0.56 W | 67.6 | 0.06 / 0.02 W |
| 2 × 0.375 mm | 0.9 mm | 41.65 | 11 | parallel | 18.9 | 0.525 | 21.3 | 4855 / 104 | 0.082 W | 0.05 mW (20 mW) | 0.39 mN·m | 0.58 W | 13.3 | 0.20 / 0.05 W |
| 2 × 0.375 mm | 0.9 mm | 41.65 | 11 | series | 37.8 | 2.067 | 21.5 | 2427 / 52 | 0.082 W | 0.05 mW (20 mW) | 0.39 mN·m | 0.57 W | 53.3 | 0.07 / 0.02 W |
| 2 × 0.425 mm | 1 mm | 41.55 | 10 | parallel | 17.0 | 0.425 | 21.3 | 5391 / 115 | 0.108 W | 0.07 mW (24 mW) | 0.52 mN·m | 0.60 W | 10.9 | 0.23 / 0.06 W |
| 2 × 0.425 mm | 1 mm | 41.55 | 10 | series | 34.1 | 1.668 | 21.5 | 2695 / 57 | 0.108 W | 0.07 mW (24 mW) | 0.52 mN·m | 0.59 W | 43.5 | 0.07 / 0.02 W |
| 2 × 0.525 mm | 1.2 mm | 41.35 | 9 | parallel | 14.3 | 0.303 | 21.2 | 6435 / 128 | 0.176 W | 0.13 mW (39 mW) | 0.84 mN·m | 0.68 W | 7.7 | 0.31 / 0.08 W |
| 2 × 0.525 mm | 1.2 mm | 41.35 | 9 | series | 28.5 | 1.181 | 21.4 | 3217 / 69 | 0.176 W | 0.13 mW (39 mW) | 0.84 mN·m | 0.67 W | 30.9 | 0.09 / 0.02 W |
| 3 × 0.3 mm | 1.2 mm | 41.35 | 8 | parallel | 13.9 | 0.334 | 19.6 | 6609 / 128 | 0.047 W | 0.14 mW (32 mW) | 0.22 mN·m | 0.63 W | 8.0 | 0.31 / 0.08 W |
| 3 × 0.3 mm | 1.2 mm | 41.35 | 8 | series | 27.8 | 1.306 | 19.8 | 3305 / 70 | 0.047 W | 0.14 mW (32 mW) | 0.22 mN·m | 0.62 W | 31.8 | 0.09 / 0.02 W |
| 2 × 0.725 mm | 1.6 mm | 40.95 | 6 | parallel | 10.0 | 0.169 | 19.9 | 9170 / 128 | 0.328 W | 0.40 mW (51 mW) | 1.57 mN·m | 0.90 W | 4.0 | 0.54 / 0.14 W |
| 2 × 0.725 mm | 1.6 mm | 40.95 | 6 | series | 20.0 | 0.648 | 20.3 | 4585 / 98 | 0.328 W | 0.40 mW (51 mW) | 1.57 mN·m | 0.87 W | 16.1 | 0.17 / 0.04 W |
| 3 × 0.433 mm | 1.6 mm | 40.95 | 6 | parallel | 10.0 | 0.184 | 19.0 | 9173 / 128 | 0.105 W | 0.50 mW (55 mW) | 0.50 mN·m | 0.73 W | 4.4 | 0.47 / 0.12 W |
| 3 × 0.433 mm | 1.6 mm | 40.95 | 6 | series | 20.0 | 0.710 | 19.4 | 4587 / 98 | 0.105 W | 0.50 mW (55 mW) | 0.50 mN·m | 0.70 W | 17.7 | 0.15 / 0.04 W |
| 3 × 0.567 mm | 2 mm | 40.55 | 5 | parallel | 7.7 | 0.123 | 18.0 | 11888 / 128 | 0.188 W | 1.32 mW (93 mW) | 0.90 mN·m | 0.88 W | 2.9 | 0.66 / 0.16 W |
| 3 × 0.567 mm | 2 mm | 40.55 | 5 | series | 15.4 | 0.468 | 18.4 | 5944 / 127 | 0.188 W | 1.32 mW (93 mW) | 0.90 mN·m | 0.85 W | 11.6 | 0.20 / 0.05 W |
| 4 × 0.338 mm | 1.8 mm | 40.75 | 5 | parallel | 8.4 | 0.155 | 17.4 | 10926 / 128 | 0.056 W | 0.98 mW (58 mW) | 0.27 mN·m | 0.80 W | 3.2 | 0.69 / 0.17 W |
| 4 × 0.338 mm | 1.8 mm | 40.75 | 5 | series | 16.8 | 0.595 | 17.8 | 5463 / 117 | 0.056 W | 0.98 mW (58 mW) | 0.27 mN·m | 0.77 W | 12.8 | 0.22 / 0.06 W |

Eddy against the solid coil of the same bundle width, model vs (strand / bundle)² × copper fraction:

| Bundle | Strands | Eddy ratio | (w_s / W)² × n w_s / W |
|---|---|---|---|
| 0.8 mm | 2 | 0.134 | 0.134 |
| 1.2 mm | 2 | 0.167 | 0.167 |
| 1.2 mm | 3 | 0.044 | 0.047 |
