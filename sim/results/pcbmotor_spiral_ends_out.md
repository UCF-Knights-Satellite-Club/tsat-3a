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
| jlc4_2oz_out | 0.5 | 20 | series | 61.1 | 7.11 | 18.7 | 1503 | 32 | 0.09 | 0.73 |
| jlc4_2oz_out | 0.5 | 20 | parallel | 30.5 | 1.78 | 18.7 | 3006 | 64 | 0.09 | 0.73 |
| jlc4_2oz_out | 0.6 | 18 | series | 52.9 | 5.15 | 19.1 | 1734 | 37 | 0.13 | 0.75 |
| jlc4_2oz_out | 0.6 | 18 | parallel | 26.5 | 1.29 | 19.1 | 3467 | 74 | 0.13 | 0.75 |
| jlc4_2oz_out | 0.8 | 14 | series | 41.8 | 3.05 | 19.6 | 2196 | 47 | 0.24 | 0.83 |
| jlc4_2oz_out | 0.8 | 14 | parallel | 20.9 | 0.76 | 19.6 | 4391 | 94 | 0.24 | 0.83 |
| jlc4_2oz_out | 1 | 11 | series | 34.5 | 2.00 | 19.9 | 2659 | 57 | 0.39 | 0.95 |
| jlc4_2oz_out | 1 | 11 | parallel | 17.3 | 0.50 | 19.9 | 5317 | 113 | 0.39 | 0.95 |
| jlc4_2oz_out | 1.2 | 10 | series | 29.4 | 1.43 | 20.1 | 3118 | 67 | 0.57 | 1.13 |
| jlc4_2oz_out | 1.2 | 10 | parallel | 14.7 | 0.36 | 20.1 | 6237 | 128 | 0.57 | 1.13 |
| jlc4_2oz_out | 1.6 | 8 | series | 22.7 | 0.83 | 20.4 | 4039 | 86 | 1.05 | 1.59 |
| jlc4_2oz_out | 1.6 | 8 | parallel | 11.4 | 0.21 | 20.4 | 8077 | 128 | 1.05 | 1.59 |
| jlc4_2oz | 0.5 | 20 | series | 61.1 | 4.74 | 22.9 | 1503 | 32 | 0.11 | 0.54 |
| jlc4_2oz | 0.5 | 20 | parallel | 30.5 | 1.18 | 22.9 | 3006 | 64 | 0.11 | 0.54 |
| jlc4_2oz | 0.6 | 18 | series | 52.9 | 3.43 | 23.3 | 1734 | 37 | 0.17 | 0.58 |
| jlc4_2oz | 0.6 | 18 | parallel | 26.5 | 0.86 | 23.3 | 3467 | 74 | 0.17 | 0.58 |
| jlc4_2oz | 0.8 | 14 | series | 41.8 | 2.03 | 24.0 | 2196 | 47 | 0.31 | 0.71 |
| jlc4_2oz | 0.8 | 14 | parallel | 20.9 | 0.51 | 24.0 | 4391 | 94 | 0.31 | 0.71 |
| jlc4_2oz | 1 | 11 | series | 34.5 | 1.33 | 24.4 | 2659 | 57 | 0.50 | 0.88 |
| jlc4_2oz | 1 | 11 | parallel | 17.3 | 0.33 | 24.4 | 5317 | 113 | 0.50 | 0.88 |
| jlc4_2oz | 1.2 | 10 | series | 29.4 | 0.95 | 24.6 | 3118 | 67 | 0.75 | 1.12 |
| jlc4_2oz | 1.2 | 10 | parallel | 14.7 | 0.24 | 24.6 | 6237 | 128 | 0.75 | 1.12 |
| jlc4_2oz | 1.6 | 8 | series | 22.7 | 0.55 | 25.0 | 4039 | 86 | 1.37 | 1.73 |
| jlc4_2oz | 1.6 | 8 | parallel | 11.4 | 0.14 | 25.0 | 8077 | 128 | 1.37 | 1.73 |

## As built (real spirals, JLC 0.3 / 0.56 mm vias, ring and leads) against the wave

| Design | Stackup | Kt | R Ω | Km | No-load rpm | H mN·m·s | Eddy + circ @2000 | Loss | Ls µH | PWM ripple 50 / 100 kHz | Board |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Spiral w1.2 r21–41, 6 turns, series | jlc4_2oz_out | 19.2 | 1.124 | **14.8** | 4793 | 102 | 0.70 W | **1.74 W** | 19.7 | 0.19 / 0.05 W | 87.8 mm |
| Spiral w0.8 r15–42, 13 turns, parallel | jlc4_2oz_out | 21.8 | 0.792 | **20.0** | 4213 | 90 | 0.34 W | **0.91 W** | 19.4 | 0.14 / 0.04 W | 89.8 mm |
| Spiral w1.2 r21–41, 6 turns, series | jlc4_2oz | 19.2 | 0.736 | **18.2** | 4793 | 102 | 0.91 W | **1.58 W** | 19.7 | 0.12 / 0.03 W | 87.8 mm |
| Spiral w0.6 r15–42, 16 turns, parallel | jlc4_2oz | 27.7 | 0.870 | **24.3** | 3308 | 71 | 0.24 W | **0.62 W** | 29.2 | 0.08 / 0.02 W | 89.8 mm |
| Spiral w0.8 r15–42, 13 turns, parallel | jlc4_2oz | 21.8 | 0.528 | **24.5** | 4213 | 90 | 0.45 W | **0.82 W** | 19.4 | 0.10 / 0.03 W | 89.8 mm |
| Spiral w1 r15–42, 11 turns, parallel | jlc4_2oz | 17.9 | 0.359 | **24.4** | 5132 | 109 | 0.72 W | **1.10 W** | 12.5 | 0.16 / 0.04 W | 89.8 mm |
| Spiral w0.8 r15–44, 14 turns, parallel | jlc4_2oz | 25.0 | 0.589 | **26.6** | 3672 | 78 | 0.45 W | **0.77 W** | 23.1 | 0.09 / 0.02 W | 93.8 mm |
| Wave n48, 2 strands swapped, ideal (5f) | jlc4_2oz_out | 23.0 | – | 18.8 | ~4000 | 85 | 0.13 W | 0.77 W | 8.1 (5h) | 1.05 / 0.26 W (5h) | 90 mm (ideal, no jumpers) |
| Wave n48, 2 strands swapped, as built (through vias) | jlc4_2oz | 22.4 | – | 19.7 | ~4000 | 85 | 0.16 W | 0.74 W | 8.1 (5h) | 1.05 / 0.26 W (5h) | ~129 mm |
| Wave n48, 2 strands swapped, as built (blind vias) | jlc4_2oz | 22.6 | – | 21.2 | ~4000 | 85 | 0.16 W | 0.66 W | 8.1 (5h) | 1.05 / 0.26 W (5h) | ~117 mm |

Circulating current between the two parallel coils of a phase with the rotor tilted (gap ±dg at opposite coils), at 2000 rpm:

| Design | ±0.05 mm | ±0.1 mm |
|---|---|---|
| w0.8 r15–42 jlc4_2oz_out | 1.40 mW | 5.59 mW |
| w0.6 r15–42 jlc4_2oz | 2.06 mW | 8.25 mW |
| w0.8 r15–42 jlc4_2oz | 2.11 mW | 8.45 mW |
| w1 r15–42 jlc4_2oz | 2.10 mW | 8.42 mW |
| w0.8 r15–44 jlc4_2oz | 2.44 mW | 9.75 mW |


## Stranded (strands swap lanes at every layer via, bundle.py), r_in 15, r_out fitted to 90 mm

JLC 4L 2 oz on all layers, JLC 0.3 / 0.56 mm vias, ring and leads as above, strand gap 0.15 mm. The first row is the solid 0.8 mm coil built by the same code. Drag = eddy + circulating between strands, at 2000 rpm. "Untransposed" is the circulating loss if each strand kept its lane on every layer. Rows in **bold** land in the 4000–4300 rpm no-load window (H about 85–90 mN·m·s). bundle.py cannot lay out 3 × 1.0, 3 × 1.8, 4 × 1.6 or 4 × 2.0 mm bundles (via row in the opening), so they are missing.

| Strands × width | Bundle | r_out | Turns | Coils | Kt | R Ω | Km | No-load rpm / H | Eddy | Circulating (untransposed) | Drag @2000 | Loss | Ls µH | PWM ripple 50 / 100 kHz |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| **1 × 0.8 mm** | 0.8 mm | 41.75 | 13 | parallel | 21.4 | 0.524 | 24.1 | **4297 / 92** | 0.399 W | 0.00 mW | 1.91 mN·m | **0.79 W** | 19.0 | 0.11 / 0.03 W |
| 1 × 0.8 mm | 0.8 mm | 41.75 | 13 | series | 42.7 | 2.060 | 24.3 | 2148 / 46 | 0.399 W | 0.00 mW | 1.91 mN·m | 0.78 W | 75.9 | 0.04 / 0.01 W |
| 1 × 1.2 mm | 1.2 mm | 41.35 | 9 | parallel | 14.3 | 0.253 | 23.2 | 6434 / 128 | 0.933 W | 0.00 mW | 4.46 mN·m | 1.35 W | 8.3 | 0.24 / 0.06 W |
| 1 × 1.2 mm | 1.2 mm | 41.35 | 9 | series | 28.5 | 0.981 | 23.5 | 3217 / 69 | 0.933 W | 0.00 mW | 4.46 mN·m | 1.34 W | 33.3 | 0.07 / 0.02 W |
| **2 × 0.3 mm** | 0.75 mm | 41.80 | 13 | parallel | 22.5 | 0.715 | 21.7 | **4077 / 87** | 0.044 W | 0.01 mW (15 mW) | 0.21 mN·m | **0.52 W** | 19.1 | 0.14 / 0.03 W |
| 2 × 0.3 mm | 0.75 mm | 41.80 | 13 | series | 45.0 | 2.826 | 21.9 | 2039 / 43 | 0.044 W | 0.01 mW (15 mW) | 0.21 mN·m | 0.51 W | 76.6 | 0.05 / 0.01 W |
| **2 × 0.325 mm** | 0.8 mm | 41.75 | 13 | parallel | 21.4 | 0.636 | 21.9 | **4299 / 92** | 0.054 W | 0.01 mW (18 mW) | 0.26 mN·m | **0.52 W** | 18.0 | 0.14 / 0.03 W |
| 2 × 0.325 mm | 0.8 mm | 41.75 | 13 | series | 42.7 | 2.511 | 22.0 | 2149 / 46 | 0.054 W | 0.01 mW (18 mW) | 0.26 mN·m | 0.52 W | 71.9 | 0.05 / 0.01 W |
| 2 × 0.375 mm | 0.9 mm | 41.65 | 11 | parallel | 18.9 | 0.492 | 22.0 | 4858 / 104 | 0.072 W | 0.02 mW (21 mW) | 0.35 mN·m | 0.54 W | 14.5 | 0.16 / 0.04 W |
| 2 × 0.375 mm | 0.9 mm | 41.65 | 11 | series | 37.8 | 1.933 | 22.2 | 2429 / 52 | 0.072 W | 0.02 mW (21 mW) | 0.35 mN·m | 0.53 W | 58.1 | 0.05 / 0.01 W |
| 2 × 0.425 mm | 1 mm | 41.55 | 10 | parallel | 17.0 | 0.398 | 22.0 | 5395 / 115 | 0.096 W | 0.03 mW (26 mW) | 0.46 mN·m | 0.56 W | 11.0 | 0.22 / 0.05 W |
| 2 × 0.425 mm | 1 mm | 41.55 | 10 | series | 34.0 | 1.560 | 22.3 | 2697 / 58 | 0.096 W | 0.03 mW (26 mW) | 0.46 mN·m | 0.55 W | 43.9 | 0.07 / 0.02 W |
| 2 × 0.525 mm | 1.2 mm | 41.35 | 9 | parallel | 14.3 | 0.284 | 21.8 | 6440 / 128 | 0.156 W | 0.07 mW (41 mW) | 0.74 mN·m | 0.63 W | 7.7 | 0.30 / 0.07 W |
| 2 × 0.525 mm | 1.2 mm | 41.35 | 9 | series | 28.5 | 1.104 | 22.2 | 3220 / 69 | 0.156 W | 0.07 mW (41 mW) | 0.74 mN·m | 0.61 W | 30.8 | 0.09 / 0.02 W |
| 3 × 0.3 mm | 1.2 mm | 41.35 | 8 | parallel | 13.9 | 0.313 | 20.3 | 6613 / 128 | 0.041 W | 0.10 mW (34 mW) | 0.20 mN·m | 0.59 W | 7.9 | 0.30 / 0.07 W |
| 3 × 0.3 mm | 1.2 mm | 41.35 | 8 | series | 27.8 | 1.221 | 20.5 | 3307 / 71 | 0.041 W | 0.10 mW (34 mW) | 0.20 mN·m | 0.58 W | 31.7 | 0.09 / 0.02 W |
| 2 × 0.725 mm | 1.6 mm | 40.95 | 6 | parallel | 10.0 | 0.158 | 20.6 | 9173 / 128 | 0.291 W | 0.36 mW (54 mW) | 1.39 mN·m | 0.82 W | 3.9 | 0.56 / 0.14 W |
| 2 × 0.725 mm | 1.6 mm | 40.95 | 6 | series | 20.0 | 0.604 | 21.0 | 4587 / 98 | 0.291 W | 0.36 mW (54 mW) | 1.39 mN·m | 0.80 W | 15.7 | 0.17 / 0.04 W |
| 3 × 0.433 mm | 1.6 mm | 40.95 | 6 | parallel | 10.0 | 0.173 | 19.6 | 9176 / 128 | 0.093 W | 0.46 mW (59 mW) | 0.45 mN·m | 0.68 W | 4.3 | 0.49 / 0.12 W |
| 3 × 0.433 mm | 1.6 mm | 40.95 | 6 | series | 20.0 | 0.662 | 20.1 | 4588 / 98 | 0.093 W | 0.46 mW (59 mW) | 0.45 mN·m | 0.65 W | 17.2 | 0.15 / 0.04 W |
| 3 × 0.567 mm | 2 mm | 40.55 | 5 | parallel | 7.7 | 0.115 | 18.6 | 11890 / 128 | 0.167 W | 1.32 mW (99 mW) | 0.80 mN·m | 0.82 W | 2.7 | 0.74 / 0.18 W |
| 3 × 0.567 mm | 2 mm | 40.55 | 5 | series | 15.4 | 0.435 | 19.1 | 5945 / 127 | 0.167 W | 1.32 mW (99 mW) | 0.80 mN·m | 0.78 W | 10.8 | 0.22 / 0.06 W |
| 4 × 0.338 mm | 1.8 mm | 40.75 | 5 | parallel | 8.4 | 0.145 | 18.0 | 10925 / 128 | 0.050 W | 1.02 mW (62 mW) | 0.24 mN·m | 0.75 W | 3.0 | 0.75 / 0.19 W |
| 4 × 0.338 mm | 1.8 mm | 40.75 | 5 | series | 16.8 | 0.554 | 18.4 | 5462 / 117 | 0.050 W | 1.02 mW (62 mW) | 0.24 mN·m | 0.71 W | 12.0 | 0.24 / 0.06 W |

Eddy against the solid coil of the same bundle width, model vs (strand / bundle)² × copper fraction:

| Bundle | Strands | Eddy ratio | (w_s / W)² × n w_s / W |
|---|---|---|---|
| 0.8 mm | 2 | 0.134 | 0.134 |
| 1.2 mm | 2 | 0.167 | 0.167 |
| 1.2 mm | 3 | 0.044 | 0.047 |
