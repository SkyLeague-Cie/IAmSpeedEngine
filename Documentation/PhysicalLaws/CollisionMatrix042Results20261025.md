# Collision matrix 0.4.2 — measured baseline, 25 October 2026

**Status: FAIL / NOT QUALIFIED.** A successful Editor exit is not a successful
matrix. These results establish a reproducible failing baseline and preserve
the individual failures; no existing accepted-debt policy waives this matrix.

The measured native source is `ca562712918010967006183b5fc1fda055d6e823`, based
on IAmSpeed main `6176be81cac530da24ce2b741d430e9736c415eb` including the reviewed
editor/server fixture guard from PR55. Subsequent reporting-only changes do not
alter the native source tested here. Windows Editor and the controller exited
0, with an exact one-test Automation report in state Fail. Canonical log
processing completed before the results were interpreted. This is not Linux
CI, a complete physics qualification, or a hardware/audio/rendering certificate.

## Execution and evidence

| Measurement | Observed |
| --- | ---: |
| Worlds created by the harness | 1 |
| Cases attempted | 4,800 |
| Cases completed | 4,792 |
| Actual / expected published frames | 100,184 / 100,200 |
| Cases with a terminal query failure | 8 |
| Cases exhausting the solver iteration budget | 10 |
| Cases with a selected CCD contact | 3,680 |
| Cases with diagnostic failures or missing required observations | 3,709 |
| Cases with native wheel suspension samples | 1,389 |
| Suspension samples | 15,981 |

All cases run sequentially in real FastSimulation. A terminal production driver
is retired after its worker joins; the next independent case starts a new test
driver in the same world. Nineteen test drivers were created, with one active
at a time. No production terminal latch is cleared and no failed frame receipt
is fabricated. Eight cases published one of their three expected frames.

The raw corpus contains `spec.json`, `cases.csv`, `events.csv`,
`initial-states.csv`, `end-states.csv`, `support-samples.csv`, `summary.json` and
`execution.json`. Their schema and frozen limits are described in
[the matrix contract](CollisionMatrix042.md). The private completion receipt is
identified by SHA256
`a51012c0a47c101f15ede6812620e31ee84ba43683a14b2e9fefc3d229b3bac4`.
The independent raw diagnostic report is identified by SHA256
`38fc548b53d5d27f4f6c850431fff1c5bcb964206154b2ba81ac57cc9c67e496`.
Neither a receipt hash nor a portable parser test substitutes for native evidence.

## Contact observations

Each cell is cases with a **selected CCD contact**, out of 320 declared cases.
Zero selected CCD contacts must not be read as zero suspension contacts.

| Dynamic fixture | Analytic plane | Static Ring | Sphere | Box | Prescribed Cylinder |
| --- | ---: | ---: | ---: | ---: | ---: |
| Sphere | 281 | 279 | 275 | 288 | 281 |
| Box | 288 | 286 | 288 | 287 | 283 |
| Wheel | 0 | 0 | 275 | 288 | 281 |

Wheel intentionally has no ordinary `SweepTOI` path. Its separate native
suspension path produced support samples in 271 plane cases, 275 ring cases,
275 sphere cases, 288 box cases and 280 cylinder cases. There were 546 cases
with support observations and no selected CCD event. A frame-end sample time
and the native support query parameter are **not** relabeled as a CCD TOI.
The missing CCD contract remains visible rather than being waived by support.

The initial cylinder fixture accidentally retained Ignore in one side of the
native response containers. Explicitly authoring blocking responses corrected
the fixture; the previous raw failure was retained. No production collision
filter or solver was changed to obtain these contacts.

## Frequency and translation differences

The matrix compares each scenario with its 1440 Hz counterpart, and each
translated case with its origin counterpart at the same frequency. Initial
position equivalence was independently verified: the maximum residual after
removing the common translation was `5.960487214906607e-9` cm, below the
predeclared `2e-7` cm input check. The environment was translated with the actors.

Independent diagnostics found 59 frequency point differences, 178 frequency
TOI differences, 140 frequency normal differences, 329 immediate linear
response differences and 1,052 linear response differences at the common
50 ms sample, beyond their frozen limits. Translation diagnostics found 1,181
point differences and 1,552 time differences. Categories overlap; incomplete
episodes and missing reference contacts are counted separately and are not
invented numerical deltas. The native aggregate gates remain failed.

The large-coordinate and fractional-grid cases are retained. The existing
absolute position quantizer uses a 0.01 cm grid and an intermediate float in
`RoundVectorToNetQuantize`. This is a source-supported candidate explanation
for part of the translation sensitivity, not an isolated causal proof for every
failure. No grid, tolerance, physics rate in production or quantizer was changed.

## Concrete reproductions and limits

Terminal cases are: 439 Sphere/Ring, 1398–1399 Sphere/Cylinder, 2037–2038 and
2040 Box/Ring, and 4598–4599 Wheel/Cylinder. They all use variant 2 (4,600 cm/s)
at 60 Hz; `cases.csv` gives the exact common translation index and published
frame count. The terminal query is retained as a failure rather than a miss.

The corpus varies authored speed, angular speed, initial rotation, relative
offset, rate and common translation in a sparse design. Ring is static and
Cylinder prescribed; free dynamic Ring/Cylinder and Wheel/Wheel are not
qualified by these 4,800 cases. Production remains 300 Hz. The matrix's test-only
episode rate hook does not qualify production multirate simulation or rollback.

Portable CI checks passed 60 tests (23 artifact contracts and 37 runner
contracts). Their synthetic fixtures are explicitly marked and rejected as
native qualification evidence. CI independently recomputes the raw physical
gates, exports diagnostics even when the matrix fails, and cannot inherit a
PASS from a successful Editor exit or another Automation test.
