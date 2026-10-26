# Collision matrix 0.4.2 — prescribed fixture correction

**Status: FAIL / NOT QUALIFIED.** This is a distinct native execution of source
`e2c77d5fd92683aefcbd58f590da7abd17663ad7`, not a reinterpretation or rerun of
the [original baseline](CollisionMatrix042Results20261025.md). Only the test
owner's prescribed-state setter and free-body quantization behavior changed.
Production solver code, production 300 Hz, and the predeclared numerical gates
are unchanged.

## What the correction establishes

The stationary prescribed Cylinder kept its authored relative position, linear
velocity and angular velocity exactly in all 960 Cylinder cases. The previous
845 deflected-owner cases are therefore a corrected fixture defect, not proof
of a production Cylinder defect. This observation does not qualify free dynamic
Cylinder or Ring behavior.

The incremental Windows compilation succeeded, with native and controller exits
0. The current private Project's 779 retained input files, four module pins,
metadata and seven protected Engine pins were checked around the runtime.
There is no claim of a separate Demo or SkyLeague build from this private run.

## Execution and remaining failures

| Measurement | Observed |
| --- | ---: |
| Unreal worlds | 1 |
| Cases attempted / completed | 4,800 / 4,792 |
| Published / expected frames | 100,184 / 100,200 |
| Sequential test drivers, one active at a time | 388 |
| Terminal query cases | 8 |
| Cases exhausting the iteration budget | 380 |
| Cases with a selected CCD contact | 3,680 |
| Cases with diagnostic faults or missing observations | 3,730 |
| Cases with native wheel suspension samples | 1,380 |
| Native suspension samples | 13,715 |
| Prescribed fixture deflections | 0 / 960 |

Cases still execute sequentially in real FastSimulation. A terminal or
iteration-exhausted production driver is joined and retired before a fresh
test driver runs the next independent case in the same world. Production
terminal latches are not cleared and no failed frame receipt is fabricated.
The changed driver count and increased iteration exhaustion remain visible.

The Editor exited 0 with an exact one-test Automation report in state **Fail**.
The controller exited **1** because its elapsed-time guard sampled
`90.954 s > 90 s`. At that sample, the native root had already exited 0 and
the Job had zero active processes; all other sampled resource limits were
within their bounds. The controller checks its guard before the normal
completion branch. Final elapsed time was 96.172 s including drain and
reconciliation. The timing failure is retained; neither the native exit nor
the presence of all 4,800 rows grants a PASS.

## Frequency and common-translation evidence

The same eight rates (60 through 1440 Hz), variants and five common actor and
environment translations were used. Maximum initial translation residual was
`5.960487214906607e-9 cm`, below the original `2e-7 cm` input check.

Under the frozen limits, diagnostics report 59 frequency point differences,
178 frequency TOI differences, 140 frequency normal differences, 247 immediate
linear-response differences and 865 linear-response differences at the common
50 ms endpoint. Translation comparisons report 1,181 point differences and
1,552 TOI differences. Categories overlap; missing contacts, missing references
and incomplete episodes are recorded separately from measured differences.
The correction has not made the matrix physically invariant.

The separate suspension report includes 1,235 frequency comparisons and 1,379
translation comparisons, with endpoint velocities of both actors measured at
50 ms. First support observations may occur at different frame-end times.
Their sample-time difference and native query parameter are not CCD TOI errors.
These are measurements only, without a new threshold or PASS policy. Wheel's
analytic-ground suspension path must not be mistaken for its selected CCD
contact stream: 546 cases have support samples without a selected CCD event.

## Reproduction, provenance and scope

Run `IAmSpeed.PhysicalLaws.CollisionMatrix042.FastBatch` through the existing
CI runner and independently gate its raw `CollisionMatrix042` corpus. The
[contract](CollisionMatrix042.md) defines the schema, input matrix and frozen
gates. This remains a sparse design with three dynamic fixtures (Sphere, Box,
real Wheel) and five peers (analytic Plane, static Ring, Sphere, Box, prescribed
Cylinder), not all possible dynamic shape pairs.

The native completion receipt has SHA256
`ca3da3b6c4cbc35db3bc3071f3f537bb3393dc0492918dc4a29fa1e04e99e5d9`.
The independent raw diagnostic report has SHA256
`962e1da363260a82c90446c9500566cb593b8f6d618fe57f73cfa51417e3a245`.
The closed cycle receipt has SHA256
`724e422f2ff20b5fd59471ec408693dbe4a5178bd2bd713a4029b7a6fba4aacc`.
Original reports, rows, preimages, failures and costs are retained. The single
approved corrective native cycle is consumed; a remaining byte allowance is
not authorization to replay it.

Portable validation remains 26 artifact-contract tests and 37 runner tests,
plus 11 Demo integration-contract tests. Synthetic portable fixtures are not
native evidence. No Full physics, hardware, audio, rendering, Netcode,
production multirate, release tag or main merge is qualified here.
