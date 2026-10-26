# Collision and translation invariance candidate — 26 October 2026

This private successor starts from `b036a5bf7eba2515ab76228fb3b3e72feee1216f`.
The measured production change is `04c98c6d71db7fded2f959ec6cbcb045e4114980`.
The previous delivery branches and pull requests are unchanged.

## Analytic box contact witnesses

The box response rounded admitted curved analytic contacts onto an absolute
millimetre grid. `QuantizeVectorCm` also converted each component to float.
Consequently a common translation changed both the reported point and the
response lever arm; the old Box/Ring matrix contained 160 point violations,
including a 5.866857135 cm discrepancy at a 100,000,000 cm offset.

The initial box sweep and the established curved support refresh now preserve
the admitted provider's point, normal, time, penetration and error bound.
The legacy Unreal hit path retains its existing quantization. The exact planar
refinement, query tolerances, iteration budgets, layouts and caches are unchanged.
This is a contact precision policy change; network behaviour is not qualified.

## Measured results

The unchanged sparse matrix uses 4,800 cases, eight frequencies from 60 to
1,440 Hz, five common translations, and the original acceptance thresholds.
The existing private host also contains an unchanged cold-input test extension;
that extension is outside this test queue. No Demo or SkyLeague runtime result
is transferred to this candidate.

| Observation | Previous native reference `40fd721` | Candidate `04c98c6` |
| --- | ---: | ---: |
| Cases completed | 4,800 | 4,800 |
| Committed frames | 100,200 | 100,200 |
| Terminal query failures | 0 | 0 |
| Cases with diagnostic faults or missing observations | 2,031 | 1,916 |
| Box/Ring translation point violations | 160 | 0 |
| Box/Ring cases with any diagnostic fault | 166 | 51 |
| Missing Wheel CCD observations | 640 | 640 |
| Cases reaching the iteration limit | 436 | 436 |

All 30 targeted native regressions succeeded without warnings or errors.
The full matrix and its Automation test remain **FAIL**. Post-impact velocities
and angular velocities at the common 50 ms time still differ across some
frequencies and translations. Frame-end wheel support observations are not CCD
impact times and do not waive the missing-CCD failures.

The new matrix used one UWorld with sequential joined test drivers. The final
native sample observed root exit 0 and zero active Job members at 86.328 s.
Closure and storage reconciliation brought the final controller duration to
91.734 s; this is not a claim that the entire closure finished below 90 s.

All seven Engine pins, 780 source/config/content inputs, four module DLL pins
and metadata stayed unchanged during the native runs. Canonical readers returned
0 before interpretation. The pre-Resume controller failure in attempt 3101 was
preserved; it executed no SDK payload. No acceptance threshold was relaxed.

Evidence identities:

- Native regressions receipt SHA256:
  `910ba91c8c75590dc10c5289088f64a8890d741e995c3781c47c2141b92b5e7e`.
- Full matrix receipt SHA256:
  `189db315b3e203de0f38dfd28ba92a12ea001a1c10bde285a6929f19985c5d0d`.
- Raw diagnostics SHA256:
  `524443ec88d6f82c794f974857a5bdfa3aa461e108adc161fc839fe7ffa154c2`.

The next work is to diagnose the remaining post-impact state discrepancies with
unchanged paired inputs and thresholds. This result does not qualify the whole
physics engine, hardware, rendering, audio, Netcode, or production multirate use.

## Follow-up: keep the witness through the response

Native source `d455bca1d7a9c994c185489f03aa4b350ad0826c` also removes the
second absolute-coordinate rounding in the analytic wall impulse and projected
support consumers. The legacy Unreal response still uses its existing policy.
Before this change, a pair could report the same point and normal but acquire
different angular velocities because the impulse used a separately rounded
point. A retained pair differed by 0.329091867 rad/s after a common translation.

The same 30 native regressions pass without warnings or errors. All 4,800 matrix
cases and 100,200 frames complete with one world and no terminal query failure;
the controller closes normally in 77.953 s. The complete matrix remains **FAIL**.

| Observation | Previous reference `40fd721` | First fix `04c98c6` | Follow-up `d455bca` |
| --- | ---: | ---: | ---: |
| Cases with diagnostic faults or missing observations | 2,031 | 1,916 | 1,880 |
| Box/Ring cases with any diagnostic fault | 166 | 51 | 15 |
| Box/Ring translation point violations | 160 | 0 | 0 |
| Box/Ring immediate translation angular-velocity violations | 40 | 40 | 0 |
| Box/Ring immediate translation linear-velocity violations | 24 | 24 | 0 |
| Box/Ring common-50-ms translation angular-velocity violations | 40 | 40 | 0 |
| Box/Ring common-50-ms translation linear-velocity violations | 43 | 42 | 0 |

The remaining 15 Box/Ring cases concern common-50-ms velocity comparisons
between frequencies. Translation checks for point, normal, TOI and both actors'
post-impact/common-time velocities now show no diagnostic violations in this
320-case subset. This is not a qualification of the other shape combinations.
Wheel missing observations, iteration limits and other frequency-dependent
post-impact states remain explicit failures, with unchanged thresholds.

Follow-up evidence:

- Native regressions receipt SHA256:
  `d6885defce250375b8a0424554bda76f1feb9783711632f948eb9ba560e224f6`.
- Full matrix receipt SHA256:
  `d2172415325b528790fcc28dcfdfb4e341663d03edec08c910b8374618a265d2`.
- Raw diagnostics SHA256:
  `b194c27216c515222d67a879c0bcbe23b82ac42a6c57eb9b406cb522a0130fa1`.
