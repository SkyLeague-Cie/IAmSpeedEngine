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


## Follow-up: preserve uniform unforced rotation

Native source `74175d978377fc0034491121de0fe642c3e35290` preserves the
integrated quaternion when angular acceleration is exactly zero. Repeated
packed-Euler rounding otherwise changes constant spin according to the number
of steps and pins sufficiently slow rotations. A nonzero angular acceleration
retains the existing outer rotation quantizer. Inner codec/cache, velocity
quantization, layouts and query thresholds are unchanged. This explicitly
changes the outer pose precision policy; network/full-state restore is not
qualified by this result.

The new independent axis-angle regression covers 60/100/120/240/300/360/720/
1440 Hz, three axes, two initial orientations and angular speeds 0.0001,
0.01 and 20 rad/s. All **31 targeted native regressions** succeed with no
warnings or errors. This is an actual native result, not the scalar model.

The full matrix remains **FAIL**, with 4,800 completed cases, 100,200 frames,
one world, 436 sequential joined drivers and zero terminal query failures.
Controller closure takes 74.125 s. There are still **1,880 cases with a
diagnostic fault or missing observation**; no previously faulty case becomes
entirely clean and no previously clean case becomes faulty.

| Diagnostic | Previous `d455bca` | Candidate `74175d9` |
| --- | ---: | ---: |
| Frequency common-50-ms angular-velocity violations | 631 | 621 |
| Frequency common-50-ms linear-velocity violations | 887 | 887 |
| Translation common-50-ms angular-velocity violations | 32 | 26 |
| Translation common-50-ms linear-velocity violations | 38 | 35 |
| Missing Wheel CCD observations | 640 | 640 |
| Cases reaching the iteration limit | 436 | 436 |

These category counts overlap and include mixed case-level changes: the
comparison artifact retains every removed **and added** violation. For example,
case 3129 newly exceeds frequency velocity/angular bounds while case 3134
ceases to exceed them. Both remain in the already failing Cylinder subset;
this is not a claim that every individual comparison improves.

All Box/Ring translation checks remain clear for their 320 cases; its remaining
15 failures still concern frequency common-time velocity. Sphere/Plane,
Sphere/Sphere, Sphere/Ring and Sphere/Cylinder remain clear in their respective
320-case diagnostic subsets. Wheel frame-end supports remain observations,
not fabricated CCD impact times. The spec and all acceptance thresholds are
byte-identical to the previous run.

Evidence SHA256:

- Exact31 regressions receipt:
  `405ca121ea8a4d5c3b8b26b0f2e4a7be15ab555a06c95131877bd094bfe32465`.
- Matrix receipt:
  `086b5577526a8b0ae3d24d874992d80577fc55bbd69c51f66977de27e63bf24b`.
- Raw diagnostics:
  `825adc4495cf2bd218fbbf704256f9caa02fe7eeaded5d2d9e4e9dc3cda87aff`.
- Complete case-level before/after diagnostic:
  `b95dc2421ecbfd5e75fcbd8ae48878505a6b083b7135102c76c18fe42da46aae`.

No Demo/SkyLeague adoption, public CI, release, whole-engine physics,
hardware/audio/rendering or Netcode qualification is inferred. The previous
measured candidate remains separately retained.


## Follow-up: normal impulse precision

Native source `9c887e2d0f25957a0aa61ed235de1355ed3d7cdf` avoids narrowing
reciprocal masses, normal contact velocity, effective-mass terms and default
normal/friction impulse arithmetic to float. Public signatures, layouts,
coefficients, thresholds, iteration budgets and the quantization cache are
unchanged. Alternate coupled solver policies are not separately qualified.

The new native regression applies the returned impulses through independent
Newton/Euler momentum equations and checks the restitution law. It covers
multiple masses (including a prescribed peer), two restitution values, two
normals and four translations. All **32 targeted native regressions** succeed
without warnings/errors. In 64 retained offset-zero Box/Cylinder first-impact
observations, the maximum absolute post-impact normal velocity falls from
`9.179536405667932e-5` to `3.6514843353806237e-13` cm/s. For case 3081, it changes
from `-2.743010550076974e-5` to `1.5631940186722204e-13` cm/s. These measurements
use the first actual CCD event and affine pre-impact COM trajectory; no support
sample is reinterpreted as an impact time.

This correction does **not** remove contact continuation failures. The matrix
still completes 4,800 cases / 100,200 frames in one world, with zero terminal
query failures and a 71.906 s controller closure. It remains **FAIL**:
1,880 cases have diagnostic faults/missing observations, including 640 missing
Wheel CCD observations and 436 cases reaching the unchanged iteration limit.
The current native source limit is 24; older historical labels are not current.

The before/after diagnostic is mixed, rather than uniformly greener. Translation
common-time angular violations change from 26 to 27 (case 3139 removed; 3129
and 3188 added). Translation common-time linear violations remain 35, with two
removed and two added comparisons. No previously clean case becomes faulty,
and no previously faulty case becomes fully clean. Frequency category counts
are unchanged. Spec and acceptance thresholds are byte-identical.

This measured successor retains the more accurate restitution law and the extra
native regression; the previous `722f386`/`74175d9` measured candidate remains
available, with the comparative tradeoff explicit. No whole-matrix PASS,
Demo/SkyLeague runtime adoption, public CI, release or Netcode certificate is
inferred.

Evidence SHA256:

- Exact32 regressions receipt:
  `f6b80710dc9d58795dda81fa9481068c87c0dc23d57a2666ab8c0cecaa028bfc`.
- Matrix receipt:
  `55d98f9ed5f8e7265bc709599feab2c404d9cae5c080656f68799f93e5887d53`.
- Raw diagnostics:
  `9f254fdf3eb417193eb1ed2f628975edbf455fad7d64033d3f3527dbef4ef37d`.
- Complete before/after diagnostic:
  `9a9178b765337ddf8cdfd01f3983fe63b256cc01a9be97a52bfaa51b1cd54482`.
- First-impact residual measurements before/after:
  `83e56533b2a0b0cac6de781d637b32cc13b0a5a816a9ed929ef4e52191df6dbd` /
  `2ff41684c3fc9de7fe64c5ea72cb421ab01081c5e13388b5fbba8f2ef91f38a5`.
