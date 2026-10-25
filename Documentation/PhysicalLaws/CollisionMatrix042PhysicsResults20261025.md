# Measured IAmSpeed0.4.2 physics corrections — 25October2026

The new native candidate is `40fd72115748b4835d66a41359e32bd56f607e17`, Source tree `9d17d19b97d7b7023547c17a56dd193b1b4ab7e1`.
The final run attempted4800 cases, completed4800, selected
4160 CCD contacts and published100200/100200 frames.
Editor and controller both exited0 in72.468s under the90s guard.
The Automation matrix and independent physical gates remain **FAIL**.
The separate exact30 native regressions all succeeded without report warnings
or errors. This is bounded private Windows evidence, not Full qualification.

## Changes measured

* Keep double precision through the declared quantization-grid division at
  large coordinates; no float narrowing of world positions.
* Solve the full affine sphere/sphere crossing interval, including entry and
  exit inside a coarse frame, and return the accepted accelerated bracket root.
* Preserve the integrated affine pose on axes with zero acceleration instead
  of injecting a frame-rate-dependent absolute-grid displacement. This is an
  explicit free-flight pose-policy change. The inner quantizer/cache and
  wire/snapshot layouts remain unchanged; network equivalence is not qualified.
* Refine uncertainty-dominated CCD cells instead of prematurely returning
  Unresolved. Spatial thresholds, pose-error bounds, earliest-interval order,
  finite4096 evaluation budget and fail-closed collapse remain unchanged.
* Keep five bounded CSV writers open during the matrix and close/check them
  before summarizing, avoiding per-case reopen overhead without changing rows,
  cases, comparison thresholds or the90s external guard.

## Comparable history, including regressions

| Native stage | Complete | Cases with faults | Terminal | Translation point | Translation TOI | Translation normal |
|---|---:|---:|---:|---:|---:|---:|
| Fixture corrected baseline | 4792 | 3730 | 8 | 1181 | 1552 | 332 |
| Double/continuous sphere | 4796 | 2724 | 4 | 924 | 369 | 212 |
| Stationary axes experiment | 4797 | 2596 | 3 | 267 | 628 | 65 |
| Affine trajectory | 4785 | 2041 | 15 | 160 | 0 | 0 |
| Refined CCD candidate | 4800 | 2031 | 0 | 160 | 0 | 0 |

Categories overlap. Fault cases include missing observations and incomplete
episodes; they are not counts of independent numerical defects. A zero measured
delta category covers only comparable contacts, not missing/reference contacts.
The stationary-only experiment increased TOI failures; it is retained rather
than presented as a uniform improvement. The affine successor fixes the common
relative trajectory. The earlier guarded run and every prior FAIL remain raw.

The final comparable frequency point/TOI/normal violation counts are
0/0/0.
Current per-pair outcomes and exact raw-file digests are in
[the measured summary](CollisionMatrix042MeasuredSummary20261025.json).
The report explicitly preserves436 iteration-limit cases and
640 missing selected CCD contacts.

## Coverage and remaining limits

The sparse design varies three dynamic fixture shapes (Sphere, Box, real Wheel)
against Plane, static Ring, Sphere, Box and prescribed Cylinder, eight authored
velocity/spin/pose variants, eight rates60–1440Hz and five common translations.
It uses one Unreal world and strictly sequential FastSimulation episodes;
terminal/exhausted drivers are joined and replaced in that same world, never
cleared. It does not qualify free dynamic Ring/Cylinder or Wheel/Wheel.

Wheel frame-end support samples remain separately typed observations. Their
sample time is not a CCD TOI, and they do not waive the selected-CCD gate.
Post-impact common50ms velocities/angular velocities and iteration limits still
need work wherever the strict report is red. No tolerance was widened and no
missing result is called a successful collision or invariance comparison.

The [causal explanations](CollisionMatrix042PhysicsCorrections.md) describe
the individual fixes and their negative controls. The exact source was compiled
and tested in the private Project; vendoring it into Demo or changing SkyLeague's
gitlink does not constitute another native build, image admission or CI PASS.
Production remains300Hz. No live SkyLeague project, hardware/audio/rendering,
Netcode0.5, public release tag or main merge is qualified by this delivery.

The result-documentation commit uses `[skip ci]` so publishing a review branch
does not implicitly start an unadmitted self-hosted SDK run. The Physics Reviewer
owns branch cleanup, an explicitly admitted exact-source CI and any later main
merge. Required checks and image protections remain blocking.
