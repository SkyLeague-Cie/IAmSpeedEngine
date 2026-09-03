# Simulation performance observability

IAmSpeed measures every physical `Step` independently from its Unreal callback.
This distinction is required because a fast-simulation callback may execute many
canonical physical frames, while a real-time callback normally executes one.

`FSpeedStepDiagnostics` reports the physical delta, solver iterations, component
sweep passes, static-world query counts, resolved events, and wall time split
between reset, sweep, integration, resolution, projection, and post phases.
These measurements are observability only and never enter deterministic state,
collision selection, stable identifiers, or result hashes.

Reaching the bounded CCD iteration limit emits
`[SpeedSolverIterationLimit]`. Scenario runners must treat any occurrence as a
failed physical contract, even if the scenario otherwise reaches its terminal
result.

`p.IAmSpeed.Simulation.WarningFrameFraction` defaults to `0.04` (`1/25`). A physical step
whose wall time exceeds that fraction of its simulated delta increments the
performance-warning counter. At the canonical `1/300 s` delta, the threshold is
`0.133333 ms`. This is initially a warning distribution rather than a pass/fail
gate; promotion to a hard budget requires representative measurements.

Fast simulation emits one `[FastSimulationPerformance]` summary per completed
run. It includes physical-frame count, warning-frame count, iteration-limit
count, average step time, maximum step time, warning fraction, and effective
threshold. It also reports aggregate solver-phase timings, static-query volume
and iteration maxima. It does not log every fast physical frame.

Real-time diagnostics use `[SpeedPhysicalFrame]`. Detailed per-frame reporting
is enabled by `p.IAmSpeed.Simulation.PerformanceAudit=1`; slow-frame warnings
are throttled in ordinary play. `StaticWorldQueryFrame` is emitted only when the
static-world audit or shadow mode is explicitly enabled, so selecting an
analytical authority backend does not impose per-frame logging by itself.

Hybrid coverage returns the already-evaluated authority hit through
`TrySweepAuthority`. Callers must reuse that hit rather than immediately
executing the same authority sweep again. This does not weaken the
provider-vs-draft comparison or the Unreal fallback decision.

Derived acceleration structures are part of the runtime query contract, not
of serialized provider identity. Extruded-quintic section segments use a
deterministic BVH built from their exact swept bounds. Hybrid provider
selection also reuses the certified residual-triangle winner from the
authority pass, while draft triangles remain eligible in the follow-up. These
optimizations change neither stable identifiers nor hit arbitration.

An analytical miss is also authoritative when the swept broad phase cannot
reach an incomplete source, or when every reached source has a closed residual
triangle certificate. Hybrid must not execute an Unreal fallback merely to
confirm empty space already excluded by those certificates.
