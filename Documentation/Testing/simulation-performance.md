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
When both audit and shadow comparison are disabled, query counters remain
available, but legacy diagnostic hashes and multi-hit sorting are skipped.
This path adds no cache or persistent memory and does not alter physical hits.

Real-time frame starts retain recoverable wall-clock lateness and spread it over
`p.IAmSpeed.Simulation.RealTimeRecoveryWindowMs`, which defaults to 10 ms (three
intervals at 300 Hz). The fixed simulated delta never changes. Recovery debt is
bounded to two frames and each interval can be shortened by at most 25%, so an
OS wake-up delay is smoothed while a large hitch cannot cause an unbounded
catch-up burst. Set the recovery window to zero to disable this policy.

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

Extruded-quintic traversal also tests the continuously swept shape against
surviving node bounds after the cheap AABB rejection. Rotated thin boxes can
exclude unrelated facets this way. The full [0,1] interval and left-first
traversal are retained, with no persistent memory added. A deterministic
600-query oracle compares contacts with deliberately unpruned node/facet bounds.
OBBs also reject individual facets within surviving leaves before their more
expensive plane/triangle sweeps. Sphere/ray queries keep their cheaper node-level
pruning. Each chord stores one derived AABB (56 bytes), rebuilt with the immutable
query representation, never serialized or hashed. Development world builds report
the allocation via `AnalyticQueryStorage`; this counter and log do not exist in
Shipping. The storage grows with the authored approximation, not elapsed frames.

Bounds traversal lazily prepares its nine OBB/world cross-axis projections once
per query, in bounded stack storage below 1 KiB. Node-dependent expressions and
SAT axis order are unchanged. Extruded facets and their triangles also share the
same immutable box context, while standalone planes prepare their own context.
No transformed query/component state is retained after the query completes.

Misses from synthetic extruded facets return before stable-ID hashing and hit
metadata decoration. Tensor approximation triangles likewise receive their
stable ID only after a geometric hit and before any provider arbitration;
their face normal is prepared only by sphere/ray paths that need it. Finite
domains, filters, triangle order and all accepted witness fields are unchanged.

Ray/sphere sweeps against synthetic planes also reuse one call-local hit scratch
per extrusion. The boolean kernel leaves it untouched on a miss and initializes
every field on a hit, so previous provider decoration cannot leak into the next
candidate. The existing result-returning wrapper preserves its default miss;
OBB and coplanar-union behavior are unchanged. An independent 4096-query oracle
compares every result field and checks poisoned scratch preservation. The local
kernel gain is small and does not establish a uniform whole-frame speedup.

Tensor positions and derivatives reuse two call-local control-polygon scratch
arrays. De Casteljau reduces each populated row in place instead of copying it
to another temporary, preserving every interpolation and its order. Supported
degrees (1..15) fit the inline storage; public evaluation still retains the
previous dynamic-array path for larger degrees. Authored controls remain
untouched and no results survive the call. An independent copy-then-reduce
oracle checks output bits, clamping, signed zeros and asymmetric degrees.

Non-degenerate rectangular plane domains emit their two known triangles
directly, preserving the original ear-clipping diagonal, polygon indices and
vertex arithmetic. Authored polygons and numerically collapsed rectangles
retain generic ear clipping. A 1200-sweep oracle compares ordinary, rotated
and thin rectangles with the same domains supplied as explicit polygons.

Piecewise providers reject non-intersecting query AABBs before entering their
instrumented sweep routine. This uses the same conservative provider bounds
as the inner sweep, preserving provider order and final hit arbitration while
avoiding result initialization and traversal setup for distant providers.

The query service prepares an immutable ordered provider index at construction.
A 16-by-16 grid covers the two widest world axes; at most 16384 memberships are
stored. Large providers and saturation retain conservative global candidates.
Invalid/small/degenerate worlds use the linear policy; large or invalid queries
retain all candidates. A caller-local bitset restores canonical provider order
before the unchanged exact bounds/filter/kernel tests. Storage depends on the
world, never elapsed frames. This policy is chosen polymorphically once, not by
checking provider subtypes in the hot loop. Independent exhaustive tests cover
axis permutations, extreme scales, tangencies, invalid bounds and saturation.

Bicubic normal evaluation uses fixed-size derivative scratch and the same
de Casteljau reduction helper as generic degrees. Control differences, scaling,
interpolation and normal construction retain their original order; no expanded
polynomial or cached surface data is introduced. Degrees remain mutable authored
data, with the generic path preserved. The independent 1024-net normal oracle
and paired microbenchmark protect output bits and expose local cost separately
from full-frame throughput. Windows validation covers the 26-case matrix,
eight wake cases, four RealTime scenarios and player camera/throttle/pause flow.

An analytical miss is also authoritative when the swept broad phase cannot
reach an incomplete source, or when every reached source has a closed residual
triangle certificate. Hybrid must not execute an Unreal fallback merely to
confirm empty space already excluded by those certificates.

The analytical sweep result cache has 64 slots in 16 four-way buckets, bounded
to 64 KiB per world. Hashing only selects a bucket: full exact query equality is
required for reuse. Collisions and eviction cause recomputation, never approximate
hits or changed provider ordering. Projection uses typed sphere/box views rebuilt
with the body registry instead of allocating and classifying bodies each frame.

`p.IAmSpeed.AnalyticWorld.CacheProfile=1` reports exact cache hits/misses by
shape, hit-way distribution and evictions every 1000 eligible requests through
`AnalyticQueryCacheProfile`. It uses at most 128 bytes per query service and
does not sample clocks or alter eviction. Empty-filter rejections and direct
detailed sweeps are excluded; partial final windows are not emitted. Disabling
the CVar discards a partial window on the next eligible query. The CVar, counter
storage and reporting code are excluded from Shipping. Counts attribute reuse,
not saved CPU time, and enabled reporting is not a normal throughput benchmark.

`FSimulationSleepState` is a derived, non-serialized optimization. A virtual
support certificate, an unchanged completed pose, and exactly zero linear/angular
velocities AND accelerations are required. Restore invalidates eligibility.
Eligible bodies omit redundant rigid integration/projection work, but input,
force, contact detection, support maintenance and post-integration hooks remain
active so external interactions can revoke eligibility. This is partial sleep,
not a promise of zero CPU. Unsupported bodies remain awake by default.
Development Fast runs expose aggregate `[SimulationSleep] IntegrationSkips`;
the instrumentation and its storage are excluded from Shipping.

`p.IAmSpeed.Simulation.ActorProfile=1` enables opt-in Fast-run attribution by
stable actor ID and phase (Prepare/Reset/Sweep/Integrate/Projection/Post).
`SimulationActorProfile` emits totals once per run, including sub-body count and
static query count. Storage is fixed below 16 KiB for IDs 1..63; unsupported IDs
increment the explicitly reported overflow counter. Timings include diagnostic
overhead and must not be used as the normal performance baseline. The same
switch emits `SimulationFrameProfile` for the complete canonical pipeline:
Initialize, Prepare, Core, Snapshot, Publish, Journal and Finalize. These totals
are inclusive: actor phases belong to Prepare/Core, and SnapshotBodies,
SnapshotPairs and SnapshotHash belong to Snapshot. Do not sum parents and
children. Both diagnostics, their CVar and fixed storage do not exist in
Shipping. The normal disabled path does not sample their clocks.

Snapshot hashing preserves the original byte-wise FNV-1a result and payload
format. Eight zero bytes can be processed as one multiplication by prime^8
modulo 2^64; nonzero blocks and tails keep the original byte order. Unaligned
loads use memcpy and never read past the payload. This adds no cache or
persistent memory. An independent original-loop oracle covers dense, sparse
and zero payloads, short/large lengths and all tested alignments.

The snapshot writer appends the 19 kinematic scalar fields in one block with
their original field/byte order. It does not serialize the native struct or
its padding. A two-body oracle checks 1024 states, including signed zeros,
subnormals and the second body's unaligned payload offset, against individual
scalar writes. No persistent buffer or snapshot schema change is introduced.

Queries whose mutual collision filter cannot block anything return a miss before
analytical traversal or cache insertion. Dynamic sub-body overlaps are separate
and remain active. Legacy trace parameters are constructed only when an actual
Unreal sweep (including explicit shadow replay) needs them. Canonical queries
reuse the frame's bridge only for that same UWorld; other calls resolve normally.

Dynamic sweeps borrow their ordered external-body arrays by const reference;
the registry applies mutations outside candidate traversal. Implementations of
GetExternal*SubBodies must return persistent storage, not a temporary. Empty
candidate lists return before constructing box/sphere geometry, preserving each
sweep's existing miss-output contract. No candidates or events are reordered.
Each box/sphere/wheel candidate resolves its weak UObject reference once into
a call-local pointer. Invalid/garbage objects and the same ignore/virtual skip
policies are still checked; the winning result retains its original weak
identity. No raw pointer is cached across sweeps. Candidate tests cover the six
shape paths, equal-time ordering, exclusions and subsequent lifetime invalidation.

Sphere/box partner loops can use `TryIntersectNextFrame`: a miss leaves the
caller's scratch untouched and only a successful call writes a complete hit.
Scratch is local to the loop and read only when the returned bool is true.
The historical value-returning API still returns a default result for a miss.
`TrySphereBoxBits` compares 2048 cases with the pre-refactor CCD control flow,
including all result fields, poisoned scratch, initial overlap, invalid step
counts and non-unit rotations. `TrySphereBoxCost` has no timing pass threshold.

Sphere/OBB distance-only pretests do not construct an unused world-space
contact point. The signed-separation routine transforms that optional witness
only when requested; the face order, separation and requested witness retain
the original bits. Identical start/final evaluations within a sweep are reused
locally, without persistent state. `SeparationWitnessBits` covers 1024 poses
and nine points per pose, including non-unit quaternions and signed zeros.
`SeparationWitnessCost` alternates five paired micro-measurements with equal
checksums; its times are diagnostic, not pass/fail thresholds or frame throughput.

Hoisting repeated quintic powers was rejected after its independent derivative
oracle detected changed output bits with the current compiler. The original
Pow/Square expressions and `ExtrudedDerivativeBits` oracle remain. Shared
sphere bounds and early normal arbitration were also measured and removed:
their exact targeted runs did not establish a net frame-time gain.

Compact extruded providers apply their exact bounds and collision filters
before entering the facet kernel. This avoids its unused miss result and
instrumentation scope; accepted candidates keep the same traversal and
arbitration order. The unpruned extrusion oracle covers the outer compact
hierarchy, provider bounds, section nodes and individual facet bounds.

The frame-local resolved-pair membership set reserves its ordinary 128-entry
capacity at the first actual insertion, not at every frame start. It remains
the same TSet with no iteration-driven physical order or storage retained
between frames. `ResolvedPairSet` checks duplicate keys, growth beyond 128,
membership and capacity against the eagerly reserved set. Container storage
accounting includes Unreal's inline hash; it is not a heap-allocation counter.

Origin-position and velocity getters reconstruct only the requested field.
A local COM-to-origin view shares the original formulas with the complete
state getter; current-frame recorded history still takes precedence over live
state. No persistent cache is added. `OriginGetterBits` compares 1024 full and
partial projections with an independent oracle, including signed zeros,
non-unit quaternions and deliberately different live/recorded states.
`OriginGetterCost` reports paired local timings, not whole-frame acceleration.

Per-frame sub-body exclusions reuse their array allocation while clearing
temporary contacts and restoring permanent exclusions in their existing order.
An exceptional allocation larger than 1 KiB (or the permanent list's allocation)
is released on the next reset. The common helper is shared by ordinary and
wheel reset paths; wheels still preserve their current suspension hit.

The response-container to blocking-mask conversion is shared by ordinary world
queries and box contact projection. It reads only the stored response array:
Unreal's ECC_MAX also counts a deprecated flag outside those 32 entries. On x86,
two unaligned SSE2 byte comparisons and their bit masks replace the scalar loop;
other CPU families retain the scalar implementation. Only the exact blocking
byte matches, including when other bytes are invalid enum values. Exhaustive
byte/position tests, mixed masks and container alignments protect the conversion.
There is no cache or persistent storage. Paired microbenchmarks and repeated
Windows scenarios establish a local gain, not cross-platform throughput.

Pure rotation conversions use tiny four-entry thread-local caches (each below
512 bytes). Full input bits, including signed zero, must match. Quaternion
quantization retains the original normalize/compress/decompress order and
reapplies the current hemisphere reference even on a cache hit. Local Euler
conversion never borrows USceneComponent's mutable rotation cache across lanes.
Independent tests compare both caches with the original operations bit for bit.

Sensor static-world queries with a provably empty filter can return before
constructing geometry or Unreal hit adapters. This is restricted to a valid
strict canonical frame; dynamic overlaps remain independent. Query/attempt/
coverage counters are preserved. Enabled audit, shadow or query-detail modes,
missing worlds and Legacy/Hybrid retain their full ordinary path, including
query hashes and diagnostics.

Common component quantization remembers only proved identity operations: all
quantized fields and the previous-rotation reference must match exactly, and
the earlier canonical quantization must have left those bits unchanged. The
cache never replaces accelerations (which are not quantized), is invalidated
on restore, and stays below 192 bytes per component outside the snapshot
payload. Wheel-suspension quantization and the wheeled CanMove gate are unchanged.

`p.IAmSpeed.AnalyticWorld.PhaseTiming=1` classifies analytical kernel time in
windows of 10000 queries; mode 2 uses 1000-query windows and also reports
bounded slow piecewise traversals and `AnalyticQueryKernelTiming` for planes,
triangulation, coplanar boundaries and extruded quintics. These kernel times
are inclusive: child work is already counted in its parent, so do not sum them.
Frequent nested clock samples can materially slow mode 2.
Cached full-query results and empty-filter
rejections do not enter these kernels. Partial final windows are not emitted;
absence of a report does not prove zero cost. These opt-in timings include
instrumentation overhead and are not normal Fast throughput measurements.
The phase diagnostic's CVar, clocks, counters and log strings are excluded from
Shipping.
