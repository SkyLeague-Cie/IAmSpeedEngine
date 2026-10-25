# Simulation architecture

IAmSpeed's 0.4 development line owns the canonical simulation timeline,
real-time pacing and bounded intra-frame executor. Physical frames are
addressed by an integer `N` and advance with a fixed `1/300 s` delta. Rendering
can read completed publications at a different rate.

The default execution host is an IAmSpeed-owned simulation thread rather than
Unreal's async-physics callback. This autonomy concerns the simulation's clock
and ownership. IAmSpeed is still an Unreal plugin: current integration uses
Unreal value types, `UObject` lifecycle, world adapters and thread/event services.
It does not yet provide a complete Unreal-free SDK.

## Real-time and Fast modes

| Mode | Physical progression | Wall-clock policy | Camera |
| --- | --- | --- | --- |
| Real-time | Consecutive canonical frames at fixed `1/300 s` | The owned lane waits for its next deadline; pause/resume and bounded lateness recovery retain the fixed physical delta | An admitted local camera can produce its result from the completed frame |
| Fast | The same canonical physical-frame path, driven by a sealed, frame-addressed run | No real-time wait between committed frames; a run controller and maximum-frame safety bound are required | Camera producers are rejected and filtered out |

`AFastSimulation` is a separate driver, not a larger physical timestep. It is
useful for scenario tests and provides an unpaced execution path for future
prediction/replay work. A completed forward Fast run does not prove network
reconciliation or rollback correctness.

Fast does not present a player camera. Canonical snapshots and input/state
validation remain active. The generic presentation-extension interface can
still execute non-camera producers that a host has explicitly registered;
camera exclusion does not remove the physical pipeline.

Choose Fast before the simulation actor is created. The default game mode
selects `ARealTimeSimulation` or `AFastSimulation` at world startup. A Fast run
waits for complete scenario admission and fails if its frame budget is exhausted
before a terminal result; incomplete input is not converted into a successful
shorter run.

## One owner, joined work within a frame

The canonical owner executes the ordered frame transaction:

1. Admit frame `N` and obtain its frame-addressed inputs.
2. Install inputs and prepare the registered components.
3. Run the physical world step, including its ordered CCD/solve work.
4. Capture the completed physical snapshot.
5. Produce admitted presentation outputs and validate input completion.
6. Publish the common frame packet, commit the frame and advance its journal.

Independent calculations can run on other threads within these phases. They
join before their results are consumed at the original owner boundary. The
implementation does not parallelize consecutive dependent physical frames or
reorder contact effects to obtain throughput.

The canonical lane is implemented by
[`FSimulationWorker`](../Source/IAmSpeed/World/Simulation/SimulationWorker.h).
It still uses Unreal's `FRunnableThread`/event integration. The separate
[`FrameParallelExecutor`](../Source/IAmSpeed/World/Simulation/FrameParallelExecutor.cpp)
owns a bounded pool of `std::thread` workers with standard C++ synchronization;
it does not dispatch intra-frame work to Unreal TaskGraph.

## Parallel work and its boundaries

| Work | What may run independently | What remains ordered |
| --- | --- | --- |
| Component/sub-body preparation | Supported values-only kinematic calculations over captured inputs | The original virtual operation, wheel correction, callbacks and reset/publication boundaries |
| Actor kinematic waves | An opt-in correctness shadow; a separate opt-in can reuse validated completed base arithmetic at its native call site | Exact value and floating-point checks before consumption; changed or unsupported inputs use the original owner operation |
| Analytic world queries | Admitted independent queries, providers or complete Tensor-cell approximations, with private scratch | The original query/provider fold, search bound, contact arbitration and effect order |
| Camera in Normal mode | One certified independent producer may run while the owner performs independent input validation | Both paths join before output assembly, physical publication, commit, rollback or retirement |

Current camera dispatch preserves the serial route for unsupported or
multi-producer lists. Selecting several lanes does not make arbitrary callbacks
or every producer safe to execute concurrently.

Analytic speculative results are accepted only when their exact query,
world/invocation identity, search bound and floating-point context still match.
Rejected or unused work is reported separately from results actually consumed
by the canonical path. A completed worker task is not proof of useful speedup.

Workers have private scratch and floating-point environments. Unsafe or unknown
numeric conditions, insufficient independent work, nested dispatch and missing
capacity preserve a serial route. CCD, solve and publication barriers remain
part of the physical contract.

## Configuration

Configure host/backend capacity before simulation starts. The private pool is
not resized in the middle of a frame; changing its startup capacity requires a
new process.

| Console variable | Code default | Meaning |
| --- | --- | --- |
| `p.IAmSpeed.CanonicalFrame.Enabled` | `1` | Enable the common canonical driver |
| `p.IAmSpeed.Simulation.ExecutionMode` | `2` | `0`: Unreal async comparison host; `1`: game-thread host; `2`: owned IAmSpeed simulation thread |
| `p.IAmSpeed.FastSimulation.Enabled` | `0` | Select Fast instead of real-time at simulation creation |
| `p.IAmSpeed.FastSimulation.MaxFramesPerRun` | `1000000` | Safety bound for a sealed Fast run |
| `p.IAmSpeed.Parallel.WorkerBackend` | `0` | `0`: automatic owned pool; `1`: explicit owned pool; legacy backend `2` is rejected |
| `p.IAmSpeed.Parallel.Lanes` | `1` | Total lanes **including the canonical caller**; `1` is serial, `0` requests host policy, positive values are bounded overrides |
| `p.IAmSpeed.Parallel.MaxLanes` | `12` | Capacity limit, further bounded by observed host topology |
| `p.IAmSpeed.Parallel.ReservedPhysicalCores` | `2` | Physical cores kept outside host-policy intra-frame capacity |
| `p.IAmSpeed.Parallel.MinPrepareItems` | `8` | Minimum independent preparation items before dispatch |
| `p.IAmSpeed.Parallel.MinQueryItems` | `8` | Minimum independent query items before dispatch |
| `p.IAmSpeed.Parallel.MinFacetItems` | `32` | Minimum independent Analytic facets before dispatch |
| `p.IAmSpeed.Parallel.Camera` | `1` | Allow an eligible independent camera producer when more than one lane is selected |

Eight lanes is a measured configuration used during development, not a universal
default or automatic optimum. Eight total lanes means at most seven auxiliary
workers plus the caller. Twelve requested lanes do not prove twelve useful
participants. Host limits, workload size, dispatch cost and join tails must be
measured.

Several deeper paths remain explicit opt-ins, including
`p.IAmSpeed.Parallel.ActorKinematics1213`,
`p.IAmSpeed.Parallel.ActorBaseConsume1430`,
`p.IAmSpeed.Parallel.PiecewiseProviderWave1075` and
`p.IAmSpeed.Parallel.TensorCellWave1213`; their code defaults are `0`. Setting
`Parallel.Lanes` alone does not enable every experimental path. Keep these
switches and the exact configuration in any comparison report.

See the implementation in
[`CanonicalFrameDriver`](../Source/IAmSpeed/World/Simulation/CanonicalFrameDriver.cpp),
[`FrameParallelSettings`](../Source/IAmSpeed/World/Simulation/FrameParallelSettings.cpp)
and [`FrameParallelPolicy`](../Source/IAmSpeed/World/Simulation/FrameParallelPolicy.h).

## Coverage and measurement

Native comparison fixtures cover Normal/Fast configurations with requested lane
counts `1`, `2`, `4`, `8` and `12` on Analytic contact and grounded-wheel cases.
A separate eight-car fixture compares `1` and `8` lanes. These fixtures retain
frame-addressed consumed commands, ordered contacts, causal vehicle and wheel
states, numeric context and camera outputs according to the mode contract.
They do not imply that every vehicle count is tested against every lane count
or every authored surface.

Relevant fixtures:

- [`NativeAnalyticContactTests1221.cpp`](../Source/IAmSpeed/World/Simulation/NativeAnalyticContactTests1221.cpp)
- [`NativeAnalyticGroundedTests1368.cpp`](../Source/IAmSpeed/World/Simulation/NativeAnalyticGroundedTests1368.cpp)
- [`NativeAnalyticEightCars1534.cpp`](../Source/IAmSpeed/World/Simulation/NativeAnalyticEightCars1534.cpp)
- [`FrameParallelExecutorTests.cpp`](../Source/IAmSpeed/World/Simulation/FrameParallelExecutorTests.cpp)
- [`FrameFloatingPointContextTests.cpp`](../Source/IAmSpeed/World/Simulation/FrameFloatingPointContextTests.cpp)

Report requested lanes, admitted lanes, native participants and consumed useful
work separately. Distinguish physical-frame cost from scenario duration, owner
observations, dispatch, useful compute, imbalance and join waiting. Inclusive or
simultaneous timings cannot be added as exclusive costs. Diagnostic capture can
itself dominate a fixture and must remain visible in an instrumented result.

See [performance observability](Testing/simulation-performance.md),
[automation coverage](Testing/automation-matrix.md) and
[contract identities](Testing/contract-identifiers.md). Passing a local fixture
does not qualify the complete Stadium matrix, every callback/audio path or a
new source/platform combination. Simulating `100 ms` within a `3.333 ms`
physical-frame budget remains a target, not a guarantee of this API.

## Camera, inputs and network status

The physical camera and its game-thread adapter are described in
[CarCamera.md](CarCamera.md) and
[PresentationBoundary.md](PresentationBoundary.md). Input ownership and
frame-addressed delivery are described in [InputProducers.md](InputProducers.md).

End-to-end netcode is currently not operational in the 0.4 development line.
Restoration is targeted for 0.5. Existing input, snapshot, checkpoint and replay
seams are groundwork; they do not establish complete multiplayer prediction,
reconciliation or network rollback acceptance.
