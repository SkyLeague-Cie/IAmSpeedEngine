# Physical input frames (CP1)

Status: opt-in source checkpoint. Existing player controls use the migration
adapter by default. This checkpoint is not Unreal build, control-parity,
multiplayer or rollback acceptance.

## Contract and ownership

`Input/InputFrame.h` defines a values-only immutable API with source frame,
zero-based canonical consumption frame, producer kind/ID, 32 signed 16-bit
action slots, and at most 64 ordered edges. The first three slots retain the
existing throttle/brake [0,255] and steering [-127,127] quantization. Games own
the other slots, their names and scales. No game-specific action law is added
to the engine. Native tests use Jump/Powerslide as example game slots.

`SourceFrame` belongs to the injected sample's producer clock. The real device
acquisition backend and clock remain unresolved; Unreal game frame counters
are not an authoritative input clock. Source frames are not necessarily
comparable to the physical consumption clock.
Edges carry their own source frame and retain callback insertion order,
including press-release-press in the same source frame. Edges newer than the
snapshot source, decreasing source order, invalid IDs/ranges, and nonzero
unused edge records are rejected. Never serialize/hash C++ object bytes:
padding, mutexes and pointer representations are not a wire format.

Producer identity is a nonzero, serializable numeric ID scoped to the target
stream and run, plus Device/AI/Network kind. The host owns uniqueness across
targets and restarts. This is not a network-stable actor identity.

`ASpeedController` can retain an externally configured `IInputProducer` and an
`FInputStream`, but never constructs or feeds a device source. A device adapter
accepts already mapped/filtered/quantized injected samples. Its short mutex
protects a complete held-state/edge/source transaction. A single acquisition
writer stages every action with `SetAction(SourceFrame, ...)`, then calls
`CommitSample(SourceFrame)` once. Until commit, the worker sees only the previous
committed sample. Source frames increase strictly between commits; repeated
updates and press-release-press are allowed within one staged source frame.
Any failed staged action invalidates that transaction until `CancelSample`;
no valid prefix can leak through a later commit. The worker never
samples independent atomic axes or reads EnhancedInput. Capture transactions
are independent of the GameThread. Tests inject samples directly; no OS/HID,
Enhanced Input, UObject callback or other real capture backend is supplied.
Choosing that backend requires a separate developer/lead decision. A physical
sample can fall between two complete injected transactions. Quantization rejects NaN/infinity and uses
the legacy round-to-nearest, ties toward positive infinity.

The component latches the shared stream under a lifecycle mutex at its input
frame boundary, then releases that mutex before producer consumption. Both
sides retain `std::shared_ptr` handles; no controller/UObject pointer travels
with the stream. Detach deactivates the old stream under its own mutex, including
when UnPossess occurs inside a presentation callback. Installation remains
guarded; null teardown always proceeds. An already returned frame remains valid,
but after deactivation even a latched worker cannot consume, skip or publish
through that stream. A new
possession creates a fresh stream. The controller releases its handles on
unpossession; an in-flight worker handle may outlive them without accessing it.

`FromLegacyLocalFrame` is the single named conversion from component frame
N+1 to canonical frame N; the legacy zero sentinel is rejected. Network's
`FromCanMove` uses a canonical CanMove frame with a checked signed offset.
No wall-clock/SimTime reconstruction belongs in a producer.

## Producer policies

| Situation | Device | AI / Network minimal exact-frame inbox |
| --- | --- | --- |
| New frame | First requested frame, then contiguous frames | Explicit FirstFrame, then contiguous frames |
| No new injected sample | Same held values, no repeated edges | Missing input returns no frame; no prediction |
| Duplicate/replay request | Exact retained immutable copy | Exact retained immutable copy |
| Evicted past / skipped consumption | Reject | Reject |
| Late submission | Non-increasing committed source rejected | Already consumed/suppressed target rejected |
| Duplicate submission | Allowed inside staging; committed source cannot reopen | Reject; no implicit replacement |
| Future submission | Device events carry no physical target | At most 255 frames ahead of NextFrame; out-of-order arrival allowed |
| Edge overflow | Reject whole sample transaction; caller must handle failure | Invalid payload rejected |
| Storage | 256 frames, 64 pending edges | 256 pending frames plus 256 retained frames |

AI has no planning implementation; network has no transport, authentication,
prediction, correction or loss handling. Both accept the same canonical
payload, differing only in identity and upstream policy. Live arrival timing
is not deterministic; replay of the recorded physical frames is.

## Consumption and migration

The default source is null, so existing controls retain the legacy adapter.
Native `ConfigureInputProducer` can install an independently supplied test
source only before possession; it cannot acquire or publish samples. It is not
a Blueprint switch to working device acquisition. No production caller is
added in CP1. When a source is supplied, the common driving Enhanced Input
bindings are omitted, and legacy handlers return without writing components.
There is no simultaneous legacy/new route for those axes. Derived game actions,
camera, pause, and their laws remain on their old paths; migrating those is open.

When enabled, the physical component consumes one explicitly addressed
snapshot, updates its user target values, and applies the existing slew law.
It does not also consume legacy live/network wheeled queues. Lifecycle
changes neutralize the old target and purge stale queues at the next input
boundary. Missing/invalid input produces a diagnostic and neutral targets;
it never falls back to a live Unreal action. The old queue's reverse iteration
behavior is retained only in the legacy adapter, not copied into producers.

The sealed scenario override still owns scripted frames. Explicit `Skip` returns
success/failure, checked by the component. Device advances normally. AI/network
advance the exact expected frame even when missing; any submitted payload is
retained in producer history and future pending frames remain intact. Late input
for a suppressed frame is rejected. Suppression never records/publishes that
payload as physically consumed by the stream. Gaps and terminal wrap fail.
This is a migration guard, not producer-based test-harness parity.
Execution-mode 0 is diagnostic legacy; completed input publication is wired
through the canonical simulation pipeline only.

`FInputStream::Consume` records input at the beginning of a physical frame.
It does not publish it to the GameThread. After canonical state publication
succeeds, `OnCanonicalFramePublished` publishes the matching input snapshot
through a short mutex, including a publication serial. Failed or incomplete
physical frames do not appear as completed inputs. The input stream retains
256 addressed frames; longer replay requires the simulation's sealed journal
or a future coordinated rolling history. It is not integrated into physical
snapshot serialization/hash or live correction acceptance in CP1.

## One-way presentation

`SpeedController::Tick` copies only `ReadLatest()` and invokes `HandleInputs`.
`BindAction(Name, Slot, Callback)` associates an explicitly mapped, case-sensitive
nonempty name with a valid slot and a callback receiving only a const frame and
slot ID. There is no implicit name lookup. Duplicate names, empty callbacks,
unknown slots and more than 32 bindings fail; there is no rebind operation.
Different names may observe the same slot. Registration order is dispatch order.

Each publication serial dispatches at most once. A slow GT can omit intermediate
values/edges; only the newest completed frame is observed. The physical history
is never drained or changed by polling. For reliable effects across missed
frames, a separate future presentation-event protocol would be necessary.

Callbacks are for sound/VFX and other non-authoritative presentation. They must
not bind the existing `SetThrottleInput` physical writer. A thread-scoped guard
rejects synchronous calls into producer mutation/consumption/publication and
the generic wheeled setters/queues during dispatch. Calls outside dispatch
remain valid. This is defense in depth, not a sandbox for arbitrary C++:
callbacks must not capture physical writers or schedule deferred mutations.
The callback signature exposes no component, controller or producer handle.

## Evidence and remaining gates

`Tests/InputProducerProbe.cpp` uses only injected synthetic samples, runs without Unreal and covers quantization,
copy/immutability, held replay, ordered multi-edges, Jump/Powerslide boundaries,
exact frame policies, capacity/overflow, three-producer parity, CanMove offsets,
completed publication, polling coalescence, input-history hashes, rejected
reinjection, concurrency and shared lifetime. `test_input_source_boundary.py`
checks Unreal source wiring and the opt-in/default boundaries; it cannot prove
UHT/compile/runtime integration. Run the native probe with
`Tests/RunInputProducerProbe.ps1`, supplying the local MSVC vcvars path, a fresh
artifact directory, and the parent canonical observability summarizer.

Exit contract for source review: these probes pass on immutable source SHAs,
the parent records exact gitlinks, and no default route or gameplay law is
replaced. Remaining gates are Unreal compile/link and automation, generic
device parity, full game action migration with CanMove/edge timing tests,
full regression/blocking Gold qualification, controller lifecycle/Freeplay,
snapshot/journal rollback integration, and network frame/identity policy.
The real local/client device acquisition source is an additional open gate.
The migration adapter must remain until these relevant parity gates pass.

The native Jump adapter is synthetic: it applies each exact-frame edge once
on the simulated worker, then polls presentation repeatedly without adding a
physical transition. Game-controller EnhancedInput Started/Completed bindings
for Jump have **not** been replaced. Their migration must split physical
edge handling from the existing controller methods that write the component,
then bind only presentation observers through `SpeedController::BindAction`.

Superseded checkpoint `f8ab01b` was rejected because its controller callbacks
fed DeviceInputProducer from Enhanced Input/GFrameCounter. Its native test pass
did not qualify that Unreal coupling. The corrected checkpoint removes that
entire source path; only deterministic synthetic injection is qualified here.
