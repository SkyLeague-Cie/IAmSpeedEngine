# Owned presentation boundary (experimental)

`ISimulationPresentationProducer` is an optional values-only extension to a
completed canonical frame. It must not retain or dereference an Unreal actor,
component, live world query or mutable game-thread configuration. Game-specific
policy and payload schemas belong to the host product, not this interface.

`ASpeedSimulation::BindPresentationAtFrameBoundary` is a GT lifecycle operation.
It pauses the owned lane, waits for a numbered acknowledgment (bounded to1000ms),
resolves run-stable component IDs and supplies the exact next frame to a factory.
The new endpoint is registered before resuming; a supplied previous endpoint can
be atomically replaced in the same owner/channel slot. A null factory result or
invalid replacement preserves the previous registration. A timeout creates no
new endpoint and leaves the pause requested; it never authorizes live-state
access or an automatic partial resume. A successful operation preserves the
prior instance pause. Unreal-async hosting is unsupported because it has no
owned-lane acknowledgment.

Registration uses shared handles. The canonical lane copies them under a short
mutex before invoking producers without that mutex. Unregistration does not
destroy an in-flight values-only endpoint. EndPlay closes new bindings, joins
the worker, then releases the registry. The game-thread component must release
its own handle on destruction. No producer may outlive its owned state through
an untracked raw pointer.

Outputs are addressed by owner/channel/frame and live in the same
`FSimulationSnapshot` as body presentation. One locked publication assigns their
common serial; one GT latch preserves that packet across body/output readers.
Duplicate output addresses and mismatched frames fail publication. Presentation
body extensions, output bytes and state bytes share a bounded presentation byte
budget (64KiB by default), separate from the physical payload budget.

`StatePayload` is opaque, optional presentation restore state, separate from the
canonical physics payload/hash. For rollback, every registered producer must
validate its checkpoint before any physical restore begins. Only after all
validation and physical restoration succeed does `RestoreValidated` update
presentation state. Implementations must include settings/command history and
payload-integrity validation as needed by their schema; default implementations
reject restoration. A rejected canonical rollback is terminal on GT1 and the
owned worker, with no implicit subsequent frame/hash/output publication.

Tests: `IAmSpeed.Simulation.PresentationOutputAtomicJoin`, `WorkerBoundedPause`,
`WorkerPauseBoundary`, plus product-owned body/presentation integration tests.
These are same-run boundary contracts, not complete network rollback,
cross-platform bit identity, or gameplay-presentation acceptance.
