# Acquisition and control delivery — partial host checkpoint

The Windows raw leaf now reaches DeviceInputProducer V2 through
GameInputRawAcquisition -> RawAcquisitionJournal -> FDeviceInputProducer.
Portable/fake tests execute that path. No production Unreal host creates it
yet. This is not an integrated source candidate or a gameplay/CI qualification.

## Two owners and two clocks

FInputAcquisitionWorker schedules one IInputAcquisition at an explicit cadence
(positive microseconds, at most one second), independent of UE/physics ticks.
Only that thread may Pump the Windows acquisition object. A slow OS call skips
missed scheduling deadlines instead of generating a catch-up burst. This is
frame-rate independence, not a real-time scheduling or latency guarantee.

The selected Windows cursor stages at most64 readings. Admission checks the
whole batch against its selected device under the hotplug fence. The hub stages
the physical changes and validates all states/capacities before installing
either lane. Canonical snapshots of one reading are atomic: raw changes share
AtomicGroup=acquisition sequence, and the physical mapper evaluates only after
the group's last control. Swapping two keys OR-bound to one action therefore
does not manufacture Completed/Started. Event sources may use group0 to retain
their original individually ordered events.

Physical consumption has its own contiguous delivery sequence and physical
frame address. It preserves all changes in acquisition order; source group IDs
retain reading boundaries. NoChange acquisition adds no fictitious OS reading.
A physical frame may explicitly retain held state without a new OS reading.
The first baseline cannot carry transitions under the V2 Reset contract, so
the hub delivers its original state first and retains later real readings for
the following physical frame. This adds at most one physical frame of edge
delivery at baseline establishment; it does not discard a brief press/release.

Control readers use an independent serial cursor and acquisition identities,
never fake physical frame numbers. ControlActionReader accepts explicit
Bool-action bindings for Pause, ResetWorld and AutoControl from the same frozen
action contract. It returns ordered Started/Completed requests without invoking
game callbacks. The game must normally execute these commands on Started only,
with actual eligibility/application receipts. That executor is not implemented.
Requests retain session, producer, device generation, acquisition sequence,
ordinal and action. Reset/replacement must reject old-session requests rather
than retargeting them. A new session requires a new journal/reader.

## Pause and two fences

The simulation worker's existing full physical quiescence is unchanged. The
host must obtain it and drain/abort every reservation before calling stream
lifecycle methods. A pending or prepared publication rejects pause. The
acquisition journal is not a substitute for that proof.

SetLifecyclePaused(true) purges physical pending data and advances its
acquisition fence; controls and device discovery continue. RequestFreshResume,
issued only after the host resolves every other pause gate, authorizes logical
resume at the next genuinely fresh current reading with the new fence.
IsResumeReady acknowledges that exact logical boundary. Readings started with
an older ticket may still feed controls but cannot acknowledge physical resume.
The held baseline contains no invented edge. Inputs before this acknowledgment
are discarded from the physical lane. Real edges after it are post-resume and
must survive a delayed worker wake. SetLifecyclePaused(false) releases physical
consumption; it is not a second discard boundary. The host must reflect this
same boundary in game/UI pause state; that host coordination remains open.

Physical cancellation permanently closes that delivery owner while independent
controls remain available. Replacing an authority requires a new physical
owner/session, fresh baseline and never-reused stream epoch. The actual host
epoch allocator/AutoControl/reset wiring is still open.

Acquisition shutdown is a different fence: Stop joins the acquisition thread
before returning. The acquisition source closes consumer-visible data before
attempting SDK callback teardown. Terminal Pump rejection/exception also closes
the source before announcing Finished, so cached held input cannot survive a
dead acquisition owner. Failed teardown retains ownership for an explicit retry.
Reentrant Stop from the acquisition thread refuses before the join mutex.

## Overflow and recovery

At most256 pending physical control changes and256 control observations are
retained. Refusal installs neither a physical nor a control prefix. The Windows
owner invalidates both lanes and publishes an explicit neutral reset; recovery
requires a fresh current reading. Control history overflow reports Gap with no
partial suffix. ReadControlBaseline atomically returns the latest valid state
and matching cursor; it refuses stale state after invalidation. Resynchronizing
never invents Started/Completed for a maintained control.

## Verified scope and remaining work

Standalone MSVC probes cover fake GameInput, source tickets, raw/hub/mapper/stream
composition, short presses, atomic OR handover, capacity boundaries, pause with
pending/prepared publication, stale acquisition tickets, held resume, independent
control delivery, baseline recovery, shutdown/join and terminal failure. These
are library assertions, not actual Jump/Flip/Powerslide or camera effects.

Still required: controller/car/wheeled V2 ownership and publication hooks,
production factory/module scheduling, persistent host epochs, reset/AutoControl
replacement, GT control execution while UE is paused, saved mappings and the
SkyLeague action catalogue, ordered game mechanics and SwitchCam application
receipts/HUD, all remaining booleans, native compilation/runtime, complete CI
input migration/regressions/Gold and clean PRs. Keep the legacy UE bindings
until the complete replacement path can be reviewed and qualified together.
