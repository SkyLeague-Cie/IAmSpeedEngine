# Same-frame input owner: portable phase 1

This unit implements the manager's bounded SAMEFRAME_V2_PORTABLE_PHASE_1 grant.
It does not install a new input path in Unreal. The accepted design is
SameFrame-v2, SHA256
25650a5ff2adeab58d8d03ef0b51b1dc937fe89fe0cb5d7625b1d4f1b8d9616a.

## Implemented authority

`FSameFrameInputOwner` uniquely owns its selected `V2::IInputProducer` and is
bound to the thread which created it. Its immutable session binding contains
adapter ID, producer ID/kind, epoch, exact action contract and processing policy.
The canonical byte fingerprint includes each binding field and each slew step.
A source replacement requires an idle/paused/quarantined boundary and a strictly
newer epoch. Producer contract/lifecycle callbacks cannot reenter and revive a
failed session. A cancellation failure retains the old source in quarantine.

`Poll(N)` is the only producer entry. It accepts exactly the next target and
refuses duplicate, pending, early, late, foreign-owner and presentation calls.
The producer must implement `IInputProducerPollFence`: it freezes actual evidence
before `Produce(N)`, then atomically validates and closes that cutoff afterwards.
The returned sequence/generation must match. Late ordinary arrivals stay in the
source for a later poll; lifecycle invalidation or overflow rejects the result.
Unadapted producers, including the current production Device host, are refused.
The additional interface is portable source infrastructure, not a claim that
GameInput has been retrofitted or qualified.

This lane accepts Device-kind identity only. The probe wraps the existing sealed
V2 TestInputProducer with a cutoff adapter; it also uses a mutex-protected source
inbox to exercise actual cross-thread arrival timing. No AI/Network V2 producer
is introduced. A source supplies an action-domain frame which is validated
against the IAmSpeed contract; raw mapping remains in the existing IAmSpeed
mapper and is not independently qualified by this unit.

## Transaction and processing

`Poll` prepares immutable requested/applied snapshots, a receipt, history entry
and opaque move-only capability before installation. `Install` exposes that
snapshot only to the owner with its token. `BeginStep` verifies the same N.
`CompleteStep` commits only a stepping transaction with the same target; there
is no input setter between installation and the first consumer. These last
operations allocate nothing and call no producer/platform/presentation code.
The probe tests this with allocation failure enabled.

The class does not execute a physical step. Its future simulation adapter must
invoke CompleteStep only after the real step and all world participants have
validated. The portable receipt is not evidence of a Jump impulse, a Powerslide
state change, or a camera toggle in Unreal.

For Axis1D actions, policy step zero applies the target directly; a nonzero step
limits quantized delta from the previous committed applied value. The first
increment is available before the same N step. Boolean slew is rejected.
Initial/fresh reset baselines apply immediately without synthetic edges. Ordered
transitions remain intact, including same-frame Started/Completed pairs.

Pause neutralizes owner boundary input immediately and hides the latest live
publication. Resume requires a fresh Reset baseline. Reset/reconnect is exercised
through that baseline or a new epoch binding. Missing/malformed/gapped data,
allocation failure after polling, and explicit abort quarantine the epoch;
the already advanced source is never polled again as if nothing happened.
No partial receipt or installed snapshot remains available. Only previously
completed history can still be read as historical evidence.

## Replay and bounds

The fixed 256-entry history retains the admitted frame, processing state before
it, applied state and completed receipt. Replay creates new owner capabilities
from retained data, recomputes the same applied values and checks exact equality.
It never calls the live producer, changes the live cursor or republishes a live
receipt. Replay receipts are marked internal with a separate pass ID. Aborting
replay quarantines the owner but still permits explicit recovery in a new epoch.
Evicted history cannot be replayed; existing immutable receipt holders remain
valid. Frame and counter exhaustion cannot wrap into reused authority.

Reads are owner-thread-only. A future presentation publication adapter is not
implemented here; this class must not be used as a cross-thread live accessor.
Quarantine covers this input transaction, not restoration of a real mutated
physics world. That restoration or terminal-world boundary remains a later
integration gate.

## Validation and exclusions

MSVC 14.38, C++17, /W4 /WX standalone probes:

- SameFrameInputOwnerProbe: 1261 checks, including the bounded history loop.
- ActionStreamProbe: 4583 checks, existing stream regression.
- InputTransactionProbe: 316 checks, existing transaction regression.

The new probe covers Jump/Powerslide/SwitchCam short taps, axes/direct/slew,
N-1/N/N+1, stale/forged/moved capabilities, invalid contracts/values/ordinals,
generation and cutoff mismatch, allocation failure, phase aborts, reentrancy,
pause/fresh held resume, new-epoch recovery, replay, history exhaustion, and
cross-thread before/after-cutoff delivery. These are portable interface outcomes,
not actual gameplay effects. The initial compile failure from using vector resize
on a non-default-constructible frame was fixed; the final run is owner-v3.

Remaining: InitialBoundary and its one-time initialization receipt; production
source cutoff adapters; existing stream/world transaction attachment; real
normalization/processing integration; owner-vs-legacy exclusion in components;
complete rollback checkpoint/presentation reconciliation; removal of both
UserInput fields; Desktop V3; native gameplay tests and full CI migration.
No component, Shot977 shim, OS backend or existing producer implementation was
changed. No UE/Docker/runtime/CI run, activation, push or merge occurred.
