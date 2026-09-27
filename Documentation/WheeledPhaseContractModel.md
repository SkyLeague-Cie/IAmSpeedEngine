# W5a — wheeled phase/ownership specification model

`Tests/WheeledPhaseContractModel.h` is a portable specification experiment.
It is not runtime code, a producer, an adapter, or an implementation of the
physical consumer. No production file includes it. W1 remains restrictive;
FInputFrame, InputStream and UpdateInputs are unchanged. Passing this model
does not qualify a scenario migration or establish Unreal phase order.

## Source facts and limits

All line ranges below refer to the source bytes pinned in the W5a receipt.

| Source | Relevant fact |
|---|---|
| SLTestCarMechanicsScenarios.cpp:10–44 | Case29 sets initial throttle .25, equilibrium preservation, and a key at59 with CanMove and bypass. |
| SLTestGameMode.h:124–125 | Equilibrium warmup defaults to five frames. |
| SLTestGameMode.cpp:2900–2939 | Warmup neutralizes initial setters, shifts profile keys by five, adds neutral CanMove+bypass key0, then queues the profile. |
| SpeedCar.cpp:123–141; SpeedWheeledComponent.cpp:3038–3072 | Initial setters stage quantized axes in a live mailbox. They do not immediately set physical axes. |
| SLTestGameComponent.cpp:1220–1252 | Profile installation enables test override, stores/sorts profiles and resets active commands. |
| SLTestGameComponent.cpp:1327–1332,1439–1441,1470–1495 | The runtime supplies the scenario origin. Due keys update active input; active input is requeued each PostPhysics for the next local frame. |
| SLTestGameComponent.cpp:8279–8310 | Active input retains the complete wheeled payload and bypass policy. Bypass uses QueueTestWheeledPhysicalInputForFrame. |
| SpeedWheeledComponent.cpp:1321–1358 | Due command sets UserInput; bypass copies Physical and BeforeSlew, stamps LastSlewFrame and syncs state before ordinary slew. |
| SpeedWheeledComponent.cpp:1580–1608,1622–1627,1702–1706 | Ordinary slew advances axes by16 once per frame. State sync stores three axes; RecordPhysicsState syncs again. |
| SpeedWheeledComponent.cpp:2756–2759,2914–2917 | PostGameplayTick rewrites user bCanMove from CanMove(); actual movement permission depends on the physics countdown. |

Payload CanMove is therefore NOT ownership of the movement countdown. The
model exposes an external engine-mobility observation and rejects requests to
override the countdown. It does not prove where that observation belongs in
the complete engine transaction. In particular, ordinary slew only updates
physical axes; unlike bypass, it does not copy the payload CanMove bit.

## Explicit modeled phases

1. Acquire a nonzero owner and monotonically increasing generation while idle.
   Reset model state, then stage a compiled seed. This ownership/reset protocol
   is a proposal for a future common contract, not an existing runtime API.
2. At the first physical input frame, consume the staged seed into user axes.
   Without a bypass command it advances physical axes through ordinary slew.
   A nonneutral seed at this point is incompatible with W1's neutral C0;
   it cannot be silently encoded as a W1 reset frame.
3. At each exact sequential frame, apply a due command or its held successor.
   The legacy harness requeues held input every frame. Held bypass repeats the
   full User → Physical → BeforeSlew → LastSlew → synced-axes effect each frame.
   Ordinary slew preserves BeforeSlew and defers axis sync until record.
4. An optional external mobility observation can rewrite user CanMove, without
   changing the physical payload or claiming that an input unlocked movement.
   Conflicting repeated observations on one frame reject.
5. Record syncs physical axes. A later frame requires the current record first.
   Exact repeats are idempotent; missed frames, different ownership, stale
   generations and end-of-horizon steps reject without advancing state.
6. Release neutralizes model state and invalidates that owner/generation.
   Reacquisition requires a newer generation; no held command leaks across it.

Compilation rejects nonfinite values, duplicate frames, missing/overflowing
local anchors, out-of-horizon commands, inconsistent warmup policy and declared
unmodelled actions. Signed local activation frames are bounded conservatively.
There is no fast-forward rule for multiple overdue commands: their legacy
iteration order is not silently reinterpreted as latest-wins.

This three-axis/payload-bit model requires neutral camera and no Sky, discrete
or reactive actions. The caller must declare any such stimulus unsupported.
It does not model suspension, countdown evolution, forces, rollback commit,
controller dispatch, camera state or atomic publication.

## Case29 fixture and clock

The fixture chooses a SYNTHETIC first PostPhysics local frame L0=1. It is not
a measured or qualified scenario origin. With warmup W=5, scenario key N=59
becomes local activation L0+N+W+1=66, canonical C=local−1=65. The injected
warmup key activates at C1. Initial .25 throttle is suppressed, so the seed is
neutral; the drive command later quantizes to64. Holds repeat their bypass
effects. The model separately tests a synthetic nonneutral seed without warmup.

Compile/run using RunInputProducerProbe.ps1 with
`-ProbeName WheeledPhaseContractProbe`, MSVC14.38 and a fresh artifact directory.
The result must say SPECIFICATION_MODEL_ONLY, UE=not_run, migrated=0. Negative
checks also prove W1 still rejects initial-nonneutral, CanMove and bypass.

Before production work: qualify the real W4 consumer test, pin the actual
scenario origin and engine ordering, settle reset/ownership semantics against
the canonical transaction, then test the full effect on the real component.
Only an unchanged-baseline scenario run can establish migration parity.
