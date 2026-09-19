# Portable wheeled test profile (W1)

Status: SOURCE_NATIVE_PORTABLE_ONLY. This helper is not attached to Unreal,
SLTest, a player, a physical session or V2. No existing scenario is migrated.

`Input/Testing/WheeledTestProfile.h` compiles normalized throttle/brake/steering
targets into the existing V1 `FTestInputProducer`. The native probe uses that
producer, `FInputStream` and the actual shared `ReadDrivingInputTargets` helper.
It does not invoke `USpeedWheeledComponent::ConsumeProducedWheeledInputs`, apply
physical slew, map raw devices or simulate a vehicle.

## Accepted profile

The frame count includes neutral reset frame C0. It is in [1,16384], a bounded
precomputed test timeline; longer or adaptive tests need a different contract.
Identity is nonzero and fixed. Initial throttle, brake and steering must all be
exactly zero (signed zero is accepted). Nonzero initial live setter values are
rejected rather than retimed as scenario keys. Initial NaN/infinity also return
`InitialInputNotNeutral`, intentionally before key validation; nonfinite key
values return `NonFiniteValue`.

Keys are sorted by scenario frame without modifying the caller's vector.
Duplicate frames are rejected, not resolved by incidental input order.
A key at scenario N first applies at canonical C(N+1), because scenario keys
are issued after physics. Keys must take effect inside the finite horizon.
Targets hold until the next key or the last frame; reads after the horizon fail.
Source and consumption frame numbers both equal C. Only C0 has RequiresReset;
there are no synthetic action edges. Caller mutation cannot change the output.

Every key axis is quantized once through the existing V1 QuantizeAxis contract:
reject NaN/infinity; clamp throttle/brake to [0,1] and steering to [-1,1];
multiply by 255/127 and round with floor(scaled + .5). Negative half ties go
toward positive infinity. Already quantized physical bytes must not be passed
as normalized floats. No second filtering or slew is performed by the helper.

The caller describes the entire scenario. Any wheeled/Sky slew bypass,
CanMove command, Sky input, discrete action, reactive/contact trigger or camera
input rejects the profile. Dropping unsupported fields before compilation would
violate this contract. This is preparation for eligible wheeled-only tests,
not a universal adapter for existing SLTest FTestStruct values.

## Validation and next boundary

`Tests/WheeledTestProfileProbe.cpp` contains independent literal expected
quantization/timing values, rejection cases, caller-mutation isolation, frame
identity/reset/hold checks, history/replay/horizon checks and repeated timelines
with different observation cadences. A forwarding counter wraps the actual
producer to distinguish first production from stream-history rereads; it never
supplies mock physical state. Publication in the probe only tests the stream.

Use `Tests/RunInputProducerProbe.ps1 -ProbeName WheeledTestProfileProbe` with
the existing required compiler/output/summarizer arguments. Its original
InputProducerProbe default remains unchanged; TestInputProducerProbe is also
selectable. Each run uses a fresh directory and canonical observability report.

Next: qualify a dynamic test through the real UObject consumer, compare physical
quantization/slew/frame timing and prove the old injector is not read. Only then
may a corresponding SLTest group claim migration. Raw-input/mapping parity,
Sky actions, initial-state fixture migration, reactive scenarios, UE execution
and regression/Gold equivalence remain separate work.
