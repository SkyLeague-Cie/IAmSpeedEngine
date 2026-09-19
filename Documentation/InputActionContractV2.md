# Portable action contract v2 — A1

Source checkpoint only. The mapper, sink execution, publication journal and
stateful dispatch are subsequent checkpoints. No production caller includes
these headers. No platform backend, asset import or game-control parity is
qualified. Existing `InputFrame.h`, producers, controller and units 1–3 retain
their v1 semantics and ABI. `InputFrameV2.h` uses `Speed::Input::V2`; there is no
implicit conversion from either version. Integration requires an explicit
future opt-in; it cannot supply missing activity by examining old values.

`FInputActionContract::Create` validates and copies a description, returning
`shared_ptr<const FInputActionContract>`. Its catalogue, raw bindings, physical
descriptors and exact content fingerprint remain immutable. The fingerprint
encodes each field (including mapping order and revision) in canonical bytes,
not object padding or addresses; it is not a security hash or a network format.
Producer, stream and sink must share this contract when implemented.

The built-in definitions are IAmSpeed.Throttle (0, unsigned 255), Brake (1,
unsigned 255), Steering (2, signed 127). They use Axis1D; extensions own slots
3–31 and a namespace, version, aliases, Bool/Axis1D type, quantization, response
parameters, activation hysteresis and Wired/Unwired status. Axis2D is rejected.
BaseActions supplies identity response and zero activation thresholds; these
are portable defaults, not exported SkyLeague settings. Nothing assigns keys
automatically. An unwired action cannot be mapped or populated by a frame.
Exactly one descriptor per physical destination is required; raw contributions
may be multiple. Actual evaluation and the grouped physical commit are not A1.

Raw gamepad controls use positions, never vendor button numbers. Sticks are
signed; triggers unsigned. Keyboard controls use supported HID page 0x07
usages. This header performs no OS translation. An adapter must prove its
translation or report Unsupported. Complete final state doubles as the sorted
capability list. Ordered changes retain every intermediate value; final values
must agree with each control's last change. Stateful sequence/generation and
previous-state consistency checks belong to the future mapper. Overflow is
rejected, resync supplies a fresh complete baseline without old changes.

`FInputFrameData` explicitly carries final values, an independent ActiveMask,
ordered Started/Completed transitions with transition values and source order,
and reset. Triggered is an observation of current activity, never stored as an
edge. Reset prohibits transitions and permits an active held baseline. Integer
quantization does not determine activity. Structural validation is separate
from catalogue/fingerprint validation; transition alternation and final activity
are checked within a frame. Cross-frame continuity is a future stream gate.
StreamEpoch (possession), MappingRevision (frozen configuration), and
DeviceGeneration (acquisition reset) are distinct types. Hosts allocate them;
no wall clock or render frame is consulted.

Prepared test: `Tests/InputActionContractProbe.cpp` (not executed at A1 source
submission). Intended gate is C++17 MSVC14.38 /W4 /WX plus portable producer
and units 1–3 regression probes, after reviewer authorization. This test does
not establish hardware support, asset parity or Unreal runtime integration.
