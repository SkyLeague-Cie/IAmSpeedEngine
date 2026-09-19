# Raw input ingress envelope

`Speed::Input::V2::FRawInputIngressEnvelope` is an unused, portable owning
copy of one raw sample and its destination metadata. It does not poll, map,
queue, schedule, authorize, admit, apply or publish input. There are no
production references to this schema. It contains no simulated state.

`CreateCopied` accepts a participant handle, stream epoch, mapping revision,
uint64 consumption-frame address and const raw sample. It checks structural
metadata and the existing `FRawInputSample::IsValid` predicate before copying
either sample vector. Invalid input returns an empty optional unchanged; it
is never sorted, clamped, repaired or replaced with neutral input. Allocation
failure propagates. Construction is not noexcept or allocation-free.

The private fields have const accessors. Copies own separate sample vectors;
caller mutation or destruction cannot change them. There is no move-ingestion
factory that could transfer storage while the caller retains aliases. An
accessor's borrowed reference is valid only while its envelope exists.

## Field ownership

| Field | Meaning owner | Envelope treatment and limit |
| --- | --- | --- |
| Target.Session | Existing lifecycle protocol | Copy opaque FSessionKey intact; require nonzero epoch. Its existing shared identity byte is owned, not a session or module lease. |
| Target.Slot | Existing lifecycle participant address | Preserve size_t exactly; no capacity-specific authorization. |
| Target.Generation | Existing lifecycle participant generation | Preserve uint64, reject zero; never increment. |
| StreamEpoch | Input stream identity | Preserve wrapper, reject zero; distinct from session/device generations. |
| MappingRevision | Immutable mapping configuration identity | Preserve wrapper, reject zero; no contract lookup or fingerprint check. |
| ConsumptionFrame | Sole canonical consumer's destination address | Preserve full uint64 including zero/maximum; no cadence, clock or sequencing logic. |
| Sample.DeviceId | Acquisition/discovery | Copy nonzero uint64; no OS identity lookup. |
| Sample.Generation | Acquisition device generation | Copy nonzero wrapper; no connection/selection check. |
| Sample.Kind | Acquisition canonical device kind | Copy Keyboard/Gamepad under existing validation. |
| Sample.Sequence | Acquisition sequence | Copy nonzero uint64; no cross-sample freshness check. |
| Sample.Status | Raw acquisition validity/resync | Accept Valid/Resync under existing validation; no error recovery. |
| Sample.FinalState | Complete supported-control baseline | Deep copy, canonical sorted unique controls including neutral; existing maximum256. |
| FinalState[].Control | Generic keyboard HID page07 or pad semantics | Preserve kind/code; no OS/vendor decoding. |
| FinalState[].Value | Canonical raw numeric domain | Preserve finite float exactly: key/button0 or1, stick[-1,1], trigger[0,1]. |
| Sample.Changes | Ordered acquisition changes | Deep copy, maximum256; existing capability/order/final-value consistency; empty for Resync. |
| Changes[].State | Same raw control/value domain | Preserve control/value exactly. |
| Changes[].Order | Source sequence and within-sequence ordinal | Preserve both fields and order; never renumber. |

There is no wire codec: size_t and the opaque shared identity are process-local.
No duplicate numeric session authority is derived from the epoch. The identity
byte can outlive its protocol; retaining it does not keep the protocol, worker,
participant, code lease or module alive.

## Structural validity is not authorization

A handle with an unregistered slot or foreign session can be structurally
accepted. Only the lifecycle owner can establish actual destination validity,
registration, generation freshness and admission. The envelope does not perform
those operations. It likewise does not prove that a mapping revision exists,
that a frame is next, or that a sample sequence is fresh.

Unsupported/Overflow yields no envelope. The future lifecycle/error boundary
must handle that outcome explicitly; an empty optional is not permission to
continue applying stale controls. Pause, disconnection, neutralization and
reconnection policy are outside this schema.

## Exactly-once transformations

| Operation | Sole responsible stage | Envelope behavior |
| --- | --- | --- |
| OS layout/range to canonical raw control/value | Platform acquisition adapter before the raw DTO | No conversion. |
| Device selection/activity hysteresis | Acquisition selection policy | Preserve one source; no merge. Activity threshold is not gameplay deadzone. |
| Raw shape/domain/order validation | Existing RawInput predicate | Non-mutating validation; repeat validation is not a second transform. |
| Binding sign/inversion/scale and combination | Existing FActionMapper::Evaluate | No mapping call. |
| Gameplay deadzone, exponent, sensitivity, clamp, quantization | Existing FActionMapper::Evaluate in its existing order | None. |
| Activation hysteresis, Started/Completed | Existing mapper history | No history or synthesized action edges. |
| Triggered notification | Existing action dispatch contract | No dispatch or action state. |
| Reset/fresh-held baseline | Separately qualified acquisition/stream lifecycle | Preserve Resync verbatim; no new transition. |
| Physical application / HandleInputs observation | Separately qualified sink / presentation | Both absent. |

These are responsibilities, not evidence of production integration. Existing
controller transformations and physical Unreal bindings still require migration
and actual removal, including mapping contexts and saved/Blueprint routes.

## Authored portable probe

`Tests/RawInputIngressEnvelopeProbe.cpp` covers exact metadata/raw copying,
independent storage after caller mutation/destruction, held Resync without
edges, frame-address boundaries, structural rejection without repair, all256
changes and257-change rejection. The generic maximum256 controls is validated
as a bound; the existing control vocabulary has fewer than256 distinct valid
controls for one device, so a fabricated256-control valid baseline is not used.

Lifecycle fixtures remain Configured. Manually assembled handles are explicitly
synthetic and unregistered; no registration, start or frame admission runs.
Equal numeric epochs from distinct protocols are accepted structurally; the
schema copies their opaque keys and does not authenticate or compare them.
The probe also shows that retaining an envelope does not retain the code lease.

This checkpoint authors the probe only. Compilation/execution and runner
preparation require separate review. It qualifies no worker, Unreal teardown,
module unload, gamepad runtime, full action pipeline or gameplay behavior.
