# Portable action mapping — A2 source checkpoint

`FActionMapper` owns the immutable A1 contract and an explicit stream epoch,
producer identity and first physical frame. It accepts one complete raw DTO for
each forward frame. An upstream polling adapter must supply explicit samples;
this class does not poll an OS, invent readings, or implement a hold cache.
No product caller activates it. A3 stream integration remains separate.

The first sample must be Resync: fresh held values are applied immediately,
activity is computed from the response, Reset is true and no Started event is
invented. Normal samples retain device identity, kind, generation and the exact
capability set. A no-change sample advances source sequence by exactly one.
A batch containing changes covers consecutive sequences after the prior sample
through its final sequence; all changes within a sequence retain strict raw
ordering. Missing neutral readings must be represented explicitly, never
silently skipped. Repeated physical requests are rejected by this mapper;
replay belongs to a sealed source/history. Any failure returns no frame and
requires Resync at the still-unconsumed physical address. Same-device Resync
must not move generation backwards or reuse an old sequence within generation.

Raw changes are applied transactionally to a working copy. Their final result
must exactly explain FinalState. Bindings for the other device kind do not
contribute; a mapped control missing from the selected device is rejected.
Unmapped controls may still appear in the capability/state list.

Bool combines nonzero contributions with OR. Axis1D accumulates in mapping
order, clamps to its signed/unsigned normalized range, then applies exactly one
response: magnitude deadzone with `(abs(x)-deadzone)/(1-deadzone)` rescaling,
exponent, sensitivity and final clamp. Signed response retains the sign.
Mapper support is narrower than A1 structural validity: only exponent 1
(identity) and 2 (explicit multiplication) are supported. `Create` rejects any
other exponent before acquisition. Direct construction also returns
InvalidConfiguration before changing frame/state. The A1 schema and fingerprint
remain unchanged; an importer must reject unsupported curves rather than
approximate them. No general library power function is used.
Activation/hysteresis runs before canonical round-to-nearest/ties-positive
quantization using the declared action scale. No threshold is inferred.
Cross-toolchain bit parity is not established by source inspection or a single
native run. Remaining float operations require Windows/Linux boundary-vector
gates before U4B qualification; identity/square support alone is not that proof.

Every intermediate raw state is evaluated. Transitions are emitted in raw
change order then action-ID order. Their values describe that intermediate
state, not the end of the physical frame. Generated ordinals start at zero per
source sequence independently of raw ordinals, including UINT32_MAX raw
ordinals. At most 64 edges can be committed; overflow rejects the whole frame
and requires resync. Finite/range, sequence and physical address errors are
explicit; no fallback or partial prefix is returned.

`AssembleDrivingTargets` is the same pure function for mapped player frames and
sealed test frames. It validates contract fingerprint, epoch and physical
address, then assembles all three targets; failure returns neutral invalid
targets. It performs no component mutation, slew, publication or callbacks.
The immutable contract already proves exactly one descriptor per destination.

`V2::FTestInputProducer` copies and validates a sealed action-frame timeline
after mapping. It never translates v1 frames or executes the mapper. Its first
frame supplies a reset baseline; later transition continuity is validated.
Forward requests are exact; retained replay is bounded to 256 frames. It is not
yet attached to a V2 stream (A3). The raw oracle in ActionMappingProbe is authored
independently and compares mapped and sealed frames through the SAME sink.
No parallel test physics consumer is added.

Prepared validation (not run at source submission): ActionMappingProbe plus A1
and accepted portable unit 1–3 regression probes. No Windows, Unreal, asset,
SkyLeague gameplay or controller binding changes belong to this checkpoint.
