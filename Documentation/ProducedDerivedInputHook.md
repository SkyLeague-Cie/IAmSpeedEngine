# Derived physical input hook — source candidate

The existing V1 wheeled consumer offers a protected pure validation hook,
followed by a protected application hook and cancellation hook. The default
implementations accept/do nothing. Games receive the exact immutable frame
already acquired by the sole physical consumer; they must never call Consume,
ReadLatest, a device poll or a GameThread action to obtain another input.

Derived validation completes before wheeled target assignment. Application
runs on the physical lane after wheeled target assignment and before the
derived component's normal physical-input copy/slew. A rejection neutralizes
targets, cancels derived state and deactivates the stream. Only fully accepted
input retains the stream handle used by OnCanonicalFramePublished. No rejected
or missing input is published as completed input, and no live fallback supplies
that frame. This is not a V2 transaction integration or rollback mechanism.

Cancellation is separate from a user release: a game must clear pending/held
input without manufacturing gameplay edges. Attachment/detachment requests
are latched under the existing mutex and applied by the physical lane. Once
the component opted into produced input, detachment keeps its legacy authority
closed for that component's lifetime. A fresh component is needed to return
to legacy ownership; null does not grant a former writer authority again.

The existing legacy scripted override retains its explicit base Skip policy;
the new game fixture disables it and uses FTestInputProducer through the common
stream. No game action IDs, gameplay types, raw mapper or platform dependency is
added to the generic consumer. The test-only friend declaration supports a
derived-module real-worker fixture without exposing mutable setup publicly.

Qualification pending: compilation and W4/U3/L1/L2 regression on the changed
host, plus the game-owned worker fixture. Historical d220/305789d results do not
qualify this diff. No source tests or engine executions have run for it yet.

This is an explicitly temporary release bridge. The final named action/mapping
and stateful presentation host remains V2; remove/adapt this hook during that
single-host migration, without a conversion layer or two physical consumers.
