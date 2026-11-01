# Client prediction implementation campaign

The new human mandate requires local predicted physics, same-frame divergence measurement,
authoritative rollback and bounded canonical resimulation with two real clients and a dedicated server.
The previous presentation-only campaign remains independently closed; its results do not qualify this work.

Ownership: IAS owns protocol, full state, inputs, physics worker and restore; Netcode Sky League owns
gameplay integration, camera/HUD and parent pins. A predicted-client profile must be separate from
PresentationClient; no existing guard is weakened. The server remains authority; each predicted client
has one physical writer for its complete causal world. Start with the full two-car world, not a guessed
single-car island. Predict the opponent only from information actually received, never test schedule knowledge.

The historical divergence helper compares the same frame, world incarnation, branch, authority epoch,
schema and registry generation. It measures position, linear velocity, orientation and angular velocity
for each actor, while also comparing full canonical actor and world payload bytes. Hidden mechanical
or gameplay differences require reconciliation even when visible poses coincide. Diagnostic quaternion
normalization never changes physics. Publication serials are local and are not compared across processes.
History has explicit frame and byte budgets; expiry, future authority, roster changes and malformed state
are distinct outcomes. No fabricated zero-error result is returned for unavailable history.

This helper neither authenticates a sender nor proves payload completeness. Its caller must obtain
local state from the completed native publisher and server state from the authenticated original connection,
with certified complete native serializers. Current native complete-restore providers are absent.
RequestRollbackAndResimulation still rejects live input sessions. No native guard is opened by this tranche.

Next implementation: complete stock-car/subbody/common/contact/world checkpoint providers and boundary
lease; atomic input+world restore; exact retained local inputs and explicitly predicted remote inputs;
bounded Fast catch-up through the same physical pipeline; resimulation receipts and final-only presentation.
No rollback acknowledgement may be issued before the actual restored/replayed publication is observed.
Missing history requires an explicit full resynchronization, not a teleport labelled rollback.

Acceptance needs actual client prediction before server delivery; forced incorrect remote predictions and
collisions; detected historical mismatches; authenticated restore/replay receipts; post-replay equality
of full causal state/contacts/inputs; no duplicated effects; finite natural process drains and exact-source audits.
Loss/delay/reordering trials and human camera/control continuity are separate measured gates.

RemoteRequestPrediction implements bounded hold-last-REQUEST speculation with explicit evidence frame,
not hold-last-applied filtering. Predictions are a distinct type with no counterfeit device sequence,
raw edge or reset. Only earlier received server commands may seed them. Late changed requests and
even within-frame taps with unchanged final values invalidate the affected historical frame. The host
must then restore/replay through the native transaction; the helper cannot acknowledge that operation.
Confirmed data and speculation have finite capacity; no silent eviction of commands needed for correction.
