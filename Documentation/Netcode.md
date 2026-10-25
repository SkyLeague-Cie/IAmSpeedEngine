# Netcode status and 0.5 target

**Status, 21 October 2026:** multiplayer netcode is not operational in the 0.4
migration. Restoring a supported end-to-end networked simulation path is a
target for **0.5**, not a feature already delivered or a guaranteed release
date. The existing Unreal network-physics integration points and private
serialization contracts must not be presented as working multiplayer.

## Foundations available to the restoration

IAmSpeed owns its canonical fixed-step execution, addressed physical inputs,
completed-frame publication and simulation snapshots. Intra-frame parallel work
joins before the owner consumes results in canonical order. Real-time execution
adds wall-clock pacing; [FastSimulation](Simulation.md) runs the same physical
transitions without that wait and without camera presentation.

FastSimulation already helps execute bounded tests without waiting for real
time. Its main future netcode use is bounded catch-up and resimulation after a
correction. Fast execution is an execution mechanism, not a complete rollback
protocol: it does not by itself define transport, peer authority, frame
admission, history retention or live-world correction acceptance.

The [input-producer contracts](InputProducers.md) describe producer/session
identity, frame consumption, absence and private input framing. Missing input
must remain absent unless an explicitly specified network policy supplies a
prediction; absence is not an implicit neutral command. Packet framing and
transactional decode tests cover their stated archive types and cases. They do
not establish that live peers exchange and reconcile all required state.

The [presentation boundary](PresentationBoundary.md) keeps optional output
publication apart from authoritative physical state. The camera uses this
boundary; sound/VFX paths need their own integration and qualification. A Normal/Fast comparison
must respect its mode contract: Normal has the real camera; Fast intentionally
does not. Headless tests and a `-nosound` host cannot certify audio behavior.

## Bounded acceptance target for 0.5

The 0.5 work targets an explicitly scoped authority/client match path,
then broadens it from measured results. Its acceptance needs:

1. Real transport between rebuilt, compatible peers, with player/session
   lifecycle and one defined owner for frame and authority decisions.
2. Frame-addressed command admission, an explicit policy for missing/late inputs,
   and preserved edge/held-command semantics through loss, duplication,
   reordering and reconnect scenarios.
3. A restore/correction boundary for the actual world state, followed by bounded
   Fast resimulation of the intended frames. Compare consumed commands,
   canonical ordered contacts, causal component state and wheels/suspensions;
   a pose-only or hash-only comparison is insufficient.
4. Deterministic results under the selected worker configurations, with actual
   useful parallel participation distinguished from requested lanes. Physics,
   contact arbitration, floating-point context and publication barriers remain
   unchanged by scheduling.
5. A complete return to Normal publication after catch-up, with camera/input
   continuity and relevant interactive effects tested in their real mode.
6. Exact-source native regression results and an actual multiplayer player-flow
   test. Unsupported callback/effect paths remain explicitly unqualified.

Existing local codec, snapshot, input-producer and parallel fixtures support
this work but do not replace those end-to-end gates. The product release sequence belongs to the SkyLeague roadmap; this
page records IAmSpeed's current boundary and its intended netcode restoration target.
