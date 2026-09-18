# Selected keyboard hardware gate — proposed protocol

Base: accepted 5893e1acc263aa69d963311d680febf9a9ed072c. Source changes only until
reviewer authorizes bounded build/fake execution; capture requires a later exact
binary/launcher manifest GO. No UE, Docker, gameplay activation or principal push.

A standalone Win32 diagnostic reuses the accepted UI message loop, seven FCues,
250ms display acknowledgement checks and no-console launcher. Added list control
shows connected keyboard ID64hex and revision. No item is preselected. The user
must select a row and click Demarrer even with only one entry. Discovery/runtime
initializes when the window opens; the 30-second capture clock starts only after
explicit selection. Empty inventory fails. Multiple indistinguishable IDs are
not silently resolved: cancel and inspect inventory before another reviewed run.

During capture keep the window foreground, follow its local F9 hold/release cues,
do not type other text, unplug devices or switch windows. No timing is driven by
chat. The wrapper has a 90-second total deadline including selection time; a fresh
output directory is mandatory. One user launch only, no automatic retries.

The real FGameInputSelectedSource and FInputStream are used. This is simulated
frame completion in a diagnostic, not vehicle physics. Optional observations are
disabled by default. They copy the acquisition ticket under the source Gate,
actual cursor current/next branches, return HRESULTs and every returned reading's
GetDevice ID. COM IUnknown identity is compared internally; only flags and
ordinals leave the diagnostic, never addresses. Observation adds identity queries
but no extra polling call. Submit successes/failures are counted independently.

Pin selection to ID AND inventory revision. Any differing lease revision/ID,
missing/wrong observed reading device, invalid/repeated COM reading identity,
failed submission, changed generation, resynchronization, disconnect or pause
fails this segment; no automatic fallback/reselection/reset is accepted. Every
poll logs attempted generation from its lease ticket; committed_generation is
that ticket generation only when SubmitAccepted > 0. Frame source_generation
remains null: an acquisition ticket is not an atomic final-latch generation.
Control epoch is zero and never used as generation evidence.

PASS requires: exact inventory selection; unchanged ID/revision/generation;
observed ID match for every returned reading; exactly one successful initial
current reading and at least one successful next reading (NOT_FOUND separate);
all returned readings submitted; all frames paired; seven timely visual cues,
foreground maintained, four ordinary F9 transitions, clean callback shutdown,
worker join, window destruction, native/wrapper exit0 and clean pinned postflight.
Validator reports selected ID/revision, acquisition generation, current/next
successes, NOT_FOUND, mismatch_count=0, first/last observed IDs and frame count.

Bundle pins source HEAD/base/headers/tests, GameInput header/import lib, toolchain,
EXE, scripts, protocol, and actual loaded GameInput DLL allowlist. Launch uses
unchanged reviewed ProcessStartInfo CreateNoWindow, concurrent pipe drains,
retained handle, runtime child-module inspection, timeout/tree cleanup and
canonical observability before validation. Binary pins are added only after the
reviewed build, before final capture GO.

Qualified scope: one explicitly chosen keyboard, foreground, ordinary F9
transitions, selected-source current/next and observed identity/acquisition ticket.
Excluded: rapid physical edges between polls, actual multi-keyboard isolation,
hotplug/reconnect, focus loss, no-pump, GT stalls, cadence/latency/buffering bounds,
UE/package/default activation. Existing any-device evidence is not reused as
selected-device proof. No product focus policy or polling frequency is inferred.
