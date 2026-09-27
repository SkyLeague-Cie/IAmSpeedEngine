# Keyboard acquisition diagnostic — source-only preparation

This is an out-of-product investigation, not a replacement input backend.
`WindowsKeyboardDiagnostic.cpp` is never part of the Unreal module or a game
target. Building it does not initialize GameInput. Do not run its hardware mode
without a separately reviewed command, receipt and hardware session mandate.
The portable `KeyboardDiagnosticModelProbe.cpp` has no Windows/GameInput includes
and is the only executable permitted in this source-only checkpoint.

## Observation to explain

A preceding standalone test obtained initial keyboard readings with F9 false
but no later logged timestamp while a participant reported pressing F9.
Gamepad readings continued. Three enumerated keyboard identities do not prove
that three distinct physical keyboards were connected. A later direct F9-only
diagnostic reproduced the observation. No failure HRESULT established a cause.
The earlier diagnostic deduplicated by timestamp; this is insufficient to
prove that reading identity never changed. No raw keycodes or text were saved.

## Contract inventory (checked 18 September 2026)

| Topic | Evidence and consequence |
| --- | --- |
| Focus | The default policy delivers focused input; `GameInputEnableBackgroundInput` requests delivery outside focus. Record the chosen policy and whether **our own** window is foreground. Do not record another application's identity or assume background delivery from setting the flag alone. [Microsoft focus policy](https://learn.microsoft.com/en-us/gaming/gdk/docs/reference/input/gameinput/enums/gameinputfocuspolicy) |
| Factory and apartment | GameInput is nano-COM: reference counting applies, but no COM runtime, apartment model or `CoInitialize` prerequisite. Its objects are agile. The diagnostic creates/releases its API and reading on one worker for simple ownership; this is not an API threading restriction. [Microsoft fundamentals](https://learn.microsoft.com/en-us/gaming/gdk/docs/features/common/input/overviews/input-fundamentals) |
| Initialization cost | Create once before starting the measurement clock. Initial factory work is inappropriate inside a time-critical loop. [Microsoft GameInputCreate](https://learn.microsoft.com/en-us/gaming/gdk/docs/reference/input/gameinput/functions/gameinputcreate) |
| Readings | `GetCurrentReading(Keyboard, nullptr)` queries the latest keyboard reading without a particular device filter. Reading objects are reference-counted singletons; retained pointer identity detects the same reading. Current-state polling does not establish complete rapid-edge history; that would require a separate bounded `GetNextReading` test. [Microsoft readings](https://learn.microsoft.com/en-us/gaming/gdk/docs/features/common/input/overviews/input-readings) |
| Time | Reading timestamps are microseconds and can be compared to `GetCurrentTimestamp`. They are not guaranteed to distinguish two readings. Distinct objects with equal timestamps must not be discarded. A backwards timestamp is retained as a diagnostic failure, not silently repaired. [Microsoft GetTimestamp](https://learn.microsoft.com/en-us/gaming/gdk/docs/reference/input/gameinput/interfaces/igameinputreading/methods/igameinputreading_gettimestamp) |
| Window/message pump | A normal Win32 window needs its message loop. Microsoft's GameInput sample has one. That is evidence for a useful controlled experiment, **not proof** that GameInput polling universally requires the caller's message pump. This probe pumps the UI thread and polls on a separate worker, outside rendering/Unreal. [Microsoft sample](https://github.com/microsoft/Xbox-GDK-Samples/blob/main/Samples/System/GameInputInterfacing/Main.cpp) |
| Version | Compile against the exact pinned v3 header/import library and record the actual loaded module and hash at runtime. Generic overview pages contain older enum examples; use the installed v3 declarations, not copied numeric kind values. No SDK/runtime installation or update is part of this experiment. |

## Hypotheses and discriminating experiments

These are hypotheses, not diagnoses or product changes:

1. **Focus/host context:** the prior console process was hidden. Establish a
   visible focused-window baseline with a live UI pump. A later separately
   approved background comparison changes only focus policy and focus phase.
2. **Device filter/discovery:** first query any keyboard, without the production
   catalogue, mapper or last-active policy. If F9 arrives here, compare an
   explicitly selected physical keyboard in a later probe. The unfiltered
   query is not an aggregate union of held states and does not prove per-device
   support or keyboard/gamepad arbitration.
3. **Timestamp deduplication:** retain the previous reading object and compare
   identity. Log a distinct reading even if its timestamp equals the previous
   one. Fixture coverage specifically prevents the earlier timestamp-only
   diagnostic from being repeated.
4. **Key routing/function layer:** physical F9 may be transformed by a keyboard
   mode, remapper or focus routing. Do not inspect other keys to infer this.
   A future explicit F9-only foreground control experiment can test the route.
5. **Runtime/environment:** if the isolated foreground probe remains silent,
   preserve the negative observation, API clock progress, exact runtime and
   host context. Do not blame IAmSpeed policy or change drivers on that basis.

No current evidence establishes that a collector thread is necessary, that
GameInput cannot read this keyboard, or that UE's frame dispatch is required.

## Implemented minimal probe

The only accepted CLI forms are `--hardware --foreground` and
`--hardware --background`; all others return before GameInput initialization.
Neither form is authorized to execute by this document.

The visible window has a local **Demarrer** button. Only its click starts the
worker. GameInput initializes before the capture epoch. The worker samples the
latest unfiltered keyboard reading approximately every 5 ms, reducing temporary
key arrays immediately to an F9 boolean. It never logs another key, character,
key count, device name or global keyboard state. It installs no keyboard hook,
injects no input and does not register Raw Input or reading/device callbacks.

Data records contain `type=f9`, local `us`, GameInput `os_us`, and `f9` boolean.
Diagnostic context contains config, initialization/read HRESULT, our foreground
boolean, one-second API-clock/sample-count heartbeat, cue timing and cleanup.
The previous reading stays alive until the replacement is retained, so address
reuse cannot masquerade as an unchanged object. A backward timestamp or mutated
same-object state fails the capture. Missing readings are not invented as F9 up.

The first implemented protocol is **foreground only**, even when the explicit
API policy is background-enabled. Losing foreground invalidates coverage; it
does not rewrite an input reading. This deliberately separates the API flag
comparison from a later background-focus experiment. UI closure cancels the
worker; no callbacks need unregistration. The worker releases reading/API before
posting completion, and the main thread joins it before final cleanup receipt.

## Local cues and protocol evidence

The worker clock schedules phases; chat messages play no role. The UI displays
and redraws each cue, logs the display time, then acknowledges that phase to the
worker. A requested system sound is advisory: the OS may mute it, so the visible
cue is authoritative. A cue more than 250 ms late, unacknowledged, out of order,
or skipped invalidates the protocol. This bound is a diagnostic UI validity
budget, not a product input tolerance or a claim of exact human timing.

| Seconds after initialization | Visible instruction | Required observation |
| --- | --- | --- |
| 0–5 | Rest / release F9 | F9 false |
| 5–10 | Hold F9 | F9 true |
| 10–15 | Release F9 | F9 false |
| 15–20 | Hold F9 | F9 true |
| 20–25 | Release F9 | F9 false |
| 25–30 | Rest | F9 false |
| 30 | Finished | All phases covered, at least four transitions |

An initial held key cannot substitute for the rest baseline. A successful
process with no updated input must never qualify the keyboard. Exit 0 requires
observed protocol coverage and clean completion; exit 6 denotes failure or an
incomplete/cancelled experiment. Preserve the actual cause and raw output; no
automatic retry. Even exit 0 would not qualify rapid taps, arbitration, gameplay,
latency guarantees, background input or any production integration.

### Next lifecycle harness design (not implemented or runnable here)

Reuse the local-clock/display-ack contract, but advance on observed preconditions
with bounded timeouts, rather than trusting a wall-clock gesture schedule:

1. Ready button after initialization; require fresh neutral baseline.
2. Display hold-R2 cue; require selected gamepad and fresh nonzero throttle.
3. Display unplug cue; require removal, then neutral/reset with no stale edges.
4. Display replug-still-held cue; require same intended device and first fresh
   held snapshot. Do not qualify an unselected startup baseline as reconnection
   of a previously selected source.
5. Display release cue; require fresh zero before the next phase.
6. Display hold cue; require nonzero before scheduling pause. Keep the hold
   instruction displayed throughout pause and resume; require paused neutral
   and first fresh held resume. Only then display release.

Every phase records planned/post/display/observed times and actual boundary
snapshots. Failure to establish a precondition is **not exercised**, not PASS.
Timeout terminates the run and preserves the receipt. No lifecycle acquisition
or implementation is included in this keyboard checkpoint.

## Proposed future fail-closed launch receipt

A later reviewed launcher must pin exact clean HEAD, source files, executable,
wrapper, SDK header/import library and runtime DLLs before launch and recheck
them afterwards. It must also inventory the actual loaded runtime path/hash,
which this minimal probe does not yet emit; that gap blocks hardware execution
until the reviewer accepts the completed launcher/instrumentation bundle.

Use a new output directory and retain stdout/stderr without overwriting. The
interactive probe window must be visible; a hidden console helper is acceptable.
Bound the whole process (including waiting for Start, initialization and shutdown)
to 90 seconds. Retain its native process handle, wait with a timeout, kill only
that process by handle if needed, then use a second bounded cleanup wait.
Never infer exit 0 from a null exit code. Require process exited, exact exit 0,
nonempty valid JSONL stdout, empty stderr, joined worker, released reading/API,
destroyed window, all cue acknowledgements, covered phases and immutable pins.
Record process ID, exit, timeout, raw sizes/hashes, runtime provenance, validity
failure and cleanup. Canonical log summarization must succeed before inspection;
summarizer failure also makes the launch fail. Do not substitute process success
for behavioral evidence. The existing hardware-session launcher is not a
reviewed launcher for this executable.

## No-device fixtures

`KeyboardDiagnosticModelProbe.cpp` covers initial neutral/held, repeated identical
readings, press/release, equal timestamps on distinct readings, late timestamps,
immutable identity mismatch, focus loss without fabricated input, sticky failure,
stopped observation, cue ordering, late/missing acknowledgements, missed input
coverage, monotonic local time and incomplete/complete cleanup.

These fixtures test protocol/observation semantics. They do not exercise Win32
painting, sound, OS focus, actual GameInput, process termination or COM lifetime;
the UI worker and real process cleanup remain compile-only until a separately
approved session. Build/test receipts must preserve that distinction.
