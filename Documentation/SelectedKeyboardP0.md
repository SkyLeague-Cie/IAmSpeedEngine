# Selected keyboard P0: source-only diagnostic contract

This checkpoint adds test sources and a protocol. It has not been compiled or
executed. It does not create GameInput, access hardware, open a window, change
the production implementation, or activate an Unreal input source.

The prerequisite host checkpoint is `c6e6bac7e924e2edc786bdab853fa1d70cf3866b`.
The previously accepted foreground F9 capture used an any-device current
reading. That result does not qualify selected-device history traversal.

## Explicit identity

The diagnostic contract requires exactly 64 hexadecimal characters encoding
the backend's opaque 32-byte device ID. Uppercase is accepted and traces use
lowercase. There is no `any`, first-enumerated, partial-ID or implicit fallback.
The inventory eligibility check requires exactly one matching connected entry
supporting keyboard input. This check is an initial diagnostic precondition,
not a synchronization guarantee: the selected source still owns lifecycle
validation and resets if the device disappears after inventory inspection.

A future hardware launcher must obtain that inventory through reviewed
discovery code and require an explicit ID. An ID is not a friendly device name
and does not prove which physical keyboard a human pressed. Inventory mapping
and a user-visible identification procedure remain a separate hardware gate.
The current fake fixture intentionally selects ID 1 and then explicitly ID 2.
It exercises the existing selected source through `FGameInputShadowHost`, not
a replacement acquisition implementation. Automatic arbitration is disabled.

## Prepared fake scenarios

`WindowsSelectedKeyboardProbe.cpp` injects a fake SDK interface with independent
per-device reading histories into the real host factory. Every current/next
call requires a non-null device and the keyboard kind. Histories retain COM
reading objects; object identity, rather than timestamp equality, locates the
next record. Monotonic fake tokens make distinct returned objects inspectable.

- Another keyboard's held F9 cannot leak into the selected keyboard snapshot.
- First acquisition uses current; subsequent acquisition traverses next.
- Distinct readings with equal timestamps preserve a press followed by release
  entirely between polls, in order, even though final held state is neutral.
- Reaching history end creates no duplicate edges; immutable replay does not
  issue another SDK read.
- Losing the retained reference produces neutral/reset, followed by a fresh
  current held baseline without a manufactured start edge.
- Pause acknowledges independently, invalidates a pending unpublished frame,
  and prevents SDK polling while paused. Explicit reselection during pause
  resumes the new keyboard's fresh held baseline without inherited edges.
- A separate portable catalogue fixture verifies actual generation advancement
  across pause/resume and rejection of a stale generation ticket.
- The existing seven local F9 cues and display acknowledgement model are
  reused with synthetic times, not reimplemented or driven by chat messages.

These are prepared assertions, not passing results. The fake implementation
does not establish the real runtime's buffering, polling rate or device support.

## Trace provenance

`SelectedKeyboardDiagnostic.h` formats valid snapshot values as JSON. Callers
must emit host snapshots only after successful `CompleteFrame`; pending frames
remain private. The portable catalogue fixture models a completed boundary
without running a physics simulation. Each trace contains:

- requested device ID (a request, not a claim of observed acquisition identity);
- consumption frame and producer-local source sequence;
- host control epoch, reset flag, held F9 and ordered action edges with sequence;
- source generation when the caller possesses the real catalogue ticket.

The host intentionally does not expose its internal catalogue ticket. Its
traces therefore use `source_generation: null`. A host control epoch is never
substituted for a device generation or an OS timestamp. The portable fixture
can report the actual ticket generation, but does not establish generation
observability through the Windows host. Instrumenting that path for hardware
qualification requires a separately reviewed change; full frame-to-generation
hardware evidence remains open.

Only F9 is mapped (diagnostic action slot 3); no raw keyboard text is recorded.
No gameplay mapping or focus-loss product policy is introduced.

## Later execution gates

1. Review this exact source SHA and protocol. Obtain a separate bounded native
   mandate before compiling or running even the fake probe. Proposed toolchain:
   MSVC 14.38, C++17, `/W4 /WX`, accepted GameInput v3 declarations, no runtime
   import library. Preserve commands, hashes, exit codes and canonical reports.
2. After native acceptance, review a concrete standalone selected-ID hardware
   harness and launcher. Reuse the accepted visible UI, local seven-phase cues,
   acknowledged display timing and no-console launch method. Preserve initial
   inventory, explicit ID, actual acquired identity, lifecycle/generation and
   current/next call evidence. Do not silently fall back to any-device reads.
3. Request a separate hardware session only after launcher qualification. A
   foreground F9 session can qualify held input and ordinary transitions; rapid
   transitions, history loss, hotplug, multiple keyboards, no-pump operation,
   background focus and Unreal frame stalls each require their own evidence.

This P0 does not satisfy UE integration, real scheduling independence, packaged
runtime staging, gameplay parity, console portability or default activation.
