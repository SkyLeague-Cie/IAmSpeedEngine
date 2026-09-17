# Windows acquisition adapter — source-only candidate

`Source/IAmSpeed/Input/Windows/GameInputAcquisition.h` is an optional Windows
leaf targeting the GameInput v3 header bundled with UE 5.8.2 (package declared
as 3.1.26100.6879). No portable header includes it, no module dependency or
plugin activation is added, and no production caller constructs it yet.

**Not compiled or executed.** The active resource lease prohibits builds.
CP2's 946 native checks qualify the portable lifecycle only, not this file.

## Ownership and operation

The host supplies a retained GameInput interface, one explicitly selected
device, exactly Keyboard or Gamepad kind, a producer identity, digital-action
mask and a pure bounded nonthrowing mapping function. The adapter takes COM
references and privately owns a portable DeviceInputSession. It is itself an
IInputProducer, so a future host can configure it through the existing boundary.
The mapping converts Windows virtual-key state or gamepad buttons/axes into
complete canonical values. This leaf does not choose game action mappings.

Forward Produce/Skip calls traverse current/next readings, bounded to 64
readings per poll. All reports retain SDK order; a separate integer sequence
identifies commits even when SDK timestamps are equal. A backwards timestamp,
SDK error, mapping rejection, edge overflow or excessive backlog neutralizes
and resynchronizes, with an explicit status. After a discontinuity the next
poll establishes a fresh baseline, without synthetic Start/Stop. Replay and
invalid frame-gap requests do not query live hardware.

Connection registration uses blocking enumeration plus callbacks on the
selected device. Callbacks only update connected state and a monotonic epoch
under a separate mailbox mutex. Polling checks the epoch through sample commit
and again at the physical latch. A disconnect/reconnect pair between polls
therefore still creates a new baseline. Mapper execution never owns that
mailbox mutex. API lifetime is held by COM references; destruction unregisters
the callback before its context is released, without holding the mailbox lock.

Gate serializes Produce, Skip and pause transitions. Lock order is Gate ->
MailboxMutex -> portable session mutex; callbacks take MailboxMutex only.
Destruction must occur after all caller handles stop using the instance and
must never run from the SDK callback. Mapper must not reenter acquisition or
lifecycle methods. The SDK may dispatch hotplug on its own internal thread;
this candidate creates no collector thread.

SetPaused resets the portable session and reading cursor. Resume accepts the
first newly fetched held state immediately, without synthetic triggers. This
method still needs host pause wiring outside presentation-observation callbacks.
The reset marker must reach real game consumers before this source is enabled.

## Explicitly unfinished

- Compile this header against the exact v3 SDK in a bounded native fixture.
- Fake-API tests for reading order, no OS reads during replay, all HRESULT
  outcomes, 64/65-report boundary, callback during mapping/final latch, shutdown,
  pause/override reset retention, equal timestamps and mapping failures.
- Register an all-device discovery service for arrival of new device identities.
  The current factory requires a selected device; it handles that device's
  connection notifications, not discovery/selection of a different new device.
- Define host single-player keyboard/gamepad coexistence/selection; no silent
  first-callback or last-UE-frame arbitration is introduced.
- Establish runtime redistributable provenance and package dependency wiring.
  SDK presence in the engine is not runtime/package compatibility evidence.
- Hardware exercise for available keyboard/gamepad, unplug/replug, repeated
  initialization/shutdown and history loss. No hardware support claim yet.
- UE Windows include wrappers/build integration, pause/focus/possession wiring,
  physical digital cancellation and exact-tuple runtime qualification.

No DualSense-specific mapping, raw-HID support, Sony SDK dependency or console
implementation is included. Those can be implemented behind another platform
leaf without introducing platform handles into the portable session.

## Primary references

- [GameInput callbacks and unregister lifetime](https://learn.microsoft.com/en-us/gaming/gdk/docs/features/common/input/advanced/input-callbacks)
- [Ordered reading traversal and bounded history](https://learn.microsoft.com/en-us/gaming/gdk/docs/reference/input/gameinput/interfaces/igameinput/methods/igameinput_getnextreading)
- Local UE 5.8.2 GameInputWindowsLibrary.Build.cs, ThirdParty/GameInput.h and
  GameInputBaseIncludes.h. API signatures checked by inspection only.
