# Windows acquisition adapter — isolated native candidate

`Source/IAmSpeed/Input/Windows/GameInputAcquisition.h` is an optional Windows
leaf targeting the GameInput v3 header bundled with UE 5.8.2 (package declared
as 3.1.26100.6879). No portable header includes it, no module dependency or
plugin activation is added, and no production caller constructs it yet.

The production header at source checkpoint `188b38e` compiled against the local
GameInput v3 SDK with MSVC 14.38 C++17 /W4 /WX. The isolated fake-API fixture
passed **125 checks**, including an isolated terminate-handler subprocess.
No real GameInput runtime, hardware, Unreal build or activation was used.
CP2's earlier 946 native checks qualify the portable lifecycle separately.

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
mapping rejection, edge overflow or excessive backlog neutralizes
and resynchronizes, with an explicit status. After a discontinuity the next
poll establishes a fresh baseline, without synthetic Start/Stop. Replay and
invalid frame-gap requests do not query live hardware.

Connection registration uses blocking enumeration plus callbacks on the
selected device. Callbacks only update connected state and a monotonic epoch
under a separate mailbox mutex. Polling checks the epoch through sample commit
and again at the physical latch. A disconnect/reconnect pair between polls
therefore still creates a new baseline. Mapper execution never owns that
mailbox mutex. API lifetime is held by COM references. Explicit `Shutdown()`
must succeed before the host releases ownership. On false, registration/token,
API/device/context and object storage remain alive for retry, while new reads,
skips and pause calls are rejected. Successful shutdown is idempotent.
Unregister runs without the mailbox lock; holding Gate is safe because the
callback never takes Gate. Shutdown/destruction from the callback is forbidden.
The destructor retries shutdown; if registration still cannot be cleared it
calls `std::terminate` rather than freeing a possibly live callback context.
This last resort is not graceful recovery: the host must retain ownership and
resolve explicit shutdown failure before destruction. It requires a separate
death-test subprocess before activation, never the main test runner.

SDK error policy is explicit and `GetLastError()` preserves the HRESULT:

| SDK outcome | Policy |
| --- | --- |
| READING_NOT_FOUND | Normal end, NoChange or Updated; no reset |
| REFERENCE_READING_TOO_OLD | Neutralize and Resynchronized; fresh baseline next poll |
| DEVICE_DISCONNECTED | Neutralize immediately, Disconnected; no reads until a new callback epoch |
| DEVICE_NOT_FOUND / OBJECT_NO_LONGER_EXISTS / INPUT_KIND_NOT_PRESENT | Failed latched, no further OS reads |
| Other failed HRESULT / success with null reading | Failed latched, exact error or E_UNEXPECTED |

Only reconstruction recovers a permanent failure. Existing immutable replay
remains readable until shutdown. A read-side disconnect does not forge a new
callback event; the mailbox remains authoritative for subsequent reconnection.

Permanent failure disables OS polling, not canonical production. After the
neutral reset is prepared, forward Produce continues yielding neutral frames
and the first true consumption receives RequiresReset. Skip advances the
neutral session while preserving its pending marker across overrides.
LastStatus remains Failed and the exact HRESULT remains available, including
after later hotplug notifications. Following frames stay neutral until explicit
reconstruction; retained pre-failure replay remains unchanged. If the portable
reset itself cannot be prepared (for example sequence exhaustion), the source
returns no frame and requires replacement instead of claiming neutral output.

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

## Native evidence and remaining gates

`Tests/WindowsGameInputProbe.cpp` implements fake COM interfaces using the real
v3 declarations. `Tests/RunWindowsGameInputProbe.ps1` builds/runs it in a fresh
artifact directory and summarizes raw logs. The successful fixture covers SDK
HRESULT policies, 64/65 bound, equal/backwards timestamps, keyboard/gamepad
mapping, epoch changes during mapping and the physical latch, pause/resume,
unregister false/retry with retained ownership, in-flight callback waiting,
idempotent shutdown, isolated fail-fast, fatal reset before consumption and
across multiple skips, no further OS calls after fatal, and immutable replay.

First run: 125 PASS in 2.4716673 seconds, no source fix or retry.
SDK header SHA256 `AC1F091174A9B80BA60F16A7F90A9B6B71208B490AAEAFDBF1004F2025B03C60`.
Fixture SHA256 `F3525EE2A7CA5D8EF60BC52575F4F221AF3632C4E45B46F1AC16FD230C7EA225`.
Executable SHA256 `BF97341665ADA09E6CE4CA43254FDFFDF633282CB31D24ACC1E94BB923AA6583`.
Host artifact: `Artifacts/InputProducersCP2/windows-native-188b38e-v1/`.
Metadata uses the generic package summary schema; no Unreal package was built.
Fake failures prove adapter responses to those simulated outcomes, not actual
SDK delivery/lifetime timing. Production header remained byte-identical to the
approved source candidate throughout the run.

Still unfinished:

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
