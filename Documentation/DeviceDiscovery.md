# Device discovery and explicit selection

Source/native candidate after `87fde649`; no production host constructs it.
The portable `DeviceDiscovery.h` catalogue stores opaque backend-scoped 32-byte
IDs, lifecycle revisions, connection state and selectable capabilities. It
includes unsupported devices with a zero capability mask and removed devices
as tombstones. Snapshots sort by ID bytes, never by callback order. There are
256 retained identities per instance; exhaustion fails closed until rebuild.
These IDs are neither producer IDs nor network/player identities.

The host explicitly requests an ID and Keyboard or Gamepad. Enumeration never
chooses a default. Requests for absent/unsupported devices stay neutral. An
unrelated arrival cannot steal selection. Removing the selected device cancels
held state and pending edges; reconnecting that requested ID or replacing its
lifecycle revision starts a new generation. Selecting another ID or kind has
the same reset boundary. Tickets include ID, revision, kind and generation;
stale samples cannot be relabelled or accepted. The first fresh held baseline
is accepted immediately without synthetic Start/Stop. Pause/resume invalidates
tickets. Old immutable frames remain readable, and Skip preserves pending reset.

All catalogue/session operations share a mutex through validation and commit.
Per-ID revisions must order lifecycle events; stale revisions are ignored,
identical revisions are idempotent, conflicting equal revisions fail closed.
A failure clears selection, delivers a reset and continues neutral canonical
frames. If resetting fails, production returns no frame rather than stale data.

## Windows boundary

`Windows/GameInputDiscovery.h` registers one blocking enumeration and connection
callback for every input kind declared in the inspected GameInput v3 SDK.
This means devices exposed by GameInput, not all Windows PnP devices. Unknown
future SDK kinds and devices unsupported by GameInput are outside this gate.
No aggregate device is created. Each app-local device ID is copied losslessly;
COM handles stay in the Windows leaf. Catalogue tombstones retain the last
handle until shutdown, bounded to 256 identities, preventing pointer reuse.

Callback timestamps order each ID. An old timestamp is ignored. An obsolete
object's disconnect cannot remove its replacement. Conflicting equal
timestamps fail closed: this candidate does not invent ordering for ambiguity.
Real-runtime qualification must measure whether this conservative rule needs
an SDK-ordered sequence instead. Different-device callback permutations have
identical sorted catalogues and explicit selections; live arrival timing across
physical frames is still not a reproducibility guarantee.

`AcquireSelected` returns a retained COM handle and generation ticket. Future
polling code must fetch a fresh raw reading for that handle and submit mapped
values with the original ticket; removal/replacement between those calls rejects
the sample. Submission and final frame consumption serialize against callbacks.
The discovery object does not call GetCurrentReading/GetNextReading. Integrating
this selection owner with the existing single-device acquisition traversal is a
subsequent source gate: do not chain two producers or feed produced frames back
as samples. This checkpoint tests selection/lifecycle using injected values.

Shutdown marks the object stopped before unregistering, without holding the
callback mailbox lock. A false unregister retains all callback resources for
retry; successful shutdown is idempotent. Destruction after unresolved unregister
fails fast, matching the existing acquisition lifetime rule. No host may destroy
this object from its callback or while another caller uses it. Callback failures
retain their first HRESULT and neutralize the selected session.

## Product policy remains open

| Option | Required decision |
| --- | --- |
| Explicit host selection | Who chooses the ID/type and when; no default here |
| Fixed priority | Keyboard/gamepad priority and stable tie-break among same-type devices |
| Coexistence | Per-action merge and conflicting axes/edges rules |
| Last active device | Activity thresholds, analog drift filtering and hysteresis |

Explicit selection is injected by fixtures only. None of these options is
selected as a game default. Future console backends can supply the portable
contract without Windows types. Focus policy, runtime redistribution, hardware,
Unreal integration, mappings and gameplay migration remain separate gates.

## Verification

`DeviceDiscoveryProbe.cpp` covers all six three-device enumeration permutations,
unsupported devices, no default, explicit switching, stale/duplicate/conflicting
revisions, pending-edge purge, reset across overrides, reconnect, replacement,
pause/resume, replay, presentation rejection and capacity failure.
`WindowsDeviceDiscoveryProbe.cpp` uses real SDK interfaces with shared fakes,
covering enumeration filters, COM replacement, delayed obsolete callbacks,
fresh baseline, no hardware reads, shutdown failure/retry and in-flight callback.
`RunDeviceDiscoveryProbe.ps1 -ProbeName <name>` records raw artifacts and invokes
the canonical observability summarizer. Portable compilation needs no SDK include.
The original portable and Windows acquisition probes must also pass unchanged.

Primary API references: [device IDs](https://learn.microsoft.com/en-us/gaming/gdk/docs/features/common/input/overviews/input-devices),
[registration](https://learn.microsoft.com/en-us/gaming/gdk/docs/reference/input/gameinput/interfaces/igameinput/methods/igameinput_registerdevicecallback),
[callback lifetime](https://learn.microsoft.com/en-us/gaming/gdk/docs/features/common/input/advanced/input-callbacks).
Signatures are checked against the local v3 header; online examples may show
older API signatures.
