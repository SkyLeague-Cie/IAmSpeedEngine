# Raw reading seam — host migration in progress

FGameInputReadCursor::PollRaw returns up to 64 complete hardware states in OS
reading order, including timestamps, virtual keys, physical scan codes, pad buttons and axes. It
reuses the existing decoder/traversal before action mapping; no V1 action frame
is converted. NoChange is an empty batch, not a manufactured repeated sample.
The first current-reading traversal identifies its fresh baseline explicitly.
Any traversal failure or overflow discards the entire partial batch and resets
the reading cursor so the next attempt starts from a current reading.

SelectedSource now has a distinct CreateRaw ownership mode. PollRaw applies
selection/activity requests on an acquisition clock and installs a complete
batch through Discovery::CommitRaw under the hotplug mailbox lock. It verifies
device object, ID, revision, kind and generation. Stale tickets, traversal
overflow and sink refusal deliver no prefix and force a fresh reading. Raw mode
rejects legacy Produce/Skip; legacy mode rejects PollRaw. Physical pause must
not invoke this source's acquisition-wide SetPaused.

GameInputCanonicalControls translates physical scan codes to supported page07
usages and standard SDK pad semantics to canonical buttons/axes. It never uses
virtual-key numbers as HID codes. Unsupported scan codes remain unsupported;
saved logical key names still need layout-aware game migration. The Scan Code
table in Microsoft's Keyboard Input Overview is the table reference:
https://learn.microsoft.com/windows/win32/inputdev/about-keyboard-input#scan-codes
GameInputKeyState documents scanCode separately from virtualKey/codePoint:
https://learn.microsoft.com/gaming/gdk/docs/reference/input/gameinput/structs/gameinputkeystate
The extended right-Shift alias follows Microsoft's DirectXTK GameInput path:
https://github.com/microsoft/DirectXTK/blob/main/Src/Keyboard.cpp

GameInputRawAcquisition connects admitted raw batches to the portable dual-lane
journal. Registry indices preserve distinct 32-byte device IDs without hashing.
Disconnect/failure publishes a distinct neutral baseline. A fresh reconnect
restores held values without synthetic edges. See AcquisitionControlPlaneV4.md
for exact ownership, pause/resume and integration limits.

WindowsRawReadBatchProbe covers 63/64/65 readings and a fresh current reading
after overflow. WindowsSelectedSourceProbe exercises legacy and raw ownership,
hotplug admission, canonicalization and the raw journal connection. Evidence is parent-owned,
standalone MSVC against GameInput v3 with fake devices, not an Unreal build.
