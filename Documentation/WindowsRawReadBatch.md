# Raw reading seam — host migration in progress

FGameInputReadCursor::PollRaw returns up to 64 complete hardware states in OS
reading order, including timestamps, virtual keys, pad buttons and axes. It
reuses the existing decoder/traversal before action mapping; no V1 action frame
is converted. NoChange is an empty batch, not a manufactured repeated sample.
The first current-reading traversal identifies its fresh baseline explicitly.
Any traversal failure or overflow discards the entire partial batch and resets
the reading cursor so the next attempt starts from a current reading.

The selected-device owner still must validate its selection/generation ticket
around admission, translate platform keys to canonical raw controls, and decide
which baseline/changes belong to V2 or the pause-independent control plane.
This method alone does not migrate SelectedSource, install a V2 host, provide
pause/reset policy, or qualify real hardware. Do not interleave old Poll and
PollRaw owners on one cursor.

WindowsRawReadBatchProbe covers 63/64/65 readings and a fresh current reading
after overflow. The existing WindowsSelectedSourceProbe and WindowsGameInputProbe
remain regression checks of the unchanged V1 route. Evidence is parent-owned,
standalone MSVC against GameInput v3 with fake devices, not an Unreal build.
