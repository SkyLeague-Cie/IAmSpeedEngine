# Last-active device policy and manual ID lock

`DeviceActivityPolicy.h` is a values-only policy owned by the acquisition caller.
`FGameInputSelectedSource::CreateAutomatic` composes it with the reviewed raw
polling and single discovery/session/journal. The explicit-selection factory
remains available; automatic mode rejects explicit selection requests and takes
queued manual ID lock/unlock requests instead. Sky League's Windows input host
uses `CreateRaw` with this policy on an acquisition cadence independent of UE
render frames.

There are no numeric configuration defaults. Creation requires stick and trigger
Schmitt enter/exit thresholds, an activity displacement threshold for each,
minimum residence in physical frames, a maximum candidate-device count and the
preferred kind for hybrid keyboard/gamepad devices. Validation requires finite
`0 <= exit < enter <= 1`, `0 < delta <= 1` and 1..256 candidates. The fixtures use
stick `0.20/0.15/0.10`, trigger `0.08/0.05/0.04`, residence 0 (plus a 3-frame test),
8 devices and Gamepad for hybrids. The optional `StartupPreferredKind` defaults
to absent, preserving activity-only selection for other hosts. Sky League
explicitly configures Gamepad then Keyboard as its startup search order. It
only considers devices that have supplied a valid OS reading. These are explicit
test hypotheses, not calibrated hardware values. Activity filtering does not
change mapped gameplay values.

Raw observation runs outside Unreal action dispatch on each forward acquisition
poll, in stable ID order. Every eligible connected device has an independent raw
history cursor; each traverses at most 64 reports plus one overflow probe. The
selected gameplay cursor is independent so observing activity cannot consume
gameplay edges. Maximum calls per acquisition poll are bounded by `(MaximumDevices + 1) * 65`.
Exceeding catalogue capacity fails closed; there is no silent first-N selection.
Only the selected session commits gameplay values. The separate cursors are
reading-history references, not additional producers or canonical journals.

The first report per lifecycle/resynchronization is a baseline, never activity.
Later button rising edges count; release and unchanged held state never reclaim
ownership. Triggers use absolute normalized values. Sticks use Euclidean radial
magnitude from their two normalized components. Crossing enter activates a channel;
returning to or below exit rearms it. The intervening band emits no activity.
Outside enter, displacement from the last activity anchor must reach delta;
the anchor changes only for accepted activity (or neutral rearm/baseline).
This allows deliberate direction changes without treating tiny active-stick drift
as activity. Nonfinite/out-of-range samples or backwards timestamps reset that
device's observation; there is no manufactured button edge after reset.

At each physical boundary, the newest significant activity in a common backend
timestamp domain wins. Equal timestamps retain the current eligible ID; otherwise
the lowest stable ID wins. A delayed older timestamp cannot steal ownership from
a more recent accepted activity. The accepted timestamp remains a global floor
after removal of that device: older delayed activity leaves selection neutral;
activity equal to the floor can choose the lowest eligible ID when the current
ID is absent. No candidate activity means keep the current
eligible device. By default startup without activity is neutral; a first held
baseline alone does not claim ownership. With `StartupPreferredKind`, the first
valid OS baseline may instead select one connected device, preferring that kind
then the other, with the lowest stable ID within each kind. Real activity wins
if present at that boundary. The startup choice does not impose a residence
period on a later real activity claim. It runs only before any device has owned
the session, never after its disconnection or while a manual lock exists.
Remembering the selected ID permits fresh-held reconnection if another device
has not taken ownership.

A manual lock selects exactly its ID, ignoring other activity. Missing/removed
locked devices yield neutral; no fallback or merge is allowed. Reconnecting the
locked ID restores fresh held state. Unlock retains the current source until new
activity arrives; events suppressed while locked are not replayed. When nonzero
residence suppresses a candidate, its pending activity is discarded at that
boundary and cannot trigger a delayed switch at expiry. Lock requests override
residence. These rules are fixed policy semantics; numerical tuning is injected.

The policy decision runs once on a forward Produce/Skip boundary. Same-frame
decision replay retains its choice; the source's canonical replay never calls the
policy or OS. Switching uses the reviewed selection generation reset: old pending
transitions are purged and the winner is fetched freshly through GetCurrentReading.
Edges from its unselected history are not replayed. Its later real transitions
remain ordered. Final submission/latch still serializes against hotplug. Activity
from an invalidated revision is removed before arbitration. Callback removal is
allowed to neutralize immediately but cannot publish another source's values.

An explicit source pause clears activity baselines/cursors and stops raw polling.
Sky League's physical pause keeps acquisition alive; physical resume waits for
the selected device's next real current reading and reuses its fresh held state.
A synthetic no-device neutral baseline never acknowledges that resume. Read-side
disconnect suppresses that candidate for its revision even if its connection
callback is delayed. Permanent SDK errors fail the source closed; history loss
clears affected activity/cursor rather than reconstructing missing events.

Portable `DeviceActivityProbe` covers button rules, timestamp ties and reordering,
held baselines, drift, Schmitt band/trigger boundaries, displacement, diagonal
sticks, residence, lock/unlock/absence/reconnect, pause, invalid data, budget and
decision replay. `WindowsActivitySourceProbe` uses real SDK interfaces with a fake
non-destructive reading history shared by independent cursors. It covers raw
activity-to-selection, explicit startup owner through raw acquisition and
journal resume acknowledgement, exclusive mapped frames, held/edge timing,
replay with queued lock, pause/hotplug, drift and reset across overrides.

Hardware calibration, focus policy, visual Freeplay and packaged qualification
remain separate gates. A discovered device whose activity cursor reads
successfully but whose selected raw cursor cannot supply a current reading leaves
resume pending; it is never treated as a fresh neutral reading. No guarantee is
made that live OS arrivals
repeat identically across runs; recorded canonical frames remain the replay input.
