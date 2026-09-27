# Explicit selection to raw polling composition

Source/native checkpoint following the reviewed discovery/explicit-selection
candidate `1bd30e1`. `GameInputSelectedSource.h` owns one `GameInputDiscovery`
and therefore exactly one portable session and canonical frame journal.
`GameInputReadings.h` contains the reusable current/next raw traversal, with no
producer or session. The original single-device adapter uses that same traversal.
No produced frame is fed into a second producer.

The host supplies pure, bounded, nonthrowing keyboard and gamepad mappers and
explicitly requests an ID/kind (or deselection). A request is queued; the latest
request applies once at the next forward Produce/Skip boundary. Replay and
invalid frame gaps do not consume requests or touch the OS. Enumeration never
chooses a default and only the selected device is polled in this checkpoint.
Automatic last-active activity detection and manual-kind lock policy are the
following checkpoint, not an implicit feature of explicit selection.

Generation/device/kind changes reset the raw cursor and sample sequence. The
first current reading provides fresh held values with no synthetic Start/Stop;
subsequent reports retain SDK order and genuine edges. Selection reset purges
old pending transitions and survives overridden frames. A frame never merges
values from the former source. Pause and device callbacks can neutralize without
a running physical frame; resume/reconnect establish a fresh baseline.

Raw polling and mapping occur without the discovery callback lock. Each commit
validates the original selection ticket; hotplug during an SDK call or mapper
therefore rejects stale data. Final Produce/Skip also shares the callback lock,
so removal after the last commit cancels pending values before consumption.
A changed generation is not retried within the same frame; a replacement gets
its fresh baseline on the next forward poll. Lifecycle callbacks may reset to
neutral immediately but do not poll or install another device's values.

The shared traversal keeps the existing 64-report budget (one additional probe
detects overflow), ordered equal timestamps, backwards-time/mapping-loss reset
and explicit SDK errors. History loss resets the current ticket and cursor;
an obsolete poll cannot reset a newer selection. A read-side disconnect resets
and suppresses reads for that ID/revision until a new lifecycle revision arrives.
Unknown/permanent errors latch failure, preserve their HRESULT and keep neutral
canonical cadence with a pending cancellation marker. No live reads occur after
permanent failure. Reset exhaustion returns no frame through the portable owner.

Lock order is source Gate -> discovery Mailbox -> portable catalogue/session.
Callbacks take no source Gate. Mappers cannot reenter the source. Shutdown closes
the source, clears its cursor, then unregisters discovery without holding the
callback mailbox. A failed unregister retains the owned discovery and all live
callback resources for explicit retry. Destruction retains the fail-fast fallback.

`WindowsSelectedSourceProbe.cpp` tests raw current/next composition, request
boundaries, replay/gaps without OS access, exclusive device values, baseline and
real edges, pause, hotplug during mapping/end-of-traversal, replacement, read-side
disconnect recovery, history/permanent errors through overrides, 64/65 budget
and shutdown retry. The existing discovery and acquisition probes remain required
regressions after extraction of the shared traversal.

The candidate is optional and unconstructed by production Unreal code. It does
not create a GameInput runtime, qualify hardware, choose activity thresholds,
change gameplay mappings, or stage package dependencies. Future last-active
policy must explicitly bound polling across unselected candidates and arbitrate
once at a physical boundary before committing only the winner's fresh values.
