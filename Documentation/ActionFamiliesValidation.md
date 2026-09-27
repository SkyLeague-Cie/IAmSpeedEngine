# V2 producer/action-family coverage delta

Base: accepted 2cc92df91617956bd804c69572425ce4766223fd (A3). This
checkpoint adds a native test only; production headers and previous probes are
unchanged. Status: SOURCE PREPARED, NOT EXECUTED. No UE evidence is claimed.

| Boundary | Reused evidence | New focused coverage |
| --- | --- | --- |
| Raw mapping and response | A1/A2 contract and mapping probes | Gamepad three axes plus three simultaneous Boolean families, independent canonical oracle |
| Common producer/consumer | A3 raw device and sealed test stream | Device, sealed TestInputProducer, injected AI and Network implementations of the same IInputProducer, same stream/sink/commit calls |
| Physical values/history | A3 publication, replay, overflow, commit failure | Complete tuple and every retained frame compared before/after presentation across 12 source/schedule combinations |
| Stateful bindings | A3 Started/Completed ordering and latest Triggered | All six actions/all three states; within-frame release/repress; explicit event-list oracle |
| Render cadence | A3 observation schedules | Every frame, frames 0/3/7, and a single stalled observation at 7; identical edges, latest-only Triggered, repeated ticks silent |
| Reset/resume | A2/A3 resync baseline | Neutral reset followed by fresh held reset; values apply immediately without fabricated Started; three reset notifications |
| Presentation non-mutation | A3 mutation/reentry/exception/weak-lifetime tests | Every family attempts forbidden next-frame Consume; retained snapshots remain equal to oracle |
| Lifetime | A3 detachment and failure paths | End-of-stream deactivation observed by consumer and presentation for every fixture |

The new probe is ActionFamiliesProbe.cpp. Pulse/Hold/ToggleRequest are authored
fixture names, not game mechanics or shipped mappings. Boolean transition
delivery does not implement a toggle, jump, boost or powerslide. AI/network
sources are canonical doubles, not real planners/transports. Raw input is fake,
not GameInput/hardware. Reset frames model lifecycle input semantics; no actual
pause, unplug or host lifecycle is simulated. Physical commit is modeled at the
native boundary, not performed by a UE vehicle component.

Only the new native probe requires compilation/execution for this test-only
delta. Preserve the accepted A3 artifact manifest and receipts as historical
evidence for unchanged production code and old tests; do not relabel their
execution SHA. Review source first, then freeze a bounded MSVC14.38 runner with
transitive input/tool pins and canonical observability. No build has run yet.

Future UE5.7.4 integration must use the actual controller/component consumer and
carry PROVISIONAL_ENGINE_5_7_4. Native results are NATIVE_MSVC_ONLY. Neither
qualifies final UE5.8.2 UHT/UBT, EnhancedInput migration, assets, packaging or CI.
