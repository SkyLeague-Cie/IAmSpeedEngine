# Device session lifecycle at the physical consumer

`IAmSpeed.Simulation.ProducedDeviceLifecycle` is a test-only L1 fixture. Its
source is prepared, not compiled or executed. It uses the production
FDeviceInputSession, FInputStream, real finite simulation worker, canonical
driver and wheeled consumer. It does not use TestInputProducer or a device OS.

Ten canonical frames exercise:

| Frame | Event | Driving targets (throttle, brake, steer) | Reset | Digital event |
| --- | --- | --- | --- | --- |
| 0 | Connect and fresh held baseline | 200, 80, -60 | yes | none |
| 1 | Actual release of fixture action 3 | 200, 80, -60 | no | Stop |
| 2 | Queue a press, then disconnect | 0, 0, 0 | yes | none; queued press cancelled |
| 3 | Reconnect, fresh held baseline | 200, 80, -60 | yes | none |
| 4 | Pause while held | 0, 0, 0 | yes | none |
| 5 | Stay paused; reject another reading | 0, 0, 0 | no | none |
| 6 | Resume without a fresh reading | 0, 0, 0 | yes | none |
| 7 | First fresh held reading | 200, 80, -60 | no | none |
| 8 | Pause/resume and fresh reading at one boundary | 200, 80, -60 | yes | none |
| 9 | No new reading | 200, 80, -60 | no | none |

Generation checks reject old connection and pause tokens. One counted Produce
call per frame forwards to the real session. Lifecycle calls are injected at
explicit worker boundaries; observations are recorded as values, then asserted
after join. Production publication supplies immutable history. The accepted U3
test remains unchanged, and helper types have distinct names for unity builds.

After join, detachment deactivates the retained stream. Attempts to consume a
new frame or publish an old frame must fail without increasing the acquisition
count or changing the copied history/latest publication. This is a direct
stream-detachment test, not controller possession/destruction coverage.

Neutrality here means all three authoritative input targets are zero. Normal
physical slew remains enabled; this fixture does not promise instant filtered
vehicle response. It leaves the configured countdown and CanMove untouched.
Digital slot 3 belongs only to the fixture; no Sky League action is bound.

L2 must separately qualify controller pause/possession/ownership and synchronous
cancellation while its worker is stopped. Windows GameInput initialization,
polling, discovery, hotplug, shutdown and hardware require D1. This fixture does
not change mappings, module dependencies, default activation or Enhanced Input.
The only existing header edits add test friendship inside existing DEV guards.

Exact selection: `Tests/ProducedDeviceLifecycleAutomation.json`. Source guards
are structural checks only; they cannot substitute for compilation or runtime.
The intended next build combines reviewed L1 and L2, with separate exact test
selections and verdicts, unless a defect requires an isolated correction first.
