# U3 real-worker input ordering fixture

Status: SOURCE_ONLY_UNCOMPILED. This is a separate gate from accepted W4.
No CI scenario, player input route, physics coefficient or runtime default changes.

`IAmSpeed.Simulation.ProducedInputWorkerOrder` owns a real game world, ASpeedCar,
ARealTimeSimulation, wheeled component, immutable empty analytical collision
world and test-owned Chaos wheel/suspension storage. The existing admission
fixture supplies the storage/lifetime pattern; an empty collision world removes
contacts without replacing the production world step or physical input consumer.
The vehicle retains its default immobile state: no CanMove or axis setter is used.
This unit proves consumption/slew/publication ordering, not a moving-car response.
Before and after every frame it records the natural countdown value (1), its
unstarted flag and CanMove=false; these values are asserted, never forced.

The sealed W1 profile passes through the actual FTestInputProducer and FInputStream.
A forwarding wrapper records acquisition only. It must not re-enter FInputStream
from Produce, because Consume owns the stream mutex during that call.
The real FSimulationWorker invokes RunCanonicalFrames(1) for five frames. The
production driver assigns local frame C+1 and runs preparation, core simulation,
snapshot construction and publication; the test never calls UpdateInputs or any
publication hook itself. Test-only friendship exposes fixture setup and read-only
witnesses; no production layout or execution changes are introduced.

Witness order per frame: StepBegin, Produce, BeforePublication, StepEnd. The
presentation producer reads values/stream only and observes the previous input
publication while the current global snapshot is being constructed. After the
step, global and input frame/serial must agree with C/C+1. The literal throttle
oracles verify neutral C0, scenario key0 at C1, later key2 at C3 and one slew step
per frame. History is checked against the literal timeline after later frames;
repeated presentation reads must not call the producer.

All fixture construction/registration and teardown occur on the game thread.
Only the worker writes the reserved witness array, counters and completed-frame
records; the test reads/asserts them after StopAndJoin. Observer removal, stream
detachment, wheel pointer clearing and simulator destruction precede configuration
and world destruction. A five-second wait detects a stalled finite run and always
requests stop/join. StopAndJoin cannot forcibly cancel a wedged canonical frame;
an eventual authorized native runner needs its outer process timeout as well.

The one expected wheel-binding diagnostic is explicit and bounded. No unexpected
errors/warnings are suppressed. The default movement state is asserted rather than
silently altered. Healthy full-frame execution remains to be established by a
reviewed Unreal build/run; source guards are not that evidence.

Gaps: owned-worker startup/cadence, pause/lifecycle, live devices, nonzero initial
profiles/CanMove commands, dynamic contact/vehicle response, unsupported fault
injection, broad CI migration and Freeplay. The separate JSON inventory proposes
only this test; it does not activate the W4 wrapper or authorize execution.
