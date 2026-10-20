# IAmSpeed Docker physical contracts

The CI input is an exact committed IAmSpeed checkout and an existing Linux Docker
image containing UE 5.8.2. The host needs Git, Python 3 and a reachable Linux Docker
engine. It needs no Unreal installation, private UBT manifest or engine path.

Set `IAMSPEED_UE_IMAGE` to a local image tag, image ID, or already present digest.
GitHub Actions accepts the repository variable of the same name, or inherits the
runner environment when that variable is empty. No image is pulled automatically.
Run from any host OS supported by Docker:

```
python -B HostProject/Scripts/RunDockerContracts.py --expected-sha <full-commit-sha>
```

The image supplies a non-root user (Windows hosts), or accepts the invoking user's
UID/GID (Linux hosts), `/bin/bash`, Python 3, a prebuilt Linux Unreal Editor and its
matching C++ toolchain. The engine is found using the image's `UE_ROOT`,
`UE_ENGINE_ROOT`, `UNREAL_ENGINE_ROOT`, or a standard internal path such as
`/opt/unreal`. Build.version must identify 5.8.2 with CompatibleChangelist 55116800.
An image that only contains packaged games cannot compile a new checkout.

The launcher archives the clean exact Git commit, mounts it read-only, copies it
to a unique generated output directory and builds the repository HostProject.
The immutable image ID and source archive hash are recorded. Runtime uses no
network, no host engine mount, no Docker socket, and private writable output/cache
directories. UE's early internal traces use the container's private writable
layer; they never change the immutable base image or a host Engine. Each run is capped at 2 CPUs, 12 GiB
RAM/swap and finite build/test deadlines. A timed-out owned container is stopped
only after its full ID, owner label and image ID match; a stopped run never passes.
Containers and diagnostics are preserved for inspection; no prune is performed.

The suite runs every Linux-eligible static test declared in the plugin checkout: the `IAmSpeed.`
namespace, including AnalyticWorld, Collision, ContactDetection, Motion and
Simulation, plus its retained `SkyLeague.Camera.GenericInputLifecycle` identity.
Strict success requires editor exit
zero, one nonempty queue-completion marker,
unique matching report identities, no failure/warning/incomplete tests, and the
exact static test manifest extracted from the committed source, including the
required sensor Box/Sphere/Wheel and prescribed Cylinder Box/Sphere/Wheel tests,
SweeperCandidates, MovingCCD and Torus.WorldProvider. This is the IAmSpeed physical
contract suite; it does not qualify the complete SkyLeague CI or gameplay FPS.

Editor usage analytics are disabled for the isolated NullRHI process. UE 5.8.2
otherwise queries missing shader compiler statistics during editor analytics
startup when shader compilation is disabled. This does not alter test selection,
physics, report validation or the host user's preferences.

The source manifest explicitly respects `WITH_DEV_AUTOMATION_TESTS &&
PLATFORM_WINDOWS` guards. Eight native Windows sidecar tests are absent from
the Linux build by design; the current Linux manifest has 124 tests. This
platform distinction does not waive warnings or failures in executed tests.

`python -B HostProject/Scripts/TestDockerContracts.py` tests report rejection,
archive containment and launch isolation without starting Unreal or Docker.
The legacy Windows RunContracts.ps1 remains available for historical diagnostics;
the GitHub CI entry point uses RunDockerContracts.py exclusively.

Artifacts are written beneath `Artifacts/PhysicalContracts/<unique-run>/` by
default. `--output` may select a new empty directory on the host; its path is only
generated output storage, never an engine dependency. On remote Docker daemons,
the bind paths must exist on the daemon host, as required by Docker bind mounts.

## Human-accepted integration debt (20 October 2026)

The human accepts exactly two tests as deferred, nonblocking integration debt:
`IAmSpeed.Camera.NetworkInputBoundary` (malformed/truncated frame serialization,
to revisit with netcode restoration) and
`IAmSpeed.PhysicalLaws.BoxEquilibrium.CanonicalDrops` (four face-support cases,
to revisit with Central physics). See `Scripts/accepted-test-debt.json`.

Both tests stay in the complete source manifest and are executed normally.
Their assertions, tolerances, time horizon and raw UE results remain unchanged.
A completed failure of these exact identities may produce
`PASS_WITH_ACCEPTED_DEBT`, with a separate `debt-assessment.json`, the original
editor exit (0 or automation failure1), and `full_ci_qualified=false`. The
container's zero integration exit does not rewrite the editor's raw failure.
When both tests are fixed, the same runner returns ordinary PASS.

Crashes/signals/timeouts, missing or duplicate identities, incomplete tests,
new failing identities and warnings/errors on other tests remain blocking.
This policy does not waive the SDL warnings on other tests in historical
Docker534: its raw124-test result stays72Success/50warningtests/2Fail.
No new UE/Docker run is implied by installing this classification. The
portable classifier has its own Python checks; full project image CI must
carry this exact policy when the managers prepare integration. Existing
seven-test main debt is a separate policy and is not broadened here.
