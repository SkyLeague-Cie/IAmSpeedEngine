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

The suite runs every static test declared in the plugin checkout: the `IAmSpeed.`
namespace, including AnalyticWorld, Collision, ContactDetection, Motion and
Simulation, plus its retained `SkyLeague.Camera.GenericInputLifecycle` identity.
Success requires editor exit
zero, one nonempty queue-completion marker,
unique matching report identities, no failure/warning/incomplete tests, and the
exact static test manifest extracted from the committed source, including the
required sensor Box/Sphere/Wheel and prescribed Cylinder Box/Sphere/Wheel tests,
SweeperCandidates, MovingCCD and Torus.WorldProvider. This is the IAmSpeed physical
contract suite; it does not qualify the complete SkyLeague CI or gameplay FPS.

`python -B HostProject/Scripts/TestDockerContracts.py` tests report rejection,
archive containment and launch isolation without starting Unreal or Docker.
The legacy Windows RunContracts.ps1 remains available for historical diagnostics;
the GitHub CI entry point uses RunDockerContracts.py exclusively.

Artifacts are written beneath `Artifacts/PhysicalContracts/<unique-run>/` by
default. `--output` may select a new empty directory on the host; its path is only
generated output storage, never an engine dependency. On remote Docker daemons,
the bind paths must exist on the daemon host, as required by Docker bind mounts.
