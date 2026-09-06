<div align="center">

<img src="Docs/iam-speed-logo.png" alt="IAmSpeed logo" width="480" />

# IAmSpeedEngine

Deterministic-oriented gameplay physics for Unreal Engine 5.

![Unreal Engine](https://img.shields.io/badge/Unreal%20Engine-5.x-313131?logo=unrealengine&logoColor=white)
![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)

</div>

IAmSpeed is an early Unreal Engine plugin for physics-driven games that need
explicit simulation state, continuous collision handling, reusable solid and
sensor sub-bodies, and deterministic testability.

## Current capabilities

- box, sphere, wheel, ray-wheel, and sensor sub-bodies;
- continuous collision candidate selection and bounded iterative resolution;
- movement and wheeled-component foundations;
- persistent contact and rolling-pair state;
- a canonical fixed-step frame path shared by real-time and fast test drivers;
- immutable analytical static-world data with deterministic ray, sphere, and
  oriented-box queries against bounded planes, triangle faces, and experimental
  compact patches;
- editor bake/validation tooling for generated analytical collision assets;
- Unreal network-physics integration points retained during the migration to
  engine-owned snapshots and command queues.

## Important limits

The project is under active development. The analytical static-world backend is
not yet a complete replacement for Unreal/Chaos: exact curved queries,
edge/vertex Minkowski features, complete provider coverage, and strict
zero-`UWorld` authority validation are still in progress. Compact analytical
patches remain experimental and authority-ineligible unless explicitly
certified. The current canonical simulation is single-lane; task-parallel
physics is not part of this release.

Do not infer rollback readiness, cross-platform bit identity, or complete rigid-
body coverage from the available foundations. Public contracts and their test
status are documented under [`Documentation/`](Documentation/).

## PhysicalLaw contracts

IAmSpeed assigns stable identifiers such as
`IAMSPEED.PHYS.CONTACT_DETECTION.V1` to the physical laws selected by the
engine. Laws use `Atomic`, `Molecular`, or `System` composition levels and an
independent blocking `Foundation` attribute. See the
[`PhysicalLaw catalogue`](Documentation/PhysicalLaws/README.md).

## Repository layout

- `IAmSpeed.uplugin` — plugin descriptor;
- `Source/IAmSpeed/` — runtime module;
- `Source/IAmSpeedEditor/` — editor bake and validation tooling;
- `Documentation/` — public contracts, owners, test identifiers, and limits;
- `LICENSE` — MIT license.

## Installation

1. Place this repository under `YourProject/Plugins/IAmSpeed`.
2. Enable the plugin for the required targets.
3. Regenerate project files when needed and build the host project.

The current repository is a plugin, not a standalone Unreal project. A minimal
repository-owned CI HostProject runs the initial contracts independently
of demo or product projects.

## Contributing

Please include a minimal reproduction, the affected PhysicalLaw identifier,
the backend and platform, fixed physical delta, semantic failure, and any
justified tolerance. A backend difference should be classified by hit, TOI,
point, normal, feature, transition, determinism, or cost; cross-backend bit
identity is required only where the represented surface and algorithm make it
a valid contract.

## License

MIT. See [`LICENSE`](LICENSE).
