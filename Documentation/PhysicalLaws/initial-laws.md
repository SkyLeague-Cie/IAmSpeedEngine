# Initial PhysicalLaw contracts

## `IAMSPEED.PHYS.CONTACT_DETECTION.V1` — ContactDetection

- `CompositionLevel`: Atomic
- `Foundation`: yes

For a query and immutable collision world, select the earliest admissible
contact in stable order. The result distinguishes hit/miss, initial
penetration, normalized TOI, swept-body location, contact point, outward
response normal, material, source, surface, feature, and primitive identity.
Collision policy must be mutual. Exact primitives require exact-domain
semantics; approximated surfaces require declared geometric and numerical
error bounds. Equal candidates use stable public identifiers, never pointer or
task order. Owners include analytical query services, legacy adapters, solid
sweepers, and shape sub-bodies.

For translational oriented-box sweeps against an exact finite planar domain,
continuous SAT covers face, edge and vertex axes. The native result carries a
witness on each shape and their local feature dimensions. Initial overlap uses
the minimum translation axis and depth. An explicit `0.05 cm` contact shell
releases shallow stationary, tangent or separating overlaps; closing motion at
the same depth remains a time-zero blocking contact. This is an engine-chosen
contact/release law, not a tolerance for copying a particular mesh backend.

## `IAMSPEED.PHYS.NORMAL_RESPONSE.V1` — NormalResponse

- `CompositionLevel`: Atomic
- `Foundation`: yes

At an admissible contact, remove closing normal relative velocity and apply the
configured restitution law without creating an impulse that attracts separated
bodies. Initial-overlap correction and impact restitution are distinct phases.
The contract observes post-response normal velocity, impulse direction,
penetration residual, bounded energy, symmetry, and release. Owners include
solid sub-bodies, pair resolution, and the world solver.

## `IAMSPEED.PHYS.FRICTION.V1` — Friction

- `CompositionLevel`: Atomic
- `Foundation`: yes

Tangential response opposes relative slip. Static friction may cancel slip only
inside its configured bound; otherwise dynamic friction applies with bounded
magnitude. Zero slip has no arbitrary tangent direction. The contract observes
tangential impulse/acceleration, work sign, isotropic symmetry where configured,
material combination, and deterministic zero-speed behavior. Owners include
solid contact resolution, sphere/box pair solvers, and wheeled components.

## `IAMSPEED.PHYS.SLIDING.V1` — Sliding

- `CompositionLevel`: Molecular
- `Foundation`: no
- Dependencies: ContactDetection, NormalResponse, Friction

A body with sustained tangential relative speed remains in admissible contact,
does not gain tangential energy from friction, and transitions continuously to
rest, release, or a new feature according to explicit predicates. Horizontal
and inclined surfaces use the same law expressed in the contact frame. Owners
include solid sub-bodies, movement components, persistent pairs, and the world
solver.

## `IAMSPEED.PHYS.RESTING_EQUILIBRIUM.V1` — RestingEquilibrium

- `CompositionLevel`: Molecular
- `Foundation`: yes
- Dependencies: ContactDetection, NormalResponse, Friction

A valid low-energy support may become an exact deterministic equilibrium.
Compensating forces, point fixes, or cached support are applied and cleared as
one coherent state. External impulse, incompatible motion, topology change,
invalid geometry, or support loss releases the equilibrium at a defined frame
phase. Owners include movement components, sub-bodies, support state, and the
canonical frame solver.

## `IAMSPEED.PHYS.ROLLING.V1` — Rolling

- `CompositionLevel`: Molecular
- `Foundation`: no
- Dependencies: ContactDetection, NormalResponse, Friction

Rolling couples center velocity and angular velocity at the contact. Pure roll
has bounded slip residual; rolling resistance removes energy rather than adding
it. Release, reversal, incline response, and transition between static friction
and sliding are explicit. Owners include sphere sub-bodies, rolling-pair state,
and the world solver.

## `IAMSPEED.PHYS.SUPPORT.V1` — Support

- `CompositionLevel`: Molecular
- `Foundation`: yes
- Dependencies: ContactDetection, NormalResponse

Support is a load-bearing contact state, not any geometric hit. Acquisition,
retention, handoff, and release use stable surface/feature identity, relative
normal motion, admissible separation, and topology policy. A smooth geometric
join may still be an intentional release boundary. Owners include wheel and
ray-wheel sub-bodies, wheeled components, movement components, and the static
collision world.

## `IAMSPEED.PHYS.MULTI_CONTACT_EQUILIBRIUM.V1` — MultiContactEquilibrium

- `CompositionLevel`: System
- `Foundation`: yes
- Dependencies: ContactDetection, NormalResponse, Friction, Support,
  RestingEquilibrium

Compatible simultaneous contacts are solved as one deterministic constraint
set. Hard non-penetration and feasibility precede softer retention objectives.
Incompatible constraints are rejected and surfaced rather than averaged into
penetration. Result does not depend on registration, pointer, worker, or
container order. Owners include the world subsystem, component pose projection,
persistent-pair solve, and sub-body contact reducers.

## `IAMSPEED.PHYS.SUSPENSION.V1` — Suspension

- `CompositionLevel`: System
- `Foundation`: no
- Dependencies: ContactDetection, NormalResponse, Support,
  MultiContactEquilibrium

Suspension maps certified support distance and relative normal motion to
bounded spring/damper response over explicit travel, bump-stop, and rebound
limits. Sprung loads, per-support forces, chassis force/torque, loss and
reacquisition are observable. The same fixed physical delta is used in real-
time and fast drivers. Owners include wheel sub-bodies, wheeled components,
suspension configuration, and the canonical simulation step.
