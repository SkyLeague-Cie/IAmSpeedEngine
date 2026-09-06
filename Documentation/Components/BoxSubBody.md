# BoxSubBody

`UBoxSubBody` is a solid oriented-box sub-body owned by an `ISpeedComponent`.
It provides box sweep geometry, dynamic box/box and box/sphere contact
participation, static penetration gathering, and normal/friction response
hooks. It is not itself the complete rigid-body or world solver.

Its PhysicalLaw ownership is many-to-many: ContactDetection, NormalResponse,
Friction, Sliding, Support, and MultiContactEquilibrium. Backend-independent
tests should cover ray/sphere/box contact against exact primitives, oriented
symmetries, initial penetration, edges/vertices, filtering, release, and stable
tie-breaking. Unreal `FHitResult` is an integration-adapter representation,
not the intended authoritative backend result.

Smooth extruded providers must not expose a concave internal approximation
edge as a collision wall. The provider retains finite SAT intersection/TOI
proof, then reconstructs normal, depth and both witnesses from the finite face.
Both adjacent chords must lie on the query side and the complete swept box
footprint must remain inside the extrusion ends. Convex and authored terminal
edges keep finite SAT. A face separation is not a global minimum translation
for a compound concave world; the bounded projection still reacquires contacts.

The analytical static-world path returns both witnesses, the minimum-
translation normal/depth for an overlap, local face/edge/vertex dimensions and
stable source/surface/feature/primitive identity. `SHitResult` preserves these
fields directly; it does not reconstruct them from an Unreal `FHitResult`.

For an established varying-normal static contact, the owning component uses a
bounded contact-transport integration. A canonical 1/300 s interval is split
into at most four 1/1200 s stages; each stage reacquires the exact provider,
projects the pose, and parallel-transports linear velocity into the refreshed
tangent space. The legacy cached-plane support correction is not applied to
the same authoritative contact. Published poses must have at most 0.01 cm
analytical penetration, while pure laws use their tighter numerical tolerance.

## Exact planar equilibrium

`GetStaticRestingReaction` implements the box specialization of
`IAMSPEED.PHYS.BOX_STANDING_STILL.V1`. `TryBuildBoxRestingSupport` checks the
actual finite collision geometry: no overlap, four coplanar corner witnesses,
normal load, and COM strictly inside the supporting face. Positive bilinear
load weights place the center of pressure below the COM. The resultant is
applied once to avoid manufacturing torque by summing cancelling point forces.

The common component scopes this reaction separately around CCD prediction
and integration. External acceleration is restored afterwards, so an impulse
between two substeps releases the reaction immediately. No velocity/pose fix,
sleep acquisition, persistent force cache, or extra snapshot state is involved.
The borrowed immutable world pointer is cleared at frame completion/restore.

Enabled by default through `p.IAmSpeed.Collision.ExactBoxRestingSupport=1`,
strict SurfaceAnalytic only; set0 for comparison. Legacy/Hybrid and shadow-only
worlds do not consume this reaction. Canonical rest/acquisition and dependent
host integration are qualified; this is not exhaustive curved/multi-contact
or arbitrary host recovery certification.

The separate `IAMSPEED.PHYS.BOX_EQUILIBRIUM.V1` path adds planar acquisition:
rotation-aware candidate discovery/vertex event refinement, simultaneous normal
and Coulomb response and unilateral projection. A plastic full-face impulse
can certify complete sticking; a two-point impulse instead conserves angular
momentum about the edge, retaining that free rotation. Both require feasible
individual non-attractive Coulomb witnesses. The general coupled iteration
remains bounded; a failed solve is not proof of convergence.

Finite supported sliding has a geometry-derived face height, a feasible center
of pressure and a continuous path certificate on the finite provider. Its
Coulomb stop splits the physical interval rather than reversing velocity.
Passive support publication is independent of a new impact. Canonical
quantization cannot round a body into a plane or destroy exact supported pose
and coupled velocities. No preferred face belongs to the generic box law.
