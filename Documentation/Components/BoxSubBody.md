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
