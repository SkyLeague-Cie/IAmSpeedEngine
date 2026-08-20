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
