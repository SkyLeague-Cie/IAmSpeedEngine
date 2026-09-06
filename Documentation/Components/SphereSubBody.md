# SphereSubBody

`USphereSubBody` is a solid sphere sub-body owned by an `ISpeedComponent`. It
participates in sphere sweeps, dynamic sphere/sphere and sphere/box contacts,
static penetration projection, persistent contact, and rolling state. It does
not own global candidate ordering or canonical frame orchestration.

Its PhysicalLaw ownership includes ContactDetection, NormalResponse, Friction,
Sliding, RestingEquilibrium, Rolling, and Support. Contract tests should cover
exact radius geometry, face/edge/vertex contacts, containment and penetration,
restitution, horizontal and inclined sliding, pure rolling, release, CCD,
filtering, symmetry, and deterministic repeated execution.
