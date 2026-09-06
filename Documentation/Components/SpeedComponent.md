# SpeedComponent

`ISpeedComponent` is the generic simulation-facing owner contract for one or
more sub-bodies. `USpeedMovementComponent` is its Unreal movement-component
implementation. A component owns coherent kinematic state and frame
preparation; `USpeedWorldSubsystem` owns global registration, candidates,
contacts, integration, persistent pairs, and canonical reduction.

The component participates in several PhysicalLaws rather than defining one:
NormalResponse, Sliding, RestingEquilibrium, Support, and
MultiContactEquilibrium. A minimal harness should construct an Actor with a
SpeedComponent and BoxSubBody or SphereSubBody, seal initial state and
frame-addressed inputs, run the same fixed-delta canonical step through each
backend, and compare semantic observables. Tests must not depend on a product-
specific actor, map, fixture name, coefficient, or mechanic.
