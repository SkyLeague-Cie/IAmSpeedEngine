# `<IAMSPEED.PHYS.NAME.V1>` — `<Name>`

- `CompositionLevel`: `Atomic | Molecular | System`
- `Foundation`: `yes | no`
- `Status`: `draft | measured | accepted`

## Chosen law

State the engine's selected physical rule in observable terms. Distinguish the
chosen model from a universal claim about real-world physics.

## Preconditions and inputs

List geometry, state, material, time-step, filtering, and history inputs.

## Semantic invariants

List blocking truths such as non-penetration, impulse direction, release,
symmetry, bounded energy, ordering, and determinism.

## Outputs and observables

Name the canonical result fields and event-relative measurements.

## Dependencies

Reference stable PhysicalLaw identifiers, from foundations upward.

## Implementation owners

List public generic types or subsystems. Ownership is many-to-many and does not
imply that one class alone defines the law.

## Backend contract

Describe which assertions are backend-independent, which surfaces are exact,
and which comparisons are tolerance-classified. Do not require bit identity
between different numerical backends except where the represented surface and
algorithm are exact by contract.

## Tolerances

For every tolerance, state unit, scale, numerical/physical justification, and
whether it gates hit, TOI, point, normal, feature, transition, or end state.

## Contract tests

Every test has a stable identifier beginning with this law identifier. Record
pure tests, component harness tests, backend matrix cells, determinism repeats,
serialization, cost, and known unsupported domains.

## Failure classification

Classify failures as geometry/domain, filtering/material, contact selection,
normal response, friction, transition/history, determinism, serialization, or
performance. A classified difference is not automatically accepted.
