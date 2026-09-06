# PhysicalLaw catalogue

IAmSpeed documents its chosen physical behavior as versioned `PhysicalLaw`
contracts. These are the laws selected by the engine; they are not claims that
one formulation is the only possible model of real-world physics.

Every law has two independent classifications:

- `CompositionLevel`: `Atomic`, `Molecular`, or `System`;
- `Foundation`: a blocking attribute. A Foundation law must pass before a law
  that depends on it can be accepted, regardless of composition level.

`Atomic` describes one independently observable rule. `Molecular` composes
multiple laws into one behavior. `System` constrains orchestration across
owners, frames, bodies, or contacts. Composition level is not a quality rank.

Stable identifiers use `IAMSPEED.PHYS.<LAW>.V<MAJOR>`. The major suffix changes
only when the public semantic contract is incompatible. Test case suffixes may
be appended after the law identifier without changing it.

## Initial catalogue

| Stable identifier | Level | Foundation | Principal contract |
| --- | --- | --- | --- |
| `IAMSPEED.PHYS.CONTACT_DETECTION.V1` | Atomic | yes | Determine first contact, feature, point, normal, TOI, and penetration state. |
| `IAMSPEED.PHYS.NORMAL_RESPONSE.V1` | Atomic | yes | Prevent closing normal motion and apply configured restitution. |
| `IAMSPEED.PHYS.FRICTION.V1` | Atomic | yes | Bound tangential response by the selected static/dynamic friction model. |
| `IAMSPEED.PHYS.SLIDING.V1` | Molecular | no | Compose contact, normal response, and dynamic friction during sustained slip. |
| `IAMSPEED.PHYS.RESTING_EQUILIBRIUM.V1` | Molecular | yes | Preserve a valid low-energy equilibrium and release it on invalidation. |
| `IAMSPEED.PHYS.BOX_STANDING_STILL.V1` | Molecular | yes | [Preserve an already supported stationary box](BoxStandingStill.md), with zero linear/angular motion and penetration. |
| `IAMSPEED.PHYS.BOX_EQUILIBRIUM.V1` | Molecular | yes | [Converge to and retain a stable supported box face](BoxEquilibrium.md); tangential translation is allowed, spin and normal translation are not. |
| `IAMSPEED.PHYS.ROLLING.V1` | Molecular | no | Couple translation, angular motion, contact, and rolling resistance without artificial slip. |
| `IAMSPEED.PHYS.SUPPORT.V1` | Molecular | yes | Acquire, retain, and release load-bearing contact with explicit topology. |
| `IAMSPEED.PHYS.MULTI_CONTACT_EQUILIBRIUM.V1` | System | yes | Solve compatible simultaneous contacts deterministically without violating hard feasibility. |
| `IAMSPEED.PHYS.SUSPENSION.V1` | System | no | Couple support queries, travel, spring, damping, bump/rebound limits, and body response. |

The owner mapping is deliberately many-to-many. A law may be implemented by
several types, and a type may participate in several laws. See
[`implementation-owners.md`](implementation-owners.md).
