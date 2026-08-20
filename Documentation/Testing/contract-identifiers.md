# Contract test identifiers

Every public contract case begins with its PhysicalLaw identifier. The initial
reserved cases are:

| Identifier | Contract |
| --- | --- |
| `IAMSPEED.PHYS.CONTACT_DETECTION.V1.RAY.PLANE` | Ray versus bounded plane |
| `IAMSPEED.PHYS.CONTACT_DETECTION.V1.SPHERE.PLANE` | Sphere sweep versus bounded plane |
| `IAMSPEED.PHYS.CONTACT_DETECTION.V1.BOX.PLANE` | Oriented box sweep versus bounded plane |
| `IAMSPEED.PHYS.CONTACT_DETECTION.V1.EDGE_VERTEX` | Edge/vertex feature ownership |
| `IAMSPEED.PHYS.CONTACT_DETECTION.V1.DOMAIN` | Finite domain acceptance/rejection |
| `IAMSPEED.PHYS.CONTACT_DETECTION.V1.PENETRATION` | Initial penetration metadata |
| `IAMSPEED.PHYS.CONTACT_DETECTION.V1.FILTERS` | Mutual query/object filtering |
| `IAMSPEED.PHYS.CONTACT_DETECTION.V1.SYMMETRY` | Reflected/rotated equivalents |
| `IAMSPEED.PHYS.CONTACT_DETECTION.V1.DETERMINISM` | Repeated stable result and ordering |
| `IAMSPEED.PHYS.CONTACT_DETECTION.V1.SERIALIZATION` | Asset/runtime reconstruction |
| `IAMSPEED.PHYS.NORMAL_RESPONSE.V1.FALL_CONTACT` | Free fall, first contact, non-closing response |
| `IAMSPEED.PHYS.NORMAL_RESPONSE.V1.RESTITUTION` | Configured bounce response |
| `IAMSPEED.PHYS.FRICTION.V1.HORIZONTAL` | Horizontal slip decay/work sign |
| `IAMSPEED.PHYS.SLIDING.V1.INCLINE` | Sustained inclined sliding |
| `IAMSPEED.PHYS.ROLLING.V1.HORIZONTAL` | Pure rolling/slip residual |
| `IAMSPEED.PHYS.RESTING_EQUILIBRIUM.V1.RELEASE` | Exact rest and external release |
| `IAMSPEED.PHYS.SUPPORT.V1.ACQUIRE_RELEASE` | Support acquisition and release topology |
| `IAMSPEED.PHYS.MULTI_CONTACT_EQUILIBRIUM.V1.BOX` | Deterministic compatible multi-contact pose |
| `IAMSPEED.PHYS.SUSPENSION.V1.CCD` | Fast approach, travel, force, and release |

Every harness case is executed through the same sealed fixed-step scenario for
`UnrealLegacy` and `Analytic`. Results carry backend, platform, physical delta,
seed/initial state, semantic assertions, field comparison classes, duration,
and deterministic hash.
