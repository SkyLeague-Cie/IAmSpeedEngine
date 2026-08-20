# Automation-to-contract matrix

| Unreal automation path | Stable contract identifiers |
| --- | --- |
| `IAmSpeed.AnalyticWorld.BoundedPlane` | `IAMSPEED.PHYS.CONTACT_DETECTION.V1.RAY.PLANE`, `.SPHERE.PLANE`, `.BOX.PLANE`, `.DOMAIN`, `.FILTERS` |
| `IAmSpeed.AnalyticWorld.TriangleFaceBvh` | `IAMSPEED.PHYS.CONTACT_DETECTION.V1.SYMMETRY`, `.FILTERS`, `.DETERMINISM` |
| `IAmSpeed.AnalyticWorld.ExtrudedQuintic` | `IAMSPEED.PHYS.CONTACT_DETECTION.V1.BOX.PLANE`, `.SYMMETRY`, `.DETERMINISM` |
| `IAmSpeed.AnalyticWorld.FlatLandscapeAdapter` | `IAMSPEED.PHYS.CONTACT_DETECTION.V1.DOMAIN`, `.FILTERS`, `.SERIALIZATION` |

These are current pure-query tests. Reserved identifiers without an automation
path are open coverage, not implied passing tests. Actor/component harness
paths will be added only when they execute from the repository-owned
HostProject against both `UnrealLegacy` and `Analytic`.
