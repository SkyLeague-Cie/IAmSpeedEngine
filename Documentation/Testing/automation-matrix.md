# Automation-to-contract matrix

| Unreal automation path | Stable contract identifiers |
| --- | --- |
| `IAmSpeed.AnalyticWorld.BoundedPlane` | `IAMSPEED.PHYS.CONTACT_DETECTION.V1.RAY.PLANE`, `.SPHERE.PLANE`, `.BOX.PLANE`, `.DOMAIN`, `.FILTERS` |
| `IAmSpeed.AnalyticWorld.TriangleFaceBvh` | `IAMSPEED.PHYS.CONTACT_DETECTION.V1.SYMMETRY`, `.FILTERS`, `.DETERMINISM` |
| `IAmSpeed.AnalyticWorld.ExtrudedQuintic` | `IAMSPEED.PHYS.CONTACT_DETECTION.V1.BOX.PLANE`, `.SYMMETRY`, `.DETERMINISM` |
| `IAmSpeed.AnalyticWorld.FlatLandscapeAdapter` | `IAMSPEED.PHYS.CONTACT_DETECTION.V1.DOMAIN`, `.FILTERS`, `.SERIALIZATION` |
| `IAmSpeed.AnalyticWorld.LandscapeSourceIdentity` | Actual Landscape adapter/runtime bridge in normal and PIE 0/7 packages; authored source/primitive identity and support-component lookup (not physical map equilibrium) |
| `IAmSpeed.PhysicalLaws.RestingEquilibrium.BoxSupportCertificate` | `IAMSPEED.PHYS.RESTING_EQUILIBRIUM.V1`, `.CONTACT_DETECTION.V1.DOMAIN`, `.FILTERS` |
| `IAmSpeed.PhysicalLaws.RestingEquilibrium.BoxUnquantizedIntegration` | `IAMSPEED.PHYS.RESTING_EQUILIBRIUM.V1` (six faces, finite plane, actual component integrator, no sleep/quantization; not a complete host-world test) |
| `IAmSpeed.PhysicalLaws.BoxStandingStill.CanonicalQuantization` | `IAMSPEED.PHYS.BOX_STANDING_STILL.V1` (36 cells, 9000 frames, non-grid planes and local box rotation, component quantization/integration and release controls) |
| `IAmSpeed.PhysicalLaws.BoxEquilibrium.CanonicalDrops` | `IAMSPEED.PHYS.BOX_EQUILIBRIUM.V1` (215 real canonical cells: six aligned face drops, 108 perturbations, sliding/COM/material/phase/provider controls and independent inertia covariance; runs83–87 pass convergence, sampled nonpenetration and zero budget exhaustion) |
| `IAmSpeed.PhysicalLaws.BoxEquilibrium.MicroRockingGuards` | `.BOX_EQUILIBRIUM.MICRO_ROCKING.V1` (half-position-quantum energy bound; finite support/COM/load; rejection of pure slide/spin, resolvable motion, bounce, frictionless response, separation and torque) |
| `IAmSpeed.PhysicalLaws.Friction.SupportedFaceSlideAndSpin` | Continuous four-point sliding reaction, including normal-axis spin; pressure/traction balance, dissipative power, force/torque witnesses, frictionless and permutation controls; possible slip reversal is refused |
| `IAmSpeed.PhysicalLaws.NormalResponse.SimultaneousPlanarContacts` | `.NORMAL_RESPONSE.V1`, `.FRICTION.V1`, `.SUPPORT.V1` (bounded normal complementarity, coupled Coulomb, energy/restitution/permutation/symmetry and acceleration-bias controls; not host convergence) |
| `IAmSpeed.PhysicalLaws.NormalResponse.PlanarStickingCertificate` | `.NORMAL_RESPONSE.V1`, `.FRICTION.V1`, `.SUPPORT.V1` (216 feasible impulse/COM/permutation cells, independent impulse/moment and Coulomb-cone reconstruction, infeasible/elastic controls) |
| `IAmSpeed.PhysicalLaws.NormalResponse.EdgeStickingCertificate` | `.NORMAL_RESPONSE.V1`, `.FRICTION.V1` (recorded two-point failure, independent impulse/moment/cone/energy checks and 36 rotated/translated/spinning/order variants; leaves edge-axis rotation free) |
| `IAmSpeed.PhysicalLaws.BoxEquilibrium.FiniteSupportSegment` | `IAMSPEED.PHYS.BOX_EQUILIBRIUM.V1`, `.CONTACT_DETECTION.V1.DOMAIN` (continuous planar support, concave notch with supported endpoints, filters; not a release-trajectory qualification) |
| `IAmSpeed.PhysicalLaws.BoxEquilibrium.NonGridApproachQuantization` | `.BOX_EQUILIBRIUM.V1`, `.CONTACT_DETECTION.V1` (no rounding of an approaching, not-yet-supported body into a non-grid static plane) |
| `IAmSpeed.PhysicalLaws.BoxEquilibrium.PlanarCornerRounding` | `.BOX_EQUILIBRIUM.V1`, `.CONTACT_DETECTION.V1.BOX.PLANE` (six faces × three yaws/heights; 270 adjacent-representable poses report arithmetic-only disagreement as advisory debt; 108 finite overlap/separation controls remain blocking) |
| `IAmSpeed.PhysicalLaws.Integration.SmallRotationContinuity` | `IAMSPEED.PHYS.BOX_EQUILIBRIUM.V1` integrator prerequisite (finite tiny rotations and split constant-spin integration across the former 1e-8-radian cutoff) |

Most AnalyticWorld rows are pure-query tests; LandscapeSourceIdentity also
exercises UObject import and source registration. The RestingEquilibrium rows add
load feasibility and component integration; they do not certify host gameplay.
Reserved identifiers without an automation
path are open coverage, not implied passing tests. Actor/component harness
paths will be added only when they execute from the repository-owned
HostProject against both `UnrealLegacy` and `Analytic`.
