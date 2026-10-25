# Automation-to-contract matrix

| Unreal automation path | Stable contract identifiers |
| --- | --- |
| `IAmSpeed.AnalyticWorld.BoundedPlane` | `IAMSPEED.PHYS.CONTACT_DETECTION.V1.RAY.PLANE`, `.SPHERE.PLANE`, `.BOX.PLANE`, `.DOMAIN`, `.FILTERS` |
| `IAmSpeed.AnalyticWorld.TriangleFaceBvh` | `IAMSPEED.PHYS.CONTACT_DETECTION.V1.SYMMETRY`, `.FILTERS`, `.DETERMINISM` |
| `IAmSpeed.AnalyticWorld.ExtrudedQuintic` | `IAMSPEED.PHYS.CONTACT_DETECTION.V1.BOX.PLANE`, `.SYMMETRY`, `.DETERMINISM` |
| `IAmSpeed.AnalyticWorld.SphereFiniteFaceDomain` | Finite plane/extruded sphere-face witnesses, radius monotonicity, fresh CCD, real boundaries, conservative Hybrid coverage and cache-policy isolation; frozen curved-profile regression |
| `IAmSpeed.AnalyticWorld.BoxFiniteContactDomain` | Finite overhangs, section/extrusion edge CCD, overlapping-AABB negative OBB, coplanar provider seams, Hybrid isolation and frozen curved overlap; matching MTD witnesses |
| `IAmSpeed.AnalyticWorld.ExtrudedInternalContactNormal` | Finite concave internal-facet contact: exact overlapping-provider regression, coherent depth/normal/witnesses, four XY reflections and three positional perturbations; no host-trajectory certification |
| `IAmSpeed.AnalyticWorld.TensorInternalContactNormal` | Frozen finite piecewise tensor neighborhood reproducing a false internal-edge normal; depth/witness and mirror/perturbation checks; no host certification |
| `IAmSpeed.AnalyticWorld.TensorConcavityTopology` | Derived internal-edge masks: concave/convex sides, real free boundaries and absent C2 adjacency; no mask relaxation at unproved edges |
| `IAmSpeed.AnalyticWorld.TensorInteriorVertexNormal` | Frozen finite saddle neighborhood reproducing a false interior-vertex MTD; no host certification |
| `IAmSpeed.AnalyticWorld.TensorCornerConeTopology` | Complete C2 four-cell fans on plane/bowl/saddle, open-ring refusal, exterior borders and derived-data hash exclusion |
| `IAmSpeed.AnalyticWorld.FlatLandscapeAdapter` | `IAMSPEED.PHYS.CONTACT_DETECTION.V1.DOMAIN`, `.FILTERS`, `.SERIALIZATION` |
| `IAmSpeed.AnalyticWorld.LandscapeSourceIdentity` | Actual Landscape adapter/runtime bridge in normal and PIE 0/7 packages; authored source/primitive identity and support-component lookup (not physical map equilibrium) |
| `IAmSpeed.PhysicalLaws.RestingEquilibrium.BoxSupportCertificate` | `IAMSPEED.PHYS.RESTING_EQUILIBRIUM.V1`, `.CONTACT_DETECTION.V1.DOMAIN`, `.FILTERS` |
| `IAmSpeed.PhysicalLaws.RestingEquilibrium.BoxUnquantizedIntegration` | `IAMSPEED.PHYS.RESTING_EQUILIBRIUM.V1` (six faces, finite plane, actual component integrator, no sleep/quantization; not a complete host-world test) |
| `IAmSpeed.PhysicalLaws.BoxStandingStill.CanonicalQuantization` | `IAMSPEED.PHYS.BOX_STANDING_STILL.V1` (36 cells, 9000 frames, non-grid planes and local box rotation, component quantization/integration and release controls) |
| `IAmSpeed.PhysicalLaws.BoxEquilibrium.CanonicalDrops` | `IAMSPEED.PHYS.BOX_EQUILIBRIUM.V1` (215 real canonical cells: six aligned face drops, 108 perturbations, sliding/COM/material/phase/provider controls and independent inertia covariance; convergence, sampled nonpenetration and zero-budget-exhaustion checks; current results must come from the exact native report, including any admitted debt) |
| `IAmSpeed.PhysicalLaws.BoxEquilibrium.MicroRockingGuards` | `.BOX_EQUILIBRIUM.MICRO_ROCKING.V1` (half-position-quantum energy bound; finite support/COM/load; rejection of pure slide/spin, resolvable motion, bounce, frictionless response, separation and torque) |
| `IAmSpeed.PhysicalLaws.Friction.SupportedFaceSlideAndSpin` | Continuous four-point sliding reaction, including normal-axis spin; pressure/traction balance, dissipative power, force/torque witnesses, frictionless and permutation controls; possible slip reversal is refused |
| `IAmSpeed.PhysicalLaws.NormalResponse.SimultaneousPlanarContacts` | `.NORMAL_RESPONSE.V1`, `.FRICTION.V1`, `.SUPPORT.V1` (bounded normal complementarity, coupled Coulomb, energy/restitution/permutation/symmetry and acceleration-bias controls; not host convergence) |
| `IAmSpeed.PhysicalLaws.NormalResponse.PlanarStickingCertificate` | `.NORMAL_RESPONSE.V1`, `.FRICTION.V1`, `.SUPPORT.V1` (216 feasible impulse/COM/permutation cells, independent impulse/moment and Coulomb-cone reconstruction, infeasible/elastic controls) |
| `IAmSpeed.PhysicalLaws.NormalResponse.EdgeStickingCertificate` | `.NORMAL_RESPONSE.V1`, `.FRICTION.V1` (recorded two-point failure, independent impulse/moment/cone/energy checks and 36 rotated/translated/spinning/order variants; leaves edge-axis rotation free) |
| `IAmSpeed.PhysicalLaws.BoxEquilibrium.FiniteSupportSegment` | `IAMSPEED.PHYS.BOX_EQUILIBRIUM.V1`, `.CONTACT_DETECTION.V1.DOMAIN` (continuous planar support, concave notch with supported endpoints, filters; not a release-trajectory qualification) |
| `IAmSpeed.PhysicalLaws.BoxEquilibrium.NonGridApproachQuantization` | `.BOX_EQUILIBRIUM.V1`, `.CONTACT_DETECTION.V1` (no rounding of an approaching, not-yet-supported body into a non-grid static plane) |
| `IAmSpeed.PhysicalLaws.BoxEquilibrium.PlanarCornerRounding` | `.BOX_EQUILIBRIUM.V1`, `.CONTACT_DETECTION.V1.BOX.PLANE` (six faces × three yaws/heights; 270 adjacent-representable poses report arithmetic-only disagreement as advisory debt; 108 finite overlap/separation controls remain blocking) |
| `IAmSpeed.PhysicalLaws.Integration.SmallRotationContinuity` | `IAMSPEED.PHYS.BOX_EQUILIBRIUM.V1` integrator prerequisite (finite tiny rotations and split constant-spin integration across the former 1e-8-radian cutoff) |
| `IAmSpeed.Simulation.FrameParallel.NativeAnalyticContacts1221` | Native owner Normal/Fast at 300 Hz, requests 1/2/4/8/12, ordered contact/state/input comparisons and evidence of analytical work computed and consumed in parallel; no whole-Stadium qualification. |
| `IAmSpeed.Simulation.FrameParallel.NativeAnalyticGrounded1368` | Normal/Fast requests 1/2/4/8/12 with grounded wheel/suspension and contact-state witnesses; mode-specific camera contract. |
| `IAmSpeed.Simulation.FrameParallel.NativeAnalyticEightCars1552` | Eight cars Normal/Fast requests 1/8, real consumed commands, canonical state/contacts, wheel and joined-preparation witnesses; complete callback/audio coverage remains separate. |

Most AnalyticWorld rows are pure-query tests; LandscapeSourceIdentity also
exercises UObject import and source registration. The RestingEquilibrium rows add
load feasibility and component integration; they do not certify host gameplay.
Reserved identifiers without an automation
path are open coverage, not implied passing tests. Actor/component and native
worker fixtures now exist alongside pure-query tests. Their backend and worker
matrices are defined by each fixture; a NativeAnalytic result does not implicitly
qualify UnrealLegacy, all worker counts or a product map. The inventory and this
table describe test scope, not a current green CI verdict. See
[performance observability](simulation-performance.md) for timing scope and
[netcode status](../Netcode.md) for the remaining end-to-end network gates.
