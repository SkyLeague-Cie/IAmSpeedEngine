# BoxEquilibrium

Identifier: `IAMSPEED.PHYS.BOX_EQUILIBRIUM.V1`.
CompositionLevel: Molecular. Foundation: yes.
Dependencies: BoxStandingStill, ContactDetection, NormalResponse, Friction,
Support and Sliding.

This is **acquisition and maintenance of a stable supported face** after a
fall, impact or tipping motion. For each declared dissipative fixture, measure
first contact, transient impacts, convergence and a sustained terminal interval.
Initial pose/velocities are set once, then ordinary physics owns the trajectory.

For a horizontal static plane with normal n, the supported terminal state has:

- a box face parallel to the plane, with feasible force/torque support;
- dot(v, n) = 0 and omega = 0 in every component;
- zero penetration throughout, not only at the terminal snapshot;
- arbitrary admissible tangential velocity v - dot(v,n)n, evolving under the
  configured friction/load law rather than being artificially held constant.

With zero tangential velocity the state must satisfy BoxStandingStill. Sliding
is not total mechanical rest; friction may dissipate its kinetic energy. If
the complete load (including friction torque) no longer admits face support,
tipping/release is physical and must not be suppressed by a pose lock.

For one selected face of a rigid box on a fixed plane, the geometric center's
normal distance is the corresponding half-extent, independent of the approach,
slide speed and world tangential position. The face normal fixes inclination;
heading about the plane normal remains free. Once acquired, sliding with zero
omega preserves that heading and height while the support remains feasible.
Derive host actor and COM poses using the actual local box transform and COM
offset, never a separately tuned per-scenario equilibrium height or rotation.
Sliding tests must measure these geometric invariants throughout supported
travel, not merely sample a stationary final pose.

Finite observation/convergence horizons and dissipative material parameters
are explicit test inputs. Do not promise convergence for a perfectly elastic
model, ongoing excitation, insufficient support, or exact unstable edge/vertex
equilibria. Such cases are reported separately; they are not silently dropped
from coverage. Perturbations of unstable equilibria test the physical departure.

An impulse, changed topology, hole or incompatible contact can invalidate the
supported state. The body must resume ordinary CCD/response before the affected
segment. No preferred face belongs in this generic law.

The isolated planar impulse/reaction kernel uses free-body inverse inertia.
It must defer to the coupled response while another sub-body supplies active
support. `HasActivePhysicalConstraintsOtherThan` owns that distinction:
ordinary rigid contacts are handled in the common component; the wheeled
component additionally reports its separate live and pending wheel manifold.
A pending contact still belongs to the grouped solve even after its ground
flag clears. The same guard applies before planar position projection: moving
the box alone could lift another valid support before the coupled solve.
This is a state-dependent support query, not a box/vehicle subtype test.

Post-transport velocity feasibility uses the fresh geometric projection
witness, not a previous CCD event on a nearby plane. Such a projection has
zero restitution and does not replay an impact notification. Restitution is
reserved for actual incoming CCD events. The pending event is preserved.

Resolution-bounded micro-rocking closure (explicitly approved 2026-09-06):
after a solved inelastic impact on a complete planar face, a fresh geometric
support certificate may terminate an accumulating sequence of tiny rocking
impacts. Incoming total kinetic energy per mass, using the actual inertia,
must be strictly below normal load times half the canonical position step
(0.005 cm). Only velocity and spin are set to zero; the event pose is unchanged.
Additional conservative energy bounds limit Coulomb stopping travel, every
corner/COM rocking excursion and escape over the nearest support edge. Thus
tiny rocking cannot erase a nearly frictionless slide or capture a COM close
to tipping merely because its projection is still inside the face.
This is deliberate dissipative numerical stabilization, not an exact impulse
solution. It is neither a general sleep threshold nor an angular pose snap.
Pure slides/spins, elastic or frictionless impacts, separating motion, ongoing
torque, incompatible loads, invalid COM, penetration and incomplete support
are refused. The next frame revalidates ordinary support, so an impulse or
removed support immediately releases it. `MicroRockingGuards` protects the
energy boundary and these exclusions; canonical trajectories qualify its use.

Status: the focused canonical engine campaign passes at
`Boundary104Validation`: 215 cells, including six aligned drops requiring
their own face, 108 perturbed acquisitions and sliding/COM/material/phase/provider
controls. Every cell maintains its required terminal interval, with no sampled
penetration and no CCD-budget exhaustion. The approved micro-rocking closure
resolved the two nonconvergent / three budget-exhausting cells of binary79.
All 63 IAmSpeed automations pass (one has expected synthetic Landscape warnings).
This is focused engine evidence, not a claim that all host or stadium cases pass.
Native host rest/sliding and selected acquisitions also pass on stadium and
actual flat Landscape; separate host recovery debts remain documented there.
Broader angular/contact-domain and finite-support release coverage is not
implied by this focused qualification.
The existing box resting certificate is not evidence of general acquisition.

A supported face can also slide while spinning around its normal. This is not
StandingStill or zero-spin Equilibrium, but still requires continuous support.
`SolvePlanarSlidingReaction` couples normal pressure with per-point Coulomb
traction against the actual slip. It retains tangent motion/normal spin,
rejects possible slip reversal during the frame horizon, and does not use the
micro-rocking stop certificate. `SupportedFaceSlideAndSpin` checks pressure,
force/torque reconstruction, dissipation and frictionless/permutation controls.

Moving full-face reactions are admitted over a common conservative frame
horizon in sweep prediction and integration. Each supporting point must stay
on one exact finite planar provider over its entire straight Coulomb trajectory,
up to its physical stop. This rejects a concave notch even if both endpoints
are supported. Unsupported provider handover returns to general collision
resolution, whose release/acquisition trajectory still needs qualification.
