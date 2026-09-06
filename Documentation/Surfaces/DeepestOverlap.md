# Explicit stationary overlap queries

`IStaticCollisionWorld::TryFindDeepestOverlap` answers a different question
from `SweepSingle`: which eligible finite constraint has the greatest positive
penetration at a fixed pose? A tangent or shallow first CCD contact cannot
certify that the rest of a compound world is clear.

The return value describes operation availability. `false` rejects unsupported
or malformed queries, including movement between Start and End. `true` with a
miss denotes no positive overlap. Sphere/box shapes retain their exact pose,
dimensions, collision filters and optional source/surface/normal restrictions.
The operation removes the CCD initial-contact release shell, not the kernel's
finite-domain/numerical classification. It never extends an infinite plane or
adds approximation error to the physical depth.

The analytic implementation reuses the bounded spatial traversal and finite
kernels. A PRIVATE selection policy reaches plane triangles, extruded chords,
tensor approximations/cells and the final world reducer. Depths are compared
exactly, then stable source/surface/feature/primitive IDs resolve exact ties.
The policy is included in cache-entry equality; the existing bounded cache is
retained. Neither public Sweep nor Hybrid can select that private policy.

This is a maximum single-constraint depth, NOT the compound world's global
minimum translation or a list of all contacts. Projection may require bounded
iterations and a fresh residual check. Missing/unsupported operations must
never be converted into an empty-space certificate or unsafe Legacy fallback.

The strict `TryCompactAuthorityMulti` adapter returns zero or one deepest hit.
Its Hybrid behavior is unchanged. All six existing callers are stationary:
sphere projection/residual, hitbox feasibility and three test overlap oracles.

Final box-pose quantization also uses this operation. It uses the same static
object-type scope as hitbox penetration projection, including query-enabled
surfaces with an empty trace response mask. This does not enable their CCD trace
response. Unsupported queries reject rounding; exterior integrated pose is
retained, rather than accepting an unproved quantized pose. Finite domains,
query-disabled data and non-static object types remain excluded as appropriate.

Zero domain allowance is supported. For a SAT-proven box overlap on a joined
tensor face, projected and barycentrically reconstructed points need not have
identical bits. Their finite-face membership comparison retains a coordinate-
scaled double-roundoff bound even when the requested domain allowance is zero.
This does not expand SAT, alter depth or relax the penetration acceptance test.
`BoxPoseJoinedQueryTolerance` reproduces the old false miss and exercises
reflections, translations, perturbations and the finite exterior.

Qualification: RED187 independently demonstrates shallow-first masking for
spheres/boxes against two planes. The first piecewise fixture also had an
incorrect adjacency expectation for degree-one cells; degree-two exact planar
cells carry the required second derivatives. Native193 passes78 tests, including
connected-cell, shallow-shell, finite-query validation, filtering and cache probes.
Host overlap comparisons must observe the same completed pose, with full sample
coverage, and retain intermediate evidence separately. This does not certify all
compound-world trajectories or eliminate the need for final residual validation.
