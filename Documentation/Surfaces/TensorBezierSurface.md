# Tensor Bezier surface

`FTensorBezierSurface` is the public mathematical foundation for finite smooth
surfaces whose geometry varies in two independent parameters. It is generic:
it contains no host-project fixture, arena, scenario, or gameplay vocabulary.

The control net is stored in U-major order. Evaluation uses de Casteljau in
both parameters and exposes exact first derivatives and the oriented smooth
normal. Supported degrees are 1 through 15 on each axis; validation rejects an
incomplete, non-finite, or sampled-singular control net.

## Finite query approximation

`BuildBilinearApproximation` recursively subdivides the tensor surface into
finite parameter cells. For each cell, the four corners define a bilinear
patch. That patch is degree-elevated to the tensor degree and subtracted from
the cell control net. The Bezier convex-hull property therefore makes the
largest difference-control-point norm a conservative positional error bound
for the complete cell, not merely for sampled points.

`FTensorBezierPatch` is the immutable runtime query owner around this
mathematical surface. It adds:

- stable source, surface, feature, primitive, and canonical-group identity;
- collision policy/filtering and per-patch authority eligibility;
- finite certified cells split into deterministic triangle facets;
- a deterministic query-only BVH over those cells, rebuilt from the exact
  control net and excluded from serialization/stable hashing;
- native ray, sphere, and oriented-box TOI, penetration, release, feature, and
  witness results. Sphere casts include face, edge, and vertex contacts;
- smooth-normal recovery from the exact tensor surface while preserving a
  separating-axis MTD for initial overlaps;
- schema-versioned serialization and stable hashing of the exact control nets.

Authority is still explicit and blocking. A patch must be query-enabled, carry
valid nonzero identities, and rebuild a certified approximation before
`bAuthorityEligible` can be accepted. Recognition or fitting alone therefore
remains draft geometry. Authored support-transition policy is intentionally a
separate host mapping rather than vocabulary embedded in this generic type.

The query path traverses cell bounds expanded by each cell's geometric error
certificate. Leaf traversal retains original cell indices for stable feature
identities and hit tie-breaking, so spatial pruning changes cost rather than
physical results.

Generated assets persist tensor patches in runtime schema 6 / bake schema 8.
The editor bake command also supports an optional `-CompactTemplate=/Game/...`
source. It accepts only matching mesh path, LOD and triangle hash, maps exact
control nets through source-local space into the target transform, remaps
stable identifiers and collision policy, and fails explicitly if a requested
template has no compatible tensor patch.

## Contract

`IAMSPEED.PHYS.CONTACT_DETECTION.V1.TENSOR_APPROXIMATION`, implemented by
`IAmSpeed.AnalyticWorld.TensorBezierSurface`, covers exact position and
derivative evaluation, orientation, regularity, determinism, recursive
subdivision, the conservative bilinear error certificate, runtime authority and
canonical-family filters, ray/sphere/oriented-box sweeps, exact repetition,
initial penetration, shallow separating/closing release semantics, and complete
construction of the deterministic cell hierarchy.
