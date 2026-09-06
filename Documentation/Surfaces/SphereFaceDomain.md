# Finite contact domains and conservative replacement coverage

A finite sphere/face contact is geometrically valid when the plane contact
witness projects inside the actual finite face. The whole sphere need not fit
inside that domain. Increasing the radius cannot invalidate the same interior
face overlap solely by shrinking the admissible domain.

`FAnalyticStaticCollisionWorld::SweepSingle` selects this strict face policy.
The round plane kernel retains `ContainsProjectedPoint`; extruded profiles
retain their actual chord/extrusion domains. No infinite tangent plane, phantom
bridge, additional margin, or artificial terminal edge wall is introduced.
Finite OBB SAT proves box intersections even when the box overhangs an authored
domain. Its geometry, feature selection and concave internal-face proof remain
unchanged. Strict mode must not discard such a proven intersection merely
because its witness does not satisfy the conservative replacement guard.
In particular, no projection of an extreme vertex onto an infinite tangent
plane substitutes for the finite SAT proof near an extrusion boundary.

`TrySweepAuthority` is different: it certifies replacement of legacy collision.
Its full-footprint requirement and draft-provider comparison remain intact,
including when the caller supplied a strict finite-domain query. Established
contact reacquisition remains a separate policy, not a prerequisite for a fresh
finite sphere/face impact. The exact-query cache includes the domain-policy bit.

The regression witness exposed an inflated projection sphere missing a finite
gutter face that its smaller physical sphere penetrated by about0.22cm. An
earlier full-footprint handover had also introduced a delayed12cm projection.
This is a query-coverage error, not restitution or a radius-tuning problem.

`IAmSpeed.AnalyticWorld.SphereFiniteFaceDomain` covers all four rectangle
boundaries for planes/extrusions, increasing radii, finite-domain misses,
fresh CCD, conservative coverage preservation, cache isolation and a frozen
curved-profile witness. Initial source170 is deliberately red; candidate171
passes the native and isolated ball reproducer, with wider trajectory gates
still unresolved. `IAmSpeed.AnalyticWorld.BoxFiniteContactDomain` adds partial
overhangs, real-edge CCD, finite OBB misses despite overlapping AABBs, Hybrid
policy isolation and a frozen curved overlap previously hidden by coverage.
Its source175 is deliberately red;176 admits the proven finite contact and178
repairs the selected box witness AFTER world arbitration, preserving contact
point, normal, depth and TOI. Native184 passes the finite-domain controls and
three adverse witness tests (including valid Hybrid covered misses). The former
large lip overlap is removed in paired host replay, but other trajectory gates
remain open. Neither document nor a green native test certifies the stadium.

This first-hit sweep contract is not a deepest-overlap certificate. An equal-TOI
tangency can win over another provider's penetration under existing sweep
ordering. An explicit stationary overlap operation is planned separately; it
must not silently change moving-sweep or Hybrid replacement semantics.
