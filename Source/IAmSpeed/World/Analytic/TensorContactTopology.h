#pragma once

namespace Speed::Analytic
{
struct FPiecewiseTensorBezierPatch;
/** Rebuild immutable concave-edge masks and closed-corner normal envelopes.
 * Only certified finite C2 topology can remove an internal feature's false
 * response direction; open, non-manifold and unproved features retain SAT. */
void BuildTensorContactTopology(FPiecewiseTensorBezierPatch& Patch);
}
