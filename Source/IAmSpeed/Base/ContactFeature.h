#pragma once

#include "CoreMinimal.h"

namespace Speed
{

// Local geometric dimension of a contact witness. Authored surface identity
// remains represented separately by the stable source/surface/feature ids.
enum class EContactFeatureKind : uint8
{
	Unknown = 0,
	Face = 1,
	Edge = 2,
	Vertex = 3,
};

} // namespace Speed
