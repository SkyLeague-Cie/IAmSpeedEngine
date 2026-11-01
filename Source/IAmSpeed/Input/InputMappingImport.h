#pragma once
#include "CoreMinimal.h"
#include "InputActionContract.h"
class UInputAction;
class UInputMappingContext;

namespace Speed::Input::V2
{
// GT construction only. The returned contract contains no UObject references.
// Action identity is supplied explicitly by the game, never guessed from names.
IAMSPEED_API bool ImportInputMappings(const UInputMappingContext* Context,
	const TMap<const UInputAction*, FActionId>& Actions,
	FInputActionContractDescription& Description, FString& Error);
// GT-only presentation axes. Validates the same authored mapping transforms,
// refuses physical destinations, and creates no physical contract or mapper.
IAMSPEED_API bool ImportPresentationInputMappings(const UInputMappingContext* Context,
    const TMap<const UInputAction*, FActionId>& Actions,
    FInputActionContractDescription& Description, FString& Error);
}
