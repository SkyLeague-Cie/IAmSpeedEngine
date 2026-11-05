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
// Explicit product policy for intentionally unassigned authored action slots.
// Each listed Wired action must have one actual asset identity and only
// Invalid/None rows; missing/corrupt rows never count as player intent.
// IDs, wiring, physical destinations and final Create validation stay intact.
IAMSPEED_API bool ImportInputMappingsAllowUnbound(const UInputMappingContext* Context,
    const TMap<const UInputAction*, FActionId>& Actions,const TSet<FActionId>& UnboundActions,
    FInputActionContractDescription& Description,FString& Error);
// GT-only presentation axes. Validates the same authored mapping transforms,
// refuses physical destinations, and creates no physical contract or mapper.
IAMSPEED_API bool ImportPresentationInputMappings(const UInputMappingContext* Context,
    const TMap<const UInputAction*, FActionId>& Actions,
    FInputActionContractDescription& Description, FString& Error);
}
