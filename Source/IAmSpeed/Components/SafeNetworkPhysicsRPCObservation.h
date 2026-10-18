#pragma once

#include "CoreMinimal.h"
#include "Physics/NetworkPhysicsComponent.h"
#include "UObject/UnrealType.h"

// Reads the actual reflected RPC argument, not InputsToNetwork_Owner's target.
// This is a pre-dispatch observation: no assertion of serialization or delivery.
struct FSpeedInputRPCObservation
{
    bool bParameterMatched = false;
    bool bOwnerMatched = false;
    bool bHistoryPresent = false;
    bool bHistoryHasData = false;
    bool bDeltaSerializationIssue = false;
    int32 HistoryCount = INDEX_NONE;
    int32 EarliestFrame = INDEX_NONE;
    int32 LatestFrame = INDEX_NONE;
};

inline FSpeedInputRPCObservation ObserveSpeedInputRPC(
    const UNetworkPhysicsComponent* ExpectedOwner, const UFunction* Function, const void* Parameters)
{
    FSpeedInputRPCObservation Result;
    if (!Function || !Parameters || Function->GetFName() != FName(TEXT("ServerReceiveInputData")))
        return Result;
    const FStructProperty* Property = FindFProperty<FStructProperty>(Function, TEXT("ClientInputs"));
    if (!Property || !Property->HasAnyPropertyFlags(CPF_Parm) ||
        Property->Struct != FNetworkPhysicsRewindDataInputProxy::StaticStruct())
        return Result;
    const auto* Proxy = Property->ContainerPtrToValuePtr<FNetworkPhysicsRewindDataInputProxy>(Parameters);
    Result.bParameterMatched = true;
    Result.bOwnerMatched = Proxy->Owner.Get() == ExpectedOwner;
    Result.bHistoryPresent = bool(Proxy->History);
    Result.bDeltaSerializationIssue = Proxy->bDeltaSerializationIssue;
    if (Proxy->History)
    {
        Result.HistoryCount = Proxy->History->GetHistorySize();
        Result.EarliestFrame = Proxy->History->GetEarliestFrame();
        Result.LatestFrame = Proxy->History->GetLatestFrame();
        Result.bHistoryHasData = Proxy->History->HasDataInHistory();
    }
    return Result;
}
