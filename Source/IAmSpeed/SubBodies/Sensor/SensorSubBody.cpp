// Fill out your copyright notice in the Description page of Project Settings.


#include "SensorSubBody.h"
#include "Engine/OverlapResult.h"

USensorSubBody::USensorSubBody(const FObjectInitializer& ObjectInitializer) :
    Super(ObjectInitializer)
{

}

void USensorSubBody::Initialize(ISpeedComponent* InParentComponent)
{
    Super::Initialize(InParentComponent);

    // Sensors should overlap, not block.
    ResponseParams.CollisionResponse.SetAllChannels(ECR_Ignore);
}

void USensorSubBody::ResetForFrame(const float& Delta)
{
    Super::ResetForFrame(Delta);
    TouchedThisFrame.Reset();
}

void USensorSubBody::ResolveCurrentHit(const float& Delta, const float& SimTime)
{
	if (CurrentHit.bHit)
    {
		ResolveCurrentHitPrv(Delta, SimTime);
    }
}

void USensorSubBody::ResolveCurrentHitPrv(const float& Delta, const float& SimTime)
{
    UPrimitiveComponent* Other = GetHit().Component.Get();
    if (!Other) return;

    // Mark touched this frame
    TouchedThisFrame.Add(Other);

    // Already overlapping
    if (OverlapsSet.Contains(Other))
    {
        return;
    }

    // BeginOverlap: preserve order of first contact
    OverlapsOrdered.Add(Other);
    OverlapsSet.Add(Other);

    OnBeginOverlap.Broadcast(this, Other);
}

void USensorSubBody::PostPhysicsUpdate()
{
    // 1) EndOverlap: any previously overlapping component not touched this frame
    for (int32 i = OverlapsOrdered.Num() - 1; i >= 0; --i)
    {
        UPrimitiveComponent* C = OverlapsOrdered[i].Get();
        const bool bStillValid = (C != nullptr);
        const bool bTouched = bStillValid && TouchedThisFrame.Contains(C);

        if (!bTouched)
        {
            if (C) OnEndOverlap.Broadcast(this, C);
            OverlapsSet.Remove(C);
            OverlapsOrdered.RemoveAtSwap(i);
        }
    }

    // 2) ContinuousOverlap: in begin order
    for (const TWeakObjectPtr<UPrimitiveComponent>& W : OverlapsOrdered)
    {
        if (UPrimitiveComponent* C = W.Get())
        {
            if (TouchedThisFrame.Contains(C))
            {
                OnContinuousOverlap.Broadcast(this, C);
            }
        }
    }

    // clear per-frame touched cache
    TouchedThisFrame.Reset();
}

UPrimitiveComponent* USensorSubBody::GetFirstOverlappingComponent() const
{
    for (const TWeakObjectPtr<UPrimitiveComponent>& W : OverlapsOrdered)
    {
        if (UPrimitiveComponent* C = W.Get())
        {
            return C;
        }
    }
    return nullptr;
}

bool USensorSubBody::ShouldIgnore(const UPrimitiveComponent& Other) const
{
    return ComponentHasBeenIgnored(Other); // uses IgnoredComponents :contentReference[oaicite:5]{index=5}
}

FCollisionQueryParams USensorSubBody::BuildTraceParams() const
{
    FCollisionQueryParams Params = Super::BuildTraceParams();
    Params.bFindInitialOverlaps = true;   // IMPORTANT for persistent overlaps
    return Params;
}

