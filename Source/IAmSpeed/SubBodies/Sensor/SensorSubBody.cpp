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
    PrevKinematics = GetKinematicState();
    // keep CurrentOverlaps across frames (so EndOverlap fires correctly)
    // OR clear if you want "stateless" sensors.
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

    // Mark touched this substep
    TouchedThisSubstep.Add(Other);

    // Already overlapping => nothing to do here (continuous handled later)
    if (OverlapsSet.Contains(Other))
    {
        return;
    }

    // BeginOverlap: preserve order = time we first detect it via TOI
    OverlapsOrdered.Add(Other);
    OverlapsSet.Add(Other);
    OnBeginOverlap.Broadcast(this, Other);
}

void USensorSubBody::PostPhysicsUpdate()
{
    UWorld* World = GetWorld();
    if (!World) return;

    // Recompute current overlaps at current pose
    TArray<FOverlapResult> Overlaps;
    FCollisionQueryParams Params = BuildTraceParams();
    // (BuildTraceParams already includes IgnoredComponents)

    const FVector Pos = GetKinematicState().Location;
    const FQuat Rot = GetKinematicState().Rotation;

    const bool bAny = World->OverlapMultiByChannel(
        Overlaps,
        Pos,
        Rot,
        GetCollisionChannel(),
        GetCollisionShape(),
        Params,
        GetResponseParams()
    );

    TSet<TWeakObjectPtr<UPrimitiveComponent>> Now;
    if (bAny)
    {
        for (const FOverlapResult& R : Overlaps)
        {
            if (UPrimitiveComponent* C = R.Component.Get())
            {
                Now.Add(C);
            }
        }
    }

    // 1) EndOverlap in begin-order (stable)
    for (int32 i = OverlapsOrdered.Num() - 1; i >= 0; --i)
    {
        UPrimitiveComponent* C = OverlapsOrdered[i].Get();
        if (!C || !Now.Contains(C))
        {
            if (C) OnEndOverlap.Broadcast(this, C);
            OverlapsSet.Remove(C);
            OverlapsOrdered.RemoveAtSwap(i);
        }
    }

    // 2) ContinuousOverlap in begin-order
    for (TWeakObjectPtr<UPrimitiveComponent> WeakC : OverlapsOrdered)
    {
        if (UPrimitiveComponent* C = WeakC.Get())
        {
            if (Now.Contains(C))
            {
                OnContinuousOverlap.Broadcast(this, C);
            }
        }
    }

    TouchedThisSubstep.Reset();
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

