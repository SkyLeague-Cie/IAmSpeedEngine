// Fill out your copyright notice in the Description page of Project Settings.

#include "SphereSensor.h"
#include "PhysicsEngine/BodySetup.h"

USphereSensor::USphereSensor(const FObjectInitializer& ObjectInitializer) :
    Super(ObjectInitializer)
{
    SubBodyType = ESubBodyType::Sphere;
    SetRadius(32.0f);
    ShapeColor = FColor(255, 0, 0, 255);

    bUseEditorCompositing = true;
}

void USphereSensor::Initialize(ISpeedComponent* InParentComponent)
{
    Super::Initialize(InParentComponent);
    SubBodyType = ESubBodyType::Sphere; // ou laisse Unknown si tu veux “sensor-only”
}

//======================================================
// Sweep Methods
//======================================================

bool USphereSensor::SweepTOI(const float& RemainingDelta, float& OutTOI)
{
    if (!ParentComponent)
    {
        return false;
    }

    float TOI_ground = RemainingDelta;
    float TOI_Box = RemainingDelta;
    float TOI_Sphere = RemainingDelta;
    float TOI_Wheel = RemainingDelta;
    FutureHit = SHitResult();

    SHitResult GroundHitresult;
    SHitResult BoxHitresult;
    SHitResult SphereHitresult;
    SHitResult WheelHitresult;
    bool bHitGround = SweepVsGround(GetWorld(), GroundHitresult, RemainingDelta, TOI_ground);
    bool bHitBox = SweepVsBoxes(GetWorld(), BoxHitresult, RemainingDelta, TOI_Box);
    bool bHitSphere = SweepVsSpheres(GetWorld(), SphereHitresult, RemainingDelta, TOI_Sphere);
    bool bHitWheel = SweepVsWheels(GetWorld(), WheelHitresult, RemainingDelta, TOI_Wheel);

    if (!bHitGround && !bHitBox && !bHitSphere && !bHitWheel)
        return false;

    OutTOI = RemainingDelta;

    if (bHitGround && TOI_ground < OutTOI)
    {
        OutTOI = TOI_ground;
        GroundHit = GroundHitresult;
        FutureHit = GroundHit;
    }

    if (bHitBox && TOI_Box < OutTOI)
    {
        OutTOI = TOI_Box;
        BoxHit = BoxHitresult;
        FutureHit = BoxHit;
    }

    if (bHitSphere && TOI_Sphere < OutTOI)
    {
        OutTOI = TOI_Sphere;
        SphereHit = SphereHitresult;
        FutureHit = SphereHit;
    }

    if (bHitWheel && TOI_Wheel < OutTOI)
    {
        OutTOI = TOI_Wheel;
        WheelHit = WheelHitresult;
        FutureHit = WheelHit;
    }

    return true;
}


template <EShapeBodySetupHelper UpdateBodySetupAction, typename BodySetupType>
bool InvalidateOrUpdateSphereSensorSetup(BodySetupType& ShapeBodySetup, bool bUseArchetypeBodySetup, float SphereRadius)
{
    check((bUseArchetypeBodySetup && UpdateBodySetupAction == EShapeBodySetupHelper::InvalidateSharingIfStale) || (!bUseArchetypeBodySetup && UpdateBodySetupAction == EShapeBodySetupHelper::UpdateBodySetup));
    check(ShapeBodySetup->AggGeom.SphereElems.Num() == 1);
    FKSphereElem* SphereElem = ShapeBodySetup->AggGeom.SphereElems.GetData();

    // check for mal formed values
    float Radius = SphereRadius;
    if (Radius < UE_KINDA_SMALL_NUMBER)
    {
        Radius = 0.1f;
    }

    if (UpdateBodySetupAction == EShapeBodySetupHelper::UpdateBodySetup)
    {
        // now set the PhysX data values
        SphereElem->Center = FVector::ZeroVector;
        SphereElem->Radius = Radius;
    }
    else
    {
        if (SphereElem->Radius != Radius)
        {
            ShapeBodySetup = nullptr;
            bUseArchetypeBodySetup = false;
        }
    }

    return bUseArchetypeBodySetup;
}


FPrimitiveSceneProxy* USphereSensor::CreateSceneProxy()
{
    /** Represents a DrawLightRadiusComponent to the scene manager. */
    class FSphereSceneProxy final : public FPrimitiveSceneProxy
    {
    public:
        SIZE_T GetTypeHash() const override
        {
            static size_t UniquePointer;
            return reinterpret_cast<size_t>(&UniquePointer);
        }

        /** Initialization constructor. */
        FSphereSceneProxy(const USphereSensor* InComponent)
            : FPrimitiveSceneProxy(InComponent)
            , bDrawOnlyIfSelected(InComponent->bDrawOnlyIfSelected)
            , SphereColor(InComponent->ShapeColor)
            , SphereRadius(InComponent->Radius)
            , LineThickness(InComponent->LineThickness)
        {
            bWillEverBeLit = false;
        }

        // FPrimitiveSceneProxy interface.

        virtual void GetDynamicMeshElements(const TArray<const FSceneView*>& Views, const FSceneViewFamily& ViewFamily, uint32 VisibilityMap, FMeshElementCollector& Collector) const override
        {
            QUICK_SCOPE_CYCLE_COUNTER(STAT_SphereSceneProxy_GetDynamicMeshElements);

            for (int32 ViewIndex = 0; ViewIndex < Views.Num(); ViewIndex++)
            {
                if (VisibilityMap & (1 << ViewIndex))
                {
                    const FSceneView* View = Views[ViewIndex];
                    FPrimitiveDrawInterface* PDI = Collector.GetPDI(ViewIndex);

                    const FMatrix& LocalToWorld = GetLocalToWorld();
                    const FLinearColor DrawSphereColor = GetViewSelectionColor(SphereColor, *View, IsSelected(), IsHovered(), false, IsIndividuallySelected());

                    // Taking into account the min and maximum drawing distance
                    const float DistanceSqr = (View->ViewMatrices.GetViewOrigin() - LocalToWorld.GetOrigin()).SizeSquared();
                    if (DistanceSqr < FMath::Square(GetMinDrawDistance()) || DistanceSqr > FMath::Square(GetMaxDrawDistance()))
                    {
                        continue;
                    }

                    float AbsScaleX = LocalToWorld.GetScaledAxis(EAxis::X).Size();
                    float AbsScaleY = LocalToWorld.GetScaledAxis(EAxis::Y).Size();
                    float AbsScaleZ = LocalToWorld.GetScaledAxis(EAxis::Z).Size();
                    float MinAbsScale = FMath::Min3(AbsScaleX, AbsScaleY, AbsScaleZ);

                    FVector ScaledX = LocalToWorld.GetUnitAxis(EAxis::X) * MinAbsScale;
                    FVector ScaledY = LocalToWorld.GetUnitAxis(EAxis::Y) * MinAbsScale;
                    FVector ScaledZ = LocalToWorld.GetUnitAxis(EAxis::Z) * MinAbsScale;

                    const int32 SphereSides = FMath::Clamp<int32>(SphereRadius / 4.f, 16, 64);
                    DrawCircle(PDI, LocalToWorld.GetOrigin(), ScaledX, ScaledY, DrawSphereColor, SphereRadius, SphereSides, SDPG_World, LineThickness);
                    DrawCircle(PDI, LocalToWorld.GetOrigin(), ScaledX, ScaledZ, DrawSphereColor, SphereRadius, SphereSides, SDPG_World, LineThickness);
                    DrawCircle(PDI, LocalToWorld.GetOrigin(), ScaledY, ScaledZ, DrawSphereColor, SphereRadius, SphereSides, SDPG_World, LineThickness);
                }
            }
        }

        virtual FPrimitiveViewRelevance GetViewRelevance(const FSceneView* View) const override
        {
            const bool bVisibleForSelection = !bDrawOnlyIfSelected || IsSelected();
            const bool bVisibleForShowFlags = true; // @TODO

            // Should we draw this because collision drawing is enabled, and we have collision
            const bool bShowForCollision = View->Family->EngineShowFlags.Collision && IsCollisionEnabled();

            FPrimitiveViewRelevance Result;
            Result.bDrawRelevance = (IsShown(View) && bVisibleForSelection && bVisibleForShowFlags) || bShowForCollision;
            Result.bDynamicRelevance = true;
            Result.bShadowRelevance = IsShadowCast(View);
            Result.bEditorPrimitiveRelevance = UseEditorCompositing(View);
            return Result;
        }

        virtual uint32 GetMemoryFootprint(void) const override { return(sizeof(*this) + GetAllocatedSize()); }
        uint32 GetAllocatedSize(void) const { return(FPrimitiveSceneProxy::GetAllocatedSize()); }

    private:
        const uint32				bDrawOnlyIfSelected : 1;
        const FColor				SphereColor;
        const float					SphereRadius;
        const float					LineThickness;
    };

    return new FSphereSceneProxy(this);
}

FBoxSphereBounds USphereSensor::CalcBounds(const FTransform& LocalToWorld) const
{
    return FBoxSphereBounds(FVector::ZeroVector, FVector(Radius), Radius).TransformBy(LocalToWorld);
}

void USphereSensor::UpdateBodySetup()
{
    if (PrepareSharedBodySetup<USphereSensor>())
    {
        bUseArchetypeBodySetup = InvalidateOrUpdateSphereSensorSetup<EShapeBodySetupHelper::InvalidateSharingIfStale>(ShapeBodySetup, bUseArchetypeBodySetup, Radius);
    }

    CreateShapeBodySetupIfNeeded<FKSphereElem>();

    if (!bUseArchetypeBodySetup)
    {
        InvalidateOrUpdateSphereSensorSetup<EShapeBodySetupHelper::UpdateBodySetup>(ShapeBodySetup, bUseArchetypeBodySetup, Radius);
    }
}

float USphereSensor::GetRadiusWithMargin() const
{
    return Radius + CollisionMargin();
}

FCollisionShape USphereSensor::GetCollisionShape(float Inflation) const
{
    return FCollisionShape::MakeSphere(FMath::Max(0.f, Radius + Inflation));
}
