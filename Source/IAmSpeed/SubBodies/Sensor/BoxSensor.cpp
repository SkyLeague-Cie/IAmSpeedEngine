// Fill out your copyright notice in the Description page of Project Settings.


#include "BoxSensor.h"
#include "PhysicsEngine/BodySetup.h"

UBoxSensor::UBoxSensor(const FObjectInitializer& ObjectInitializer) :
    Super(ObjectInitializer)
#if WITH_EDITOR
    , ShowFlags(ESFIM_All0)
#endif // WITH_EDITOR
{
    SubBodyType = ESubBodyType::Hitbox;
    bUseEditorCompositing = true;
    BoxExtent = FVector(32.f, 32.f, 32.f);
}

void UBoxSensor::Initialize(ISpeedComponent* InParentComponent)
{
    Super::Initialize(InParentComponent);
    SubBodyType = ESubBodyType::Hitbox; // ou Unknown
}

bool UBoxSensor::SweepTOI(const float& RemainingDelta, float& OutTOI)
{
    if (!ParentComponent)
    {
        return false;
    }

    float TOI_ground = RemainingDelta;
    float TOI_Sphere = RemainingDelta;
    float TOI_Box = RemainingDelta;
    float TOI_Wheel = RemainingDelta;
    FutureHit = SHitResult();
    SHitResult OldGroundHit = GroundHit;

    SHitResult GroundHitresult;
    SHitResult BoxHitresult;
    SHitResult SphereHitresult;
    SHitResult WheelHitresult;
    bool bHitGround = SweepVsGround(GetWorld(), GroundHitresult, RemainingDelta, TOI_ground);
    bool bHitBox = SweepVsBoxes(GetWorld(), BoxHitresult, RemainingDelta, TOI_Box);
    bool bHitSphere = SweepVsSpheres(GetWorld(), SphereHitresult, RemainingDelta, TOI_Sphere);
    bool bHitWheel = SweepVsWheels(GetWorld(), WheelHitresult, RemainingDelta, TOI_Wheel);
    if (!bHitGround && !bHitBox && !bHitSphere && !bHitWheel)
    {
        return false;
    }

    OutTOI = RemainingDelta;

    if (bHitGround && TOI_ground < OutTOI)
    {
        OutTOI = TOI_ground;
        GroundHit = GroundHitresult;
        FutureHit = GroundHit;
    }

    if (bHitSphere && TOI_Sphere < OutTOI)
    {
        OutTOI = TOI_Sphere;
        SphereHit = SphereHitresult;
        FutureHit = SphereHit;
    }

    if (bHitBox && TOI_Box < OutTOI)
    {
        OutTOI = TOI_Box;
        BoxHit = BoxHitresult;
        FutureHit = BoxHit;
    }

    if (bHitWheel && TOI_Wheel < OutTOI)
    {
        OutTOI = TOI_Wheel;
        WheelHit = WheelHitresult;
        FutureHit = WheelHit;
    }

    return true;
}

FPrimitiveSceneProxy* UBoxSensor::CreateSceneProxy()
{
    /** Represents a UBoxSubBody to the scene manager. */
    class FBoxSceneProxy final : public FPrimitiveSceneProxy
    {
    public:
        SIZE_T GetTypeHash() const override
        {
            static size_t UniquePointer;
            return reinterpret_cast<size_t>(&UniquePointer);
        }

        FBoxSceneProxy(const UBoxSensor* InComponent)
            : FPrimitiveSceneProxy(InComponent)
            , bDrawOnlyIfSelected(InComponent->bDrawOnlyIfSelected)
            , BoxExtents(InComponent->BoxExtent)
            , BoxColor(InComponent->ShapeColor)
            , LineThickness(InComponent->LineThickness)
        {
            bWillEverBeLit = false;

#if WITH_EDITOR
            struct FIterSink
            {
                FIterSink(const FEngineShowFlags InSelectedShowFlags)
                    : SelectedShowFlags(InSelectedShowFlags)
                {
                    SelectedShowFlagIndices.SetNum(FEngineShowFlags::SF_FirstCustom, false);
                }

                bool HandleShowFlag(uint32 InIndex, const FString& InName)
                {
                    if (SelectedShowFlags.GetSingleFlag(InIndex) == true)
                    {
                        SelectedShowFlagIndices.PadToNum(InIndex + 1, false);
                        SelectedShowFlagIndices[InIndex] = true;
                    }

                    return true;
                }

                bool OnEngineShowFlag(uint32 InIndex, const FString& InName)
                {
                    return HandleShowFlag(InIndex, InName);
                }

                bool OnCustomShowFlag(uint32 InIndex, const FString& InName)
                {
                    return HandleShowFlag(InIndex, InName);
                }

                const FEngineShowFlags SelectedShowFlags;

                TBitArray<> SelectedShowFlagIndices;
            };

            FIterSink Sink(InComponent->ShowFlags);
            FEngineShowFlags::IterateAllFlags(Sink);
            SelectedShowFlagIndices = MoveTemp(Sink.SelectedShowFlagIndices);
#endif // WITH_EDITOR
        }

        virtual void GetDynamicMeshElements(const TArray<const FSceneView*>& Views, const FSceneViewFamily& ViewFamily, uint32 VisibilityMap, FMeshElementCollector& Collector) const override
        {
            QUICK_SCOPE_CYCLE_COUNTER(STAT_BoxSceneProxy_GetDynamicMeshElements);

            const FMatrix& LocalToWorld = GetLocalToWorld();

            for (int32 ViewIndex = 0; ViewIndex < Views.Num(); ViewIndex++)
            {
                if (VisibilityMap & (1 << ViewIndex))
                {
                    const FSceneView* View = Views[ViewIndex];

                    const FLinearColor DrawColor = GetViewSelectionColor(BoxColor, *View, IsSelected(), IsHovered(), false, IsIndividuallySelected());

                    FPrimitiveDrawInterface* PDI = Collector.GetPDI(ViewIndex);
                    DrawOrientedWireBox(PDI, LocalToWorld.GetOrigin(), LocalToWorld.GetScaledAxis(EAxis::X), LocalToWorld.GetScaledAxis(EAxis::Y), LocalToWorld.GetScaledAxis(EAxis::Z), BoxExtents, DrawColor, SDPG_World, LineThickness);
                }
            }
        }

        virtual FPrimitiveViewRelevance GetViewRelevance(const FSceneView* View) const override
        {
            const bool bProxyVisible = !bDrawOnlyIfSelected || IsSelected();

            // Should we draw this because collision drawing is enabled, and we have collision
            const bool bShowForCollision = View->Family->EngineShowFlags.Collision && IsCollisionEnabled();

            FPrimitiveViewRelevance Result;
            Result.bDrawRelevance = (IsShown(View) && bProxyVisible) || bShowForCollision;
#if WITH_EDITOR
            bool bAreAllSelectedFlagsEnabled = true;
            for (TConstSetBitIterator<> It(SelectedShowFlagIndices); It; ++It)
            {
                bAreAllSelectedFlagsEnabled &= View->Family->EngineShowFlags.GetSingleFlag(It.GetIndex());
            }

            Result.bDrawRelevance &= bAreAllSelectedFlagsEnabled;
#endif // WITH_EDITOR
            Result.bDynamicRelevance = true;
            Result.bShadowRelevance = IsShadowCast(View);
            Result.bEditorPrimitiveRelevance = UseEditorCompositing(View);
            return Result;
        }
        virtual uint32 GetMemoryFootprint(void) const override { return(sizeof(*this) + GetAllocatedSize()); }
        uint32 GetAllocatedSize(void) const { return(FPrimitiveSceneProxy::GetAllocatedSize()); }

    private:
        const uint32	bDrawOnlyIfSelected : 1;
        const FVector	BoxExtents;
        const FColor	BoxColor;
        const float		LineThickness;
#if WITH_EDITOR
        TBitArray<>		SelectedShowFlagIndices;
#endif // WITH_EDITOR
    };

    return new FBoxSceneProxy(this);
}

FBoxSphereBounds UBoxSensor::CalcBounds(const FTransform& LocalToWorld) const
{
    return FBoxSphereBounds(FBox(-BoxExtent, BoxExtent)).TransformBy(LocalToWorld);
}

template <EShapeBodySetupHelper UpdateBodySetupAction, typename BodySetupType>
bool InvalidateOrUpdateBoxSensorSetup(BodySetupType& ShapeBodySetup, bool bUseArchetypeBodySetup, FVector BoxExtent)
{
    check((bUseArchetypeBodySetup && UpdateBodySetupAction == EShapeBodySetupHelper::InvalidateSharingIfStale) || (!bUseArchetypeBodySetup && UpdateBodySetupAction == EShapeBodySetupHelper::UpdateBodySetup));
    check(ShapeBodySetup->AggGeom.BoxElems.Num() == 1);
    FKBoxElem* se = ShapeBodySetup->AggGeom.BoxElems.GetData();

    // @todo do we allow this now?
    // check for malformed values
    if (BoxExtent.X < UE_KINDA_SMALL_NUMBER)
    {
        BoxExtent.X = 1.0f;
    }

    if (BoxExtent.Y < UE_KINDA_SMALL_NUMBER)
    {
        BoxExtent.Y = 1.0f;
    }

    if (BoxExtent.Z < UE_KINDA_SMALL_NUMBER)
    {
        BoxExtent.Z = 1.0f;
    }

    float XExtent = BoxExtent.X * 2.f;
    float YExtent = BoxExtent.Y * 2.f;
    float ZExtent = BoxExtent.Z * 2.f;

    if (UpdateBodySetupAction == EShapeBodySetupHelper::UpdateBodySetup)
    {
        // now set the PhysX data values
        se->SetTransform(FTransform::Identity);
        se->X = XExtent;
        se->Y = YExtent;
        se->Z = ZExtent;
    }
    else if (se->X != XExtent || se->Y != YExtent || se->Z != ZExtent)
    {
        ShapeBodySetup = nullptr;
        bUseArchetypeBodySetup = false;
    }

    return bUseArchetypeBodySetup;
}

void UBoxSensor::UpdateBodySetup()
{
    if (PrepareSharedBodySetup<UBoxSensor>())
    {
        bUseArchetypeBodySetup = InvalidateOrUpdateBoxSensorSetup<EShapeBodySetupHelper::InvalidateSharingIfStale>(ShapeBodySetup, bUseArchetypeBodySetup, BoxExtent);
    }

    CreateShapeBodySetupIfNeeded<FKBoxElem>();

    if (!bUseArchetypeBodySetup)
    {
        InvalidateOrUpdateBoxSensorSetup<EShapeBodySetupHelper::UpdateBodySetup>(ShapeBodySetup, bUseArchetypeBodySetup, BoxExtent);
    }
}

FCollisionShape UBoxSensor::GetCollisionShape(float Inflation) const
{
    return FCollisionShape::MakeBox(BoxExtent + FVector(Inflation));
}

#if WITH_EDITOR
void UBoxSensor::SetShowFlags(const FEngineShowFlags& InShowFlags)
{
    ShowFlags = InShowFlags;
    MarkRenderStateDirty();
}
#endif // WITH_EDITOR
