#include "SpeedAnalyticContactComponent.h"

#include "SpeedAnalyticSourceComponent.h"
#include "SpeedAnalyticCollisionAsset.h"
#include "GameFramework/Actor.h"

namespace
{
    const FSpeedAnalyticAuthoredSourceRecord* ResolveDeclaration(
        USpeedAnalyticContactComponent& Contact, USpeedAnalyticSourceComponent*& Source,
        USpeedAnalyticCollisionAsset*& Asset, FString& Reason)
    {
        Source = nullptr; Asset = nullptr;
        if (!IsInGameThread()) { Reason = TEXT("Game thread required."); return nullptr; }
        AActor* Owner = Contact.GetOwner();
        if (!Owner) { Reason = TEXT("Owned analytic contact on game thread required."); return nullptr; }
        TArray<USpeedAnalyticSourceComponent*> Sources;
        Owner->GetComponents(Sources);
        if (Sources.Num() != 1 || !Sources[0] || Sources[0]->AuthoredSourceId == 0 ||
            Sources[0]->bIncludeOwnedStaticMeshes)
        { Reason = TEXT("Expected one explicit authored source, without mesh discovery."); return nullptr; }
        Source = Sources[0];
        Asset = Source->AnalyticCollisionAsset.LoadSynchronous();
        if (!Asset) { Reason = TEXT("The declaring source asset is unavailable."); return nullptr; }
        const FSpeedAnalyticAuthoredSourceRecord* Record = nullptr;
        for (const auto& Candidate : Asset->AuthoredSources)
            if (Candidate.SourceId == Source->AuthoredSourceId)
            {
                if (Record) { Reason = TEXT("Duplicate authored source record."); return nullptr; }
                Record = &Candidate;
            }
        if (!Record) { Reason = TEXT("Authored source record missing."); return nullptr; }
        if (!Record->IsValid(&Reason)) return nullptr;
        return Record;
    }
}

USpeedAnalyticContactComponent::USpeedAnalyticContactComponent()
{
    PrimaryComponentTick.bCanEverTick = false;
    SetMobility(EComponentMobility::Static);
    SetCollisionEnabled(ECollisionEnabled::NoCollision);
    SetGenerateOverlapEvents(false);
    SetCanEverAffectNavigation(false);
}

void USpeedAnalyticContactComponent::ApplyAuthoredSourcePolicy(bool& OutSuccess, FString& OutReason)
{
    OutSuccess = false; OutReason.Reset();
    USpeedAnalyticSourceComponent* Source; USpeedAnalyticCollisionAsset* Asset;
    const auto* Record = ResolveDeclaration(*this, Source, Asset, OutReason);
    if (!Record) return;
    SetCollisionObjectType(static_cast<ECollisionChannel>(Record->ObjectType));
    SetCollisionEnabled(static_cast<ECollisionEnabled::Type>(Record->CollisionEnabled));
    for (uint32 Channel = 0; Channel < UE_ARRAY_COUNT(FCollisionResponseContainer::EnumArray); ++Channel)
    {
        const uint64 Bit = uint64(1) << Channel;
        SetCollisionResponseToChannel(static_cast<ECollisionChannel>(Channel),
            Record->BlockingChannels & Bit ? ECR_Block : Record->OverlapChannels & Bit ? ECR_Overlap : ECR_Ignore);
    }
    OutSuccess = true;
}

void USpeedAnalyticContactComponent::RefreshAuthoredBounds(bool& OutSuccess, FString& OutReason)
{
    OutSuccess = false; OutReason.Reset();
    if (!IsInGameThread()) { OutReason = TEXT("Game thread required."); return; }
    LocalAnalyticBounds = FBox(EForceInit::ForceInit);
    USpeedAnalyticSourceComponent* Source; USpeedAnalyticCollisionAsset* Asset;
    const auto* Record = ResolveDeclaration(*this, Source, Asset, OutReason);
    if (!Record || Source->ResolveAuthoredContact(*Record, *Asset, &OutReason) != this)
    { UpdateBounds(); return; }
    if (!Asset->ValidateGeneratedData(&OutReason, false)) { UpdateBounds(); return; }
    FBox Candidate(EForceInit::ForceInit); bool Finite = true;
    auto Point = [&](const FVector& World)
    {
        const FVector Local = Record->ContactWorldTransform.InverseTransformPosition(World);
        if (World.ContainsNaN() || Local.ContainsNaN()) Finite = false;
        else Candidate += Local;
    };
    for (const auto& Plane : Asset->BoundedPlanes) if (Plane.SourceId == Record->SourceId)
    {
        if (!Plane.DomainVertices.IsEmpty())
            for (const auto& UV : Plane.DomainVertices) Point(Plane.Origin + Plane.AxisU * UV.X + Plane.AxisV * UV.Y);
        else for (int32 U : {-1, 1}) for (int32 V : {-1, 1})
            Point(Plane.Origin + Plane.AxisU * (U * Plane.HalfExtents.X) + Plane.AxisV * (V * Plane.HalfExtents.Y));
    }
    for (const auto& Patch : Asset->TensorBezierPatches) if (Patch.SourceId == Record->SourceId)
        for (const FVector& CP : Patch.ControlPoints) Point(CP);
    for (const auto& Patch : Asset->PiecewiseTensorBezierPatches) if (Patch.SourceId == Record->SourceId)
        for (const auto& Cell : Patch.Cells) for (const FVector& CP : Cell.ControlPoints) Point(CP);
    for (const auto& Patch : Asset->ExtrudedQuinticPatches) if (Patch.SourceId == Record->SourceId && Patch.Bounds.IsValid)
        for (int32 X : {0, 1}) for (int32 Y : {0, 1}) for (int32 Z : {0, 1})
            Point(FVector(X ? Patch.Bounds.Max.X : Patch.Bounds.Min.X,
                Y ? Patch.Bounds.Max.Y : Patch.Bounds.Min.Y, Z ? Patch.Bounds.Max.Z : Patch.Bounds.Min.Z));
    if (!Finite || !Candidate.IsValid) { OutReason = TEXT("Finite authored geometry bounds required."); UpdateBounds(); return; }
    LocalAnalyticBounds = Candidate;
    UpdateBounds(); MarkRenderTransformDirty();
    OutSuccess = true;
}

void USpeedAnalyticContactComponent::OnRegister()
{
    Super::OnRegister();
    bool Ready; FString Reason;
    RefreshAuthoredBounds(Ready, Reason);
}

FBoxSphereBounds USpeedAnalyticContactComponent::CalcBounds(const FTransform& LocalToWorld) const
{
    return LocalAnalyticBounds.IsValid
        ? FBoxSphereBounds(LocalAnalyticBounds).TransformBy(LocalToWorld)
        : FBoxSphereBounds(LocalToWorld.GetLocation(), FVector::ZeroVector, 0.0);
}
