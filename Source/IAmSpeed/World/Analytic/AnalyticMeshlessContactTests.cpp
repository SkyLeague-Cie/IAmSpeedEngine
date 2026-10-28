#if WITH_DEV_AUTOMATION_TESTS
#include "SpeedAnalyticContactComponent.h"
#include "SpeedAnalyticSourceComponent.h"
#include "SpeedAnalyticCollisionAsset.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Misc/AutomationTest.h"
#include "UObject/Package.h"
#include "UObject/StrongObjectPtr.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticMeshlessContactTest,
    "IAmSpeed.AnalyticWorld.AuthoredSource.MeshlessContact",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FAnalyticMeshlessContactTest::RunTest(const FString&)
{
    if (!GEngine) { AddError(TEXT("Engine required.")); return false; }
    const auto Options = UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(true)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false)
        .ShouldSimulatePhysics(false).SetTransactional(false);
    UWorld* World = UWorld::CreateWorld(EWorldType::Editor, false, TEXT("MeshlessAuthoredTest"),
        CreatePackage(TEXT("/Temp/MeshlessAuthoredTest")), true, ERHIFeatureLevel::Num, &Options);
    if (!World) { AddError(TEXT("World required.")); return false; }
    GEngine->CreateNewWorldContext(EWorldType::Editor).SetCurrentWorld(World);
    AActor* Owner = World->SpawnActor<AActor>();
    if (!Owner) { World->DestroyWorld(false); GEngine->DestroyWorldContext(World); AddError(TEXT("Owner required.")); return false; }
    TStrongObjectPtr<USpeedAnalyticCollisionAsset> Asset(NewObject<USpeedAnalyticCollisionAsset>());
    // Synthetic finite plane fixture, never a stadium certificate/profile.
    Asset->SourceHash = 41; Asset->AuthoredSourceSchemaVersion = 1;
    FSpeedAnalyticAuthoredSourceRecord Record;
    Record.Kind = ESpeedAnalyticSourceKind::AuthoredAnalytic; Record.SourceId = 17;
    Record.RecipeSchema = TEXT("test.meshless/v1"); Record.RecipeSha256 = FString::ChrN(64, 'a');
    Record.PayloadSha256 = FString::ChrN(64, 'b'); Record.SourceComponentPath = TEXT("Source");
    Record.ContactComponentPath = TEXT("Contact"); Record.ObjectType = ECC_WorldStatic;
    Record.CollisionEnabled = ECollisionEnabled::QueryOnly; Record.bQueryCollisionEnabled = true;
    Record.BlockingChannels = uint64(1) << ECC_PhysicsBody; Asset->AuthoredSources = {Record};
    FSpeedAnalyticBoundedPlaneRecord Plane;
    Plane.SourceId = 17; Plane.SurfaceId = 19; Plane.FeatureId = 23; Plane.PrimitiveId = 29;
    Plane.ObjectType = Record.ObjectType; Plane.BlockingChannels = Record.BlockingChannels;
    Plane.HalfExtents = FVector2D(100, 100); Plane.bAuthorityEligible = true; Asset->BoundedPlanes = {Plane};
    auto* Source = NewObject<USpeedAnalyticSourceComponent>(Owner, TEXT("Source"));
    Owner->AddInstanceComponent(Source); Source->AuthoredSourceId = Record.SourceId;
    Source->AuthoredContactComponentPath = Record.ContactComponentPath; Source->bIncludeOwnedStaticMeshes = false;
    Source->AnalyticCollisionAsset = Asset.Get(); Source->RegisterComponent();
    auto* Contact = NewObject<USpeedAnalyticContactComponent>(Owner, TEXT("Contact"));
    Owner->AddInstanceComponent(Contact); Owner->SetRootComponent(Contact); Contact->RegisterComponent();
    bool Success; FString Reason;
    Contact->ApplyAuthoredSourcePolicy(Success, Reason); TestTrue(TEXT("copies declared policy"), Success);
    Contact->RefreshAuthoredBounds(Success, Reason); TestTrue(TEXT("real finite plane bounds refresh"), Success);
    if (!Success) AddError(Reason);
    TestTrue(TEXT("exact live contact resolves without mesh"), Source->ResolveAuthoredContact(Record, *Asset, &Reason) == Contact);
    TestFalse(TEXT("no Chaos physics state"), Contact->IsPhysicsStateCreated());
    TestNull(TEXT("no BodySetup"), Contact->GetBodySetup());
    TestNull(TEXT("no render proxy"), Contact->CreateSceneProxy());
    TestTrue(TEXT("finite plane X extent"), FMath::IsNearlyEqual(Contact->Bounds.BoxExtent.X, 100.0));
    TestTrue(TEXT("finite plane Y extent"), FMath::IsNearlyEqual(Contact->Bounds.BoxExtent.Y, 100.0));
    TArray<UStaticMeshComponent*> Meshes; Owner->GetComponents(Meshes); TestTrue(TEXT("no mesh component"), Meshes.IsEmpty());
    Contact->SetCollisionResponseToChannel(ECC_PhysicsBody, ECR_Ignore);
    Contact->RefreshAuthoredBounds(Success, Reason); TestFalse(TEXT("policy drift refused"), Success);
    TestTrue(TEXT("failed refresh clears stale bounds"), Contact->Bounds.BoxExtent.IsNearlyZero());
    Contact->ApplyAuthoredSourcePolicy(Success, Reason); Contact->RefreshAuthoredBounds(Success, Reason);
    TestTrue(TEXT("exact policy restoration permits binding"), Success);
    Source->AuthoredContactComponentPath = TEXT("Missing");
    Contact->RefreshAuthoredBounds(Success, Reason); TestFalse(TEXT("wrong relative path refused"), Success);
    Source->AuthoredContactComponentPath = Record.ContactComponentPath;
    Asset->BoundedPlanes[0].HalfExtents = FVector2D(175, 120);
    Contact->RefreshAuthoredBounds(Success, Reason); TestTrue(TEXT("changed source geometry refreshes"), Success);
    TestTrue(TEXT("bounds follow current asset, no manual fixture dimensions in component"),
        FMath::IsNearlyEqual(Contact->Bounds.BoxExtent.X, 175.0) && FMath::IsNearlyEqual(Contact->Bounds.BoxExtent.Y, 120.0));
    World->DestroyWorld(false); GEngine->DestroyWorldContext(World);
    return !HasAnyErrors();
}
#endif
