#if WITH_DEV_AUTOMATION_TESTS
#include "AnalyticLandscapeAdapter.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "HAL/IConsoleManager.h"
#include "Landscape.h"
#include "LandscapeHeightfieldCollisionComponent.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include "UObject/Package.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedLandscapeSourceIdentityTest,
	"IAmSpeed.AnalyticWorld.LandscapeSourceIdentity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedLandscapeSourceIdentityTest::RunTest(const FString& Parameters)
{
	using namespace Speed::Analytic;
	IConsoleVariable* Backend = IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.StaticCollision.Backend"));
	if (!TestNotNull(TEXT("static collision backend exists"), Backend)) return false;
	const int32 PreviousBackend = Backend->GetInt();
	Backend->Set(2, ECVF_SetByCode);
	ON_SCOPE_EXIT { Backend->Set(PreviousBackend, ECVF_SetByCode); };
	const uint64 ExpectedSource = StableStringId(TEXT("/Temp/IAmSpeedLandscapeIdentity.IAmSpeedLandscapeIdentity:PersistentLevel.Landscape_0"));
	uint64 ReferencePrimitive = 0;
	for (const int32 Instance : { -1, 0, 7 })
	{
		// Only the package is duplicated in PIE; actor/world names remain authored.
		// Exercise the UObject adapter and real subsystem registration, not a
		// second hand-written normalization formula in an isolated hash test.
		const FString PackageName = Instance == INDEX_NONE ? TEXT("/Temp/IAmSpeedLandscapeIdentity") :
			FString::Printf(TEXT("/Temp/UEDPIE_%d_IAmSpeedLandscapeIdentity"), Instance);
		const auto Options = UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(true)
			.RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false)
			.ShouldSimulatePhysics(false).SetTransactional(false);
		UWorld* World = UWorld::CreateWorld(EWorldType::Game, false, TEXT("IAmSpeedLandscapeIdentity"),
			CreatePackage(*PackageName), true, ERHIFeatureLevel::Num, &Options);
		if (!TestNotNull(TEXT("Landscape identity world exists"), World)) return false;
		GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
		ON_SCOPE_EXIT { World->DestroyWorld(false); GEngine->DestroyWorldContext(World); };
		FActorSpawnParameters Spawn;
		Spawn.Name = TEXT("Landscape_0");
		ALandscape* Landscape = World->SpawnActor<ALandscape>(Spawn);
		if (!TestNotNull(TEXT("Landscape actor exists"), Landscape)) return false;
		auto* Collision = NewObject<ULandscapeHeightfieldCollisionComponent>(Landscape);
		Landscape->AddInstanceComponent(Collision);
		Landscape->CollisionComponents.Add(Collision);
		Collision->CachedLocalBox = FBox(FVector(-200, -200, -1), FVector(200, 200, 1));
		Collision->SetWorldLocation(FVector(0, 0, 42));
		Collision->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
		Collision->SetCollisionObjectType(ECC_WorldStatic);
		Collision->SetCollisionResponseToAllChannels(ECR_Block);
		const auto Output = BuildFlatLandscapePlane(*Landscape, 0);
		TestEqual(TEXT("flat collision metadata produces an eligible provider"), Output.Result,
			EFlatLandscapeAdapterResult::SuccessAuthorityEligible);
		TestEqual(TEXT("PIE import retains the authored Landscape source id"), Output.Plane.SourceId, ExpectedSource);
		TestEqual(TEXT("Landscape provider retains the actual support height"), Output.Plane.Origin.Z, 42.0);
		if (Instance == INDEX_NONE) ReferencePrimitive = Output.Plane.PrimitiveId;
		else TestEqual(TEXT("PIE instance does not change primitive identity"), Output.Plane.PrimitiveId, ReferencePrimitive);
		USpeedWorldSubsystem* Bridge = World->GetSubsystem<USpeedWorldSubsystem>();
		if (!TestNotNull(TEXT("runtime bridge exists"), Bridge)) return false;
		Bridge->OnWorldBeginPlay(*World);
		TestTrue(TEXT("the imported source resolves its support component in every PIE instance"),
			Bridge->FindAnalyticSourceComponent(Output.Plane.SourceId).Get() == Collision);
		const auto* Data = Bridge->GetAnalyticWorldData();
		if (!TestNotNull(TEXT("runtime analytical world was built"), Data)) return false;
		TestEqual(TEXT("runtime world includes the Landscape plane"), Data->Planes.Num(), 1);
		if (Data->Planes.Num() == 1)
			TestTrue(TEXT("the actually loaded provider also resolves its source component"),
				Bridge->FindAnalyticSourceComponent(Data->Planes[0].SourceId).Get() == Collision);
	}
	return true;
}
#endif
