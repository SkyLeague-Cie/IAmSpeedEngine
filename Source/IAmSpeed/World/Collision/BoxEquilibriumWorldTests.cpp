#if WITH_DEV_AUTOMATION_TESTS
#include "BoxRestingSupport.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "IAmSpeed/SubBodies/Configs/SubBodyConfig.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "IAmSpeed/World/Simulation/CanonicalFrameContext.h"
#include "IAmSpeed/World/Simulation/CcdPoseAudit.h"
#include "IAmSpeed/World/Analytic/StaticWorldQueryAudit.h"
#include "Components/BoxComponent.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "GameFramework/Actor.h"
#include "HAL/IConsoleManager.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Math/QuatRotationTranslationMatrix.h"

namespace
{
struct FBoxLawGeometry
{
	FVector Extent = FVector(60, 40, 20);
	FVector Offset = FVector::ZeroVector;
	FVector COM = FVector::ZeroVector;
	FQuat LocalRotation = FQuat::Identity;
	float Restitution = 0;
	float Mass = 10, Friction = .5f, ImpactThreshold = 5;
	float MaxAngularSpeed = 100;
};
// The adapter owns only rigid-body state and material/geometry. Integration,
// CCD, contact impulses and frame ordering remain the production engine's.
class FBoxLawBody final : public ISpeedComponent
{
public:
	SKinematic State;
	SubBodyConfig Config;
	FVector LocalCOM = FVector::ZeroVector;
	float MaxAngularSpeed = 100;
	UBoxSubBody* Box = nullptr;
	float MaxLinearSpeed = 10000;
	TArray<USSubBody*> Bodies;
	uint32 Frame = 0;
	bool bFrozen = false;
	bool bLateFeasibilityPass = false;
	double FloorZ = 0;
	double MinimumClearance = DBL_MAX;
	uint32 IntegrationSamples = 0;
	uint32 PassiveProjectionFailures = 0;
	FQuat PreviousRotation = FQuat::Identity;

	// Keep only the first contact interval; observers never modify body state.
	void Trace(const TCHAR* Phase)
	{
		if (Frame >= 2999)
		{
			UE_LOG(LogTemp, Display, TEXT("[BoxLawLate] phase=%s frame=%u gap=%.17g z=%.17g q=%s v=%s w=%s hit_toi=%.17g index=%d"),
				Phase, Frame, Clearance(), double(State.Location.Z), *State.Rotation.ToString(), *State.Velocity.ToString(),
				*State.AngularVelocity.ToString(), double(Box->GetHit().TOI), int32(Box->GetHit().ContactFeatureIndexThis));
		}
		if (Clearance() < 3 && TraceSamples < 32)
		{
			++TraceSamples;
			UE_LOG(LogTemp, Display, TEXT("[BoxLawTrace] phase=%s frame=%u gap=%.17g pose=%s v=%s w=%s"),
				Phase, Frame, Clearance(), *State.Location.ToString(), *State.Velocity.ToString(), *State.AngularVelocity.ToString());
		}
	}
	int32 TraceSamples = 0;
	bool bReportedNegative = false, bReportedNegativeBeyondRoundoff = false;
	void ObserveClearance(const TCHAR* Phase)
	{
		const double Gap = Clearance();
		MinimumClearance = FMath::Min(MinimumClearance, Gap);
		const double Roundoff = 64 * DBL_EPSILON * FMath::Max(1.0, State.Location.GetAbsMax() + Config.BoxExtent.GetMax());
		const bool bFirstSign = Gap < 0 && !bReportedNegative;
		const bool bFirstBeyondRoundoff = Gap < -Roundoff && !bReportedNegativeBeyondRoundoff;
		if (bFirstSign || bFirstBeyondRoundoff)
		{
			const SHitResult& Hit = Box->GetHit();
			UE_LOG(LogTemp, Display, TEXT("[BoxLawFirstNegative] phase=%s frame=%u gap=%.17g beyond_roundoff=%d z=%.17g q=%s v=%s w=%s hit_toi=%.17g feature=%d index=%d"),
				Phase, Frame, Gap, bFirstBeyondRoundoff ? 1 : 0, double(State.Location.Z), *State.Rotation.ToString(),
				*State.Velocity.ToString(), *State.AngularVelocity.ToString(), double(Hit.TOI), int32(Hit.ContactFeatureThis), int32(Hit.ContactFeatureIndexThis));
			UE_LOG(LogTemp, Display, TEXT("[BoxLawContactIdentity] other_feature=%d normal=(%.17g,%.17g,%.17g) point=(%.17g,%.17g,%.17g) source=%llu primitive=%llu variable=%d error=%.17g contact_precision=%d"),
				int32(Hit.ContactFeatureOther), double(Hit.ImpactNormal.X), double(Hit.ImpactNormal.Y), double(Hit.ImpactNormal.Z),
				double(Hit.ImpactPoint.X), double(Hit.ImpactPoint.Y), double(Hit.ImpactPoint.Z), Hit.SourceId, Hit.PrimitiveId,
				Hit.bSurfaceNormalMayVary ? 1 : 0, double(Hit.GeometricErrorBoundCm), Box->RequiresUnquantizedContactPose() ? 1 : 0);
		}
		bReportedNegative |= Gap < 0;
		bReportedNegativeBeyondRoundoff |= Gap < -Roundoff;
	}

	explicit FBoxLawBody(AActor& Actor, float Friction = 0.5f, const FBoxLawGeometry& Geometry = {})
	{
		Config.bValid = true;
		Config.bIsMainSubBody = true;
		Config.Mass = Geometry.Mass;
		Config.BoxExtent = Geometry.Extent;
		Config.LocalOffset = Geometry.Offset;
		Config.LocalRotation = Geometry.LocalRotation;
		LocalCOM = Geometry.COM;
		MaxAngularSpeed = Geometry.MaxAngularSpeed;
		Config.StaticFriction = Friction;
		Config.DynamicFriction = Friction;
		Config.Restitution = Geometry.Restitution;
		Config.ImpactThreshold = Geometry.ImpactThreshold;
		Box = NewObject<UBoxSubBody>(&Actor);
		Bodies.Add(Box);
		Box->Initialize(this);
		Actor.AddInstanceComponent(Box);
	}
	void PrepareCanonicalFrame(const FCanonicalFrameContext& Context) override
	{
		Frame = static_cast<uint32>(Context.NumFrame) + 1;
		State.Acceleration = GetNominalGravityAcceleration();
		State.AngularAcceleration = FVector::ZeroVector;
	}
	unsigned int NumFrame() const override { return Frame; }
	float GetPhysMass() const override { return Config.Mass; }
	const FVector& GetPhysCOM() const override { return State.Location; }
	const FVector& GetPhysCenterOfMassLocal() const override { return LocalCOM; }
	const TArray<USSubBody*>& GetSubBodies() const override { return Bodies; }
	TArray<USSubBody*> CreateSubBodies() override { return Bodies; }
	SubBodyConfig GetSubBodyConfig(const USSubBody&) const override { return Config; }
	bool IsFrozen() const override { return bFrozen; }
	void SetIsFrozen(bool Value) override { bFrozen = Value; }
	SKinematic GetKinematicsOfSubBody(const USSubBody&, const unsigned int& AtFrame) const override
	{
		return Box->GetKinematicsFromOwnerKS(GetOriginKinematicStateForFrame(AtFrame));
	}
	const SKinematic& GetKinematicState() const override { return State; }
	const SKinematic& GetKinematicStateForFrame(const unsigned int&) const override { return State; }
	void SetKinematicState(const SKinematic& Value) override { State = Value; }
	float GetPhysMaxSpeed() const override { return MaxLinearSpeed; }
	float GetPhysMaxAngularSpeed() const override { return MaxAngularSpeed; }
	FVector GetNominalGravityAcceleration() const override { return FVector(0, 0, -650); }
	void SetIsUpsideDown(bool) override {}
	bool IsUpsideDown() const override { return false; }
	bool IsInAutoRecover() const override { return false; }
	bool IsSubBodyInAutoRecoverMode() const override { return false; }
	void RcvImpactOnSubBody(const USSubBody&, const FVector&) override { Trace(TEXT("resolved")); }
	FMatrix ComputeWorldInvInertiaTensor() const override
	{
		if (!LocalCOM.IsZero() || !Config.LocalOffset.IsZero() || Config.LocalRotation != FQuat::Identity)
			return Box->ComputeWorldInvInertiaTensor();
		// Uniform box about its central COM; no game-specific inertia or helper torque.
		const FVector E = Config.BoxExtent;
		FMatrix Inverse = FMatrix::Identity;
		Inverse.M[0][0] = 3.0 / (Config.Mass * (E.Y * E.Y + E.Z * E.Z));
		Inverse.M[1][1] = 3.0 / (Config.Mass * (E.X * E.X + E.Z * E.Z));
		Inverse.M[2][2] = 3.0 / (Config.Mass * (E.X * E.X + E.Y * E.Y));
		const FMatrix R = FQuatRotationMatrix(State.Rotation);
		return R.GetTransposed() * Inverse * R;
	}
	FMatrix ComputeWorldInvInertiaTensorOfSubBody(const USSubBody&) const override { return ComputeWorldInvInertiaTensor(); }
	double Clearance() const
	{
		double Gap = DBL_MAX;
		// Reconstruct the actual origin -> local-box transform in the same order
		// as the component scene geometry; independent corner tests then compare
		// it to the plane, not to the provider's dot-radius overlap result.
		const FVector Origin = State.Location - State.Rotation.RotateVector(LocalCOM);
		const FVector Center = Origin + State.Rotation.RotateVector(Config.LocalOffset);
		const FQuat Rotation = State.Rotation * Config.LocalRotation;
		for (int32 Corner = 0; Corner < 8; ++Corner)
		{
			const FVector E = Config.BoxExtent;
			const FVector P((Corner & 1) ? E.X : -E.X, (Corner & 2) ? E.Y : -E.Y, (Corner & 4) ? E.Z : -E.Z);
			Gap = FMath::Min(Gap, (Center + Rotation.RotateVector(P)).Z - FloorZ);
		}
		return Gap;
	}
	void PostIntegrateKinematics(const float&) override
	{
		++IntegrationSamples;
		ObserveClearance(TEXT("integrated"));
		Trace(TEXT("integrated"));
	}
	void PostPhysicsUpdatePrv(const float& Delta) override
	{
		if (HasExactStaticFaceSupport())
		{
			// Mirrors an owner's later feasibility pass. A nonpenetrating pose
			// must keep its independently published passive contact, unchanged.
			const SKinematic Before = State;
			const bool bWasPublished = Box->HasPhysicsTickGroundContact();
			Box->ProjectEstablishedStaticContact(0);
			if (!bWasPublished || !Box->HasPhysicsTickGroundContact() ||
				State.Location != Before.Location || State.Rotation != Before.Rotation ||
				State.Velocity != Before.Velocity || State.AngularVelocity != Before.AngularVelocity)
				++PassiveProjectionFailures;
		}
		else if (bLateFeasibilityPass)
		{
			// Same late feasibility boundary as a composed/wheeled owner, also
			// during partial support. It must not destroy acquisition history.
			Box->ProjectEstablishedStaticContact(Delta);
		}
		QuantizeKinematicState(State, PreviousRotation);
		PreviousRotation = State.Rotation;
		ObserveClearance(TEXT("quantized"));
		Trace(TEXT("quantized"));
	}
};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedBoxEquilibriumWorldTest,
	"IAmSpeed.PhysicalLaws.BoxEquilibrium.CanonicalDrops",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedBoxEquilibriumWorldTest::RunTest(const FString& Parameters)
{
	FString SelectedCell;
	FParse::Value(FCommandLine::Get(), TEXT("BoxEquilibriumCell="), SelectedCell);
	if (!SelectedCell.IsEmpty()) AddInfo(TEXT("Focused diagnostic only; not the full canonical matrix: ") + SelectedCell);
	IConsoleVariable* Backend = IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.StaticCollision.Backend"));
	IConsoleVariable* Rest = IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.Collision.ExactBoxRestingSupport"));
	IConsoleVariable* ContactDebug = IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.AutoRecoverContactDebug"));
	const int32 OldDebug = ContactDebug ? ContactDebug->GetInt() : 0;
	const int32 OldBackend = Backend->GetInt(), OldRest = Rest->GetInt();
	Backend->Set(2, ECVF_SetByCode);
	Rest->Set(1, ECVF_SetByCode);
	ON_SCOPE_EXIT { Backend->Set(OldBackend, ECVF_SetByCode); Rest->Set(OldRest, ECVF_SetByCode);
		if (ContactDebug) ContactDebug->Set(OldDebug, ECVF_SetByCode); };

	const UWorld::InitializationValues WorldOptions = UWorld::InitializationValues().AllowAudioPlayback(false)
		.CreatePhysicsScene(true).RequiresHitProxies(false).CreateNavigation(false)
		.CreateAISystem(false).ShouldSimulatePhysics(false).SetTransactional(false);
	UWorld* World = UWorld::CreateWorld(EWorldType::Game, false, NAME_None, nullptr,
		true, ERHIFeatureLevel::Num, &WorldOptions);
	if (!TestNotNull(TEXT("canonical test world can be created"), World)) return false;
	GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
	ON_SCOPE_EXIT { World->DestroyWorld(false); GEngine->DestroyWorldContext(World); };
	USpeedWorldSubsystem* Bridge = World->GetSubsystem<USpeedWorldSubsystem>();
	if (!TestNotNull(TEXT("canonical test world owns its runtime bridge"), Bridge)) return false;

	auto Data = MakeShared<Speed::Analytic::FAnalyticWorldData>();
	Speed::Analytic::FBoundedPlane Plane;
	Plane.SourceId = 1; Plane.SurfaceId = 2; Plane.FeatureId = 3; Plane.PrimitiveId = 4;
	Plane.HalfExtents = FVector2d(10000, 10000);
	Plane.BlockingChannels = MAX_uint64;
	Plane.bAuthorityEligible = true;
	Data->Planes.Add(Plane);
	if (!TestTrue(TEXT("finite test plane validates"), Data->FinalizeAndValidate())) return false;
	Bridge->AnalyticWorldData = Data;
	Bridge->SimulationWorld.SetStaticCollisionWorld(MakeUnique<Speed::FAnalyticStaticCollisionWorld>(*Data));
	AActor* FloorActor = World->SpawnActor<AActor>();
	UBoxComponent* Floor = NewObject<UBoxComponent>(FloorActor);
	FloorActor->AddInstanceComponent(Floor);
	Floor->SetMobility(EComponentMobility::Static);
	Floor->SetCollisionObjectType(ECC_WorldStatic);
	Floor->SetCollisionResponseToAllChannels(ECR_Block);
	Bridge->AnalyticSourceComponents.Add(1, Floor);
    // A real late-drop state reproduces a resolved vertex at time zero,
    // another arriving vertex, then a positive-time return of the first one.
    // Observe actual World dispatch, not a duplicate call to the pair helper.
    if (SelectedCell.IsEmpty())
    {
        AActor* ReturnActor = World->SpawnActor<AActor>();
        FBoxLawBody Body(*ReturnActor);
        Body.State.Location = FVector(11.149403915201553,4.856227467809191,20.003513503819136);
        Body.State.Rotation = FQuat(-0.99999812033462188,-0.0019385609136272487,-5.5043970713212138e-6,-3.5756242132778936e-5);
        Body.State.Velocity = FVector(.69425621613199984,.11150737165457771,-1.8660927358541797);
        Body.State.AngularVelocity = FVector(.00063574231657515066,.030601447411696948,.002065092988575479);
        Body.State.Acceleration = Body.GetNominalGravityAcceleration();
        Body.PreviousRotation = Body.State.Rotation;Body.UpdateSubBodiesKinematics();
        Bridge->AddComponent(Body);
        ON_SCOPE_EXIT { Bridge->RemoveComponent(Body);ReturnActor->Destroy(); };
        struct FReturnDispatchTrace
        {
            const USpeedWorldSubsystem* ExpectedWorld=nullptr;const UBoxSubBody* Box=nullptr;
            struct FEvent { int32 Corner=-1;float TOI=0,TimePassed=0;bool bIgnored=true; };
            FEvent Events[24];int32 Count=0;bool Overflow=false;
            static void Observe(void* O,const Speed::Testing::FCcdPoseAuditSample& Sample) noexcept
            {
                auto& Trace=*static_cast<FReturnDispatchTrace*>(O);
                if(Sample.World!=Trace.ExpectedWorld||Sample.Stage!=Speed::Testing::ECcdPoseAuditStage::BeforeResolve)return;
                if(Trace.Count>=24){Trace.Overflow=true;return;}
                const auto& Hit=Trace.Box->GetHit();auto& E=Trace.Events[Trace.Count++];
                E.Corner=Hit.ContactFeatureIndexThis;E.TOI=Hit.TOI;E.TimePassed=Sample.TimePassed;
                E.bIgnored=!Hit.Component.IsValid()||Trace.Box->ComponentHasBeenIgnored(*Hit.Component);
            }
        } Trace;
        Trace.ExpectedWorld=Bridge;Trace.Box=Body.Box;
        if(!TestTrue(TEXT("exact return dispatch observer installs without replacing a reader"),
            Speed::Testing::SetCcdPoseAuditObserverForCurrentThread(&FReturnDispatchTrace::Observe,&Trace)))return false;
        bool ObserverInstalled=true;
        ON_SCOPE_EXIT { if(ObserverInstalled)Speed::Testing::ClearCcdPoseAuditObserverForCurrentThread(&FReturnDispatchTrace::Observe,&Trace); };
        const FCanonicalFrameContext Context(0);Bridge->PrepareCanonicalFrame(Context);
        Bridge->Step(Context.PhysicalDeltaTime,Context.SimTime,0);
        const bool ObserverCleared=Speed::Testing::ClearCcdPoseAuditObserverForCurrentThread(&FReturnDispatchTrace::Observe,&Trace);ObserverInstalled=false;
        if(!TestTrue(TEXT("exact return dispatch observer closes"),ObserverCleared))return false;
        for(int32 Index=0;Index<Trace.Count;++Index)AddInfo(FString::Printf(TEXT("[ExactInitialReturnDispatch] Index=%d Vertex=%d TOI=%.17g TimePassed=%.17g Ignored=%d"),Index,Trace.Events[Index].Corner,double(Trace.Events[Index].TOI),double(Trace.Events[Index].TimePassed),int32(Trace.Events[Index].bIgnored)));
        const bool bHasDispatch=TestTrue(TEXT("exact initial event followed by two real positive arrivals is dispatched"),!Trace.Overflow&&Trace.Count>=3);
        if(bHasDispatch)
        {
        TestTrue(TEXT("first actual resolution is initial vertex time zero"),Trace.Events[0].Corner>=0&&Trace.Events[0].TOI==0&&Trace.Events[0].TimePassed==0);
        TestTrue(TEXT("second actual resolution is another positive-time vertex"),Trace.Events[1].Corner!=Trace.Events[0].Corner&&Trace.Events[1].TOI>0);
        TestTrue(TEXT("initial vertex can genuinely return after the second event"),Trace.Events[2].Corner==Trace.Events[0].Corner&&Trace.Events[2].TOI>0&&Trace.Events[2].TimePassed>Trace.Events[1].TimePassed&&Trace.Events[2].TimePassed<Context.PhysicalDeltaTime);
        }
        const bool bHasFourth=TestTrue(TEXT("fourth distinct exact contact reaches actual World dispatch"),!Trace.Overflow&&Trace.Count>=4);
        if(bHasFourth)
        {
            const auto& Fourth=Trace.Events[3];
            TestTrue(TEXT("fourth exact vertex is distinct from both earlier vertices"),Fourth.Corner>=0&&Fourth.Corner!=Trace.Events[0].Corner&&Fourth.Corner!=Trace.Events[1].Corner);
            TestTrue(TEXT("fourth actual arrival advances positive physical time"),Fourth.TOI>0&&Fourth.TimePassed>Trace.Events[2].TimePassed&&Fourth.TimePassed<Context.PhysicalDeltaTime);
            TestFalse(TEXT("fourth exact static vertex is not component-ignored"),Fourth.bIgnored);
        }
    }
    // Stage02 frame2979 authors only pose/velocities at frame start.
    // Gravity/zero external angular acceleration are reset by the production fixture.
    // Observe actual World dispatch, not a duplicate call to the pair helper.
    if (SelectedCell.IsEmpty())
    {
        AActor* ReturnActor = World->SpawnActor<AActor>();
        FBoxLawBody Body(*ReturnActor);
        Body.State.Location = FVector(7.0528285513881812,4.9581409305892645,20.001249557755372);
        Body.State.Rotation = FQuat(-0.99999266029863321,-0.0038313537884594106,-6.458705169172705e-06,-5.9409647241519729e-06);
        Body.State.Velocity = FVector(0.060069147987176816,-0.050084677292159499,-0.72249402326139656);
        Body.State.AngularVelocity = FVector(0.009592993793980523,0.0056910903328626691,0.00070872950238149118);
        Body.State.Acceleration = Body.GetNominalGravityAcceleration();
        Body.PreviousRotation = Body.State.Rotation;Body.UpdateSubBodiesKinematics();
        Bridge->AddComponent(Body);
        ON_SCOPE_EXIT { Bridge->RemoveComponent(Body);ReturnActor->Destroy(); };
        struct FReturnDispatchTrace
        {
            const USpeedWorldSubsystem* ExpectedWorld=nullptr;const UBoxSubBody* Box=nullptr;
            struct FEvent { int32 Corner=-1;float TOI=0,TimePassed=0;bool bIgnored=true; };
            FEvent Events[24];int32 Count=0;bool Overflow=false;
            static void Observe(void* O,const Speed::Testing::FCcdPoseAuditSample& Sample) noexcept
            {
                auto& Trace=*static_cast<FReturnDispatchTrace*>(O);
                if(Sample.World!=Trace.ExpectedWorld||Sample.Stage!=Speed::Testing::ECcdPoseAuditStage::BeforeResolve)return;
                if(Trace.Count>=24){Trace.Overflow=true;return;}
                const auto& Hit=Trace.Box->GetHit();auto& E=Trace.Events[Trace.Count++];
                E.Corner=Hit.ContactFeatureIndexThis;E.TOI=Hit.TOI;E.TimePassed=Sample.TimePassed;
                E.bIgnored=!Hit.Component.IsValid()||Trace.Box->ComponentHasBeenIgnored(*Hit.Component);
            }
        } Trace;
        Trace.ExpectedWorld=Bridge;Trace.Box=Body.Box;
        if(!TestTrue(TEXT("Stage02 seed: exact return dispatch observer installs without replacing a reader"),
            Speed::Testing::SetCcdPoseAuditObserverForCurrentThread(&FReturnDispatchTrace::Observe,&Trace)))return false;
        bool ObserverInstalled=true;
        ON_SCOPE_EXIT { if(ObserverInstalled)Speed::Testing::ClearCcdPoseAuditObserverForCurrentThread(&FReturnDispatchTrace::Observe,&Trace); };
        const FCanonicalFrameContext Context(0);Bridge->PrepareCanonicalFrame(Context);
        Bridge->Step(Context.PhysicalDeltaTime,Context.SimTime,0);
        const bool ObserverCleared=Speed::Testing::ClearCcdPoseAuditObserverForCurrentThread(&FReturnDispatchTrace::Observe,&Trace);ObserverInstalled=false;
        if(!TestTrue(TEXT("Stage02 seed: exact return dispatch observer closes"),ObserverCleared))return false;
        for(int32 Index=0;Index<Trace.Count;++Index)AddInfo(FString::Printf(TEXT("Stage02 seed: [ExactFourthStage02Dispatch] Index=%d Vertex=%d TOI=%.17g TimePassed=%.17g Ignored=%d"),Index,Trace.Events[Index].Corner,double(Trace.Events[Index].TOI),double(Trace.Events[Index].TimePassed),int32(Trace.Events[Index].bIgnored)));
        const bool bHasDispatch=TestTrue(TEXT("Stage02 seed: exact initial event followed by two real positive arrivals is dispatched"),!Trace.Overflow&&Trace.Count>=3);
        if(bHasDispatch)
        {
        TestTrue(TEXT("Stage02 seed: first actual resolution is initial vertex time zero"),Trace.Events[0].Corner>=0&&Trace.Events[0].TOI==0&&Trace.Events[0].TimePassed==0);
        TestTrue(TEXT("Stage02 seed: second actual resolution is another positive-time vertex"),Trace.Events[1].Corner!=Trace.Events[0].Corner&&Trace.Events[1].TOI>0);
        TestTrue(TEXT("Stage02 seed: third arrival is a new positive-time vertex"),Trace.Events[2].Corner!=Trace.Events[0].Corner&&Trace.Events[2].Corner!=Trace.Events[1].Corner&&Trace.Events[2].TOI>0&&Trace.Events[2].TimePassed>Trace.Events[1].TimePassed&&Trace.Events[2].TimePassed<Context.PhysicalDeltaTime);
        }
        const bool bHasFourth=TestTrue(TEXT("Stage02 seed: fourth distinct exact contact reaches actual World dispatch"),!Trace.Overflow&&Trace.Count>=4);
        if(bHasFourth)
        {
            const auto& Fourth=Trace.Events[3];
            TestTrue(TEXT("Stage02 seed: fourth exact vertex is distinct from both earlier vertices"),Fourth.Corner>=0&&Fourth.Corner!=Trace.Events[0].Corner&&Fourth.Corner!=Trace.Events[1].Corner&&Fourth.Corner!=Trace.Events[2].Corner);
            TestTrue(TEXT("Stage02 seed: fourth actual arrival advances positive physical time"),Fourth.TOI>0&&Fourth.TimePassed>Trace.Events[2].TimePassed&&Fourth.TimePassed<Context.PhysicalDeltaTime);
            TestFalse(TEXT("Stage02 seed: fourth exact static vertex is not component-ignored"),Fourth.bIgnored);
            bool bInterveningOtherVertex=false,bFourthReturned=false;
            for(int32 Index=4;Index<Trace.Count;++Index)
            {
                const auto& Event=Trace.Events[Index];
                if(Event.Corner!=Fourth.Corner)bInterveningOtherVertex=true;
                if(bInterveningOtherVertex&&Event.Corner==Fourth.Corner&&Event.TOI>0&&Event.TimePassed>Fourth.TimePassed&&Event.TimePassed<Context.PhysicalDeltaTime&&!Event.bIgnored)
                {bFourthReturned=true;break;}
            }
            TestTrue(TEXT("Stage02 seed: fourth vertex returns nonignored after another real vertex and positive physical time"),bFourthReturned);
        }
    }
    // Complement the real dispatch oracle with the existing four-hit fallback:
    // exact native vertex/face arrivals are the sole exemption; every other
    // component policy still ignores after its fourth accepted contact.
    if(SelectedCell.IsEmpty())
    for(int32 PolicyCase=0;PolicyCase<11;++PolicyCase)
    {
        AActor* PolicyActor=World->SpawnActor<AActor>();
        FBoxLawBody Body(*PolicyActor);
        ON_SCOPE_EXIT { PolicyActor->Destroy(); };
        UBoxComponent* PolicySurface=NewObject<UBoxComponent>(PolicyActor);
        PolicyActor->AddInstanceComponent(PolicySurface);
        PolicySurface->SetMobility(EComponentMobility::Static);
        PolicySurface->SetCollisionObjectType(PolicyCase==7?ECC_WorldDynamic:ECC_WorldStatic);
        Body.SetStaticCollisionWorldForFrame(PolicyCase==8?nullptr:Bridge->GetStaticCollisionWorld(),FCanonicalFrameContext(0).PhysicalDeltaTime);
        SHitResult Hit;Hit.bBlockingHit=true;Hit.Component=PolicySurface;Hit.TOI=PolicyCase==1?0.f:.001f;
        Hit.SourceId=1;Hit.SurfaceId=2;Hit.ContactFeatureThis=Speed::EContactFeatureKind::Vertex;
        Hit.ContactFeatureIndexThis=0;Hit.ContactFeatureOther=Speed::EContactFeatureKind::Face;
        if(PolicyCase==2)Hit.bSurfaceNormalMayVary=true;
        if(PolicyCase==3)Hit.GeometricErrorBoundCm=.001f;
        if(PolicyCase==4)Hit.SourceId=0;
        if(PolicyCase==5){Hit.bStartPenetrating=true;Hit.PenetrationDepth=.001f;}
        if(PolicyCase==6)Hit.ContactFeatureOther=Speed::EContactFeatureKind::Edge;
        if(PolicyCase==9)Hit.SurfaceId=0;
        if(PolicyCase==10)Hit.ContactFeatureThis=Speed::EContactFeatureKind::Face;
        for(int32 Contact=0;Contact<3;++Contact){Body.Box->SetFutureHit(Hit);Body.Box->AcceptHit();}
        TestFalse(TEXT("legacy component policy does not ignore before fourth acceptance"),Body.Box->ComponentHasBeenIgnored(*PolicySurface));
        Body.Box->SetFutureHit(Hit);Body.Box->AcceptHit();
        const bool bExactNative=PolicyCase<2;
        TestEqual(TEXT("fourth acceptance preserves exact-only exemption and every legacy fallback"),Body.Box->ComponentHasBeenIgnored(*PolicySurface),!bExactNative);
        if(bExactNative)
        {
            SHitResult Ordinary=Hit;Ordinary.bSurfaceNormalMayVary=true;
            Body.Box->SetFutureHit(Ordinary);Body.Box->AcceptHit();
            TestTrue(TEXT("exact exemption keeps contact count so a later ordinary contact still ignores"),Body.Box->ComponentHasBeenIgnored(*PolicySurface));
        }
        else
        {
            SHitResult Later=Hit;Later.bSurfaceNormalMayVary=false;Later.GeometricErrorBoundCm=0;
            Later.SourceId=1;Later.SurfaceId=2;Later.bStartPenetrating=false;Later.PenetrationDepth=0;
            Later.ContactFeatureOther=Speed::EContactFeatureKind::Face;
            Body.Box->SetFutureHit(Later);Body.Box->AcceptHit();
            TestTrue(TEXT("later exact-looking acceptance never removes prior ignored membership"),Body.Box->ComponentHasBeenIgnored(*PolicySurface));
        }
    }
	// Regression at the actual Box -> owner integration boundary, rather than
	// another call to the stand-alone reaction helper. Only INITIAL geometry is
	// authored here. All measured reactions use a fresh finite provider witness.
	if (SelectedCell.IsEmpty())
	for (int32 ExpectedPoints : { 1, 2 })
	for (double Sign : { -1.0, 1.0 })
	for (float Friction : { 0.0f, 0.3f })
	{
		AActor* SlidingActor = World->SpawnActor<AActor>();
		ON_SCOPE_EXIT { SlidingActor->Destroy(); };
		FBoxLawBody Body(*SlidingActor, Friction);
		const float Dt = FCanonicalFrameContext(0).PhysicalDeltaTime;
		Body.SetStaticCollisionWorldForFrame(Bridge->GetStaticCollisionWorld(), Dt);
		Body.State.Rotation = FQuat(FVector::ForwardVector, 0.2);
		if (ExpectedPoints == 1)
			Body.State.Rotation = FQuat(FVector::RightVector, 0.13) * Body.State.Rotation;
		// Configure a legal initial exterior pose; no correction is applied in
		// response to a measured force or after integration begins.
		Body.State.Location.Z -= Body.Clearance();
		Body.State.Location.Z += 8 * DBL_EPSILON * 120;
		Body.State.Velocity = FVector(Sign * 450, 0, 0);
		Body.State.Acceleration = Body.GetNominalGravityAcceleration();
		Body.UpdateSubBodiesKinematics();
		const SKinematic Initial = Body.State;
		FVector Corners[8];double Lowest = DBL_MAX;
		for (int32 Corner = 0; Corner < 8; ++Corner)
		{
			const FVector E = Body.Config.BoxExtent;
			const FVector Local((Corner & 1) ? E.X : -E.X,
				(Corner & 2) ? E.Y : -E.Y, (Corner & 4) ? E.Z : -E.Z);
			Corners[Corner] = Initial.Location + Initial.Rotation.RotateVector(Local);
			Lowest = FMath::Min(Lowest, Corners[Corner].Z);
		}
		TArray<FVector> Points;
		for (const FVector& Point : Corners) if (Point.Z == Lowest) Points.Add(Point);
		if (!TestEqual(TEXT("partial sliding fixture has the authored vertex/edge"), Points.Num(), ExpectedPoints)) return false;
		Speed::Analytic::FWorldQuery Ray;
		Ray.Shape = Speed::Analytic::EQueryShape::Ray;
		Ray.Start = Points[0] + FVector(0, 0, .01);
		Ray.End = Points[0] - FVector(0, 0, .01);
		Ray.RequiredSourceId = 1; Ray.RequiredSurfaceId = 2;
		Ray.bAuthorityOnly = true; Ray.bIncludeCompactPatches = true;
		Ray.DomainTolerance = 0; Ray.InitialOverlapTolerance = 0;
		const auto Witness = Bridge->GetStaticCollisionWorld()->SweepSingle(Ray);
		if (!TestTrue(TEXT("partial sliding support has a fresh exact finite face witness"),
			Witness.bHit && !Witness.bStartPenetrating && !Witness.bSurfaceNormalMayVary &&
			Witness.GeometricErrorBoundCm == 0 && Witness.SurfaceFeatureKind == Speed::EContactFeatureKind::Face &&
			Witness.Normal == FVector::UpVector)) return false;
		Body.Box->SetFutureHit(SHitResult::FromAnalyticHit(Witness, 0, Floor));
		Body.Box->AcceptHit(); // Genuine current provider contact, no response/pose mutation.
		FVector Linear, Angular;
		Body.GetStaticContactAcceleration(Linear, Angular);
		const FString Label = FString::Printf(TEXT("partial sliding points%d sign%.0f mu%.1f"), ExpectedPoints, Sign, double(Friction));
		TestTrue(Label + TEXT(": reaction query preserves pose and velocity"),
			Body.State.Location == Initial.Location && Body.State.Rotation == Initial.Rotation &&
			Body.State.Velocity == Initial.Velocity && Body.State.AngularVelocity == Initial.AngularVelocity);
		TestTrue(Label + TEXT(": normal pressure remains non-attractive"), Linear.Z > 0);
		for (const FVector& Point : Points)
		{
			// Initial spin is zero, so this independently checks the actual
			// acceleration-level reaction on every geometrically supplied row.
			const FVector AP = Initial.Acceleration + Linear + FVector::CrossProduct(Angular, Point - Initial.Location);
			TestTrue(Label + TEXT(": coupled contact normal acceleration balances"), FMath::Abs(AP.Z) < 1.e-7);
		}
		if (Friction == 0)
			TestTrue(Label + TEXT(": frictionless contact retains zero tangent reaction"), Linear.X == 0 && Linear.Y == 0);
		else
		{
			TestTrue(Label + TEXT(": established partial slip has dissipative continuous traction"),
				FVector::DotProduct(Linear, Initial.Velocity) < 0);
			TestTrue(Label + TEXT(": total traction stays in the Coulomb cone"),
				FVector(Linear.X, Linear.Y, 0).Size() <= double(Friction) * Linear.Z + 1.e-7);
			Body.IntegrateKinematics(Dt);
			TestTrue(Label + TEXT(": real owner integration reduces the signed slip"),
				Sign * Body.State.Velocity.X < 450 && Sign * Body.State.Velocity.X > 0);
		}
	}
	{
		AActor* GuardActor = World->SpawnActor<AActor>();
		ON_SCOPE_EXIT { GuardActor->Destroy(); };
		FBoxLawBody Body(*GuardActor, 0); // Frictionless speed at the cap is still admissible.
		TestFalse(TEXT("an isolated box has no foreign support constraint"),
			Body.HasActivePhysicalConstraintsOtherThan(Body.Box));
		FPhysicalContactConstraint Contact;
		Contact.Normal = FVector::UpVector;
		Contact.OtherComponent = Floor;
		Contact.SourceSubBody = Body.Box;
		Body.RegisterPhysicalConstraint(Contact);
		TestFalse(TEXT("its own contact does not exclude an isolated box solve"),
			Body.HasActivePhysicalConstraintsOtherThan(Body.Box));
		Body.ClearPhysicalConstraints();
		Contact.SourceSubBody = NewObject<UBoxSubBody>(GuardActor);
		Body.RegisterPhysicalConstraint(Contact);
		TestTrue(TEXT("another sub-body's live support requires a coupled solve"),
			Body.HasActivePhysicalConstraintsOtherThan(Body.Box));
		Body.ClearPhysicalConstraints();
		TestFalse(TEXT("released support cannot keep the isolated solve disabled"),
			Body.HasActivePhysicalConstraintsOtherThan(Body.Box));
		Body.MaxLinearSpeed = 2300;
		const float Dt = FCanonicalFrameContext(0).PhysicalDeltaTime;
		Body.SetStaticCollisionWorldForFrame(Bridge->GetStaticCollisionWorld(), Dt);
		for (int32 Face = 0; Face < 6; ++Face)
		{
			FVector FaceNormal = FVector::ZeroVector;
			FaceNormal[Face / 2] = Face % 2 ? 1.0 : -1.0;
			Body.State.Rotation = FQuat::FindBetweenNormals(FaceNormal, FVector::DownVector);
			Body.State.Location = FVector(0, 0, Body.Config.BoxExtent[Face / 2] + 2 * DBL_EPSILON * 120);
			Body.State.Velocity = FVector(2300, 0, 0);
			Body.State.Acceleration = Body.GetNominalGravityAcceleration();
			TestTrue(TEXT("speed cap must not include gravity already opposed by exact face support"),
				Body.IsSpeedLimitPreservedByExactSupport(Dt));
		}
		const SKinematic Supported = Body.State;
		for (const double Yaw : { 0.0, 40.0, 80.0 })
		{
			const FQuat Local = FRotator(-.55, 0, 0).Quaternion();
			const FQuat Owner = FRotator(0, Yaw, 0).Quaternion() * Supported.Rotation * Local.Inverse();
			FVector Direction = (Owner * Local).RotateVector(FVector::ForwardVector);
			Direction.Z = 0; Direction.Normalize();
			Body.State.Velocity = Direction * 2300;
			TestTrue(FString::Printf(TEXT("supported speed cap heading=%.0f squared_speed=%.17g"), Yaw, Body.State.Velocity.SizeSquared()),
				Body.IsSpeedLimitPreservedByExactSupport(Dt));
		}
		Body.State = Supported;
		Body.State.Location.X = 10100;
		TestFalse(TEXT("support outside the finite provider cannot bypass the speed cap"), Body.IsSpeedLimitPreservedByExactSupport(Dt));
		Body.State = Supported; Body.State.Velocity.Z = 1;
		TestFalse(TEXT("a separating impulse releases supported speed admission"), Body.IsSpeedLimitPreservedByExactSupport(Dt));
		Body.State = Supported; Body.State.AngularVelocity.X = 1.e-8;
		TestFalse(TEXT("a finite spin cannot be snapped by supported speed admission"), Body.IsSpeedLimitPreservedByExactSupport(Dt));
		Body.State = Supported; Body.State.Acceleration.X = 1;
		TestFalse(TEXT("uncertified tangential load retains ordinary speed limiting"), Body.IsSpeedLimitPreservedByExactSupport(Dt));
		Body.State = Supported;
		Body.SetStaticCollisionWorldForFrame(Bridge->GetStaticCollisionWorld(), Dt * .5f);
		TestFalse(TEXT("a shorter certified horizon cannot authorize a full-frame speed bypass"), Body.IsSpeedLimitPreservedByExactSupport(Dt));
		Body.SetStaticCollisionWorldForFrame(Bridge->GetStaticCollisionWorld(), Dt);
		Body.MaxLinearSpeed = 2200;
		TestFalse(TEXT("an already excessive velocity still requires the speed cap"), Body.IsSpeedLimitPreservedByExactSupport(Dt));
	}
	// Independent covariance check: apply a world torque by unrotating it to
	// principal box axes, applying the diagonal tensor, and rotating it back.
	// This avoids assuming an FMatrix row/column convention in the oracle.
	{
		AActor* InertiaActor = World->SpawnActor<AActor>();
		ON_SCOPE_EXIT { InertiaActor->Destroy(); };
		for (const FQuat& LocalQ : { FQuat::Identity, FRotator(20, 30, 40).Quaternion() })
		{
			FBoxLawGeometry Geometry;
			Geometry.LocalRotation = LocalQ;
			FBoxLawBody Body(*InertiaActor, .5f, Geometry);
			const FVector E = Geometry.Extent;
			// Test the frame conversion independently of the existing float
			// material/inertia initialization, whose representable values are inputs.
			const FVector Principal = Speed::SBox::ComputeInertiaTensor(E, Geometry.Mass);
			const FVector Diagonal(1.0 / Principal.X, 1.0 / Principal.Y, 1.0 / Principal.Z);
			for (const FRotator& Angles : { FRotator(0, 45, 0), FRotator(40, 30, 20),
				FRotator(89.9999, 30, 40), FRotator(-89.9999, -30, -40) })
			{
				Body.State.Rotation = Angles.Quaternion();
				const FQuat Q = Body.State.Rotation * LocalQ;
				for (const FVector& Torque : { FVector(10000, 0, 0), FVector(0, 10000, 0), FVector(0, 0, 10000) })
				{
					const FVector Expected = Q.RotateVector(Diagonal * Q.UnrotateVector(Torque));
					const FVector Actual = Body.Box->ComputeWorldInvInertiaTensor().TransformVector(Torque);
					TestTrue(TEXT("world inverse inertia rotates torque through principal axes without Euler loss"),
						Actual.Equals(Expected, 256 * DBL_EPSILON * FMath::Max(1.0, Expected.Size())));
					const FVector LocalExpected = LocalQ.RotateVector(Diagonal * LocalQ.UnrotateVector(Torque));
					const FVector LocalActual = Body.Box->ComputeChassisLocalInvInertiaTensor().TransformVector(Torque);
					TestTrue(TEXT("chassis inverse inertia respects the local box rotation"),
						LocalActual.Equals(LocalExpected, 256 * DBL_EPSILON * FMath::Max(1.0, LocalExpected.Size())));
				}
			}
		}
	}

	struct FCell
	{
		FString Name;
		FQuat Rotation;
		FVector Velocity = FVector::ZeroVector;
		double CenterHeight = 100;
		bool bSlide = false;
		float Friction = 0;
		FVector Spin = FVector::ZeroVector;
		FBoxLawGeometry Geometry;
		FVector CenterXY = FVector::ZeroVector;
		double FloorZ = 0;
		bool bLateFeasibility = false;
		bool bDuplicateSupport = false;
		int32 ExpectedFaceAxis = INDEX_NONE;
	};
	TArray<FCell> Cells;
	Cells.Add({ TEXT("normal-drop-roof"), FRotator(0, 0, 180).Quaternion() });
	Cells.Add({ TEXT("tilted-drop"), FRotator(20, 0, 160).Quaternion() });
	Cells.Add({ TEXT("roll-drop"), FRotator(0, 0, 160).Quaternion() });
	Cells.Add({ TEXT("pitch-drop"), FRotator(20, 0, 180).Quaternion() });
	Cells.Add({ TEXT("rotation-only-first-contact"), FRotator(0, 0, 180).Quaternion(), FVector::ZeroVector,
		20.5, false, 0.5f, FVector(4, 0, 0) });
	const FRotator Faces[] = { FRotator(0, 0, 180), FRotator::ZeroRotator,
		FRotator(90, 0, 0), FRotator(-90, 0, 0), FRotator(0, 0, 90), FRotator(0, 0, -90) };
	const double FaceHeights[] = { 20, 20, 60, 60, 40, 40 };
	for (int32 Face = 0; Face < 6; ++Face)
	{
		// A central, face-parallel, non-spinning drop must acquire that very
		// face. Unlike a vehicle, no recovery policy can prefer another face.
		FCell Aligned;
		Aligned.Name = FString::Printf(TEXT("six-face-aligned-drop%d"), Face);
		Aligned.Rotation = Faces[Face].Quaternion();
		Aligned.CenterHeight = FaceHeights[Face] + 30;
		Aligned.ExpectedFaceAxis = Face < 2 ? 2 : Face < 4 ? 0 : 1;
		Cells.Add(Aligned);
		for (int32 TiltAxis = 0; TiltAxis < 2; ++TiltAxis)
		for (double Tilt : { 20.0, 40.0, 80.0 })
		for (double Spin : { 0.0, 2.0, 4.0 })
		{
			FCell Perturbed = Aligned;
			Perturbed.Name = FString::Printf(TEXT("six-face-acquisition%d-axis%d-tilt%.0f-spin%.0f"), Face, TiltAxis, Tilt, Spin);
			const FVector Axis = TiltAxis == 0 ? FVector::ForwardVector : FVector::RightVector;
			Perturbed.Rotation = FQuat(Axis, FMath::DegreesToRadians(Tilt)) * Aligned.Rotation;
			Perturbed.Spin = Axis * Spin;
			const FVector E = Perturbed.Geometry.Extent;
			const FQuat Q = Perturbed.Rotation;
			Perturbed.CenterHeight = 30 + FMath::Abs(Q.GetAxisX().Z) * E.X +
				FMath::Abs(Q.GetAxisY().Z) * E.Y + FMath::Abs(Q.GetAxisZ().Z) * E.Z;
			// A finite perturbation may physically tip onto another face. Its
			// terminal face must satisfy the same generic geometric certificate.
			Perturbed.ExpectedFaceAxis = INDEX_NONE;
			Cells.Add(Perturbed);
		}
	}
	for (int32 Face = 0; Face < 6; ++Face)
	for (double Heading : { 0.0, 40.0, 80.0 })
	for (double Speed : { -1400.0, 1400.0, 2300.0 })
	{
		const FQuat Yaw = FRotator(0, Heading, 0).Quaternion();
		Cells.Add({ FString::Printf(TEXT("slide-face%d-yaw%.0f-speed%.0f"), Face, Heading, Speed),
			(Yaw * Faces[Face].Quaternion()).GetNormalized(), Yaw.RotateVector(FVector(Speed, 0, 0)), FaceHeights[Face], true });
	}
	for (double Heading : { 0.0, 40.0, 80.0 })
	for (double Speed : { 1.0, 100.0, -1400.0, 1400.0, 2300.0 })
	{
		const FQuat Yaw = FRotator(0, Heading, 0).Quaternion();
		Cells.Add({ FString::Printf(TEXT("coulomb-slide-roof-yaw%.0f-speed%.0f"), Heading, Speed),
			(Yaw * Faces[0].Quaternion()).GetNormalized(), Yaw.RotateVector(FVector(Speed, 0, 0)), 20, true, 0.5f });
	}
	// Separate local geometry/COM from every vehicle mechanism. Unequal box,
	// non-grid local inclination and displaced mass are generic rigid-body inputs.
	for (int32 Face = 0; Face < 6; ++Face)
	{
		FCell Cell;
		Cell.Name = FString::Printf(TEXT("offset-com-drop-face%d"), Face);
		Cell.Geometry = { FVector(79, 52, 19), FVector(-5, 0, 37), FVector(-16, 0, 22),
			FRotator(-.55, 0, 0).Quaternion(), .3f };
		Cell.Rotation = Faces[Face].Quaternion() * Cell.Geometry.LocalRotation.Inverse();
		const int32 Axis = Face < 2 ? 2 : Face < 4 ? 0 : 1;
		Cell.CenterHeight = Cell.Geometry.Extent[Axis] + 30;
		Cells.Add(Cell);
	}
	for (double Height : { 0.0, -0.0035156249068749703 })
	for (int32 Heading = 0; Heading < 2; ++Heading)
	for (bool bLate : { false, true })
	{
		FCell Cell;
		Cell.Name = FString::Printf(TEXT("high-mass-material-local-roof%d-floor%.6f-late%d"), Heading, Height, bLate ? 1 : 0);
		Cell.Geometry = { FVector(78.79f, 52.30f, 19.14f), FVector(-5.19f, 0, 36.51f), FVector(-16, 0, 22),
			FRotator(-.55, 0, 0).Quaternion(), .3f, 1000, .3f, 10 };
		Cell.Rotation = FRotator(0, Heading * 180.0, 0).Quaternion() *
			FQuat::FindBetweenNormals(FVector::UpVector, FVector::DownVector) * Cell.Geometry.LocalRotation.Inverse();
		Cell.CenterHeight = Cell.Geometry.Extent.Z + 30;
		Cell.CenterXY = FVector(-3000, 1500, 0);
		Cell.FloorZ = Height;
		Cell.bLateFeasibility = bLate;
		Cells.Add(Cell);
	}
	// Arrival phase and an inactive angular ceiling cannot choose the final
	// geometric equilibrium. Cover both without any vehicle helper in the loop.
	const FCell PhaseBase = Cells.Last();
	FCell Duplicate = PhaseBase;
	Duplicate.Name = TEXT("duplicate-coplanar-provider");
	Duplicate.bDuplicateSupport = true;
	Cells.Add(Duplicate);
	for (float Cap : { 5.5f, 100.f })
	for (double PhaseGap : { 0.0, .1, .2, .3, .4, .5 })
	{
		FCell Cell = PhaseBase;
		Cell.Name = FString::Printf(TEXT("arrival-phase-gap%.1f-cap%.1f"), PhaseGap, Cap);
		Cell.Geometry.MaxAngularSpeed = Cap;
		Cell.CenterHeight += PhaseGap;
		Cells.Add(Cell);
	}
	TArray<SKinematic> SingleProviderTrace;
	int32 ExecutedCells = 0;
	for (int32 Cell = 0; Cell < Cells.Num(); ++Cell)
	{
		const FCell& Fixture = Cells[Cell];
		if (!SelectedCell.IsEmpty() && Fixture.Name != SelectedCell) continue;
		++ExecutedCells;
		if (Data->Planes[0].Origin.Z != Fixture.FloorZ || Data->Planes.Num() != (Fixture.bDuplicateSupport ? 2 : 1))
		{
			// Replace the immutable world between isolated cells, never during a
			// physical frame. This retains both grid-aligned and off-grid floors.
			auto NewData = MakeShared<Speed::Analytic::FAnalyticWorldData>();
			Plane.Origin.Z = Fixture.FloorZ;
			NewData->Planes.Add(Plane);
			if (Fixture.bDuplicateSupport)
			{
				auto DuplicatePlane = Plane;
				DuplicatePlane.FeatureId = 5;
				DuplicatePlane.PrimitiveId = 6;
				NewData->Planes.Add(DuplicatePlane);
			}
			if (!TestTrue(TEXT("translated finite plane validates"), NewData->FinalizeAndValidate())) return false;
			Data = NewData;
			Bridge->AnalyticWorldData = Data;
			Bridge->SimulationWorld.SetStaticCollisionWorld(MakeUnique<Speed::FAnalyticStaticCollisionWorld>(*Data));
		}
		AActor* Actor = World->SpawnActor<AActor>();
		FBoxLawBody Body(*Actor, Fixture.bSlide ? Fixture.Friction : Fixture.Geometry.Friction, Fixture.Geometry);
		Body.FloorZ = Fixture.FloorZ;
		Body.bLateFeasibilityPass = Fixture.bLateFeasibility;
		Body.State.Rotation = Fixture.Rotation;
		Body.State.Location = FVector(Fixture.CenterXY.X, Fixture.CenterXY.Y, Fixture.CenterHeight + Fixture.FloorZ) +
			Fixture.Rotation.RotateVector(Fixture.Geometry.COM - Fixture.Geometry.Offset);
		Body.State.Velocity = Fixture.Velocity;
		Body.State.AngularVelocity = Fixture.Spin;
		// Inputs enter the physical test on the existing canonical velocity grid.
		// Quantize once before observation, not after a measured force-free step.
		SKinematic CanonicalInput = Body.State;
		CanonicalInput.Quantize(Body.State.Rotation);
		Body.State.Velocity = CanonicalInput.Velocity;
		const FVector InitialVelocity = Body.State.Velocity;
		if (Fixture.bSlide)
		{
			// Initial representability only; observation never moves a sliding body.
			Body.State.Location.Z += 2.0 * DBL_EPSILON * 120.0;
			Body.TraceSamples = 32;
		}
		Body.PreviousRotation = Body.State.Rotation;
		Bridge->AddComponent(Body);
		ON_SCOPE_EXIT { Bridge->RemoveComponent(Body); Actor->Destroy(); };
		int32 StableFrames = 0;
		const int32 FrameCount = Fixture.bSlide ? 900 : 3000;
		double MaxHeightError = 0, MaxRotationError = 0, MaxVelocityError = 0;
		double MinimumForwardSpeed = InitialVelocity.Size();
		int32 BudgetExhaustions = 0;
		int32 DuplicateTrajectoryDifferences = 0;
		for (int32 Frame = 0; Frame < FrameCount; ++Frame)
		{
			if (ContactDebug) ContactDebug->Set(((Fixture.Name == TEXT("roll-drop") && Frame >= 159 && Frame <= 164) ||
				(Fixture.bLateFeasibility && Frame >= 89 && Frame <= 139) ||
				((!SelectedCell.IsEmpty() || Fixture.Name == TEXT("six-face-acquisition1-axis1-tilt80-spin0")) && Frame >= 2979)) ? 1 : 0, ECVF_SetByCode);
			const FCanonicalFrameContext Context(Frame);
			Speed::Analytic::FStaticWorldQueryAudit::BeginFrame(Frame, &Data.Get(), Bridge);
			Bridge->PrepareCanonicalFrame(Context);
			Bridge->Step(Context.PhysicalDeltaTime, Context.SimTime, Frame);
			BudgetExhaustions += Bridge->GetLastStepDiagnostics().bIterationLimitReached ? 1 : 0;
			Speed::Analytic::FStaticWorldQueryAudit::EndFrame();
			if (Fixture.Name == PhaseBase.Name) SingleProviderTrace.Add(Body.State);
			if (Fixture.bDuplicateSupport && SingleProviderTrace.IsValidIndex(Frame))
			{
				const SKinematic& Reference = SingleProviderTrace[Frame];
				DuplicateTrajectoryDifferences += Body.State.Location != Reference.Location ||
					Body.State.Rotation != Reference.Rotation || Body.State.Velocity != Reference.Velocity ||
					Body.State.AngularVelocity != Reference.AngularVelocity ? 1 : 0;
			}
			Speed::FBoxRestingSupport Support;
			const bool bSupported = Body.Box->EvaluateStaticRestingSupport(*Bridge->GetStaticCollisionWorld(), Body.GetNominalGravityAcceleration(), Support);
			StableFrames = bSupported && Body.State.Velocity.Z == 0 && Body.State.AngularVelocity.IsZero() ? StableFrames + 1 : 0;
			if (Fixture.bSlide)
			{
				MaxHeightError = FMath::Max(MaxHeightError, FMath::Abs(Body.State.Location.Z - Fixture.CenterHeight));
				const double Sign = (Body.State.Rotation | Fixture.Rotation) < 0 ? -1.0 : 1.0;
				MaxRotationError = FMath::Max(MaxRotationError, (Body.State.Rotation - Sign * Fixture.Rotation).SizeSquared());
				MaxVelocityError = FMath::Max(MaxVelocityError, (Body.State.Velocity - InitialVelocity).Size());
				MinimumForwardSpeed = FMath::Min(MinimumForwardSpeed, FVector::DotProduct(Body.State.Velocity, InitialVelocity.GetSafeNormal()));
			}
		}
		AddInfo(FString::Printf(TEXT("box-law cell=%s stable_frames=%d min_gap=%.17g final_gap=%.17g speed=%.17g spin=%.17g samples=%u height_error=%.17g rotation_error=%.17g velocity_error=%.17g"),
			*Fixture.Name, StableFrames, Body.MinimumClearance, Body.Clearance(), Body.State.Velocity.Size(), Body.State.AngularVelocity.Size(), Body.IntegrationSamples,
			MaxHeightError, MaxRotationError, MaxVelocityError));
		if (!Fixture.bSlide) AddInfo(FString::Printf(TEXT("box-law final_orientation cell=%s z=%.17g q=%s"),
			*Fixture.Name, double(Body.State.Location.Z), *Body.State.Rotation.ToString()));
		TestTrue(Fixture.Name + TEXT(": converges to exact face support and holds for at least one second"), StableFrames >= 300);
		TestEqual(TEXT("later feasibility projection preserves passive support without a new impulse"), Body.PassiveProjectionFailures, 0u);
		TestTrue(Fixture.Name + TEXT(": no observed integration or canonical state penetrates the plane"), Body.MinimumClearance >= 0);
		TestEqual(Fixture.Name + TEXT(": support events never exhaust the canonical solver budget"), BudgetExhaustions, 0);
		if (Fixture.ExpectedFaceAxis != INDEX_NONE)
		{
			const FVector TargetNormal = Fixture.Rotation.UnrotateVector(FVector::UpVector);
			const FVector FinalNormal = Body.State.Rotation.RotateVector(TargetNormal);
			TestTrue(Fixture.Name + TEXT(": acquires the selected face without a preferred-face policy"),
				FinalNormal.Equals(FVector::UpVector, 64 * DBL_EPSILON));
			TestTrue(Fixture.Name + TEXT(": final height is that face's geometric half-extent"),
				FMath::Abs(Body.State.Location.Z - Fixture.Geometry.Extent[Fixture.ExpectedFaceAxis]) <= 64 * DBL_EPSILON * 120);
		}
		if (Fixture.bDuplicateSupport)
		{
			TestEqual(TEXT("single-provider trajectory reference is complete"), SingleProviderTrace.Num(), FrameCount);
			TestEqual(TEXT("a coincident provider cannot mask a later vertex arrival or change the trajectory"), DuplicateTrajectoryDifferences, 0);
		}
		if (Fixture.bSlide)
		{
			TestEqual(TEXT("face support holds throughout flat-floor sliding"), StableFrames, FrameCount);
			TestTrue(TEXT("height is the selected face half-extent within transform roundoff"), MaxHeightError < 64 * DBL_EPSILON * 120);
			TestTrue(TEXT("zero-spin sliding preserves inclination and heading exactly"), MaxRotationError == 0);
			if (Fixture.Friction == 0)
				TestTrue(TEXT("frictionless sliding preserves canonical tangential velocity exactly"), MaxVelocityError == 0);
			else
			{
				TestTrue(TEXT("friction dissipates translation"), Body.State.Velocity.Size() < InitialVelocity.Size());
				TestTrue(TEXT("friction never reverses the slide"), MinimumForwardSpeed >= 0);
				if (InitialVelocity.Size() <= 100)
					TestTrue(TEXT("Coulomb sliding reaches exact rest without sign reversal"), Body.State.Velocity.IsZero());
				if (InitialVelocity.Size() < 2)
				{
					const FVector ExpectedDisplacement = InitialVelocity * (InitialVelocity.Size() / (2 * Fixture.Friction * 650));
					const FVector ActualDisplacement(Body.State.Location.X, Body.State.Location.Y, 0);
					TestTrue(TEXT("within-frame stop has the analytic Coulomb stopping distance"), ActualDisplacement.Equals(ExpectedDisplacement, 1.e-12));
				}
			}
		}
	}
	TestEqual(TEXT("all requested canonical cells were executed"), ExecutedCells,
		SelectedCell.IsEmpty() ? Cells.Num() : 1);
	return true;
}
#endif
