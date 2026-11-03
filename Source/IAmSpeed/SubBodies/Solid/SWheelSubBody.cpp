#include "SWheelSubBody.h"
#include "IAmSpeed/World/Simulation/Internal/ComponentKinematicRead.h"
#include "IAmSpeed/World/Simulation/RoadSettings/NativeRoadSettingsRead.h"
#include "IAmSpeed/World/Simulation/NativeEpochDiagnostics.h"
// Fill out your copyright notice in the Description page of Project Settings.


#include "Engine/World.h"
#include "IAmSpeed/World/Analytic/AnalyticWorldQuery.h"
#include "CylinderSubBody.h"
#include "CylinderSweep.h"
#include <stdexcept>
#include "IAmSpeed/World/Analytic/StaticWorldQueryAudit.h"
#include "IAmSpeed/World/Analytic/JoinedAnalyticQueryPrefetch1075.h"
#include "IAmSpeed/Components/ISpeedWheeledComponent.h"
#include "IAmSpeed/Base/SpeedConstant.h"
#include "Configs/WheelSubBodyConfig.h"
#include "ChaosVehicleWheel.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "HAL/IConsoleManager.h"

DEFINE_LOG_CATEGORY(WheelSubBodyLog);

static TAutoConsoleVariable<float> CVarSkyLeagueSuspensionCompressionDampingScale(
	TEXT("p.SkyLeague.Suspension.CompressionDampingScale"),
	1.0f,
	TEXT("Scale applied to suspension compression damping."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarSkyLeagueSuspensionCompressionQuadraticDamping(
	TEXT("p.SkyLeague.Suspension.CompressionQuadraticDamping"),
	0.0f,
	TEXT("Additional compression damping coefficient multiplied by absolute wheel-normal velocity. 0 preserves the production model."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarSkyLeagueSuspensionReboundDampingScale(
	TEXT("p.SkyLeague.Suspension.ReboundDampingScale"),
	1.0f,
	TEXT("Scale applied to suspension rebound damping."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarSkyLeagueSuspensionStiffnessScale(
	TEXT("p.SkyLeague.Suspension.StiffnessScale"),
	1.0f,
	TEXT("Scale applied to suspension stiffness force."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarSkyLeagueSuspensionStiffnessReferenceDisplacement(
	TEXT("p.SkyLeague.Suspension.StiffnessReferenceDisplacement"),
	0.0f,
	TEXT("Spring displacement in cm used as the zero point for stiffness force. 0 preserves the production model."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarSkyLeagueSuspensionPreRestCompressionDampingScale(
	TEXT("p.SkyLeague.Suspension.PreRestCompressionDampingScale"),
	0.35f,
	TEXT("Scale applied to damping while a contacted wheel is still above rest height and moving toward compression."),
	ECVF_Default);

static TAutoConsoleVariable<int32> CVarSkyLeagueSuspensionClampPositiveForce(
	TEXT("p.SkyLeague.Suspension.ClampPositiveForce"),
	0,
	TEXT("When non-zero, suspension force is clamped to zero instead of pulling the vehicle toward the surface."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarSkyLeagueSuspensionMaxAttractiveForce(
	TEXT("p.SkyLeague.Suspension.MaxAttractiveForce"),
	250000.0f,
	TEXT("Maximum suspension force magnitude pulling the vehicle toward support. Zero selects the unrestricted signed model."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarSkyLeagueSuspensionMaxAttractiveForceMinNormalZ(
	TEXT("p.SkyLeague.Suspension.MaxAttractiveForceMinNormalZ"),
	-1.0f,
	TEXT("Minimum world-Z support-normal component using MaxAttractiveForce. Set -1 to include every surface."),
	ECVF_Default);

static TAutoConsoleVariable<int32> CVarSkyLeagueSuspensionJumpStateClampsNegativeForce(
	TEXT("p.SkyLeague.Suspension.JumpStateClampsNegativeForce"),
	0,
	TEXT("When non-zero, an explicit unilateral jump-support release cannot pull the vehicle back toward the surface."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarSkyLeagueSuspensionRestingForceScale(
	TEXT("p.SkyLeague.Suspension.RestingForceScale"),
	0.0f,
	TEXT("Scale applied to the suspension resting force before adding spring displacement and damping. 0 restores the old model."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarSkyLeagueSuspensionCeilingFadeStartNormalZ(
	TEXT("p.SkyLeague.Suspension.CeilingFadeStartNormalZ"),
	-0.98f,
	TEXT("Surface-normal Z below which wheel suspension smoothly fades to zero at the ceiling."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarSkyLeagueSuspensionBumpStopStartCompression(
	TEXT("p.SkyLeague.Suspension.BumpStopStartCompression"),
	5.0f,
	TEXT("Spring compression in cm above which the progressive suspension bump stop starts."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarSkyLeagueSuspensionBumpStopRate(
	TEXT("p.SkyLeague.Suspension.BumpStopRate"),
	150000.0f,
	TEXT("Quadratic bump-stop force coefficient in force units per squared cm."),
	ECVF_Default);

static TAutoConsoleVariable<int32> CVarSkyLeagueSuspensionClampNegativeDisplacement(
	TEXT("p.SkyLeague.Suspension.ClampNegativeDisplacement"),
	0,
	TEXT("When non-zero, over-extended suspension displacement is clamped to zero for force computation."),
	ECVF_Default);

static TAutoConsoleVariable<int32> CVarSkyLeagueSuspensionRestingForceRequiresCompression(
	TEXT("p.SkyLeague.Suspension.RestingForceRequiresCompression"),
	0,
	TEXT("When non-zero, suspension resting force is applied only after the spring has positive compression."),
	ECVF_Default);

static TAutoConsoleVariable<int32> CVarSkyLeagueSuspensionSkipOverextendedWheelContact(
	TEXT("p.SkyLeague.Suspension.SkipOverextendedWheelContact"),
	0,
	TEXT("When non-zero, wheel contact velocity solving is skipped while suspension displacement is negative."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarSkyLeagueSuspensionSkipOverextendedWheelContactMinGroundDot(
	TEXT("p.SkyLeague.Suspension.SkipOverextendedWheelContactMinGroundDot"),
	0.95f,
	TEXT("Minimum contact normal dot with world up required to skip over-extended wheel contact solving."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarSkyLeagueSuspensionSkipOverextendedWheelContactMinCarUpDot(
	TEXT("p.SkyLeague.Suspension.SkipOverextendedWheelContactMinCarUpDot"),
	0.95f,
	TEXT("Minimum car up dot with world up required to skip over-extended wheel contact solving."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarSkyLeagueSuspensionSkipOverextendedWheelContactMaxInwardNormalVelocity(
	TEXT("p.SkyLeague.Suspension.SkipOverextendedWheelContactMaxInwardNormalVelocity"),
	40.0f,
	TEXT("Maximum inward normal velocity, in cm/s, allowing over-extended wheel contact solving to be skipped. Faster impacts still use the wheel contact solver."),
	ECVF_Default);

static TAutoConsoleVariable<float> CVarSkyLeagueSuspensionSkipOverextendedWheelContactMaxInwardBodyNormalVelocity(
	TEXT("p.SkyLeague.Suspension.SkipOverextendedWheelContactMaxInwardBodyNormalVelocity"),
	40.0f,
	TEXT("Maximum chassis inward normal velocity, in cm/s, allowing over-extended wheel contact solving to be skipped. This keeps inclined landings from skipping while the car body is still falling fast."),
	ECVF_Default);

static TAutoConsoleVariable<int32> CVarSkyLeagueSuspensionForceDebug(
	TEXT("p.SkyLeague.Suspension.ForceDebug"),
	0,
	TEXT("Logs per-wheel suspension force components when non-zero."),
	ECVF_Default);

namespace
{
    constexpr float MaxSuspensionForce = 5e6f;

    struct FSuspensionForceDebugData
    {
        float RestingForce = 0.0f;
        float StiffnessForce = 0.0f;
        float DampingForce = 0.0f;
        float UnclampedForce = 0.0f;
        float QuantizedForce = 0.0f;
        float WheelRelativeVelocity = 0.0f;
        float Damping = 0.0f;
        float ForceLastDisplacement = 0.0f;
        float ForceCurrentDisplacement = 0.0f;
        bool bCompression = false;
        bool bPreRestCompression = false;
    };

    float QuantizeSuspensionDisplacement(const float Displacement)
    {
        const int32 QuantizedDisplacement = FMath::RoundToInt(Displacement * SpeedConstants::SuspScale);
        return static_cast<float>(QuantizedDisplacement) / SpeedConstants::SuspScale;
    }

    float QuantizeSuspensionForce(const float Force)
    {
        const float ClampedForce = FMath::Clamp(Force, -MaxSuspensionForce, MaxSuspensionForce);
        return static_cast<float>(FMath::RoundToInt(ClampedForce));
    }

    float QuantizeSuspensionDelta(const float Delta)
    {
        const int32 StepRate = FMath::Max(1, FMath::RoundToInt(1.f / FMath::Max(Delta, SMALL_NUMBER)));
        return 1.f / static_cast<float>(StepRate);
    }

    float ComputeQuantizedSuspensionForce(
        const Chaos::FSimpleSuspensionSim& Suspension,
        const float LastDisplacement,
        const float CurrentDisplacement,
        const float WheelRelativeVelocity,
        const bool bForcePositive,
        const bool bIncreasingDisplacementIsCompression,
        const float ConfiguredStiffnessScale,
        const float ConfiguredStiffnessReferenceDisplacement,
        const float ConfiguredPreRestCompressionDampingScale,
        const bool bConfiguredClampNegativeDisplacement,
        const bool bDisableBumpStopForce,
        FSuspensionForceDebugData* DebugData = nullptr)
    {
        const bool bClampNegativeDisplacement = bConfiguredClampNegativeDisplacement ||
            Speed::RoadSettings::ReadOriginalOrLegacy<156>([](){ return CVarSkyLeagueSuspensionClampNegativeDisplacement.GetValueOnAnyThread(); }) != 0;
        const float ForceLastDisplacement = bClampNegativeDisplacement
            ? FMath::Max(0.0f, LastDisplacement)
            : LastDisplacement;
        const float ForceCurrentDisplacement = bClampNegativeDisplacement
            ? FMath::Max(0.0f, CurrentDisplacement)
            : CurrentDisplacement;
        const bool bPreRestCompression = bClampNegativeDisplacement &&
            LastDisplacement < 0.0f &&
            CurrentDisplacement < 0.0f &&
            CurrentDisplacement > LastDisplacement;
        const bool bCompression = bPreRestCompression || (bIncreasingDisplacementIsCompression
            ? ForceCurrentDisplacement > ForceLastDisplacement
            : ForceCurrentDisplacement < ForceLastDisplacement);
        const float DampingScale = bPreRestCompression
            ? (ConfiguredPreRestCompressionDampingScale >= 0.0f
                ? ConfiguredPreRestCompressionDampingScale
                : Speed::RoadSettings::ReadOriginalOrLegacy<167>([](){ return CVarSkyLeagueSuspensionPreRestCompressionDampingScale.GetValueOnAnyThread(); }))
            : (bCompression
                ? Speed::RoadSettings::ReadOriginalOrLegacy<158>([](){ return CVarSkyLeagueSuspensionCompressionDampingScale.GetValueOnAnyThread(); })
                : Speed::RoadSettings::ReadOriginalOrLegacy<168>([](){ return CVarSkyLeagueSuspensionReboundDampingScale.GetValueOnAnyThread(); }));
        const float Damping = (bCompression
            ? Suspension.Setup().CompressionDamping
            : Suspension.Setup().ReboundDamping) * FMath::Max(0.0f, DampingScale);
        const bool bApplyRestingForce = Speed::RoadSettings::ReadOriginalOrLegacy<169>([](){ return CVarSkyLeagueSuspensionRestingForceRequiresCompression.GetValueOnAnyThread(); }) == 0 ||
            ForceCurrentDisplacement > 0.0f;
        const double RestingForce = bApplyRestingForce
            ? static_cast<double>(Suspension.Setup().RestingForce) *
				static_cast<double>(FMath::Max(0.0f,
					Speed::RoadSettings::ReadOriginalOrLegacy<170>([](){ return CVarSkyLeagueSuspensionRestingForceScale.GetValueOnAnyThread(); })))
            : 0.0;
        const double StiffnessReferenceDisplacement = static_cast<double>(
			ConfiguredStiffnessReferenceDisplacement +
			Speed::RoadSettings::ReadOriginalOrLegacy<176>([](){ return CVarSkyLeagueSuspensionStiffnessReferenceDisplacement.GetValueOnAnyThread(); }));
        const double StiffnessForce =
			(static_cast<double>(ForceCurrentDisplacement) - StiffnessReferenceDisplacement) *
			static_cast<double>(Suspension.Setup().SpringRate) *
			static_cast<double>(FMath::Max(0.0f, ConfiguredStiffnessScale)) *
			static_cast<double>(FMath::Max(0.0f,
				Speed::RoadSettings::ReadOriginalOrLegacy<177>([](){ return CVarSkyLeagueSuspensionStiffnessScale.GetValueOnAnyThread(); })));
        const double EffectiveDamping = static_cast<double>(Damping) +
			(bCompression
				? static_cast<double>(FMath::Abs(WheelRelativeVelocity)) *
					static_cast<double>(FMath::Max(0.0f,
						Speed::RoadSettings::ReadOriginalOrLegacy<159>([](){ return CVarSkyLeagueSuspensionCompressionQuadraticDamping.GetValueOnAnyThread(); })))
				: 0.0);
        const double DampingForce = WheelRelativeVelocity * EffectiveDamping;
        float Force = static_cast<float>(RestingForce + StiffnessForce - DampingForce);
		const float BumpStopStart = FMath::Max(0.0f,
			Speed::RoadSettings::ReadOriginalOrLegacy<154>([](){ return CVarSkyLeagueSuspensionBumpStopStartCompression.GetValueOnAnyThread(); }));
		const float BumpStopCompression = FMath::Max(
			0.0f, ForceCurrentDisplacement - BumpStopStart);
		if (!bDisableBumpStopForce)
		{
			Force += FMath::Square(BumpStopCompression) * FMath::Max(
				0.0f, Speed::RoadSettings::ReadOriginalOrLegacy<153>([](){ return CVarSkyLeagueSuspensionBumpStopRate.GetValueOnAnyThread(); }));
		}
        const float UnclampedForce = Force;
        if (bForcePositive || Speed::RoadSettings::ReadOriginalOrLegacy<157>([](){ return CVarSkyLeagueSuspensionClampPositiveForce.GetValueOnAnyThread(); }) != 0)
        {
            Force = FMath::Max(Force, 0.0f);
        }
        const float QuantizedForce = QuantizeSuspensionForce(Force);
        if (DebugData)
        {
            DebugData->RestingForce = static_cast<float>(RestingForce);
            DebugData->StiffnessForce = static_cast<float>(StiffnessForce);
            DebugData->DampingForce = static_cast<float>(DampingForce);
            DebugData->UnclampedForce = UnclampedForce;
            DebugData->QuantizedForce = QuantizedForce;
            DebugData->WheelRelativeVelocity = static_cast<float>(WheelRelativeVelocity);
            DebugData->Damping = static_cast<float>(EffectiveDamping);
            DebugData->ForceLastDisplacement = ForceLastDisplacement;
            DebugData->ForceCurrentDisplacement = ForceCurrentDisplacement;
            DebugData->bCompression = bCompression;
            DebugData->bPreRestCompression = bPreRestCompression;
        }
        return QuantizedForce;
    }

    void SetQuantizedSuspensionLength(Chaos::FSimpleSuspensionSim& Suspension, const float Length, const float WheelRadius)
    {
        const float DisplacementInput = FMath::Max(0.f, Length - Suspension.Setup().RaycastSafetyMargin - WheelRadius);
        const float SpringDisplacement = Suspension.Setup().MaxLength - DisplacementInput;
        const float QuantizedSpringDisplacement = QuantizeSuspensionDisplacement(SpringDisplacement);
        const float QuantizedDisplacementInput = Suspension.Setup().MaxLength - QuantizedSpringDisplacement;
        const float QuantizedLength = QuantizedDisplacementInput + Suspension.Setup().RaycastSafetyMargin + WheelRadius;
        Suspension.SetSuspensionLength(QuantizedLength, WheelRadius);
    }
}

USWheelSubBody::USWheelSubBody(const FObjectInitializer& ObjectInitializer):
	Super(ObjectInitializer)
{
	SubBodyTypeBodyDomainStorage() = ESubBodyType::Wheel;
    IgnoredSubBodyTypesBodyDomainStorage().AddUnique(ESubBodyType::Wheel);
	bApplyRestForceSolidStorage() = false; // wheel apply suspension force directly, so we do not want rest force to interfere with it
    SetMass(WheelMassWheelDomainStorage());
}

void USWheelSubBody::Initialize(ISpeedComponent* InParentComponent)
{
	Super::Initialize(InParentComponent);
	if (InParentComponent)
	{
		ParentComponentBodyDomainStorage() = InParentComponent;
		WheelComponentWheelDomainStorage() = static_cast<ISpeedWheeledComponent*>(InParentComponent);

		if (WheelComponentWheelDomainStorage())
		{
			WheelSubBodyConfig Config = WheelComponentWheelDomainStorage()->GetWheelSubBodyConfig(*this);
            if (Config.bValid)
            {
                SetIdx(Config.WheelIndex);
                ChaosWheelWheelDomainStorage() = Config.ChaosWheel;
                PWheelWheelDomainStorage() = Config.PWheel;
                PSuspensionWheelDomainStorage() = Config.PSuspension;
                // keep consistency with radius and mass
                SetRadius(PWheelWheelDomainStorage()->GetEffectiveRadius());
                PWheelWheelDomainStorage()->SetMassPerWheel(WheelMassWheelDomainStorage());
                SetMass(WheelMassWheelDomainStorage());
                InvInertiaLocalSolidStorage() = InitInvInertiaTensor();
            }
		}
	}
}


void USWheelSubBody::ResetForFrame(const float& Delta)
{
    // CurrentHit = SHitResult(); // do not reset current hit, because netcode needs it to know if wheel is transitionning on wall/ground
    FutureHitStorage() = SHitResult();
    ResetIgnoredComponents();
    bWasOnGroundPrevFrameRuntimeStorage() = IsOnGround();
    bCountedGroundFrameRuntimeStorage() = false;
}

void USWheelSubBody::UpdateKinematicsFromOwner(const SKinematic& ParentKinematic)
{
    Super::UpdateKinematicsFromOwner(ParentKinematic);

    const FVector SuspensionOffset = ParentKinematic.Rotation.RotateVector(
        SpringDisplacement() * FVector::UpVector);
    KinematicsStorage().Location += SuspensionOffset;
    KinematicsStorage().Velocity = ParentKinematic.Velocity + FVector::CrossProduct(
        ParentKinematic.AngularVelocity,
        KinematicsStorage().Location - ParentKinematic.Location);
}

SSphere USWheelSubBody::MakeSphere() const
{
    const SKinematic& KS = GetKinematicState();
    return SSphere(
        KS.Location,
        Radius() + CollisionMargin(),
        KS.Velocity,
        KS.Acceleration);
}

FCollisionShape USWheelSubBody::GetCollisionShape(const float Inflation) const
{
    if (!bUseEffectiveSuspensionSweepRadiusWheelDomainStorage())
    {
        return Super::GetCollisionShape(Inflation);
    }
    return FCollisionShape::MakeSphere(
        FMath::Max(KINDA_SMALL_NUMBER, Radius() + CollisionMargin() + Inflation));
}

void USWheelSubBody::AcceptHit()
{
    // Do nothing; we keep CurrentHit as is (suspension handles it)
    // CurrentHit = FutureHit;
    // Ignore this component for the rest of the frame
    if (CurrentHitStorage().Component.IsValid())
        IgnoredComponentsBodyDomainStorage().AddUnique(CurrentHitStorage().Component.Get());
    else if (FutureHitStorage().Component.IsValid())
        // for safety, ignore FutureHit component too
        IgnoredComponentsBodyDomainStorage().AddUnique(FutureHitStorage().Component.Get());
}

SKinematic USWheelSubBody::GetKinematicsFromOwner(const unsigned int& NumFrame) const
{
    SKinematic WheelKinematics;
    if (!ParentComponentBodyDomainStorage())
        return WheelKinematics;
    SKinematic CarKinematics = ParentComponentBodyDomainStorage()->GetOriginKinematicStateForFrame(NumFrame);
    FTransform ChassisTM(CarKinematics.Rotation, CarKinematics.Location);
    auto WheelPos = WorldPosFromCarTransform(ChassisTM);
    auto WheelVel = ParentComponentBodyDomainStorage()->GetPhysVelocityAtPoint(WheelPos);
    auto WheelAcc = ParentComponentBodyDomainStorage()->GetPhysAccelerationAtPoint(WheelPos);

    WheelKinematics.Location = WheelPos;
    WheelKinematics.Velocity = WheelVel;
    WheelKinematics.Acceleration = WheelAcc;
    return WheelKinematics;
}


// ==================================================
// =============== SweepTOI methods =================
// ==================================================

bool USWheelSubBody::SweepTOI(const float& RemainingDelta, float& OutTOI)
{
    // wheel does not do TOI sweep, it only sweeps for suspension.
    // We could implement it in the future if we want to detect early collision on wheels
    // (currently we rely on sweep suspension to detect collision with ground and walls, 
    // and rely on hitboxes and "real" spheres to detect collision with other cars and spheres which is sufficient for now)
	return false;
}

// ==================================================
// =========== Sweep suspension methods =============
// ==================================================

void USWheelSubBody::SweepSuspension(const float& delta)
{
    bool bHitSphere = SweepSuspensionOnSpheres(CurrentHitStorage(), delta);
    if (bHitSphere)
    {
        return;
    }
    bool bHitBox = SweepSuspensionOnBoxes(CurrentHitStorage(), delta);
    if (bHitBox)
    {
        return;
    }

    if(SweepSuspensionOnCylinders(CurrentHitStorage(),delta))return;
    bool bHitGround = SweepSuspensionOnGround(CurrentHitStorage(), delta);
    if (!bHitGround)
    {
        SetOnGround(false);
        if (WheelComponentWheelDomainStorage())
        {
            WheelComponentWheelDomainStorage()->NotifyWheelOnGroundStateChanged();
        }
    }
}

bool USWheelSubBody::SweepSuspensionAlongNormal(
    const FVector& Normal, const float SearchDistance, const float Delta, SHitResult& OutHit) const
{
    UWorld* World = GetWorld();
    const FVector SweepNormal = Normal.GetSafeNormal();
    if (!World || !ParentComponentBodyDomainStorage() || SweepNormal.IsNearlyZero() || SearchDistance <= 0.0f)
    {
        return false;
    }

    FCollisionQueryParams Params(NAME_None, false);
    Params.bReturnFaceIndex = true;
    Params.bReturnPhysicalMaterial = true;
    if (const AActor* VehicleOwner = GetOwner())
    {
        Params.AddIgnoredActor(VehicleOwner);
    }

    FHitResult UnrealHit;
	const FVector CurrentPos = WorldPos();
	const FVector Start = CurrentPos + SearchDistance * SweepNormal;
	const FVector End = CurrentPos - SearchDistance * SweepNormal;
	bool bHit = false;
	Speed::Analytic::FWorldHit AnalyticHit;
	const bool bUsedAnalyticAuthority =
		Speed::Analytic::FStaticWorldQueryAudit::TryCompactAuthoritySingle(
		World,
		Start, End, KinematicsStorage().Rotation, GetCollisionShape(),
		static_cast<uint8>(GetCollisionChannel()), GetResponseParams(), UnrealHit, bHit,
		&AnalyticHit);
	if (bUsedAnalyticAuthority && !bHit && IsOnGround())
	{
		const SHitResult& PreviousHit = GetHit();
		const FVector PreviousNormal = PreviousHit.ImpactNormal.GetSafeNormal();
		const float InwardSpeed = FVector::DotProduct(
			ParentComponentBodyDomainStorage()->GetPhysCOMVelocity(), PreviousNormal);
		if (!PreviousNormal.IsNearlyZero() && InwardSpeed < 0.0f &&
			PreviousHit.SourceId != 0)
		{
			Speed::Analytic::FStaticWorldQueryAudit::TryCompactAuthoritySingle(
				World, Start, End, KinematicsStorage().Rotation, GetCollisionShape(),
				static_cast<uint8>(GetCollisionChannel()), GetResponseParams(),
				UnrealHit, bHit, &AnalyticHit, PreviousHit.SourceId, 0, 0,
				PreviousNormal, 0.9f, true);
		}
	}
	if (!bUsedAnalyticAuthority)
	{
		Speed::Analytic::FStaticWorldQueryAudit::RecordLegacySweep();
		bHit = World->SweepSingleByChannel(UnrealHit, Start, End,
			KinematicsStorage().Rotation, GetCollisionChannel(), GetCollisionShape(),
			Params, GetResponseParams());
	}
	Speed::Analytic::FStaticWorldQueryAudit::RecordSingle(
		Speed::Analytic::EStaticQuerySite::WheelEstablishedSupportProbe,
		Start, End,
		KinematicsStorage().Rotation, GetCollisionShape(),
		static_cast<uint8>(GetCollisionChannel()), GetResponseParams(), bHit, UnrealHit);
	if (!bHit)
	{
		return false;
    }

	OutHit = bUsedAnalyticAuthority
		? SHitResult::FromAnalyticHit(
			AnalyticHit, Delta, UnrealHit.Component)
		: SHitResult::FromUnrealHit(UnrealHit, Delta);
    OutHit.ImpactNormal = Speed::QuantizeUnitNormal(OutHit.ImpactNormal);
    return true;
}

void USWheelSubBody::GetSuspensionSweepSegment(
    const float Delta, FVector& OutStart, FVector& OutEnd) const
{
    const FTransform ChassisTM(
        ParentComponentBodyDomainStorage()->GetPhysRotation(), ParentComponentBodyDomainStorage()->GetPhysLocation());
    const FVector CarUp = ChassisTM.GetUnitAxis(EAxis::Z);
    const FVector WorldRest = ChassisTM.TransformPosition(GetLocalOffset());
    OutStart = WorldRest + SuspensionMaxRaise() * CarUp;
    const float NewDisplacement = PredictNextDisplacement(Delta);
    OutEnd = WorldPos()
        + (NewDisplacement - SpringDisplacement() - CollisionMargin()) * CarUp;
}

bool USWheelSubBody::TryCaptureGroundQuery1075(
    float Delta, Speed::Analytic::FAnalyticPrefetchQuery1075& Out) const
{
    if (!IsGroundQueryCaptureCertified1075() || !GetWorld() || !ParentComponentBodyDomainStorage() || !PSuspensionWheelDomainStorage())
        return false;
    FVector Start=FVector::ZeroVector, End=FVector::ZeroVector;
    GetSuspensionSweepSegment(Delta, Start, End);
    Out.Consumer.Owner=this;
    Out.Consumer.ExactType=GetClass();
    FMemory::Memcpy(&Out.Consumer.DeltaBits, &Delta, sizeof(Delta));
    Out.MaximumSearchTime=1.0;
    Out.Query=Speed::Analytic::FStaticWorldQueryAudit::MakeStrictSingleQuery1075(
        Start, End, KinematicsStorage().Rotation, GetCollisionShape(),
        static_cast<uint8>(GetCollisionChannel()), GetResponseParams());
    return true;
}

bool USWheelSubBody::ProbeSuspensionOnGround(
    SHitResult& OutHit, const float Delta) const
{
    UWorld* World = GetWorld();
    if (!World || !ParentComponentBodyDomainStorage())
    {
        return false;
    }

    Speed::Analytic::FScopedAnalyticPrefetchConsumer1075 Consumer(this, Delta);
    FVector Start = FVector::ZeroVector;
    FVector End = FVector::ZeroVector;
    GetSuspensionSweepSegment(Delta, Start, End);

    FCollisionQueryParams Params(NAME_None, false);
    Params.bReturnFaceIndex = true;
    Params.bReturnPhysicalMaterial = true;
    if (const AActor* VehicleOwner = GetOwner())
    {
        Params.AddIgnoredActor(VehicleOwner);
    }

    FHitResult UnrealHit;
	bool bHit = false;
	Speed::Analytic::FWorldHit AnalyticHit;
	const bool bUsedAnalyticAuthority =
		Speed::Analytic::FStaticWorldQueryAudit::TryCompactAuthoritySingle(
		World,
		Start, End, KinematicsStorage().Rotation, GetCollisionShape(),
		static_cast<uint8>(GetCollisionChannel()), GetResponseParams(), UnrealHit, bHit,
		&AnalyticHit);
	if (bUsedAnalyticAuthority && !bHit && IsOnGround())
	{
		const SHitResult& PreviousHit = GetHit();
		const FVector PreviousNormal = PreviousHit.ImpactNormal.GetSafeNormal();
		const float InwardSpeed = FVector::DotProduct(
			ParentComponentBodyDomainStorage()->GetPhysCOMVelocity(), PreviousNormal);
		if (!PreviousNormal.IsNearlyZero() && InwardSpeed < 0.0f &&
			PreviousHit.SourceId != 0)
		{
			Speed::Analytic::FStaticWorldQueryAudit::TryCompactAuthoritySingle(
				World, Start, End, KinematicsStorage().Rotation, GetCollisionShape(),
				static_cast<uint8>(GetCollisionChannel()), GetResponseParams(),
				UnrealHit, bHit, &AnalyticHit, PreviousHit.SourceId, 0, 0,
				PreviousNormal, 0.9f, true);
		}
	}
	if (!bUsedAnalyticAuthority)
	{
		Speed::Analytic::FStaticWorldQueryAudit::RecordLegacySweep();
		bHit = World->SweepSingleByChannel(UnrealHit, Start, End,
			KinematicsStorage().Rotation, GetCollisionChannel(), GetCollisionShape(),
			Params, GetResponseParams());
	}
	Speed::Analytic::FStaticWorldQueryAudit::RecordSingle(
		Speed::Analytic::EStaticQuerySite::WheelSuspensionProbe,
		Start, End, KinematicsStorage().Rotation, GetCollisionShape(),
		static_cast<uint8>(GetCollisionChannel()), GetResponseParams(), bHit, UnrealHit);
	if (!bHit)
    {
		if (bUsedAnalyticAuthority && Speed::RoadSettings::ReadOriginalOrLegacy<161>([](){ return CVarSkyLeagueSuspensionForceDebug.GetValueOnAnyThread(); }) != 0)
		{
			SPEED_NATIVE_LOG(WheelSubBodyLog, Warning,
				TEXT("[SuspensionAnalyticMiss] Frame=%d Wheel=%d Start=%s End=%s Hit=%d Time=%.9g Source=%016llX Surface=%016llX Feature=%016llX Primitive=%016llX Group=%016llX Point=%s Normal=%s ErrorCm=%.3f"),
				ParentComponentBodyDomainStorage() ? ParentComponentBodyDomainStorage()->NumFrame() : -1,
				Idx(), *Start.ToString(), *End.ToString(),
				AnalyticHit.bHit ? 1 : 0, AnalyticHit.Time,
				AnalyticHit.SourceId, AnalyticHit.SurfaceId,
				AnalyticHit.FeatureId, AnalyticHit.PrimitiveId,
				AnalyticHit.CanonicalGroupId,
				*AnalyticHit.Point.ToString(), *AnalyticHit.Normal.ToString(),
				AnalyticHit.GeometricErrorBoundCm);
		}
        return false;
    }

	OutHit = bUsedAnalyticAuthority
		? SHitResult::FromAnalyticHit(
			AnalyticHit, Delta, UnrealHit.Component)
		: SHitResult::FromUnrealHit(UnrealHit, Delta);
    OutHit.ImpactNormal = Speed::QuantizeUnitNormal(OutHit.ImpactNormal);
    return true;
}

bool USWheelSubBody::SweepSuspensionOnGround(SHitResult& OutHit, const float& delta)
{
    UWorld* World = GetWorld();
    if (!World || !ParentComponentBodyDomainStorage()) return false;

    FVector CurrentPos = WorldPos();
    const bool bWasOnGround = IsOnGround();

    FTransform ChassisTM(ParentComponentBodyDomainStorage()->GetPhysRotation(), ParentComponentBodyDomainStorage()->GetPhysLocation());
    FVector WorldRestingPos = ChassisTM.TransformPosition(GetLocalOffset());
    FVector CarUpVector = ChassisTM.GetUnitAxis(EAxis::Z);
    FVector Start = FVector::ZeroVector;
    FVector End = FVector::ZeroVector;
    GetSuspensionSweepSegment(delta, Start, End);
    float NewSpringDisplacement = PredictNextDisplacement(delta); // if wheel will be in air next frame
    SHitResult OldCurrentHit = CurrentHitStorage();
    // auto Sphere = SSphere(WorldPos(), Radius(), FVector::ZeroVector, FVector::ZeroVector);
    // Sphere.DrawDebug(GetWorld());

    bool ret = false;
    SHitResult ProbeHit;
    const bool bHasGroundHit = ProbeSuspensionOnGround(ProbeHit, delta);
    if (bHasGroundHit)
    {
        OutHit = ProbeHit;
        SetOnGround(true);
        ret = true;
        auto OldNormal = OldCurrentHit.ImpactNormal;
        auto NewNormal = OutHit.ImpactNormal;
        auto OldCurrentDot = FVector::DotProduct(OldCurrentHit.ImpactNormal, OutHit.ImpactNormal);
        // detect wheel is transitionning from ground to wall or wall to ground
        if (!NewNormal.Equals(OldNormal))
        {
            ParentComponentBodyDomainStorage()->RcvImpactOnSubBody(*this, OutHit.ImpactPoint);
            // UE_LOG(WheelSubBodyLog, Warning, TEXT("[CCDWheel(%d)] [%s] NumFrame=%d, Wheel transitioned between ground/wall contact"),
            //	Idx(), *ParentComponent->GetRole(), ParentComponent->NumFrame());
        }
    }
    else
    {
#if !(UE_BUILD_SHIPPING)
        if (Speed::RoadSettings::ReadOriginalOrLegacy<161>([](){ return CVarSkyLeagueSuspensionForceDebug.GetValueOnAnyThread(); }) != 0)
        {
            SPEED_NATIVE_LOG(WheelSubBodyLog, Log,
                TEXT("[SuspensionSweep] Frame=%d Wheel=%d Result=Miss WasGrounded=%d Start=%s End=%s CurrentPos=%s Rest=%s CarUp=%s SpringDisp=%.3f NewDisp=%.3f MaxRaise=%.3f MaxDrop=%.3f Radius=%.3f PrevHitN=%s PrevHitLoc=%s Vel=%s"),
                ParentComponentBodyDomainStorage()->NumFrame(),
                Idx(),
                bWasOnGround ? 1 : 0,
                *Start.ToString(),
                *End.ToString(),
                *CurrentPos.ToString(),
                *WorldRestingPos.ToString(),
                *CarUpVector.ToString(),
                SpringDisplacement(),
                NewSpringDisplacement,
                SuspensionMaxRaise(),
                SuspensionMaxDrop(),
                Radius(),
                *OldCurrentHit.ImpactNormal.ToString(),
                *OldCurrentHit.Location.ToString(),
                *ParentComponentBodyDomainStorage()->GetPhysCOMVelocity().ToString());
        }
#endif
        SetOnGround(false);
    }
    if (WheelComponentWheelDomainStorage())
    {
        WheelComponentWheelDomainStorage()->NotifyWheelOnGroundStateChanged();
    }
    return ret;
}

bool USWheelSubBody::SweepSuspensionOnSpheres(SHitResult& OutHit,  const float& delta)
{
    // --------------------------------------------------
    // Build suspension sweep segment
    // --------------------------------------------------
    FTransform ChassisTM(ParentComponentBodyDomainStorage()->GetPhysRotation(), ParentComponentBodyDomainStorage()->GetPhysLocation());
    const FVector CarUp = ChassisTM.GetUnitAxis(EAxis::Z);

    const FVector WorldRest = ChassisTM.TransformPosition(GetLocalOffset());
    const FVector Start = WorldRest + SuspensionMaxRaise() * CarUp;

    const float NewDisp = PredictNextDisplacement(delta);
    const FVector End =
        WorldPos() +
        (NewDisp - SpringDisplacement() - CollisionMargin()) * CarUp;

    // --------------------------------------------------
    // Prepare This Sphere
    // --------------------------------------------------
    SSphere ThisSphere(
        Start,
        Radius(),
        FVector::ZeroVector,
        FVector::ZeroVector
    );

    bool bHit = false;
    float BestTOI = delta;

    SHitResult BestHit;
    TWeakObjectPtr<USphereSubBody> BestSphere = nullptr;
    const TArray<TWeakObjectPtr<USphereSubBody>>& OtherSpheres = ExternalSphereSubBodiesBodyDomainStorage();

    for (auto& OtherSphere : OtherSpheres)
    {
        if (!OtherSphere.IsValid()) continue;
        if (OtherSphere->GetOwner() == GetOwner()) continue;

        // Ignore already-hit Spheres this frame
        if (IgnoredComponentsBodyDomainStorage().Contains(OtherSphere.Get()))
            continue;

        SSphere OSphere = OtherSphere->MakeSphere();

        SHitResult Hit = ThisSphere.IntersectDuringMovement(OSphere, Start, End, delta);
        if (!Hit.bHit)
            continue;

        if (Hit.TOI < BestTOI)
        {
            BestTOI = Hit.TOI;
            BestHit = Hit;
            BestSphere = OtherSphere;
            bHit = true;
        }
    }

    if (!bHit)
        return false;

    OutHit = BestHit;
    // Geometric intersections do not carry the selected support component.
    // UpdateSuspension needs it to clamp attraction on non-static supports.
    if(BestSphere.IsValid())
    {
        OutHit.Component = BestSphere;
        OutHit.SubBody = BestSphere;
    }
    else
    {
        OutHit.Component.Reset();
        OutHit.SubBody.Reset();
    }
    OutHit.FrameTag = ParentComponentBodyDomainStorage()->NumFrame();
    OutHit.ImpactNormal = Speed::QuantizeUnitNormal(OutHit.ImpactNormal);
    OutHit.Location = OutHit.ImpactPoint + Radius() * OutHit.ImpactNormal;

    SetOnGround(true);
    if (WheelComponentWheelDomainStorage())
    {
        WheelComponentWheelDomainStorage()->NotifyWheelOnGroundStateChanged();
    }
    // ParentComponent->SetHadImpactThisFrame(true);

    return true;
}

bool USWheelSubBody::SweepSuspensionOnBoxes(SHitResult& OutHit, const float& delta)
{
    // --------------------------------------------------
    // Build suspension sweep segment
    // --------------------------------------------------
    FTransform ChassisTM(ParentComponentBodyDomainStorage()->GetPhysRotation(), ParentComponentBodyDomainStorage()->GetPhysLocation());
    const FVector CarUp = ChassisTM.GetUnitAxis(EAxis::Z);

    const FVector WorldRest = ChassisTM.TransformPosition(GetLocalOffset());
    const FVector Start = WorldRest + SuspensionMaxRaise() * CarUp;

    const float NewDisp = PredictNextDisplacement(delta);
    const FVector End =
        WorldPos() +
        (NewDisp - SpringDisplacement() - CollisionMargin()) * CarUp;

    // --------------------------------------------------
    // Prepare This Sphere
    // --------------------------------------------------
    SSphere ThisSphere(
        Start,
        Radius(),
        FVector::ZeroVector,
        FVector::ZeroVector
    );

    bool bHit = false;
    float BestTOI = delta;

    SHitResult BestHit;
    TWeakObjectPtr<UBoxSubBody> BestBox = nullptr;
	const TArray<TWeakObjectPtr<UBoxSubBody>>& OtherBoxes = ExternalBoxSubBodiesBodyDomainStorage();

    for (auto& OtherBox : OtherBoxes)
    {
        if (!OtherBox.IsValid()) continue;
        if (OtherBox->GetOwner() == GetOwner()) continue;
        // Ignore already-hit Boxes this frame
        if (IgnoredComponentsBodyDomainStorage().Contains(OtherBox.Get()))
            continue;

        SSBox OBox = OtherBox->MakeBox();

        SHitResult Hit = ThisSphere.IntersectDuringMovement(OBox, Start, End, delta);
        if (!Hit.bHit)
            continue;

        if (Hit.TOI < BestTOI)
        {
            BestTOI = Hit.TOI;
            BestHit = Hit;
            BestBox = OtherBox;
            bHit = true;
        }
    }

    if (!bHit)
        return false;

    OutHit = BestHit;
    // Preserve the selected support for suspension and contact classification.
    if(BestBox.IsValid())
    {
        OutHit.Component = BestBox;
        OutHit.SubBody = BestBox;
    }
    else
    {
        OutHit.Component.Reset();
        OutHit.SubBody.Reset();
    }
    OutHit.FrameTag = ParentComponentBodyDomainStorage()->NumFrame();
    OutHit.ImpactNormal = Speed::QuantizeUnitNormal(OutHit.ImpactNormal);
    OutHit.Location = OutHit.ImpactPoint + Radius() * OutHit.ImpactNormal;

    SetOnGround(true);
    if (WheelComponentWheelDomainStorage())
    {
        WheelComponentWheelDomainStorage()->NotifyWheelOnGroundStateChanged();
    }
    // ParentComponent->SetHadImpactThisFrame(true);

    return true;
}

void USWheelSubBody::UpdateSuspension(const float& delta)
{
    if (delta <= KINDA_SMALL_NUMBER)
        return;

    bCrossedIntoCompressionThisFrameRuntimeStorage() = false;

    FTransform ChassisTM(ParentComponentBodyDomainStorage()->GetPhysRotation(), ParentComponentBodyDomainStorage()->GetPhysLocation());
    const FVector Up = ChassisTM.GetUnitAxis(EAxis::Z);
    // update steering angle
    UpdateSteerAngle(delta);

    if (IsOnGround())
    {
        // compute world anchor pos
        FVector SuspensionAnchorWorldPos = ChassisTM.TransformPosition(GetLocalOffset() +
            PSuspensionWheelDomainStorage()->Setup().MaxLength * FVector::UpVector);
        const FVector SuspensionAxis = Up; // Car Up Vector
        const float ProjectedCompression = FVector::DotProduct(SuspensionAnchorWorldPos - CurrentHitStorage().Location, SuspensionAxis);

        // update suspension state
        const float LastDisplacement = QuantizeSuspensionDisplacement(SpringDisplacement());
        float NewDesiredLength = ProjectedCompression + Radius();
        SetQuantizedSuspensionLength(*PSuspensionWheelDomainStorage(), NewDesiredLength, Radius());
        const float SuspensionDelta = QuantizeSuspensionDelta(delta);
        PSuspensionWheelDomainStorage()->Simulate(SuspensionDelta);
        const float CurrentDisplacement = QuantizeSuspensionDisplacement(SpringDisplacement());
        bCrossedIntoCompressionThisFrameRuntimeStorage() = LastDisplacement < 0.0f &&
            CurrentDisplacement >= 0.0f;
        FSuspensionForceDebugData ForceDebugData;
		FVector RelativePointVelocity=ParentComponentBodyDomainStorage()->GetPhysVelocityAtPoint(CurrentHitStorage().ImpactPoint);
        FVector SurfaceVelocity;
        if(TryGetPrescribedSupportVelocity(CurrentHitStorage().ImpactPoint,SurfaceVelocity))RelativePointVelocity-=SurfaceVelocity;
        const float ProjectedVelocity = RelativePointVelocity.Dot(CurrentHitStorage().ImpactNormal);
        SuspensionForceRuntimeStorage() = ComputeQuantizedSuspensionForce(*PSuspensionWheelDomainStorage(), LastDisplacement,
            CurrentDisplacement, ProjectedVelocity, ClampsSuspensionForceToPositive(),
            bUseIncreasingDisplacementAsCompressionWheelDomainStorage(),
            SuspensionStiffnessForceScaleWheelDomainStorage(),
            SuspensionStiffnessReferenceDisplacementWheelDomainStorage(),
            SuspensionPreRestCompressionDampingScaleWheelDomainStorage(),
            bClampNegativeSuspensionDisplacementWheelDomainStorage(),
            bDisableSuspensionBumpStopForceWheelDomainStorage(),
            &ForceDebugData);
		float SuspensionForceOverride = 0.0f;
		if (WheelComponentWheelDomainStorage() && WheelComponentWheelDomainStorage()->TryComputeWheelSuspensionForceOverride(
			*this, LastDisplacement, CurrentDisplacement, ProjectedVelocity,
			SuspensionForceOverride))
		{
			SuspensionForceRuntimeStorage() = QuantizeSuspensionForce(SuspensionForceOverride);
		}
        PSuspensionWheelDomainStorage()->SetLastDisplacement(CurrentDisplacement);

		const float Dot = FVector::DotProduct(CurrentHitStorage().ImpactNormal, FVector::UpVector);
		// Keep full support through both gutters. Only fade at the final ceiling
		// approach so gravity can release the car without letting the chassis
		// bottom out throughout the upper transition.
		const float CeilingFadeStart = FMath::Clamp(
			Speed::RoadSettings::ReadOriginalOrLegacy<155>([](){ return CVarSkyLeagueSuspensionCeilingFadeStartNormalZ.GetValueOnAnyThread(); }),
			-0.999f, 0.0f);
		const float CeilingFadeAlpha = FMath::GetMappedRangeValueClamped(
			FVector2D(-1.0f, CeilingFadeStart), FVector2D(0.0f, 1.0f), Dot);
		const float SuspensionScale = FMath::SmoothStep(0.0f, 1.0f, CeilingFadeAlpha);

        // apply suspension scale
        SuspensionForceRuntimeStorage() *= SuspensionScale;
        const float SuspensionMaxValue = MaxSuspensionForce;
        /*if (FMath::Abs(SuspensionForce) > SuspensionMaxValue)
        {
            UE_LOG(WheelSubBodyLog, Warning, TEXT("[Suspension] In frame %d, Suspension is way too strong!!!! For Wheel num %d SuspensionForce = %f"),
                ParentComponent->NumFrame(), Idx(), SuspensionForce);
        }*/
        // A jumping wheel may keep a sweep hit while its suspension extends,
        // but it must no longer pull the chassis back toward that support.
        const bool bClampJumpForce =
            Speed::RoadSettings::ReadOriginalOrLegacy<163>([](){ return CVarSkyLeagueSuspensionJumpStateClampsNegativeForce.GetValueOnAnyThread(); }) != 0 &&
            HasJumpUnilateralSupport();
        const bool bClampPositive = bClampJumpForce || ClampsSuspensionForceToPositive() ||
            Speed::RoadSettings::ReadOriginalOrLegacy<157>([](){ return CVarSkyLeagueSuspensionClampPositiveForce.GetValueOnAnyThread(); }) != 0;
        const float MaxAttractiveForce = FMath::Max(
            0.0f, Speed::RoadSettings::ReadOriginalOrLegacy<164>([](){ return CVarSkyLeagueSuspensionMaxAttractiveForce.GetValueOnAnyThread(); }));
        const float MaxAttractionMinNormalZ = FMath::Clamp(
            Speed::RoadSettings::ReadOriginalOrLegacy<165>([](){ return CVarSkyLeagueSuspensionMaxAttractiveForceMinNormalZ.GetValueOnAnyThread(); }),
            -1.0f, 1.0f);
        const bool bCappedAttractionSupport = FVector::DotProduct(
            CurrentHitStorage().ImpactNormal.GetSafeNormal(), FVector::UpVector)
            >= MaxAttractionMinNormalZ;
        const float MinSuspensionForce = bClampPositive
            ? 0.0f
            : (bCappedAttractionSupport && MaxAttractiveForce > 0.0f
                ? -FMath::Min(MaxAttractiveForce, SuspensionMaxValue)
                : -SuspensionMaxValue);
        SuspensionForceRuntimeStorage() = FMath::Clamp(
            SuspensionForceRuntimeStorage(), MinSuspensionForce, SuspensionMaxValue);
		// Special Actor case: if we do not hit the ground but an other actor, we keep suspension force only for positive values
        // to avoid wheel to stick to it too much and create unrealistic behavior.
        if (CurrentHitStorage().Component.IsValid() &&
            CurrentHitStorage().Component->Mobility !=
            EComponentMobility::Static)
        {
            SuspensionForceRuntimeStorage() = FMath::Max(0.0f, SuspensionForceRuntimeStorage());
        }
        SuspensionForceRuntimeStorage() = QuantizeSuspensionForce(SuspensionForceRuntimeStorage());
#if !(UE_BUILD_SHIPPING)
        if (Speed::RoadSettings::ReadOriginalOrLegacy<161>([](){ return CVarSkyLeagueSuspensionForceDebug.GetValueOnAnyThread(); }) != 0)
        {
            const FVector ParentVelocity = ParentComponentBodyDomainStorage() ? ParentComponentBodyDomainStorage()->GetPhysCOMVelocity() : FVector::ZeroVector;
            SPEED_NATIVE_LOG(WheelSubBodyLog, Log,
                TEXT("[SuspensionForce] Frame=%d Wheel=%d Dt=%.5f SuspDt=%.5f HitTOI=%.5f PenDepth=%.3f LastDisp=%.3f CurDisp=%.3f ForceLastDisp=%.3f ForceCurDisp=%.3f SpringLen=%.3f NewDesiredLen=%.3f ProjectedCompression=%.3f Rest=%.1f Stiff=%.1f Damp=%.1f WheelRelativeVelocity=%.3f Damping=%.1f Compress=%d PreRestCompress=%d Raw=%.1f Quant=%.1f Scale=%.3f Final=%.1f VelZ=%.3f HitN=%s HitLoc=%s"),
                ParentComponentBodyDomainStorage() ? ParentComponentBodyDomainStorage()->NumFrame() : INDEX_NONE,
                Idx(),
                delta,
                SuspensionDelta,
                CurrentHitStorage().TOI,
                CurrentHitStorage().PenetrationDepth,
                LastDisplacement,
                CurrentDisplacement,
                ForceDebugData.ForceLastDisplacement,
                ForceDebugData.ForceCurrentDisplacement,
                SpringLength(),
                NewDesiredLength,
                ProjectedCompression,
                ForceDebugData.RestingForce,
                ForceDebugData.StiffnessForce,
                ForceDebugData.DampingForce,
                ForceDebugData.WheelRelativeVelocity,
                ForceDebugData.Damping,
                ForceDebugData.bCompression ? 1 : 0,
                ForceDebugData.bPreRestCompression ? 1 : 0,
                ForceDebugData.UnclampedForce,
                ForceDebugData.QuantizedForce,
                SuspensionScale,
                SuspensionForceRuntimeStorage(),
                ParentVelocity.Z,
                *CurrentHitStorage().ImpactNormal.ToString(),
                *CurrentHitStorage().Location.ToString());
        }
#endif
    }
    else
    {
        SuspensionForceRuntimeStorage() = 0.0;

        const float LastDisplacement = QuantizeSuspensionDisplacement(SpringDisplacement());
        const float NextAirLength = ComputeNextAirLength(delta);
        SetQuantizedSuspensionLength(*PSuspensionWheelDomainStorage(), NextAirLength, Radius());
        PWheelWheelDomainStorage()->SetWheelLoadForce(0.f);
        PSuspensionWheelDomainStorage()->Simulate(QuantizeSuspensionDelta(delta));
        PSuspensionWheelDomainStorage()->SetLastDisplacement(QuantizeSuspensionDisplacement(SpringDisplacement()));
#if !(UE_BUILD_SHIPPING)
        if (Speed::RoadSettings::ReadOriginalOrLegacy<161>([](){ return CVarSkyLeagueSuspensionForceDebug.GetValueOnAnyThread(); }) != 0)
        {
            const float CurrentDisplacement = QuantizeSuspensionDisplacement(SpringDisplacement());
            const FVector ParentVelocity = ParentComponentBodyDomainStorage() ? ParentComponentBodyDomainStorage()->GetPhysCOMVelocity() : FVector::ZeroVector;
            const FVector ParentUp = ParentComponentBodyDomainStorage() ? ParentComponentBodyDomainStorage()->GetPhysUpVector().GetSafeNormal() : FVector::UpVector;
            SPEED_NATIVE_LOG(WheelSubBodyLog, Log,
                TEXT("[SuspensionAir] Frame=%d Wheel=%d Dt=%.5f LastDisp=%.3f CurDisp=%.3f NextAirLength=%.3f SpringLen=%.3f WorldPos=%s CarUp=%s Vel=%s PrevHitN=%s PrevHitLoc=%s"),
                ParentComponentBodyDomainStorage() ? ParentComponentBodyDomainStorage()->NumFrame() : INDEX_NONE,
                Idx(),
                delta,
                LastDisplacement,
                CurrentDisplacement,
                NextAirLength,
                SpringLength(),
                *WorldPos().ToString(),
                *ParentUp.ToString(),
                *ParentVelocity.ToString(),
                *CurrentHitStorage().ImpactNormal.ToString(),
                *CurrentHitStorage().Location.ToString());
        }
#endif
    }
}



/*SSphere USWheelSubBody::MakeSphere(const unsigned int& NumFrame, const float& RemainingDelta, const float& TimePassed) const
{
    // --- 1) Car kinematics ---
    SKinematic CarKS0 = Speed::NativeKinematics::FComponentRead::ForFrame(*ParentComponent, NumFrame);
    SKinematic CarKS1 = CarKS0.Integrate(TimePassed);
    SKinematic CarKS2 = CarKS1.Integrate(RemainingDelta);

    // --- 2) Chassis transforms ---
    const FTransform TM1(CarKS1.Rotation, CarKS1.Location);
    const FTransform TM2(CarKS2.Rotation, CarKS2.Location);

    const FVector Up1 = TM1.GetUnitAxis(EAxis::Z);
    const FVector Up2 = TM2.GetUnitAxis(EAxis::Z);

    // --- 3) Wheel positions ---
    const float Disp1 = PredictNextDisplacement(TimePassed);
    const float Disp2 = PredictNextDisplacement(TimePassed + RemainingDelta);

    const FVector Start =
        WorldPosFromCarTransform(TM1) +
        (Disp1 - SpringDisplacement()) * Up1;

    const FVector End =
        WorldPosFromCarTransform(TM2) +
        (Disp2 - SpringDisplacement()) * Up2;

    // --- 4) Wheel velocity at t = TimePassed ---
    const FVector r1 = Start - CarKS1.Location;
    const FVector v1 = CarKS1.Velocity + FVector::CrossProduct(CarKS1.AngularVelocity, r1);

    // --- 5) Constant acceleration matching Start -> End ---
    FVector a = FVector::ZeroVector;
    if (RemainingDelta > KINDA_SMALL_NUMBER)
    {
        a = 2.f * (End - Start - v1 * RemainingDelta)
            / (RemainingDelta * RemainingDelta);
    }

    // --- 6) Build sphere ---
    return SSphere(
        Start,
        Radius(),
        v1,
        a
    );
}*/

void USWheelSubBody::ApplyImpulse(const FVector& LinearImpulse, const FVector& WorldPoint)
{
    if (WheelComponentWheelDomainStorage())
    {
        const FVector ContactNormal = CurrentHitStorage().ImpactNormal.GetSafeNormal();
        const float ContactGroundDot = FVector::DotProduct(ContactNormal, FVector::UpVector);
        const float CarUpGroundDot = FVector::DotProduct(WheelComponentWheelDomainStorage()->GetPhysUpVector().GetSafeNormal(), FVector::UpVector);
        FVector VelAtPoint = WheelComponentWheelDomainStorage()->GetPhysVelocityAtPoint(WorldPoint);
        FVector BodyVelocity=WheelComponentWheelDomainStorage()->GetPhysCOMVelocity(),SurfaceVelocity;
        if(TryGetPrescribedSupportVelocity(WorldPoint,SurfaceVelocity)){VelAtPoint-=SurfaceVelocity;BodyVelocity-=SurfaceVelocity;}
        const float vN = FVector::DotProduct(VelAtPoint, ContactNormal);
        const float BodyVN = FVector::DotProduct(BodyVelocity, ContactNormal);
        const float SkipMaxInwardNormalVelocity = FMath::Max(0.0f, Speed::RoadSettings::ReadOriginalOrLegacy<173>([](){ return CVarSkyLeagueSuspensionSkipOverextendedWheelContactMaxInwardNormalVelocity.GetValueOnAnyThread(); }));
        const float SkipMaxInwardBodyNormalVelocity = FMath::Max(0.0f, Speed::RoadSettings::ReadOriginalOrLegacy<172>([](){ return CVarSkyLeagueSuspensionSkipOverextendedWheelContactMaxInwardBodyNormalVelocity.GetValueOnAnyThread(); }));
        if (Speed::RoadSettings::ReadOriginalOrLegacy<171>([](){ return CVarSkyLeagueSuspensionSkipOverextendedWheelContact.GetValueOnAnyThread(); }) != 0 &&
            QuantizeSuspensionDisplacement(SpringDisplacement()) < 0.0f &&
            ContactGroundDot >= Speed::RoadSettings::ReadOriginalOrLegacy<175>([](){ return CVarSkyLeagueSuspensionSkipOverextendedWheelContactMinGroundDot.GetValueOnAnyThread(); }) &&
            CarUpGroundDot >= Speed::RoadSettings::ReadOriginalOrLegacy<174>([](){ return CVarSkyLeagueSuspensionSkipOverextendedWheelContactMinCarUpDot.GetValueOnAnyThread(); }) &&
            vN >= -SkipMaxInwardNormalVelocity &&
            BodyVN >= -SkipMaxInwardBodyNormalVelocity)
        {
#if !(UE_BUILD_SHIPPING)
            if (Speed::RoadSettings::ReadOriginalOrLegacy<161>([](){ return CVarSkyLeagueSuspensionForceDebug.GetValueOnAnyThread(); }) != 0)
            {
                SPEED_NATIVE_LOG(WheelSubBodyLog, Log,
                    TEXT("[SuspensionForce] Frame=%d Wheel=%d SkipOverextendedWheelContact Disp=%.3f ContactGroundDot=%.3f CarUpGroundDot=%.3f vN=%.3f BodyVN=%.3f VelZ=%.3f"),
                    WheelComponentWheelDomainStorage()->NumFrame(),
                    Idx(),
                    QuantizeSuspensionDisplacement(SpringDisplacement()),
                    ContactGroundDot,
                    CarUpGroundDot,
                    vN,
                    BodyVN,
                    WheelComponentWheelDomainStorage()->GetPhysCOMVelocity().Z);
            }
#endif
            return;
        }

        FVector N = ContactNormal;
        UpdateContactVelocityLock();

        const FVector ContactPoint = GetContactImpulseApplicationPoint();
        const FVector r = ContactPoint - WheelComponentWheelDomainStorage()->GetPhysCOM();
        const FMatrix InvI = WheelComponentWheelDomainStorage()->ComputeWorldInvInertiaTensor();
        const FVector rxn = FVector::CrossProduct(r, N);
        const FVector term = FVector::CrossProduct(
            InvI.TransformVector(rxn), r);

        const float invMassEff =
            (1.f / WheelComponentWheelDomainStorage()->GetPhysMass()) + FVector::DotProduct(N, term);

        // UE_LOG(WheelSubBodyLog, Log, TEXT("[CCDWheel(%d)] RegisterWheelGroundContact at NumFrame=%d"),
        //    Idx(), WheelComponent->NumFrame());
        WheelComponentWheelDomainStorage()->RegisterWheelGroundContact({
            this,
            CurrentHitStorage().Component,
            CurrentHitStorage().ImpactPoint,
            CurrentHitStorage().FaceIndex,
			CurrentHitStorage().SourceId,
			CurrentHitStorage().SurfaceId,
			CurrentHitStorage().FeatureId,
            ContactPoint,
            N,
            r,
            vN,
            invMassEff,
            QuantizeSuspensionDisplacement(SpringDisplacement()),
            IsContactVelocityLocked(),
            !bUseCompressionCrossingContactImpulseWheelDomainStorage() && !WasOnGroundPreviousFrame(),
            IsAtSuspensionBumpStop()
            });
	}
}

float USWheelSubBody::PredictNextDisplacement(const float& delta) const
{
    float DisplacementInput = ComputeNextAirLength(delta) - PSuspensionWheelDomainStorage()->Setup().RaycastSafetyMargin - Radius();
    DisplacementInput = FMath::Max(0.f, DisplacementInput);
    return MaxLength() - DisplacementInput;
}

float USWheelSubBody::ComputeNextAirLength(const float& DeltaTime) const
{
    return MaxLength() + SuspensionMaxDrop() + Radius();
}


void USWheelSubBody::HandleTimers()
{
    if (bIsJumpingRuntimeStorage() > 0)
    {
        bIsJumpingRuntimeStorage()--;
    }
}

void USWheelSubBody::UpdatePhysicsState(const float& delta)
{
    // --- 1. Base axes from chassis ---
    const FVector CarUp = ParentComponentBodyDomainStorage()->GetPhysUpVector();
    const FVector CarFwd = ParentComponentBodyDomainStorage()->GetPhysForwardVector();
    const FVector CarRight = ParentComponentBodyDomainStorage()->GetPhysRightVector();

    UpAxisRuntimeStorage() = CarUp; // stays constant

    // --- 2. Compute steering rotation around UpAxis ---
    // SteeringAngle in radians
    FQuat SteerQuat(UpAxisRuntimeStorage(), SteeringAngleRuntimeStorage());

    // --- 3. Apply steering to forward axis ---
    ForwardAxisRuntimeStorage() = SteerQuat.RotateVector(CarFwd).GetSafeNormal();

    // --- 4. Right axis = orthogonal ---
    RightAxisRuntimeStorage() = FVector::CrossProduct(UpAxisRuntimeStorage(), ForwardAxisRuntimeStorage()).GetSafeNormal();

    // --- 5. Update rolling angle (rotation around RightAxis) ---
    // update omega
    OmegaRuntimeStorage() = ParentComponentBodyDomainStorage()->GetPhysForwardSpeed() / FMath::Max(Radius(), KINDA_SMALL_NUMBER);
    float angVel = AngularVelocity(); // rad/s
    RollAngleRuntimeStorage() += angVel * delta;
    RollAngleRuntimeStorage() = FMath::Fmod(RollAngleRuntimeStorage(), 2.f * PI);

    // --- 6. World position from suspension ---
    FVector pos = WorldPos();

    // --- 7. Fill render data struct for the visual thread ---
    RenderDataRuntimeStorage().WorldPosition = pos;
    RenderDataRuntimeStorage().UpAxis = UpAxisRuntimeStorage();
    RenderDataRuntimeStorage().ForwardAxis = ForwardAxisRuntimeStorage();
    RenderDataRuntimeStorage().RightAxis = RightAxisRuntimeStorage();

    RenderDataRuntimeStorage().SpringOffset = SuspensionRestLength() - SpringLength();

    RenderDataRuntimeStorage().SteerRotation = FQuat(UpAxisRuntimeStorage(), SteeringAngleRuntimeStorage());
    RenderDataRuntimeStorage().RollRotation = FQuat(RightAxisRuntimeStorage(), RollAngleRuntimeStorage());
}

void USWheelSubBody::UpdateSteerAngle(const float& delta)
{
    float steerInput = 0.0;
    if (WheelComponentWheelDomainStorage())
    {
		steerInput = WheelComponentWheelDomainStorage()->GetPhysSteeringInput(); // expected to be in range [-1, 1]
    }

    if (IsSteeringEnabled())
    {
        float speed = ParentComponentBodyDomainStorage()->GetPhysCOMVelocity().Size();

        // saturation max speed
        float normSpeed = FMath::Clamp(speed / ParentComponentBodyDomainStorage()->GetPhysMaxSpeed(), 0.f, 1.f);

        // steering reduction curve
        float steerFactor =
            1.0f
            - 0.85f * normSpeed;  // 1.0 at low speeds, ~0.25 at max speed

        if (speed < 300.f)
            steerFactor *= 1.2f; // +20%

        // final angle
        SteeringAngleRuntimeStorage() = FMath::DegreesToRadians(MaxSteeringAngle() * steerInput * steerFactor);
    }
    else
    {
        SteeringAngleRuntimeStorage() = 0.0f;
    }
}

FVector USWheelSubBody::WorldPos() const
{
    FTransform ChassisTM(ParentComponentBodyDomainStorage()->GetPhysRotation(), ParentComponentBodyDomainStorage()->GetPhysLocation());
    return WorldPosFromCarTransform(ChassisTM);
}

FVector USWheelSubBody::SteeringPos() const
{
    const FVector ChassisUp = ParentComponentBodyDomainStorage()->GetPhysUpVector();

    // Compute world position of the wheel
    const FVector WheelPosWS = WorldPos();

    // Get the world center of mass of the vehicle
    const FVector CoMWorld = ParentComponentBodyDomainStorage()->GetPhysCOM();

    // Compute the vertical distance between the wheel and the center of mass
    const float VerticalDist = FVector::DotProduct((WheelPosWS - CoMWorld), ChassisUp);

    // Project the wheel onto the horizontal plane of the CoM
    const FVector SteeringPosWS = WheelPosWS - VerticalDist * ChassisUp;

    return SteeringPosWS;
}

FVector USWheelSubBody::WorldPosFromCarTransform(const FTransform& CarTransform) const
{
	return CarTransform.TransformPosition(GetLocalOffset() +
        SpringDisplacement() * FVector::UpVector);
}

FVector USWheelSubBody::GetSuspensionDirectionWS() const
{
    return -ParentComponentBodyDomainStorage()->GetPhysUpVector();
}

float USWheelSubBody::Radius() const
{
    // IAmSpeed models gameplay wheels as spheres. The serialized sub-body
    // radius is therefore authoritative; the Chaos wheel class only supplies
    // legacy vehicle/suspension setup and may intentionally use disk geometry.
    return GetRadius();
}

float USWheelSubBody::ChaosEffectiveRadius() const
{
    return PWheelWheelDomainStorage() ? PWheelWheelDomainStorage()->GetEffectiveRadius() : 0.0f;
}

float USWheelSubBody::AngularVelocity() const
{
    return OmegaRuntimeStorage();
}

void USWheelSubBody::SetAngularVelocity(const float& InOmega)
{
    OmegaRuntimeStorage() = InOmega;
}

float USWheelSubBody::SpringDisplacement() const
{
    return PSuspensionWheelDomainStorage() ? PSuspensionWheelDomainStorage()->GetLastDisplacement() : 0.0f;
}

float USWheelSubBody::ContactSpringDisplacement() const
{
    if (!PSuspensionWheelDomainStorage() || !ParentComponentBodyDomainStorage() || !IsOnGround())
    {
        return SpringDisplacement();
    }

    return ContactSpringDisplacement(CurrentHitStorage());
}

float USWheelSubBody::ContactSpringDisplacement(const SHitResult& ContactHit) const
{
    if (!PSuspensionWheelDomainStorage() || !ParentComponentBodyDomainStorage())
    {
        return SpringDisplacement();
    }
    const FTransform ChassisTM(
        ParentComponentBodyDomainStorage()->GetPhysRotation(), ParentComponentBodyDomainStorage()->GetPhysLocation());
    const FVector Up = ChassisTM.GetUnitAxis(EAxis::Z);
    const FVector SuspensionAnchor = ChassisTM.TransformPosition(
        GetLocalOffset() + PSuspensionWheelDomainStorage()->Setup().MaxLength * FVector::UpVector);
    const float ProjectedCompression = FVector::DotProduct(
        SuspensionAnchor - ContactHit.Location, Up);
    const float DisplacementInput = FMath::Max(
        0.0f, ProjectedCompression - PSuspensionWheelDomainStorage()->Setup().RaycastSafetyMargin);
    return QuantizeSuspensionDisplacement(
        PSuspensionWheelDomainStorage()->Setup().MaxLength - DisplacementInput);
}
float USWheelSubBody::SpringLength() const
{
    return PSuspensionWheelDomainStorage()->GetSpringLength();
}

float USWheelSubBody::GetLastDisplacement() const
{
    return PSuspensionWheelDomainStorage()->GetLastDisplacement();
}

void USWheelSubBody::SetLastDisplacement(const float& displacement)
{
    PSuspensionWheelDomainStorage()->SetLastDisplacement(displacement);
}

float USWheelSubBody::MaxLength() const
{
    return PSuspensionWheelDomainStorage()->Setup().MaxLength;
}

float USWheelSubBody::SuspensionRestLength() const
{
	return PhysicalSuspensionRestLength();
}

float USWheelSubBody::PhysicalSuspensionRestLength() const {return ChaosWheelWheelDomainStorage()->SuspensionMaxRaise;}

float USWheelSubBody::SuspensionMaxRaise() const
{
    return SuspensionMaxRaiseOverrideWheelDomainStorage() >= 0.0f
        ? SuspensionMaxRaiseOverrideWheelDomainStorage()
        : ChaosWheelWheelDomainStorage()->SuspensionMaxRaise;
}

float USWheelSubBody::SuspensionMaxDrop() const
{
    return SuspensionMaxDropOverrideWheelDomainStorage() >= 0.0f
        ? SuspensionMaxDropOverrideWheelDomainStorage()
        : ChaosWheelWheelDomainStorage()->SuspensionMaxDrop;
}

void USWheelSubBody::ConfigureSuspensionTravel(const float MaxRaise, const float MaxDrop)
{
    SuspensionMaxRaiseOverrideWheelDomainStorage() = FMath::Max(0.0f, MaxRaise);
    SuspensionMaxDropOverrideWheelDomainStorage() = FMath::Max(0.0f, MaxDrop);
    if (ChaosWheelWheelDomainStorage())
    {
        ChaosWheelWheelDomainStorage()->SuspensionMaxRaise = SuspensionMaxRaiseOverrideWheelDomainStorage();
        ChaosWheelWheelDomainStorage()->SuspensionMaxDrop = SuspensionMaxDropOverrideWheelDomainStorage();
    }
}

bool USWheelSubBody::HasConfiguredSuspensionTravel() const
{
    return SuspensionMaxRaiseOverrideWheelDomainStorage() >= 0.0f && SuspensionMaxDropOverrideWheelDomainStorage() >= 0.0f;
}

float USWheelSubBody::SuspensionSpringRateCm() const
{
    return bUseSuspensionForceModelOverrideWheelDomainStorage()
        ? SuspensionSpringRateCmOverrideWheelDomainStorage()
        : SuspensionSpringRateWheelDomainStorage() * 100.0f;
}

bool USWheelSubBody::IsAtSuspensionBumpStop() const
{
    if (bUseCompressionCrossingContactImpulseWheelDomainStorage())
    {
        return bCrossedIntoCompressionThisFrameRuntimeStorage();
    }
    return ContactSpringDisplacement() >= FMath::Max(
        0.0f, Speed::RoadSettings::ReadOriginalOrLegacy<154>([](){ return CVarSkyLeagueSuspensionBumpStopStartCompression.GetValueOnAnyThread(); }));
}

float USWheelSubBody::StaticSpringCompression() const
{
    return ConfiguredStaticSpringCompressionWheelDomainStorage();
}

void USWheelSubBody::ConfigureStaticSpringCompression(const float RestingForce)
{
    const float SpringRate = SuspensionSpringRateCm();
    ConfiguredStaticSpringCompressionWheelDomainStorage() = SpringRate > SMALL_NUMBER
        ? QuantizeSuspensionDisplacement(RestingForce / SpringRate)
        : 0.0f;
}

float USWheelSubBody::SuspensionCompressionDamping() const
{
    return SuspensionCompressionDampingOverrideWheelDomainStorage();
}

float USWheelSubBody::SuspensionReboundDamping() const
{
    return SuspensionReboundDampingOverrideWheelDomainStorage();
}

bool USWheelSubBody::UsesDirectSuspensionDamping() const
{
    return bUseSuspensionForceModelOverrideWheelDomainStorage();
}

bool USWheelSubBody::ClampsSuspensionForceToPositive() const
{
    return bUseSuspensionForceModelOverrideWheelDomainStorage() &&
		(bClampSuspensionForceToPositiveWheelDomainStorage() ||
			(bClampPositiveUntilFirstCompressionWheelDomainStorage() && !bContactVelocityLockedRuntimeStorage()));
}

void USWheelSubBody::ConfigureSuspensionForceModel(const float SpringRateCm,
    const float CompressionDamping, const float ReboundDamping,
	const bool bClampForceToPositive)
{
    bUseSuspensionForceModelOverrideWheelDomainStorage() = true;
    bClampSuspensionForceToPositiveWheelDomainStorage() = bClampForceToPositive;
    SuspensionSpringRateCmOverrideWheelDomainStorage() = FMath::Max(0.0f, SpringRateCm);
    SuspensionCompressionDampingOverrideWheelDomainStorage() = FMath::Max(0.0f, CompressionDamping);
    SuspensionReboundDampingOverrideWheelDomainStorage() = FMath::Max(0.0f, ReboundDamping);
}

void USWheelSubBody::SetUseEffectiveSuspensionSweepRadius(const bool bEnabled)
{
    bUseEffectiveSuspensionSweepRadiusWheelDomainStorage() = bEnabled;
}

void USWheelSubBody::SetUseIncreasingDisplacementAsCompression(const bool bEnabled)
{
    bUseIncreasingDisplacementAsCompressionWheelDomainStorage() = bEnabled;
}

void USWheelSubBody::SetClampPositiveUntilFirstCompression(const bool bEnabled)
{
    bClampPositiveUntilFirstCompressionWheelDomainStorage() = bEnabled;
}

void USWheelSubBody::SetUseCompressionCrossingContactImpulse(const bool bEnabled)
{
	bUseCompressionCrossingContactImpulseWheelDomainStorage() = bEnabled;
}

void USWheelSubBody::SetGroundForceApplicationMode(const uint8 Mode)
{
	GroundForceApplicationModeWheelDomainStorage() = FMath::Min<uint8>(Mode, 3);
	ContactImpulseApplicationModeWheelDomainStorage() = GroundForceApplicationModeWheelDomainStorage();
}

void USWheelSubBody::SetContactImpulseApplicationMode(const uint8 Mode)
{
	ContactImpulseApplicationModeWheelDomainStorage() = FMath::Min<uint8>(Mode, 3);
}

FVector USWheelSubBody::GetSuspensionForceApplicationPoint() const
{
	switch (GroundForceApplicationModeWheelDomainStorage())
	{
	case 2:
		return WorldPos();
	case 3:
		return WorldPos() - SpringDisplacement() * ParentComponentBodyDomainStorage()->GetPhysUpVector();
	default:
		return CurrentHitStorage().ImpactPoint;
	}
}

FVector USWheelSubBody::GetContactImpulseApplicationPoint() const
{
	switch (ContactImpulseApplicationModeWheelDomainStorage())
	{
	case 1:
		return CurrentHitStorage().ImpactPoint;
	case 3:
		return WorldPos() - SpringDisplacement() * ParentComponentBodyDomainStorage()->GetPhysUpVector();
	default:
		return WorldPos();
	}
}

void USWheelSubBody::ConfigureSuspensionForceBehavior(const float StiffnessScale,
    const float StiffnessReferenceDisplacement,
    const float PreRestCompressionDampingScale,
    const bool bClampNegativeDisplacement,
    const bool bDisableBumpStopForce)
{
    SuspensionStiffnessForceScaleWheelDomainStorage() = FMath::Max(0.0f, StiffnessScale);
    SuspensionStiffnessReferenceDisplacementWheelDomainStorage() = StiffnessReferenceDisplacement;
    SuspensionPreRestCompressionDampingScaleWheelDomainStorage() = FMath::Max(
        0.0f, PreRestCompressionDampingScale);
    bClampNegativeSuspensionDisplacementWheelDomainStorage() = bClampNegativeDisplacement;
    bDisableSuspensionBumpStopForceWheelDomainStorage() = bDisableBumpStopForce;
}

float USWheelSubBody::GetSuspensionDampingReboundRatio() const
{
    return SuspensionDampingReboundRatioWheelDomainStorage();
}

float USWheelSubBody::GetSuspensionDampingCompressionRatio() const
{
    return SuspensionDampingCompressionRatioWheelDomainStorage();
}

float USWheelSubBody::GetSuspensionForce() const
{
    return SuspensionForceRuntimeStorage();
}

bool USWheelSubBody::IsContactVelocityLocked() const
{
    return bContactVelocityLockedRuntimeStorage();
}

bool USWheelSubBody::WasOnGroundPreviousFrame() const
{
    return bWasOnGroundPrevFrameRuntimeStorage();
}

bool USWheelSubBody::IsOnGround() const
{
	if(PWheelWheelDomainStorage() == nullptr)
	{
		return false;
	}
    return PWheelWheelDomainStorage()->InContact();
}

bool USWheelSubBody::IsOnStaticObject() const
{
    const UPrimitiveComponent* Support = CurrentHitStorage().Component.Get();
    return IsOnGround() && CurrentHitStorage().bHit && IsValid(Support)
        && Support->GetMobility() == EComponentMobility::Static;
}

bool USWheelSubBody::IsOnDynamicObject() const
{
    const UPrimitiveComponent* Support = CurrentHitStorage().Component.Get();
    return IsOnGround() && CurrentHitStorage().bHit && IsValid(Support)
        && Support->GetMobility() == EComponentMobility::Movable;
}

void USWheelSubBody::SetOnGround(const bool& on_ground)
{
    bool bWasOnGround = IsOnGround();

    if (!bWasOnGround && on_ground)
    {
        const float NormalSpeed = FVector::DotProduct(ParentComponentBodyDomainStorage()->GetPhysVelocityAtPoint(WorldPos()), CurrentHitStorage().ImpactNormal);

        // Real impact
        if (NormalSpeed < -5.0f) // cm/s, to tune
        {
			ParentComponentBodyDomainStorage()->RcvImpactOnSubBody(*this, CurrentHitStorage().ImpactPoint);
        }
    }

    // RegisterWheelContact
    if (on_ground)
    {
        if (!bCountedGroundFrameRuntimeStorage())
        {
            ConsecutiveGroundFramesRuntimeStorage() = FMath::Min<uint8>(ConsecutiveGroundFramesRuntimeStorage() + 1, 31);
            bCountedGroundFrameRuntimeStorage() = true;
        }
        ApplyImpulse(FVector::UpVector, CurrentHitStorage().ImpactPoint);
    }
    else
    {
        ConsecutiveGroundFramesRuntimeStorage() = 0;
        ResetContactVelocityLock();
    }

    PWheelWheelDomainStorage()->SetOnGround(on_ground);
}

void USWheelSubBody::SetIsJumping(const uint8 NbFrames)
{
    bIsJumpingRuntimeStorage() = NbFrames;
    if (NbFrames > 0)
    {
        ConsecutiveGroundFramesRuntimeStorage() = 0;
        ResetContactVelocityLock();
    }
}

uint8 USWheelSubBody::IsJumping() const
{
    return bIsJumpingRuntimeStorage();
}

void USWheelSubBody::SetJumpUnilateralSupport(const bool bEnabled)
{
    bJumpUnilateralSupportRuntimeStorage() = bEnabled;
}

bool USWheelSubBody::HasJumpUnilateralSupport() const
{
    return bJumpUnilateralSupportRuntimeStorage();
}

uint8 USWheelSubBody::GetConsecutiveGroundFrames() const
{
    return ConsecutiveGroundFramesRuntimeStorage();
}

void USWheelSubBody::SetConsecutiveGroundFrames(const uint8 Frames)
{
    ConsecutiveGroundFramesRuntimeStorage() = FMath::Min<uint8>(Frames, 31);
}

void USWheelSubBody::UpdateContactVelocityLock()
{
    if (QuantizeSuspensionDisplacement(SpringDisplacement()) >= 0.0f)
    {
        bContactVelocityLockedRuntimeStorage() = true;
    }
}

void USWheelSubBody::ResetContactVelocityLock()
{
    bContactVelocityLockedRuntimeStorage() = false;
}

bool USWheelSubBody::IsSteeringEnabled() const
{
    return PWheelWheelDomainStorage()->SteeringEnabled;
}

float USWheelSubBody::MaxSteeringAngle() const
{
    return PWheelWheelDomainStorage()->MaxSteeringAngle;
}

float USWheelSubBody::GetSuspensionOffset() const
{
    return SpringDisplacement();
}

FVector USWheelSubBody::GetHitContactNormal() const
{
    if (IsOnGround())
    {
        return CurrentHitStorage().ImpactNormal;
    }
    else
    {
        return FVector::ZeroVector;
    }
}

FVector USWheelSubBody::GetHitContactPoint() const
{
    if (IsOnGround())
    {
        return CurrentHitStorage().ImpactPoint;
    }

    return WorldPos();
}

void USWheelSubBody::SetHitContactNormal(const FVector& ImpactNormal)
{
    CurrentHitStorage().ImpactNormal = ImpactNormal;
}

const SWheelRenderData& USWheelSubBody::GetRenderData() const
{
    return RenderDataRuntimeStorage();
}

void USWheelSubBody::SetRollAngle(const float& angle)
{
    FRotator RollRot(RenderDataRuntimeStorage().RollRotation);
    RollRot.Roll = FMath::RadiansToDegrees(angle);
    RenderDataRuntimeStorage().RollRotation = FQuat(RollRot);
}

float USWheelSubBody::GetRollAngle() const
{
    FRotator RollRot(RenderDataRuntimeStorage().RollRotation);
    return FMath::DegreesToRadians(RollRot.Roll);
}

void USWheelSubBody::SetChaosWheel(UChaosVehicleWheel* InChaosWheel)
{
    ChaosWheelWheelDomainStorage() = InChaosWheel;
    if (ChaosWheelWheelDomainStorage() && SuspensionMaxRaiseOverrideWheelDomainStorage() >= 0.0f
        && SuspensionMaxDropOverrideWheelDomainStorage() >= 0.0f)
    {
        ChaosWheelWheelDomainStorage()->SuspensionMaxRaise = SuspensionMaxRaiseOverrideWheelDomainStorage();
        ChaosWheelWheelDomainStorage()->SuspensionMaxDrop = SuspensionMaxDropOverrideWheelDomainStorage();
    }
}

void USWheelSubBody::SetWheelSim(Chaos::FSimpleWheelSim* InPWheel)
{
	PWheelWheelDomainStorage() = InPWheel;
	if (!PWheelWheelDomainStorage())
	{
		return;
	}
	PWheelWheelDomainStorage()->SetMassPerWheel(WheelMassWheelDomainStorage());
    SetMass(WheelMassWheelDomainStorage());
}

void USWheelSubBody::SetSuspensionSim(Chaos::FSimpleSuspensionSim* InPSuspension)
{
    PSuspensionWheelDomainStorage() = InPSuspension;
	// Print parameters for debug
	// UE_LOG(WheelSubBodyLog, Warning, TEXT("WheelSubBody %d Suspension parameters: MaxLength= %.2f, SpringRate= %.2f, CompressionDamping= %.2f, ReboundDamping= %.2f"),
    //    Idx(), PSuspension->Setup().MaxLength, PSuspension->Setup().SpringRate, PSuspension->Setup().CompressionDamping, PSuspension->Setup().ReboundDamping);
}

void USWheelSubBody::SetLocalOffset(const FVector& InLocalOffset)
{
    SetPhysicalLocalOffset(InLocalOffset);
}
