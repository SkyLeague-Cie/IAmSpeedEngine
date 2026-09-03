// Fill out your copyright notice in the Description page of Project Settings.


#include "SWheelSubBody.h"
#include "IAmSpeed/World/Analytic/StaticWorldQueryAudit.h"
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
            CVarSkyLeagueSuspensionClampNegativeDisplacement.GetValueOnAnyThread() != 0;
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
                : CVarSkyLeagueSuspensionPreRestCompressionDampingScale.GetValueOnAnyThread())
            : (bCompression
                ? CVarSkyLeagueSuspensionCompressionDampingScale.GetValueOnAnyThread()
                : CVarSkyLeagueSuspensionReboundDampingScale.GetValueOnAnyThread());
        const float Damping = (bCompression
            ? Suspension.Setup().CompressionDamping
            : Suspension.Setup().ReboundDamping) * FMath::Max(0.0f, DampingScale);
        const bool bApplyRestingForce = CVarSkyLeagueSuspensionRestingForceRequiresCompression.GetValueOnAnyThread() == 0 ||
            ForceCurrentDisplacement > 0.0f;
        const double RestingForce = bApplyRestingForce
            ? static_cast<double>(Suspension.Setup().RestingForce) *
				static_cast<double>(FMath::Max(0.0f,
					CVarSkyLeagueSuspensionRestingForceScale.GetValueOnAnyThread()))
            : 0.0;
        const double StiffnessReferenceDisplacement = static_cast<double>(
			ConfiguredStiffnessReferenceDisplacement +
			CVarSkyLeagueSuspensionStiffnessReferenceDisplacement.GetValueOnAnyThread());
        const double StiffnessForce =
			(static_cast<double>(ForceCurrentDisplacement) - StiffnessReferenceDisplacement) *
			static_cast<double>(Suspension.Setup().SpringRate) *
			static_cast<double>(FMath::Max(0.0f, ConfiguredStiffnessScale)) *
			static_cast<double>(FMath::Max(0.0f,
				CVarSkyLeagueSuspensionStiffnessScale.GetValueOnAnyThread()));
        const double EffectiveDamping = static_cast<double>(Damping) +
			(bCompression
				? static_cast<double>(FMath::Abs(WheelRelativeVelocity)) *
					static_cast<double>(FMath::Max(0.0f,
						CVarSkyLeagueSuspensionCompressionQuadraticDamping.GetValueOnAnyThread()))
				: 0.0);
        const double DampingForce = WheelRelativeVelocity * EffectiveDamping;
        float Force = static_cast<float>(RestingForce + StiffnessForce - DampingForce);
		const float BumpStopStart = FMath::Max(0.0f,
			CVarSkyLeagueSuspensionBumpStopStartCompression.GetValueOnAnyThread());
		const float BumpStopCompression = FMath::Max(
			0.0f, ForceCurrentDisplacement - BumpStopStart);
		if (!bDisableBumpStopForce)
		{
			Force += FMath::Square(BumpStopCompression) * FMath::Max(
				0.0f, CVarSkyLeagueSuspensionBumpStopRate.GetValueOnAnyThread());
		}
        const float UnclampedForce = Force;
        if (bForcePositive || CVarSkyLeagueSuspensionClampPositiveForce.GetValueOnAnyThread() != 0)
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
	SubBodyType = ESubBodyType::Wheel;
    IgnoredSubBodyTypes.AddUnique(ESubBodyType::Wheel);
	bApplyRestForce = false; // wheel apply suspension force directly, so we do not want rest force to interfere with it
    SetMass(WheelMass);
}

void USWheelSubBody::Initialize(ISpeedComponent* InParentComponent)
{
	Super::Initialize(InParentComponent);
	if (InParentComponent)
	{
		ParentComponent = InParentComponent;
		WheelComponent = static_cast<ISpeedWheeledComponent*>(InParentComponent);

		if (WheelComponent)
		{
			WheelSubBodyConfig Config = WheelComponent->GetWheelSubBodyConfig(*this);
            if (Config.bValid)
            {
                SetIdx(Config.WheelIndex);
                ChaosWheel = Config.ChaosWheel;
                PWheel = Config.PWheel;
                PSuspension = Config.PSuspension;
                // keep consistency with radius and mass
                SetRadius(PWheel->GetEffectiveRadius());
                PWheel->SetMassPerWheel(WheelMass);
                SetMass(WheelMass);
                InvInertiaLocal = InitInvInertiaTensor();
            }
		}
	}
}


void USWheelSubBody::ResetForFrame(const float& Delta)
{
    // CurrentHit = SHitResult(); // do not reset current hit, because netcode needs it to know if wheel is transitionning on wall/ground
    FutureHit = SHitResult();
    IgnoredComponents.Empty();
    bWasOnGroundPrevFrame = IsOnGround();
    bCountedGroundFrame = false;
	IgnoredComponents.Append(AlwaysIgnoredComponents);
}

void USWheelSubBody::UpdateKinematicsFromOwner(const SKinematic& ParentKinematic)
{
    Super::UpdateKinematicsFromOwner(ParentKinematic);

    const FVector SuspensionOffset = ParentKinematic.Rotation.RotateVector(
        SpringDisplacement() * FVector::UpVector);
    Kinematics.Location += SuspensionOffset;
    Kinematics.Velocity = ParentKinematic.Velocity + FVector::CrossProduct(
        ParentKinematic.AngularVelocity,
        Kinematics.Location - ParentKinematic.Location);
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
    if (!bUseEffectiveSuspensionSweepRadius)
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
    if (CurrentHit.Component.IsValid())
        IgnoredComponents.AddUnique(CurrentHit.Component.Get());
    else if (FutureHit.Component.IsValid())
        // for safety, ignore FutureHit component too
        IgnoredComponents.AddUnique(FutureHit.Component.Get());
}

SKinematic USWheelSubBody::GetKinematicsFromOwner(const unsigned int& NumFrame) const
{
    SKinematic WheelKinematics;
    if (!ParentComponent)
        return WheelKinematics;
    SKinematic CarKinematics = ParentComponent->GetOriginKinematicStateForFrame(NumFrame);
    FTransform ChassisTM(CarKinematics.Rotation, CarKinematics.Location);
    auto WheelPos = WorldPosFromCarTransform(ChassisTM);
    auto WheelVel = ParentComponent->GetPhysVelocityAtPoint(WheelPos);
    auto WheelAcc = ParentComponent->GetPhysAccelerationAtPoint(WheelPos);

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
    bool bHitSphere = SweepSuspensionOnSpheres(CurrentHit, delta);
    if (bHitSphere)
    {
        return;
    }
    bool bHitBox = SweepSuspensionOnBoxes(CurrentHit, delta);
    if (bHitBox)
    {
        return;
    }

    bool bHitGround = SweepSuspensionOnGround(CurrentHit, delta);
    if (!bHitGround)
    {
        SetOnGround(false);
        if (WheelComponent)
        {
            WheelComponent->NotifyWheelOnGroundStateChanged();
        }
    }
}

bool USWheelSubBody::SweepSuspensionAlongNormal(
    const FVector& Normal, const float SearchDistance, const float Delta, SHitResult& OutHit) const
{
    UWorld* World = GetWorld();
    const FVector SweepNormal = Normal.GetSafeNormal();
    if (!World || !ParentComponent || SweepNormal.IsNearlyZero() || SearchDistance <= 0.0f)
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
		Start, End, Kinematics.Rotation, GetCollisionShape(),
		static_cast<uint8>(GetCollisionChannel()), GetResponseParams(), UnrealHit, bHit,
		&AnalyticHit);
	if (bUsedAnalyticAuthority && !bHit && IsOnGround())
	{
		const SHitResult& PreviousHit = GetHit();
		const FVector PreviousNormal = PreviousHit.ImpactNormal.GetSafeNormal();
		const float InwardSpeed = FVector::DotProduct(
			ParentComponent->GetPhysCOMVelocity(), PreviousNormal);
		if (!PreviousNormal.IsNearlyZero() && InwardSpeed < 0.0f &&
			PreviousHit.SourceId != 0)
		{
			Speed::Analytic::FStaticWorldQueryAudit::TryCompactAuthoritySingle(
				World, Start, End, Kinematics.Rotation, GetCollisionShape(),
				static_cast<uint8>(GetCollisionChannel()), GetResponseParams(),
				UnrealHit, bHit, &AnalyticHit, PreviousHit.SourceId, 0, 0,
				PreviousNormal, 0.9f, true);
		}
	}
	if (!bUsedAnalyticAuthority)
	{
		Speed::Analytic::FStaticWorldQueryAudit::RecordLegacySweep();
		bHit = World->SweepSingleByChannel(UnrealHit, Start, End,
			Kinematics.Rotation, GetCollisionChannel(), GetCollisionShape(),
			Params, GetResponseParams());
	}
	Speed::Analytic::FStaticWorldQueryAudit::RecordSingle(
		Speed::Analytic::EStaticQuerySite::WheelEstablishedSupportProbe,
		Start, End,
		Kinematics.Rotation, GetCollisionShape(),
		static_cast<uint8>(GetCollisionChannel()), GetResponseParams(), bHit, UnrealHit);
	if (!bHit)
	{
		return false;
    }

	OutHit = bUsedAnalyticAuthority
		? SHitResult::FromAnalyticHit(
			AnalyticHit, Delta, UnrealHit.Component.Get())
		: SHitResult::FromUnrealHit(UnrealHit, Delta);
    OutHit.ImpactNormal = Speed::QuantizeUnitNormal(OutHit.ImpactNormal);
    return true;
}

void USWheelSubBody::GetSuspensionSweepSegment(
    const float Delta, FVector& OutStart, FVector& OutEnd) const
{
    const FTransform ChassisTM(
        ParentComponent->GetPhysRotation(), ParentComponent->GetPhysLocation());
    const FVector CarUp = ChassisTM.GetUnitAxis(EAxis::Z);
    const FVector WorldRest = ChassisTM.TransformPosition(GetLocalOffset());
    OutStart = WorldRest + SuspensionMaxRaise() * CarUp;
    const float NewDisplacement = PredictNextDisplacement(Delta);
    OutEnd = WorldPos()
        + (NewDisplacement - SpringDisplacement() - CollisionMargin()) * CarUp;
}

bool USWheelSubBody::ProbeSuspensionOnGround(
    SHitResult& OutHit, const float Delta) const
{
    UWorld* World = GetWorld();
    if (!World || !ParentComponent)
    {
        return false;
    }

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
		Start, End, Kinematics.Rotation, GetCollisionShape(),
		static_cast<uint8>(GetCollisionChannel()), GetResponseParams(), UnrealHit, bHit,
		&AnalyticHit);
	if (bUsedAnalyticAuthority && !bHit && IsOnGround())
	{
		const SHitResult& PreviousHit = GetHit();
		const FVector PreviousNormal = PreviousHit.ImpactNormal.GetSafeNormal();
		const float InwardSpeed = FVector::DotProduct(
			ParentComponent->GetPhysCOMVelocity(), PreviousNormal);
		if (!PreviousNormal.IsNearlyZero() && InwardSpeed < 0.0f &&
			PreviousHit.SourceId != 0)
		{
			Speed::Analytic::FStaticWorldQueryAudit::TryCompactAuthoritySingle(
				World, Start, End, Kinematics.Rotation, GetCollisionShape(),
				static_cast<uint8>(GetCollisionChannel()), GetResponseParams(),
				UnrealHit, bHit, &AnalyticHit, PreviousHit.SourceId, 0, 0,
				PreviousNormal, 0.9f, true);
		}
	}
	if (!bUsedAnalyticAuthority)
	{
		Speed::Analytic::FStaticWorldQueryAudit::RecordLegacySweep();
		bHit = World->SweepSingleByChannel(UnrealHit, Start, End,
			Kinematics.Rotation, GetCollisionChannel(), GetCollisionShape(),
			Params, GetResponseParams());
	}
	Speed::Analytic::FStaticWorldQueryAudit::RecordSingle(
		Speed::Analytic::EStaticQuerySite::WheelSuspensionProbe,
		Start, End, Kinematics.Rotation, GetCollisionShape(),
		static_cast<uint8>(GetCollisionChannel()), GetResponseParams(), bHit, UnrealHit);
	if (!bHit)
    {
		if (bUsedAnalyticAuthority && CVarSkyLeagueSuspensionForceDebug.GetValueOnAnyThread() != 0)
		{
			UE_LOG(WheelSubBodyLog, Warning,
				TEXT("[SuspensionAnalyticMiss] Frame=%d Wheel=%d Start=%s End=%s Hit=%d Time=%.9g Source=%016llX Surface=%016llX Feature=%016llX Primitive=%016llX Group=%016llX Point=%s Normal=%s ErrorCm=%.3f"),
				ParentComponent ? ParentComponent->NumFrame() : -1,
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
			AnalyticHit, Delta, UnrealHit.Component.Get())
		: SHitResult::FromUnrealHit(UnrealHit, Delta);
    OutHit.ImpactNormal = Speed::QuantizeUnitNormal(OutHit.ImpactNormal);
    return true;
}

bool USWheelSubBody::SweepSuspensionOnGround(SHitResult& OutHit, const float& delta)
{
    UWorld* World = GetWorld();
    if (!World || !ParentComponent) return false;

    FVector CurrentPos = WorldPos();
    const bool bWasOnGround = IsOnGround();

    FTransform ChassisTM(ParentComponent->GetPhysRotation(), ParentComponent->GetPhysLocation());
    FVector WorldRestingPos = ChassisTM.TransformPosition(GetLocalOffset());
    FVector CarUpVector = ChassisTM.GetUnitAxis(EAxis::Z);
    FVector Start = FVector::ZeroVector;
    FVector End = FVector::ZeroVector;
    GetSuspensionSweepSegment(delta, Start, End);
    float NewSpringDisplacement = PredictNextDisplacement(delta); // if wheel will be in air next frame
    SHitResult OldCurrentHit = CurrentHit;
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
            ParentComponent->RcvImpactOnSubBody(*this, OutHit.ImpactPoint);
            // UE_LOG(WheelSubBodyLog, Warning, TEXT("[CCDWheel(%d)] [%s] NumFrame=%d, Wheel transitioned between ground/wall contact"),
            //	Idx(), *ParentComponent->GetRole(), ParentComponent->NumFrame());
        }
    }
    else
    {
#if !(UE_BUILD_SHIPPING)
        if (CVarSkyLeagueSuspensionForceDebug.GetValueOnAnyThread() != 0)
        {
            UE_LOG(WheelSubBodyLog, Log,
                TEXT("[SuspensionSweep] Frame=%d Wheel=%d Result=Miss WasGrounded=%d Start=%s End=%s CurrentPos=%s Rest=%s CarUp=%s SpringDisp=%.3f NewDisp=%.3f MaxRaise=%.3f MaxDrop=%.3f Radius=%.3f PrevHitN=%s PrevHitLoc=%s Vel=%s"),
                ParentComponent->NumFrame(),
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
                *ParentComponent->GetPhysCOMVelocity().ToString());
        }
#endif
        SetOnGround(false);
    }
    if (WheelComponent)
    {
        WheelComponent->NotifyWheelOnGroundStateChanged();
    }
    return ret;
}

bool USWheelSubBody::SweepSuspensionOnSpheres(SHitResult& OutHit,  const float& delta)
{
    // --------------------------------------------------
    // Build suspension sweep segment
    // --------------------------------------------------
    FTransform ChassisTM(ParentComponent->GetPhysRotation(), ParentComponent->GetPhysLocation());
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
    const TArray<TWeakObjectPtr<USphereSubBody>>& OtherSpheres = ExternalSphereSubBodies;

    for (auto& OtherSphere : OtherSpheres)
    {
        if (!OtherSphere.IsValid()) continue;
        if (OtherSphere->GetOwner() == GetOwner()) continue;

        // Ignore already-hit Spheres this frame
        if (IgnoredComponents.Contains(OtherSphere.Get()))
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
    OutHit.ImpactNormal = Speed::QuantizeUnitNormal(OutHit.ImpactNormal);
    OutHit.Location = OutHit.ImpactPoint + Radius() * OutHit.ImpactNormal;

    SetOnGround(true);
    if (WheelComponent)
    {
        WheelComponent->NotifyWheelOnGroundStateChanged();
    }
    // ParentComponent->SetHadImpactThisFrame(true);

    return true;
}

bool USWheelSubBody::SweepSuspensionOnBoxes(SHitResult& OutHit, const float& delta)
{
    // --------------------------------------------------
    // Build suspension sweep segment
    // --------------------------------------------------
    FTransform ChassisTM(ParentComponent->GetPhysRotation(), ParentComponent->GetPhysLocation());
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
	const TArray<TWeakObjectPtr<UBoxSubBody>>& OtherBoxes = ExternalBoxSubBodies;

    for (auto& OtherBox : OtherBoxes)
    {
        if (!OtherBox.IsValid()) continue;
        if (OtherBox->GetOwner() == GetOwner()) continue;
        // Ignore already-hit Boxes this frame
        if (IgnoredComponents.Contains(OtherBox.Get()))
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
    OutHit.ImpactNormal = Speed::QuantizeUnitNormal(OutHit.ImpactNormal);
    OutHit.Location = OutHit.ImpactPoint + Radius() * OutHit.ImpactNormal;

    SetOnGround(true);
    if (WheelComponent)
    {
        WheelComponent->NotifyWheelOnGroundStateChanged();
    }
    // ParentComponent->SetHadImpactThisFrame(true);

    return true;
}

void USWheelSubBody::UpdateSuspension(const float& delta)
{
    if (delta <= KINDA_SMALL_NUMBER)
        return;

    bCrossedIntoCompressionThisFrame = false;

    FTransform ChassisTM(ParentComponent->GetPhysRotation(), ParentComponent->GetPhysLocation());
    const FVector Up = ChassisTM.GetUnitAxis(EAxis::Z);
    // update steering angle
    UpdateSteerAngle(delta);

    if (IsOnGround())
    {
        // compute world anchor pos
        FVector SuspensionAnchorWorldPos = ChassisTM.TransformPosition(GetLocalOffset() +
            PSuspension->Setup().MaxLength * FVector::UpVector);
        const FVector SuspensionAxis = Up; // Car Up Vector
        const float ProjectedCompression = FVector::DotProduct(SuspensionAnchorWorldPos - CurrentHit.Location, SuspensionAxis);

        // update suspension state
        const float LastDisplacement = QuantizeSuspensionDisplacement(SpringDisplacement());
        float NewDesiredLength = ProjectedCompression + Radius();
        SetQuantizedSuspensionLength(*PSuspension, NewDesiredLength, Radius());
        const float SuspensionDelta = QuantizeSuspensionDelta(delta);
        PSuspension->Simulate(SuspensionDelta);
        const float CurrentDisplacement = QuantizeSuspensionDisplacement(SpringDisplacement());
        bCrossedIntoCompressionThisFrame = LastDisplacement < 0.0f &&
            CurrentDisplacement >= 0.0f;
        FSuspensionForceDebugData ForceDebugData;
		const float ProjectedVelocity = ParentComponent->GetPhysVelocityAtPoint(CurrentHit.ImpactPoint).Dot(CurrentHit.ImpactNormal);
        SuspensionForce = ComputeQuantizedSuspensionForce(*PSuspension, LastDisplacement,
            CurrentDisplacement, ProjectedVelocity, ClampsSuspensionForceToPositive(),
            bUseIncreasingDisplacementAsCompression,
            SuspensionStiffnessForceScale,
            SuspensionStiffnessReferenceDisplacement,
            SuspensionPreRestCompressionDampingScale,
            bClampNegativeSuspensionDisplacement,
            bDisableSuspensionBumpStopForce,
            &ForceDebugData);
		float SuspensionForceOverride = 0.0f;
		if (WheelComponent && WheelComponent->TryComputeWheelSuspensionForceOverride(
			*this, LastDisplacement, CurrentDisplacement, ProjectedVelocity,
			SuspensionForceOverride))
		{
			SuspensionForce = QuantizeSuspensionForce(SuspensionForceOverride);
		}
        PSuspension->SetLastDisplacement(CurrentDisplacement);

		const float Dot = FVector::DotProduct(CurrentHit.ImpactNormal, FVector::UpVector);
		// Keep full support through both gutters. Only fade at the final ceiling
		// approach so gravity can release the car without letting the chassis
		// bottom out throughout the upper transition.
		const float CeilingFadeStart = FMath::Clamp(
			CVarSkyLeagueSuspensionCeilingFadeStartNormalZ.GetValueOnAnyThread(),
			-0.999f, 0.0f);
		const float CeilingFadeAlpha = FMath::GetMappedRangeValueClamped(
			FVector2D(-1.0f, CeilingFadeStart), FVector2D(0.0f, 1.0f), Dot);
		const float SuspensionScale = FMath::SmoothStep(0.0f, 1.0f, CeilingFadeAlpha);

        // apply suspension scale
        SuspensionForce *= SuspensionScale;
        const float SuspensionMaxValue = MaxSuspensionForce;
        /*if (FMath::Abs(SuspensionForce) > SuspensionMaxValue)
        {
            UE_LOG(WheelSubBodyLog, Warning, TEXT("[Suspension] In frame %d, Suspension is way too strong!!!! For Wheel num %d SuspensionForce = %f"),
                ParentComponent->NumFrame(), Idx(), SuspensionForce);
        }*/
        // A jumping wheel may keep a sweep hit while its suspension extends,
        // but it must no longer pull the chassis back toward that support.
        const bool bClampJumpForce =
            CVarSkyLeagueSuspensionJumpStateClampsNegativeForce.GetValueOnAnyThread() != 0 &&
            HasJumpUnilateralSupport();
        const bool bClampPositive = bClampJumpForce || ClampsSuspensionForceToPositive() ||
            CVarSkyLeagueSuspensionClampPositiveForce.GetValueOnAnyThread() != 0;
        const float MaxAttractiveForce = FMath::Max(
            0.0f, CVarSkyLeagueSuspensionMaxAttractiveForce.GetValueOnAnyThread());
        const float MaxAttractionMinNormalZ = FMath::Clamp(
            CVarSkyLeagueSuspensionMaxAttractiveForceMinNormalZ.GetValueOnAnyThread(),
            -1.0f, 1.0f);
        const bool bCappedAttractionSupport = FVector::DotProduct(
            CurrentHit.ImpactNormal.GetSafeNormal(), FVector::UpVector)
            >= MaxAttractionMinNormalZ;
        const float MinSuspensionForce = bClampPositive
            ? 0.0f
            : (bCappedAttractionSupport && MaxAttractiveForce > 0.0f
                ? -FMath::Min(MaxAttractiveForce, SuspensionMaxValue)
                : -SuspensionMaxValue);
        SuspensionForce = FMath::Clamp(
            SuspensionForce, MinSuspensionForce, SuspensionMaxValue);
		// Special Actor case: if we do not hit the ground but an other actor, we keep suspension force only for positive values
        // to avoid wheel to stick to it too much and create unrealistic behavior.
        if (CurrentHit.Component.IsValid() &&
            CurrentHit.Component->Mobility !=
            EComponentMobility::Static)
        {
            SuspensionForce = FMath::Max(0.0f, SuspensionForce);
        }
        SuspensionForce = QuantizeSuspensionForce(SuspensionForce);
#if !(UE_BUILD_SHIPPING)
        if (CVarSkyLeagueSuspensionForceDebug.GetValueOnAnyThread() != 0)
        {
            const FVector ParentVelocity = ParentComponent ? ParentComponent->GetPhysCOMVelocity() : FVector::ZeroVector;
            UE_LOG(WheelSubBodyLog, Log,
                TEXT("[SuspensionForce] Frame=%d Wheel=%d Dt=%.5f SuspDt=%.5f HitTOI=%.5f PenDepth=%.3f LastDisp=%.3f CurDisp=%.3f ForceLastDisp=%.3f ForceCurDisp=%.3f SpringLen=%.3f NewDesiredLen=%.3f ProjectedCompression=%.3f Rest=%.1f Stiff=%.1f Damp=%.1f WheelRelativeVelocity=%.3f Damping=%.1f Compress=%d PreRestCompress=%d Raw=%.1f Quant=%.1f Scale=%.3f Final=%.1f VelZ=%.3f HitN=%s HitLoc=%s"),
                ParentComponent ? ParentComponent->NumFrame() : INDEX_NONE,
                Idx(),
                delta,
                SuspensionDelta,
                CurrentHit.TOI,
                CurrentHit.PenetrationDepth,
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
                SuspensionForce,
                ParentVelocity.Z,
                *CurrentHit.ImpactNormal.ToString(),
                *CurrentHit.Location.ToString());
        }
#endif
    }
    else
    {
        SuspensionForce = 0.0;

        const float LastDisplacement = QuantizeSuspensionDisplacement(SpringDisplacement());
        const float NextAirLength = ComputeNextAirLength(delta);
        SetQuantizedSuspensionLength(*PSuspension, NextAirLength, Radius());
        PWheel->SetWheelLoadForce(0.f);
        PSuspension->Simulate(QuantizeSuspensionDelta(delta));
        PSuspension->SetLastDisplacement(QuantizeSuspensionDisplacement(SpringDisplacement()));
#if !(UE_BUILD_SHIPPING)
        if (CVarSkyLeagueSuspensionForceDebug.GetValueOnAnyThread() != 0)
        {
            const float CurrentDisplacement = QuantizeSuspensionDisplacement(SpringDisplacement());
            const FVector ParentVelocity = ParentComponent ? ParentComponent->GetPhysCOMVelocity() : FVector::ZeroVector;
            const FVector ParentUp = ParentComponent ? ParentComponent->GetPhysUpVector().GetSafeNormal() : FVector::UpVector;
            UE_LOG(WheelSubBodyLog, Log,
                TEXT("[SuspensionAir] Frame=%d Wheel=%d Dt=%.5f LastDisp=%.3f CurDisp=%.3f NextAirLength=%.3f SpringLen=%.3f WorldPos=%s CarUp=%s Vel=%s PrevHitN=%s PrevHitLoc=%s"),
                ParentComponent ? ParentComponent->NumFrame() : INDEX_NONE,
                Idx(),
                delta,
                LastDisplacement,
                CurrentDisplacement,
                NextAirLength,
                SpringLength(),
                *WorldPos().ToString(),
                *ParentUp.ToString(),
                *ParentVelocity.ToString(),
                *CurrentHit.ImpactNormal.ToString(),
                *CurrentHit.Location.ToString());
        }
#endif
    }
}



/*SSphere USWheelSubBody::MakeSphere(const unsigned int& NumFrame, const float& RemainingDelta, const float& TimePassed) const
{
    // --- 1) Car kinematics ---
    SKinematic CarKS0 = ParentComponent->GetKinematicStateForFrame(NumFrame);
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
    if (WheelComponent)
    {
        const FVector ContactNormal = CurrentHit.ImpactNormal.GetSafeNormal();
        const float ContactGroundDot = FVector::DotProduct(ContactNormal, FVector::UpVector);
        const float CarUpGroundDot = FVector::DotProduct(WheelComponent->GetPhysUpVector().GetSafeNormal(), FVector::UpVector);
        const FVector VelAtPoint = WheelComponent->GetPhysVelocityAtPoint(WorldPoint);
        const float vN = FVector::DotProduct(VelAtPoint, ContactNormal);
        const float BodyVN = FVector::DotProduct(WheelComponent->GetPhysCOMVelocity(), ContactNormal);
        const float SkipMaxInwardNormalVelocity = FMath::Max(0.0f, CVarSkyLeagueSuspensionSkipOverextendedWheelContactMaxInwardNormalVelocity.GetValueOnAnyThread());
        const float SkipMaxInwardBodyNormalVelocity = FMath::Max(0.0f, CVarSkyLeagueSuspensionSkipOverextendedWheelContactMaxInwardBodyNormalVelocity.GetValueOnAnyThread());
        if (CVarSkyLeagueSuspensionSkipOverextendedWheelContact.GetValueOnAnyThread() != 0 &&
            QuantizeSuspensionDisplacement(SpringDisplacement()) < 0.0f &&
            ContactGroundDot >= CVarSkyLeagueSuspensionSkipOverextendedWheelContactMinGroundDot.GetValueOnAnyThread() &&
            CarUpGroundDot >= CVarSkyLeagueSuspensionSkipOverextendedWheelContactMinCarUpDot.GetValueOnAnyThread() &&
            vN >= -SkipMaxInwardNormalVelocity &&
            BodyVN >= -SkipMaxInwardBodyNormalVelocity)
        {
#if !(UE_BUILD_SHIPPING)
            if (CVarSkyLeagueSuspensionForceDebug.GetValueOnAnyThread() != 0)
            {
                UE_LOG(WheelSubBodyLog, Log,
                    TEXT("[SuspensionForce] Frame=%d Wheel=%d SkipOverextendedWheelContact Disp=%.3f ContactGroundDot=%.3f CarUpGroundDot=%.3f vN=%.3f BodyVN=%.3f VelZ=%.3f"),
                    WheelComponent->NumFrame(),
                    Idx(),
                    QuantizeSuspensionDisplacement(SpringDisplacement()),
                    ContactGroundDot,
                    CarUpGroundDot,
                    vN,
                    BodyVN,
                    WheelComponent->GetPhysCOMVelocity().Z);
            }
#endif
            return;
        }

        FVector N = ContactNormal;
        UpdateContactVelocityLock();

        const FVector ContactPoint = GetContactImpulseApplicationPoint();
        const FVector r = ContactPoint - WheelComponent->GetPhysCOM();
        const FMatrix InvI = WheelComponent->ComputeWorldInvInertiaTensor();
        const FVector rxn = FVector::CrossProduct(r, N);
        const FVector term = FVector::CrossProduct(
            InvI.TransformVector(rxn), r);

        const float invMassEff =
            (1.f / WheelComponent->GetPhysMass()) + FVector::DotProduct(N, term);

        // UE_LOG(WheelSubBodyLog, Log, TEXT("[CCDWheel(%d)] RegisterWheelGroundContact at NumFrame=%d"),
        //    Idx(), WheelComponent->NumFrame());
        WheelComponent->RegisterWheelGroundContact({
            this,
            CurrentHit.Component,
            CurrentHit.ImpactPoint,
            CurrentHit.FaceIndex,
			CurrentHit.SourceId,
			CurrentHit.SurfaceId,
			CurrentHit.FeatureId,
            ContactPoint,
            N,
            r,
            vN,
            invMassEff,
            QuantizeSuspensionDisplacement(SpringDisplacement()),
            IsContactVelocityLocked(),
            !bUseCompressionCrossingContactImpulse && !WasOnGroundPreviousFrame(),
            IsAtSuspensionBumpStop()
            });
	}
}

float USWheelSubBody::PredictNextDisplacement(const float& delta) const
{
    float DisplacementInput = ComputeNextAirLength(delta) - PSuspension->Setup().RaycastSafetyMargin - Radius();
    DisplacementInput = FMath::Max(0.f, DisplacementInput);
    return MaxLength() - DisplacementInput;
}

float USWheelSubBody::ComputeNextAirLength(const float& DeltaTime) const
{
    return MaxLength() + SuspensionMaxDrop() + Radius();
}


void USWheelSubBody::HandleTimers()
{
    if (bIsJumping > 0)
    {
        bIsJumping--;
    }
}

void USWheelSubBody::UpdatePhysicsState(const float& delta)
{
    // --- 1. Base axes from chassis ---
    const FVector CarUp = ParentComponent->GetPhysUpVector();
    const FVector CarFwd = ParentComponent->GetPhysForwardVector();
    const FVector CarRight = ParentComponent->GetPhysRightVector();

    UpAxis = CarUp; // stays constant

    // --- 2. Compute steering rotation around UpAxis ---
    // SteeringAngle in radians
    FQuat SteerQuat(UpAxis, SteeringAngle);

    // --- 3. Apply steering to forward axis ---
    ForwardAxis = SteerQuat.RotateVector(CarFwd).GetSafeNormal();

    // --- 4. Right axis = orthogonal ---
    RightAxis = FVector::CrossProduct(UpAxis, ForwardAxis).GetSafeNormal();

    // --- 5. Update rolling angle (rotation around RightAxis) ---
    // update omega
    Omega = ParentComponent->GetPhysForwardSpeed() / FMath::Max(Radius(), KINDA_SMALL_NUMBER);
    float angVel = AngularVelocity(); // rad/s
    RollAngle += angVel * delta;
    RollAngle = FMath::Fmod(RollAngle, 2.f * PI);

    // --- 6. World position from suspension ---
    FVector pos = WorldPos();

    // --- 7. Fill render data struct for the visual thread ---
    RenderData.WorldPosition = pos;
    RenderData.UpAxis = UpAxis;
    RenderData.ForwardAxis = ForwardAxis;
    RenderData.RightAxis = RightAxis;

    RenderData.SpringOffset = SuspensionRestLength() - SpringLength();

    RenderData.SteerRotation = FQuat(UpAxis, SteeringAngle);
    RenderData.RollRotation = FQuat(RightAxis, RollAngle);
}

void USWheelSubBody::UpdateSteerAngle(const float& delta)
{
    float steerInput = 0.0;
    if (WheelComponent)
    {
		steerInput = WheelComponent->GetPhysSteeringInput(); // expected to be in range [-1, 1]
    }

    if (IsSteeringEnabled())
    {
        float speed = ParentComponent->GetPhysCOMVelocity().Size();

        // saturation max speed
        float normSpeed = FMath::Clamp(speed / ParentComponent->GetPhysMaxSpeed(), 0.f, 1.f);

        // steering reduction curve
        float steerFactor =
            1.0f
            - 0.85f * normSpeed;  // 1.0 at low speeds, ~0.25 at max speed

        if (speed < 300.f)
            steerFactor *= 1.2f; // +20%

        // final angle
        SteeringAngle = FMath::DegreesToRadians(MaxSteeringAngle() * steerInput * steerFactor);
    }
    else
    {
        SteeringAngle = 0.0f;
    }
}

FVector USWheelSubBody::WorldPos() const
{
    FTransform ChassisTM(ParentComponent->GetPhysRotation(), ParentComponent->GetPhysLocation());
    return WorldPosFromCarTransform(ChassisTM);
}

FVector USWheelSubBody::SteeringPos() const
{
    const FVector ChassisUp = ParentComponent->GetPhysUpVector();

    // Compute world position of the wheel
    const FVector WheelPosWS = WorldPos();

    // Get the world center of mass of the vehicle
    const FVector CoMWorld = ParentComponent->GetPhysCOM();

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
    return -ParentComponent->GetPhysUpVector();
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
    return PWheel ? PWheel->GetEffectiveRadius() : 0.0f;
}

float USWheelSubBody::AngularVelocity() const
{
    return Omega;
}

void USWheelSubBody::SetAngularVelocity(const float& InOmega)
{
    Omega = InOmega;
}

float USWheelSubBody::SpringDisplacement() const
{
    return PSuspension ? PSuspension->GetLastDisplacement() : 0.0f;
}

float USWheelSubBody::ContactSpringDisplacement() const
{
    if (!PSuspension || !ParentComponent || !IsOnGround())
    {
        return SpringDisplacement();
    }

    const FTransform ChassisTM(
        ParentComponent->GetPhysRotation(), ParentComponent->GetPhysLocation());
    const FVector Up = ChassisTM.GetUnitAxis(EAxis::Z);
    const FVector SuspensionAnchor = ChassisTM.TransformPosition(
        GetLocalOffset() + PSuspension->Setup().MaxLength * FVector::UpVector);
    const float ProjectedCompression = FVector::DotProduct(
        SuspensionAnchor - CurrentHit.Location, Up);
    const float DisplacementInput = FMath::Max(
        0.0f, ProjectedCompression - PSuspension->Setup().RaycastSafetyMargin);
    return QuantizeSuspensionDisplacement(
        PSuspension->Setup().MaxLength - DisplacementInput);
}
float USWheelSubBody::SpringLength() const
{
    return PSuspension->GetSpringLength();
}

float USWheelSubBody::GetLastDisplacement() const
{
    return PSuspension->GetLastDisplacement();
}

void USWheelSubBody::SetLastDisplacement(const float& displacement)
{
    PSuspension->SetLastDisplacement(displacement);
}

float USWheelSubBody::MaxLength() const
{
    return PSuspension->Setup().MaxLength;
}

float USWheelSubBody::SuspensionRestLength() const
{
	return ChaosWheel->SuspensionMaxRaise;
}

float USWheelSubBody::SuspensionMaxRaise() const
{
    return SuspensionMaxRaiseOverride >= 0.0f
        ? SuspensionMaxRaiseOverride
        : ChaosWheel->SuspensionMaxRaise;
}

float USWheelSubBody::SuspensionMaxDrop() const
{
    return SuspensionMaxDropOverride >= 0.0f
        ? SuspensionMaxDropOverride
        : ChaosWheel->SuspensionMaxDrop;
}

void USWheelSubBody::ConfigureSuspensionTravel(const float MaxRaise, const float MaxDrop)
{
    SuspensionMaxRaiseOverride = FMath::Max(0.0f, MaxRaise);
    SuspensionMaxDropOverride = FMath::Max(0.0f, MaxDrop);
    if (ChaosWheel)
    {
        ChaosWheel->SuspensionMaxRaise = SuspensionMaxRaiseOverride;
        ChaosWheel->SuspensionMaxDrop = SuspensionMaxDropOverride;
    }
}

bool USWheelSubBody::HasConfiguredSuspensionTravel() const
{
    return SuspensionMaxRaiseOverride >= 0.0f && SuspensionMaxDropOverride >= 0.0f;
}

float USWheelSubBody::SuspensionSpringRateCm() const
{
    return bUseSuspensionForceModelOverride
        ? SuspensionSpringRateCmOverride
        : SuspensionSpringRate * 100.0f;
}

bool USWheelSubBody::IsAtSuspensionBumpStop() const
{
    if (bUseCompressionCrossingContactImpulse)
    {
        return bCrossedIntoCompressionThisFrame;
    }
    return ContactSpringDisplacement() >= FMath::Max(
        0.0f, CVarSkyLeagueSuspensionBumpStopStartCompression.GetValueOnAnyThread());
}

float USWheelSubBody::StaticSpringCompression() const
{
    if (!PSuspension || PSuspension->Setup().SpringRate <= SMALL_NUMBER)
    {
        return 0.0f;
    }

    return QuantizeSuspensionDisplacement(
        PSuspension->Setup().RestingForce / PSuspension->Setup().SpringRate);
}

float USWheelSubBody::SuspensionCompressionDamping() const
{
    return SuspensionCompressionDampingOverride;
}

float USWheelSubBody::SuspensionReboundDamping() const
{
    return SuspensionReboundDampingOverride;
}

bool USWheelSubBody::UsesDirectSuspensionDamping() const
{
    return bUseSuspensionForceModelOverride;
}

bool USWheelSubBody::ClampsSuspensionForceToPositive() const
{
    return bUseSuspensionForceModelOverride &&
		(bClampSuspensionForceToPositive ||
			(bClampPositiveUntilFirstCompression && !bContactVelocityLocked));
}

void USWheelSubBody::ConfigureSuspensionForceModel(const float SpringRateCm,
    const float CompressionDamping, const float ReboundDamping,
	const bool bClampForceToPositive)
{
    bUseSuspensionForceModelOverride = true;
    bClampSuspensionForceToPositive = bClampForceToPositive;
    SuspensionSpringRateCmOverride = FMath::Max(0.0f, SpringRateCm);
    SuspensionCompressionDampingOverride = FMath::Max(0.0f, CompressionDamping);
    SuspensionReboundDampingOverride = FMath::Max(0.0f, ReboundDamping);
}

void USWheelSubBody::SetUseEffectiveSuspensionSweepRadius(const bool bEnabled)
{
    bUseEffectiveSuspensionSweepRadius = bEnabled;
}

void USWheelSubBody::SetUseIncreasingDisplacementAsCompression(const bool bEnabled)
{
    bUseIncreasingDisplacementAsCompression = bEnabled;
}

void USWheelSubBody::SetClampPositiveUntilFirstCompression(const bool bEnabled)
{
    bClampPositiveUntilFirstCompression = bEnabled;
}

void USWheelSubBody::SetUseCompressionCrossingContactImpulse(const bool bEnabled)
{
	bUseCompressionCrossingContactImpulse = bEnabled;
}

void USWheelSubBody::SetGroundForceApplicationMode(const uint8 Mode)
{
	GroundForceApplicationMode = FMath::Min<uint8>(Mode, 3);
	ContactImpulseApplicationMode = GroundForceApplicationMode;
}

void USWheelSubBody::SetContactImpulseApplicationMode(const uint8 Mode)
{
	ContactImpulseApplicationMode = FMath::Min<uint8>(Mode, 3);
}

FVector USWheelSubBody::GetSuspensionForceApplicationPoint() const
{
	switch (GroundForceApplicationMode)
	{
	case 2:
		return WorldPos();
	case 3:
		return WorldPos() - SpringDisplacement() * ParentComponent->GetPhysUpVector();
	default:
		return CurrentHit.ImpactPoint;
	}
}

FVector USWheelSubBody::GetContactImpulseApplicationPoint() const
{
	switch (ContactImpulseApplicationMode)
	{
	case 1:
		return CurrentHit.ImpactPoint;
	case 3:
		return WorldPos() - SpringDisplacement() * ParentComponent->GetPhysUpVector();
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
    SuspensionStiffnessForceScale = FMath::Max(0.0f, StiffnessScale);
    SuspensionStiffnessReferenceDisplacement = StiffnessReferenceDisplacement;
    SuspensionPreRestCompressionDampingScale = FMath::Max(
        0.0f, PreRestCompressionDampingScale);
    bClampNegativeSuspensionDisplacement = bClampNegativeDisplacement;
    bDisableSuspensionBumpStopForce = bDisableBumpStopForce;
}

float USWheelSubBody::GetSuspensionDampingReboundRatio() const
{
    return SuspensionDampingReboundRatio;
}

float USWheelSubBody::GetSuspensionDampingCompressionRatio() const
{
    return SuspensionDampingCompressionRatio;
}

float USWheelSubBody::GetSuspensionForce() const
{
    return SuspensionForce;
}

bool USWheelSubBody::IsContactVelocityLocked() const
{
    return bContactVelocityLocked;
}

bool USWheelSubBody::WasOnGroundPreviousFrame() const
{
    return bWasOnGroundPrevFrame;
}

bool USWheelSubBody::IsOnGround() const
{
	if(PWheel == nullptr)
	{
		return false;
	}
    return PWheel->InContact();
}

void USWheelSubBody::SetOnGround(const bool& on_ground)
{
    bool bWasOnGround = IsOnGround();

    if (!bWasOnGround && on_ground)
    {
        const float NormalSpeed = FVector::DotProduct(ParentComponent->GetPhysVelocityAtPoint(WorldPos()), CurrentHit.ImpactNormal);

        // Real impact
        if (NormalSpeed < -5.0f) // cm/s, to tune
        {
			ParentComponent->RcvImpactOnSubBody(*this, CurrentHit.ImpactPoint);
        }
    }

    // RegisterWheelContact
    if (on_ground)
    {
        if (!bCountedGroundFrame)
        {
            ConsecutiveGroundFrames = FMath::Min<uint8>(ConsecutiveGroundFrames + 1, 31);
            bCountedGroundFrame = true;
        }
        ApplyImpulse(FVector::UpVector, CurrentHit.ImpactPoint);
    }
    else
    {
        ConsecutiveGroundFrames = 0;
        ResetContactVelocityLock();
    }

    PWheel->SetOnGround(on_ground);
}

void USWheelSubBody::SetIsJumping(const uint8 NbFrames)
{
    bIsJumping = NbFrames;
    if (NbFrames > 0)
    {
        ConsecutiveGroundFrames = 0;
        ResetContactVelocityLock();
    }
}

uint8 USWheelSubBody::IsJumping() const
{
    return bIsJumping;
}

void USWheelSubBody::SetJumpUnilateralSupport(const bool bEnabled)
{
    bJumpUnilateralSupport = bEnabled;
}

bool USWheelSubBody::HasJumpUnilateralSupport() const
{
    return bJumpUnilateralSupport;
}

uint8 USWheelSubBody::GetConsecutiveGroundFrames() const
{
    return ConsecutiveGroundFrames;
}

void USWheelSubBody::SetConsecutiveGroundFrames(const uint8 Frames)
{
    ConsecutiveGroundFrames = FMath::Min<uint8>(Frames, 31);
}

void USWheelSubBody::UpdateContactVelocityLock()
{
    if (QuantizeSuspensionDisplacement(SpringDisplacement()) >= 0.0f)
    {
        bContactVelocityLocked = true;
    }
}

void USWheelSubBody::ResetContactVelocityLock()
{
    bContactVelocityLocked = false;
}

bool USWheelSubBody::IsSteeringEnabled() const
{
    return PWheel->SteeringEnabled;
}

float USWheelSubBody::MaxSteeringAngle() const
{
    return PWheel->MaxSteeringAngle;
}

float USWheelSubBody::GetSuspensionOffset() const
{
    return SpringDisplacement();
}

FVector USWheelSubBody::GetHitContactNormal() const
{
    if (IsOnGround())
    {
        return CurrentHit.ImpactNormal;
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
        return CurrentHit.ImpactPoint;
    }

    return WorldPos();
}

void USWheelSubBody::SetHitContactNormal(const FVector& ImpactNormal)
{
    CurrentHit.ImpactNormal = ImpactNormal;
}

const SWheelRenderData& USWheelSubBody::GetRenderData() const
{
    return RenderData;
}

void USWheelSubBody::SetRollAngle(const float& angle)
{
    FRotator RollRot(RenderData.RollRotation);
    RollRot.Roll = FMath::RadiansToDegrees(angle);
    RenderData.RollRotation = FQuat(RollRot);
}

float USWheelSubBody::GetRollAngle() const
{
    FRotator RollRot(RenderData.RollRotation);
    return FMath::DegreesToRadians(RollRot.Roll);
}

void USWheelSubBody::SetChaosWheel(UChaosVehicleWheel* InChaosWheel)
{
    ChaosWheel = InChaosWheel;
    if (ChaosWheel && SuspensionMaxRaiseOverride >= 0.0f
        && SuspensionMaxDropOverride >= 0.0f)
    {
        ChaosWheel->SuspensionMaxRaise = SuspensionMaxRaiseOverride;
        ChaosWheel->SuspensionMaxDrop = SuspensionMaxDropOverride;
    }
}

void USWheelSubBody::SetWheelSim(Chaos::FSimpleWheelSim* InPWheel)
{
    if (!InPWheel)
    {
        return;
    }
    PWheel = InPWheel;
    PWheel->SetMassPerWheel(WheelMass);
    SetMass(WheelMass);
}

void USWheelSubBody::SetSuspensionSim(Chaos::FSimpleSuspensionSim* InPSuspension)
{
    PSuspension = InPSuspension;
	// Print parameters for debug
	// UE_LOG(WheelSubBodyLog, Warning, TEXT("WheelSubBody %d Suspension parameters: MaxLength= %.2f, SpringRate= %.2f, CompressionDamping= %.2f, ReboundDamping= %.2f"),
    //    Idx(), PSuspension->Setup().MaxLength, PSuspension->Setup().SpringRate, PSuspension->Setup().CompressionDamping, PSuspension->Setup().ReboundDamping);
}

void USWheelSubBody::SetLocalOffset(const FVector& InLocalOffset)
{
    SetRelativeLocation(InLocalOffset);
}
