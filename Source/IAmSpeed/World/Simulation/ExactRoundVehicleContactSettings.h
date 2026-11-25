#pragma once

#include "ExactRoundConfiguration.h"

namespace Speed
{
struct FExactRoundBoxContactSettings final
{
    bool CoupledSpeedLimits = false;
    float ContactResponseTime = 0.f;
    bool InitialOverlapManifold = false;
    bool SinglePointGroundCoupled = false;
    bool RetireSeparatingOwnGroundConstraints = false;
    bool SupportedTransitionResponse = false;
    bool DeferSupportedTransitionVelocityTransport = false;
    float RoofSlideSupportMinUpDot = 0.f;
    float EdgeLatchFaceReleaseDot = 0.f;
    float FaceSupportContactEpsilonCm = 0.f;
    float FaceSupportPenetrationTolCm = 0.f;
    float FaceSupportSeparatingSpeedCmS = 0.f;
    float EdgeRecoverPenetrationTolCm = 0.f;
    float EdgeRecoverSeparatingSpeedCmS = 0.f;
    float VertexSupportContactTolCm = 0.f;
    float VertexSupportSeparatingSpeedCmS = 0.f;
    float SoftWallDepenMaxNormalKillCmS = 0.f;
    float GutterDepenMaxUpDeltaCmS = 0.f;
    float NearGutterVerticalDepenMaxNormalKillCmS = 0.f;
    float GutterSlideDamping = 0.f;
    float GutterSlideCrossDamping = 0.f;
    float GutterSlideMaxDeltaCmS = 0.f;
    float GutterSlideMinSpeedCmS = 0.f;
    float GutterSlideMaxClimbSpeedCmS = 0.f;
    float GutterSlideMaxClimbBrakeDeltaCmS = 0.f;
    bool RoofBoxSupportReuse = false;
    bool RoofBoxSupportRepublish = false;

    bool operator==(const FExactRoundBoxContactSettings&) const = default;

    void AppendHash(FExactRoundConfigurationWriter& Writer) const noexcept
    {
        Writer.Boolean(CoupledSpeedLimits);
        Writer.Float(ContactResponseTime);
        Writer.Boolean(InitialOverlapManifold);
        Writer.Boolean(SinglePointGroundCoupled);
        Writer.Boolean(RetireSeparatingOwnGroundConstraints);
        Writer.Boolean(SupportedTransitionResponse);
        Writer.Boolean(DeferSupportedTransitionVelocityTransport);
        Writer.Float(RoofSlideSupportMinUpDot);
        Writer.Float(EdgeLatchFaceReleaseDot);
        Writer.Float(FaceSupportContactEpsilonCm);
        Writer.Float(FaceSupportPenetrationTolCm);
        Writer.Float(FaceSupportSeparatingSpeedCmS);
        Writer.Float(EdgeRecoverPenetrationTolCm);
        Writer.Float(EdgeRecoverSeparatingSpeedCmS);
        Writer.Float(VertexSupportContactTolCm);
        Writer.Float(VertexSupportSeparatingSpeedCmS);
        Writer.Float(SoftWallDepenMaxNormalKillCmS);
        Writer.Float(GutterDepenMaxUpDeltaCmS);
        Writer.Float(NearGutterVerticalDepenMaxNormalKillCmS);
        Writer.Float(GutterSlideDamping);
        Writer.Float(GutterSlideCrossDamping);
        Writer.Float(GutterSlideMaxDeltaCmS);
        Writer.Float(GutterSlideMinSpeedCmS);
        Writer.Float(GutterSlideMaxClimbSpeedCmS);
        Writer.Float(GutterSlideMaxClimbBrakeDeltaCmS);
        Writer.Boolean(RoofBoxSupportReuse);
        Writer.Boolean(RoofBoxSupportRepublish);
    }
};

struct FExactRoundSuspensionSettings final
{
    float CompressionDampingScale = 0.f;
    float CompressionQuadraticDamping = 0.f;
    float ReboundDampingScale = 0.f;
    float StiffnessScale = 0.f;
    float StiffnessReferenceDisplacement = 0.f;
    float PreRestCompressionDampingScale = 0.f;
    bool ClampPositiveForce = false;
    float MaxAttractiveForce = 0.f;
    float MaxAttractiveForceMinNormalZ = 0.f;
    bool JumpStateClampsNegativeForce = false;
    float RestingForceScale = 0.f;
    float CeilingFadeStartNormalZ = 0.f;
    float BumpStopStartCompression = 0.f;
    float BumpStopRate = 0.f;
    bool ClampNegativeDisplacement = false;
    bool RestingForceRequiresCompression = false;
    bool SkipOverextendedWheelContact = false;
    float SkipOverextendedWheelContactMinGroundDot = 0.f;
    float SkipOverextendedWheelContactMinCarUpDot = 0.f;
    float SkipOverextendedWheelContactMaxInwardNormalVelocity = 0.f;
    float SkipOverextendedWheelContactMaxInwardBodyNormalVelocity = 0.f;
    bool ForceDiagnosticReads = false;

    bool operator==(const FExactRoundSuspensionSettings&) const = default;

    void AppendHash(FExactRoundConfigurationWriter& Writer) const noexcept
    {
        Writer.Float(CompressionDampingScale);
        Writer.Float(CompressionQuadraticDamping);
        Writer.Float(ReboundDampingScale);
        Writer.Float(StiffnessScale);
        Writer.Float(StiffnessReferenceDisplacement);
        Writer.Float(PreRestCompressionDampingScale);
        Writer.Boolean(ClampPositiveForce);
        Writer.Float(MaxAttractiveForce);
        Writer.Float(MaxAttractiveForceMinNormalZ);
        Writer.Boolean(JumpStateClampsNegativeForce);
        Writer.Float(RestingForceScale);
        Writer.Float(CeilingFadeStartNormalZ);
        Writer.Float(BumpStopStartCompression);
        Writer.Float(BumpStopRate);
        Writer.Boolean(ClampNegativeDisplacement);
        Writer.Boolean(RestingForceRequiresCompression);
        Writer.Boolean(SkipOverextendedWheelContact);
        Writer.Float(SkipOverextendedWheelContactMinGroundDot);
        Writer.Float(SkipOverextendedWheelContactMinCarUpDot);
        Writer.Float(SkipOverextendedWheelContactMaxInwardNormalVelocity);
        Writer.Float(SkipOverextendedWheelContactMaxInwardBodyNormalVelocity);
        Writer.Boolean(ForceDiagnosticReads);
    }
};

// Response shared by the grouped impulse solve and the delayed support checks.
struct FExactRoundWheelGroundContactSettings final
{
    float NormalVelocityTimeConstantOverride = -1.f;
    float NormalVelocityDeadzone = 0.f;
    float LockedSeparatingTimeConstant = 0.f;
    float LockedSeparatingMaxNormalVelocity = 0.f;
    bool LockedSeparatingDamping = false;
    float ImpulseDeadzone = 0.f;
    float Softness = 0.f;
    std::int32_t SolverMode = 0;
    bool RequireFinalSupport = false;

    bool operator==(const FExactRoundWheelGroundContactSettings&) const = default;

    void AppendHash(FExactRoundConfigurationWriter& Writer) const noexcept
    {
        Writer.Float(NormalVelocityTimeConstantOverride);
        Writer.Float(NormalVelocityDeadzone);
        Writer.Float(LockedSeparatingTimeConstant);
        Writer.Float(LockedSeparatingMaxNormalVelocity);
        Writer.Boolean(LockedSeparatingDamping);
        Writer.Float(ImpulseDeadzone);
        Writer.Float(Softness);
        Writer.Signed(SolverMode);
        Writer.Boolean(RequireFinalSupport);
    }
};
}
