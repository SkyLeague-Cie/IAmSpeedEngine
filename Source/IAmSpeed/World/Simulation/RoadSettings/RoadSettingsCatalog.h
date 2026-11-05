#pragma once
// Declared native CVar slots ONLY. This catalog is not a complete physical
// configuration, native code identity or an authority/restore certificate.
#include <array>
#include <cstdint>
#include <string_view>
namespace Speed::RoadSettings
{
enum class EKind : std::uint8_t {Integer=1,Scalar=2};
struct FKey {std::uint16_t Index;EKind Kind;std::string_view Name;};
inline constexpr std::size_t RequiredSlots=178;
inline constexpr std::string_view Coverage="declared-native-cvar-catalog-only/1";
inline constexpr std::array<FKey,RequiredSlots> Catalog={{
    {0,EKind::Scalar,"p.IAmSpeed.AnalyticWorld.Landscape.FlatnessToleranceCm"},
    {1,EKind::Integer,"p.IAmSpeed.AutoRecoverContactDebug"},
    {2,EKind::Integer,"p.IAmSpeed.Box.DebugGroundSupport"},
    {3,EKind::Integer,"p.IAmSpeed.Box.DeferSupportedTransitionVelocityTransport"},
    {4,EKind::Scalar,"p.IAmSpeed.Box.InertiaScale"},
    {5,EKind::Integer,"p.IAmSpeed.Box.SupportedTransitionResponse"},
    {6,EKind::Integer,"p.IAmSpeed.CanonicalFrame.Enabled"},
    {7,EKind::Integer,"p.IAmSpeed.Collision.CoupledContactImpulse"},
    {8,EKind::Integer,"p.IAmSpeed.Collision.CoupledContactImpulseIterations"},
    {9,EKind::Scalar,"p.IAmSpeed.Collision.CoupledContactImpulseMinNormalSpeed"},
    {10,EKind::Integer,"p.IAmSpeed.Collision.DebugPersistentDynamicPairs"},
    {11,EKind::Integer,"p.IAmSpeed.Collision.DebugSphereBoxImpulse"},
    {12,EKind::Integer,"p.IAmSpeed.Collision.ExactBoxRestingSupport"},
    {13,EKind::Integer,"p.IAmSpeed.Collision.PersistentDynamicPairs"},
    {14,EKind::Integer,"p.IAmSpeed.Collision.RetireSeparatingOwnGroundConstraints"},
    {15,EKind::Scalar,"p.IAmSpeed.Collision.RollingBaumgarte"},
    {16,EKind::Scalar,"p.IAmSpeed.Collision.RollingGentleAcquisitionSpeedCmS"},
    {17,EKind::Scalar,"p.IAmSpeed.Collision.RollingMaxCorrectionSpeed"},
    {18,EKind::Scalar,"p.IAmSpeed.Collision.RollingPenetrationSlopCm"},
    {19,EKind::Scalar,"p.IAmSpeed.Collision.RollingReleaseSpeedCmS"},
    {20,EKind::Scalar,"p.IAmSpeed.Collision.RollingSeparationToleranceCm"},
    {21,EKind::Scalar,"p.IAmSpeed.Collision.RollingTangentialFakePhysicsScale"},
    {22,EKind::Integer,"p.IAmSpeed.Collision.SinglePointGroundCoupled"},
    {23,EKind::Integer,"p.IAmSpeed.Collision.SolverEventTraceFrame"},
    {24,EKind::Scalar,"p.IAmSpeed.Collision.SphereBoxFrictionOverride"},
    {25,EKind::Scalar,"p.IAmSpeed.Collision.SphereBoxManifoldFrictionOverride"},
    {26,EKind::Integer,"p.IAmSpeed.Collision.SphereBoxNonpenetratingV2"},
    {27,EKind::Integer,"p.IAmSpeed.Collision.SphereBoxNonpenetratingV2ForTesting"},
    {28,EKind::Integer,"p.IAmSpeed.Collision.SphereBoxProjection"},
    {29,EKind::Scalar,"p.IAmSpeed.Collision.SphereBoxRestitutionOverride"},
    {30,EKind::Integer,"p.IAmSpeed.Collision.SphereWorldProjection"},
    {31,EKind::Integer,"p.IAmSpeed.Collision.UnilateralRollingPairs"},
    {32,EKind::Integer,"p.IAmSpeed.CoupledPose.Debug"},
    {33,EKind::Integer,"p.IAmSpeed.CoupledPose.Enabled"},
    {34,EKind::Scalar,"p.IAmSpeed.CoupledPose.FloorPivotRotationLengthCm"},
    {35,EKind::Scalar,"p.IAmSpeed.CoupledPose.HitboxSlopCm"},
    {36,EKind::Integer,"p.IAmSpeed.CoupledPose.Passes"},
    {37,EKind::Scalar,"p.IAmSpeed.CoupledPose.RotationLengthCm"},
    {38,EKind::Integer,"p.IAmSpeed.CoupledPose.SharedWallPlaneRetention"},
    {39,EKind::Integer,"p.IAmSpeed.CoupledPose.SupportedSurfaceContactPoint"},
    {40,EKind::Integer,"p.IAmSpeed.CoupledPose.VelocityCorrection"},
    {41,EKind::Scalar,"p.IAmSpeed.CoupledPose.WheelGapCm"},
    {42,EKind::Scalar,"p.IAmSpeed.CoupledPose.WheelManifoldRotationLengthCm"},
    {43,EKind::Integer,"p.IAmSpeed.DebugKinematics"},
    {44,EKind::Integer,"p.IAmSpeed.DebugSteering"},
    {45,EKind::Integer,"p.IAmSpeed.DebugWheelFriction"},
    {46,EKind::Integer,"p.IAmSpeed.DebugWheelFrictionDetailed"},
    {47,EKind::Scalar,"p.IAmSpeed.EdgeLatchFaceReleaseDot"},
    {48,EKind::Scalar,"p.IAmSpeed.EdgeRecoverPenetrationTolCm"},
    {49,EKind::Scalar,"p.IAmSpeed.EdgeRecoverSeparatingSpeedCmS"},
    {50,EKind::Scalar,"p.IAmSpeed.FaceSupportContactEpsilonCm"},
    {51,EKind::Scalar,"p.IAmSpeed.FaceSupportPenetrationTolCm"},
    {52,EKind::Scalar,"p.IAmSpeed.FaceSupportSeparatingSpeedCmS"},
    {53,EKind::Integer,"p.IAmSpeed.FastSimulation.Enabled"},
    {54,EKind::Integer,"p.IAmSpeed.FastSimulation.MaxFramesPerRun"},
    {55,EKind::Integer,"p.IAmSpeed.FastSimulation.ReadyDelayCallbacks"},
    {56,EKind::Integer,"p.IAmSpeed.FastSimulation.WindowTiming"},
    {57,EKind::Integer,"p.IAmSpeed.FrameWitness.Compression"},
    {58,EKind::Integer,"p.IAmSpeed.FrameWitness.Enabled"},
    {59,EKind::Scalar,"p.IAmSpeed.GutterDepenMaxUpDeltaCmS"},
    {60,EKind::Scalar,"p.IAmSpeed.GutterSlideCrossDamping"},
    {61,EKind::Scalar,"p.IAmSpeed.GutterSlideDamping"},
    {62,EKind::Scalar,"p.IAmSpeed.GutterSlideMaxClimbBrakeDeltaCmS"},
    {63,EKind::Scalar,"p.IAmSpeed.GutterSlideMaxClimbSpeedCmS"},
    {64,EKind::Scalar,"p.IAmSpeed.GutterSlideMaxDeltaCmS"},
    {65,EKind::Scalar,"p.IAmSpeed.GutterSlideMinSpeedCmS"},
    {66,EKind::Scalar,"p.IAmSpeed.NearGutterVerticalDepenMaxNormalKillCmS"},
    {67,EKind::Integer,"p.IAmSpeed.Parallel.ActorBaseConsume1430"},
    {68,EKind::Integer,"p.IAmSpeed.Parallel.ActorKinematics1213"},
    {69,EKind::Integer,"p.IAmSpeed.Parallel.Camera"},
    {70,EKind::Integer,"p.IAmSpeed.Parallel.FrameCostPhases"},
    {71,EKind::Integer,"p.IAmSpeed.Parallel.FrameTiming"},
    {72,EKind::Integer,"p.IAmSpeed.Parallel.Lanes"},
    {73,EKind::Integer,"p.IAmSpeed.Parallel.MaxLanes"},
    {74,EKind::Integer,"p.IAmSpeed.Parallel.MinFacetItems"},
    {75,EKind::Integer,"p.IAmSpeed.Parallel.MinPrepareItems"},
    {76,EKind::Integer,"p.IAmSpeed.Parallel.MinQueryItems"},
    {77,EKind::Integer,"p.IAmSpeed.Parallel.PrivateIdleSpinMicros1403"},
    {78,EKind::Integer,"p.IAmSpeed.Parallel.QueryChunkItems1075"},
    {79,EKind::Integer,"p.IAmSpeed.Parallel.ReservedPhysicalCores"},
    {80,EKind::Integer,"p.IAmSpeed.Parallel.WorkerBackend"},
    {81,EKind::Scalar,"p.IAmSpeed.RoofSlideSupportMinUpDot"},
    {82,EKind::Integer,"p.IAmSpeed.Simulation.ActorProfile"},
    {83,EKind::Integer,"p.IAmSpeed.Simulation.ExecutionMode"},
    {84,EKind::Integer,"p.IAmSpeed.Simulation.PerformanceAudit"},
    {85,EKind::Scalar,"p.IAmSpeed.Simulation.RealTimeCadenceToleranceMs"},
    {86,EKind::Scalar,"p.IAmSpeed.Simulation.RealTimeCadenceWarningCooldownSeconds"},
    {87,EKind::Integer,"p.IAmSpeed.Simulation.RealTimeCadenceWarnings"},
    {88,EKind::Scalar,"p.IAmSpeed.Simulation.RealTimePrecisionWindowMs"},
    {89,EKind::Scalar,"p.IAmSpeed.Simulation.RealTimeRecoveryWindowMs"},
    {90,EKind::Integer,"p.IAmSpeed.Simulation.SlowFrameWarnings"},
    {91,EKind::Scalar,"p.IAmSpeed.Simulation.WarningFrameFraction"},
    {92,EKind::Integer,"p.IAmSpeed.Simulation.WorkerPaused"},
    {93,EKind::Scalar,"p.IAmSpeed.SoftWallDepenMaxNormalKillCmS"},
    {94,EKind::Integer,"p.IAmSpeed.Suspension.HullContactContinuousAngularDrag"},
    {95,EKind::Integer,"p.IAmSpeed.VehicleInertia.CovariantFrame"},
    {96,EKind::Scalar,"p.IAmSpeed.VertexSupportContactTolCm"},
    {97,EKind::Scalar,"p.IAmSpeed.VertexSupportSeparatingSpeedCmS"},
    {98,EKind::Scalar,"p.IAmSpeed.WheelContact.CanonicalSupportMaxAngularSpeed"},
    {99,EKind::Scalar,"p.IAmSpeed.WheelContact.CanonicalSupportMaxNormalSpeed"},
    {100,EKind::Integer,"p.IAmSpeed.WheelContact.CanonicalSupportPose"},
    {101,EKind::Integer,"p.IAmSpeed.WheelContact.Debug"},
    {102,EKind::Scalar,"p.IAmSpeed.WheelContact.ImpulseDeadzone"},
    {103,EKind::Integer,"p.IAmSpeed.WheelContact.LockedSeparatingDamping"},
    {104,EKind::Scalar,"p.IAmSpeed.WheelContact.LockedSeparatingMaxNormalVelocity"},
    {105,EKind::Scalar,"p.IAmSpeed.WheelContact.LockedSeparatingNormalVelTimeConstant"},
    {106,EKind::Scalar,"p.IAmSpeed.WheelContact.NormalVelTimeConstant"},
    {107,EKind::Scalar,"p.IAmSpeed.WheelContact.NormalVelocityDeadzone"},
    {108,EKind::Integer,"p.IAmSpeed.WheelContact.RequireFinalSupport"},
    {109,EKind::Scalar,"p.IAmSpeed.WheelContact.Softness"},
    {110,EKind::Integer,"p.IAmSpeed.WheelContact.SolverMode"},
    {111,EKind::Integer,"p.IAmSpeed.WheelFrame.ClampLateralImpulse"},
    {112,EKind::Integer,"p.IAmSpeed.WheelFrame.DebugIntervalFrames"},
    {113,EKind::Integer,"p.IAmSpeed.WheelFrame.HandbrakeDriveForceBlendModel"},
    {114,EKind::Scalar,"p.IAmSpeed.WheelFrame.HandbrakeDriveForceScale"},
    {115,EKind::Integer,"p.IAmSpeed.WheelFrame.HandbrakeDriveModel"},
    {116,EKind::Scalar,"p.IAmSpeed.WheelFrame.HandbrakeFrontLateralFrictionFactor"},
    {117,EKind::Integer,"p.IAmSpeed.WheelFrame.HandbrakeFullInputCOMSpeedModel"},
    {118,EKind::Scalar,"p.IAmSpeed.WheelFrame.HandbrakeLateralImpulseScale"},
    {119,EKind::Scalar,"p.IAmSpeed.WheelFrame.HandbrakeRearLateralFrictionFactor"},
    {120,EKind::Integer,"p.IAmSpeed.WheelFrame.HandbrakeSteeringBlendModel"},
    {121,EKind::Integer,"p.IAmSpeed.WheelFrame.HandbrakeSupportProjectionGuard"},
    {122,EKind::Scalar,"p.IAmSpeed.WheelFrame.LateralForwardLossScale"},
    {123,EKind::Scalar,"p.IAmSpeed.WheelFrame.LateralFrictionScale"},
    {124,EKind::Scalar,"p.IAmSpeed.WheelFrame.LongitudinalScrubScale"},
    {125,EKind::Scalar,"p.IAmSpeed.WheelFrame.NormalInputFullFraction"},
    {126,EKind::Scalar,"p.IAmSpeed.WheelFrame.SteeringCurvatureScale"},
    {127,EKind::Scalar,"p.IAmSpeed.WheelFrame.SteeringRearLateralFrictionReverseScale"},
    {128,EKind::Scalar,"p.IAmSpeed.WheelFrame.SteeringRearLateralFrictionScale"},
    {129,EKind::Integer,"p.IAmSpeed.WheelFrame.TurnSummaryIntervalFrames"},
    {130,EKind::Scalar,"p.IAmSpeed.WheelFrame.UnsteeredAligningYawFullSlipSpeed"},
    {131,EKind::Scalar,"p.IAmSpeed.WheelFrame.UnsteeredAligningYawMaxRate"},
    {132,EKind::Scalar,"p.IAmSpeed.WheelFrame.UnsteeredAligningYawMinSlipSpeed"},
    {133,EKind::Integer,"p.IAmSpeed.WheelFrame.UnsteeredAligningYawResidualDamping"},
    {134,EKind::Scalar,"p.IAmSpeed.WheelFrame.UnsteeredAligningYawTimeConstant"},
    {135,EKind::Scalar,"p.IAmSpeed.WheelFrame.ZeroSteerLateralFrictionScale"},
    {136,EKind::Integer,"p.IAmSpeed.WheelSupport.ProjectionCertifiedExtruded"},
    {137,EKind::Integer,"p.IAmSpeed.WheelSupport.ProjectionCoherentSpringState"},
    {138,EKind::Integer,"p.IAmSpeed.WheelSupport.ProjectionDebug"},
    {139,EKind::Scalar,"p.IAmSpeed.WheelSupport.ProjectionMaxGap"},
    {140,EKind::Scalar,"p.IAmSpeed.WheelSupport.ProjectionMaxRotationDegrees"},
    {141,EKind::Scalar,"p.IAmSpeed.WheelSupport.ProjectionMinGravityAlignment"},
    {142,EKind::Scalar,"p.IAmSpeed.WheelSupport.ProjectionNormalDot"},
    {143,EKind::Integer,"p.IAmSpeed.WheelSupport.ProjectionPasses"},
    {144,EKind::Scalar,"p.IAmSpeed.WheelSupport.ProjectionPatchTravelSlack"},
    {145,EKind::Integer,"p.IAmSpeed.WheelSupport.ProjectionPreserveProbeHits"},
    {146,EKind::Scalar,"p.IAmSpeed.WheelSupport.ProjectionReachSkin"},
    {147,EKind::Scalar,"p.IAmSpeed.WheelSupport.ProjectionRotationLength"},
    {148,EKind::Integer,"p.IAmSpeed.WheelSupport.TransactionalProjection"},
    {149,EKind::Scalar,"p.SkyLeague.Steering.MinForwardSpeed"},
    {150,EKind::Scalar,"p.SkyLeague.Steering.ReleaseInterpSpeed"},
    {151,EKind::Scalar,"p.SkyLeague.Steering.TimeConstant"},
    {152,EKind::Scalar,"p.SkyLeague.Steering.YawScale"},
    {153,EKind::Scalar,"p.SkyLeague.Suspension.BumpStopRate"},
    {154,EKind::Scalar,"p.SkyLeague.Suspension.BumpStopStartCompression"},
    {155,EKind::Scalar,"p.SkyLeague.Suspension.CeilingFadeStartNormalZ"},
    {156,EKind::Integer,"p.SkyLeague.Suspension.ClampNegativeDisplacement"},
    {157,EKind::Integer,"p.SkyLeague.Suspension.ClampPositiveForce"},
    {158,EKind::Scalar,"p.SkyLeague.Suspension.CompressionDampingScale"},
    {159,EKind::Scalar,"p.SkyLeague.Suspension.CompressionQuadraticDamping"},
    {160,EKind::Integer,"p.SkyLeague.Suspension.Debug"},
    {161,EKind::Integer,"p.SkyLeague.Suspension.ForceDebug"},
    {162,EKind::Scalar,"p.SkyLeague.Suspension.InvDotBlend"},
    {163,EKind::Integer,"p.SkyLeague.Suspension.JumpStateClampsNegativeForce"},
    {164,EKind::Scalar,"p.SkyLeague.Suspension.MaxAttractiveForce"},
    {165,EKind::Scalar,"p.SkyLeague.Suspension.MaxAttractiveForceMinNormalZ"},
    {166,EKind::Scalar,"p.SkyLeague.Suspension.NormalBlend"},
    {167,EKind::Scalar,"p.SkyLeague.Suspension.PreRestCompressionDampingScale"},
    {168,EKind::Scalar,"p.SkyLeague.Suspension.ReboundDampingScale"},
    {169,EKind::Integer,"p.SkyLeague.Suspension.RestingForceRequiresCompression"},
    {170,EKind::Scalar,"p.SkyLeague.Suspension.RestingForceScale"},
    {171,EKind::Integer,"p.SkyLeague.Suspension.SkipOverextendedWheelContact"},
    {172,EKind::Scalar,"p.SkyLeague.Suspension.SkipOverextendedWheelContactMaxInwardBodyNormalVelocity"},
    {173,EKind::Scalar,"p.SkyLeague.Suspension.SkipOverextendedWheelContactMaxInwardNormalVelocity"},
    {174,EKind::Scalar,"p.SkyLeague.Suspension.SkipOverextendedWheelContactMinCarUpDot"},
    {175,EKind::Scalar,"p.SkyLeague.Suspension.SkipOverextendedWheelContactMinGroundDot"},
    {176,EKind::Scalar,"p.SkyLeague.Suspension.StiffnessReferenceDisplacement"},
    {177,EKind::Scalar,"p.SkyLeague.Suspension.StiffnessScale"},
}};
namespace Keys {
inline constexpr FKey CVarIAmSpeedAnalyticLandscapeFlatnessTolerance=Catalog[0];
inline constexpr FKey CVarIAmSpeedAutoRecoverContactDebug=Catalog[1];
inline constexpr FKey CVarIAmSpeedBoxGroundSupportDebug=Catalog[2];
inline constexpr FKey CVarIAmSpeedBoxDeferSupportedTransitionVelocityTransport=Catalog[3];
inline constexpr FKey CVarIAmSpeedBoxInertiaScale=Catalog[4];
inline constexpr FKey CVarIAmSpeedBoxSupportedTransitionResponse=Catalog[5];
inline constexpr FKey CVarEnabled=Catalog[6];
inline constexpr FKey CVarIAmSpeedCoupledContactImpulse=Catalog[7];
inline constexpr FKey CVarIAmSpeedCoupledContactImpulseIterations=Catalog[8];
inline constexpr FKey CVarIAmSpeedCoupledContactImpulseMinNormalSpeed=Catalog[9];
inline constexpr FKey CVarIAmSpeedDebugPersistentDynamicPairs=Catalog[10];
inline constexpr FKey CVarIAmSpeedCollisionDebugSphereBoxImpulse=Catalog[11];
inline constexpr FKey CVarIAmSpeedExactBoxRestingSupport=Catalog[12];
inline constexpr FKey CVarIAmSpeedPersistentDynamicPairs=Catalog[13];
inline constexpr FKey CVarRetireSeparatingOwnGroundConstraints=Catalog[14];
inline constexpr FKey CVarIAmSpeedRollingBaumgarte=Catalog[15];
inline constexpr FKey CVarIAmSpeedRollingGentleAcquisitionSpeed=Catalog[16];
inline constexpr FKey CVarIAmSpeedRollingMaxCorrectionSpeed=Catalog[17];
inline constexpr FKey CVarIAmSpeedRollingPenetrationSlop=Catalog[18];
inline constexpr FKey CVarIAmSpeedRollingReleaseSpeed=Catalog[19];
inline constexpr FKey CVarIAmSpeedRollingSeparationTolerance=Catalog[20];
inline constexpr FKey CVarIAmSpeedRollingTangentialFakePhysicsScale=Catalog[21];
inline constexpr FKey CVarSinglePointGroundCoupled=Catalog[22];
inline constexpr FKey CVarIAmSpeedSolverEventTraceFrame=Catalog[23];
inline constexpr FKey CVarIAmSpeedSphereBoxFrictionOverride=Catalog[24];
inline constexpr FKey CVarIAmSpeedSphereBoxManifoldFrictionOverride=Catalog[25];
inline constexpr FKey CVarIAmSpeedSphereBoxNonpenetratingV2=Catalog[26];
inline constexpr FKey CVarIAmSpeedSphereBoxNonpenetratingV2ForTesting=Catalog[27];
inline constexpr FKey CVarIAmSpeedSphereBoxProjection=Catalog[28];
inline constexpr FKey CVarIAmSpeedSphereBoxRestitutionOverride=Catalog[29];
inline constexpr FKey CVarIAmSpeedSphereWorldProjection=Catalog[30];
inline constexpr FKey CVarIAmSpeedUnilateralRollingPairs=Catalog[31];
inline constexpr FKey CVarIAmSpeedCoupledPoseDebug=Catalog[32];
inline constexpr FKey CVarIAmSpeedCoupledSubBodyPoseSolver=Catalog[33];
inline constexpr FKey CVarIAmSpeedCoupledPoseFloorPivotRotationLengthCm=Catalog[34];
inline constexpr FKey CVarIAmSpeedCoupledPoseHitboxSlopCm=Catalog[35];
inline constexpr FKey CVarIAmSpeedCoupledPosePasses=Catalog[36];
inline constexpr FKey CVarIAmSpeedCoupledPoseRotationLengthCm=Catalog[37];
inline constexpr FKey CVarIAmSpeedCoupledPoseSharedWallPlaneRetention=Catalog[38];
inline constexpr FKey CVarIAmSpeedCoupledPoseSupportedSurfaceContactPoint=Catalog[39];
inline constexpr FKey CVarIAmSpeedCoupledPoseVelocityCorrection=Catalog[40];
inline constexpr FKey CVarIAmSpeedCoupledPoseWheelGapCm=Catalog[41];
inline constexpr FKey CVarIAmSpeedCoupledPoseWheelManifoldRotationLengthCm=Catalog[42];
inline constexpr FKey CVarIAmSpeedDebugKinematics=Catalog[43];
inline constexpr FKey CVarIAmSpeedDebugSteering=Catalog[44];
inline constexpr FKey CVarIAmSpeedDebugWheelFriction=Catalog[45];
inline constexpr FKey CVarIAmSpeedDebugWheelFrictionDetailed=Catalog[46];
inline constexpr FKey CVarIAmSpeedEdgeLatchFaceReleaseDot=Catalog[47];
inline constexpr FKey CVarIAmSpeedEdgeRecoverPenetrationTolCm=Catalog[48];
inline constexpr FKey CVarIAmSpeedEdgeRecoverSeparatingSpeedCmS=Catalog[49];
inline constexpr FKey CVarIAmSpeedFaceSupportContactEpsilonCm=Catalog[50];
inline constexpr FKey CVarIAmSpeedFaceSupportPenetrationTolCm=Catalog[51];
inline constexpr FKey CVarIAmSpeedFaceSupportSeparatingSpeedCmS=Catalog[52];
inline constexpr FKey CVarFastEnabled=Catalog[53];
inline constexpr FKey CVarFastMaxFramesPerRun=Catalog[54];
inline constexpr FKey CVarFastReadyDelayCallbacks=Catalog[55];
inline constexpr FKey CVarFastWindowTiming=Catalog[56];
inline constexpr FKey CVarNativeFrameWitnessCompression=Catalog[57];
inline constexpr FKey CVarNativeFrameWitness=Catalog[58];
inline constexpr FKey CVarIAmSpeedGutterDepenMaxUpDeltaCmS=Catalog[59];
inline constexpr FKey CVarIAmSpeedGutterSlideCrossDamping=Catalog[60];
inline constexpr FKey CVarIAmSpeedGutterSlideDamping=Catalog[61];
inline constexpr FKey CVarIAmSpeedGutterSlideMaxClimbBrakeDeltaCmS=Catalog[62];
inline constexpr FKey CVarIAmSpeedGutterSlideMaxClimbSpeedCmS=Catalog[63];
inline constexpr FKey CVarIAmSpeedGutterSlideMaxDeltaCmS=Catalog[64];
inline constexpr FKey CVarIAmSpeedGutterSlideMinSpeedCmS=Catalog[65];
inline constexpr FKey CVarIAmSpeedNearGutterVerticalDepenMaxNormalKillCmS=Catalog[66];
inline constexpr FKey CVarJoinedBaseConsume1430=Catalog[67];
inline constexpr FKey CVarJoinedActors1213=Catalog[68];
inline constexpr FKey CVarFrameParallelCamera=Catalog[69];
inline constexpr FKey CVarFrameCostPhases=Catalog[70];
inline constexpr FKey CVarFrameTiming=Catalog[71];
inline constexpr FKey CVarFrameParallelLanes=Catalog[72];
inline constexpr FKey CVarFrameParallelMaxLanes=Catalog[73];
inline constexpr FKey CVarFrameParallelMinFacetItems=Catalog[74];
inline constexpr FKey CVarFrameParallelMinPrepareItems=Catalog[75];
inline constexpr FKey CVarFrameParallelMinQueryItems=Catalog[76];
inline constexpr FKey CVarPrivateIdleSpinMicros1403=Catalog[77];
inline constexpr FKey CVarIndependentQueryChunkItems1075=Catalog[78];
inline constexpr FKey CVarFrameParallelReservedCores=Catalog[79];
inline constexpr FKey CVarFrameExecutorBackend=Catalog[80];
inline constexpr FKey CVarIAmSpeedRoofSlideSupportMinUpDot=Catalog[81];
inline constexpr FKey CVarActorProfile=Catalog[82];
inline constexpr FKey CVarExecutionMode=Catalog[83];
inline constexpr FKey CVarSimulationPerformanceAudit=Catalog[84];
inline constexpr FKey CVarRealTimeCadenceToleranceMilliseconds=Catalog[85];
inline constexpr FKey CVarRealTimeCadenceWarningCooldownSeconds=Catalog[86];
inline constexpr FKey CVarRealTimeCadenceWarnings=Catalog[87];
inline constexpr FKey CVarRealTimePrecisionWindowMilliseconds=Catalog[88];
inline constexpr FKey CVarRealTimeRecoveryWindowMilliseconds=Catalog[89];
inline constexpr FKey CVarSimulationSlowFrameWarnings=Catalog[90];
inline constexpr FKey CVarSimulationWarningFrameFraction=Catalog[91];
inline constexpr FKey CVarOwnedThreadPaused=Catalog[92];
inline constexpr FKey CVarIAmSpeedSoftWallDepenMaxNormalKillCmS=Catalog[93];
inline constexpr FKey CVarIAmSpeedHullContactContinuousAngularDrag=Catalog[94];
inline constexpr FKey CVarIAmSpeedCovariantVehicleInertia=Catalog[95];
inline constexpr FKey CVarIAmSpeedVertexSupportContactTolCm=Catalog[96];
inline constexpr FKey CVarIAmSpeedVertexSupportSeparatingSpeedCmS=Catalog[97];
inline constexpr FKey CVarIAmSpeedCanonicalWheelSupportMaxAngularSpeed=Catalog[98];
inline constexpr FKey CVarIAmSpeedCanonicalWheelSupportMaxNormalSpeed=Catalog[99];
inline constexpr FKey CVarIAmSpeedCanonicalWheelSupportPose=Catalog[100];
inline constexpr FKey CVarIAmSpeedWheelContactDebug=Catalog[101];
inline constexpr FKey CVarIAmSpeedWheelContactImpulseDeadzone=Catalog[102];
inline constexpr FKey CVarIAmSpeedWheelContactLockedSeparatingDamping=Catalog[103];
inline constexpr FKey CVarIAmSpeedWheelContactLockedSeparatingMaxNormalVelocity=Catalog[104];
inline constexpr FKey CVarIAmSpeedWheelContactLockedSeparatingNormalVelTimeConstant=Catalog[105];
inline constexpr FKey CVarIAmSpeedWheelContactNormalVelTimeConstant=Catalog[106];
inline constexpr FKey CVarIAmSpeedWheelContactNormalVelocityDeadzone=Catalog[107];
inline constexpr FKey CVarIAmSpeedWheelContactRequireFinalSupport=Catalog[108];
inline constexpr FKey CVarIAmSpeedWheelContactSoftness=Catalog[109];
inline constexpr FKey CVarIAmSpeedWheelContactSolverMode=Catalog[110];
inline constexpr FKey CVarIAmSpeedWheelFrameClampLateralImpulse=Catalog[111];
inline constexpr FKey CVarIAmSpeedWheelFrameDebugIntervalFrames=Catalog[112];
inline constexpr FKey CVarIAmSpeedWheelFrameHandbrakeDriveForceBlendModel=Catalog[113];
inline constexpr FKey CVarIAmSpeedWheelFrameHandbrakeDriveForceScale=Catalog[114];
inline constexpr FKey CVarIAmSpeedWheelFrameHandbrakeDriveModel=Catalog[115];
inline constexpr FKey CVarIAmSpeedWheelFrameHandbrakeFrontLateralFrictionFactor=Catalog[116];
inline constexpr FKey CVarIAmSpeedWheelFrameHandbrakeFullInputCOMSpeedModel=Catalog[117];
inline constexpr FKey CVarIAmSpeedWheelFrameHandbrakeLateralImpulseScale=Catalog[118];
inline constexpr FKey CVarIAmSpeedWheelFrameHandbrakeRearLateralFrictionFactor=Catalog[119];
inline constexpr FKey CVarIAmSpeedWheelFrameHandbrakeSteeringBlendModel=Catalog[120];
inline constexpr FKey CVarIAmSpeedWheelFrameHandbrakeSupportProjectionGuard=Catalog[121];
inline constexpr FKey CVarIAmSpeedWheelFrameLateralForwardLossScale=Catalog[122];
inline constexpr FKey CVarIAmSpeedWheelFrameLateralFrictionScale=Catalog[123];
inline constexpr FKey CVarIAmSpeedWheelFrameLongitudinalScrubScale=Catalog[124];
inline constexpr FKey CVarIAmSpeedWheelFrameNormalInputFullFraction=Catalog[125];
inline constexpr FKey CVarIAmSpeedWheelFrameSteeringCurvatureScale=Catalog[126];
inline constexpr FKey CVarIAmSpeedWheelFrameSteeringRearLateralFrictionReverseScale=Catalog[127];
inline constexpr FKey CVarIAmSpeedWheelFrameSteeringRearLateralFrictionScale=Catalog[128];
inline constexpr FKey CVarIAmSpeedWheelFrameTurnSummaryIntervalFrames=Catalog[129];
inline constexpr FKey CVarIAmSpeedWheelFrameUnsteeredAligningYawFullSlipSpeed=Catalog[130];
inline constexpr FKey CVarIAmSpeedWheelFrameUnsteeredAligningYawMaxRate=Catalog[131];
inline constexpr FKey CVarIAmSpeedWheelFrameUnsteeredAligningYawMinSlipSpeed=Catalog[132];
inline constexpr FKey CVarIAmSpeedWheelFrameUnsteeredAligningYawResidualDamping=Catalog[133];
inline constexpr FKey CVarIAmSpeedWheelFrameUnsteeredAligningYawTimeConstant=Catalog[134];
inline constexpr FKey CVarIAmSpeedWheelFrameZeroSteerLateralFrictionScale=Catalog[135];
inline constexpr FKey CVarIAmSpeedCertifiedExtrudedProjection=Catalog[136];
inline constexpr FKey CVarIAmSpeedWheelSupportProjectionCoherentSpringState=Catalog[137];
inline constexpr FKey CVarIAmSpeedWheelSupportProjectionDebug=Catalog[138];
inline constexpr FKey CVarIAmSpeedWheelSupportProjectionMaxGap=Catalog[139];
inline constexpr FKey CVarIAmSpeedWheelSupportProjectionMaxRotationDegrees=Catalog[140];
inline constexpr FKey CVarIAmSpeedWheelSupportProjectionMinGravityAlignment=Catalog[141];
inline constexpr FKey CVarIAmSpeedWheelSupportProjectionNormalDot=Catalog[142];
inline constexpr FKey CVarIAmSpeedWheelSupportProjectionPasses=Catalog[143];
inline constexpr FKey CVarIAmSpeedWheelSupportProjectionPatchTravelSlack=Catalog[144];
inline constexpr FKey CVarIAmSpeedWheelSupportProjectionPreserveProbeHits=Catalog[145];
inline constexpr FKey CVarIAmSpeedWheelSupportProjectionReachSkin=Catalog[146];
inline constexpr FKey CVarIAmSpeedWheelSupportProjectionRotationLength=Catalog[147];
inline constexpr FKey CVarIAmSpeedWheelSupportTransactionalProjection=Catalog[148];
inline constexpr FKey CVarSkyLeagueSteeringMinForwardSpeed=Catalog[149];
inline constexpr FKey CVarSkyLeagueSteeringReleaseInterpSpeed=Catalog[150];
inline constexpr FKey CVarSkyLeagueSteeringTimeConstant=Catalog[151];
inline constexpr FKey CVarSkyLeagueSteeringYawScale=Catalog[152];
inline constexpr FKey CVarSkyLeagueSuspensionBumpStopRate=Catalog[153];
inline constexpr FKey CVarSkyLeagueSuspensionBumpStopStartCompression=Catalog[154];
inline constexpr FKey CVarSkyLeagueSuspensionCeilingFadeStartNormalZ=Catalog[155];
inline constexpr FKey CVarSkyLeagueSuspensionClampNegativeDisplacement=Catalog[156];
inline constexpr FKey CVarSkyLeagueSuspensionClampPositiveForce=Catalog[157];
inline constexpr FKey CVarSkyLeagueSuspensionCompressionDampingScale=Catalog[158];
inline constexpr FKey CVarSkyLeagueSuspensionCompressionQuadraticDamping=Catalog[159];
inline constexpr FKey CVarSkyLeagueSuspensionDebug=Catalog[160];
inline constexpr FKey CVarSkyLeagueSuspensionForceDebug=Catalog[161];
inline constexpr FKey CVarSkyLeagueSuspensionInvDotBlend=Catalog[162];
inline constexpr FKey CVarSkyLeagueSuspensionJumpStateClampsNegativeForce=Catalog[163];
inline constexpr FKey CVarSkyLeagueSuspensionMaxAttractiveForce=Catalog[164];
inline constexpr FKey CVarSkyLeagueSuspensionMaxAttractiveForceMinNormalZ=Catalog[165];
inline constexpr FKey CVarSkyLeagueSuspensionNormalBlend=Catalog[166];
inline constexpr FKey CVarSkyLeagueSuspensionPreRestCompressionDampingScale=Catalog[167];
inline constexpr FKey CVarSkyLeagueSuspensionReboundDampingScale=Catalog[168];
inline constexpr FKey CVarSkyLeagueSuspensionRestingForceRequiresCompression=Catalog[169];
inline constexpr FKey CVarSkyLeagueSuspensionRestingForceScale=Catalog[170];
inline constexpr FKey CVarSkyLeagueSuspensionSkipOverextendedWheelContact=Catalog[171];
inline constexpr FKey CVarSkyLeagueSuspensionSkipOverextendedWheelContactMaxInwardBodyNormalVelocity=Catalog[172];
inline constexpr FKey CVarSkyLeagueSuspensionSkipOverextendedWheelContactMaxInwardNormalVelocity=Catalog[173];
inline constexpr FKey CVarSkyLeagueSuspensionSkipOverextendedWheelContactMinCarUpDot=Catalog[174];
inline constexpr FKey CVarSkyLeagueSuspensionSkipOverextendedWheelContactMinGroundDot=Catalog[175];
inline constexpr FKey CVarSkyLeagueSuspensionStiffnessReferenceDisplacement=Catalog[176];
inline constexpr FKey CVarSkyLeagueSuspensionStiffnessScale=Catalog[177];
}
}
