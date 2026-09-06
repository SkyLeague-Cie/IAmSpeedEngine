#pragma once

#include "CoreMinimal.h"

namespace Speed::Analytic
{

constexpr uint32 AnalyticWorldSchemaVersion = 6u;
constexpr double ExtrudedQuinticChordToleranceCm = 0.001;
constexpr double TensorBezierQueryApproximationToleranceCm = 0.01;
// Authority-pruned runtime worlds do not retain residual triangle providers.
// A sub-millimetre certified tensor envelope is sufficient for their contact
// queries and avoids rebuilding millions of transient facets at map load.
constexpr double AuthorityTensorBezierQueryApproximationToleranceCm = 0.05;

enum class ESurfaceFamily : uint8
{
	BoundedPlane = 0,
	TriangleMesh = 1,
	ExtrudedQuintic = 2,
};

struct IAMSPEED_API FTriangleSurface
{
	uint64 SourceId = 0;
	uint64 SurfaceId = 0;
	uint64 FeatureId = 0;
	uint64 PrimitiveId = 0;
	uint32 MaterialId = 0;
	uint32 ObjectType = 0;
	uint64 BlockingChannels = 0;
	FVector3d Vertices[3] = {
		FVector3d::ZeroVector, FVector3d::ZeroVector, FVector3d::ZeroVector };
	FVector3d VertexNormals[3] = {
		FVector3d::UpVector, FVector3d::UpVector, FVector3d::UpVector };
	FVector3d FaceNormal = FVector3d::UpVector;
	FBox3d Bounds = FBox3d(EForceInit::ForceInit);
	bool bQueryCollisionEnabled = false;
	bool bAuthorityEligible = false;

	bool IsValid(FString* OutReason = nullptr) const;
};

struct IAMSPEED_API FTriangleBvhNode
{
	FBox3d Bounds = FBox3d(EForceInit::ForceInit);
	int32 FirstIndex = 0;
	int32 IndexCount = 0;
	int32 LeftChild = INDEX_NONE;
	int32 RightChild = INDEX_NONE;

	bool IsLeaf() const { return IndexCount > 0; }
};

enum class EEdgeContinuity : uint8
{
	Boundary = 0,
	Smooth = 1,
	Crease = 2,
	NonManifold = 3,
};

struct IAMSPEED_API FTriangleMeshEdge
{
	int32 VertexA = INDEX_NONE;
	int32 VertexB = INDEX_NONE;
	int32 FirstIncidentTriangle = 0;
	int32 IncidentTriangleCount = 0;
	double DihedralAngleDegrees = 0.0;
	double MaximumNormalDiscontinuityDegrees = 0.0;
	EEdgeContinuity Continuity = EEdgeContinuity::Boundary;

	bool IsBoundary() const { return IncidentTriangleCount == 1; }
	bool IsManifold() const { return IncidentTriangleCount == 2; }
	bool IsNonManifold() const { return IncidentTriangleCount > 2; }
};

struct IAMSPEED_API FSmoothSurfaceRegion
{
	uint64 RegionId = 0;
	uint64 SourceId = 0;
	uint64 SurfaceId = 0;
	uint32 MaterialId = 0;
	int32 FirstTriangleIndex = 0;
	int32 TriangleCount = 0;
	double Area = 0.0;
	double MaximumFaceNormalAngleDegrees = 0.0;
	FVector3d SeedNormal = FVector3d::UpVector;
	FBox3d Bounds = FBox3d(EForceInit::ForceInit);
};

struct IAMSPEED_API FVertexShapeSample
{
	// Diagnostic positional estimator. Normal is reconstructed from incident
	// face geometry; imported shading normals are not curvature authority.
	bool bValid = false;
	int32 NeighborCount = 0;
	int32 SupportRingCount = 0;
	double SupportRadiusCm = 0.0;
	double MinimumPrincipalCurvature = 0.0;
	double MaximumPrincipalCurvature = 0.0;
	double MeanCurvature = 0.0;
	double GaussianCurvature = 0.0;
	double RootMeanSquareResidual = 0.0;
	FVector3d Normal = FVector3d::UpVector;
	FVector3d MinimumDirection = FVector3d::ForwardVector;
	FVector3d MaximumDirection = FVector3d::RightVector;
};

enum class ESurfacePatchKind : uint8
{
	Residual = 0,
	PlanarCandidate = 1,
};

struct IAMSPEED_API FSurfacePatch
{
	uint64 PatchId = 0;
	uint64 SourceId = 0;
	uint64 SurfaceId = 0;
	uint32 MaterialId = 0;
	ESurfacePatchKind Kind = ESurfacePatchKind::Residual;
	int32 FirstTriangleIndex = 0;
	int32 TriangleCount = 0;
	double Area = 0.0;
	double MaximumPlaneResidual = 0.0;
	double MaximumNormalAngleDegrees = 0.0;
	FVector3d Origin = FVector3d::ZeroVector;
	FVector3d Normal = FVector3d::UpVector;
	FBox3d Bounds = FBox3d(EForceInit::ForceInit);
};

struct IAMSPEED_API FPlanarSurfaceGroup
{
	uint64 GroupId = 0;
	uint64 SourceId = 0;
	uint64 SurfaceId = 0;
	uint32 MaterialId = 0;
	int32 FirstPatchIndex = 0;
	int32 PatchCount = 0;
	int32 TriangleCount = 0;
	double Area = 0.0;
	double PlaneOffset = 0.0;
	double MaximumPlaneResidual = 0.0;
	double MaximumNormalAngleDegrees = 0.0;
	FVector3d Normal = FVector3d::UpVector;
	FBox3d Bounds = FBox3d(EForceInit::ForceInit);
	bool bArchitecturalConstraint = false;
};

enum class EPlanarSymmetryAxis : uint8
{
	X = 0,
	Y = 1,
};

struct IAMSPEED_API FPlanarGroupMirrorMatch
{
	EPlanarSymmetryAxis Axis = EPlanarSymmetryAxis::X;
	int32 SourceGroupIndex = INDEX_NONE;
	int32 TargetGroupIndex = INDEX_NONE;
	double PlaneOffsetResidual = 0.0;
	double NormalAngleResidualDegrees = 0.0;
	double BoundsResidual = 0.0;
	double RelativeAreaResidual = 0.0;
	double Score = 0.0;
	bool bReciprocal = false;
};

enum class ECurvatureEvidenceKind : uint8
{
	Unresolved = 0,
	PlanarConstraint = 1,
	DevelopableCandidate = 2,
	EllipticCandidate = 3,
	SaddleCandidate = 4,
};

struct IAMSPEED_API FTriangleCurvatureEvidence
{
	ECurvatureEvidenceKind Kind = ECurvatureEvidenceKind::Unresolved;
	int32 ValidSampleCount = 0;
	double MinimumPrincipalCurvature = 0.0;
	double MaximumPrincipalCurvature = 0.0;
	double MaximumAbsoluteCurvature = 0.0;
	double CurvatureAnisotropy = 1.0;
	double MaximumRelativeFitResidual = 0.0;
	FVector3d LowCurvatureDirection = FVector3d::ForwardVector;
	bool bReliable = false;
};

struct IAMSPEED_API FCurvatureSurfaceRegion
{
	uint64 RegionId = 0;
	uint64 SourceId = 0;
	uint64 SurfaceId = 0;
	uint32 MaterialId = 0;
	ECurvatureEvidenceKind Kind = ECurvatureEvidenceKind::Unresolved;
	int32 FirstTriangleIndex = 0;
	int32 TriangleCount = 0;
	double Area = 0.0;
	double MeanMinimumPrincipalCurvature = 0.0;
	double MeanMaximumPrincipalCurvature = 0.0;
	double MinimumAbsoluteDominantCurvature = TNumericLimits<double>::Max();
	double MaximumAbsoluteDominantCurvature = 0.0;
	double MaximumRelativeFitResidual = 0.0;
	FVector3d MeanLowCurvatureDirection = FVector3d::ForwardVector;
	FBox3d Bounds = FBox3d(EForceInit::ForceInit);
};

struct IAMSPEED_API FCurvatureRegionMirrorMatch
{
	EPlanarSymmetryAxis Axis = EPlanarSymmetryAxis::X;
	int32 SourceRegionIndex = INDEX_NONE;
	int32 TargetRegionIndex = INDEX_NONE;
	double MinimumCurvatureResidual = 0.0;
	double MaximumCurvatureResidual = 0.0;
	double LowDirectionAngleResidualDegrees = 0.0;
	double BoundsResidual = 0.0;
	double RelativeAreaResidual = 0.0;
	double Score = 0.0;
	bool bReciprocal = false;
	bool bSelfMirror = false;
	bool bPlausible = false;
};

struct IAMSPEED_API FTriangleExtrusionEvidence
{
	bool bCandidate = false;
	double AspectRatio = 0.0;
	double AxialLength = 0.0;
	double CrossSectionLength = 0.0;
	FVector3d Axis = FVector3d::ForwardVector;
};

struct IAMSPEED_API FExtrusionSurfaceRegion
{
	uint64 RegionId = 0;
	uint64 SourceId = 0;
	uint64 SurfaceId = 0;
	uint32 MaterialId = 0;
	int32 FirstTriangleIndex = 0;
	int32 TriangleCount = 0;
	double Area = 0.0;
	double MinimumAspectRatio = TNumericLimits<double>::Max();
	double MaximumAxisDeviationDegrees = 0.0;
	FVector3d Axis = FVector3d::ForwardVector;
	FBox3d Bounds = FBox3d(EForceInit::ForceInit);
	bool bQuarterEllipseFitValid = false;
	bool bQuarterEllipsePlausible = false;
	bool bEllipseCenterAtMaximumU = false;
	bool bEllipseCenterAtMaximumV = false;
	FVector3d SectionAxisU = FVector3d::RightVector;
	FVector3d SectionAxisV = FVector3d::UpVector;
	FVector2d EllipseCenterCoordinates = FVector2d::ZeroVector;
	double EllipseRadiusU = 0.0;
	double EllipseRadiusV = 0.0;
	double EllipseRootMeanSquareResidualCm = 0.0;
	double EllipseMaximumResidualCm = 0.0;
};

struct IAMSPEED_API FExtrusionRegionMirrorMatch
{
	EPlanarSymmetryAxis Axis = EPlanarSymmetryAxis::X;
	int32 SourceRegionIndex = INDEX_NONE;
	int32 TargetRegionIndex = INDEX_NONE;
	double AxisAngleResidualDegrees = 0.0;
	double CenterLineResidualCm = 0.0;
	double RelativeRadiusUResidual = 0.0;
	double RelativeRadiusVResidual = 0.0;
	double BoundsResidualCm = 0.0;
	double RelativeAreaResidual = 0.0;
	double Score = 0.0;
	bool bReciprocal = false;
	bool bSelfMirror = false;
	bool bPlausible = false;
};

enum class EQuarterEllipseEndpoint : uint8
{
	U = 0,
	V = 1,
};

struct IAMSPEED_API FQuarterEllipseBoundaryMatch
{
	int32 ExtrusionRegionIndex = INDEX_NONE;
	int32 PlanarGroupIndex = INDEX_NONE;
	EQuarterEllipseEndpoint Endpoint = EQuarterEllipseEndpoint::U;
	FVector3d EndpointPosition = FVector3d::ZeroVector;
	double PositionResidualCm = 0.0;
	double BoundsResidualCm = 0.0;
	double NormalAngleResidualDegrees = 0.0;
	double Score = 0.0;
	bool bC0Plausible = false;
	bool bG1Plausible = false;
};

// Recognition-only comparison profile. The section is a quintic Bezier whose
// first three control points are collinear and equally spaced at each planar
// endpoint. Position, tangent and second derivative can therefore join a line
// exactly. It is a C2 candidate family, not a continuity certificate or a
// runtime query surface.
struct IAMSPEED_API FC2TransitionSectionFit
{
	int32 ExtrusionRegionIndex = INDEX_NONE;
	int32 BoundaryMatchUIndex = INDEX_NONE;
	int32 BoundaryMatchVIndex = INDEX_NONE;
	FVector2d CenterCoordinates = FVector2d::ZeroVector;
	double RadiusU = 0.0;
	double RadiusV = 0.0;
	double SignU = 1.0;
	double SignV = 1.0;
	double FlatteningFraction = 0.0;
	double RootMeanSquareResidualCm = 0.0;
	double MaximumResidualCm = 0.0;
	double MaximumEndpointPlaneResidualCm = 0.0;
	double MaximumEndpointNormalAdjustmentDegrees = 0.0;
	bool bBoundaryEvidenceUsable = false;
	bool bZeroEndpointCurvatureByConstruction = false;

	FVector2d EvaluatePosition(double T) const;
	FVector2d EvaluateFirstDerivative(double T) const;
	FVector2d EvaluateSecondDerivative(double T) const;
};

struct IAMSPEED_API FSharedC2TransitionPairFit
{
	EPlanarSymmetryAxis SymmetryAxis = EPlanarSymmetryAxis::X;
	int32 RegionAIndex = INDEX_NONE;
	int32 RegionBIndex = INDEX_NONE;
	double SharedRadiusU = 0.0;
	double SharedRadiusV = 0.0;
	double SharedFlatteningFraction = 0.0;
	double RegionARootMeanSquareResidualCm = 0.0;
	double RegionAMaximumResidualCm = 0.0;
	double RegionBRootMeanSquareResidualCm = 0.0;
	double RegionBMaximumResidualCm = 0.0;
	double BalancedRootMeanSquareResidualCm = 0.0;
	double MaximumEndpointNormalAdjustmentDegrees = 0.0;
	bool bBothBoundaryEvidenceUsable = false;
	bool bPlausible = false;
};

struct IAMSPEED_API FSharedC2FrameLedgerEntry
{
	int32 SharedPairFitIndex = INDEX_NONE;
	FVector3d RegularizedAxisA = FVector3d::ForwardVector;
	FVector3d RegularizedAxisB = FVector3d::ForwardVector;
	double MaximumExtrusionAxisAdjustmentDegrees = 0.0;
	double MaximumPlaneNormalAdjustmentDegrees = 0.0;
	double PlanarSourceRootMeanSquareResidualCm = 0.0;
	double PlanarSourceMaximumResidualCm = 0.0;
	double MaximumOriginalPlaneEndpointResidualCm = 0.0;
	double MaximumConstructedEndpointPositionResidualCm = 0.0;
	double MaximumEndpointTangentDirectionResidual = 0.0;
	double MinimumEndpointTangentMagnitude = 0.0;
	double MaximumEndpointSecondDerivativeMagnitude = 0.0;
	bool bC0ByConstruction = false;
	bool bG1ByConstruction = false;
	bool bG2ByConstruction = false;
	bool bParametricC2ByConstruction = false;
	bool bSourceRegularizationPlausible = false;
};

// Recognition-only network merge of every pair-local request that addresses
// the same architectural plane. This is the first place where floor/wall/
// ceiling constraints become unique across several transition pairs.
struct IAMSPEED_API FGlobalC2PlaneConstraint
{
	int32 PlanarGroupIndex = INDEX_NONE;
	int32 RequestCount = 0;
	FVector3d Normal = FVector3d::UpVector;
	double PlaneOffset = 0.0;
	double MaximumRequestedNormalConflictDegrees = 0.0;
	double MaximumRequestedEndpointResidualCm = 0.0;
	double SourceRootMeanSquareResidualCm = 0.0;
	double SourceMaximumResidualCm = 0.0;
	double SourceNormalAdjustmentDegrees = 0.0;
	bool bConstraintCompatible = false;
};

struct IAMSPEED_API FGlobalC2JoinCertificate
{
	int32 SharedPairFitIndex = INDEX_NONE;
	int32 BoundaryCount = 0;
	double MaximumEndpointPlaneResidualCm = 0.0;
	double MaximumTangentNormalResidualDegrees = 0.0;
	bool bNetworkC0Plausible = false;
	bool bNetworkG1Plausible = false;
	bool bLocalC2Retained = false;
};

struct IAMSPEED_API FSymmetrizedC2PlaneConstraint
{
	uint64 OrbitId = 0;
	int32 PlanarGroupIndex = INDEX_NONE;
	uint8 TransformMaskFromOrbitRoot = 0;
	FVector3d Normal = FVector3d::UpVector;
	double PlaneOffset = 0.0;
	double SourceNormalAdjustmentDegrees = 0.0;
	double SourceRootMeanSquareResidualCm = 0.0;
	double SourceMaximumResidualCm = 0.0;
	double MaximumMirrorNormalResidualDegrees = 0.0;
	double MaximumMirrorOffsetResidualCm = 0.0;
	bool bSourceFitPlausible = false;
	bool bExactMirrorPlacement = false;
};

struct IAMSPEED_API FC2SymmetryPlacementFrame
{
	FVector2d BoundsCenter = FVector2d::ZeroVector;
	FVector2d SourceInferredCenter = FVector2d::ZeroVector;
	FVector2d TargetCenter = FVector2d::ZeroVector;
	int32 XConstraintCount = 0;
	int32 YConstraintCount = 0;
	double XRootMeanSquareResidualCm = 0.0;
	double XMaximumResidualCm = 0.0;
	double YRootMeanSquareResidualCm = 0.0;
	double YMaximumResidualCm = 0.0;
	bool bFittedFromMirrorPlanes = false;
};

// Recognition-only exact-boundary reconstruction. Coordinates are the two
// unique global plane equations themselves; the dual basis maps them back to
// world space, so each endpoint and tangent lies on its incident plane even
// when the measured plane normals are not perfectly orthogonal.
struct IAMSPEED_API FCoupledC2TransitionPairSolution
{
	int32 SharedPairFitIndex = INDEX_NONE;
	int32 RegionAPlaneUIndex = INDEX_NONE;
	int32 RegionAPlaneVIndex = INDEX_NONE;
	int32 RegionBPlaneUIndex = INDEX_NONE;
	int32 RegionBPlaneVIndex = INDEX_NONE;
	FVector2d RegionACenterCoordinates = FVector2d::ZeroVector;
	FVector2d RegionBCenterCoordinates = FVector2d::ZeroVector;
	double SharedRadiusU = 0.0;
	double SharedRadiusV = 0.0;
	double SharedFlatteningFraction = 0.0;
	double RegionARootMeanSquareResidualCm = 0.0;
	double RegionAMaximumResidualCm = 0.0;
	double RegionBRootMeanSquareResidualCm = 0.0;
	double RegionBMaximumResidualCm = 0.0;
	double BalancedRootMeanSquareResidualCm = 0.0;
	double MaximumPlaneOrthogonalityResidual = 0.0;
	double MaximumEndpointPlaneResidualCm = 0.0;
	double MaximumEndpointTangentPlaneResidual = 0.0;
	double MaximumEndpointSecondDerivativeMagnitude = 0.0;
	bool bExactC0G1C2ByConstruction = false;
	bool bSourceFitPlausible = false;
};

struct IAMSPEED_API FCoupledC2TransitionFamilySolution
{
	uint64 FamilyId = 0;
	int32 FirstRegionIndex = 0;
	int32 RegionCount = 0;
	int32 PairConstraintCount = 0;
	uint8 SymmetryAxisMask = 0;
	double SharedRadiusU = 0.0;
	double SharedRadiusV = 0.0;
	double SharedFlatteningFraction = 0.0;
	double BalancedRootMeanSquareResidualCm = 0.0;
	double MaximumRegionRootMeanSquareResidualCm = 0.0;
	double MaximumResidualCm = 0.0;
	double MaximumEndpointPlaneResidualCm = 0.0;
	double MaximumEndpointTangentPlaneResidual = 0.0;
	double MaximumEndpointSecondDerivativeMagnitude = 0.0;
	bool bExactC0G1C2ByConstruction = false;
	bool bSourceFitPlausible = false;
};

enum class EC2TransitionCoverageKind : uint8
{
	CoveredFamily = 0,
	AxisAlignedSelfMirror = 1,
	DiagonalCornerCandidate = 2,
	UnpairedAxisAligned = 3,
	Other = 4,
};

enum class EC2TransitionSurfaceLayer : uint8
{
	Unknown = 0,
	PlayableInner = 1,
	OuterBacking = 2,
};

struct IAMSPEED_API FC2TransitionCoverageEntry
{
	int32 TransitionFitIndex = INDEX_NONE;
	int32 ExtrusionRegionIndex = INDEX_NONE;
	int32 FamilyIndex = INDEX_NONE;
	int32 XMirrorTargetRegionIndex = INDEX_NONE;
	int32 YMirrorTargetRegionIndex = INDEX_NONE;
	EC2TransitionCoverageKind Kind = EC2TransitionCoverageKind::Other;
	EC2TransitionSurfaceLayer SurfaceLayer = EC2TransitionSurfaceLayer::Unknown;
	int32 HorizontalPlanarGroupIndex = INDEX_NONE;
	double HorizontalPlaneHeightCm = 0.0;
	double MaximumBoundaryPositionResidualCm = 0.0;
	double MaximumBoundaryNormalResidualDegrees = 0.0;
	bool bXMirrorPlausible = false;
	bool bYMirrorPlausible = false;
	bool bXSelfMirror = false;
	bool bYSelfMirror = false;
};

struct IAMSPEED_API FPlayableC2OrbitMember
{
	int32 ExtrusionRegionIndex = INDEX_NONE;
	uint8 TransformMaskFromSeed = 0;
	double RootMeanSquareResidualCm = 0.0;
	double MaximumResidualCm = 0.0;
};

// Recognition-only synthesis from one playable transition seed. Members are
// reflected into the seed frame and receive one shared quintic section. A
// candidate remains rejected when the region mirror graph is not geometrically
// equivalent; later triangle-level correspondence may then replace that graph.
struct IAMSPEED_API FPlayableC2OrbitCandidate
{
	uint64 OrbitId = 0;
	int32 SeedCoverageIndex = INDEX_NONE;
	int32 FirstMemberIndex = 0;
	int32 MemberCount = 0;
	int32 ExpectedMemberCount = 0;
	uint8 SymmetryAxisMask = 0;
	double SharedRadiusU = 0.0;
	double SharedRadiusV = 0.0;
	double SharedFlatteningFraction = 0.0;
	double BalancedRootMeanSquareResidualCm = 0.0;
	double MaximumRegionRootMeanSquareResidualCm = 0.0;
	double MaximumResidualCm = 0.0;
	double MaximumMirrorAxisResidualDegrees = 0.0;
	double MaximumMirrorCenterLineResidualCm = 0.0;
	double MaximumMirrorBoundsResidualCm = 0.0;
	double MaximumMirrorRelativeAreaResidual = 0.0;
	double MaximumEndpointPlaneResidualCm = 0.0;
	double MaximumEndpointTangentPlaneResidual = 0.0;
	double MaximumEndpointSecondDerivativeMagnitude = 0.0;
	bool bOrbitComplete = false;
	bool bMirrorTopologyPlausible = false;
	bool bExactC0G1C2ByConstruction = false;
	bool bSourceFitPlausible = false;
};

struct IAMSPEED_API FPlayableC2TriangleSupportMember
{
	uint8 TransformMaskFromSeed = 0;
	int32 FirstCanonicalPointIndex = 0;
	int32 CanonicalPointCount = 0;
	int32 SampleCount = 0;
	int32 MatchedSampleCount = 0;
	int32 UniqueTargetTriangleCount = 0;
	double RootMeanSquarePositionResidualCm = 0.0;
	double MaximumPositionResidualCm = 0.0;
	double RootMeanSquareNormalResidualDegrees = 0.0;
	double MaximumNormalResidualDegrees = 0.0;
	double ProfileRootMeanSquareResidualCm = 0.0;
	double ProfileMaximumResidualCm = 0.0;
	bool bComplete = false;
};

// Region segmentation is allowed to differ across mirror quadrants. This
// recognition-only ledger reflects one seed sampling into each requested
// quadrant and finds the corresponding surface on the same smooth shell.
struct IAMSPEED_API FPlayableC2TriangleSupportCandidate
{
	uint64 OrbitId = 0;
	int32 SeedCoverageIndex = INDEX_NONE;
	int32 SmoothRegionIndex = INDEX_NONE;
	int32 FirstMemberIndex = 0;
	int32 MemberCount = 0;
	int32 ExpectedMemberCount = 0;
	double MaximumMemberRootMeanSquarePositionResidualCm = 0.0;
	double MaximumPositionResidualCm = 0.0;
	double MaximumMemberRootMeanSquareNormalResidualDegrees = 0.0;
	double MaximumNormalResidualDegrees = 0.0;
	double SharedRadiusU = 0.0;
	double SharedRadiusV = 0.0;
	double SharedFlatteningFraction = 0.0;
	double BalancedProfileRootMeanSquareResidualCm = 0.0;
	double MaximumMemberProfileRootMeanSquareResidualCm = 0.0;
	double MaximumProfileResidualCm = 0.0;
	double MaximumEndpointPlaneResidualCm = 0.0;
	double MaximumEndpointTangentPlaneResidual = 0.0;
	double MaximumEndpointSecondDerivativeMagnitude = 0.0;
	bool bComplete = false;
	bool bSupportPlausible = false;
	bool bExactC0G1C2ByConstruction = false;
	bool bSourceFitPlausible = false;
	double NetworkSharedRadiusU = 0.0;
	double NetworkSharedRadiusV = 0.0;
	double NetworkSharedFlatteningFraction = 0.0;
	double NetworkBalancedProfileRootMeanSquareResidualCm = 0.0;
	double NetworkMaximumMemberProfileRootMeanSquareResidualCm = 0.0;
	double NetworkMaximumProfileResidualCm = 0.0;
	double NetworkMaximumEndpointPlaneResidualCm = 0.0;
	double NetworkMaximumEndpointTangentPlaneResidual = 0.0;
	double NetworkMaximumEndpointSecondDerivativeMagnitude = 0.0;
	bool bNetworkExactC0G1C2ByConstruction = false;
	bool bNetworkSourceFitPlausible = false;
};

// Recognition-only binding between each incident plane of a playable C2
// orbit and the already symmetrized global plane network. A compatible match
// may reuse a geometrically identical plane even when source segmentation gave
// it a different planar group.
struct IAMSPEED_API FPlayableC2PlaneBinding
{
	int32 TriangleSupportCandidateIndex = INDEX_NONE;
	EQuarterEllipseEndpoint Endpoint = EQuarterEllipseEndpoint::U;
	int32 SourcePlanarGroupIndex = INDEX_NONE;
	int32 SymmetrizedConstraintIndex = INDEX_NONE;
	FVector3d EndpointPosition = FVector3d::ZeroVector;
	double NormalResidualDegrees = 0.0;
	double EndpointPlaneResidualCm = 0.0;
	double OrientedOffsetResidualCm = 0.0;
	bool bSameSourcePlanarGroup = false;
	bool bCompatible = false;
	int32 NetworkPlaneConstraintIndex = INDEX_NONE;
	double NetworkNormalResidualDegrees = 0.0;
	double NetworkEndpointPlaneResidualCm = 0.0;
	double NetworkOrientedOffsetResidualCm = 0.0;
	bool bNetworkCompatible = false;
};

// Recognition-only union of the established symmetrized plane constraints and
// any deterministic symmetry orbit required by the playable C2 candidates.
// Synthetic members may have no direct source planar group in a quadrant.
struct IAMSPEED_API FPlayableC2NetworkPlaneConstraint
{
	uint64 OrbitId = 0;
	int32 SourcePlanarGroupIndex = INDEX_NONE;
	uint8 TransformMaskFromOrbitRoot = 0;
	FVector3d Normal = FVector3d::UpVector;
	double PlaneOffset = 0.0;
	int32 SourceObservationCount = 0;
	double MaximumSourceNormalResidualDegrees = 0.0;
	double MaximumSourceOffsetResidualCm = 0.0;
	bool bInheritedFromSymmetrizedNetwork = false;
	bool bSourceFitPlausible = false;
	bool bExactMirrorPlacement = false;
};

// Recognition-only connected interface between a principal vertical planar
// group and the non-planar mesh attached to it. Opening/boundary plane semantics are
// assigned only after comparing both horizontal axes and their protrusions.
struct IAMSPEED_API FEndWallBoundaryComponent
{
	uint64 ComponentId = 0;
	int32 NetworkPlaneConstraintIndex = INDEX_NONE;
	int32 PlanarGroupIndex = INDEX_NONE;
	int32 FirstEdgeIndex = 0;
	int32 EdgeCount = 0;
	int32 UniqueVertexCount = 0;
	int32 AdjacentTriangleCount = 0;
	double PerimeterCm = 0.0;
	FBox3d Bounds = FBox3d(EForceInit::ForceInit);
	EC2TransitionSurfaceLayer SurfaceLayer =
		EC2TransitionSurfaceLayer::Unknown;
	bool bClosedLoop = false;
	bool bCrossesXSymmetryPlane = false;
};

// Residual inner chain after removing the four outer envelope sides of a
// principal wall boundary. On the longitudinal playable wall this is the
// topology-derived opening-mouth rim (vertical segments plus horizontal span), open at its baseline.
struct IAMSPEED_API FOpenRimCandidate
{
	uint64 CandidateId = 0;
	int32 WallBoundaryComponentIndex = INDEX_NONE;
	int32 FirstEdgeIndex = 0;
	int32 EdgeCount = 0;
	int32 UniqueVertexCount = 0;
	double ArcLengthCm = 0.0;
	FBox3d Bounds = FBox3d(EForceInit::ForceInit);
	double BaselineHeightCm = 0.0;
	uint8 WallAxis = 0;
	EC2TransitionSurfaceLayer SurfaceLayer =
		EC2TransitionSurfaceLayer::Unknown;
	int32 LongitudinalMirrorCandidateIndex = INDEX_NONE;
	double LongitudinalMirrorRootMeanSquareResidualCm = 0.0;
	double LongitudinalMirrorMaximumResidualCm = 0.0;
	double TransverseSelfRootMeanSquareResidualCm = 0.0;
	double TransverseSelfMaximumResidualCm = 0.0;
	int32 NegativeVerticalEdgeCount = 0;
	int32 PositiveVerticalEdgeCount = 0;
	int32 HorizontalSpanEdgeCount = 0;
	int32 UpperTransitionEdgeCount = 0;
	double HorizontalSpanHeightCm = 0.0;
	double HorizontalSpanSpanCm = 0.0;
	double NegativeVerticalHeightCm = 0.0;
	double PositiveVerticalHeightCm = 0.0;
	bool bOpenAtBaseline = false;
	bool bCrossesTransverseSymmetryPlane = false;
	bool bLongitudinalMirrorPlausible = false;
	bool bTransverseSelfPlausible = false;
	bool bFeaturePartitionComplete = false;
};

// One equally weighted half-rim observation used by the shared playable opening
// fit. Transverse sign is removed before fitting, so two opening ends times two
// sides constrain one exact X/Y-symmetric canonical profile.
struct IAMSPEED_API FCanonicalOpenArchMemberFit
{
	int32 OpenRimCandidateIndex = INDEX_NONE;
	uint8 TransverseSide = 0;
	int32 SampleCount = 0;
	double RootMeanSquareResidualCm = 0.0;
	double MaximumResidualCm = 0.0;
};

// Recognition-only composite opening boundary: vertical vertical segment, quintic upper
// corner and horizontal horizontal span. The quintic has zero second derivative at
// both endpoints, making the two joins C0/G1/C2 by construction. This record
// is deliberately not a runtime query surface or authority candidate.
struct IAMSPEED_API FCanonicalOpenArchSolution
{
	uint64 SolutionId = 0;
	int32 FirstMemberIndex = 0;
	int32 MemberCount = 0;
	double BaselineHeightCm = 0.0;
	double VerticalSegmentHalfWidthCm = 0.0;
	double VerticalSegmentTransitionHeightCm = 0.0;
	double HorizontalSpanHalfSpanCm = 0.0;
	double HorizontalSpanHeightCm = 0.0;
	double FlatteningFraction = 0.0;
	double BalancedRootMeanSquareResidualCm = 0.0;
	double MaximumMemberRootMeanSquareResidualCm = 0.0;
	double MaximumResidualCm = 0.0;
	double MaximumEndpointPositionResidualCm = 0.0;
	double MaximumEndpointTangentResidualDegrees = 0.0;
	double MaximumEndpointSecondDerivativeMagnitude = 0.0;
	bool bCompleteFourMemberOrbit = false;
	bool bExactXYMirrorPlacement = false;
	bool bExactC0G1C2ByConstruction = false;
	bool bSourceRegularizationPlausible = false;
};

enum class EOpenRimFeature : uint8
{
	NegativeVertical = 0,
	PositiveVertical = 1,
	HorizontalSpan = 2,
	UpperTransition = 3,
};

// Recognition-only first surface strip across a opening-rim edge. It records the
// planar boundary plane face and the immediately adjacent three-dimensional opening
// face without growing an unbounded region or assigning collision authority.
struct IAMSPEED_API FOpenRimSurfaceBandObservation
{
	int32 OpenRimCandidateIndex = INDEX_NONE;
	int32 RimEdgeIndex = INDEX_NONE;
	int32 BoundaryPlaneTriangleIndex = INDEX_NONE;
	int32 OpeningSurfaceTriangleIndex = INDEX_NONE;
	EOpenRimFeature Feature = EOpenRimFeature::UpperTransition;
	double EdgeLengthCm = 0.0;
	double FaceDihedralDegrees = 0.0;
	double ImportedNormalJumpDegrees = 0.0;
	double FirstLongitudinalStepCm = 0.0;
	double FirstStepAxisResidualDegrees = 0.0;
	bool bSameSmoothRegion = false;
	bool bSourceG1Plausible = false;
};

// Recognition-only planar section through a playable vertical segment or horizontal span fillet.
// Local X is longitudinal depth into the opening and local Y moves from the rim
// toward the opening. Points are the connected source-mesh section selected at
// the rim; no fitted surface or runtime query consumes them.
struct IAMSPEED_API FOpenRimTransverseSection
{
	uint64 SectionId = 0;
	int32 OpenRimCandidateIndex = INDEX_NONE;
	int32 RimEdgeIndex = INDEX_NONE;
	int32 InterpolationRimEdgeA = INDEX_NONE;
	int32 InterpolationRimEdgeB = INDEX_NONE;
	double InterpolationAlpha = 0.0;
	EOpenRimFeature Feature = EOpenRimFeature::UpperTransition;
	double SliceCoordinateCm = 0.0;
	FVector3d SliceOrigin = FVector3d::ZeroVector;
	FVector3d SlicePlaneNormal = FVector3d::ZeroVector;
	FVector3d OpeningDirection = FVector3d::ZeroVector;
	double CanonicalRimParameter = 0.0;
	double TopologyCanonicalRimParameter = 0.0;
	int32 FirstPointIndex = 0;
	int32 PointCount = 0;
	int32 FirstTransitionPointIndex = 0;
	int32 TransitionPointCount = 0;
	// Ordered source-topology lane beginning at the first longitudinal run and
	// continuing toward the finite interior.  It is recognition evidence only;
	// no runtime query consumes these mesh samples directly.
	int32 FirstLongitudinalContinuationPointIndex = 0;
	int32 LongitudinalContinuationPointCount = 0;
	int32 FirstLongitudinalSampleIndex = 0;
	// Natural-cubic Bezier pieces through the topology-ordered tube witnesses.
	// This remains recognition data until a compact surface has independently
	// passed its spatial and support/release certificates.
	int32 FirstLongitudinalSplineSegmentIndex = INDEX_NONE;
	int32 LongitudinalSplineSegmentCount = 0;
	// The initially near-longitudinal portion of the ordered continuation. It
	// ends at the first sustained tangent departure and is the only part that
	// may constrain a tube fit; the remaining ordered points belong to the
	// rear turn/cap fit.
	int32 LongitudinalTubePointCount = 0;
	int32 FirstLongitudinalRearTurnPointOffset = INDEX_NONE;
	double LongitudinalContinuationArcLengthCm = 0.0;
	double LongitudinalTubeArcLengthCm = 0.0;
	double LongitudinalTubeTerminalDepthCm = 0.0;
	double LongitudinalTubeTerminalOpeningOffsetCm = 0.0;
	double LongitudinalTubeMaximumDepartureDegrees = 0.0;
	double LongitudinalSplineRootMeanSquareResidualCm = 0.0;
	double LongitudinalSplineMaximumResidualCm = 0.0;
	double LongitudinalContinuationMaximumTurnDegrees = 0.0;
	double LongitudinalContinuationTerminalDepthCm = 0.0;
	double LongitudinalContinuationTerminalOpeningOffsetCm = 0.0;
	int32 TransitionSegmentCount = 0;
	int32 SegmentCount = 0;
	double ArcLengthCm = 0.0;
	double RimDistanceCm = 0.0;
	double MaximumLongitudinalDepthCm = 0.0;
	double MaximumOpeningOffsetCm = 0.0;
	double BoundaryPlaneTangentResidualDegrees = 0.0;
	double LongitudinalTangentResidualDegrees = 0.0;
	double LongitudinalTangentDepthCm = 0.0;
	double LongitudinalTangentOpeningOffsetCm = 0.0;
	int32 LongitudinalTransitionHop = INDEX_NONE;
	FBox2d Bounds = FBox2d(EForceInit::ForceInit);
	bool bConnectedToRim = false;
	bool bSourceBoundaryPlaneG1Plausible = false;
	bool bLongitudinalRunPlausible = false;
	bool bTopologyCanonicalRimParameterValid = false;
	bool bLongitudinalSplineC2 = false;
	double IndividualC2BoundaryPlaneTangentMagnitudeCm = 0.0;
	double IndividualC2LongitudinalTangentMagnitudeCm = 0.0;
	double IndividualC2RootMeanSquareResidualCm = 0.0;
	double IndividualC2MaximumResidualCm = 0.0;
	bool bIndividualC2FitValid = false;
};

enum class EOpenRimTransitionFitFamily : uint8
{
	SharedOpeningRim = 0,
	VerticalSegments = 1,
	HorizontalSegments = 2,
	UpperTransitions = 3,
};

// Recognition-only exact-C2 quintic fitted to the bounded rim-to-first-
// longitudinal samples. The fit begins tangent to the boundary plane section and
// ends tangent to the longitudinal opening run, with zero second derivative at
// both endpoints.
struct IAMSPEED_API FOpenRimTransitionFamilyFit
{
	uint64 FitId = 0;
	EOpenRimTransitionFitFamily Family =
		EOpenRimTransitionFitFamily::SharedOpeningRim;
	int32 FirstMemberFitIndex = 0;
	int32 MemberCount = 0;
	int32 SampleCount = 0;
	double EndDepthCm = 0.0;
	double EndOpeningOffsetCm = 0.0;
	double BoundaryPlaneTangentMagnitudeCm = 0.0;
	double LongitudinalTangentMagnitudeCm = 0.0;
	double BalancedRootMeanSquareResidualCm = 0.0;
	double MaximumMemberRootMeanSquareResidualCm = 0.0;
	double MaximumResidualCm = 0.0;
	bool bExactC0G1C2ByConstruction = false;
};

struct IAMSPEED_API FOpenRimTransitionMemberFit
{
	int32 OpenRimTransitionFamilyFitIndex = INDEX_NONE;
	int32 OpenRimTransverseSectionIndex = INDEX_NONE;
	int32 SampleCount = 0;
	double RootMeanSquareResidualCm = 0.0;
	double MaximumResidualCm = 0.0;
};

enum class EOpenRimC2LoftModel : uint8
{
	GlobalBernstein5 = 0,
	PiecewiseC2Transition = 1,
	SymmetryStationBernstein5 = 2,
	SymmetryStationArchitecturalLocalC2 = 3,
};

enum class EOpenRimC2LoftStationFamily : uint8
{
	VerticalSegment = 0,
	Transition = 1,
	HorizontalSpan = 2,
};

// One symmetry-balanced observation of the canonical half-rim. VerticalSegment and
// horizontal span stations group exact generated slices; transition stations resample
// each of the four opening/side lanes at the same canonical coordinate before
// averaging, so source triangulation density cannot bias the loft.
struct IAMSPEED_API FOpenRimC2LoftStation
{
	uint64 StationId = 0;
	EOpenRimC2LoftStationFamily Family =
		EOpenRimC2LoftStationFamily::VerticalSegment;
	double CanonicalRimParameter = 0.0;
	int32 MemberCount = 0;
	double MeanEndDepthCm = 0.0;
	double MeanEndOpeningOffsetCm = 0.0;
	double MeanBoundaryPlaneTangentCm = 0.0;
	double MeanLongitudinalTangentCm = 0.0;
	double MedianEndDepthCm = 0.0;
	double MedianEndOpeningOffsetCm = 0.0;
	double MedianBoundaryPlaneTangentCm = 0.0;
	double MedianLongitudinalTangentCm = 0.0;
	double GeometricFitEndDepthCm = 0.0;
	double GeometricFitEndOpeningOffsetCm = 0.0;
	double GeometricFitBoundaryPlaneTangentCm = 0.0;
	double GeometricFitLongitudinalTangentCm = 0.0;
	double GeometricFitRootMeanSquareResidualCm = 0.0;
	double GeometricFitMaximumResidualCm = 0.0;
	double RootMeanSquareParameterSpreadCm = 0.0;
	double MaximumParameterSpreadCm = 0.0;
	bool bHasBothOpenings = false;
	bool bHasBothSides = false;
	bool bGeometricFitValid = false;
};

// One interval of a local C2 parameter loft. Values and derivatives are with
// respect to the canonical rim coordinate; endpoint second derivatives are
// zero, so adjacent segments share value, first derivative and curvature.
struct IAMSPEED_API FOpenRimC2LoftSegment
{
	double StartCanonicalRimParameter = 0.0;
	double EndCanonicalRimParameter = 0.0;
	double StartValuesCm[4] = {};
	double EndValuesCm[4] = {};
	double StartDerivativesCm[4] = {};
	double EndDerivativesCm[4] = {};
};

struct IAMSPEED_API FOpenRimCanonicalSurfaceSample
{
	int32 OpenRimTransverseSectionIndex = INDEX_NONE;
	int32 TransitionPointOffset = 0;
	double CanonicalRimParameter = 0.0;
	double TransitionParameter = 0.0;
	FVector3d CanonicalPositionCm = FVector3d::ZeroVector;
};

// One topology-ordered sample of the finite continuation beyond an opening
// rim. The canonical frame is (depth, abs(transverse), height); the source
// section and arc coordinate preserve correspondence for later tube/cap fits.
struct IAMSPEED_API FOpenRimLongitudinalSample
{
	int32 OpenRimTransverseSectionIndex = INDEX_NONE;
	int32 ContinuationPointOffset = 0;
	double NormalizedArcLength = 0.0;
	FVector3d CanonicalPositionCm = FVector3d::ZeroVector;
};

// One parameter-bounded cubic Bezier interval of a topology-ordered tube
// witness spline. Coordinates use the section-local normalized tube arc.
struct IAMSPEED_API FOpenRimLongitudinalSplineSegment
{
	int32 OpenRimTransverseSectionIndex = INDEX_NONE;
	double MinimumTubeParameter = 0.0;
	double MaximumTubeParameter = 1.0;
	FVector3d ControlPoints[4] = {};
};

// One recognition-only interval of the ordered terminal closure beyond the
// near-longitudinal prefix. Degree is three for the natural-cubic interior and
// five for the first interval that carries the prefix C2 boundary data.
struct IAMSPEED_API FOpenRimTerminalClosureRailSegment
{
	int32 OpenRimTransverseSectionIndex = INDEX_NONE;
	double MinimumClosureParameter = 0.0;
	double MaximumClosureParameter = 1.0;
	int32 Degree = 3;
	FVector3d ControlPoints[6] = {};
};

struct IAMSPEED_API FOpenRimTerminalClosureRailFit
{
	int32 OpenRimTransverseSectionIndex = INDEX_NONE;
	int32 FirstSegmentIndex = INDEX_NONE;
	int32 SegmentCount = 0;
	int32 SourceSampleCount = 0;
	double ClosureToPrefixParameterScale = 1.0;
	double MaximumSourceResidualCm = TNumericLimits<double>::Max();
	double MaximumPolylineResidualCm = TNumericLimits<double>::Max();
	double PositionJoinResidualCm = TNumericLimits<double>::Max();
	double FirstDerivativeJoinResidualCm = TNumericLimits<double>::Max();
	double SecondDerivativeJoinResidualCm = TNumericLimits<double>::Max();
	bool bC2PrefixJoinByConstruction = false;
	bool bRegularFinite = false;
};

// Auditable section-level map from the two independent rim witnesses to the
// jointly optimized surface coordinate. Lane rank is topology ordered.
struct IAMSPEED_API FOpenRimCanonicalSectionCorrespondence
{
	int32 OpenRimTransverseSectionIndex = INDEX_NONE;
	uint64 LaneId = 0;
	int32 LaneRank = INDEX_NONE;
	int32 LaneSectionCount = 0;
	double GeometricSurfaceParameter = 0.0;
	double TopologySurfaceParameter = 0.0;
	double OptimizedSurfaceParameter = 0.0;
	double RootMeanSquareResidualCm = 0.0;
	bool bWithinQuantizedWitnessBounds = false;
	bool bStrictlyInterior = false;
};

// Direct common-frame 3D surface hypothesis. Six degree-five Bernstein fields
// define the boundary plane-tangent vector, longitudinal endpoint curve and terminal
// depth tangent. The t-boundaries are exact C2 by Bezier construction.
struct IAMSPEED_API FOpenRimCanonicalSurfaceFit
{
	uint64 FitId = 0;
	int32 SampleCount = 0;
	double VerticalSegmentParameterWidth = 0.0;
	double TransitionParameterWidth = 0.0;
	double HorizontalSpanParameterWidth = 0.0;
	// Three degree-five canonical boundary segments in traversal order. The
	// first is vertical, the second is the smooth transition and the third is
	// horizontal. Linear segments are degree-elevated exactly.
	FVector2d CanonicalRimSegmentControlPoints[18] = {};
	double StartTangentYCoefficients[6] = {};
	double StartTangentZCoefficients[6] = {};
	double EndDepthCoefficients[6] = {};
	double EndYCoefficients[6] = {};
	double EndZCoefficients[6] = {};
	double EndLongitudinalTangentCoefficients[6] = {};
	double TransitionStartTangentYCoefficients[4] = {};
	double TransitionStartTangentZCoefficients[4] = {};
	double TransitionEndDepthCoefficients[4] = {};
	double TransitionEndYCoefficients[4] = {};
	double TransitionEndZCoefficients[4] = {};
	double RootMeanSquareResidualCm = 0.0;
	double MaximumResidualCm = 0.0;
	int32 MaximumResidualSectionIndex = INDEX_NONE;
	int32 MaximumResidualPointOffset = INDEX_NONE;
	double MaximumResidualRimParameter = 0.0;
	double MaximumResidualTransitionParameter = 0.0;
	FVector3d MaximumResidualSourcePositionCm = FVector3d::ZeroVector;
	FVector3d MaximumResidualFitPositionCm = FVector3d::ZeroVector;
	int32 TransitionEndpointClampedSampleCount = 0;
	double TransitionEndpointClampedRootMeanSquareResidualCm = 0.0;
	double TransitionEndpointClampedMaximumResidualCm = 0.0;
	int32 TransitionInteriorSampleCount = 0;
	double TransitionInteriorRootMeanSquareResidualCm = 0.0;
	double TransitionInteriorMaximumResidualCm = 0.0;
	int32 CorrespondenceLaneCount = 0;
	int32 CorrespondenceSectionCount = 0;
	double MaximumCorrespondenceDisplacement = 0.0;
	double MinimumCorrespondenceSpacing = 0.0;
	int32 FeatureSampleCounts[4] = {};
	double FeatureRootMeanSquareResidualsCm[4] = {};
	double FeatureMaximumResidualsCm[4] = {};
	double MinimumEndDepthCm = 0.0;
	double MinimumLongitudinalTangentCm = 0.0;
	double MaximumFirstDerivativeJoinResidualCm = 0.0;
	double MaximumSecondDerivativeJoinResidualCm = 0.0;
	double MaximumNormalJoinResidualDegrees = 0.0;
	bool bExactXYMirrorPlacement = false;
	bool bExactTransverseC2ByConstruction = false;
	bool bLongitudinallyC2ByConstruction = false;
	bool bTransitionCorrectionC2ByConstruction = false;
	bool bCorrespondenceStrictlyMonotone = false;
	bool bCorrespondenceWithinQuantizedWitnessBounds = false;
	bool bCorrespondenceStrictlyInterior = false;
	bool bRegularPositiveParameterization = false;

	FVector3d EvaluateCanonicalSurface(double S, double T) const;
};

// Symmetry-balanced target curve at the end of the topology-certified tube
// prefix. It deliberately excludes the rear turn/cap witnesses. A later
// tensor provider may join this curve to FOpenRimCanonicalSurfaceFit only
// after its residual and C2 certificates are accepted.
struct IAMSPEED_API FOpenRimTubeTerminalSplineSegment
{
	double MinimumCanonicalRimParameter = 0.0;
	double MaximumCanonicalRimParameter = 0.0;
	FVector3d ControlPoints[4] = {};
};

struct IAMSPEED_API FOpenRimCanonicalTubeFit
{
	uint64 FitId = 0;
	int8 OpeningSide = 0;
	int8 TransverseSide = 0;
	int32 SampleCount = 0;
	double TerminalDepthCoefficients[6] = {};
	double TerminalYCoefficients[6] = {};
	double TerminalZCoefficients[6] = {};
	double RootMeanSquareResidualCm = 0.0;
	double MaximumResidualCm = 0.0;
	int32 MaximumResidualSectionIndex = INDEX_NONE;
	TArray<FOpenRimTubeTerminalSplineSegment> TerminalSplineSegments;
	bool bExactXYMirrorPlacement = false;
	bool bRegularFiniteTube = false;
	bool bTerminalCurveC2 = false;

	FVector3d EvaluateTerminal(double S) const;
};

// One exact-X/Y loft hypothesis over the complete half-rim. Each cross-section
// remains an endpoint-C2 quintic and its four positive parameters are smooth
// functions of the canonical vertical segment-to-horizontal span rim coordinate.
struct IAMSPEED_API FOpenRimC2LoftFit
{
	uint64 FitId = 0;
	EOpenRimC2LoftModel Model = EOpenRimC2LoftModel::GlobalBernstein5;
	int32 CoefficientCount = 0;
	int32 FirstSegmentIndex = 0;
	int32 SegmentCount = 0;
	int32 MemberCount = 0;
	int32 SampleCount = 0;
	double EndDepthCoefficients[6] = {};
	double EndOpeningOffsetCoefficients[6] = {};
	double BoundaryPlaneTangentCoefficients[6] = {};
	double LongitudinalTangentCoefficients[6] = {};
	double BalancedRootMeanSquareResidualCm = 0.0;
	double MaximumMemberRootMeanSquareResidualCm = 0.0;
	double MaximumResidualCm = 0.0;
	double MinimumSampledParameterValueCm = 0.0;
	double MaximumSampledFirstDerivativeMagnitudeCm = 0.0;
	double MaximumSampledSecondDerivativeMagnitudeCm = 0.0;
	double MaximumFirstDerivativeJoinResidualCm = 0.0;
	double MaximumSecondDerivativeJoinResidualCm = 0.0;
	bool bExactXYMirrorPlacement = false;
	bool bC2AlongRimByPolynomialConstruction = false;
	bool bRegularPositiveParameterization = false;
};

// Recognition-only gameplay intent attached to a topologically identified
// surface transition. Geometry may remain smooth; this policy records whether
// the contact manifold is allowed to carry support across the feature.
enum class EAnalyticSupportTransitionPolicy : uint8
{
	ContinuousSupport = 0,
	BidirectionalReleaseBoundary = 1,
};

struct IAMSPEED_API FOpenRimSupportTransitionIntent
{
	uint64 IntentId = 0;
	int32 OpenRimTransverseSectionIndex = INDEX_NONE;
	EOpenRimFeature Feature = EOpenRimFeature::UpperTransition;
	EAnalyticSupportTransitionPolicy Policy =
		EAnalyticSupportTransitionPolicy::ContinuousSupport;
	bool bGeometricC2Permitted = false;
	bool bBidirectional = false;
	bool bImmediateAdjacentSurfaceHandoffForbidden = false;
	bool bFreshApproachMayAcquireAdjacentSurface = false;
};

struct IAMSPEED_API FPlanarSymmetryMetrics
{
	EPlanarSymmetryAxis Axis = EPlanarSymmetryAxis::X;
	double CenterCoordinate = 0.0;
	int32 MatchedVertexCount = 0;
	int32 UnmatchedVertexCount = 0;
	double RootMeanSquareResidual = 0.0;
	double MaximumResidual = 0.0;
};

struct IAMSPEED_API FSurfaceSymmetryMetrics
{
	EPlanarSymmetryAxis Axis = EPlanarSymmetryAxis::X;
	double CenterCoordinate = 0.0;
	int32 WithinToleranceVertexCount = 0;
	int32 OutsideToleranceVertexCount = 0;
	double RootMeanSquareNearestResidual = 0.0;
	double MaximumNearestResidual = 0.0;
};

enum class EFeatureKind : uint8
{
	Interior = 0,
	Boundary = 1,
};

struct IAMSPEED_API FBoundedPlane
{
	uint64 SourceId = 0;
	uint64 SurfaceId = 0;
	uint64 FeatureId = 0;
	uint64 PrimitiveId = 0;
	uint32 MaterialId = 0;
	uint32 ObjectType = 0;
	uint64 BlockingChannels = 0;
	FVector3d Origin = FVector3d::ZeroVector;
	FVector3d Normal = FVector3d::UpVector;
	FVector3d AxisU = FVector3d::ForwardVector;
	FVector3d AxisV = FVector3d::RightVector;
	FVector2d HalfExtents = FVector2d::ZeroVector;
	// Optional simple polygon in the plane's local (U,V) coordinates. Empty
	// retains the legacy rectangular domain described by HalfExtents.
	TArray<FVector2d> DomainVertices;
	// Derived by FinalizeAndValidate; never serialized or hashed.
	FBox3d Bounds = FBox3d(EForceInit::ForceInit);
	bool bQueryCollisionEnabled = true;
	bool bRequiresCompactOptIn = false;
	bool bAuthorityEligible = false;

	double SignedDistance(const FVector3d& Point) const;
	FVector3d ClosestPoint(const FVector3d& Point) const;
	bool ContainsProjectedPoint(const FVector3d& Point, double Tolerance = 0.0) const;
	/** Certifies a whole straight segment inside this finite domain, not only its endpoints.
	 * Conservatively refuses polygon-boundary crossings/tangencies; no domain skin. */
	bool ContainsProjectedSegment(const FVector3d& Start, const FVector3d& End) const;
	double DistanceToDomainBoundary(const FVector3d& Point) const;
	bool IsValid(FString* OutReason = nullptr) const;
};

// Compact tensor-product patch: a quintic section curve swept linearly along
// one normalized axis. Six world-space control points define the section at
// ExtrusionCoordinate=0; the two scalar limits close the finite surface.
// Runtime queries tessellate only the one-dimensional section, never the
// source mesh, and retain stable face/edge/vertex feature identifiers.
struct IAMSPEED_API FExtrudedQuinticPatch
{
	uint64 SourceId = 0;
	uint64 SurfaceId = 0;
	uint64 FeatureId = 0;
	uint64 PrimitiveId = 0;
	uint64 CanonicalGroupId = 0;
	uint8 CanonicalSymmetryAxisMask = 0;
	uint32 MaterialId = 0;
	uint32 ObjectType = 0;
	uint64 BlockingChannels = 0;
	FVector3d SectionControlPoints[6] = {};
	// Degree-7 Bernstein B3/B4 interior correction. Both basis functions and
	// their first two derivatives vanish at t=0/1, so the quintic base retains
	// its exact endpoint position, tangent and second derivative.
	FVector3d InteriorCorrectionControlPoints[2] = {};
	double BaseRootMeanSquareResidualCm = 0.0;
	double BaseMaximumResidualCm = 0.0;
	double CorrectedRootMeanSquareResidualCm = 0.0;
	double CorrectedMaximumResidualCm = 0.0;
	// Additional source-agreement allowance carried only by a provider whose
	// physical surface was deliberately displaced from its recognition source.
	// Hybrid arbitration adds this certified local allowance to the unchanged
	// public residual classification tolerance; ordinary providers keep zero.
	double AdditionalResidualAgreementAllowanceCm = 0.0;
	FVector3d ExtrusionAxis = FVector3d::ForwardVector;
	double MinimumExtrusionCoordinate = 0.0;
	double MaximumExtrusionCoordinate = 0.0;
	FBox3d Bounds = FBox3d(EForceInit::ForceInit);
	bool bQueryCollisionEnabled = false;
	bool bCanonicalC2ByConstruction = false;
	bool bCanonicalSymmetryByConstruction = false;
	bool bAuthorityEligible = false;
	// Deterministic query representation derived from the degree-7 control
	// polygon. It is neither serialized nor hashed. MaximumChordErrorCm is a
	// certified Hausdorff upper bound from each Bezier subcurve to its chord.
	TArray<FVector3d> SectionPolyline;
	TArray<double> SectionParameters;
	// One immutable finite-facet AABB per section chord, rebuilt with the BVH.
	// No per-query history: storage scales only with the fixed approximation.
	TArray<FBox3d> SectionSegmentBounds;
	TArray<FTriangleBvhNode> SectionSegmentBvhNodes;
	TArray<int32> SectionSegmentBvhIndices;
	double MaximumChordErrorCm = TNumericLimits<double>::Max();

	FVector3d EvaluateSection(double T) const;
	FVector3d EvaluateSectionDerivative(double T) const;
	bool BuildQueryApproximation(
		double ChordToleranceCm = ExtrudedQuinticChordToleranceCm);
	bool IsValid(FString* OutReason = nullptr) const;
};

// Generic finite tensor-product Bezier surface. Control points are stored in
// U-major order: ControlPoints[U * (DegreeV + 1) + V]. This mathematical
// primitive carries no collision authority by itself; a runtime provider must
// additionally certify its finite query approximation and feature domains.
struct IAMSPEED_API FTensorBezierApproximationCell
{
	double MinimumU = 0.0;
	double MaximumU = 1.0;
	double MinimumV = 0.0;
	double MaximumV = 1.0;
	// U0V0, U0V1, U1V0, U1V1.
	FVector3d Corners[4] = {};
	double MaximumErrorCm = TNumericLimits<double>::Max();
};

struct IAMSPEED_API FTensorBezierSurface
{
	int32 DegreeU = 0;
	int32 DegreeV = 0;
	TArray<FVector3d, TInlineAllocator<16>> ControlPoints;

	/** Evaluate the clamped control net without modifying it; scratch is local to this call. */
	FVector3d Evaluate(double U, double V) const;
	/** Exact de Casteljau derivatives; no temporal cache or change of interpolation order. */
	FVector3d EvaluateDerivativeU(double U, double V) const;
	FVector3d EvaluateDerivativeV(double U, double V) const;
	FVector3d EvaluateNormal(double U, double V) const;
	bool BuildBilinearApproximation(
		double ToleranceCm,
		TArray<FTensorBezierApproximationCell>& OutCells,
		double* OutMaximumErrorCm = nullptr) const;
	bool IsValid(FString* OutReason = nullptr) const;
};

// Recognition bridge from one piece of the canonical fit to the generic
// tensor surface. Runtime authority must consume the generic surface rather
// than this recognition provenance record.
struct IAMSPEED_API FOpenRimCanonicalTensorSurface
{
	uint64 SourceFitId = 0;
	int32 SegmentIndex = INDEX_NONE;
	double MinimumCanonicalRimParameter = 0.0;
	double MaximumCanonicalRimParameter = 0.0;
	FTensorBezierSurface Surface;
	double MaximumConversionErrorCm = TNumericLimits<double>::Max();
};

// Shadow-only tube strip between the canonical lip terminal and one physical
// lane-local terminal spline interval. Runtime materialization is intentionally
// deferred until mesh residual, end-cap ownership and both C2 boundaries have
// been certified.
struct IAMSPEED_API FOpenRimCanonicalTubeTensorSurface
{
	uint64 SourceFitId = 0;
	int8 OpeningSide = 0;
	int8 TransverseSide = 0;
	int32 SegmentIndex = INDEX_NONE;
	double MinimumCanonicalRimParameter = 0.0;
	double MaximumCanonicalRimParameter = 0.0;
	// Explicit longitudinal subdomain. A tensor is selected only inside both
	// canonical coordinates; this keeps measured tube stations one-to-one.
	double MinimumTubeParameter = 0.0;
	double MaximumTubeParameter = 1.0;
	// Maps this local longitudinal parameter to the adjacent prefix parameter.
	// One is the prefix itself; terminal-closure cells share one positive scale
	// per physical lane so derivative ownership remains explicit.
	double LongitudinalParameterScale = 1.0;
	FTensorBezierSurface Surface;
	double MaximumConversionErrorCm = TNumericLimits<double>::Max();
	bool bLipBoundaryC2ByConstruction = false;
	// Compact tensor-product spline cell built from a lane-wide adaptive set of
	// shared longitudinal knots. Every cell is an exact bicubic conversion of
	// the same C2 surface, so adjacent cells share position and first/second
	// derivatives rather than merely agreeing at sampled boundary points.
	bool bAdaptiveCompactC2 = false;
	// Separate adaptive family beyond the near-longitudinal prefix. Its V=0
	// boundary consumes the prefix derivatives under one lane-wide parameter
	// scale; it is never implied by bAdaptiveCompactC2.
	bool bAdaptiveTerminalClosureC2 = false;
	// Recognition certificate against every topology-preserving source witness
	// in the physical lane. This is intentionally separate from the polynomial
	// conversion error, which only proves that the bicubic payload reproduces
	// the fitted C2 surface.
	bool bSourceResidualCertified = false;
	double SourceMaximumResidualCm = TNumericLimits<double>::Max();
	// Recognition-only oracle cell built directly from adjacent source witnesses.
	// It must never be materialized as a runtime provider.
	bool bMeasuredWitnessOnly = false;
};

// One compact, single-patch shadow candidate sampled from the independent C2
// longitudinal rails. It is intentionally distinct from tube tensor cells so
// an experimental residual can never accidentally enter runtime materialization.
struct IAMSPEED_API FOpenRimCanonicalTubeLoftFit
{
	uint64 FitId = 0;
	int8 OpeningSide = 0;
	int8 TransverseSide = 0;
	int32 TileIndex = INDEX_NONE;
	double MinimumCanonicalRimParameter = 0.0;
	double MaximumCanonicalRimParameter = 1.0;
	double MinimumTubeParameter = 0.0;
	double MaximumTubeParameter = 1.0;
	int32 SampleCount = 0;
	FTensorBezierSurface Surface;
	double InterpolationMaximumErrorCm = TNumericLimits<double>::Max();
	double RootMeanSquareResidualCm = 0.0;
	double MaximumResidualCm = 0.0;
	int32 MaximumResidualSectionIndex = INDEX_NONE;
	bool bLongitudinalC2Input = false;
	bool bTransverseC2Input = false;
	bool bLowDegreeCandidate = false;
};

struct IAMSPEED_API FTensorBezierPatch
{
	uint64 SourceId = 0;
	uint64 SurfaceId = 0;
	uint64 FeatureId = 0;
	uint64 PrimitiveId = 0;
	uint64 CanonicalGroupId = 0;
	uint32 MaterialId = 0;
	uint32 ObjectType = 0;
	uint64 BlockingChannels = 0;
	FTensorBezierSurface Surface;
	FBox3d Bounds = FBox3d(EForceInit::ForceInit);
	TArray<FTensorBezierApproximationCell> ApproximationCells;
	// Deterministic query-only hierarchy over the certified cells. Neither the
	// nodes nor their permutation are serialized or included in stable hashes.
	TArray<FTriangleBvhNode> ApproximationCellBvhNodes;
	TArray<int32> ApproximationCellBvhIndices;
	double MaximumApproximationErrorCm = TNumericLimits<double>::Max();
	bool bQueryCollisionEnabled = false;
	bool bApproximationCertified = false;
	bool bAuthorityEligible = false;

	bool BuildQueryApproximation(
		double ToleranceCm = TensorBezierQueryApproximationToleranceCm);
	bool IsValid(FString* OutReason = nullptr) const;
};

// A compact family of independently parameterized tensor patches.  This keeps
// a piecewise spline as one query provider instead of turning its cells into a
// large collection of unrelated broad-phase primitives.
struct IAMSPEED_API FPiecewiseTensorBezierCell
{
	// 0/1 are the U min/max edges and 2/3 the V min/max edges.  The values
	// identify authored boundaries, not tessellation edges.
	uint64 BoundaryFeatureIds[4] = {};
	uint64 FeatureId = 0;
	uint64 PrimitiveId = 0;
	double MinimumU = 0.0;
	double MaximumU = 1.0;
	double MinimumV = 0.0;
	double MaximumV = 1.0;
	double LongitudinalParameterScale = 1.0;
	bool bTerminalClosure = false;
	FTensorBezierSurface Surface;
	FBox3d Bounds = FBox3d(EForceInit::ForceInit);
	TArray<FTensorBezierApproximationCell> ApproximationCells;
	TArray<FTriangleBvhNode> ApproximationCellBvhNodes;
	TArray<int32> ApproximationCellBvhIndices;
	double MaximumApproximationErrorCm = TNumericLimits<double>::Max();
};

struct IAMSPEED_API FPiecewiseTensorBezierAdjacency
{
	uint64 BoundaryFeatureId = 0;
	uint64 AdjacentBoundaryFeatureId = 0;
	uint64 CellPrimitiveId = 0;
	uint64 AdjacentCellPrimitiveId = 0;
	uint8 BoundaryIndex = 0;
	uint8 AdjacentBoundaryIndex = 0;
	bool bC2ByConstruction = false;
};

struct IAMSPEED_API FPiecewiseTensorBezierPatch
{
	uint64 SourceId = 0;
	uint64 SurfaceId = 0;
	uint64 PrimitiveId = 0;
	uint64 CanonicalGroupId = 0;
	uint32 MaterialId = 0;
	uint32 ObjectType = 0;
	uint64 BlockingChannels = 0;
	TArray<FPiecewiseTensorBezierCell> Cells;
	TArray<FPiecewiseTensorBezierAdjacency> Adjacencies;
	FBox3d Bounds = FBox3d(EForceInit::ForceInit);
	// A deterministic, provider-level broad phase over the stable cells.
	TArray<FTriangleBvhNode> CellBvhNodes;
	TArray<int32> CellBvhIndices;
	double MaximumApproximationErrorCm = TNumericLimits<double>::Max();
	bool bQueryCollisionEnabled = false;
	bool bApproximationCertified = false;
	bool bSourceResidualCertified = false;
	bool bAuthorityEligible = false;

	bool BuildQueryApproximation(
		double ToleranceCm = TensorBezierQueryApproximationToleranceCm);
	bool IsValid(FString* OutReason = nullptr) const;
};

struct IAMSPEED_API FAnalyticWorldData
{
	uint32 SchemaVersion = AnalyticWorldSchemaVersion;
	uint64 SourceHash = 0;
	TArray<FBoundedPlane> Planes;
	TArray<FExtrudedQuinticPatch> ExtrudedQuinticPatches;
	TArray<FTensorBezierPatch> TensorBezierPatches;
	TArray<FPiecewiseTensorBezierPatch> PiecewiseTensorBezierPatches;
	// Derived once by FinalizeAndValidate; never serialized or hashed.
	FBox3d CompactBounds = FBox3d(EForceInit::ForceInit);
	TArray<FTriangleSurface> Triangles;
	// Deterministic topology derived from the sorted face payload. Faces remain
	// expanded for the schema-4 compatibility milestone; the generated asset is
	// already indexed and can later feed queries without this expansion.
	TArray<FVector3d> MeshVertices;
	TArray<FIntVector> TriangleVertexIndices;
	TArray<FIntVector> TriangleEdgeIndices;
	TArray<FIntVector> TriangleNeighborIndices;
	TArray<FTriangleMeshEdge> MeshEdges;
	TArray<int32> EdgeIncidentTriangleIndices;
	TArray<FSmoothSurfaceRegion> SmoothSurfaceRegions;
	TArray<int32> SmoothRegionTriangleIndices;
	TArray<int32> TriangleSmoothRegionIndices;
	TArray<FVertexShapeSample> VertexShapeSamples;
	TArray<FSurfacePatch> SurfacePatches;
	TArray<int32> PatchTriangleIndices;
	TArray<int32> TrianglePatchIndices;
	// Editor/commandlet recognition products. Runtime query construction leaves
	// these arrays empty and StableHash deliberately excludes them.
	TArray<FPlanarSurfaceGroup> PlanarSurfaceGroups;
	TArray<int32> PlanarGroupPatchIndices;
	TArray<int32> PatchPlanarGroupIndices;
	TArray<FPlanarGroupMirrorMatch> PlanarGroupMirrorMatches;
	TArray<FTriangleCurvatureEvidence> TriangleCurvatureEvidence;
	TArray<FCurvatureSurfaceRegion> CurvatureSurfaceRegions;
	TArray<int32> CurvatureRegionTriangleIndices;
	TArray<int32> TriangleCurvatureRegionIndices;
	TArray<FCurvatureRegionMirrorMatch> CurvatureRegionMirrorMatches;
	TArray<FTriangleExtrusionEvidence> TriangleExtrusionEvidence;
	TArray<FExtrusionSurfaceRegion> ExtrusionSurfaceRegions;
	TArray<int32> ExtrusionRegionTriangleIndices;
	TArray<int32> TriangleExtrusionRegionIndices;
	TArray<FExtrusionRegionMirrorMatch> ExtrusionRegionMirrorMatches;
	TArray<FQuarterEllipseBoundaryMatch> QuarterEllipseBoundaryMatches;
	TArray<FC2TransitionSectionFit> C2TransitionSectionFits;
	TArray<FSharedC2TransitionPairFit> SharedC2TransitionPairFits;
	TArray<FSharedC2FrameLedgerEntry> SharedC2FrameLedger;
	TArray<FGlobalC2PlaneConstraint> GlobalC2PlaneConstraints;
	TArray<FGlobalC2JoinCertificate> GlobalC2JoinCertificates;
	FC2SymmetryPlacementFrame C2SymmetryPlacementFrame;
	TArray<FSymmetrizedC2PlaneConstraint> SymmetrizedC2PlaneConstraints;
	TArray<FCoupledC2TransitionPairSolution> CoupledC2TransitionSolutions;
	TArray<FCoupledC2TransitionFamilySolution> CoupledC2TransitionFamilies;
	TArray<int32> CoupledC2FamilyRegionIndices;
	TArray<FC2TransitionCoverageEntry> C2TransitionCoverage;
	TArray<FPlayableC2OrbitCandidate> PlayableC2OrbitCandidates;
	TArray<FPlayableC2OrbitMember> PlayableC2OrbitMembers;
	TArray<FPlayableC2TriangleSupportCandidate> PlayableC2TriangleSupportCandidates;
	TArray<FPlayableC2TriangleSupportMember> PlayableC2TriangleSupportMembers;
	TArray<FVector3d> PlayableC2CanonicalSupportPoints;
	TArray<FPlayableC2PlaneBinding> PlayableC2PlaneBindings;
	TArray<FPlayableC2NetworkPlaneConstraint> PlayableC2NetworkPlaneConstraints;
	TArray<FEndWallBoundaryComponent> EndWallBoundaryComponents;
	TArray<int32> EndWallBoundaryEdgeIndices;
	TArray<FOpenRimCandidate> OpenRimCandidates;
	TArray<int32> OpenRimEdgeIndices;
	TArray<FCanonicalOpenArchSolution> CanonicalOpenArchSolutions;
	TArray<FCanonicalOpenArchMemberFit> CanonicalOpenArchMemberFits;
	TArray<FOpenRimSurfaceBandObservation>
		OpenRimSurfaceBandObservations;
	TArray<FOpenRimTransverseSection> OpenRimTransverseSections;
	TArray<FVector2d> OpenRimTransverseSectionPoints;
	TArray<FVector2d> OpenRimTransitionSectionPoints;
	TArray<FVector2d> OpenRimLongitudinalContinuationPoints;
	TArray<FOpenRimLongitudinalSample> OpenRimLongitudinalSamples;
	TArray<FOpenRimLongitudinalSplineSegment> OpenRimLongitudinalSplineSegments;
	TArray<FOpenRimTerminalClosureRailSegment>
		OpenRimTerminalClosureRailSegments;
	TArray<FOpenRimTerminalClosureRailFit> OpenRimTerminalClosureRailFits;
	TArray<FOpenRimTransitionFamilyFit> OpenRimTransitionFamilyFits;
	TArray<FOpenRimTransitionMemberFit> OpenRimTransitionMemberFits;
	TArray<FOpenRimC2LoftStation> OpenRimC2LoftStations;
	TArray<FOpenRimC2LoftSegment> OpenRimC2LoftSegments;
	TArray<FOpenRimC2LoftFit> OpenRimC2LoftFits;
	TArray<FOpenRimCanonicalSurfaceSample> OpenRimCanonicalSurfaceSamples;
	TArray<FOpenRimCanonicalSectionCorrespondence>
		OpenRimCanonicalSectionCorrespondences;
	TArray<FOpenRimCanonicalSurfaceFit> OpenRimCanonicalSurfaceFits;
	TArray<FOpenRimCanonicalTubeFit> OpenRimCanonicalTubeFits;
	TArray<FOpenRimCanonicalTensorSurface> OpenRimCanonicalTensorSurfaces;
	TArray<FOpenRimCanonicalTubeTensorSurface>
		OpenRimCanonicalTubeTensorSurfaces;
	TArray<FOpenRimCanonicalTubeLoftFit> OpenRimCanonicalTubeLoftFits;
	TArray<FOpenRimSupportTransitionIntent>
		OpenRimSupportTransitionIntents;
	TArray<int32> TriangleIndices;
	TArray<FTriangleBvhNode> TriangleBvh;
	// Derived compact broadphase. Indices below Planes.Num() address planes;
	// subsequent indices address ExtrudedQuinticPatches.
	TArray<int32> CompactPrimitiveIndices;
	TArray<FTriangleBvhNode> CompactBvh;

	bool FinalizeAndValidate(
		FString* OutReason = nullptr,
		double TensorQueryApproximationToleranceCm =
			Speed::Analytic::TensorBezierQueryApproximationToleranceCm);
	bool AppendFinalizedNonCompactPlanes(
		TArray<FBoundedPlane> AdditionalPlanes,
		FString* OutReason = nullptr);
	void BuildRecognitionDiagnostics();
	void BuildCompactRuntimePatches();
	uint64 StableHash() const;
	uint64 RecognitionDiagnosticsHash() const;
	bool IsAuthorityEligible() const;
	FPlanarSymmetryMetrics MeasurePlanarSymmetry(
		EPlanarSymmetryAxis Axis, double MatchToleranceCm) const;
	FSurfaceSymmetryMetrics MeasurePlanarSurfaceSymmetry(
		EPlanarSymmetryAxis Axis, double MatchToleranceCm,
		double CenterCoordinate) const;

private:
	bool BuildTriangleTopology(FString* OutReason);
	void BuildVertexShapeSamples();
	void BuildPlanarSurfaceGroups();
	void BuildPlanarGroupMirrorMatches();
	void BuildTriangleCurvatureEvidence();
	void BuildCurvatureSurfaceRegions();
	void BuildCurvatureRegionMirrorMatches();
	void BuildExtrusionSurfaceRegions();
	void BuildExtrusionRegionMirrorMatches();
	void BuildQuarterEllipseBoundaryMatches();
	void BuildC2TransitionSectionFits();
	void BuildSharedC2TransitionPairFits();
	void BuildSharedC2FrameLedger();
	void BuildGlobalC2NetworkLedger();
	void BuildSymmetrizedC2PlaneConstraints();
	void BuildCoupledC2TransitionSolutions();
	void BuildC2TransitionCoverage();
	void BuildPlayableC2OrbitCandidates();
	void BuildPlayableC2TriangleSupportCandidates();
	void BuildPlayableC2PlaneBindings();
	void BuildPlayableC2NetworkPlaneConstraints();
	void BuildPlayableC2NetworkSolutions();
	void BuildEndWallBoundaryComponents();
	void BuildCanonicalOpenArchSolutions();
	void BuildOpenRimSurfaceBandObservations();
	void BuildOpenRimTransverseSections();
	void BuildOpenRimTransitionFamilyFits();
	void BuildOpenRimSupportTransitionIntents();
	void BuildSmoothSurfaceRegions();
	void BuildSurfacePatches();
	int32 BuildTriangleBvhNode(int32 FirstIndex, int32 IndexCount);
	int32 BuildCompactBvhNode(int32 FirstIndex, int32 IndexCount);
};

IAMSPEED_API uint64 StableStringId(const FString& Value);
IAMSPEED_API uint64 CombineStableIds(uint64 A, uint64 B);

} // namespace Speed::Analytic
