#pragma once

#if WITH_EDITOR && !UE_BUILD_SHIPPING
#include "IAmSpeed/World/Analytic/AnalyticWorldQuery.h"
#include "IAmSpeed/World/Analytic/DirectedLinearSphereContactObservation.h"
#include "IAmSpeed/World/Analytic/ScalarSphereContactBounds.h"
#include "IAmSpeed/World/Collision/CanonicalFixedSphereWheelProbeMath.h"
#include "IAmSpeed/World/Collision/PreparedIntervalScene.h"

namespace Speed::Analytic::Certified
{
IAMSPEED_API bool ProveRegularFaceLeafWithImmutablePlanarRank(FIndexedIntervalUnionPtr,
    const FTensorSphereContactLeaf&, FDistanceBudget&, FIntervalPoint&);
}

namespace Speed::Analytic::Certified::OrdinarySphereCorpus
{
namespace A = ::AnalyticCertified;
namespace D = DirectedLinearSphere;
namespace M = FixedWheelProbeMath;

// All maxima must come from an explicitly admitted ordinary-ball consumer
// contract. The fixed wheel's suspension and normal-quantization policy differ.
struct FConsumerMargins
{
    double SampleCm = 0., SurfaceCm = 0., CenterCm = 0., PhysicalPointCm = 0.;
    double QueryPointCm = 0., StoredNormalL1 = 0., ConsumedNormalL1 = 0.;
};

struct FMaterialized
{
    FWorldHit Hit;
    FScalarSphereContact Scalar;
    uint32 TOIBits = 0;
    double QueryPointErrorCm = 0.;
};

// The caller owns the one-shot live query/snapshot/roster/World-policy join.
// Keep the observation from this exact invocation and scene on the stack.
// Null margins are for this exact offscreen qualification only. They leave the
// errors as diagnostics. This helper publishes no World state or product policy.
namespace Private
{
inline bool TryMaterialize(const Prepared::FScene& Scene, const D::FQuery& Query,
    const D::FProposal& Proposal, const D::FObservation& Observation, float Delta,
    double PhysicalRadius, const FConsumerMargins* Margins, FDistanceBudget& Budget,
    FMaterialized& Out, const TCHAR*& Stage)
{
    Out = {};
    Stage = TEXT("input");
    const auto Before = RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    const auto Within = [](double Error, double Max)
    {
        return A::Finite(Error) && A::Finite(Max) && Error >= 0. && Max > 0. && Error <= Max;
    };
    const auto Contains = [](const FIntervalPoint& Box, const FVector3d& Point)
    {
        if (!Detail::ValidPoint(Box) || !ScalarContactDetail::Finite(Point))
            return false;
        for (int32 Axis = 0; Axis < 3; ++Axis)
            if (!(Box.Axis[Axis].Lower <= Point[Axis] && Point[Axis] <= Box.Axis[Axis].Upper))
                return false;
        return true;
    };
    const auto PayEvaluation = [&Budget](int32 NumControls)
    {
        for (int32 Work = 0; Work < 4 * NumControls; ++Work)
            if (!Budget.Consume())
                return false;
        return true;
    };
    const auto& Owners = Scene.GetOwners();
    const auto& Union = Scene.GetUnion();
    if (Observation.Status != D::EStatus::FirstContactEnclosed || Observation.Stage != D::EStage::Complete ||
        !Union || Union->Num() != Owners.Num() || Observation.CandidateCount <= 0 ||
        Observation.ExcludedCount < 0 || int64(Observation.CandidateCount) + Observation.ExcludedCount != Union->Num() ||
        Observation.ProviderIndex != Proposal.ProviderIndex || !Owners.IsValidIndex(Observation.ProviderIndex) ||
        Observation.WinnerLeaves.IsEmpty() || Observation.Leaves != Observation.WinnerLeaves.Num() ||
        A::Bits(Observation.Lower) != A::Bits(Proposal.Lower) || A::Bits(Observation.Upper) != A::Bits(Proposal.Upper) ||
        !ScalarContactDetail::Finite(Query.Start) || !ScalarContactDetail::Finite(Query.End) ||
        !A::Finite(Delta) || Delta <= 0.f || !CandidateDetail::PositiveRadius(Query.Radius))
        return false;
    Stage = TEXT("owner");
    const auto& Owner = Owners[Observation.ProviderIndex];
    if (!Owner.SourceId || !Owner.SurfaceId || !Owner.PrimitiveId || !Owner.QueryEnabled || Owner.FromPlane)
        return false;

    Stage = TEXT("net");
    FIntervalTensorSurface Original;
    if (!Union->CopySurfaceForQuery(Observation.ProviderIndex, Budget, Original) ||
        Original.DegreeU < 1 || Original.DegreeU > 16 || Original.DegreeV < 1 || Original.DegreeV > 16 ||
        Original.ControlPoints.Num() != (Original.DegreeU + 1) * (Original.DegreeV + 1))
        return false;
    FTensorBezierSurface Surface;
    Surface.DegreeU = Original.DegreeU;
    Surface.DegreeV = Original.DegreeV;
    for (const auto& Control : Original.ControlPoints)
    {
        FVector3d Point;
        for (int32 Axis = 0; Axis < 3; ++Axis)
        {
            const auto& Interval = Control.Axis[Axis];
            if (!A::Valid(Interval) || A::Bits(Interval.Lower) != A::Bits(Interval.Upper))
                return false;
            Point[Axis] = Interval.Lower;
        }
        Surface.ControlPoints.Add(Point);
    }

    Stage = TEXT("leaf_chart");
    FTensorSphereContactEnclosure Contact;
    Contact.Status = ETensorSphereContactStatus::FirstContactEnclosed;
    Contact.LowerTime = Observation.Lower;
    Contact.UpperTime = Observation.Upper;
    Contact.SurfacePoint = Observation.Point;
    Contact.RadialUnitNormal = Observation.RadialNormal;
    Contact.CollisionRadiusBits = A::Bits(Query.Radius);
    FIntervalPoint DirectedChord;
    for (int32 Axis = 0; Axis < 3; ++Axis)
        DirectedChord.Axis[Axis] = A::Subtract(A::Interval::Exact(Query.End[Axis]), A::Interval::Exact(Query.Start[Axis]));
    const auto Approach = ContactDetail::Dot(DirectedChord, Contact.RadialUnitNormal);
    if (!A::Valid(Approach) || Approach.Upper >= 0.)
        return false;

    int32 Sample = INDEX_NONE;
    for (const auto& Observed : Observation.WinnerLeaves)
    {
        if (!Budget.Consume() || !(0. < Observed.MinUV.X && Observed.MinUV.X < Observed.MaxUV.X && Observed.MaxUV.X < 1.) ||
            !(0. < Observed.MinUV.Y && Observed.MinUV.Y < Observed.MaxUV.Y && Observed.MaxUV.Y < 1.) ||
            !Detail::ValidPoint(Observed.Point) || !Detail::ValidPoint(Observed.RadialNormal))
            return false;
        FTensorSphereContactLeaf Leaf;
        Leaf.SurfaceIndex = Observation.ProviderIndex;
        Leaf.MinU = Observed.MinUV.X;
        Leaf.MaxU = Observed.MaxUV.X;
        Leaf.MinV = Observed.MinUV.Y;
        Leaf.MaxV = Observed.MaxUV.Y;
        Leaf.PointHull = Observed.Point;
        Leaf.RadialNormalHull = Observed.RadialNormal;
        const bool bSample = Leaf.MinU < Proposal.UV.X && Proposal.UV.X < Leaf.MaxU &&
                             Leaf.MinV < Proposal.UV.Y && Proposal.UV.Y < Leaf.MaxV;
        Leaf.FixedParameter = bSample ? Proposal.UV : (Observed.MinUV + Observed.MaxUV) * .5;
        if (!(Leaf.MinU < Leaf.FixedParameter.X && Leaf.FixedParameter.X < Leaf.MaxU &&
              Leaf.MinV < Leaf.FixedParameter.Y && Leaf.FixedParameter.Y < Leaf.MaxV))
        {
            Stage = TEXT("sample_parameter");
            return false;
        }
        if (!PayEvaluation(Original.ControlPoints.Num()) ||
            !ProjectedDistanceDetail::EvaluateFixedUV(Original, Leaf.FixedParameter, Leaf.FixedParameterPoint))
            return false;
        const auto Aligned = [&](const FIntervalPoint& Chart)
        {
            const auto Dot = ContactDetail::Dot(Chart, Leaf.RadialNormalHull);
            return A::Valid(Dot) && (Dot.Lower > 0. || Dot.Upper < 0.);
        };
        FIntervalPoint Chart;
        if (!(Observation.TryRetainedChart(Union, Leaf, Before.ControlFingerprint(), Chart) && Aligned(Chart)) &&
            !(ProveRegularFaceLeafWithImmutablePlanarRank(Union, Leaf, Budget, Chart) && Aligned(Chart)))
            return false;
        const int32 Index = Contact.Leaves.Add(MoveTemp(Leaf));
        if (bSample)
        {
            if (Sample != INDEX_NONE)
                return false;
            Sample = Index;
        }
    }

    Stage = TEXT("toi");
    M::FUniqueFloatSeconds Time;
    if (!M::TryUniqueFloatSeconds(Contact, double(Delta), Time) || Time.NativeSecondsBits == 0)
        return false;
    const double Representative = Contact.LowerTime + (Contact.UpperTime - Contact.LowerTime) * .5;
    if (!(Contact.LowerTime <= Representative && Representative <= Contact.UpperTime) ||
        M::FloatBits(static_cast<float>(Representative * double(Delta))) != Time.NativeSecondsBits)
        return false;

    Stage = TEXT("sample_parameter");
    TArray<int32, TInlineAllocator<8>> Samples;
    if (Sample != INDEX_NONE)
        Samples.Add(Sample);
    else
    {
        if (!A::Finite(Proposal.UV.X) || !A::Finite(Proposal.UV.Y))
            return false;
        for (int32 I = 0; I < Contact.Leaves.Num(); ++I)
            Samples.Add(I);
        const auto DistanceSquared = [&](int32 Index)
        {
            const FVector2d DeltaUV = Contact.Leaves[Index].FixedParameter - Proposal.UV;
            return DeltaUV.X * DeltaUV.X + DeltaUV.Y * DeltaUV.Y;
        };
        Samples.Sort([&](int32 AIndex, int32 BIndex)
        {
            const double ADistance = DistanceSquared(AIndex), BDistance = DistanceSquared(BIndex);
            return ADistance < BDistance || (ADistance == BDistance && AIndex < BIndex);
        });
    }

    const FVector3d Center = Query.Start + (Query.End - Query.Start) * Representative;
    FMaterialized Value;
    bool bSampleClosed = false;
    for (const int32 Index : Samples)
    {
        Stage = TEXT("sample_budget");
        if (!PayEvaluation(Original.ControlPoints.Num()))
            return false;
        const auto& Leaf = Contact.Leaves[Index];
        const FVector3d Point = Surface.Evaluate(Leaf.FixedParameter.X, Leaf.FixedParameter.Y);
        auto& Scalar = Value.Scalar;
        Stage = TEXT("sample_scalar");
        if (ConvertSphereContact(Contact, Index, Point, Center, Query.Radius, PhysicalRadius, Scalar) != EScalarSphereContactStatus::Bounded ||
            !Contains(Leaf.FixedParameterPoint, Point) ||
            !Contains(Contact.SurfacePoint, Point) ||
            !Contains(Contact.RadialUnitNormal, Scalar.NormalStored) ||
            !Contains(Contact.RadialUnitNormal, Scalar.NormalConsumed) ||
            (Margins && (!Within(Scalar.SampleEvaluationErrorCm, Margins->SampleCm) ||
                         !Within(Scalar.SurfaceErrorCm, Margins->SurfaceCm) ||
                         !Within(Scalar.LocationErrorCm, Margins->CenterCm) ||
                         !Within(Scalar.ResponseErrorCm, Margins->PhysicalPointCm) ||
                         !Within(Scalar.StoredNormalErrorL1, Margins->StoredNormalL1) ||
                         !Within(Scalar.ConsumedNormalErrorL1, Margins->ConsumedNormalL1))))
            continue;

        Stage = TEXT("query_point");
        auto& Hit = Value.Hit;
        Hit.Location = Scalar.Location;
        Hit.Point = Scalar.ImpactPoint;
        Hit.Normal = Scalar.NormalStored;
        Hit.QueryPoint = Scalar.Location - Query.Radius * Scalar.NormalStored;
        if (!ScalarContactDetail::ErrorToEveryPoint(Hit.QueryPoint, Contact.SurfacePoint, Value.QueryPointErrorCm) ||
            !Contains(Contact.SurfacePoint, Hit.QueryPoint) ||
            (Margins && !Within(Value.QueryPointErrorCm, Margins->QueryPointCm)))
            continue;
        bSampleClosed = true;
        break;
    }
    if (!bSampleClosed)
        return false;

    auto& Hit = Value.Hit;
    Hit.bHit = true;
    Hit.Time = Representative;
    Hit.bSurfaceNormalMayVary = true;
    Hit.GeometricErrorBoundCm = Value.Scalar.SampleEvaluationErrorCm;
    Hit.QueryFeatureKind = EContactFeatureKind::Vertex;
    Hit.SurfaceFeatureKind = EContactFeatureKind::Face;
    Hit.SourceId = Owner.SourceId;
    Hit.SurfaceId = Owner.SurfaceId;
    Hit.FeatureId = Owner.FeatureId;
    Hit.PrimitiveId = Owner.PrimitiveId;
    Hit.CanonicalGroupId = Owner.CanonicalGroupId;
    Hit.MaterialId = Owner.MaterialId;
    Value.TOIBits = Time.NativeSecondsBits;
    const auto After = RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if (After.ControlFingerprint() != Before.ControlFingerprint())
        return false;
    Out = MoveTemp(Value);
    return true;
}
}

// Offline/corpus helper. Compute and consume one fresh observation against the
// exact supplied scene object and literal query, with the same finite budget.
// Caller still authenticates World mapping, filters, external geometry and
// policy independently. No nominal production authority follows.
inline bool ObserveAndMaterialize(const Prepared::FScene& Scene, const D::FQuery& Query,
    const D::FProposal& Proposal, const D::FLimits& Limits, float Delta,
    double PhysicalRadius, const FConsumerMargins* Margins, FDistanceBudget& Budget,
    D::FObservation& Observation, FMaterialized& Out, const TCHAR*& Stage)
{
    Out = {};
    Stage = TEXT("directed_observation");
    Observation = D::Observe(Scene.GetUnion(), Query, Proposal, Limits, Budget);
    return Private::TryMaterialize(Scene, Query, Proposal, Observation, Delta, PhysicalRadius,
        Margins, Budget, Out, Stage);
}
}
#endif

