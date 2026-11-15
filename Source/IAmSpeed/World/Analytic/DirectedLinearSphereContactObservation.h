#pragma once

#include "IndexedIntervalUnion.h"
#include "TensorSphereContactBounds.h"
#include "HAL/PlatformTime.h"
#include "Misc/ScopeExit.h"

namespace Speed::Analytic::Certified::DirectedLinearSphere
{
// The scene is the exact immutable roster supplied by the caller. Its indices
// are local to that scene. This function neither joins World policy/filters nor
// proves separation from Landscape, external providers, or dynamic bodies.
struct FQuery
{
    FVector3d Start = FVector3d::ZeroVector, End = FVector3d::ZeroVector;
    double Radius = 0., DomainTolerance = 0., InitialOverlapTolerance = 0.;
};

struct FProposal
{
    int32 ProviderIndex = INDEX_NONE;
    FVector2d UV = FVector2d::ZeroVector;
    double Lower = 0., Upper = 0.;
};

struct FLimits
{
    int32 Work = 262144, Providers = 32;
    double PointWidth = 1.e-4, NormalWidth = 1.e-5;
    bool bFuseMonotonePrefix = false;
    bool bTryDirectedInitialShell = false;
    bool bCacheEnclosureSupports = false;
    bool bReuseQuinticEnclosureSplitStorage = false;
    bool bReduceQuinticSplitScores = false;
    bool bRetainLeafChart = false;
    bool bScaleLeafForNormalWidth = false;
    bool bShareQuinticHalfScales = false;
    bool bFuseReachableMonotonePrefix = false;
};

enum class EStatus : uint8 { Unknown, FirstContactEnclosed };
enum class EStage : uint8
{
    Input, Partition, InitialShell, UpperWitness, Prefix, Enclosure, Numeric, Complete
};

struct FObservedUVLeaf
{
    FVector2d MinUV, MaxUV;
    FIntervalPoint Point, RadialNormal;
};

struct FObservedChart
{
    FVector2d MinUV, MaxUV;
    FIntervalPoint Cross;
};

struct FObservation
{
    EStatus Status = EStatus::Unknown;
    EStage Stage = EStage::Input;
    int32 CandidateCount = 0, ExcludedCount = 0, MonotoneCount = 0, Leaves = 0;
    int32 ProviderIndex = INDEX_NONE;
    double Lower = 0., Upper = 0.;
    FIntervalPoint Point, RadialNormal;
    uint64 Work = 0, PrefixNodes = 0, EnclosureNodes = 0, MonotoneWork = 0;
    uint64 ReachableMonotoneAttempts = 0, ReachableMonotoneAccepted = 0, ReachableMonotoneWork = 0;
    double SelectMs = 0., ShellMs = 0., WitnessMs = 0., PrefixMs = 0., EnclosureMs = 0.;
    uint64 EndpointLeafTests = 0, EndpointLeafWork = 0;
    double EndpointLeafMs = 0.;
    bool bDirectedShellAttempted = false, bDirectedShellClosed = false;
    uint64 DirectedShellNodes = 0, DirectedShellWork = 0;
    double DirectedShellMs = 0.;
    uint64 EnclosureSupportTests = 0, EnclosureSupportControls = 0, EnclosureSupportEarlyRejects = 0;
    uint64 EnclosureReusableSplits = 0, EnclosureSplitStorageSlots = 0;
    uint64 EnclosureReducedSplitScores = 0;
    // Populated only on complete fused observations, for ProviderIndex.
    TArray<FObservedUVLeaf> WinnerLeaves;

    bool TryRetainedChart(const FIndexedIntervalUnionPtr& Scene, const FTensorSphereContactLeaf& Leaf,
        uint64 NumericFingerprint, FIntervalPoint& Chart) const
    {
        if (Status != EStatus::FirstContactEnclosed || !ChartScene || ChartScene != Scene ||
            ChartNumericFingerprint != NumericFingerprint || ChartProvider != Leaf.SurfaceIndex)
            return false;
        for (const auto& Row : Charts)
        {
            if (Row.MinUV.X == Leaf.MinU && Row.MaxUV.X == Leaf.MaxU &&
                Row.MinUV.Y == Leaf.MinV && Row.MaxUV.Y == Leaf.MaxV)
            {
                Chart = Row.Cross;
                return true;
            }
        }
        return false;
    }

private:
    friend FObservation Observe(const FIndexedIntervalUnionPtr&, const FQuery&, const FProposal&,
        const FLimits&, FDistanceBudget&);
    FIndexedIntervalUnionPtr ChartScene;
    uint64 ChartNumericFingerprint = 0;
    int32 ChartProvider = INDEX_NONE;
    TArray<FObservedChart> Charts;
};

namespace Detail
{
namespace A = AnalyticCertified;
namespace C = Speed::Analytic::Certified;
using namespace C::ContactDetail;
constexpr auto Contract = Speed::Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush;
struct FPartition
{
    int32 Visited = 0, Leaves = 0, MaxDepth = 0, AuxiliaryAxes = 0;
    C::FIntervalPoint Point;
    TArray<FObservedUVLeaf> ObservedLeaves;
    TArray<FObservedChart> Charts;
};

inline FVector3d ApproximateAxis(const C::FIntervalPoint& Center, const C::FIntervalPoint& Point)
{
    FVector3d Direction;
    for (int32 Axis = 0; Axis < 3; ++Axis)
        Direction[Axis] = (.5 * Center.Axis[Axis].Lower + .5 * Center.Axis[Axis].Upper) -
            (.5 * Point.Axis[Axis].Lower + .5 * Point.Axis[Axis].Upper);
    return Direction;
}

struct FDirectionalSupport
{
    C::FIntervalPoint Axis;
    double CenterLower = 0.;
    A::Interval Limit = A::Interval::Exact(0.);
    bool bValid = false;

    FDirectionalSupport(const C::FIntervalPoint& First, const C::FIntervalPoint& Last,
        const FVector3d& Direction, A::Interval RadiusSquared)
        : Axis(C::Detail::Point(Direction))
    {
        const auto LengthSquared = SquaredNorm(Axis);
        const auto FirstProjection = Dot(First, Axis), LastProjection = Dot(Last, Axis);
        if (!A::Valid(LengthSquared) || LengthSquared.Lower <= 0. ||
            !A::Valid(FirstProjection) || !A::Valid(LastProjection))
            return;
        CenterLower = FMath::Min(FirstProjection.Lower, LastProjection.Lower);
        Limit = A::Multiply(RadiusSquared, LengthSquared);
        bValid = A::Valid(Limit);
    }

    bool Clears(double UpperSupport) const
    {
        const auto Gap = A::Subtract(A::Interval::Exact(CenterLower), A::Interval::Exact(UpperSupport));
        const auto GapSquared = A::Square(Gap);
        return A::Valid(Gap) && Gap.Lower > 0. && A::Valid(GapSquared) && GapSquared.Lower > Limit.Upper;
    }
};

inline bool StrictDirectionalClear(const C::Detail::FNode& Node, const FDirectionalSupport& Support,
    bool bEarlyReject = false, FObservation* Account = nullptr)
{
    if (Account) ++Account->EnclosureSupportTests;
    if (!Support.bValid) return false;
    double UpperSupport = -TNumericLimits<double>::Max();
    bool bFirst = true;
    for (const auto& Control : Node.Net)
    {
        if (Account) ++Account->EnclosureSupportControls;
        const auto Projection = Dot(Control, Support.Axis);
        if (!A::Valid(Projection)) return false;
        UpperSupport = FMath::Max(UpperSupport, Projection.Upper);
        // Support max only increases. The first control already disproves
        // this strict directional bound if its gap or square is too small.
        // Inconclusive arithmetic still goes through the ordinary full scan.
        if (bEarlyReject && bFirst)
        {
            const auto Gap = A::Subtract(A::Interval::Exact(Support.CenterLower), A::Interval::Exact(UpperSupport));
            if (A::Valid(Gap))
            {
                const auto GapSquared = A::Square(Gap);
                if (Gap.Lower <= 0. || (A::Valid(GapSquared) && GapSquared.Lower <= Support.Limit.Upper))
                {
                    if (Account) ++Account->EnclosureSupportEarlyRejects;
                    return false;
                }
            }
        }
        bFirst = false;
    }
    return Support.Clears(UpperSupport);
}

inline bool Pay(C::FDistanceBudget& Budget, int32 Work)
{
    return C::CandidateDetail::ConsumeRepeated(Budget, uint64(Work));
}

// C(t) is the original linear path. For any fixed patch point S, half
// the derivative of |C(t)-S|Â² is (C(t)-S)Â·V and increases by |V|Â² per
// unit normalized time. At L this affine expression in S is bounded by
// its values at every original Bernstein control. A negative upper bound
// proves that a strict miss at L is a miss over [0,L] for this provider.
inline bool ProvePrefixDistanceDecreasing(const C::FIntervalTensorSurface& Surface,
    const C::FIntervalBezierCenterPath& Path, const C::FIntervalPoint& AtLower,
    C::FDistanceBudget& Budget, double& Upper)
{
    Upper = TNumericLimits<double>::Max();
    if (Path.Degree != 1 || Path.ControlPoints.Num() != 2 ||
        !C::Detail::ValidPoint(AtLower) || Surface.ControlPoints.IsEmpty() ||
        !Pay(Budget, 2 * Surface.ControlPoints.Num()))
        return false;
    const auto Direction = Subtract(Path.ControlPoints[1], Path.ControlPoints[0]);
    if (!C::Detail::ValidPoint(Direction))
        return false;
    double Maximum = -TNumericLimits<double>::Max();
    for (const auto& Control : Surface.ControlPoints)
    {
        const auto HalfDerivative = Dot(Subtract(AtLower, Control), Direction);
        if (!A::Valid(HalfDerivative))
            return false;
        Maximum = FMath::Max(Maximum, HalfDerivative.Upper);
    }
    Upper = Maximum;
    return Upper < 0.;
}

// A point S that can reach the literal radius anywhere on [0,L] is in the
// intersection of the original control hull and the radius-expanded prefix
// hull. Tighten each expansion by the unavoidable squared gaps on the other
// axes. At L, a strictly negative (C(L)-S)·D suffices on this conditional
// set because its derivative in time is |D|² >= 0. A contact before L would
// therefore still be inside at L. The caller still proves original full
// provider clearance at L in the fused enclosure. This is not an Rs miss.
inline bool ProveReachablePrefixDecreasing(const C::FIntervalTensorSurface& Surface,
    const C::FIntervalBezierCenterPath& Path, const C::FIntervalPoint& AtLower,
    A::Interval RadiusSquared, C::FDistanceBudget& Budget)
{
    const int32 Count = Surface.ControlPoints.Num();
    constexpr int32 FixedWork = 64;
    if (Path.Degree != 1 || Path.ControlPoints.Num() != 2 || Count <= 0 ||
        !A::Valid(RadiusSquared) || RadiusSquared.Lower <= 0. ||
        Budget.ConsumptionFloor < 0 || int64(Budget.Remaining)-Budget.ConsumptionFloor < int64(Count)+FixedWork ||
        Budget.Used > MAX_uint64-uint64(Count+FixedWork) || !Pay(Budget,Count+FixedWork))
        return false;
    auto SurfaceBox = Surface.ControlPoints[0];
    if (!C::Detail::ValidPoint(SurfaceBox))
        return false;
    for (int32 I = 1; I < Count; ++I)
    {
        if (!C::Detail::ValidPoint(Surface.ControlPoints[I]))
            return false;
        SurfaceBox = C::TimeDetail::Hull(SurfaceBox, Surface.ControlPoints[I]);
    }
    const auto Centers = C::TimeDetail::Hull(Path.ControlPoints[0], AtLower);
    const auto Direction = Subtract(Path.ControlPoints[1], Path.ControlPoints[0]);
    if (!C::Detail::ValidPoint(Centers) || !C::Detail::ValidPoint(Direction))
        return false;

    A::Interval GapSquared[3];
    for (int32 Axis = 0; Axis < 3; ++Axis)
    {
        const auto Offset = A::Subtract(SurfaceBox.Axis[Axis], Centers.Axis[Axis]);
        if (!A::Valid(Offset))
            return false;
        const double Gap = FMath::Max(0., FMath::Max(Offset.Lower, -Offset.Upper));
        GapSquared[Axis] = A::Square(A::Interval::Exact(Gap));
        if (!A::Valid(GapSquared[Axis]))
            return false;
    }

    C::FIntervalPoint Reachable;
    for (int32 Axis = 0; Axis < 3; ++Axis)
    {
        const auto Rest = A::Add(GapSquared[(Axis+1)%3], GapSquared[(Axis+2)%3]);
        const auto ReachSquared = A::Subtract(RadiusSquared, Rest);
        if (!A::Valid(ReachSquared))
            return false;
        if (ReachSquared.Upper < 0.)
            return true; // No point on this provider can contact during the prefix.
        // sqrt only proposes a bound. The directed square independently
        // proves its adequacy, regardless of native libm rounding.
        const double Reach = FMath::Sqrt(ReachSquared.Upper) + 1.;
        const auto Verified = A::Square(A::Interval::Exact(Reach));
        if (!A::Valid(Verified) || Verified.Lower < ReachSquared.Upper)
            return false;
        const auto Expanded = A::Add(Centers.Axis[Axis], A::Interval{-Reach, Reach, true});
        if (!A::Valid(Expanded))
            return false;
        const double Lower = FMath::Max(SurfaceBox.Axis[Axis].Lower, Expanded.Lower);
        const double Upper = FMath::Min(SurfaceBox.Axis[Axis].Upper, Expanded.Upper);
        if (Lower > Upper)
            return true;
        Reachable.Axis[Axis] = {Lower, Upper, true};
    }
    const auto Upper = Dot(Subtract(AtLower, Reachable), Direction);
    return A::Valid(Upper) && Upper.Upper < 0.;
}

// Same original dyadic metadata and Half operations as Detail::Split. The
// enclosure reads only Net, degree and dyadic bounds/depth, never the generic
// node's sample/optimization fields. Source and both output slots are distinct.
inline bool SplitQuinticEnclosureNet(const C::Detail::FNode& Node, bool bU,
    C::Detail::FNode& Left, C::Detail::FNode& Right, bool bShareHalfScales = false)
{
    if ((bU ? Node.DepthU : Node.DepthV) >= 52 ||
        Node.DegreeU < 4 || Node.DegreeU > 5 || Node.DegreeV < 4 || Node.DegreeV > 5)
        return false;
    const double Min = bU ? Node.MinU : Node.MinV;
    const double Max = bU ? Node.MaxU : Node.MaxV;
    const double Middle = .5 * (Min + Max);
    if (!(Middle > Min && Middle < Max))
        return false;
    Left.DegreeU = Right.DegreeU = Node.DegreeU;
    Left.DegreeV = Right.DegreeV = Node.DegreeV;
    Left.DepthU = Right.DepthU = Node.DepthU + int32(bU);
    Left.DepthV = Right.DepthV = Node.DepthV + int32(!bU);
    Left.MinU = Node.MinU; Left.MaxU = bU ? Middle : Node.MaxU;
    Right.MinU = bU ? Middle : Node.MinU; Right.MaxU = Node.MaxU;
    Left.MinV = Node.MinV; Left.MaxV = bU ? Node.MaxV : Middle;
    Right.MinV = bU ? Node.MinV : Middle; Right.MaxV = Node.MaxV;
    if (Left.Net.Num() != Node.Net.Num()) Left.Net.SetNumUninitialized(Node.Net.Num());
    if (Right.Net.Num() != Node.Net.Num()) Right.Net.SetNumUninitialized(Node.Net.Num());

    const int32 VC = Node.DegreeV + 1;
    const int32 Count = bU ? Node.DegreeU + 1 : VC;
    const int32 Lanes = bU ? VC : Node.DegreeU + 1;
    C::FIntervalPoint Work[6];
    for (int32 Lane = 0; Lane < Lanes; ++Lane)
    {
        const auto Index = [=](int32 I) { return bU ? I * VC + Lane : Lane * VC + I; };
        for (int32 I = 0; I < Count; ++I) Work[I] = Node.Net[Index(I)];
        Left.Net[Index(0)] = Work[0];
        Right.Net[Index(Count - 1)] = Work[Count - 1];
        for (int32 Level = 1; Level < Count; ++Level)
        {
            if (bShareHalfScales && Count - Level > 1)
            {
                // Each interior control feeds two adjacent pairs at this level.
                // Retain its exact outward half only until the next pair.
                for (int32 Axis = 0; Axis < 3; ++Axis)
                {
                    auto LeftHalf = A::ScaleHalf(Work[0].Axis[Axis]);
                    for (int32 I = 0; I < Count - Level; ++I)
                    {
                        const auto RightHalf = A::ScaleHalf(Work[I + 1].Axis[Axis]);
                        Work[I].Axis[Axis] = A::Add(LeftHalf, RightHalf);
                        LeftHalf = RightHalf;
                    }
                }
            }
            else
                for (int32 I = 0; I < Count - Level; ++I)
                    Work[I] = C::Detail::Half(Work[I], Work[I + 1]);
            Left.Net[Index(Level)] = Work[0];
            Right.Net[Index(Count - 1 - Level)] = Work[Count - 1 - Level];
        }
    }
    return true;
}

// The generic score is max(0, Outward(max(rounded(abs(delta)*5)),up))
// on each axis. In this guarded numeric context and for valid delta bounds,
// each product has singleton inputs, is nonnegative and has the same
// rounding/refusal as the original Multiply. Keep every product and its
// order, and apply the monotone integer Outward only to the axis maxima.
inline bool ChooseReducedQuinticSplitU(const C::Detail::FNode& Node)
{
    const bool bBalanced = Node.DepthU <= Node.DepthV;
    const bool bCanU = Node.DepthU < 52, bCanV = Node.DepthV < 52;
    if (!bCanU && !bCanV) return bBalanced;
    if (!bCanV) return true;
    if (!bCanU) return false;

    uint64 Maximum[2] = {};
    const auto Accumulate = [&](int32 Axis, const C::FIntervalPoint& First, const C::FIntervalPoint& Second)
    {
        for (int32 D = 0; D < 3; ++D)
        {
            const auto Delta = A::Subtract(First.Axis[D], Second.Axis[D]);
            if (!A::Valid(Delta))
                return false;
            const uint64 Size = FMath::Max(A::Bits(Delta.Lower) & A::Magnitude,
                A::Bits(Delta.Upper) & A::Magnitude);
            // As in Multiply(singleton, singleton), perform exactly one
            // volatile native product and reject its maximum finite value.
            volatile double Scaled = A::FromBits(Size) * 5.;
            const uint64 Bits = A::Bits(Scaled);
            if (Bits >= 0x7fefffffffffffffULL)
                return false;
            Maximum[Axis] = FMath::Max(Maximum[Axis], Bits);
        }
        return true;
    };
    constexpr int32 Stride = 6;
    for (int32 U = 0; U < Stride; ++U)
        for (int32 V = 0; V < Stride; ++V)
        {
            const int32 I = U * Stride + V;
            if (U+1 < Stride && !Accumulate(0, Node.Net[I], Node.Net[I+Stride]))
                return bBalanced;
            if (V+1 < Stride && !Accumulate(1, Node.Net[I], Node.Net[I+1]))
                return bBalanced;
        }
    double UUpper = 0., VUpper = 0.;
    if (!A::Outward(A::FromBits(Maximum[0]), true, UUpper) ||
        !A::Outward(A::FromBits(Maximum[1]), true, VUpper))
        return bBalanced;
    return UUpper == VUpper ? bBalanced : UUpper > VUpper;
}

inline bool Partition(const C::FIntervalTensorSurface& Surface, const C::FIntervalPoint& First,
    const C::FIntervalPoint& Last, FVector3d SeedAxis, A::Interval RadiusSquared, C::FDistanceBudget& Budget,
    bool bPrefix, double LeafWidth, FPartition& Result, FObservation* Fused = nullptr, double ObservedRadius = 0.,
    bool bRootAxis = false, bool bCacheSupports = false, FObservation* SupportAccount = nullptr,
    bool bReuseSplitStorage = false, bool bRetainChart = false, bool bShareHalfScales = false,
    bool bReduceQuinticSplitScores = false)
{
    const auto Center = C::TimeDetail::Hull(First, Last);
    struct FNodeBundle
    {
        C::Detail::FNode Position, Du, Dv;
    };
    const bool bReuse = bReuseSplitStorage && !bPrefix && Surface.DegreeU == 5 && Surface.DegreeV == 5;
    TArray<FNodeBundle> Stack;
    TArray<TUniquePtr<FNodeBundle>, TInlineAllocator<64>> Storage;
    TArray<int32, TInlineAllocator<64>> Active, Free;
    const auto Acquire = [&]()
    {
        if (!Free.IsEmpty()) return Free.Pop(EAllowShrinking::No);
        return Storage.Add(MakeUnique<FNodeBundle>());
    };
    if (bReuse) Active.Add(Acquire());
    ON_SCOPE_EXIT { if (bReuse && SupportAccount) SupportAccount->EnclosureSplitStorageSlots += Storage.Num(); };
    auto& Root = bReuse ? *Storage[Active.Last()] : Stack.AddDefaulted_GetRef();
    Root.Position.DegreeU = Surface.DegreeU;
    Root.Position.DegreeV = Surface.DegreeV;
    Root.Position.Net = Surface.ControlPoints;
    if (bRootAxis)
    {
        C::FIntervalPoint RootPoint;
        if (!Pay(Budget, Root.Position.Net.Num()) || !NetHull(Root.Position, RootPoint))
            return false;
        SeedAxis = ApproximateAxis(First, RootPoint);
    }
    if (!Pay(Budget, 2 * Root.Position.Net.Num()) ||
        !OriginalDerivativeNet(Root.Position, true, Root.Du) ||
        !OriginalDerivativeNet(Root.Position, false, Root.Dv))
        return false;
    TOptional<FDirectionalSupport> SeedSupport;
    if (bCacheSupports)
        SeedSupport.Emplace(First, Last, SeedAxis, RadiusSquared);
    while (bReuse ? !Active.IsEmpty() : !Stack.IsEmpty())
    {
        const int32 Slot = bReuse ? Active.Pop(EAllowShrinking::No) : INDEX_NONE;
        ON_SCOPE_EXIT { if (bReuse) Free.Add(Slot); };
        const FNodeBundle OriginalStorage = bReuse ? FNodeBundle{} : Stack.Pop(EAllowShrinking::No);
        const auto& Bundle = bReuse ? *Storage[Slot] : OriginalStorage;
        const auto& Node = Bundle.Position;
        Result.MaxDepth = FMath::Max(Result.MaxDepth, Node.DepthU + Node.DepthV);
        if (++Result.Visited > 5000 || Node.DepthU + Node.DepthV > 60 || !Pay(Budget, 4 * Node.Net.Num())) return false;
        C::FIntervalPoint Point;
        if (!NetHull(Node, Point)) return false;
        const auto Offset = Subtract(Point, Center);
        const auto Distance = SquaredNorm(Offset);
        if (!A::Valid(Distance)) return false;
        if (Distance.Lower > RadiusSquared.Upper)
            continue;
        if (SeedSupport.IsSet()
            ? StrictDirectionalClear(Node, SeedSupport.GetValue(), true, SupportAccount)
            : StrictDirectionalClear(Node, FDirectionalSupport(First, Last, SeedAxis, RadiusSquared), false, SupportAccount))
            continue;
        ++Result.AuxiliaryAxes;
        // The midpoint supplies only a direction. The actual support bound
        // still considers every coefficient, whichever proposal is used.
        if (StrictDirectionalClear(Node, FDirectionalSupport(First, Last, ApproximateAxis(Last, Point), RadiusSquared),
                bCacheSupports, SupportAccount))
            continue;
        // KKT excludes only impossible closest points of the original patch.
        {
            C::FIntervalPoint DU, DV;
            // Identical restriction in original [0,1] coordinates, whose
            // positive Jacobian factors do not affect the KKT signs.
            if (!Pay(Budget, Bundle.Du.Net.Num() + Bundle.Dv.Net.Num()) ||
                !NetHull(Bundle.Du, DU) || !NetHull(Bundle.Dv, DV)) return false;
            const auto GU = Dot(Offset, DU), GV = Dot(Offset, DV);
            if (!A::Valid(GU) || !A::Valid(GV)) return false;
            if (!CanContainMinimum(GU, Node.MinU, Node.MaxU) || !CanContainMinimum(GV, Node.MinV, Node.MaxV)) continue;
            if (Fits(Point, LeafWidth))
            {
                if (Fused)
                {
                    // [L,U] exclusions already exclude every constrained
                    // minimum at L. Only retained leaves need a fresh strict
                    // support proof at L. Failure leaves the observation Unknown.
                    const double Began = FPlatformTime::Seconds();
                    const uint64 Before = Budget.Used;
                    ++Fused->EndpointLeafTests;
                    ON_SCOPE_EXIT
                    {
                        Fused->EndpointLeafWork += Budget.Used - Before;
                        Fused->EndpointLeafMs += 1000. * (FPlatformTime::Seconds() - Began);
                    };
                    if (!Pay(Budget, 4 * Node.Net.Num()) ||
                        !StrictDirectionalClear(Node, FDirectionalSupport(First, First, SeedAxis, RadiusSquared)))
                        return false;
                }
                if (ObservedRadius > 0.)
                {
                    C::FIntervalPoint LeafNormal;
                    if (!Pay(Budget, 16) || !RadialNormal(Center, Point, ObservedRadius, LeafNormal))
                        return false;
                    const FVector2d MinUV(Node.MinU, Node.MinV), MaxUV(Node.MaxU, Node.MaxV);
                    Result.ObservedLeaves.Add({MinUV, MaxUV, Point, LeafNormal});
                    if (bRetainChart && Pay(Budget, 3))
                    {
                        // Du/Dv were differentiated on the original patch, then
                        // restricted along exactly this same dyadic route. Their
                        // cross has original UV units. A leaf-local reparametrization
                        // multiplies it by two strictly positive side widths.
                        C::FIntervalPoint Cross;
                        bool bNonzero = false;
                        for (int32 D = 0; D < 3; ++D)
                        {
                            Cross.Axis[D] = A::Subtract(
                                A::Multiply(DU.Axis[(D + 1) % 3], DV.Axis[(D + 2) % 3]),
                                A::Multiply(DU.Axis[(D + 2) % 3], DV.Axis[(D + 1) % 3]));
                            bNonzero |= Cross.Axis[D].Lower > 0. || Cross.Axis[D].Upper < 0.;
                        }
                        if (bNonzero && C::Detail::ValidPoint(Cross))
                            Result.Charts.Add({MinUV, MaxUV, Cross});
                    }
                }
                Result.Point = Result.Leaves ? C::TimeDetail::Hull(Result.Point, Point) : Point;
                ++Result.Leaves;
                if (bPrefix) return true;
                continue;
            }
        }
        const bool bReduceScores = bReduceQuinticSplitScores && !bPrefix &&
            Node.DegreeU == 5 && Node.DegreeV == 5 && Node.Net.Num() == 36;
        if (bReduceScores && SupportAccount) ++SupportAccount->EnclosureReducedSplitScores;
        const bool bSplitU = bReduceScores ? ChooseReducedQuinticSplitU(Node) :
            C::Detail::ChooseTensorDistanceSplitU(Node);
        if (bReuse)
        {
            const int32 LeftSlot = Acquire(), RightSlot = Acquire();
            auto& Left = *Storage[LeftSlot];
            auto& Right = *Storage[RightSlot];
            if (!Pay(Budget, 4 * Node.Net.Num()) ||
                !SplitQuinticEnclosureNet(Node, bSplitU, Left.Position, Right.Position, bShareHalfScales) ||
                !Pay(Budget, 4 * (Bundle.Du.Net.Num() + Bundle.Dv.Net.Num())) ||
                !SplitQuinticEnclosureNet(Bundle.Du, bSplitU, Left.Du, Right.Du, bShareHalfScales) ||
                !SplitQuinticEnclosureNet(Bundle.Dv, bSplitU, Left.Dv, Right.Dv, bShareHalfScales))
                return false;
            if (SupportAccount) ++SupportAccount->EnclosureReusableSplits;
            Active.Add(RightSlot);
            Active.Add(LeftSlot);
            continue;
        }
        FNodeBundle Left, Right;
        if (!Pay(Budget, 4 * Node.Net.Num()) || !C::Detail::Split(Node, bSplitU, Left.Position, Right.Position))
            return false;
        if (!Pay(Budget, 4 * (Bundle.Du.Net.Num() + Bundle.Dv.Net.Num())) ||
            !C::Detail::Split(Bundle.Du, bSplitU, Left.Du, Right.Du) ||
            !C::Detail::Split(Bundle.Dv, bSplitU, Left.Dv, Right.Dv))
            return false;
        Stack.Add(MoveTemp(Right));
        Stack.Add(MoveTemp(Left));
    }
    return true;
}

}

// The proposal is never an accepted hit. A successful result proves a strict
// initial shell miss, a full positive-time prefix miss at Radius, an inside
// witness at Upper, and an enclosure of all first minima on a single provider.
// No scalar time/normal, prepared evidence, FWorldHit, or response is emitted.
inline FObservation Observe(const FIndexedIntervalUnionPtr& Scene, const FQuery& Query,
    const FProposal& Proposal, const FLimits& Limits, FDistanceBudget& Budget)
{
    namespace D = Detail;
    namespace A = ::AnalyticCertified;
    FObservation Result;
    const auto Numeric = InspectCurrentAnalyticNumericContext(D::Contract);
    if (Numeric.Status != EAnalyticNumericStatus::Complete || !Scene || Scene->Num() <= 0 ||
        Proposal.ProviderIndex < 0 || Proposal.ProviderIndex >= Scene->Num() ||
        !A::Finite(Proposal.UV.X) || !A::Finite(Proposal.UV.Y) ||
        !(0. <= Proposal.UV.X && Proposal.UV.X <= 1. && 0. <= Proposal.UV.Y && Proposal.UV.Y <= 1.) ||
        !A::Finite(Proposal.Lower) || !A::Finite(Proposal.Upper) ||
        !(0. < Proposal.Lower && Proposal.Lower < Proposal.Upper && Proposal.Upper <= 1.) ||
        !A::Finite(Query.Radius) || !A::Finite(Query.DomainTolerance) || !A::Finite(Query.InitialOverlapTolerance) ||
        Query.Radius <= 0. || Query.DomainTolerance < 0. || Query.InitialOverlapTolerance < 0. ||
        !A::Finite(Limits.PointWidth) || !A::Finite(Limits.NormalWidth) ||
        Limits.PointWidth <= 0. || Limits.NormalWidth <= 0. || Limits.Providers <= 0 ||
        Limits.Work <= 0 || Budget.ConsumptionFloor < 0 || Budget.Remaining <= Budget.ConsumptionFloor)
        return Result;

    const int32 Available = Budget.Remaining - Budget.ConsumptionFloor;
    FScopedDistanceBudgetFloor Floor(Budget, Budget.Remaining - FMath::Min(Available, Limits.Work));
    if (!Floor.IsValid())
        return Result;
    const uint64 Before = Budget.Used;
    const auto ObserveImpl = [&]
    {
        Result.Stage = EStage::Partition;
        FIntervalBezierCenterPath Path, InitialPath;
        FReferenceCandidateSetPtr Candidates, Initial;
        const auto Shell = A::Add(A::Add(A::Interval::Exact(Query.Radius),
            A::Interval::Exact(Query.DomainTolerance)), A::Interval::Exact(Query.InitialOverlapTolerance));
        const auto R2 = A::Square(A::Interval::Exact(Query.Radius));
        const auto Select = [&](const FIntervalBezierCenterPath& CandidatePath, double Radius,
                                 FReferenceCandidateSetPtr& Selection)
        {
            return FReferenceCandidateSet::Select(Scene, CandidatePath, Radius, Budget, Selection) &&
                Selection && Selection->GetScene() == Scene &&
                Selection->GetCandidateIndices().Num() + Selection->GetExcludedIndices().Num() == Scene->Num() &&
                Selection->GetCandidateIndices().Num() == Selection->GetSurfacesForFixedSource().Num();
        };
        {
            const double Began = FPlatformTime::Seconds();
            ON_SCOPE_EXIT { Result.SelectMs = 1000. * (FPlatformTime::Seconds() - Began); };
            if (!A::Valid(Shell) || !A::Valid(R2) ||
                !CenterDetail::Linear(Query.Start, Query.End, Path) ||
                !CenterDetail::Linear(Query.Start, Query.Start, InitialPath) ||
                !Select(InitialPath, Shell.Upper, Initial) || !Select(Path, Query.Radius, Candidates))
                return;
        }
        const auto Indices = Candidates->GetCandidateIndices();
        const auto Surfaces = Candidates->GetSurfacesForFixedSource();
        Result.CandidateCount = Indices.Num();
        Result.ExcludedCount = Candidates->GetExcludedIndices().Num();
        if (Indices.IsEmpty() || Indices.Num() > Limits.Providers ||
            Initial->GetCandidateIndices().Num() > Limits.Providers)
            return;

        Result.Stage = EStage::InitialShell;
        {
            const double Began = FPlatformTime::Seconds();
            ON_SCOPE_EXIT { Result.ShellMs = 1000. * (FPlatformTime::Seconds() - Began); };
            const bool bNoCandidates = Initial->GetCandidateIndices().IsEmpty();
            if (Limits.bTryDirectedInitialShell && !bNoCandidates)
            {
                const int32 ScoutCap = FMath::Min(32768, (Budget.Remaining - Budget.ConsumptionFloor) / 4);
                if (ScoutCap > 0)
                {
                    FScopedDistanceBudgetFloor Scout(Budget, Budget.Remaining - ScoutCap);
                    const auto ShellSquared = A::Square(A::Interval::Exact(Shell.Upper));
                    if (Scout.IsValid() && A::Valid(ShellSquared))
                    {
                        const double ScoutStart = FPlatformTime::Seconds();
                        const uint64 ScoutBefore = Budget.Used;
                        Result.bDirectedShellAttempted = true;
                        Result.bDirectedShellClosed = true;
                        const auto& Center = InitialPath.ControlPoints[0];
                        for (const auto& Surface : Initial->GetSurfacesForFixedSource())
                        {
                            D::FPartition Row;
                            const bool Closed = D::Partition(Surface, Center, Center, FVector3d::ZeroVector,
                                ShellSquared, Budget, true, .25 * Limits.PointWidth, Row, nullptr, 0., true);
                            Result.DirectedShellNodes += Row.Visited;
                            if (!Closed || Row.Leaves)
                            {
                                Result.bDirectedShellClosed = false;
                                break;
                            }
                        }
                        Result.DirectedShellWork = Budget.Used - ScoutBefore;
                        Result.DirectedShellMs = 1000. * (FPlatformTime::Seconds() - ScoutStart);
                    }
                }
            }
            // Reuse the original classifier on the unchanged stationary path
            // and original roster if the optional directed proof is inconclusive.
            if (!bNoCandidates && !Result.bDirectedShellClosed &&
                BoundTensorSphereFirstContact(Initial->GetSurfacesForFixedSource(), InitialPath,
                    Shell.Upper, Budget, 1. / 1048576., 1.e-8, 32768, Limits.PointWidth,
                    Limits.NormalWidth).Status != ETensorSphereContactStatus::NoContactWithProvidedTensors)
                return;
        }

        Result.Stage = EStage::UpperWitness;
        FIntervalPoint LowerCenter, UpperCenter, PointSample;
        {
            const double Began = FPlatformTime::Seconds();
            ON_SCOPE_EXIT { Result.WitnessMs = 1000. * (FPlatformTime::Seconds() - Began); };
            const int32 Local = Indices.Find(Proposal.ProviderIndex);
            if (!Surfaces.IsValidIndex(Local))
                return;
            LowerCenter = CenterDetail::Evaluate(Path, A::Interval::Exact(Proposal.Lower));
            UpperCenter = CenterDetail::Evaluate(Path, A::Interval::Exact(Proposal.Upper));
            if (!Certified::Detail::ValidPoint(LowerCenter) || !Certified::Detail::ValidPoint(UpperCenter) ||
                !D::Pay(Budget, 4 * Surfaces[Local].ControlPoints.Num()) ||
                !ProjectedDistanceDetail::EvaluateFixedUV(Surfaces[Local], Proposal.UV, PointSample))
                return;
            const auto D2 = ContactDetail::SquaredNorm(ContactDetail::Subtract(UpperCenter, PointSample));
            if (!A::Valid(D2) || D2.Upper >= R2.Lower)
                return;
        }
        const auto Axis = D::ApproximateAxis(LowerCenter, PointSample);
        double LeafWidth = .25 * Limits.PointWidth;
        if (Limits.bScaleLeafForNormalWidth)
        {
            const auto Centers = TimeDetail::Hull(LowerCenter, UpperCenter);
            double CenterWidth = 0.;
            for (int32 I = 0; I < 3; ++I)
            {
                const auto Span = A::Subtract(A::Interval::Exact(Centers.Axis[I].Upper),
                    A::Interval::Exact(Centers.Axis[I].Lower));
                if (!A::Valid(Span))
                    return;
                CenterWidth = FMath::Max(CenterWidth, Span.Upper);
            }
            const auto Remaining = A::Subtract(A::Multiply(A::Interval::Exact(Query.Radius),
                A::Interval::Exact(Limits.NormalWidth)), A::Interval::Exact(CenterWidth));
            if (!A::Valid(Remaining) || Remaining.Lower <= 0.)
                return;
            // Reserve half of the available point span for the union of
            // retained leaves. The final point and radial tests are unchanged.
            LeafWidth = FMath::Min(LeafWidth, .5 * Remaining.Lower);
        }
        if (!(LeafWidth > 0.) || !A::Finite(LeafWidth))
            return;

        TArray<uint8, TInlineAllocator<32>> FusedProviders;
        if (Limits.bFuseMonotonePrefix)
            FusedProviders.Init(0, Surfaces.Num());

        Result.Stage = EStage::Prefix;
        {
            const double Began = FPlatformTime::Seconds();
            ON_SCOPE_EXIT { Result.PrefixMs = 1000. * (FPlatformTime::Seconds() - Began); };
            for (int32 I = 0; I < Surfaces.Num(); ++I)
            {
                const auto& Surface = Surfaces[I];
                const uint64 MonoBefore = Budget.Used;
                double DerivativeUpper = 0.;
                bool bEndpointOnly = D::ProvePrefixDistanceDecreasing(Surface, Path, LowerCenter, Budget, DerivativeUpper);
                if (!bEndpointOnly && Limits.bFuseMonotonePrefix && Limits.bFuseReachableMonotonePrefix)
                {
                    const uint64 ReachableBefore = Budget.Used;
                    ++Result.ReachableMonotoneAttempts;
                    bEndpointOnly = D::ProveReachablePrefixDecreasing(Surface, Path, LowerCenter, R2, Budget);
                    Result.ReachableMonotoneAccepted += uint64(bEndpointOnly);
                    Result.ReachableMonotoneWork += Budget.Used - ReachableBefore;
                }
                Result.MonotoneWork += Budget.Used - MonoBefore;
                Result.MonotoneCount += int32(bEndpointOnly);
                if (Limits.bFuseMonotonePrefix && bEndpointOnly)
                {
                    FusedProviders[I] = 1;
                    continue;
                }
                const auto& PrefixBegin = bEndpointOnly ? LowerCenter : Path.ControlPoints[0];
                D::FPartition Row;
                const bool Closed = D::Partition(Surface, PrefixBegin, LowerCenter, Axis, R2, Budget,
                    true, LeafWidth, Row);
                Result.PrefixNodes += Row.Visited;
                if (!Closed || Row.Leaves)
                    return;
            }
        }

        Result.Stage = EStage::Enclosure;
        int32 Winner = INDEX_NONE;
        FIntervalPoint Point, Normal;
        TArray<FObservedUVLeaf> WinnerLeaves;
        TArray<FObservedChart> WinnerCharts;
        {
            const double Began = FPlatformTime::Seconds();
            ON_SCOPE_EXIT { Result.EnclosureMs = FMath::Max(0.,
                1000. * (FPlatformTime::Seconds() - Began) - Result.EndpointLeafMs); };
            for (int32 I = 0; I < Indices.Num(); ++I)
            {
                D::FPartition Row;
                FObservation* Fused = Limits.bFuseMonotonePrefix && FusedProviders[I] ? &Result : nullptr;
                const bool Closed = D::Partition(Surfaces[I], LowerCenter, UpperCenter, Axis, R2, Budget,
                    false, LeafWidth, Row, Fused, Limits.bFuseMonotonePrefix ? Query.Radius : 0.,
                    false, Limits.bCacheEnclosureSupports, &Result, Limits.bReuseQuinticEnclosureSplitStorage,
                    Limits.bRetainLeafChart, Limits.bShareQuinticHalfScales, Limits.bReduceQuinticSplitScores);
                Result.EnclosureNodes += Row.Visited;
                Result.Leaves += Row.Leaves;
                if (!Closed)
                    return;
                if (Row.Leaves)
                {
                    if (Winner != INDEX_NONE)
                        return;
                    Winner = Indices[I];
                    Point = Row.Point;
                    WinnerLeaves = MoveTemp(Row.ObservedLeaves);
                    WinnerCharts = MoveTemp(Row.Charts);
                }
            }
            if (Winner == INDEX_NONE || !ContactDetail::Fits(Point, Limits.PointWidth) ||
                !ContactDetail::RadialNormal(TimeDetail::Hull(LowerCenter, UpperCenter), Point, Query.Radius, Normal) ||
                !ContactDetail::Fits(Normal, Limits.NormalWidth))
                return;
        }
        Result.Stage = EStage::Numeric;
        const auto After = InspectCurrentAnalyticNumericContext(D::Contract);
        if (After.Status != EAnalyticNumericStatus::Complete ||
            After.ControlFingerprint() != Numeric.ControlFingerprint())
            return;
        Result.Status = EStatus::FirstContactEnclosed;
        Result.Stage = EStage::Complete;
        Result.ProviderIndex = Winner;
        Result.Lower = Proposal.Lower;
        Result.Upper = Proposal.Upper;
        Result.Point = Point;
        Result.RadialNormal = Normal;
        Result.WinnerLeaves = MoveTemp(WinnerLeaves);
        if (!WinnerCharts.IsEmpty())
        {
            Result.ChartScene = Scene;
            Result.ChartNumericFingerprint = Numeric.ControlFingerprint();
            Result.ChartProvider = Winner;
            Result.Charts = MoveTemp(WinnerCharts);
        }
    };
    ObserveImpl();
    Result.Work = Budget.Used - Before;
    return Result;
}
}
