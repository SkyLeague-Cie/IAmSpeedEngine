#pragma once
#include "ReferenceAffineExtrusionMath.h"

namespace Speed::Analytic::Certified::SignedAffineSphereResidual
{
namespace A = ::AnalyticCertified;

// Draft values-only proof. The caller authenticates FProduct against the
// selected fixed source and accounts for every provider in the original union.
enum class EDecision : uint8 { NotApplicable, Indeterminate, Clear, Inside, Work, Arithmetic };

struct FCenterRange
{
    FIntervalPoint Center;
    Interval ZMinusRadius;
    Interval ZPlusRadius;
};

struct FWitness
{
    FVector2d OriginalUV = FVector2d::ZeroVector;
    FIntervalPoint OriginalPoint;
    Interval SignedSquaredGap;
};

inline Interval Hull(Interval first, Interval last)
{
    if (!A::Valid(first) || !A::Valid(last)) return {};
    return {FMath::Min(first.Lower, last.Lower), FMath::Max(first.Upper, last.Upper), true};
}

inline Interval At(Interval base, Interval velocity, Interval acceleration, Interval seconds)
{
    const auto linear = A::Multiply(velocity, seconds);
    const auto halfAcceleration = A::ScaleHalf(acceleration);
    const auto quadratic = A::Multiply(halfAcceleration, A::Square(seconds));
    return A::Add(A::Add(base, linear), quadratic);
}

inline Interval Over(Interval base, double velocity, double acceleration, Interval first, Interval last)
{
    const auto v = Interval::Exact(velocity);
    const auto a = Interval::Exact(acceleration);
    const auto time = Hull(first, last);
    const auto derivative = A::Add(v, A::Multiply(a, time));
    if (!A::Valid(derivative)) return {};
    if (derivative.Lower >= 0 || derivative.Upper <= 0)
        return Hull(At(base, v, a, first), At(base, v, a, last));
    return At(base, v, a, time);
}

inline bool Valid(const FCenterRange& range)
{
    return Detail::ValidPoint(range.Center) &&
        A::Valid(range.ZMinusRadius) && A::Valid(range.ZPlusRadius);
}

// Time is the original normalized reference parameter. Subtract Rs before
// adding motion terms. Pz and Rs are the original captured binary64 values.
inline bool BuildReferenceRange(const FVector3d& position, const FVector3d& velocity,
    const FVector3d& acceleration, float horizon, double radius,
    double first, double last, FCenterRange& out)
{
    out = {};
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if (!A::Finite(radius) || !(radius > 0) || !A::Finite(double(horizon)) || !(horizon > 0) ||
        !A::Finite(first) || !A::Finite(last) || !(first >= 0 && first <= last && last <= 1))
        return false;
    const auto duration = Interval::Exact(double(horizon));
    const auto a = A::Multiply(Interval::Exact(first), duration);
    const auto b = A::Multiply(Interval::Exact(last), duration);
    FCenterRange range;
    for (int32 d = 0; d < 3; ++d)
        range.Center.Axis[d] = Over(Interval::Exact(position[d]), velocity[d], acceleration[d], a, b);
    const auto z = Interval::Exact(position.Z);
    const auto r = Interval::Exact(radius);
    range.ZMinusRadius = Over(A::Subtract(z, r), velocity.Z, acceleration.Z, a, b);
    range.ZPlusRadius = Over(A::Add(z, r), velocity.Z, acceleration.Z, a, b);
    if (!Valid(range)) return false;
    out = range;
    return true;
}

// The caller supplies a proven bound on the actual rounded SBox centers.
// This helper neither executes nor proves the rounded time DAG.
inline bool BuildRoundedRange(const FIntervalPoint& center, double radius, FCenterRange& out)
{
    out = {};
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if (!Detail::ValidPoint(center) || !A::Finite(radius) || !(radius > 0)) return false;
    const auto r = Interval::Exact(radius);
    const FCenterRange range{center, A::Subtract(center.Axis[2], r), A::Add(center.Axis[2], r)};
    if (!Valid(range)) return false;
    out = range;
    return true;
}

namespace Private
{
using AffineReferenceDetail::FProduct;
constexpr int32 CurveCount = 6;
constexpr int32 ProductCount = 11;

inline bool Admits(const FProduct& product)
{
    return (product.FreeWorldAxis == 0 || product.FreeWorldAxis == 1) &&
        product.Curve.DegreeU == 0 && product.Curve.DegreeV == CurveCount - 1 &&
        product.Curve.ControlPoints.Num() == CurveCount;
}

inline bool ValidProduct(const FProduct& product)
{
    if (!A::Valid(product.Start) || !A::Valid(product.End) ||
        !(product.End.Lower > product.Start.Upper || product.Start.Lower > product.End.Upper))
        return false;
    for (const auto& point : product.Curve.ControlPoints)
    {
        if (!Detail::ValidPoint(point)) return false;
        const auto axial = point.Axis[product.FreeWorldAxis];
        if (A::Bits(axial.Lower) != 0 || A::Bits(axial.Upper) != 0) return false;
    }
    return true;
}

struct FCell
{
    Interval Travel[CurveCount];
    Interval Normal[ProductCount];
    Interval Left[ProductCount];
    Interval Right[ProductCount];
    int32 Depth = 0;
};

inline bool Products(const Interval (&normalMinus)[CurveCount],
    const Interval (&normalPlus)[CurveCount], const Interval (&left)[CurveCount],
    const Interval (&right)[CurveCount], FCell& out)
{
    constexpr int32 binom5[CurveCount] = {1, 5, 10, 10, 5, 1};
    constexpr int32 binom10[ProductCount] = {1, 10, 45, 120, 210, 252, 210, 120, 45, 10, 1};
    for (int32 k = 0; k < ProductCount; ++k)
    {
        auto vertical = Interval::Exact(0);
        auto horizontalLeft = Interval::Exact(0);
        auto horizontalRight = Interval::Exact(0);
        for (int32 i = FMath::Max(0, k - CurveCount + 1); i <= FMath::Min(k, CurveCount - 1); ++i)
        {
            const int32 j = k - i;
            Interval weight;
            if (!ContactDetail::ProjectionDividePositive(Interval::Exact(binom5[i] * binom5[j]),
                Interval::Exact(binom10[k]), weight)) return false;
            const auto leftProduct = i == j ? A::Square(left[i]) : A::Multiply(left[i], left[j]);
            const auto rightProduct = i == j ? A::Square(right[i]) : A::Multiply(right[i], right[j]);
            vertical = A::Add(vertical, A::Multiply(weight, A::Multiply(normalMinus[i], normalPlus[j])));
            horizontalLeft = A::Add(horizontalLeft, A::Multiply(weight, leftProduct));
            horizontalRight = A::Add(horizontalRight, A::Multiply(weight, rightProduct));
        }
        out.Normal[k] = vertical;
        out.Left[k] = A::Add(vertical, horizontalLeft);
        out.Right[k] = A::Add(vertical, horizontalRight);
        if (!A::Valid(out.Normal[k]) || !A::Valid(out.Left[k]) || !A::Valid(out.Right[k])) return false;
    }
    return true;
}

template <int32 Count>
inline bool Split(const Interval (&source)[Count], Interval (&left)[Count], Interval (&right)[Count])
{
    Interval work[Count];
    for (int32 i = 0; i < Count; ++i) work[i] = source[i];
    left[0] = work[0];
    right[Count - 1] = work[Count - 1];
    for (int32 level = 1; level < Count; ++level)
    {
        for (int32 i = 0; i < Count - level; ++i)
        {
            work[i] = A::HalfSum(work[i], work[i + 1]);
            if (!A::Valid(work[i])) return false;
        }
        left[level] = work[0];
        right[Count - 1 - level] = work[Count - 1 - level];
    }
    return true;
}

inline bool Split(const FCell& parent, FCell& left, FCell& right)
{
    left.Depth = right.Depth = parent.Depth + 1;
    return Split(parent.Travel, left.Travel, right.Travel) &&
        Split(parent.Normal, left.Normal, right.Normal) &&
        Split(parent.Left, left.Left, right.Left) &&
        Split(parent.Right, left.Right, right.Right);
}

inline bool Clear(const FCell& cell, Interval travel)
{
    bool left = true, right = true;
    for (const auto& cp : cell.Travel)
    {
        left &= cp.Upper <= travel.Lower;
        right &= cp.Lower >= travel.Upper;
    }
    // If the cell straddles a clamp boundary, drop the nonnegative travel
    // term. There is no sampled decision or artificial infinite surface.
    const auto& polynomial = left ? cell.Left : (right ? cell.Right : cell.Normal);
    for (const auto& coefficient : polynomial)
        if (!(coefficient.Lower > 0)) return false;
    return true;
}
}

// The signed polynomial covers all represented reference or consumed centers.
// Ignoring the free-axis square is sound for separation, including off-chart.
// Local exhaustion is Indeterminate. Real account denial is sticky Work.
inline EDecision Exclude(const AffineReferenceDetail::FProduct& product, const FCenterRange& center,
    int32 maximumCells, FDistanceBudget& budget, int32* visited = nullptr)
{
    if (visited) *visited = 0;
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if (!Private::Admits(product) || maximumCells <= 0 || maximumCells > 256) return EDecision::NotApplicable;
    if (!Valid(center) || !Private::ValidProduct(product)) return EDecision::Arithmetic;
    if (!budget.Consume()) return EDecision::Work;

    const int32 axis = 1 - product.FreeWorldAxis;
    const auto travel = center.Center.Axis[axis];
    Private::FCell root;
    Interval normalMinus[Private::CurveCount], normalPlus[Private::CurveCount];
    Interval left[Private::CurveCount], right[Private::CurveCount];
    for (int32 v = 0; v < Private::CurveCount; ++v)
    {
        const auto& cp = product.Curve.ControlPoints[v];
        root.Travel[v] = cp.Axis[axis];
        left[v] = A::Subtract(cp.Axis[axis], Interval::Exact(travel.Lower));
        right[v] = A::Subtract(cp.Axis[axis], Interval::Exact(travel.Upper));
        normalMinus[v] = A::Subtract(center.ZMinusRadius, cp.Axis[2]);
        normalPlus[v] = A::Subtract(center.ZPlusRadius, cp.Axis[2]);
    }
    if (!Private::Products(normalMinus, normalPlus, left, right, root)) return EDecision::Arithmetic;

    TArray<Private::FCell> pending;
    pending.Add(MoveTemp(root));
    int32 count = 0;
    while (!pending.IsEmpty())
    {
        if (count >= maximumCells) return EDecision::Indeterminate;
        if (!budget.Consume()) return EDecision::Work;
        ++count;
        if (visited) *visited = count;
        const auto cell = pending.Pop(EAllowShrinking::No);
        if (Private::Clear(cell, travel)) continue;
        if (cell.Depth >= 52) return EDecision::Indeterminate;
        Private::FCell a, b;
        if (!Private::Split(cell, a, b)) return EDecision::Arithmetic;
        pending.Add(MoveTemp(b));
        pending.Add(MoveTemp(a));
    }
    return EDecision::Clear;
}

// Proposal only. The original signed witness below independently validates it.
inline bool ProposeCurveV(const AffineReferenceDetail::FProduct& product,
    const FCenterRange& center, double& out)
{
    if (!Private::Admits(product) || !Valid(center)) return false;
    const int32 travelAxis = 1 - product.FreeWorldAxis;
    const auto middle = [](Interval range) { return .5 * range.Lower + .5 * range.Upper; };
    const auto x = middle(center.Center.Axis[travelAxis]);
    const auto z = middle(center.Center.Axis[2]);
    FVector2d cp[Private::CurveCount];
    FVector2d d[Private::CurveCount - 1], dd[Private::CurveCount - 2];
    for (int32 k = 0; k < Private::CurveCount; ++k)
    {
        const auto& source = product.Curve.ControlPoints[k];
        if (!Detail::ValidPoint(source)) return false;
        cp[k] = FVector2d(middle(source.Axis[travelAxis]), middle(source.Axis[2]));
    }
    for (int32 k = 0; k < Private::CurveCount - 1; ++k) d[k] = (cp[k + 1] - cp[k]) * 5.;
    for (int32 k = 0; k < Private::CurveCount - 2; ++k) dd[k] = (d[k + 1] - d[k]) * 4.;
    for (auto& point : cp) point -= FVector2d(x,z);
    const auto at = [](const FVector2d* points, int32 count, double v)
    {
        FVector2d work[Private::CurveCount];
        for (int32 k = 0; k < count; ++k) work[k] = points[k];
        for (int32 level = count - 1; level > 0; --level)
            for (int32 k = 0; k < level; ++k)
                work[k] += (work[k + 1] - work[k]) * v;
        return work[0];
    };
    const double span = cp[Private::CurveCount - 1].X - cp[0].X;
    double v = span != 0 ? FMath::Clamp(-cp[0].X / span, 0., 1.) : .5;
    if (!A::Finite(v)) return false;
    for (int32 iteration = 0; iteration < 6; ++iteration)
    {
        const auto offset = at(cp, Private::CurveCount, v);
        const auto tangent = at(d, Private::CurveCount - 1, v);
        const auto curvature = at(dd, Private::CurveCount - 2, v);
        const auto slope = FVector2d::DotProduct(offset,tangent);
        const auto derivative = tangent.SizeSquared() + FVector2d::DotProduct(offset,curvature);
        if (!A::Finite(slope) || !A::Finite(derivative) || !(derivative > 0)) break;
        const auto next = FMath::Clamp(v - slope / derivative, 0., 1.);
        if (!A::Finite(next) || next == v) break;
        v = next;
    }
    out = v;
    return true;
}
// An untrusted V proposal can only win after the chosen ORIGINAL 3D point
// has a strictly negative signed gap for every center in this endpoint box.
inline EDecision ProveInside(const AffineReferenceDetail::FProduct& product, const FCenterRange& center,
    double curveV, FDistanceBudget& budget, FWitness& out)
{
    out = {};
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if (!Private::Admits(product)) return EDecision::NotApplicable;
    if (!Valid(center) || !Private::ValidProduct(product) || !A::Finite(curveV) || !(curveV >= 0 && curveV <= 1))
        return EDecision::Arithmetic;
    if (!budget.Consume()) return EDecision::Work;

    const auto axial = center.Center.Axis[product.FreeWorldAxis];
    const bool increasing = product.End.Lower > product.Start.Upper;
    const auto low = increasing ? product.Start : product.End;
    const auto high = increasing ? product.End : product.Start;
    if (!(axial.Lower > low.Upper && axial.Upper < high.Lower)) return EDecision::NotApplicable;
    Interval free;
    if (!AffineReferenceDetail::FreeParameter(product, center.Center, free)) return EDecision::Arithmetic;
    const double freeMid = .5 * free.Lower + .5 * free.Upper;
    const FVector2d uv = product.FreeAlongU ? FVector2d(freeMid, curveV) : FVector2d(curveV, freeMid);
    if (!budget.Consume()) return EDecision::Work;
    FIntervalPoint point;
    if (!AffineReferenceDetail::EvaluateFixedProductUV(product, uv, point)) return EDecision::Arithmetic;

    const auto dzMinus = A::Subtract(center.ZMinusRadius, point.Axis[2]);
    const auto dzPlus = A::Subtract(center.ZPlusRadius, point.Axis[2]);
    auto signedGap = A::Multiply(dzMinus, dzPlus);
    for (int32 d = 0; d < 2; ++d)
        signedGap = A::Add(signedGap, A::Square(A::Subtract(center.Center.Axis[d], point.Axis[d])));
    if (!A::Valid(signedGap)) return EDecision::Arithmetic;
    if (!(signedGap.Upper < 0)) return EDecision::Indeterminate;

    out = {uv, point, signedGap};
    return EDecision::Inside;
}
}
