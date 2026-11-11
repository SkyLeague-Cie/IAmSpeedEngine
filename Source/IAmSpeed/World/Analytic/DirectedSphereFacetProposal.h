#pragma once

#if WITH_EDITOR && !UE_BUILD_SHIPPING
#include "IAmSpeed/World/Analytic/AnalyticWorldQuery.h"
#include "IAmSpeed/World/Analytic/DirectedLinearSphereContactObservation.h"
#include "IAmSpeed/World/Collision/PreparedIntervalScene.h"
#include <cmath>

namespace Speed::Analytic::Certified::DirectedSphereFacetProposal
{
namespace A = ::AnalyticCertified;
namespace D = DirectedLinearSphere;

namespace Private
{
inline bool Pay(FDistanceBudget& Budget, int32 Count)
{
    for (int32 I = 0; I < Count; ++I)
        if (!Budget.Consume())
            return false;
    return true;
}

inline bool Finite(const FVector3d& Value)
{
    return A::Finite(Value.X) && A::Finite(Value.Y) && A::Finite(Value.Z);
}

inline FTensorBezierSurface Derivative(const FTensorBezierSurface& In, bool AlongU)
{
    FTensorBezierSurface Out;
    Out.DegreeU = In.DegreeU - int32(AlongU);
    Out.DegreeV = In.DegreeV - int32(!AlongU);
    const int32 Stride = In.DegreeV + 1;
    const int32 Offset = AlongU ? Stride : 1;
    const int32 Degree = AlongU ? In.DegreeU : In.DegreeV;
    for (int32 U = 0; U <= Out.DegreeU; ++U)
        for (int32 V = 0; V <= Out.DegreeV; ++V)
        {
            const int32 I = U * Stride + V;
            Out.ControlPoints.Add((In.ControlPoints[I + Offset] - In.ControlPoints[I]) * double(Degree));
        }
    return Out;
}

inline bool Solve(double (&Matrix)[3][4], FVector3d& Step)
{
    // Pivoted elimination. A failed or singular seed gives no proposal.
    for (int32 I = 0; I < 3; ++I)
    {
        int32 Pivot = I;
        for (int32 J = I + 1; J < 3; ++J)
            if (FMath::Abs(Matrix[J][I]) > FMath::Abs(Matrix[Pivot][I]))
                Pivot = J;
        if (!A::Finite(Matrix[Pivot][I]) || FMath::Abs(Matrix[Pivot][I]) < 1.e-24)
            return false;
        for (int32 K = I; K < 4; ++K)
            Swap(Matrix[Pivot][K], Matrix[I][K]);
        const double Divisor = Matrix[I][I];
        for (int32 K = I; K < 4; ++K)
            Matrix[I][K] /= Divisor;
        for (int32 J = 0; J < 3; ++J)
            if (J != I)
            {
                const double Factor = Matrix[J][I];
                for (int32 K = I; K < 4; ++K)
                    Matrix[J][K] -= Factor * Matrix[I][K];
            }
    }
    Step = FVector3d(Matrix[0][3], Matrix[1][3], Matrix[2][3]);
    return Finite(Step);
}
}

// Diagnostic proposal only. Owner identity derives from the complete authored
// key; the ordinary approximation triangle's primitive is deliberately not
// used as the original patch key. No contact follows without D::Observe.
inline bool TryPropose(const Prepared::FScene& Scene, const D::FQuery& Query,
    const FWorldHit& OrdinaryHit, FDistanceBudget& Budget, D::FProposal& Out, int32 GridSteps = 16)
{
    Out = {};
    // A coarse grid changes only the proposal. The directed strict witness
    // and all full-union admission checks remain mandatory.
    if (GridSteps != 4 && GridSteps != 16)
        return false;
    const auto Before = RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    const auto& Union = Scene.GetUnion();
    const auto Owners = Scene.GetOwners();
    if (!OrdinaryHit.bHit || OrdinaryHit.bStartPenetrating || !OrdinaryHit.SourceId ||
        !OrdinaryHit.SurfaceId || !(0. < OrdinaryHit.Time && OrdinaryHit.Time < 1.) ||
        !Private::Finite(OrdinaryHit.Point) || !Private::Finite(Query.Start) ||
        !Private::Finite(Query.End) || !CandidateDetail::PositiveRadius(Query.Radius) ||
        !Union || Union->Num() != Owners.Num())
        return false;

    int32 Index = INDEX_NONE;
    for (int32 I = 0; I < Owners.Num(); ++I)
    {
        if (!Budget.Consume())
            return false;
        const auto& Owner = Owners[I];
        if (Owner.SourceId == OrdinaryHit.SourceId && Owner.SurfaceId == OrdinaryHit.SurfaceId &&
            Owner.FeatureId == OrdinaryHit.FeatureId && Owner.CanonicalGroupId == OrdinaryHit.CanonicalGroupId &&
            Owner.MaterialId == OrdinaryHit.MaterialId)
        {
            if (Index != INDEX_NONE)
                return false;
            Index = I;
        }
    }
    if (!Owners.IsValidIndex(Index) || !Owners[Index].QueryEnabled || Owners[Index].FromPlane ||
        !Owners[Index].PrimitiveId)
        return false;

    FIntervalTensorSurface Original;
    if (!Union->CopySurfaceForQuery(Index, Budget, Original) ||
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
            const auto& Value = Control.Axis[Axis];
            if (!A::Valid(Value) || A::Bits(Value.Lower) != A::Bits(Value.Upper))
                return false;
            Point[Axis] = Value.Lower;
        }
        Surface.ControlPoints.Add(Point);
    }

    const int32 Count = Surface.ControlPoints.Num();
    const int32 EvaluationWork = 4 * Count;
    if (!Private::Pay(Budget, 5 * Count))
        return false;
    const auto Su = Private::Derivative(Surface, true);
    const auto Sv = Private::Derivative(Surface, false);
    const auto Suu = Private::Derivative(Su, true);
    const auto Suv = Private::Derivative(Su, false);
    const auto Svv = Private::Derivative(Sv, false);
    const auto Dot = [](const FVector3d& AValue, const FVector3d& BValue)
    { return FVector3d::DotProduct(AValue, BValue); };
    const auto Clamp = [](double Value) { return FMath::Clamp(Value, 0., 1.); };
    double Best = TNumericLimits<double>::Max();
    FVector2d UV = FVector2d::ZeroVector;
    for (int32 I = 0; I <= GridSteps; ++I)
        for (int32 J = 0; J <= GridSteps; ++J)
        {
            if (!Private::Pay(Budget, EvaluationWork))
                return false;
            const FVector2d Trial(double(I) / GridSteps, double(J) / GridSteps);
            const auto Residual = Surface.Evaluate(Trial.X, Trial.Y) - OrdinaryHit.Point;
            const double Error = Dot(Residual, Residual);
            if (A::Finite(Error) && Error < Best)
            {
                Best = Error;
                UV = Trial;
            }
        }
    if (Best == TNumericLimits<double>::Max())
        return false;

    for (int32 Iteration = 0; Iteration < 6; ++Iteration)
    {
        if (!Private::Pay(Budget, 4 * EvaluationWork))
            return false;
        const auto Residual = Surface.Evaluate(UV.X, UV.Y) - OrdinaryHit.Point;
        const auto U = Su.Evaluate(UV.X, UV.Y);
        const auto V = Sv.Evaluate(UV.X, UV.Y);
        const double UU = Dot(U, U), UVProduct = Dot(U, V), VV = Dot(V, V);
        const double Det = UU * VV - UVProduct * UVProduct;
        if (!A::Finite(Det) || Det <= 1.e-24)
            return false;
        const double GU = Dot(Residual, U), GV = Dot(Residual, V);
        const FVector2d Step((VV * GU - UVProduct * GV) / Det, (UU * GV - UVProduct * GU) / Det);
        if (!A::Finite(Step.X) || !A::Finite(Step.Y))
            return false;
        const FVector2d Trial(Clamp(UV.X - Step.X), Clamp(UV.Y - Step.Y));
        const auto TrialResidual = Surface.Evaluate(Trial.X, Trial.Y) - OrdinaryHit.Point;
        const double Error = Dot(TrialResidual, TrialResidual);
        if (!A::Finite(Error) || Error >= Best)
            break;
        Best = Error;
        UV = Trial;
    }

    const FVector3d Direction = Query.End - Query.Start;
    const double R2 = Query.Radius * Query.Radius;
    double Time = OrdinaryHit.Time;
    for (int32 Iteration = 0; Iteration < 8; ++Iteration)
    {
        if (!Private::Pay(Budget, 6 * EvaluationWork))
            return false;
        const auto Dp = Surface.Evaluate(UV.X, UV.Y) - (Query.Start + Direction * Time);
        const auto U = Su.Evaluate(UV.X, UV.Y), V = Sv.Evaluate(UV.X, UV.Y);
        const double GU = Dot(Dp, U), GV = Dot(Dp, V);
        const double UVHessian = Dot(U, V) + Dot(Dp, Suv.Evaluate(UV.X, UV.Y));
        double Matrix[3][4] = {
            {2. * GU, 2. * GV, -2. * Dot(Dp, Direction), Dot(Dp, Dp) - R2},
            {Dot(U, U) + Dot(Dp, Suu.Evaluate(UV.X, UV.Y)), UVHessian, -Dot(Direction, U), GU},
            {UVHessian, Dot(V, V) + Dot(Dp, Svv.Evaluate(UV.X, UV.Y)), -Dot(Direction, V), GV}
        };
        FVector3d Step;
        if (!Private::Solve(Matrix, Step))
            return false;
        UV = FVector2d(Clamp(UV.X - Step.X), Clamp(UV.Y - Step.Y));
        Time = Clamp(Time - Step.Z);
    }

    // These values are candidates only. The directed upper witness check is
    // a cheap refusal before the original full global Observe proof.
    constexpr double DyadicScale = double(1 << 28);
    UV = FVector2d(std::floor(UV.X * DyadicScale + .5) / DyadicScale,
                   std::floor(UV.Y * DyadicScale + .5) / DyadicScale);
    if (!(0. < UV.X && UV.X < 1. && 0. < UV.Y && UV.Y < 1.) ||
        !(0. < Time && Time < 1.) || !Private::Pay(Budget, EvaluationWork))
        return false;
    FIntervalPoint Point;
    if (!ProjectedDistanceDetail::EvaluateFixedUV(Original, UV, Point))
        return false;
    FIntervalBezierCenterPath Chord;
    if (!CenterDetail::Linear(Query.Start, Query.End, Chord))
        return false;
    const auto Radius2 = A::Square(A::Interval::Exact(Query.Radius));
    const double Tick = std::ceil(Time * DyadicScale);
    const double Lower = (Tick - 2.) / DyadicScale;
    const double Upper = (Tick + 1.) / DyadicScale;
    if (!(0. < Lower && Lower < Upper && Upper < 1.))
        return false;
    const auto Center = CenterDetail::Evaluate(Chord, A::Interval::Exact(Upper));
    const auto Distance2 = ContactDetail::SquaredNorm(ContactDetail::Subtract(Center, Point));
    const auto After = RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if (!A::Valid(Radius2) || !A::Valid(Distance2) || Distance2.Upper >= Radius2.Lower ||
        Before.ControlFingerprint() != After.ControlFingerprint())
        return false;
    Out = {Index, UV, Lower, Upper};
    return true;
}
}
#endif

