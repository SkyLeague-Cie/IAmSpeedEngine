#pragma once

#include "CoreMinimal.h"
#include "IAmSpeed/Base/Kinematic.h"
#include <cfloat>

// The bounded Aerial full-stick solve, including the existing angular limits.
// Rejection leaves the original coupled impulse unchanged.
namespace Speed::CarBallContact
{
struct FCapAware576Diagnostic
{
    int32 Reason = 0; // 1 no clipping; 2 invalid; 3 nonclosing; 4 singular;
                     // 5 nonconverged; 6 cone; 7 energy; 8 accepted; 9 inertia.
    int32 Iterations = 0;
    double BeforeResidual = -1.0;
    double AfterResidual = -1.0;
    double NumericTolerance = -1.0;
    double EnergyBefore = -1.0;
    double EnergyAfter = -1.0;
    double RequiredMu = -1.0;
    double CarLimitUsed = -1.0;
    double BallLimitUsed = -1.0;
};

static bool SolveColumns576(const FVector (&Columns)[3], const FVector& B, FVector& Out)
{
    double A[3][4] = {};
    double Scale = 0.0;
    for (int32 Row = 0; Row < 3; ++Row)
    {
        for (int32 Col = 0; Col < 3; ++Col)
        {
            A[Row][Col] = Columns[Col][Row];
            if (!FMath::IsFinite(A[Row][Col])) return false;
            Scale = FMath::Max(Scale, FMath::Abs(A[Row][Col]));
        }
        A[Row][3] = B[Row];
    }
    if (!(Scale > 0.0) || B.ContainsNaN()) return false;
    for (int32 Col = 0; Col < 3; ++Col)
    {
        int32 Pivot = Col;
        for (int32 Row = Col + 1; Row < 3; ++Row)
            if (FMath::Abs(A[Row][Col]) > FMath::Abs(A[Pivot][Col])) Pivot = Row;
        if (FMath::Abs(A[Pivot][Col]) <= 64.0 * DBL_EPSILON * Scale) return false;
        for (int32 J = Col; J < 4; ++J)
        {
            const double Temp = A[Col][J]; A[Col][J] = A[Pivot][J]; A[Pivot][J] = Temp;
        }
        const double Divisor = A[Col][Col];
        for (int32 J = Col; J < 4; ++J) A[Col][J] /= Divisor;
        for (int32 Row = 0; Row < 3; ++Row)
        {
            if (Row == Col) continue;
            const double Factor = A[Row][Col];
            for (int32 J = Col; J < 4; ++J) A[Row][J] -= Factor * A[Col][J];
        }
    }
    Out = FVector(A[0][3], A[1][3], A[2][3]);
    return !Out.ContainsNaN();
}

static FVector RadialDerivative576(const FVector& Unclipped, const double Limit, const FVector& Delta)
{
    const double Size = Unclipped.Size();
    if (Size <= Limit) return Delta;
    return (Limit / Size) * (Delta - Unclipped * (FVector::DotProduct(Unclipped, Delta) / (Size * Size)));
}

static bool CapAwareFullStick576(
    const SKinematic& Car, const double CarMass, const FMatrix& CarInvI, const double CarLimit,
    const SKinematic& Ball, const double BallMass, const FMatrix& BallInvI, const double BallLimit,
    const FVector& Point, const FVector& NormalBoxToSphere,
    const double Restitution, const double Mu, FVector& InOutBallImpulse, FCapAware576Diagnostic& D)
{
    D.CarLimitUsed = CarLimit; D.BallLimitUsed = BallLimit;
    D.Reason = 2;
    if (!(CarMass > 0.0 && BallMass > 0.0 && CarLimit > 0.0 && BallLimit > 0.0) ||
        !FMath::IsFinite(CarMass) || !FMath::IsFinite(BallMass) ||
        !FMath::IsFinite(CarLimit) || !FMath::IsFinite(BallLimit) ||
        !FMath::IsFinite(Mu) || Mu < 0.0 || Restitution != 0.0 ||
        Point.ContainsNaN() || Car.Location.ContainsNaN() || Ball.Location.ContainsNaN() ||
        NormalBoxToSphere.ContainsNaN() || InOutBallImpulse.ContainsNaN()) return false;
    const FVector N = NormalBoxToSphere.GetSafeNormal();
    if (N.IsNearlyZero()) return false;
    const FVector RC = Point - Car.Location, RB = Point - Ball.Location;
    const FVector PreRelative = Ball.Velocity + FVector::CrossProduct(Ball.AngularVelocity, RB) -
        Car.Velocity - FVector::CrossProduct(Car.AngularVelocity, RC);
    const double PreRelativeSpeed = PreRelative.Size();
    if (PreRelative.ContainsNaN() || !FMath::IsFinite(PreRelativeSpeed)) return false;
    D.Reason = 3;
    if (FVector::DotProduct(PreRelative, N) >= 0.0) return false;
    struct FState576 { FVector VC, VB, WC, WB, UC, UB, Relative; };
    const auto State = [&](const FVector& J)
    {
        FState576 S;
        S.VC = Car.Velocity - J / CarMass; S.VB = Ball.Velocity + J / BallMass;
        S.UC = Car.AngularVelocity - FVector(CarInvI.TransformVector(FVector::CrossProduct(RC, J)));
        S.UB = Ball.AngularVelocity + FVector(BallInvI.TransformVector(FVector::CrossProduct(RB, J)));
        S.WC = S.UC.GetClampedToMaxSize(CarLimit); S.WB = S.UB.GetClampedToMaxSize(BallLimit);
        S.Relative = S.VB + FVector::CrossProduct(S.WB, RB) - S.VC - FVector::CrossProduct(S.WC, RC);
        return S;
    };
    const auto ValidState = [](const FState576& S)
    {
        return !S.VC.ContainsNaN() && !S.VB.ContainsNaN() && !S.WC.ContainsNaN() && !S.WB.ContainsNaN() &&
            !S.UC.ContainsNaN() && !S.UB.ContainsNaN() && !S.Relative.ContainsNaN() && FMath::IsFinite(S.Relative.Size());
    };
    FVector J = InOutBallImpulse;
    FState576 S = State(J);
    if (!ValidState(S)) return false;
    D.BeforeResidual = S.Relative.Size();
    D.AfterResidual = D.BeforeResidual;
    D.Reason = 1;
    // Preserve the old impulse BIT FOR BIT when no angular clipping is predicted.
    if (S.UC.SizeSquared() <= CarLimit * CarLimit && S.UB.SizeSquared() <= BallLimit * BallLimit) return false;
    const FVector Axes[3] = { FVector(1,0,0), FVector(0,1,0), FVector(0,0,1) };
    D.NumericTolerance = 256.0 * DBL_EPSILON * FMath::Max(1.0, PreRelativeSpeed);
    for (int32 Iteration = 0; Iteration < 8 && S.Relative.Size() > D.NumericTolerance; ++Iteration)
    {
        FVector Columns[3];
        for (int32 Axis = 0; Axis < 3; ++Axis)
        {
            const FVector DC = RadialDerivative576(S.UC, CarLimit, CarInvI.TransformVector(FVector::CrossProduct(RC, Axes[Axis])));
            const FVector DB = RadialDerivative576(S.UB, BallLimit, BallInvI.TransformVector(FVector::CrossProduct(RB, Axes[Axis])));
            Columns[Axis] = (1.0 / CarMass + 1.0 / BallMass) * Axes[Axis] +
                FVector::CrossProduct(DC, RC) + FVector::CrossProduct(DB, RB);
        }
        FVector Delta;
        D.Reason = 4;
        if (!SolveColumns576(Columns, -S.Relative, Delta)) return false;
        J += Delta; S = State(J); D.Iterations = Iteration + 1;
        if (J.ContainsNaN() || !ValidState(S)) return false;
    }
    D.AfterResidual = S.Relative.Size();
    D.Reason = 5;
    if (!(D.AfterResidual <= D.NumericTolerance)) return false;
    const double JN = FVector::DotProduct(J, N);
    const double JT = (J - JN * N).Size();
    D.Reason = 6;
    if (!FMath::IsFinite(JN) || !FMath::IsFinite(JT) || !(JN > 0.0) || JT > Mu * JN) return false;
    D.RequiredMu = JT / JN;
    const auto AngularEnergy = [&](const FMatrix& InvI, const FVector& W, double& OutEnergy)
    {
        FVector Columns[3];
        for (int32 Axis = 0; Axis < 3; ++Axis) Columns[Axis] = InvI.TransformVector(Axes[Axis]);
        FVector L;
        if (!SolveColumns576(Columns, W, L)) return false;
        OutEnergy = 0.5 * FVector::DotProduct(W, L);
        return FMath::IsFinite(OutEnergy) && OutEnergy >= 0.0;
    };
    double EC0, EB0, EC1, EB1;
    D.Reason = 9;
    if (!AngularEnergy(CarInvI, Car.AngularVelocity, EC0) || !AngularEnergy(BallInvI, Ball.AngularVelocity, EB0) ||
        !AngularEnergy(CarInvI, S.WC, EC1) || !AngularEnergy(BallInvI, S.WB, EB1)) return false;
    const double EnergyBefore = 0.5 * CarMass * Car.Velocity.SizeSquared() + 0.5 * BallMass * Ball.Velocity.SizeSquared() + EC0 + EB0;
    const double EnergyAfter = 0.5 * CarMass * S.VC.SizeSquared() + 0.5 * BallMass * S.VB.SizeSquared() + EC1 + EB1;
    if (!FMath::IsFinite(EnergyBefore) || !FMath::IsFinite(EnergyAfter)) return false;
    D.EnergyBefore = EnergyBefore; D.EnergyAfter = EnergyAfter;
    D.Reason = 7;
    const double EnergyRoundoff = 256.0 * DBL_EPSILON * FMath::Max(1.0, D.EnergyBefore);
    if (!FMath::IsFinite(D.EnergyBefore) || !FMath::IsFinite(D.EnergyAfter) || D.EnergyAfter > D.EnergyBefore + EnergyRoundoff) return false;
    // Only this accepted physical impulse is changed; no body setter is used.
    InOutBallImpulse = J; D.Reason = 8;
    return true;
}

}
