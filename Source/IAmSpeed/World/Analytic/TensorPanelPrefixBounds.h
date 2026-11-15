#pragma once
// C347 experimental authenticated prefix bound. Default OFF; native retained kernels unchanged.
#if !UE_BUILD_SHIPPING && PLATFORM_WINDOWS && defined(_MSC_VER) && defined(_M_X64) && !defined(__clang__) && !defined(_M_ARM64EC)
#include "AnalyticWorldQuery.h"
#include "AnalyticNumericContext.h"
#include "Hash/Blake3.h"
#include "IAmSpeed/World/Simulation/FrameFloatingPointContext.h"
#include <cmath>
#include <limits>
namespace Speed::Analytic::Private::TensorPanelPrefixBounds
{
inline bool MatchesIdentity(const FTensorBezierPatch& Patch)
{
    return Patch.SourceId==0x11af315a281440a9ull &&
        Patch.SurfaceId==0xf69c1748c4c58304ull &&
        Patch.PrimitiveId==0x42f0db19591de3deull &&
        Patch.CanonicalGroupId==0xf53e5e40a84f6024ull;
}
template<typename FHash>
bool GeometryDigest(const FTensorBezierPatch& Patch, FHash&& Hash)
{
    if (!MatchesIdentity(Patch) || !Patch.bApproximationCertified ||
        !Patch.bAuthorityEligible || !Patch.bQueryCollisionEnabled ||
        Patch.Surface.DegreeU != 5 || Patch.Surface.DegreeV != 1 ||
        Patch.Surface.ControlPoints.Num() != 12 ||
        Patch.ApproximationCells.Num() != 4096 ||
        Patch.ApproximationCellBvhIndices.Num() != Patch.ApproximationCells.Num() ||
        Patch.ApproximationCellBvhNodes.Num() != 1023)
        return false;

    TArray<uint8> Bytes;
    Bytes.Reserve(1100000);
    static constexpr char Domain[] = "SL.CollarPair.DirectTensorBVH/v1";
    Bytes.Append(reinterpret_cast<const uint8*>(Domain), sizeof(Domain));
    const auto Word = [&](uint64 Value, uint32 Width)
    {
        for (uint32 Index = 0; Index < Width; ++Index)
            Bytes.Add(uint8(Value >> (8 * Index)));
    };
    const auto Double = [&](double Value)
    {
        static_assert(sizeof(double) == sizeof(uint64));
        uint64 Bits = 0;
        FMemory::Memcpy(&Bits, &Value, sizeof(Bits));
        Word(Bits, 8);
    };
    const auto Vector = [&](const FVector3d& Value)
    {
        Double(Value.X);
        Double(Value.Y);
        Double(Value.Z);
    };
    const auto Box = [&](const FBox3d& Value)
    {
        Word(Value.IsValid ? 1 : 0, 1);
        Vector(Value.Min);
        Vector(Value.Max);
    };
    for (uint64 Id : {Patch.SourceId, Patch.SurfaceId, Patch.PrimitiveId, Patch.CanonicalGroupId})
        Word(Id, 8);
    Double(Patch.MaximumApproximationErrorCm);
    Word(uint32(Patch.Surface.DegreeU), 4);
    Word(uint32(Patch.Surface.DegreeV), 4);
    Word(uint32(Patch.Surface.ControlPoints.Num()), 4);
    for (const auto& Control : Patch.Surface.ControlPoints) Vector(Control);

    Word(uint32(Patch.ApproximationCells.Num()), 4);
    for (const auto& Cell : Patch.ApproximationCells)
    {
        Double(Cell.MinimumU);
        Double(Cell.MaximumU);
        Double(Cell.MinimumV);
        Double(Cell.MaximumV);
        for (const auto& Corner : Cell.Corners) Vector(Corner);
        Double(Cell.MaximumErrorCm);
        for (uint8 Flags : Cell.PositiveConcaveEdges) Word(Flags, 1);
        for (uint8 Flags : Cell.NegativeConcaveEdges) Word(Flags, 1);
    }
    Word(uint32(Patch.ApproximationCellBvhNodes.Num()), 4);
    for (const auto& Node : Patch.ApproximationCellBvhNodes)
    {
        Box(Node.Bounds);
        Word(uint32(Node.FirstIndex), 4);
        Word(uint32(Node.IndexCount), 4);
        Word(uint32(Node.LeftChild), 4);
        Word(uint32(Node.RightChild), 4);
    }
    Word(uint32(Patch.ApproximationCellBvhIndices.Num()), 4);
    for (int32 Index : Patch.ApproximationCellBvhIndices) Word(uint32(Index), 4);
    return Hash(MakeArrayView(Bytes));
}
struct FSupport
{
    double Y=0, Z=0, Upper=0, LengthUpper=0;
};
static_assert(sizeof(FSupport)==32,"Four halfspaces use128 bytes.");
inline double Up(double X) { return std::nextafter(X,std::numeric_limits<double>::infinity()); }
inline double Down(double X) { return std::nextafter(X,-std::numeric_limits<double>::infinity()); }
inline double DotUpper(double Y,double Z,const FVector3d& P)
{ return Up(Up(Y*P.Y)+Up(Z*P.Z)); }
inline double DotLower(double Y,double Z,const FVector3d& P)
{ return Down(Down(Y*P.Y)+Down(Z*P.Z)); }

// This helper NEVER certifies the floating-point native ClosestPoint witness.
// This prospective local candidate has no universal computed-CP proof; qualification is separate.
struct FEnvelope
{
    FSupport Supports[4];
    const FTensorBezierPatch* BoundProvider=nullptr;
    double PlaneX=0, MaximumError=0;
    bool bReady=false;

    static bool NumericReady(EAnalyticNumericContract InContract)
    {
        const auto Evidence=InspectCurrentAnalyticNumericContext(InContract);
        return Evidence.Status==EAnalyticNumericStatus::Complete &&
            Evidence.ControlFingerprint()==0x0000027f00001f80ull;
    }

    bool Prepare(const FTensorBezierPatch& Patch,EAnalyticNumericContract InContract)
    {
        bReady=false;BoundProvider=nullptr;
        if (!NumericReady(InContract)) return false;
        if (!GeometryDigest(Patch,[](TConstArrayView<uint8> Bytes)
            { return LexToString(FBlake3::HashBuffer(Bytes.GetData(),Bytes.Num())).Equals(
                TEXT("4515adc48a6c6154d7b74f732bd313dda101ec5b1f9284890ba625615f53c236"),
                ESearchCase::IgnoreCase); })) return false;
        const auto& First=Patch.ApproximationCells[0];
        const auto& Last=Patch.ApproximationCells.Last();
        const FVector3d Hull[4]={First.Corners[0],Last.Corners[2],Last.Corners[3],First.Corners[1]};
        FVector3d Center=FVector3d::ZeroVector;
        for (const auto& P:Hull) Center+=P*0.25;
        PlaneX=Hull[0].X;MaximumError=0;
        for (int32 I=0;I<4;++I)
        {
            const auto& A=Hull[I];const auto& B=Hull[(I+1)%4];
            auto& S=Supports[I];S.Y=-(B.Z-A.Z);S.Z=B.Y-A.Y;
            // Orientation affects selectivity only: Upper below is computed
            // over EVERY actual stored native vertex, not just the four hull tips.
            if (S.Y*(Center.Y-A.Y)+S.Z*(Center.Z-A.Z)>0) { S.Y=-S.Y;S.Z=-S.Z; }
            const double LengthSquared=Up(Up(S.Y*S.Y)+Up(S.Z*S.Z));
            S.LengthUpper=Up(FMath::Sqrt(LengthSquared));
            S.Upper=-std::numeric_limits<double>::infinity();
            if (!FMath::IsFinite(S.LengthUpper) || S.LengthUpper<=0) return false;
        }
        for (const auto& Cell:Patch.ApproximationCells)
        {
            if (!FMath::IsFinite(Cell.MaximumErrorCm) || Cell.MaximumErrorCm<0) return false;
            MaximumError=FMath::Max(MaximumError,Cell.MaximumErrorCm);
            for (const auto& P:Cell.Corners)
            {
                if (P.ContainsNaN() || P.X!=PlaneX || P.GetAbsMax()>16384.) return false;
                for (auto& S:Supports) S.Upper=FMath::Max(S.Upper,DotUpper(S.Y,S.Z,P));
            }
        }
        for (const auto& S:Supports) if (!FMath::IsFinite(S.Upper)) return false;
        BoundProvider=&Patch;bReady=true;return true;
    }

};


// Global-result certificate candidate, not a claim that the full chord misses.
// The original fallback may sample past H, but a returned Time>H cannot beat
// this positive strict incumbent. The native CP rounding theorem is UNKNOWN.
inline bool IsSeparated(const FEnvelope& Envelope,const FTensorBezierPatch& Patch,
    const FWorldQuery& Query,const FWorldHit& Incumbent,double H)
{
    if (!Envelope.bReady || Envelope.BoundProvider!=&Patch || !Incumbent.bHit ||
        !FMath::IsFinite(Incumbent.Time) || Incumbent.Time<=1.e-6 ||
        !FMath::IsFinite(H) || H<=0. || H>=1. || H!=Incumbent.Time+1.e-12) return false;
    constexpr double Pad=0x1p-33;
    FVector3d Lo,Hi;
    for (int32 Axis=0;Axis<3;++Axis)
    {
        const double A=Query.Start[Axis],B=Query.End[Axis];
        const double DLo=Down(B-A),DHi=Up(B-A);
        const double ELo=Down(A+Down(DLo*H)),EHi=Up(A+Up(DHi*H));
        Lo[Axis]=Down(FMath::Min(A,ELo)-Pad);
        Hi[Axis]=Up(FMath::Max(A,EHi)+Pad);
    }
    const double XGap=FMath::Max(0.,FMath::Max(Down(Envelope.PlaneX-Hi.X),Down(Lo.X-Envelope.PlaneX)));
    double YZGap=0.;
    for (const auto& S:Envelope.Supports)
    {
        const FVector3d Extreme(0.,S.Y>=0.?Lo.Y:Hi.Y,S.Z>=0.?Lo.Z:Hi.Z);
        const double Gap=FMath::Max(0.,Down(DotLower(S.Y,S.Z,Extreme)-S.Upper));
        YZGap=FMath::Max(YZGap,FMath::Max(0.,Down(Gap/S.LengthUpper)));
    }
    const double DistanceSquaredLower=Down(Down(XGap*XGap)+Down(YZGap*YZGap));
    const double Shell=Up(Up(Up(Query.Radius+1.e-8)+Envelope.MaximumError)+0.1);
    const double ShellSquaredUpper=Up(Shell*Shell);
    return FMath::IsFinite(DistanceSquaredLower) && FMath::IsFinite(ShellSquaredUpper) &&
        DistanceSquaredLower>ShellSquaredUpper;
}

}
#endif
