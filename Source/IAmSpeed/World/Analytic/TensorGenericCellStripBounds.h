#pragma once
#include <cfloat>
// Finite tensor-cell strip miss filter. Retained cells use native response; computed CP theorem UNKNOWN.
namespace Speed::Analytic::Private::TensorGenericCellStripBounds
{
struct FSupport
{
    FVector3d Axis=FVector3d::ZeroVector;
    double Lower=0,Upper=0,Error=0,Scale=0; // Scale0 means retain native cell.
};
static_assert(sizeof(FSupport)==56,"Generic cell support footprint");
inline bool Finite(const FVector3d& V)
{return FMath::IsFinite(V.X)&&FMath::IsFinite(V.Y)&&FMath::IsFinite(V.Z);}
inline double Dot(const FVector3d& A,const FVector3d& B)
{return (A.X*B.X+A.Y*B.Y)+A.Z*B.Z;}
inline FSupport PrepareCell(const FTensorBezierApproximationCell& Cell)
{
    FSupport S;
    if(!FMath::IsFinite(Cell.MaximumErrorCm)||Cell.MaximumErrorCm<0)return S;
    double Scale=1;
    for(int32 K=0;K<4;++K)
    {if(!Finite(Cell.Corners[K]))return S;Scale=FMath::Max(Scale,Cell.Corners[K].GetAbsMax());}
    if(Scale>1.e6)return S;
    static constexpr int32 Edges[4][2]={{0,1},{1,3},{3,2},{2,0}};
    FVector3d LongEdge=FVector3d::ZeroVector;double Longest=0;
    for(int32 K=0;K<4;++K)
    {
        const FVector3d Edge=Cell.Corners[Edges[K][1]]-Cell.Corners[Edges[K][0]];
        const double LengthSquared=Dot(Edge,Edge);
        if(!FMath::IsFinite(LengthSquared))return S;
        if(LengthSquared>Longest){Longest=LengthSquared;LongEdge=Edge;}
    }
    // Chart-plane heuristic, NOT an assertion that a nonplanar quad has one plane.
    // Any resulting unit axis is valid for the support interval over ALL4 corners.
    FVector3d Normal=FVector3d::CrossProduct(Cell.Corners[2]-Cell.Corners[0],Cell.Corners[1]-Cell.Corners[0]);
    FVector3d Axis=FVector3d::CrossProduct(Normal,LongEdge);
    double Squared=Dot(Axis,Axis);
    if(!(Squared>1.e-24)||!FMath::IsFinite(Squared))return S;
    const double Length=FMath::Sqrt(Squared);Axis/=Length;
    const double UnitSquared=Dot(Axis,Axis);
    if(!Finite(Axis)||!FMath::IsFinite(UnitSquared)||UnitSquared<1.-32.*DBL_EPSILON||UnitSquared>1.+32.*DBL_EPSILON)return S;
    S.Axis=Axis;S.Lower=S.Upper=Dot(Axis,Cell.Corners[0]);
    for(int32 K=1;K<4;++K)
    {
        const double P=Dot(Axis,Cell.Corners[K]);if(!FMath::IsFinite(P))return FSupport{};
        S.Lower=FMath::Min(S.Lower,P);S.Upper=FMath::Max(S.Upper,P);
    }
    if(!FMath::IsFinite(S.Lower)||!FMath::IsFinite(S.Upper))return FSupport{};
    S.Error=Cell.MaximumErrorCm;S.Scale=Scale;return S;
}
struct FProvider
{
    const FTensorBezierPatch* Patch=nullptr;
    const void* Cells=nullptr;const void* Nodes=nullptr;const void* Permutation=nullptr;
    int32 CellCount=0,NodeCount=0,PermutationCount=0;
    TArray<FSupport> Supports;
    bool Matches(const FTensorBezierPatch& P) const
    {
        return Patch==&P&&Cells==P.ApproximationCells.GetData()&&CellCount==P.ApproximationCells.Num()&&
            Nodes==P.ApproximationCellBvhNodes.GetData()&&NodeCount==P.ApproximationCellBvhNodes.Num()&&
            Permutation==P.ApproximationCellBvhIndices.GetData()&&PermutationCount==P.ApproximationCellBvhIndices.Num()&&P.bApproximationCertified;
    }
};
struct FIndex
{
    const FAnalyticWorldData* BoundWorld=nullptr;const FTensorBezierPatch* Patches=nullptr;
    int32 PatchCount=0,PreparedProviders=0;int64 PreparedCells=0,ValidSupports=0;uint64 SourceHash=0;
    TArray<TSharedPtr<const FProvider>> Providers;
    const FProvider* Find(const FAnalyticWorldData& World,const FTensorBezierPatch& Patch) const
    {
        if(BoundWorld!=&World||Patches!=World.TensorBezierPatches.GetData()||PatchCount!=World.TensorBezierPatches.Num()||SourceHash!=World.SourceHash)return nullptr;
        const UPTRINT Address=reinterpret_cast<UPTRINT>(&Patch),First=reinterpret_cast<UPTRINT>(Patches);
        if(Address<First)return nullptr;const UPTRINT Offset=Address-First;
        if(Offset%sizeof(FTensorBezierPatch)!=0||Offset/sizeof(FTensorBezierPatch)>=UPTRINT(PatchCount))return nullptr;
        const int32 I=int32(Offset/sizeof(FTensorBezierPatch));
        return Providers[I]&&Providers[I]->Matches(Patch)?Providers[I].Get():nullptr;
    }
};
inline TSharedPtr<const FIndex> Build(const FAnalyticWorldData& World)
{
    auto Index=MakeShared<FIndex>();Index->BoundWorld=&World;Index->Patches=World.TensorBezierPatches.GetData();
    Index->PatchCount=World.TensorBezierPatches.Num();Index->SourceHash=World.SourceHash;Index->Providers.SetNum(Index->PatchCount);
    int32 ValidProviders=0;
    for(int32 I=0;I<Index->PatchCount;++I)
    {
        const auto& Patch=World.TensorBezierPatches[I];
        if(!Patch.bApproximationCertified||Patch.ApproximationCells.IsEmpty()||Patch.ApproximationCellBvhNodes.IsEmpty()||Patch.ApproximationCellBvhIndices.Num()!=Patch.ApproximationCells.Num())continue;
        TArray<uint8> Seen;Seen.Init(0,Patch.ApproximationCells.Num());bool Valid=true;
        for(int32 Cell:Patch.ApproximationCellBvhIndices)
        {if(!Seen.IsValidIndex(Cell)||Seen[Cell]){Valid=false;break;}Seen[Cell]=1;}
        if(!Valid)continue;
        auto P=MakeShared<FProvider>();P->Patch=&Patch;P->Cells=Patch.ApproximationCells.GetData();P->CellCount=Patch.ApproximationCells.Num();
        P->Nodes=Patch.ApproximationCellBvhNodes.GetData();P->NodeCount=Patch.ApproximationCellBvhNodes.Num();
        P->Permutation=Patch.ApproximationCellBvhIndices.GetData();P->PermutationCount=Patch.ApproximationCellBvhIndices.Num();
        P->Supports.Reserve(P->CellCount);int32 Ready=0;
        for(const auto& Cell:Patch.ApproximationCells){const auto S=PrepareCell(Cell);Ready+=int32(S.Scale>0);P->Supports.Add(S);}
        if(Ready){Index->Providers[I]=P;++ValidProviders;Index->PreparedCells+=P->CellCount;Index->ValidSupports+=Ready;}
    }
    Index->PreparedProviders=ValidProviders;return ValidProviders?TSharedPtr<const FIndex>(Index):TSharedPtr<const FIndex>();
}
struct FQueryContext
{FVector3d Start=FVector3d::ZeroVector,End=FVector3d::ZeroVector;double BaseShell=0,Scale=0;};
inline bool PrepareQuery(const FWorldQuery& Q,bool FirstSweep,EAnalyticNumericContract Contract,FQueryContext& Out)
{
    Out={};
    if(!FirstSweep||Q.Shape!=EQueryShape::Sphere||Q.Start==Q.End||!Q.bAuthorityOnly||!Q.bIncludeCompactPatches||
        (Q.Radius!=12.510000228881836&&Q.Radius!=17.010000228881836)||!Finite(Q.Start)||!Finite(Q.End)||
        Q.Start.GetAbsMax()>16384.||Q.End.GetAbsMax()>16384.||!FMath::IsFinite(Q.DomainTolerance)||Q.DomainTolerance<0||
        !FMath::IsFinite(Q.InitialOverlapTolerance)||Q.InitialOverlapTolerance<0)return false;
    const auto Numeric=InspectCurrentAnalyticNumericContext(Contract); // ONCE per direct query span.
    if(Numeric.Status!=EAnalyticNumericStatus::Complete||Numeric.ControlFingerprint()!=0x0000027f00001f80ull)return false;
    Out.Start=Q.Start;Out.End=Q.End;Out.BaseShell=Q.Radius+1.7320508075688772935*Q.DomainTolerance+Q.InitialOverlapTolerance+1.e-8+0.1;
    Out.Scale=FMath::Max(1.,FMath::Max(Q.Start.GetAbsMax(),Q.End.GetAbsMax()));return true;
}
inline bool IsMiss(const FSupport& S,const FQueryContext& Q)
{
    if(!(S.Scale>0)||!(Q.Scale>0))return false;
    const double A=Dot(S.Axis,Q.Start),B=Dot(S.Axis,Q.End);
    const double Scale=FMath::Max(S.Scale,Q.Scale);
    const double Shell=(Q.BaseShell+S.Error+1.e-9*Scale)*(1.+64.*DBL_EPSILON);
    const double Gap=FMath::Max(S.Lower-FMath::Max(A,B),FMath::Min(A,B)-S.Upper);
    // Full original chord includes the native degenerate fallback towards End.
    // Reserve is experimental geometric conservatism; computed CP theorem UNKNOWN.
    return FMath::IsFinite(A)&&FMath::IsFinite(B)&&FMath::IsFinite(Shell)&&FMath::IsFinite(Gap)&&Gap>Shell;
}
}
