#pragma once
// Experimental wheel-sized moving Sphere node miss filter, default1 on supported targets.
// Supported Win64/MSVC Development+Shipping; no Box/raw/stationary/ball expansion.
#include "IAmSpeed/World/Simulation/FrameFloatingPointContext.h"
#include <cmath>
#include <limits>
#if PLATFORM_WINDOWS && defined(_MSC_VER) && defined(_M_X64) && !defined(__clang__) && !defined(_M_ARM64EC)
namespace Speed::Analytic::Private::TensorCollarSphereNodeBounds
{
using FLeafSupport = TensorGenericCellStripBounds::FSupport;
struct FQueryContext
{
    TensorGenericCellStripBounds::FQueryContext Native;
    bool Ready=false,Ineligible=false;
};
inline bool PrepareQuery(const FWorldQuery& Q,bool FirstSweep,EAnalyticNumericContract Contract,FQueryContext& Out)
{
    Out={};
    const auto Saved=Speed::CaptureFrameFloatingPointContext();if(!Saved.bValid)return false;
    Speed::FScopedFrameFloatingPointContext Restore(Saved);if(!Restore.IsApplied())return false;
    const auto KnownRetain=[&](){Out.Ineligible=Restore.Restore();return false;};
    if(!FirstSweep||Q.Shape!=EQueryShape::Sphere||!Q.bAuthorityOnly||!Q.bIncludeCompactPatches)return KnownRetain();
    if(!TensorGenericCellStripBounds::Finite(Q.Start)||!TensorGenericCellStripBounds::Finite(Q.End)||
       !FMath::IsFinite(Q.Radius)||!FMath::IsFinite(Q.DomainTolerance)||!FMath::IsFinite(Q.InitialOverlapTolerance))return false;
    if(Q.Start==Q.End||(Q.Radius!=12.510000228881836&&Q.Radius!=17.010000228881836)||
       Q.Start.GetAbsMax()>16384.||Q.End.GetAbsMax()>16384.||Q.DomainTolerance<0||Q.DomainTolerance>1.||
       Q.InitialOverlapTolerance<0||Q.InitialOverlapTolerance>1.)return KnownRetain();
    const bool Ready=TensorGenericCellStripBounds::PrepareQuery(Q,FirstSweep,Contract,Out.Native);
    Out.Ready=Restore.Restore()&&Ready;return Out.Ready;
}
inline double Down(double X){return std::nextafter(X,-std::numeric_limits<double>::infinity());}
inline double Up(double X){return std::nextafter(X,std::numeric_limits<double>::infinity());}
inline double AddUp(double A,double B){return Up(A+B);}
inline double MulUp(double A,double B){return Up(A*B);}
struct FInterval {double Lower=0,Upper=0;};
inline FInterval Projection(const FVector3d& A,const FVector3d& P)
{
    const double X=A.X*P.X,Y=A.Y*P.Y,Z=A.Z*P.Z;
    return {Down(Down(Down(X)+Down(Y))+Down(Z)),Up(Up(Up(X)+Up(Y))+Up(Z))};
}
struct FSupport
{
    FVector3d Axis=FVector3d::ZeroVector;
    double Lower=0,Upper=0,MaximumError=0,Scale=0,NormUpper=0;
};
static_assert(sizeof(FSupport)==64,"Node shadow support footprint");
struct FLayout
{
    const FAnalyticWorldData* World=nullptr;
    const FTensorBezierPatch* Patches=nullptr;
    int32 PatchCount=0;
    uint64 SourceRevision=0;
    const FTensorBezierPatch* Patch=nullptr;
    const void* Cells=nullptr;const void* Nodes=nullptr;const void* Permutation=nullptr;
    int32 CellCount=0,NodeCount=0,PermutationCount=0;
    uint64 Primitive=0,Source=0,Surface=0,Group=0;
    EAnalyticNumericContract Contract=EAnalyticNumericContract::Unspecified;
    bool Matches(const FAnalyticWorldData& W,const FTensorBezierPatch& P)const
    {
        // Same immutable-world/service-lifetime contract as the public Generic index.
        // SourceRevision is equality to the captured revision, NOT a WorldHash allowlist.
        return &W==World&&W.TensorBezierPatches.GetData()==Patches&&W.TensorBezierPatches.Num()==PatchCount&&
            W.SourceHash==SourceRevision&&W.NumericContract==Contract&&
            &P==Patch&&P.PrimitiveId==Primitive&&P.SourceId==Source&&P.SurfaceId==Surface&&P.CanonicalGroupId==Group&&
            P.bApproximationCertified&&P.bAuthorityEligible&&P.bQueryCollisionEnabled&&
            Cells==P.ApproximationCells.GetData()&&CellCount==P.ApproximationCells.Num()&&
            Nodes==P.ApproximationCellBvhNodes.GetData()&&NodeCount==P.ApproximationCellBvhNodes.Num()&&
            Permutation==P.ApproximationCellBvhIndices.GetData()&&PermutationCount==P.ApproximationCellBvhIndices.Num();
    }
};
inline bool Selected(const FTensorBezierPatch& P)
{
    if(P.SourceId!=0x11af315a281440a9ull)return false;
    return
      (P.PrimitiveId==0x03b461d6a0c10a62ull&&P.SurfaceId==0x7166e4da2d9ce2a0ull&&P.CanonicalGroupId==0x7301166bbeba5c90ull)||
      (P.PrimitiveId==0x28047cc9d8d7d704ull&&P.SurfaceId==0x4e320278e32f9f8aull&&P.CanonicalGroupId==0x951a276aa5a978d6ull)||
      (P.PrimitiveId==0x521cf485f41280c0ull&&P.SurfaceId==0x1a7c9720b775841eull&&P.CanonicalGroupId==0x6b1c44a3163d3a6aull)||
      (P.PrimitiveId==0xff609f7fbf7a4fa6ull&&P.SurfaceId==0x7ad27bb3175fb7acull&&P.CanonicalGroupId==0xb147dc0207b88d9cull);
}
struct FProvider
{
    FLayout Layout;
    TArray<FSupport> NodeSupports;
};
inline TSharedPtr<const FProvider> BuildProviderUnscoped(const FAnalyticWorldData& W,const FTensorBezierPatch& P)
{
    // Caller must have captured/restored FP state and checked the same complete native
    // nearest/no-flush controls used by PrepareQuery. No geometry-generation field is invented.
    bool Owned=false;for(const auto& Candidate:W.TensorBezierPatches)if(&Candidate==&P){Owned=true;break;}
    if(!Owned||!Selected(P)||!P.bApproximationCertified||!P.bAuthorityEligible||!P.bQueryCollisionEnabled||
       P.Surface.DegreeU!=5||P.Surface.DegreeV!=1||P.ApproximationCells.Num()!=4096||
       P.ApproximationCellBvhNodes.Num()!=1023||P.ApproximationCellBvhIndices.Num()!=4096)return {};
    const auto Numeric=InspectCurrentAnalyticNumericContext(W.NumericContract);
    if(Numeric.Status!=EAnalyticNumericStatus::Complete||Numeric.ControlFingerprint()!=0x0000027f00001f80ull)return {};
    auto Out=MakeShared<FProvider>();
    Out->Layout={&W,W.TensorBezierPatches.GetData(),W.TensorBezierPatches.Num(),W.SourceHash,
      &P,P.ApproximationCells.GetData(),P.ApproximationCellBvhNodes.GetData(),P.ApproximationCellBvhIndices.GetData(),
      P.ApproximationCells.Num(),P.ApproximationCellBvhNodes.Num(),P.ApproximationCellBvhIndices.Num(),
      P.PrimitiveId,P.SourceId,P.SurfaceId,P.CanonicalGroupId,W.NumericContract};
    Out->NodeSupports.SetNum(P.ApproximationCellBvhNodes.Num());
    TArray<uint8> NodeSeen,CellSeen;NodeSeen.Init(0,P.ApproximationCellBvhNodes.Num());CellSeen.Init(0,P.ApproximationCells.Num());
    TArray<TArray<int32>> Descendants;Descendants.SetNum(P.ApproximationCellBvhNodes.Num());
    const auto Visit=[&](auto&& Self,int32 N,int32 Depth)->bool
    {
        if(Depth>63||!NodeSeen.IsValidIndex(N)||NodeSeen[N])return false;
        NodeSeen[N]=1;const auto& Node=P.ApproximationCellBvhNodes[N];auto& Cells=Descendants[N];
        if(Node.IsLeaf())
        {
            if(Node.LeftChild!=INDEX_NONE||Node.RightChild!=INDEX_NONE||Node.FirstIndex<0||Node.IndexCount<=0||
               int64(Node.FirstIndex)+int64(Node.IndexCount)>P.ApproximationCellBvhIndices.Num())return false;
            for(int32 K=0;K<Node.IndexCount;++K)
            {
                const int32 C=P.ApproximationCellBvhIndices[Node.FirstIndex+K];
                if(!CellSeen.IsValidIndex(C)||CellSeen[C])return false;CellSeen[C]=1;Cells.Add(C);
            }
        }
        else
        {
            if(Node.IndexCount!=0||!Self(Self,Node.LeftChild,Depth+1)||!Self(Self,Node.RightChild,Depth+1))return false;
            Cells.Append(Descendants[Node.LeftChild]);Cells.Append(Descendants[Node.RightChild]);
        }
        if(Cells.IsEmpty())return false;
        // ONE axis per node. Never union intervals computed on different leaf axes.
        const FLeafSupport Seed=TensorGenericCellStripBounds::PrepareCell(P.ApproximationCells[Cells[0]]);
        if(!(Seed.Scale>0))return false;
        FSupport S;S.Axis=Seed.Axis;S.Scale=1;
        const FInterval NormSquared=Projection(S.Axis,S.Axis);
        if(!(NormSquared.Upper>0)||!FMath::IsFinite(NormSquared.Upper))return false;
        S.NormUpper=Up(FMath::Sqrt(NormSquared.Upper));
        bool First=true;
        for(int32 C:Cells)
        {
            const auto& Cell=P.ApproximationCells[C];
            if(!FMath::IsFinite(Cell.MaximumErrorCm)||Cell.MaximumErrorCm<0)return false;
            S.MaximumError=FMath::Max(S.MaximumError,Cell.MaximumErrorCm);
            for(const auto& Corner:Cell.Corners)
            {
                if(!TensorGenericCellStripBounds::Finite(Corner)||Corner.GetAbsMax()>1.e6)return false;
                S.Scale=FMath::Max(S.Scale,Corner.GetAbsMax());const FInterval R=Projection(S.Axis,Corner);
                if(!FMath::IsFinite(R.Lower)||!FMath::IsFinite(R.Upper))return false;
                S.Lower=First?R.Lower:FMath::Min(S.Lower,R.Lower);S.Upper=First?R.Upper:FMath::Max(S.Upper,R.Upper);First=false;
            }
        }
        if(!FMath::IsFinite(S.NormUpper))return false;
        Out->NodeSupports[N]=S;return true;
    };
    if(!Visit(Visit,0,0))return {};
    for(uint8 Seen:NodeSeen)if(!Seen)return {};
    for(uint8 Seen:CellSeen)if(!Seen)return {};
    return TSharedPtr<const FProvider>(Out);
}
inline bool ProposedMissUnscoped(const FSupport& S,const TensorGenericCellStripBounds::FQueryContext& Q,bool& Known)
{
    Known=false;
    if(!(S.Scale>0)||!(Q.Scale>0))return false;
    const FInterval A=Projection(S.Axis,Q.Start),B=Projection(S.Axis,Q.End);
    const double Lo=FMath::Min(A.Lower,B.Lower),Hi=FMath::Max(A.Upper,B.Upper);
    const double Gap=FMath::Max(Down(S.Lower-Hi),Down(Lo-S.Upper));
    const double Scale=FMath::Max(S.Scale,Q.Scale);
    // sqrt(3)*MaximumError also covers any per-coordinate error-box interpretation.
    const double Error=MulUp(Up(FMath::Sqrt(3.)),S.MaximumError);
    const double Shell=MulUp(AddUp(AddUp(Q.BaseShell,Error),MulUp(1.e-9,Scale)),S.NormUpper);
    // Full original chord, NOT incumbent prefix: includes all native endpoint/fallback evaluations.
    // Conditional active caller may prune only Known&&WouldMiss after original node bounds.
    Known=FMath::IsFinite(Lo)&&FMath::IsFinite(Hi)&&FMath::IsFinite(Gap)&&FMath::IsFinite(Shell);
    return Known&&Gap>Shell;
}
inline TSharedPtr<const FProvider> BuildProvider(const FAnalyticWorldData& W,const FTensorBezierPatch& P)
{
    const auto Saved=Speed::CaptureFrameFloatingPointContext();if(!Saved.bValid)return {};
    Speed::FScopedFrameFloatingPointContext Restore(Saved);if(!Restore.IsApplied())return {};
    const auto Result=BuildProviderUnscoped(W,P);
    return Restore.Restore()?Result:TSharedPtr<const FProvider>();
}
struct FObservation {bool Known=false,WouldMiss=false;};
inline FObservation ObserveNode(const FProvider& Provider,const FAnalyticWorldData& W,const FTensorBezierPatch& P,
    int32 Node,const FQueryContext& PreparedQuery)
{
    if(!Provider.Layout.Matches(W,P)||!Provider.NodeSupports.IsValidIndex(Node)||!PreparedQuery.Ready)return {};
    const auto Saved=Speed::CaptureFrameFloatingPointContext();if(!Saved.bValid)return {};
    Speed::FScopedFrameFloatingPointContext Restore(Saved);if(!Restore.IsApplied())return {};
    const auto Numeric=InspectCurrentAnalyticNumericContext(W.NumericContract);
    if(Numeric.Status!=EAnalyticNumericStatus::Complete||Numeric.ControlFingerprint()!=0x0000027f00001f80ull)return {};
    bool Known=false;
    const bool Miss=ProposedMissUnscoped(Provider.NodeSupports[Node],PreparedQuery.Native,Known);
    return Restore.Restore()?FObservation{Known,Miss}:FObservation{};
}

// Same synchronous provider-local binding; no persistent query result/cache.
struct FBinding
{
    const FAnalyticWorldData* World=nullptr;const FTensorBezierPatch* Patch=nullptr;
    const FWorldQuery* NativeQuery=nullptr;const FProvider* Node=nullptr;const FQueryContext* NodeQuery=nullptr;
};
inline thread_local const FBinding* GBinding=nullptr;
struct FScopedBinding
{
    FBinding Value;const FBinding* Previous=nullptr;
    FScopedBinding(const FAnalyticWorldData& W,const FTensorBezierPatch& P,const FWorldQuery& Q,
        const FProvider* N,const FQueryContext* NQ):Value{&W,&P,&Q,N,NQ},Previous(GBinding){GBinding=&Value;}
    ~FScopedBinding(){GBinding=Previous;}
    FScopedBinding(const FScopedBinding&)=delete;FScopedBinding& operator=(const FScopedBinding&)=delete;
};
}
#endif
