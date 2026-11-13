#pragma once

// Private preview implementation, included only on the qualified Win64 target, including Shipping.
// No asset serialization or per-query mutable shared state.
#include "Hash/Blake3.h"

namespace Speed::Analytic::Private
{
namespace GeometryIdentity
{
inline bool MatchesIdentity(const FTensorBezierPatch& Patch)
{
    return Patch.SourceId == 0x11af315a281440a9ull &&
        ((Patch.SurfaceId == 0x7166e4da2d9ce2a0ull &&
          Patch.PrimitiveId == 0x03b461d6a0c10a62ull &&
          Patch.CanonicalGroupId == 0x7301166bbeba5c90ull) ||
         (Patch.SurfaceId == 0xcf93ad992354a830ull &&
          Patch.PrimitiveId == 0xefb90ece15cde4d2ull &&
          Patch.CanonicalGroupId == 0x481d5bfdd3f4c420ull));
}

// Canonical little-endian geometry wire; no query or benchmark identity enters it.
template<typename FHash>
bool Digest(const FTensorBezierPatch& Patch, FHash&& Hash)
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
    static constexpr char Domain[] = "SL.Collar20.DirectTensorBVH/v1";
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
}



struct FTensorCellObliqueIndex
{
    struct FSupport { double Y, Z, Lower, Upper; };
    static_assert(sizeof(FSupport)==32, "Keep each 4096-cell support payload at 128 KiB");
    int32 ProviderIndex=INDEX_NONE;
    TArray<FSupport> Cells;
    TArray<TSharedPtr<const FTensorCellObliqueIndex>> Children;

    const FTensorCellObliqueIndex* Find(const FAnalyticWorldData& World,
        const FTensorBezierPatch& Patch) const
    {
        for (const auto& Child:Children)
        {
            if (World.TensorBezierPatches.IsValidIndex(Child->ProviderIndex) &&
                &World.TensorBezierPatches[Child->ProviderIndex]==&Patch)
                return Child.Get();
        }
        return nullptr;
    }
};

inline bool PrepareTensorCellObliqueProvider(const FTensorBezierPatch& Patch,
    FTensorCellObliqueIndex& Provider)
{
    Provider.Cells.Reserve(Patch.ApproximationCells.Num());
    for (const auto& Cell:Patch.ApproximationCells)
    {
        // This long cell edge supplies the useful oblique strip. Its orientation
        // varies with the taper; a single provider-wide normal would be looser.
        const auto& A=Cell.Corners[1];
        const auto& B=Cell.Corners[0];
        FTensorCellObliqueIndex::FSupport S;
        S.Y=-(B.Z-A.Z); S.Z=B.Y-A.Y;
        const double Length=FMath::Sqrt(S.Y*S.Y+S.Z*S.Z);
        if (!(Length>0.) || !FMath::IsFinite(Length)) return false;
        S.Y/=Length; S.Z/=Length;
        S.Lower=S.Upper=S.Y*Cell.Corners[0].Y+S.Z*Cell.Corners[0].Z;
        if (!FMath::IsFinite(S.Y) || !FMath::IsFinite(S.Z) || !FMath::IsFinite(S.Lower)) return false;
        for (int32 K=0; K<4; ++K)
        {
            if (Cell.Corners[K].X!=-5120.) return false;
            if (K==0) continue;
            const double P=S.Y*Cell.Corners[K].Y+S.Z*Cell.Corners[K].Z;
            if (!FMath::IsFinite(P)) return false;
            S.Lower=FMath::Min(S.Lower,P); S.Upper=FMath::Max(S.Upper,P);
        }
        Provider.Cells.Add(S);
    }
    return true;
}

inline TSharedPtr<const FTensorCellObliqueIndex> BuildTensorCellObliqueIndex(const FAnalyticWorldData& World)
{
    const auto Numeric=InspectCurrentAnalyticNumericContext(World.NumericContract);
    if (Numeric.Status!=EAnalyticNumericStatus::Complete ||
        Numeric.ControlFingerprint()!=0x0000027f00001f80ull) return {};

    auto Index=MakeShared<FTensorCellObliqueIndex>();
    Index->Children.Reserve(2);
    for (const uint64 Primitive:{0x03b461d6a0c10a62ull,0xefb90ece15cde4d2ull})
    {
        int32 ProviderIndex=INDEX_NONE;
        bool bAmbiguous=false;
        for (int32 I=0; I<World.TensorBezierPatches.Num(); ++I)
        {
            const auto& Patch=World.TensorBezierPatches[I];
            if (Patch.PrimitiveId!=Primitive || !GeometryIdentity::MatchesIdentity(Patch)) continue;
            if (ProviderIndex!=INDEX_NONE) { bAmbiguous=true; break; }
            ProviderIndex=I;
        }
        if (bAmbiguous || ProviderIndex==INDEX_NONE) continue;
        const auto& Patch=World.TensorBezierPatches[ProviderIndex];
        const TCHAR* Expected=Primitive==0x03b461d6a0c10a62ull
            ? TEXT("8532cbdfab3290f8707593d165b411dd6417614ec1b5530e8500fc97c1fdaf2e")
            : TEXT("5544295f2456555837b2f3c6eaab463cf9cda36cbe93bd168ca21cfbab76148b");
        if (!GeometryIdentity::Digest(Patch,[Expected](TConstArrayView<uint8> Bytes)
            { return LexToString(FBlake3::HashBuffer(Bytes.GetData(),Bytes.Num())).Equals(
                Expected,ESearchCase::IgnoreCase); })) continue;
        auto Child=MakeShared<FTensorCellObliqueIndex>();
        Child->ProviderIndex=ProviderIndex;
        if (PrepareTensorCellObliqueProvider(Patch,*Child))
            Index->Children.Add(Child);
    }
    if (Index->Children.IsEmpty()) return {};
    return Index;
}

inline bool IsTensorCellObliqueQuery(const FWorldQuery& Query,bool bFirstSweep,
    EAnalyticNumericContract Contract)
{
    if (Query.Shape!=EQueryShape::Sphere || Query.Start==Query.End ||
        !Query.bAuthorityOnly || !Query.bIncludeCompactPatches || Query.bIncludeTriangles || Query.bExcludeTori ||
        Query.RequiredSourceId!=0 || Query.RequiredSurfaceId!=0 || Query.RequiredCanonicalGroupId!=0 ||
        Query.ReferenceNormal!=FVector3d::ZeroVector || Query.MinimumReferenceNormalDot!=-1. ||
        Query.bAllowEstablishedFaceContactAtBoundary || !bFirstSweep ||
        Query.DomainTolerance!=1.e-6 || Query.InitialOverlapTolerance!=0.05 ||
        (Query.Radius!=12.510000228881836 && Query.Radius!=17.010000228881836) ||
        !FMath::IsFinite(Query.Start.X) || !FMath::IsFinite(Query.Start.Y) || !FMath::IsFinite(Query.Start.Z) ||
        !FMath::IsFinite(Query.End.X) || !FMath::IsFinite(Query.End.Y) || !FMath::IsFinite(Query.End.Z) ||
        Query.Start.GetAbsMax()>16384. || Query.End.GetAbsMax()>16384.) return false;
    const auto Numeric=InspectCurrentAnalyticNumericContext(Contract);
    return Numeric.Status==EAnalyticNumericStatus::Complete && Numeric.ControlFingerprint()==0x0000027f00001f80ull;
}

inline bool IsTensorCellObliqueMiss(const FTensorCellObliqueIndex::FSupport& S,
    const FWorldQuery& Query,double Shell)
{
    const double A=S.Y*Query.Start.Y+S.Z*Query.Start.Z;
    const double B=S.Y*Query.End.Y+S.Z*Query.End.Z;
    const double Gap=FMath::Max(S.Lower-FMath::Max(A,B),FMath::Min(A,B)-S.Upper);
    const double Margin=Gap-Shell;
    // Full original chord, including the fallback's End. This reserve is the
    // qualified experimental scope, not a new gameplay/contact tolerance.
    return FMath::IsFinite(Margin) && Margin>0.;
}
} // namespace Speed::Analytic::Private
