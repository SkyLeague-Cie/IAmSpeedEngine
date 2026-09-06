#include "TensorContactTopology.h"
#include "AnalyticWorldData.h"

namespace Speed::Analytic
{
namespace
{
constexpr int32 TensorTopologyCornerIndices[2][3] = {{0,2,3},{0,3,1}};
struct FEdgeKey
{
	int64 Coordinate[6] = {};
	bool operator==(const FEdgeKey& Other) const
	{
		for (int32 I=0; I<6; ++I) if (Coordinate[I]!=Other.Coordinate[I]) return false;
		return true;
	}
	friend uint32 GetTypeHash(const FEdgeKey& Key)
	{
		uint32 Hash=0;
		for (const int64 C : Key.Coordinate) Hash=HashCombineFast(Hash,::GetTypeHash(C));
		return Hash;
	}
};
struct FEdgeRef { int32 Cell; int32 Approximation; uint8 Triangle; uint8 Edge; };
struct FCornerRef { int32 Cell; int32 Corner; };
// Buckets are merely an initialization accelerator. Endpoint equality and C2
// ownership are checked independently before any contact behavior can change.
FEdgeKey EdgeKey(FVector3d A, FVector3d B)
{
	if (A.X>B.X || (A.X==B.X && (A.Y>B.Y || (A.Y==B.Y && A.Z>B.Z)))) Swap(A,B);
	FEdgeKey Key;
	for (int32 Axis=0; Axis<3; ++Axis)
	{
		Key.Coordinate[Axis]=FMath::RoundToInt64(A[Axis]*1.0e6);
		Key.Coordinate[Axis+3]=FMath::RoundToInt64(B[Axis]*1.0e6);
	}
	return Key;
}
uint64 CellPair(const int32 A, const int32 B)
{
	return (uint64(uint32(FMath::Min(A,B)))<<32) | uint32(FMath::Max(A,B));
}
}

void BuildTensorContactTopology(FPiecewiseTensorBezierPatch& Patch)
{
	Patch.InternalCornerNormalCones.Reset();
	TMap<uint64,int32> CellIndices;
	for (int32 I=0; I<Patch.Cells.Num(); ++I) CellIndices.Add(Patch.Cells[I].PrimitiveId,I);
	TMap<uint64,uint16> SmoothPairs;
	for (const auto& Link : Patch.Adjacencies)
	{
		const int32* A=CellIndices.Find(Link.CellPrimitiveId);
		const int32* B=CellIndices.Find(Link.AdjacentCellPrimitiveId);
		if (A && B && Link.bC2ByConstruction && Link.BoundaryIndex<4 && Link.AdjacentBoundaryIndex<4)
		{
			const int32 FirstBoundary=*A<*B ? Link.BoundaryIndex : Link.AdjacentBoundaryIndex;
			const int32 SecondBoundary=*A<*B ? Link.AdjacentBoundaryIndex : Link.BoundaryIndex;
			SmoothPairs.FindOrAdd(CellPair(*A,*B)) |= 1u<<(4*FirstBoundary+SecondBoundary);
		}
	}
	TMap<FEdgeKey,TArray<FEdgeRef,TInlineAllocator<2>>> Edges;
	for (int32 C=0; C<Patch.Cells.Num(); ++C)
	for (int32 I=0; I<Patch.Cells[C].ApproximationCells.Num(); ++I)
	{
		auto& Cell=Patch.Cells[C].ApproximationCells[I];
		for (int32 T=0; T<2; ++T)
		{
			Cell.PositiveConcaveEdges[T]=Cell.NegativeConcaveEdges[T]=0;
			for (uint8 E=0; E<3; ++E)
			{
				const FVector3d A=Cell.Corners[TensorTopologyCornerIndices[T][E]];
				const FVector3d B=Cell.Corners[TensorTopologyCornerIndices[T][(E+1)%3]];
				// Outside the safe quantized range, retain SAT rather than overflow.
				if (A.GetAbsMax()>9.0e12 || B.GetAbsMax()>9.0e12) continue;
				Edges.FindOrAdd(EdgeKey(A,B)).Add({C,I,uint8(T),E});
			}
		}
	}
	const auto Vertex=[&Patch](const FEdgeRef& E, const int32 Offset)
	{
		return Patch.Cells[E.Cell].ApproximationCells[E.Approximation].Corners[TensorTopologyCornerIndices[E.Triangle][(E.Edge+Offset)%3]];
	};
	const auto AuthoredBoundary=[&Patch](const FEdgeRef& E) -> int32
	{
		const auto& C=Patch.Cells[E.Cell].ApproximationCells[E.Approximation];
		if (E.Triangle==0 && E.Edge==0 && C.MinimumV==0) return 2;
		if (E.Triangle==0 && E.Edge==1 && C.MaximumU==1) return 1;
		if (E.Triangle==1 && E.Edge==1 && C.MaximumV==1) return 3;
		if (E.Triangle==1 && E.Edge==2 && C.MinimumU==0) return 0;
		return INDEX_NONE;
	};
	// Hash order cannot reach physical arbitration: each unique edge only sets
	// its own independent bit. Ambiguous/non-manifold buckets remain untouched.
	for (const auto& Bucket : Edges)
	{
		if (Bucket.Value.Num()!=2) continue;
		const FEdgeRef A=Bucket.Value[0], B=Bucket.Value[1];
		if (A.Cell!=B.Cell)
		{
			const uint16* Boundaries=SmoothPairs.Find(CellPair(A.Cell,B.Cell));
			const int32 EA=AuthoredBoundary(A), EB=AuthoredBoundary(B);
			if (!Boundaries || EA==INDEX_NONE || EB==INDEX_NONE) continue;
			const int32 Bit=A.Cell<B.Cell ? 4*EA+EB : 4*EB+EA;
			if (!(*Boundaries & (1u<<Bit))) continue;
		}
		const FVector3d A0=Vertex(A,0), A1=Vertex(A,1), A2=Vertex(A,2);
		const FVector3d B0=Vertex(B,0), B1=Vertex(B,1), B2=Vertex(B,2);
		// Consistent winding and a real common finite edge, not nearby surfaces.
		if (!A0.Equals(B1,1.0e-6) || !A1.Equals(B0,1.0e-6)) continue;
		const FVector3d NA=FVector3d::CrossProduct(A1-A0,A2-A0).GetSafeNormal();
		const FVector3d NB=FVector3d::CrossProduct(B1-B0,B2-B0).GetSafeNormal();
		if (FVector3d::DotProduct(NA,NB)<=0) continue;
		const double DA=FVector3d::DotProduct(B2-A0,NA), DB=FVector3d::DotProduct(A2-B0,NB);
		auto& CA=Patch.Cells[A.Cell].ApproximationCells[A.Approximation];
		auto& CB=Patch.Cells[B.Cell].ApproximationCells[B.Approximation];
		if (DA>=0 && DB>=0)
		{
			CA.PositiveConcaveEdges[A.Triangle] |= 1u<<A.Edge;
			CB.PositiveConcaveEdges[B.Triangle] |= 1u<<B.Edge;
		}
		if (DA<=0 && DB<=0)
		{
			CA.NegativeConcaveEdges[A.Triangle] |= 1u<<A.Edge;
			CB.NegativeConcaveEdges[B.Triangle] |= 1u<<B.Edge;
		}
	}
	// A circular cone containing all incident facet normals also contains their
	// positive combinations. Rejecting directions OUTSIDE this measured cone
	// therefore preserves every possible convex normal cone; no angular cap is
	// guessed. A complete C2 four-cell ring is required, including both incident
	// boundaries of every corner. Open borders/holes/T-junctions stay unmodified.
	TMap<FEdgeKey,TArray<FCornerRef,TInlineAllocator<4>>> CornerBuckets;
	for (int32 C=0; C<Patch.Cells.Num(); ++C)
	for (int32 V=0; V<4; ++V)
	{
		auto& Cell=Patch.Cells[C];
		Cell.InternalCornerNormalCone[V]=INDEX_NONE;
		const FVector3d Point=Cell.Surface.Evaluate(V>=2 ? 1 : 0,V%2);
		if (Point.GetAbsMax()>9.0e12) continue;
		CornerBuckets.FindOrAdd(EdgeKey(Point,Point)).Add({C,V});
	}
	// Iterate cells/corners, not hash buckets, so derived cone indices are stable.
	for (int32 C=0; C<Patch.Cells.Num(); ++C)
	for (int32 V=0; V<4; ++V)
	{
		if (Patch.Cells[C].InternalCornerNormalCone[V]!=INDEX_NONE) continue;
		const FVector3d Point=Patch.Cells[C].Surface.Evaluate(V>=2 ? 1 : 0,V%2);
		if (Point.GetAbsMax()>9.0e12) continue;
		const auto* Ring=CornerBuckets.Find(EdgeKey(Point,Point));
		if (!Ring || Ring->Num()!=4) continue;
		bool bClosed=true;
		for (const auto& A : *Ring)
		{
			TArray<int32,TInlineAllocator<2>> DistinctNeighbors;
			const int32 ABoundaries[2]={A.Corner>=2 ? 1 : 0,A.Corner%2 ? 3 : 2};
			for (const int32 EA : ABoundaries)
			{
				int32 NeighborCount=0;
				for (const auto& B : *Ring)
				{
					if (A.Cell==B.Cell) continue;
					const uint16* Certified=SmoothPairs.Find(CellPair(A.Cell,B.Cell));
					if (!Certified) continue;
					const int32 BBoundaries[2]={B.Corner>=2 ? 1 : 0,B.Corner%2 ? 3 : 2};
					for (const int32 EB : BBoundaries)
					{
						const int32 Bit=A.Cell<B.Cell ? 4*EA+EB : 4*EB+EA;
						if (*Certified & (1u<<Bit))
						{ ++NeighborCount; DistinctNeighbors.AddUnique(B.Cell); }
					}
				}
				bClosed &= NeighborCount==1;
			}
			bClosed &= DistinctNeighbors.Num()==2;
		}
		if (!bClosed) continue;
		TArray<FVector3d,TInlineAllocator<8>> Normals;
		for (const auto& Ref : *Ring)
		{
			const auto& Cell=Patch.Cells[Ref.Cell];
			const FVector3d Corner=Cell.Surface.Evaluate(Ref.Corner>=2 ? 1 : 0,Ref.Corner%2);
			if (!Corner.Equals(Point,1.0e-6)) { bClosed=false; break; }
			const double U=Ref.Corner>=2 ? 1 : 0, W=Ref.Corner%2;
			for (const auto& Approx : Cell.ApproximationCells)
			{
				if ((U==0 ? Approx.MinimumU!=0 : Approx.MaximumU!=1) ||
					(W==0 ? Approx.MinimumV!=0 : Approx.MaximumV!=1)) continue;
				for (int32 T=0; T<2; ++T)
				{
					bool bIncident=false;
					for (int32 K=0; K<3; ++K) bIncident |= TensorTopologyCornerIndices[T][K]==Ref.Corner;
					if (!bIncident) continue;
					Normals.Add(FVector3d::CrossProduct(
						Approx.Corners[TensorTopologyCornerIndices[T][1]]-Approx.Corners[TensorTopologyCornerIndices[T][0]],
						Approx.Corners[TensorTopologyCornerIndices[T][2]]-Approx.Corners[TensorTopologyCornerIndices[T][0]]).GetSafeNormal());
				}
			}
		}
		if (!bClosed || Normals.IsEmpty()) continue;
		FVector3d Axis=FVector3d::ZeroVector;
		for (const auto& N : Normals) Axis+=N;
		Axis.Normalize();
		double MinimumDot=1;
		for (const auto& N : Normals) MinimumDot=FMath::Min(MinimumDot,FVector3d::DotProduct(Axis,N));
		if (MinimumDot<=0) continue;
		const int32 Index=Patch.InternalCornerNormalCones.Add(FVector4d(Axis.X,Axis.Y,Axis.Z,MinimumDot));
		for (const auto& Ref : *Ring) Patch.Cells[Ref.Cell].InternalCornerNormalCone[Ref.Corner]=Index;
	}
}
}
