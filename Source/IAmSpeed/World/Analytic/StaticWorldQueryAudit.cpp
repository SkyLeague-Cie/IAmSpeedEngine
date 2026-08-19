#include "StaticWorldQueryAudit.h"

#include "AnalyticWorldData.h"
#include "AnalyticWorldQuery.h"
#include "Components/PrimitiveComponent.h"
#include "GameFramework/Actor.h"
#include "HAL/IConsoleManager.h"

namespace Speed::Analytic
{
namespace
{
	TAutoConsoleVariable<int32> CVarAuditEnabled(
		TEXT("p.IAmSpeed.StaticWorldQuery.Audit"), 0,
		TEXT("Logs a deterministic hash of every static-world query reached by each canonical frame."));
	TAutoConsoleVariable<int32> CVarShadowEnabled(
		TEXT("p.IAmSpeed.AnalyticWorld.Shadow"), 0,
		TEXT("Runs the immutable analytical world beside Unreal static queries; never changes authority."));
	TAutoConsoleVariable<int32> CVarMeshShadowEnabled(
		TEXT("p.IAmSpeed.AnalyticWorld.MeshShadow"), 0,
		TEXT("Includes baked triangle faces in analytical shadow queries; never changes authority."));
	TAutoConsoleVariable<float> CVarPointToleranceCm(
		TEXT("p.IAmSpeed.AnalyticWorld.Shadow.PointToleranceCm"), 0.1f,
		TEXT("Maximum point/location difference reported equal in analytical shadow mode."));
	TAutoConsoleVariable<float> CVarNormalTolerance(
		TEXT("p.IAmSpeed.AnalyticWorld.Shadow.NormalTolerance"), 1.0e-4f,
		TEXT("Maximum normal vector distance reported equal in analytical shadow mode."));
	TAutoConsoleVariable<float> CVarTimeTolerance(
		TEXT("p.IAmSpeed.AnalyticWorld.Shadow.TimeTolerance"), 1.0e-6f,
		TEXT("Maximum normalized sweep-time difference reported equal in analytical shadow mode."));

	constexpr uint64 AuditFnvOffset = 14695981039346656037ull;
	constexpr uint64 AuditFnvPrime = 1099511628211ull;

	struct FPreviousTransition
	{
		bool bLegacyHit = false;
		bool bAnalyticHit = false;
		uint64 LegacyFeature = 0;
		uint64 AnalyticFeature = 0;
	};

	struct FFrameState
	{
		bool bActive = false;
		bool bFirstDivergenceReported = false;
		uint64 Frame = 0;
		uint32 QueryCount = 0;
		uint64 LegacyHash = AuditFnvOffset;
		uint64 AnalyticHash = AuditFnvOffset;
		const FAnalyticWorldData* World = nullptr;
		TArray<FPreviousTransition> PreviousTransitions;
		uint32 MatchCount = 0;
		uint32 MismatchCount = 0;
		uint32 FieldMismatches[7] = {};
	};

	thread_local FFrameState State;

	void AuditHashBytes(uint64& Hash, const void* Data, SIZE_T Size)
	{
		const uint8* Bytes = static_cast<const uint8*>(Data);
		for (SIZE_T Index = 0; Index < Size; ++Index)
		{
			Hash ^= Bytes[Index];
			Hash *= AuditFnvPrime;
		}
	}

	template <typename T>
	void AuditHashValue(uint64& Hash, const T& Value)
	{
		AuditHashBytes(Hash, &Value, sizeof(T));
	}

	void HashVector(uint64& Hash, const FVector& Value)
	{
		AuditHashValue(Hash, Value.X);
		AuditHashValue(Hash, Value.Y);
		AuditHashValue(Hash, Value.Z);
	}

	const TCHAR* SiteName(const EStaticQuerySite Site)
	{
		switch (Site)
		{
		case EStaticQuerySite::SolidSweep: return TEXT("SolidSweep");
		case EStaticQuerySite::SensorSweep: return TEXT("SensorSweep");
		case EStaticQuerySite::SubBodySweep: return TEXT("SubBodySweep");
		case EStaticQuerySite::WheelSuspensionProbe: return TEXT("WheelSuspensionProbe");
		case EStaticQuerySite::WheelEstablishedSupportProbe: return TEXT("WheelEstablishedSupportProbe");
		case EStaticQuerySite::RayWheelSuspensionProbe: return TEXT("RayWheelSuspensionProbe");
		case EStaticQuerySite::RayWheelEstablishedSupportProbe: return TEXT("RayWheelEstablishedSupportProbe");
		case EStaticQuerySite::BoxPenetrationProjection: return TEXT("BoxPenetrationProjection");
		case EStaticQuerySite::SpherePenetrationProjection: return TEXT("SpherePenetrationProjection");
		case EStaticQuerySite::SpherePenetrationResidual: return TEXT("SpherePenetrationResidual");
		default: return TEXT("Unknown");
		}
	}

	uint64 LegacySurfaceId(const FHitResult& Hit)
	{
		const UPrimitiveComponent* Component = Hit.Component.Get();
		const AActor* Owner = Component ? Component->GetOwner() : nullptr;
		return Owner ? StableStringId(Owner->GetPathName()) : 0;
	}

	uint64 LegacyFeatureId(const FHitResult& Hit)
	{
		const uint64 SurfaceId = LegacySurfaceId(Hit);
		return SurfaceId == 0 ? 0 : CombineStableIds(
			SurfaceId, static_cast<uint64>(EFeatureKind::Interior) + 1ull);
	}

	FWorldQuery MakeQuery(
		const FVector& Start,
		const FVector& End,
		const FQuat& Rotation,
		const FCollisionShape& Shape,
		const uint8 TraceChannel,
		const uint64 ObjectTypes,
		const bool bObjectQuery)
	{
		FWorldQuery Query;
		Query.Start = FVector3d(Start);
		Query.End = FVector3d(End);
		Query.Rotation = FQuat4d(Rotation);
		Query.TraceChannel = TraceChannel;
		Query.ObjectTypes = ObjectTypes;
		Query.bObjectQuery = bObjectQuery;
		Query.bApplyCollisionFilter = true;
		Query.bIncludeTriangles =
			CVarMeshShadowEnabled.GetValueOnAnyThread() != 0;
		if (Shape.IsSphere())
		{
			Query.Shape = EQueryShape::Sphere;
			Query.Radius = Shape.GetSphereRadius();
		}
		else if (Shape.IsBox())
		{
			Query.Shape = EQueryShape::Box;
			Query.HalfExtent = FVector3d(Shape.GetExtent());
		}
		else
		{
			Query.Shape = EQueryShape::Ray;
		}
		return Query;
	}

	void ReportDivergence(
		const EStaticQuerySite Site, const uint32 QueryOrdinal, const TCHAR* Field)
	{
		if (State.bFirstDivergenceReported)
		{
			return;
		}
		State.bFirstDivergenceReported = true;
		UE_LOG(LogTemp, Warning,
			TEXT("[AnalyticShadowFirstDivergence] Frame=%llu Query=%u Site=%s Field=%s"),
			State.Frame, QueryOrdinal, SiteName(Site), Field);
	}

	void HashLegacyHit(uint64& Hash, const bool bHit, const FHitResult& Hit)
	{
		AuditHashValue(Hash, bHit);
		if (!bHit)
		{
			return;
		}
		AuditHashValue(Hash, Hit.Time);
		HashVector(Hash, Hit.Location);
		HashVector(Hash, Hit.ImpactPoint);
		HashVector(Hash, Hit.ImpactNormal);
		AuditHashValue(Hash, Hit.bStartPenetrating);
		AuditHashValue(Hash, Hit.PenetrationDepth);
		const uint64 SurfaceId = LegacySurfaceId(Hit);
		const uint64 FeatureId = LegacyFeatureId(Hit);
		AuditHashValue(Hash, SurfaceId);
		AuditHashValue(Hash, FeatureId);
		AuditHashValue(Hash, Hit.FaceIndex);
	}

	void HashAnalyticHit(uint64& Hash, const FWorldHit& Hit)
	{
		AuditHashValue(Hash, Hit.bHit);
		if (!Hit.bHit)
		{
			return;
		}
		AuditHashValue(Hash, Hit.Time);
		AuditHashValue(Hash, Hit.Location.X);
		AuditHashValue(Hash, Hit.Location.Y);
		AuditHashValue(Hash, Hit.Location.Z);
		AuditHashValue(Hash, Hit.Point.X);
		AuditHashValue(Hash, Hit.Point.Y);
		AuditHashValue(Hash, Hit.Point.Z);
		AuditHashValue(Hash, Hit.Normal.X);
		AuditHashValue(Hash, Hit.Normal.Y);
		AuditHashValue(Hash, Hit.Normal.Z);
		AuditHashValue(Hash, Hit.bStartPenetrating);
		AuditHashValue(Hash, Hit.PenetrationDepth);
		AuditHashValue(Hash, Hit.SurfaceId);
		AuditHashValue(Hash, Hit.FeatureId);
	}

	void Compare(
		const EStaticQuerySite Site,
		const uint32 QueryOrdinal,
		const bool bLegacyHit,
		const FHitResult& Legacy,
		const FWorldHit& Analytic)
	{
		if (State.PreviousTransitions.Num() <= static_cast<int32>(QueryOrdinal))
		{
			State.PreviousTransitions.SetNum(QueryOrdinal + 1);
		}
		FPreviousTransition& Previous = State.PreviousTransitions[QueryOrdinal];
		const uint64 LegacyFeature = bLegacyHit ? LegacyFeatureId(Legacy) : 0;
		const bool bLegacyTransition = Previous.bLegacyHit != bLegacyHit ||
			(Previous.bLegacyHit && Previous.LegacyFeature != LegacyFeature);
		const bool bAnalyticTransition = Previous.bAnalyticHit != Analytic.bHit ||
			(Previous.bAnalyticHit && Previous.AnalyticFeature != Analytic.FeatureId);
		Previous.bLegacyHit = bLegacyHit;
		Previous.bAnalyticHit = Analytic.bHit;
		Previous.LegacyFeature = LegacyFeature;
		Previous.AnalyticFeature = Analytic.FeatureId;

		const TCHAR* DivergenceField = nullptr;
		int32 DivergenceIndex = INDEX_NONE;
		if (bLegacyHit != Analytic.bHit)
		{
			DivergenceField = TEXT("Hit");
			DivergenceIndex = 0;
		}
		else if (bLegacyHit &&
			FMath::Abs(static_cast<double>(Legacy.Time) - Analytic.Time) >
			CVarTimeTolerance.GetValueOnAnyThread())
		{
			DivergenceField = TEXT("Time");
			DivergenceIndex = 1;
		}
		else if (bLegacyHit &&
			FVector3d::Distance(FVector3d(Legacy.Location), Analytic.Location) >
			CVarPointToleranceCm.GetValueOnAnyThread())
		{
			DivergenceField = TEXT("Location");
			DivergenceIndex = 2;
		}
		else if (bLegacyHit &&
			FVector3d::Distance(FVector3d(Legacy.ImpactPoint), Analytic.Point) >
			CVarPointToleranceCm.GetValueOnAnyThread())
		{
			DivergenceField = TEXT("Point");
			DivergenceIndex = 3;
		}
		else if (bLegacyHit &&
			FVector3d::Distance(FVector3d(Legacy.ImpactNormal), Analytic.Normal) >
			CVarNormalTolerance.GetValueOnAnyThread())
		{
			DivergenceField = TEXT("Normal");
			DivergenceIndex = 4;
		}
		else if (bLegacyHit &&
			(LegacySurfaceId(Legacy) != Analytic.SurfaceId ||
				LegacyFeatureId(Legacy) != Analytic.FeatureId))
		{
			DivergenceField = TEXT("Feature");
			DivergenceIndex = 5;
		}
		else if (bLegacyTransition != bAnalyticTransition)
		{
			DivergenceField = TEXT("Transition");
			DivergenceIndex = 6;
		}
		if (DivergenceField)
		{
			++State.MismatchCount;
			++State.FieldMismatches[DivergenceIndex];
			ReportDivergence(Site, QueryOrdinal, DivergenceField);
		}
		else
		{
			++State.MatchCount;
		}
	}

	void Record(
		const EStaticQuerySite Site,
		const FVector& Start,
		const FVector& End,
		const FQuat& Rotation,
		const FCollisionShape& Shape,
		const uint8 TraceChannel,
		const uint64 ObjectTypes,
		const bool bObjectQuery,
		const bool bHit,
		const FHitResult& UnrealHit)
	{
		if (!State.bActive)
		{
			return;
		}
		const uint32 QueryOrdinal = State.QueryCount++;
		AuditHashValue(State.LegacyHash, Site);
		HashVector(State.LegacyHash, Start);
		HashVector(State.LegacyHash, End);
		AuditHashValue(State.LegacyHash, Rotation.X);
		AuditHashValue(State.LegacyHash, Rotation.Y);
		AuditHashValue(State.LegacyHash, Rotation.Z);
		AuditHashValue(State.LegacyHash, Rotation.W);
		const FVector ShapeExtent = Shape.GetExtent();
		HashVector(State.LegacyHash, ShapeExtent);
		HashLegacyHit(State.LegacyHash, bHit, UnrealHit);

		if (!FStaticWorldQueryAudit::IsShadowEnabled() || !State.World)
		{
			return;
		}
		const FWorldQuery Query = MakeQuery(
			Start, End, Rotation, Shape, TraceChannel, ObjectTypes, bObjectQuery);
		const FWorldHit AnalyticHit = FWorldQueryService(*State.World).Sweep(Query);
		AuditHashValue(State.AnalyticHash, Site);
		HashAnalyticHit(State.AnalyticHash, AnalyticHit);
		Compare(Site, QueryOrdinal, bHit, UnrealHit, AnalyticHit);
	}
}

bool FStaticWorldQueryAudit::IsEnabled()
{
	return CVarAuditEnabled.GetValueOnAnyThread() != 0 || IsShadowEnabled();
}

bool FStaticWorldQueryAudit::IsShadowEnabled()
{
	return CVarShadowEnabled.GetValueOnAnyThread() != 0;
}

bool FStaticWorldQueryAudit::ShouldBuildAnalyticWorld()
{
	// ExecCmds used by headless tests may be applied after OnWorldBeginPlay.
	// Importing immutable source data is side-effect free; only query execution
	// and comparison remain gated by the default-off shadow CVar.
	return true;
}

void FStaticWorldQueryAudit::BeginFrame(
	const uint64 Frame, const FAnalyticWorldData* WorldData)
{
	State.bActive = IsEnabled();
	State.Frame = Frame;
	State.QueryCount = 0;
	State.LegacyHash = AuditFnvOffset;
	State.AnalyticHash = AuditFnvOffset;
	State.World = WorldData;
	State.MatchCount = 0;
	State.MismatchCount = 0;
	FMemory::Memzero(State.FieldMismatches);
}

void FStaticWorldQueryAudit::EndFrame()
{
	if (!State.bActive)
	{
		return;
	}
	UE_LOG(LogTemp, Display,
		TEXT("[StaticWorldQueryFrame] Frame=%llu Queries=%u LegacyHash=%016llX ShadowHash=%016llX Shadow=%d MeshShadow=%d Matches=%u Mismatches=%u Fields=%u/%u/%u/%u/%u/%u/%u"),
		State.Frame, State.QueryCount, State.LegacyHash, State.AnalyticHash,
		IsShadowEnabled() ? 1 : 0,
		CVarMeshShadowEnabled.GetValueOnAnyThread() != 0 ? 1 : 0,
		State.MatchCount, State.MismatchCount,
		State.FieldMismatches[0], State.FieldMismatches[1],
		State.FieldMismatches[2], State.FieldMismatches[3],
		State.FieldMismatches[4], State.FieldMismatches[5],
		State.FieldMismatches[6]);
	State.bActive = false;
	State.World = nullptr;
}

void FStaticWorldQueryAudit::RecordSingle(
	const EStaticQuerySite Site,
	const FVector& Start,
	const FVector& End,
	const FQuat& Rotation,
	const FCollisionShape& Shape,
	const uint8 TraceChannel,
	const bool bHit,
	const FHitResult& UnrealHit)
{
	Record(Site, Start, End, Rotation, Shape, TraceChannel, 0, false,
		bHit, UnrealHit);
}

void FStaticWorldQueryAudit::RecordMulti(
	const EStaticQuerySite Site,
	const FVector& Start,
	const FVector& End,
	const FQuat& Rotation,
	const FCollisionShape& Shape,
	const uint64 ObjectTypes,
	const TArray<FHitResult>& UnrealHits)
{
	TArray<FHitResult> OrderedHits = UnrealHits;
	OrderedHits.Sort([](const FHitResult& A, const FHitResult& B)
	{
		const uint64 SurfaceA = LegacySurfaceId(A);
		const uint64 SurfaceB = LegacySurfaceId(B);
		if (SurfaceA != SurfaceB)
		{
			return SurfaceA < SurfaceB;
		}
		if (A.FaceIndex != B.FaceIndex)
		{
			return A.FaceIndex < B.FaceIndex;
		}
		if (!FMath::IsNearlyEqual(A.Time, B.Time))
		{
			return A.Time < B.Time;
		}
		return A.PenetrationDepth > B.PenetrationDepth;
	});

	FHitResult Representative;
	bool bHit = false;
	for (const FHitResult& Candidate : OrderedHits)
	{
		if (!bHit || Candidate.Time < Representative.Time ||
			(FMath::IsNearlyEqual(Candidate.Time, Representative.Time) &&
				(Candidate.PenetrationDepth > Representative.PenetrationDepth ||
					(FMath::IsNearlyEqual(Candidate.PenetrationDepth,
						Representative.PenetrationDepth) &&
						LegacySurfaceId(Candidate) < LegacySurfaceId(Representative)))))
		{
			Representative = Candidate;
			bHit = true;
		}
	}
	Record(Site, Start, End, Rotation, Shape, 0, ObjectTypes, true,
		bHit, Representative);
	if (State.bActive)
	{
		const int32 HitCount = OrderedHits.Num();
		AuditHashValue(State.LegacyHash, HitCount);
		for (const FHitResult& Hit : OrderedHits)
		{
			HashLegacyHit(State.LegacyHash, true, Hit);
		}
	}
}

} // namespace Speed::Analytic
