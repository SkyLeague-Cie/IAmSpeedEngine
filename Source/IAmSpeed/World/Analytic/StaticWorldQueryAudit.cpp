#include "StaticWorldQueryAudit.h"

#include "AnalyticWorldData.h"
#include "AnalyticWorldQuery.h"
#include "Components/PrimitiveComponent.h"
#include "GameFramework/Actor.h"
#include "HAL/IConsoleManager.h"
#include "IAmSpeed/World/SpeedWorldSubsystem.h"
#include "IAmSpeed/World/StaticCollisionWorld.h"

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
	TAutoConsoleVariable<int32> CVarCompactShadowEnabled(
		TEXT("p.IAmSpeed.AnalyticWorld.CompactShadow"), 0,
		TEXT("Includes compact analytical patches in shadow queries; never changes authority."));
	TAutoConsoleVariable<int32> CVarCompactShadowDetailEnabled(
		TEXT("p.IAmSpeed.AnalyticWorld.CompactShadow.Detail"), 0,
		TEXT("Logs compact-patch shadow hits for bounded divergence classification."));
	TAutoConsoleVariable<int32> CVarCompactAuthorityEnabled(
		TEXT("p.IAmSpeed.AnalyticWorld.CompactAuthority"), 0,
		TEXT("Experimental: replaces audited Unreal static queries with compact patches."));
	TAutoConsoleVariable<int32> CVarAuthorityChaosShadowEnabled(
		TEXT("p.IAmSpeed.AnalyticWorld.AuthorityChaosShadow"), 0,
		TEXT("Diagnostic only: replays the first authoritative analytical Box hit through UWorld/Chaos without changing its result."));
	TAutoConsoleVariable<int32> CVarResidualSelectionAuditEnabled(
		TEXT("p.IAmSpeed.AnalyticWorld.ResidualSelectionAudit"), 0,
		TEXT("Diagnostic only: logs the first strict residual-triangle selection in each physical frame."));
	TAutoConsoleVariable<int32> CVarStrictQueryDetailEnabled(
		TEXT("p.IAmSpeed.AnalyticWorld.StrictQueryDetail"), 0,
		TEXT("Strict analytical query diagnostics.\n")
		TEXT("0: disabled (default)\n")
		TEXT("1: compact provider-set transitions plus the first miss in a frame\n")
		TEXT("2: every strict query; use only for very short captures"));
	TAutoConsoleVariable<int32> CVarStrictQueryDetailFrameStart(
		TEXT("p.IAmSpeed.AnalyticWorld.StrictQueryDetail.FrameStart"), -1,
		TEXT("Optional inclusive canonical-frame lower bound for StrictQueryDetail logs; negative disables the bound."));
	TAutoConsoleVariable<int32> CVarStrictQueryDetailFrameEnd(
		TEXT("p.IAmSpeed.AnalyticWorld.StrictQueryDetail.FrameEnd"), -1,
		TEXT("Optional inclusive canonical-frame upper bound for StrictQueryDetail logs; negative disables the bound."));
	TAutoConsoleVariable<int32> CVarStaticCollisionBackend(
		TEXT("p.IAmSpeed.StaticCollision.Backend"), 2,
		TEXT("Static collision backend: 0=UnrealLegacy, 1=AnalyticHybrid, 2=SurfaceAnalytic. SurfaceAnalytic never falls back to UWorld."));
	TAutoConsoleVariable<float> CVarPointToleranceCm(
		TEXT("p.IAmSpeed.AnalyticWorld.Shadow.PointToleranceCm"), 0.1f,
		TEXT("Maximum point/location difference reported equal in analytical shadow mode."));
	TAutoConsoleVariable<float> CVarNormalTolerance(
		TEXT("p.IAmSpeed.AnalyticWorld.Shadow.NormalTolerance"), 1.0e-4f,
		TEXT("Maximum normal vector distance reported equal in analytical shadow mode."));
	TAutoConsoleVariable<float> CVarTimeTolerance(
		TEXT("p.IAmSpeed.AnalyticWorld.Shadow.TimeTolerance"), 1.0e-6f,
		TEXT("Maximum normalized sweep-time difference reported equal in analytical shadow mode."));
	TAutoConsoleVariable<float> CVarPenetrationToleranceCm(
		TEXT("p.IAmSpeed.AnalyticWorld.Shadow.PenetrationToleranceCm"), 0.1f,
		TEXT("Maximum penetration-depth difference reported equal in analytical shadow mode."));

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
		bool bFirstHitDivergenceReported = false;
		uint64 Frame = 0;
		uint32 QueryCount = 0;
		uint32 LegacySweepCount = 0;
		uint32 AuthorityAttemptCount = 0;
		uint32 AuthorityCoveredCount = 0;
		uint32 AuthorityFallbackCount = 0;
		uint32 StrictMissingWorldCount = 0;
		uint32 ResidualSelectionDetailCount = 0;
		uint32 StrictHitCount = 0;
		uint32 StrictMissCount = 0;
		uint32 PreviousStrictMissCount = 0;
		uint32 StrictFirstMissAuthorityAttempt = 0;
		bool bStrictFirstMissCaptured = false;
		bool bHasPreviousStrictProviderHash = false;
		double AuthorityMilliseconds = 0.0;
		double MaximumAuthorityMilliseconds = 0.0;
		uint64 LegacyHash = AuditFnvOffset;
		uint64 AnalyticHash = AuditFnvOffset;
		uint64 PreviousStrictProviderHash = 0;
		FWorldQuery StrictFirstMissQuery;
		const FAnalyticWorldData* World = nullptr;
		const USpeedWorldSubsystem* RuntimeBridge = nullptr;
		TArray<FPreviousTransition> PreviousTransitions;
		TArray<uint64, TInlineAllocator<16>> StrictProviderIds;
		uint32 MatchCount = 0;
		uint32 MismatchCount = 0;
		uint32 FieldMismatches[9] = {};
	};

	thread_local FFrameState State;

	Speed::EStaticCollisionBackend SelectedBackend()
	{
		const int32 Requested = FMath::Clamp(
			CVarStaticCollisionBackend.GetValueOnAnyThread(), 0, 2);
		if (Requested == 0 &&
			CVarCompactAuthorityEnabled.GetValueOnAnyThread() != 0)
		{
			return Speed::EStaticCollisionBackend::AnalyticHybrid;
		}
		return static_cast<Speed::EStaticCollisionBackend>(Requested);
	}

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
		case EStaticQuerySite::BoxPersistentSupportProbe: return TEXT("BoxPersistentSupportProbe");
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
		const bool bObjectQuery,
		const FCollisionResponseParams* ResponseParams = nullptr)
	{
		FWorldQuery Query;
		Query.Start = FVector3d(Start);
		Query.End = FVector3d(End);
		Query.Rotation = FQuat4d(Rotation);
		Query.TraceChannel = TraceChannel;
		Query.ObjectTypes = ObjectTypes;
		if (!bObjectQuery && ResponseParams)
		{
			Query.BlockingObjectTypes = 0;
			for (uint8 ObjectType = 0; ObjectType < ECollisionChannel::ECC_MAX;
				++ObjectType)
			{
				if (ResponseParams->CollisionResponse.GetResponse(
					static_cast<ECollisionChannel>(ObjectType)) == ECR_Block)
				{
					Query.BlockingObjectTypes |= 1ull << ObjectType;
				}
			}
		}
		Query.bObjectQuery = bObjectQuery;
		Query.bApplyCollisionFilter = true;
		Query.bIncludeCompactPatches =
			CVarCompactShadowEnabled.GetValueOnAnyThread() != 0;
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

	FHitResult ToUnrealHit(
		const FWorldQuery& Query, const FWorldHit& Analytic,
		const USpeedWorldSubsystem* RuntimeBridge)
	{
		FHitResult Result;
		if (!Analytic.bHit) return Result;
		Result.bBlockingHit = true;
		Result.bStartPenetrating = Analytic.bStartPenetrating;
		Result.Time = static_cast<float>(Analytic.Time);
		Result.Distance = static_cast<float>(
			(Query.End - Query.Start).Length() * Analytic.Time);
		Result.Location = FVector(Analytic.Location);
		Result.ImpactPoint = FVector(Analytic.Point);
		Result.Normal = FVector(Analytic.Normal);
		Result.ImpactNormal = FVector(Analytic.Normal);
		Result.PenetrationDepth = static_cast<float>(Analytic.PenetrationDepth);
		Result.TraceStart = FVector(Query.Start);
		Result.TraceEnd = FVector(Query.End);
		Result.FaceIndex = static_cast<int32>(
			Analytic.PrimitiveId & 0x7fffffffull);
		if (RuntimeBridge && Analytic.SourceId != 0)
		{
			Result.Component = RuntimeBridge->FindAnalyticSourceComponent(
				Analytic.SourceId);
		}
		return Result;
	}

	void AuditResidualSelection(
		const FWorldQuery& Query,
		const FWorldHit& AuthorityHit,
		const Speed::IStaticCollisionWorld& StaticWorld)
	{
		if (CVarResidualSelectionAuditEnabled.GetValueOnAnyThread() == 0 ||
			State.ResidualSelectionDetailCount != 0 || !AuthorityHit.bHit ||
			!Query.bIncludeTriangles)
		{
			return;
		}

		FWorldQuery PolishedQuery = Query;
		PolishedQuery.bIncludeTriangles = false;
		const FWorldHit PolishedHit = StaticWorld.SweepSingle(PolishedQuery);
		const bool bSameProvider = PolishedHit.bHit &&
			PolishedHit.SourceId == AuthorityHit.SourceId &&
			PolishedHit.SurfaceId == AuthorityHit.SurfaceId &&
			PolishedHit.PrimitiveId == AuthorityHit.PrimitiveId;
		if (bSameProvider)
		{
			return;
		}
		FWorldQuery NormalRayQuery = PolishedQuery;
		NormalRayQuery.Shape = EQueryShape::Ray;
		NormalRayQuery.Radius = 0.0;
		NormalRayQuery.HalfExtent = FVector3d::ZeroVector;
		NormalRayQuery.Rotation = FQuat4d::Identity;
		NormalRayQuery.Start = AuthorityHit.Point + 100.0 * AuthorityHit.Normal;
		NormalRayQuery.End = AuthorityHit.Point - 100.0 * AuthorityHit.Normal;
		const FWorldHit NormalRayHit = StaticWorld.SweepSingle(NormalRayQuery);
		if (State.World)
		{
			FBox3d QueryBounds(EForceInit::ForceInit);
			QueryBounds += Query.Start;
			QueryBounds += Query.End;
			if (Query.Shape == EQueryShape::Sphere)
			{
				QueryBounds = QueryBounds.ExpandBy(Query.Radius);
			}
			else if (Query.Shape == EQueryShape::Box)
			{
				QueryBounds = QueryBounds.ExpandBy(Query.HalfExtent.Length());
			}
			static thread_local TSet<uint64> LoggedCandidateSurfaces;
			for (const FPiecewiseTensorBezierPatch& Patch :
				State.World->PiecewiseTensorBezierPatches)
			{
				if (!Patch.Bounds.Intersect(QueryBounds) ||
					LoggedCandidateSurfaces.Contains(Patch.SurfaceId))
				{
					continue;
				}
				LoggedCandidateSurfaces.Add(Patch.SurfaceId);
				const bool bCanBlock = Query.bObjectQuery
					? Patch.ObjectType < 64 &&
						(Query.ObjectTypes & (1ull << Patch.ObjectType)) != 0
					: Query.TraceChannel < 64 &&
						(Patch.BlockingChannels & (1ull << Query.TraceChannel)) != 0 &&
						Patch.ObjectType < 64 &&
						(Query.BlockingObjectTypes & (1ull << Patch.ObjectType)) != 0;
				UE_LOG(LogTemp, Display,
					TEXT("[AnalyticResidualCandidate] Frame=%llu Surface=%016llX Primitive=%016llX Group=%016llX BoundsMin=%s BoundsMax=%s Cells=%d ObjectType=%u BlockingChannels=%016llX QueryTraceChannel=%u QueryBlockingObjectTypes=%016llX QueryEnabled=%d Authority=%d CanBlock=%d"),
					State.Frame, Patch.SurfaceId, Patch.PrimitiveId,
					Patch.CanonicalGroupId, *FVector(Patch.Bounds.Min).ToString(),
					*FVector(Patch.Bounds.Max).ToString(), Patch.Cells.Num(),
					Patch.ObjectType, Patch.BlockingChannels, Query.TraceChannel,
					Query.BlockingObjectTypes, Patch.bQueryCollisionEnabled ? 1 : 0,
					Patch.bAuthorityEligible ? 1 : 0, bCanBlock ? 1 : 0);
			}
		}

		++State.ResidualSelectionDetailCount;
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticResidualSelection] Frame=%llu Shape=%u Start=%s End=%s Rotation=%s HalfExtent=%s Radius=%.17g RequiredSource=%016llX RequiredSurface=%016llX RequiredGroup=%016llX ResidualTime=%.17g ResidualPoint=%s ResidualNormal=%s ResidualSource=%016llX ResidualSurface=%016llX ResidualFeature=%016llX ResidualPrimitive=%016llX PolishedHit=%d PolishedTime=%.17g PolishedPoint=%s PolishedNormal=%s PolishedSource=%016llX PolishedSurface=%016llX PolishedFeature=%016llX PolishedPrimitive=%016llX NormalRayHit=%d NormalRayTime=%.17g NormalRayPoint=%s NormalRayNormal=%s NormalRaySource=%016llX NormalRaySurface=%016llX NormalRayFeature=%016llX NormalRayPrimitive=%016llX"),
			State.Frame, static_cast<uint8>(Query.Shape),
			*FVector(Query.Start).ToString(), *FVector(Query.End).ToString(),
			*FQuat(Query.Rotation).ToString(), *FVector(Query.HalfExtent).ToString(),
			Query.Radius, Query.RequiredSourceId, Query.RequiredSurfaceId,
			Query.RequiredCanonicalGroupId, AuthorityHit.Time,
			*FVector(AuthorityHit.Point).ToString(),
			*FVector(AuthorityHit.Normal).ToString(), AuthorityHit.SourceId,
			AuthorityHit.SurfaceId, AuthorityHit.FeatureId,
			AuthorityHit.PrimitiveId, PolishedHit.bHit ? 1 : 0,
			PolishedHit.Time, *FVector(PolishedHit.Point).ToString(),
			*FVector(PolishedHit.Normal).ToString(), PolishedHit.SourceId,
			PolishedHit.SurfaceId, PolishedHit.FeatureId,
			PolishedHit.PrimitiveId, NormalRayHit.bHit ? 1 : 0,
			NormalRayHit.Time, *FVector(NormalRayHit.Point).ToString(),
			*FVector(NormalRayHit.Normal).ToString(), NormalRayHit.SourceId,
			NormalRayHit.SurfaceId, NormalRayHit.FeatureId,
			NormalRayHit.PrimitiveId);
	}

	void LogStrictQueryDetail(
		const FWorldQuery& Query,
		const FWorldHit& Hit,
		const uint32 AuthorityAttempt,
		const Speed::IStaticCollisionWorld& StaticWorld)
	{
		const int32 DetailMode =
			CVarStrictQueryDetailEnabled.GetValueOnAnyThread();
		if (DetailMode <= 0)
		{
			return;
		}
		const int32 DetailFrameStart =
			CVarStrictQueryDetailFrameStart.GetValueOnAnyThread();
		const int32 DetailFrameEnd =
			CVarStrictQueryDetailFrameEnd.GetValueOnAnyThread();
		if ((DetailFrameStart >= 0 &&
				State.Frame < static_cast<uint64>(DetailFrameStart)) ||
			(DetailFrameEnd >= 0 &&
				State.Frame > static_cast<uint64>(DetailFrameEnd)))
		{
			return;
		}
		if (Hit.bHit)
		{
			++State.StrictHitCount;
			if (Hit.PrimitiveId != 0)
			{
				State.StrictProviderIds.AddUnique(Hit.PrimitiveId);
			}
		}
		else
		{
			++State.StrictMissCount;
		}
		if (DetailMode == 1)
		{
			if (Hit.bHit || State.bStrictFirstMissCaptured)
			{
				return;
			}
			State.bStrictFirstMissCaptured = true;
			State.StrictFirstMissAuthorityAttempt = AuthorityAttempt;
			State.StrictFirstMissQuery = Query;
			return;
		}
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticStrictQuery] Frame=%llu Attempt=%u Shape=%u Start=%s End=%s Rotation=%s HalfExtent=%s Radius=%.17g RequiredSource=%016llX RequiredSurface=%016llX RequiredGroup=%016llX Hit=%d StartPenetrating=%d PenetrationDepth=%.17g ErrorCm=%.17g Time=%.17g Point=%s Normal=%s Source=%016llX Surface=%016llX Feature=%016llX Primitive=%016llX CanonicalGroup=%016llX"),
			State.Frame, AuthorityAttempt, static_cast<uint8>(Query.Shape),
			*FVector(Query.Start).ToString(), *FVector(Query.End).ToString(),
			*FQuat(Query.Rotation).ToString(), *FVector(Query.HalfExtent).ToString(),
			Query.Radius, Query.RequiredSourceId, Query.RequiredSurfaceId,
			Query.RequiredCanonicalGroupId, Hit.bHit ? 1 : 0,
			Hit.bStartPenetrating ? 1 : 0, Hit.PenetrationDepth,
			Hit.GeometricErrorBoundCm, Hit.Time,
			*FVector(Hit.Point).ToString(), *FVector(Hit.Normal).ToString(),
			Hit.SourceId, Hit.SurfaceId, Hit.FeatureId, Hit.PrimitiveId,
			Hit.CanonicalGroupId);
		if (!Hit.bHit)
		{
			FWorldQuery ResidualQuery = Query;
			ResidualQuery.bIncludeTriangles = true;
			const FWorldHit ResidualHit = StaticWorld.SweepSingle(ResidualQuery);
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticStrictResidual] Frame=%llu Attempt=%u Hit=%d StartPenetrating=%d PenetrationDepth=%.17g Time=%.17g Point=%s Normal=%s Source=%016llX Surface=%016llX Feature=%016llX Primitive=%016llX CanonicalGroup=%016llX"),
				State.Frame, AuthorityAttempt, ResidualHit.bHit ? 1 : 0,
				ResidualHit.bStartPenetrating ? 1 : 0,
				ResidualHit.PenetrationDepth, ResidualHit.Time,
				*FVector(ResidualHit.Point).ToString(),
				*FVector(ResidualHit.Normal).ToString(), ResidualHit.SourceId,
				ResidualHit.SurfaceId, ResidualHit.FeatureId,
				ResidualHit.PrimitiveId, ResidualHit.CanonicalGroupId);
		}
	}

	void ReportDivergence(
		const EStaticQuerySite Site, const uint32 QueryOrdinal, const TCHAR* Field,
		const FWorldQuery& Query, const bool bLegacyHit,
		const FHitResult& Legacy, const FWorldHit& Analytic)
	{
		if (State.bFirstDivergenceReported)
		{
			return;
		}
		State.bFirstDivergenceReported = true;
		UE_LOG(LogTemp, Warning,
			TEXT("[AnalyticShadowFirstDivergence] Frame=%llu Query=%u Site=%s Field=%s"),
			State.Frame, QueryOrdinal, SiteName(Site), Field);
		const UPrimitiveComponent* LegacyComponent = Legacy.Component.Get();
		const AActor* LegacyOwner = LegacyComponent ? LegacyComponent->GetOwner() : nullptr;
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticShadowFirstDivergenceDetail] Frame=%llu Query=%u Site=%s Shape=%u Start=%s End=%s Rotation=%s HalfExtent=%s Radius=%.17g TraceChannel=%u ObjectTypes=%016llX BlockingObjectTypes=%016llX LegacyHit=%d LegacyTime=%.9g LegacyLocation=%s LegacyPoint=%s LegacyNormal=%s LegacyStartPenetrating=%d LegacyPenetrationDepth=%.9g LegacyOwner=%s LegacyComponent=%s LegacyFace=%d AnalyticHit=%d AnalyticTime=%.17g AnalyticLocation=%s AnalyticPoint=%s AnalyticNormal=%s AnalyticStartPenetrating=%d AnalyticPenetrationDepth=%.17g AnalyticSource=%016llX AnalyticSurface=%016llX AnalyticFeature=%016llX AnalyticPrimitive=%016llX"),
			State.Frame, QueryOrdinal, SiteName(Site), static_cast<uint8>(Query.Shape),
			*FVector(Query.Start).ToString(), *FVector(Query.End).ToString(),
			*FQuat(Query.Rotation).ToString(), *FVector(Query.HalfExtent).ToString(),
			Query.Radius, Query.TraceChannel, Query.ObjectTypes,
			Query.BlockingObjectTypes, bLegacyHit ? 1 : 0,
			bLegacyHit ? Legacy.Time : 1.0f,
			bLegacyHit ? *Legacy.Location.ToString() : TEXT("None"),
			bLegacyHit ? *Legacy.ImpactPoint.ToString() : TEXT("None"),
			bLegacyHit ? *Legacy.ImpactNormal.ToString() : TEXT("None"),
			bLegacyHit && Legacy.bStartPenetrating ? 1 : 0,
			bLegacyHit ? Legacy.PenetrationDepth : 0.0f,
			LegacyOwner ? *LegacyOwner->GetPathName() : TEXT("None"),
			LegacyComponent ? *LegacyComponent->GetPathName() : TEXT("None"),
			bLegacyHit ? Legacy.FaceIndex : INDEX_NONE, Analytic.bHit ? 1 : 0,
			Analytic.Time, *FVector(Analytic.Location).ToString(),
			*FVector(Analytic.Point).ToString(), *FVector(Analytic.Normal).ToString(),
			Analytic.bStartPenetrating ? 1 : 0, Analytic.PenetrationDepth,
			Analytic.SourceId, Analytic.SurfaceId, Analytic.FeatureId,
			Analytic.PrimitiveId);
	}

	void ReportHitDivergence(
		const EStaticQuerySite Site, const uint32 QueryOrdinal,
		const FWorldQuery& Query, const bool bLegacyHit,
		const FHitResult& Legacy, const FWorldHit& Analytic)
	{
		if (State.bFirstHitDivergenceReported) return;
		State.bFirstHitDivergenceReported = true;
		const UPrimitiveComponent* LegacyComponent = Legacy.Component.Get();
		UE_LOG(LogTemp, Display,
			TEXT("[AnalyticShadowFirstHitDivergence] Frame=%llu Query=%u Site=%s Shape=%u Start=%s End=%s Rotation=%s HalfExtent=%s Radius=%.17g LegacyHit=%d LegacyTime=%.9g LegacyPoint=%s LegacyNormal=%s LegacyStartPenetrating=%d LegacyPenetrationDepth=%.9g LegacyComponent=%s LegacyFace=%d AnalyticHit=%d AnalyticTime=%.17g AnalyticPoint=%s AnalyticNormal=%s AnalyticStartPenetrating=%d AnalyticPenetrationDepth=%.17g AnalyticSource=%016llX AnalyticSurface=%016llX AnalyticFeature=%016llX AnalyticPrimitive=%016llX"),
			State.Frame, QueryOrdinal, SiteName(Site),
			static_cast<uint8>(Query.Shape), *FVector(Query.Start).ToString(),
			*FVector(Query.End).ToString(), *FQuat(Query.Rotation).ToString(),
			*FVector(Query.HalfExtent).ToString(), Query.Radius,
			bLegacyHit ? 1 : 0, bLegacyHit ? Legacy.Time : 1.0f,
			bLegacyHit ? *Legacy.ImpactPoint.ToString() : TEXT("None"),
			bLegacyHit ? *Legacy.ImpactNormal.ToString() : TEXT("None"),
			bLegacyHit && Legacy.bStartPenetrating ? 1 : 0,
			bLegacyHit ? Legacy.PenetrationDepth : 0.0f,
			LegacyComponent ? *LegacyComponent->GetPathName() : TEXT("None"),
			bLegacyHit ? Legacy.FaceIndex : INDEX_NONE,
			Analytic.bHit ? 1 : 0, Analytic.Time,
			*FVector(Analytic.Point).ToString(), *FVector(Analytic.Normal).ToString(),
			Analytic.bStartPenetrating ? 1 : 0, Analytic.PenetrationDepth,
			Analytic.SourceId, Analytic.SurfaceId, Analytic.FeatureId,
			Analytic.PrimitiveId);
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
		AuditHashValue(Hash, Hit.QueryPoint.X);
		AuditHashValue(Hash, Hit.QueryPoint.Y);
		AuditHashValue(Hash, Hit.QueryPoint.Z);
		AuditHashValue(Hash, Hit.Normal.X);
		AuditHashValue(Hash, Hit.Normal.Y);
		AuditHashValue(Hash, Hit.Normal.Z);
		AuditHashValue(Hash, Hit.bStartPenetrating);
		AuditHashValue(Hash, Hit.PenetrationDepth);
		AuditHashValue(Hash, Hit.bSurfaceNormalMayVary);
		AuditHashValue(Hash, Hit.QueryFeatureKind);
		AuditHashValue(Hash, Hit.SurfaceFeatureKind);
		AuditHashValue(Hash, Hit.QueryFeatureIndex);
		AuditHashValue(Hash, Hit.SurfaceFeatureIndex);
		AuditHashValue(Hash, Hit.SurfaceId);
		AuditHashValue(Hash, Hit.FeatureId);
	}

	void Compare(
		const EStaticQuerySite Site,
		const uint32 QueryOrdinal,
		const FWorldQuery& Query,
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
			ReportHitDivergence(
				Site, QueryOrdinal, Query, bLegacyHit, Legacy, Analytic);
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
		else if (bLegacyHit && Legacy.bStartPenetrating !=
			Analytic.bStartPenetrating)
		{
			DivergenceField = TEXT("StartPenetrating");
			DivergenceIndex = 7;
		}
		else if (bLegacyHit && FMath::Abs(
			static_cast<double>(Legacy.PenetrationDepth) -
				Analytic.PenetrationDepth) >
			CVarPenetrationToleranceCm.GetValueOnAnyThread())
		{
			DivergenceField = TEXT("PenetrationDepth");
			DivergenceIndex = 8;
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
			ReportDivergence(
				Site, QueryOrdinal, DivergenceField, Query, bLegacyHit, Legacy, Analytic);
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
		const FCollisionResponseParams* ResponseParams,
		const bool bHit,
		const FHitResult& UnrealHit,
		const uint64 RequiredSourceId = 0,
		const uint64 RequiredSurfaceId = 0,
		const uint64 RequiredCanonicalGroupId = 0)
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
		FWorldQuery Query = MakeQuery(
			Start, End, Rotation, Shape, TraceChannel, ObjectTypes, bObjectQuery,
			ResponseParams);
		Query.RequiredSourceId = RequiredSourceId;
		Query.RequiredSurfaceId = RequiredSurfaceId;
		Query.RequiredCanonicalGroupId = RequiredCanonicalGroupId;
		const FWorldHit AnalyticHit = FWorldQueryService(*State.World).Sweep(Query);
		AuditHashValue(State.AnalyticHash, Site);
		HashAnalyticHit(State.AnalyticHash, AnalyticHit);
		Compare(Site, QueryOrdinal, Query, bHit, UnrealHit, AnalyticHit);
		if (CVarCompactShadowDetailEnabled.GetValueOnAnyThread() != 0 &&
			CVarCompactShadowEnabled.GetValueOnAnyThread() != 0 &&
			Query.Shape == EQueryShape::Box)
		{
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticCompactShadowBoxQuery] Frame=%llu Query=%u Site=%s Start=%s End=%s Rotation=%s HalfExtent=%s LegacyHit=%d LegacyTime=%.9g LegacyPoint=%s LegacyNormal=%s LegacyStartPenetrating=%d LegacyPenetrationDepth=%.9g AnalyticHit=%d AnalyticTime=%.17g AnalyticPoint=%s AnalyticNormal=%s AnalyticStartPenetrating=%d AnalyticPenetrationDepth=%.17g Source=%016llX Surface=%016llX Feature=%016llX Primitive=%016llX CanonicalGroup=%016llX"),
				State.Frame, QueryOrdinal, SiteName(Site),
				*FVector(Query.Start).ToString(), *FVector(Query.End).ToString(),
				*FQuat(Query.Rotation).ToString(),
				*FVector(Query.HalfExtent).ToString(), bHit ? 1 : 0,
				bHit ? UnrealHit.Time : 1.0f,
				bHit ? *UnrealHit.ImpactPoint.ToString() : TEXT("None"),
				bHit ? *UnrealHit.ImpactNormal.ToString() : TEXT("None"),
				bHit && UnrealHit.bStartPenetrating ? 1 : 0,
				bHit ? UnrealHit.PenetrationDepth : 0.0f,
				AnalyticHit.bHit ? 1 : 0, AnalyticHit.Time,
				*FVector(AnalyticHit.Point).ToString(),
				*FVector(AnalyticHit.Normal).ToString(),
				AnalyticHit.bStartPenetrating ? 1 : 0,
				AnalyticHit.PenetrationDepth, AnalyticHit.SourceId,
				AnalyticHit.SurfaceId, AnalyticHit.FeatureId,
				AnalyticHit.PrimitiveId, AnalyticHit.CanonicalGroupId);
		}
		if (CVarCompactShadowDetailEnabled.GetValueOnAnyThread() != 0 &&
			CVarCompactShadowEnabled.GetValueOnAnyThread() != 0 &&
			AnalyticHit.bHit && AnalyticHit.PrimitiveId != 0)
		{
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticCompactShadowHit] Frame=%llu Query=%u Site=%s Shape=%u LegacyHit=%d LegacyTime=%.9g LegacyPoint=%s LegacyNormal=%s LegacyStartPenetrating=%d LegacyPenetrationDepth=%.9g AnalyticTime=%.17g AnalyticPoint=%s AnalyticNormal=%s AnalyticStartPenetrating=%d AnalyticPenetrationDepth=%.17g Surface=%016llX Feature=%016llX Primitive=%016llX CanonicalGroup=%016llX"),
				State.Frame, QueryOrdinal, SiteName(Site),
				static_cast<uint8>(Query.Shape), bHit ? 1 : 0,
				bHit ? UnrealHit.Time : 1.0f,
				bHit ? *UnrealHit.ImpactPoint.ToString() : TEXT("None"),
				bHit ? *UnrealHit.ImpactNormal.ToString() : TEXT("None"),
				bHit && UnrealHit.bStartPenetrating ? 1 : 0,
				bHit ? UnrealHit.PenetrationDepth : 0.0f,
				AnalyticHit.Time, *FVector(AnalyticHit.Point).ToString(),
				*FVector(AnalyticHit.Normal).ToString(),
				AnalyticHit.bStartPenetrating ? 1 : 0,
				AnalyticHit.PenetrationDepth, AnalyticHit.SurfaceId,
				AnalyticHit.FeatureId, AnalyticHit.PrimitiveId,
				AnalyticHit.CanonicalGroupId);
		}
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

bool FStaticWorldQueryAudit::IsCompactAuthorityEnabled()
{
	return SelectedBackend() != Speed::EStaticCollisionBackend::UnrealLegacy;
}

bool FStaticWorldQueryAudit::IsSurfaceAnalyticBackend()
{
	return SelectedBackend() == Speed::EStaticCollisionBackend::SurfaceAnalytic;
}

bool FStaticWorldQueryAudit::IsAuthorityChaosShadowEnabled()
{
	return CVarAuthorityChaosShadowEnabled.GetValueOnAnyThread() != 0;
}

bool FStaticWorldQueryAudit::TryCompactAuthoritySingle(
	UWorld* World,
	const FVector& Start, const FVector& End, const FQuat& Rotation,
	const FCollisionShape& Shape, const uint8 TraceChannel,
	const FCollisionResponseParams& ResponseParams,
	FHitResult& OutHit, bool& bOutHit, FWorldHit* OutAnalyticHit,
	const uint64 RequiredSourceId, const uint64 RequiredSurfaceId,
	const uint64 RequiredCanonicalGroupId,
	const FVector& ReferenceNormal,
	const float MinimumReferenceNormalDot,
	const bool bAllowEstablishedFaceContactAtBoundary)
{
	const double AuthorityStartSeconds = FPlatformTime::Seconds();
	ON_SCOPE_EXIT
	{
		const double ElapsedMilliseconds =
			(FPlatformTime::Seconds() - AuthorityStartSeconds) * 1000.0;
		State.AuthorityMilliseconds += ElapsedMilliseconds;
		State.MaximumAuthorityMilliseconds = FMath::Max(
			State.MaximumAuthorityMilliseconds, ElapsedMilliseconds);
	};
	if (OutAnalyticHit) *OutAnalyticHit = FWorldHit();
	if (!IsCompactAuthorityEnabled())
	{
		return false;
	}
	++State.AuthorityAttemptCount;
	const bool bStrict =
		SelectedBackend() == Speed::EStaticCollisionBackend::SurfaceAnalytic;
	const USpeedWorldSubsystem* RuntimeBridge = World
		? World->GetSubsystem<USpeedWorldSubsystem>() : nullptr;
	const Speed::IStaticCollisionWorld* StaticWorld = RuntimeBridge
		? RuntimeBridge->GetStaticCollisionWorld() : nullptr;
	if (!StaticWorld)
	{
		if (!bStrict)
		{
			++State.AuthorityFallbackCount;
			return false;
		}
		++State.StrictMissingWorldCount;
		OutHit = FHitResult();
		bOutHit = false;
		return true;
	}
	FWorldQuery Query = MakeQuery(
		Start, End, Rotation, Shape, TraceChannel, 0, false, &ResponseParams);
	Query.RequiredSourceId = RequiredSourceId;
	Query.RequiredSurfaceId = RequiredSurfaceId;
	Query.RequiredCanonicalGroupId = RequiredCanonicalGroupId;
	Query.ReferenceNormal = FVector3d(ReferenceNormal);
	Query.MinimumReferenceNormalDot = MinimumReferenceNormalDot;
	Query.bAllowEstablishedFaceContactAtBoundary =
		bAllowEstablishedFaceContactAtBoundary;
	Query.bAuthorityOnly = true;
	Query.bIncludeCompactPatches = true;
	// Hybrid retains the certified indexed-face safety net.  Strict authority
	// consumes polished providers only and therefore cannot silently select the
	// residual representation.
	Query.bIncludeTriangles = !bStrict;
	FWorldHit Analytic;
	if (!bStrict)
	{
		if (!StaticWorld->TrySweepAuthority(Query, Analytic))
		{
			++State.AuthorityFallbackCount;
			return false;
		}
	}
	else
	{
		Analytic = StaticWorld->SweepSingle(Query);
		LogStrictQueryDetail(Query, Analytic, State.AuthorityAttemptCount,
			*StaticWorld);
	}
	if (bStrict)
	{
		AuditResidualSelection(Query, Analytic, *StaticWorld);
	}
	++State.AuthorityCoveredCount;
	if (OutAnalyticHit) *OutAnalyticHit = Analytic;
	OutHit = ToUnrealHit(Query, Analytic, RuntimeBridge);
	bOutHit = Analytic.bHit;
	return true;
}

bool FStaticWorldQueryAudit::TryCompactAuthorityMulti(
	UWorld* World,
	const FVector& Start, const FVector& End, const FQuat& Rotation,
	const FCollisionShape& Shape, const uint64 ObjectTypes,
	TArray<FHitResult>& OutHits,
	FWorldHit* OutAnalyticHit)
{
	if (OutAnalyticHit)
	{
		*OutAnalyticHit = FWorldHit();
	}
	const double AuthorityStartSeconds = FPlatformTime::Seconds();
	ON_SCOPE_EXIT
	{
		const double ElapsedMilliseconds =
			(FPlatformTime::Seconds() - AuthorityStartSeconds) * 1000.0;
		State.AuthorityMilliseconds += ElapsedMilliseconds;
		State.MaximumAuthorityMilliseconds = FMath::Max(
			State.MaximumAuthorityMilliseconds, ElapsedMilliseconds);
	};
	if (!IsCompactAuthorityEnabled())
	{
		return false;
	}
	++State.AuthorityAttemptCount;
	const bool bStrict =
		SelectedBackend() == Speed::EStaticCollisionBackend::SurfaceAnalytic;
	const USpeedWorldSubsystem* RuntimeBridge = World
		? World->GetSubsystem<USpeedWorldSubsystem>() : nullptr;
	const Speed::IStaticCollisionWorld* StaticWorld = RuntimeBridge
		? RuntimeBridge->GetStaticCollisionWorld() : nullptr;
	if (!StaticWorld)
	{
		if (!bStrict)
		{
			++State.AuthorityFallbackCount;
			return false;
		}
		++State.StrictMissingWorldCount;
		OutHits.Reset();
		return true;
	}
	FWorldQuery Query = MakeQuery(
		Start, End, Rotation, Shape, 0, ObjectTypes, true);
	Query.bAuthorityOnly = true;
	Query.bIncludeCompactPatches = true;
	Query.bIncludeTriangles = !bStrict;
	FWorldHit Analytic;
	if (!bStrict)
	{
		if (!StaticWorld->TrySweepAuthority(Query, Analytic))
		{
			++State.AuthorityFallbackCount;
			return false;
		}
	}
	else
	{
		Analytic = StaticWorld->SweepSingle(Query);
		LogStrictQueryDetail(Query, Analytic, State.AuthorityAttemptCount,
			*StaticWorld);
	}
	if (bStrict)
	{
		AuditResidualSelection(Query, Analytic, *StaticWorld);
	}
	if (OutAnalyticHit)
	{
		*OutAnalyticHit = Analytic;
	}
	++State.AuthorityCoveredCount;
	OutHits.Reset();
	if (Analytic.bHit)
	{
		OutHits.Add(ToUnrealHit(Query, Analytic, RuntimeBridge));
	}
	return true;
}

bool FStaticWorldQueryAudit::ShouldBuildAnalyticWorld()
{
	// ExecCmds used by headless tests may be applied after OnWorldBeginPlay.
	// Importing immutable source data is side-effect free; only query execution
	// and comparison remain gated by the default-off shadow CVar.
	return true;
}

void FStaticWorldQueryAudit::BeginFrame(
	const uint64 Frame, const FAnalyticWorldData* WorldData,
	const USpeedWorldSubsystem* RuntimeBridge)
{
	State.bActive = IsEnabled() || IsCompactAuthorityEnabled();
	State.Frame = Frame;
	State.QueryCount = 0;
	State.LegacySweepCount = 0;
	State.AuthorityAttemptCount = 0;
	State.AuthorityCoveredCount = 0;
	State.AuthorityFallbackCount = 0;
	State.StrictMissingWorldCount = 0;
	State.ResidualSelectionDetailCount = 0;
	State.StrictHitCount = 0;
	State.StrictMissCount = 0;
	State.StrictFirstMissAuthorityAttempt = 0;
	State.bStrictFirstMissCaptured = false;
	State.StrictFirstMissQuery = FWorldQuery();
	State.StrictProviderIds.Reset();
	State.AuthorityMilliseconds = 0.0;
	State.MaximumAuthorityMilliseconds = 0.0;
	State.LegacyHash = AuditFnvOffset;
	State.AnalyticHash = AuditFnvOffset;
	State.World = WorldData;
	State.RuntimeBridge = RuntimeBridge;
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
	if (IsEnabled())
	{
		UE_LOG(LogTemp, Display,
			TEXT("[StaticWorldQueryFrame] Frame=%llu Queries=%u LegacySweeps=%u AuthorityAttempts=%u AuthorityCovered=%u AuthorityFallback=%u StrictMissingWorld=%u Backend=%u LegacyHash=%016llX ShadowHash=%016llX Shadow=%d MeshShadow=%d Matches=%u Mismatches=%u Fields=%u/%u/%u/%u/%u/%u/%u/%u/%u"),
			State.Frame, State.QueryCount, State.LegacySweepCount,
			State.AuthorityAttemptCount,
			State.AuthorityCoveredCount, State.AuthorityFallbackCount,
			State.StrictMissingWorldCount, static_cast<uint8>(SelectedBackend()),
			State.LegacyHash, State.AnalyticHash,
			IsShadowEnabled() ? 1 : 0,
			CVarMeshShadowEnabled.GetValueOnAnyThread() != 0 ? 1 : 0,
			State.MatchCount, State.MismatchCount,
			State.FieldMismatches[0], State.FieldMismatches[1],
			State.FieldMismatches[2], State.FieldMismatches[3],
			State.FieldMismatches[4], State.FieldMismatches[5],
			State.FieldMismatches[6], State.FieldMismatches[7],
			State.FieldMismatches[8]);
	}
	if (CVarStrictQueryDetailEnabled.GetValueOnAnyThread() == 1 &&
		State.AuthorityAttemptCount > 0)
	{
		State.StrictProviderIds.Sort();
		uint64 ProviderHash = AuditFnvOffset;
		FString ProviderList;
		for (const uint64 PrimitiveId : State.StrictProviderIds)
		{
			AuditHashValue(ProviderHash, PrimitiveId);
			if (!ProviderList.IsEmpty())
			{
				ProviderList += TEXT(",");
			}
			ProviderList += FString::Printf(TEXT("%016llX"), PrimitiveId);
		}
		const bool bProviderSetChanged =
			!State.bHasPreviousStrictProviderHash ||
			ProviderHash != State.PreviousStrictProviderHash;
		const bool bMissCountChanged =
			!State.bHasPreviousStrictProviderHash ||
			State.StrictMissCount != State.PreviousStrictMissCount;
		if (bProviderSetChanged || bMissCountChanged)
		{
			UE_LOG(LogTemp, Display,
				TEXT("[AnalyticStrictQueryFrame] Frame=%llu Attempts=%u Hits=%u Misses=%u ProviderCount=%d ProviderHash=%016llX Providers=%s ProviderChanged=%d MissCountChanged=%d"),
				State.Frame, State.AuthorityAttemptCount, State.StrictHitCount,
				State.StrictMissCount, State.StrictProviderIds.Num(), ProviderHash,
				*ProviderList, bProviderSetChanged ? 1 : 0,
				bMissCountChanged ? 1 : 0);
			if (State.StrictMissCount > 0 &&
				State.bStrictFirstMissCaptured)
			{
				const FWorldQuery& Query = State.StrictFirstMissQuery;
				UE_LOG(LogTemp, Display,
					TEXT("[AnalyticStrictMiss] Frame=%llu Attempt=%u Shape=%u Start=%s End=%s Rotation=%s HalfExtent=%s Radius=%.17g RequiredSource=%016llX RequiredSurface=%016llX RequiredGroup=%016llX"),
					State.Frame, State.StrictFirstMissAuthorityAttempt,
					static_cast<uint8>(Query.Shape),
					*FVector(Query.Start).ToString(),
					*FVector(Query.End).ToString(),
					*FQuat(Query.Rotation).ToString(),
					*FVector(Query.HalfExtent).ToString(), Query.Radius,
					Query.RequiredSourceId, Query.RequiredSurfaceId,
					Query.RequiredCanonicalGroupId);
			}
		}
		State.PreviousStrictProviderHash = ProviderHash;
		State.PreviousStrictMissCount = State.StrictMissCount;
		State.bHasPreviousStrictProviderHash = true;
	}
	State.bActive = false;
	State.World = nullptr;
	State.RuntimeBridge = nullptr;
}

FStaticWorldQueryCounters FStaticWorldQueryAudit::GetCurrentFrameCounters()
{
	FStaticWorldQueryCounters Counters;
	Counters.QueryCount = State.QueryCount;
	Counters.LegacySweepCount = State.LegacySweepCount;
	Counters.AuthorityAttemptCount = State.AuthorityAttemptCount;
	Counters.AuthorityCoveredCount = State.AuthorityCoveredCount;
	Counters.AuthorityFallbackCount = State.AuthorityFallbackCount;
	Counters.StrictMissingWorldCount = State.StrictMissingWorldCount;
	Counters.AuthorityMilliseconds = State.AuthorityMilliseconds;
	Counters.MaximumAuthorityMilliseconds = State.MaximumAuthorityMilliseconds;
	return Counters;
}

void FStaticWorldQueryAudit::RecordLegacySweep()
{
	if (State.bActive)
	{
		++State.LegacySweepCount;
	}
}

void FStaticWorldQueryAudit::RecordSingle(
	const EStaticQuerySite Site,
	const FVector& Start,
	const FVector& End,
	const FQuat& Rotation,
	const FCollisionShape& Shape,
	const uint8 TraceChannel,
	const FCollisionResponseParams& ResponseParams,
	const bool bHit,
	const FHitResult& UnrealHit,
	const uint64 RequiredSourceId,
	const uint64 RequiredSurfaceId,
	const uint64 RequiredCanonicalGroupId)
{
	Record(Site, Start, End, Rotation, Shape, TraceChannel, 0, false,
		&ResponseParams,
		bHit, UnrealHit, RequiredSourceId, RequiredSurfaceId,
		RequiredCanonicalGroupId);
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
	Record(Site, Start, End, Rotation, Shape, 0, ObjectTypes, true, nullptr,
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
