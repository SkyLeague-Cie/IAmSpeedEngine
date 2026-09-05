#pragma once

#include "CoreMinimal.h"
#include "SimulationFrameJournal.h"
#include "IAmSpeed/World/Collision/StaticCollisionWorld.h"
#include "IAmSpeed/SubBodies/Solid/SolidSubBody.h"

class ISpeedComponent;
class USSubBody;
class USphereSubBody;
class UBoxSubBody;

enum class ERollingManifoldContactState : uint8
{
	Absent,
	IdentityOnly,
	ActiveContact,
};

/** Complete persistent solver record for one finite-mass contact pair. */
struct IAMSPEED_API FDynamicContactPair
{
	TWeakObjectPtr<USolidSubBody> BodyA;
	TWeakObjectPtr<USolidSubBody> BodyB;
	FVector LocalAnchorA = FVector::ZeroVector;
	FVector LocalAnchorB = FVector::ZeroVector;
	FVector LocalNormalB = FVector::UpVector;
	FVector LastCOMA = FVector::ZeroVector;
	FVector LastCOMB = FVector::ZeroVector;
	uint64 PairKey = 0;
	unsigned int FirstSeenFrame = 0;
	unsigned int LastSeenFrame = 0;
	unsigned int LastSolvedFrame = TNumericLimits<unsigned int>::Max();
	unsigned int SupportedFrameCount = 0;
	unsigned int ActiveContactFrameCount = 0;
	unsigned int ReactionFrameCount = 0;
	unsigned int FeatureTransitionCount = 0;
	float AcquisitionNormalSpeed = 0.0f;
	float AccumulatedNormalImpulse = 0.0f;
	float AccumulatedNormalActivity = 0.0f;
	float AccumulatedNormalSupportActivity = 0.0f;
	float PeakNormalActivity = 0.0f;
	float PeakNormalSupportActivity = 0.0f;
	float MinimumSeparation = TNumericLimits<float>::Max();
	float MaximumSeparation = TNumericLimits<float>::Lowest();
	bool bRollingManifoldReady = false;
	bool bActiveContactLastSolve = false;
};

namespace Speed
{
	/** Transitional adapter record. The stable id survives registry reordering. */
	struct FSimulationBodyRecord
	{
		uint64 StableId = 0;
		ISpeedComponent* Adapter = nullptr;
	};

	/**
	 * Simulation-owned registry and immutable snapshot builder. UObject-backed
	 * adapters remain temporary; published payloads contain values only.
	 */
	class IAMSPEED_API FSimulationWorld final
	{
	public:
		/** Registers one live adapter and assigns its stable id exactly once. */
		bool AddAdapter(ISpeedComponent& Adapter);
		/** Removes one live adapter while keeping already-issued ids monotonic. */
		bool RemoveAdapter(ISpeedComponent& Adapter);
		bool ContainsAdapter(const ISpeedComponent& Adapter) const;
		/** Rebuilds the canonical adapter/body view after queued registry changes. */
		void RebuildOrderedAdapters();
		const TArray<ISpeedComponent*>& GetAdapters() const { return Adapters; }
		const TArray<ISpeedComponent*>& GetOrderedAdapters() const { return OrderedAdapters; }
		/** Typed views rebuilt with the registry, preserving canonical sub-body order. */
		const TArray<USphereSubBody*>& GetProjectionSpheres() const { return ProjectionSpheres; }
		const TArray<UBoxSubBody*>& GetProjectionBoxes() const { return ProjectionBoxes; }
		/** Transfers ownership of the immutable static-query authority. */
		void SetStaticCollisionWorld(TUniquePtr<IStaticCollisionWorld>&& InWorld)
		{
			StaticCollisionWorld = MoveTemp(InWorld);
		}
		const IStaticCollisionWorld* GetStaticCollisionWorld() const
		{
			return StaticCollisionWorld.Get();
		}
		void ResetStaticCollisionWorld() { StaticCollisionWorld.Reset(); }
		TArray<FDynamicContactPair>& GetDynamicContactPairs() { return DynamicContactPairs; }
		const TArray<FDynamicContactPair>& GetDynamicContactPairs() const { return DynamicContactPairs; }
		TArray<FDynamicContactPair>& GetPendingRollingContactPairs() { return PendingRollingContactPairs; }
		/** Resolves an active persistent pair by its canonical key in average O(1). */
		FDynamicContactPair* FindDynamicContactPair(uint64 PairKey);
		const FDynamicContactPair* FindDynamicContactPair(uint64 PairKey) const;
		/** Resolves a pending persistent pair by its canonical key in average O(1). */
		FDynamicContactPair* FindPendingRollingContactPair(uint64 PairKey);
		/** Creates and indexes one pair; an existing key is returned unchanged. */
		FDynamicContactPair& AddDynamicContactPair(
			uint64 PairKey, USolidSubBody& BodyA, USolidSubBody& BodyB);
		FDynamicContactPair& AddPendingRollingContactPair(
			uint64 PairKey, USolidSubBody& BodyA, USolidSubBody& BodyB);
		/** Removes an active pair while repairing the swap-removal index. */
		void RemoveDynamicContactPairAtSwap(int32 PairIndex);
		/** Promotes one pending pair without linearly searching either collection. */
		bool ActivatePendingRollingContactPair(uint64 PairKey);
		/** Merges all pending structural changes into the active indexed collection. */
		void MergePendingRollingContactPairs();
		/** Returns the active pair keys incident to one sub-body identity. */
		const TSet<uint64>* FindDynamicContactPairKeys(uint32 SubBodyId) const;
		int32 FindDynamicContactPairIndex(uint64 PairKey) const;
		unsigned int& GetCurrentStepFrame() { return CurrentStepFrame; }
		unsigned int GetCurrentStepFrame() const { return CurrentStepFrame; }
		/** Returns the run-stable id used to address inputs and snapshot records. */
		uint64 FindStableId(const ISpeedComponent& Adapter) const;
		/** Latches every sealed command addressed to this exact canonical frame. */
		bool ApplyInputsForFrame(uint64 NumFrame, const SimulationBoundary::FInputJournal& Journal);
		/** Builds a values-only immutable output snapshot for one completed frame. */
		FSimulationSnapshot CaptureSnapshot(uint64 NumFrame, uint64 InputJournalHash) const;
		/** Atomically validates and restores a snapshot captured from this registry. */
		bool RestoreSnapshot(const FSimulationSnapshot& Snapshot, uint64 ExpectedInputJournalHash);
		int32 NumBodies() const { return Bodies.Num(); }

	private:
		TMap<ISpeedComponent*, uint64> StableIds;
		TMap<uint64, ISpeedComponent*> AdaptersByStableId;
		TMap<const USSubBody*, uint64> StableSubBodyIds;
		TMap<uint64, USolidSubBody*> SolidSubBodiesByStableId;
		TArray<ISpeedComponent*> Adapters;
		TArray<ISpeedComponent*> OrderedAdapters;
		TArray<FSimulationBodyRecord> Bodies;
		TArray<USphereSubBody*> ProjectionSpheres;
		TArray<UBoxSubBody*> ProjectionBoxes;
		uint64 NextStableId = 1;
		bool bOrderDirty = true;
		TUniquePtr<IStaticCollisionWorld> StaticCollisionWorld;
		TArray<FDynamicContactPair> DynamicContactPairs;
		TArray<FDynamicContactPair> PendingRollingContactPairs;
		TMap<uint64, int32> DynamicContactPairIndices;
		TMap<uint64, int32> PendingRollingContactPairIndices;
		TMap<uint32, TSet<uint64>> DynamicContactPairKeysByBody;
		unsigned int CurrentStepFrame = 0;

		uint64 FindStableSubBodyId(const USSubBody* SubBody) const;
		void IndexDynamicContactPair(uint64 PairKey);
		void UnindexDynamicContactPair(uint64 PairKey);
		void RemovePendingRollingContactPairAtSwap(int32 PairIndex);
	};
}
