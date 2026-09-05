#include "SimulationWorld.h"
#include "SimulationActorDiagnostics.h"

#include "IAmSpeed/Components/ISpeedComponent.h"
#include "IAmSpeed/SubBodies/Solid/SphereSubBody.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include <type_traits>

namespace
{
	constexpr uint32 SimulationSnapshotSchemaVersion = 2;

	template <typename T>
	void AppendValue(TArray<uint8>& Payload, const T& Value)
	{
		static_assert(TIsTriviallyCopyConstructible<T>::Value);
		const int32 Offset = Payload.AddUninitialized(sizeof(T));
		FMemory::Memcpy(Payload.GetData() + Offset, &Value, sizeof(T));
	}

	void AppendVector(TArray<uint8>& Payload, const FVector& Value)
	{
		AppendValue(Payload, Value.X);
		AppendValue(Payload, Value.Y);
		AppendValue(Payload, Value.Z);
	}

	/** Append schema-2 kinematics with one array growth, never copying struct padding. */
	void AppendKinematic(TArray<uint8>& Payload, const SKinematic& State)
	{
		using FScalar = decltype(State.Location.X);
		static_assert(std::is_same_v<FScalar, decltype(State.Velocity.X)> &&
			std::is_same_v<FScalar, decltype(State.Acceleration.X)> &&
			std::is_same_v<FScalar, decltype(State.Rotation.X)> &&
			std::is_same_v<FScalar, decltype(State.AngularVelocity.X)> &&
			std::is_same_v<FScalar, decltype(State.AngularAcceleration.X)>);
		const FScalar Values[] = {
			State.Location.X, State.Location.Y, State.Location.Z,
			State.Velocity.X, State.Velocity.Y, State.Velocity.Z,
			State.Acceleration.X, State.Acceleration.Y, State.Acceleration.Z,
			State.Rotation.X, State.Rotation.Y, State.Rotation.Z, State.Rotation.W,
			State.AngularVelocity.X, State.AngularVelocity.Y, State.AngularVelocity.Z,
			State.AngularAcceleration.X, State.AngularAcceleration.Y, State.AngularAcceleration.Z
		};
		Payload.Append(reinterpret_cast<const uint8*>(Values), sizeof(Values));
	}

	class FSnapshotPayloadReader final
	{
	public:
		explicit FSnapshotPayloadReader(const TArray<uint8>& InPayload)
			: Payload(InPayload)
		{
		}

		template <typename T>
		bool ReadValue(T& OutValue)
		{
			static_assert(TIsTriviallyCopyConstructible<T>::Value);
			if (Offset > Payload.Num() || sizeof(T) > static_cast<SIZE_T>(Payload.Num() - Offset))
			{
				return false;
			}
			FMemory::Memcpy(&OutValue, Payload.GetData() + Offset, sizeof(T));
			Offset += sizeof(T);
			return true;
		}

		bool ReadVector(FVector& OutValue)
		{
			return ReadValue(OutValue.X) && ReadValue(OutValue.Y) && ReadValue(OutValue.Z);
		}

		bool ReadQuat(FQuat& OutValue)
		{
			return ReadValue(OutValue.X) && ReadValue(OutValue.Y) &&
				ReadValue(OutValue.Z) && ReadValue(OutValue.W);
		}

		bool ReadBytes(const uint32 Count, TArray<uint8>& OutBytes)
		{
			if (Offset > Payload.Num() || Count > static_cast<uint32>(Payload.Num() - Offset))
			{
				return false;
			}
			OutBytes.Append(Payload.GetData() + Offset, Count);
			Offset += Count;
			return true;
		}

		bool IsAtEnd() const { return Offset == Payload.Num(); }
		int32 RemainingBytes() const { return Payload.Num() - Offset; }

	private:
		const TArray<uint8>& Payload;
		int32 Offset = 0;
	};

	uint64 MakePairKey(const USolidSubBody& BodyA, const USolidSubBody& BodyB)
	{
		const uint32 LowId = FMath::Min(BodyA.GetUniqueID(), BodyB.GetUniqueID());
		const uint32 HighId = FMath::Max(BodyA.GetUniqueID(), BodyB.GetUniqueID());
		return (static_cast<uint64>(LowId) << 32) | HighId;
	}
}

namespace Speed
{
	bool FSimulationWorld::AddAdapter(ISpeedComponent& Adapter)
	{
		if (StableIds.Contains(&Adapter)) return false;
		const uint64 StableId = NextStableId++;
		Adapters.Add(&Adapter);
		StableIds.Add(&Adapter, StableId);
		AdaptersByStableId.Add(StableId, &Adapter);
		bOrderDirty = true;
		return true;
	}

	bool FSimulationWorld::RemoveAdapter(ISpeedComponent& Adapter)
	{
		const uint64 StableId = FindStableId(Adapter);
		const int32 Removed = Adapters.Remove(&Adapter);
		StableIds.Remove(&Adapter);
		AdaptersByStableId.Remove(StableId);
		bOrderDirty |= Removed > 0;
		return Removed > 0;
	}

	bool FSimulationWorld::ContainsAdapter(const ISpeedComponent& Adapter) const
	{
		return StableIds.Contains(const_cast<ISpeedComponent*>(&Adapter));
	}

	void FSimulationWorld::RebuildOrderedAdapters()
	{
		if (!bOrderDirty) return;
		Adapters.Remove(nullptr);
		OrderedAdapters = Adapters;
		Bodies.Reset(OrderedAdapters.Num());
		StableSubBodyIds.Reset();
		SolidSubBodiesByStableId.Reset();
		ProjectionSpheres.Empty();
		ProjectionBoxes.Empty();
		for (ISpeedComponent* Adapter : OrderedAdapters)
		{
			if (const uint64* StableId = StableIds.Find(Adapter))
			{
				Bodies.Add({ *StableId, Adapter });
				const TArray<USSubBody*>& SubBodies = Adapter->GetSubBodies();
				for (int32 SubBodyIndex = 0; SubBodyIndex < SubBodies.Num(); ++SubBodyIndex)
				{
					if (USphereSubBody* Sphere = Cast<USphereSubBody>(SubBodies[SubBodyIndex]))
					{
						ProjectionSpheres.Add(Sphere);
					}
					else if (UBoxSubBody* Box = Cast<UBoxSubBody>(SubBodies[SubBodyIndex]))
					{
						ProjectionBoxes.Add(Box);
					}
					if (SubBodies[SubBodyIndex] && SubBodyIndex <= MAX_uint16)
					{
						StableSubBodyIds.Add(
							SubBodies[SubBodyIndex],
							(*StableId << 16) | static_cast<uint16>(SubBodyIndex));
						if (USolidSubBody* SolidSubBody = Cast<USolidSubBody>(SubBodies[SubBodyIndex]))
						{
							SolidSubBodiesByStableId.Add(
								(*StableId << 16) | static_cast<uint16>(SubBodyIndex),
								SolidSubBody);
						}
					}
				}
			}
		}
		bOrderDirty = false;
	}

	uint64 FSimulationWorld::FindStableId(const ISpeedComponent& Adapter) const
	{
		const uint64* StableId = StableIds.Find(const_cast<ISpeedComponent*>(&Adapter));
		return StableId ? *StableId : 0;
	}

	bool FSimulationWorld::ApplyInputsForFrame(
		const uint64 NumFrame, const SimulationBoundary::FInputJournal& Journal)
	{
		if (!Journal.IsSealed()) return false;
		bool bAllApplied = true;
		for (const FSimulationInputCommand& Command : Journal.GetCommandsForFrame(NumFrame))
		{
			ISpeedComponent* const* Adapter = AdaptersByStableId.Find(Command.TargetStableId);
			bAllApplied &= Adapter && *Adapter &&
				(*Adapter)->ApplySimulationInput(Command.Payload);
		}
		return bAllApplied;
	}

	uint64 FSimulationWorld::FindStableSubBodyId(const USSubBody* SubBody) const
	{
		const uint64* StableId = StableSubBodyIds.Find(SubBody);
		return StableId ? *StableId : 0;
	}

	FSimulationSnapshot FSimulationWorld::CaptureSnapshot(
		const uint64 NumFrame, const uint64 InputJournalHash) const
	{
		IAMSPEED_FRAME_SCOPE(SnapshotBodies);
		FSimulationSnapshot Snapshot;
		Snapshot.NumFrame = NumFrame;
		Snapshot.InputJournalHash = InputJournalHash;
		AppendValue(Snapshot.Payload, SimulationSnapshotSchemaVersion);
		AppendValue(Snapshot.Payload, NumFrame);
		AppendValue(Snapshot.Payload, InputJournalHash);
		const uint32 BodyCount = static_cast<uint32>(Bodies.Num());
		AppendValue(Snapshot.Payload, BodyCount);
		for (const FSimulationBodyRecord& Body : Bodies)
		{
			if (!Body.Adapter) continue;
			const SKinematic& State = Body.Adapter->GetKinematicState();
			AppendValue(Snapshot.Payload, Body.StableId);
			AppendKinematic(Snapshot.Payload, State);
			const uint8 bFrozen = Body.Adapter->IsFrozen() ? 1 : 0;
			AppendValue(Snapshot.Payload, bFrozen);
			TArray<uint8> MechanicPayload;
			Body.Adapter->AppendSimulationSnapshot(MechanicPayload);
			const uint32 MechanicPayloadSize = static_cast<uint32>(MechanicPayload.Num());
			AppendValue(Snapshot.Payload, MechanicPayloadSize);
			Snapshot.Payload.Append(MechanicPayload);
		}

		IAMSPEED_FRAME_PHASE(SnapshotPairs);
		const auto StableSubBodyId = [this](const USolidSubBody* SubBody)
		{
			return FindStableSubBodyId(SubBody);
		};
		const auto AppendPairs = [&Snapshot, &StableSubBodyId](
			const TArray<FDynamicContactPair>& SourcePairs)
		{
			struct FSnapshotPair
			{
				const FDynamicContactPair* Pair = nullptr;
				uint64 BodyAId = 0;
				uint64 BodyBId = 0;
				uint64 LowId = 0;
				uint64 HighId = 0;
			};
			TArray<FSnapshotPair> Pairs;
			Pairs.Reserve(SourcePairs.Num());
			for (const FDynamicContactPair& Pair : SourcePairs)
			{
				const uint64 BodyAId = StableSubBodyId(Pair.BodyA.Get());
				const uint64 BodyBId = StableSubBodyId(Pair.BodyB.Get());
				Pairs.Add({ &Pair, BodyAId, BodyBId,
					FMath::Min(BodyAId, BodyBId), FMath::Max(BodyAId, BodyBId) });
			}
			Pairs.Sort([](const FSnapshotPair& A, const FSnapshotPair& B)
			{
				return A.LowId != B.LowId ? A.LowId < B.LowId : A.HighId < B.HighId;
			});
			const uint32 PairCount = static_cast<uint32>(Pairs.Num());
			AppendValue(Snapshot.Payload, PairCount);
			for (const FSnapshotPair& SnapshotPair : Pairs)
			{
				const FDynamicContactPair* Pair = SnapshotPair.Pair;
				AppendValue(Snapshot.Payload, SnapshotPair.BodyAId);
				AppendValue(Snapshot.Payload, SnapshotPair.BodyBId);
				AppendVector(Snapshot.Payload, Pair->LocalAnchorA);
				AppendVector(Snapshot.Payload, Pair->LocalAnchorB);
				AppendVector(Snapshot.Payload, Pair->LocalNormalB);
				AppendVector(Snapshot.Payload, Pair->LastCOMA);
				AppendVector(Snapshot.Payload, Pair->LastCOMB);
				AppendValue(Snapshot.Payload, Pair->FirstSeenFrame);
				AppendValue(Snapshot.Payload, Pair->LastSeenFrame);
				AppendValue(Snapshot.Payload, Pair->LastSolvedFrame);
				AppendValue(Snapshot.Payload, Pair->SupportedFrameCount);
				AppendValue(Snapshot.Payload, Pair->ActiveContactFrameCount);
				AppendValue(Snapshot.Payload, Pair->ReactionFrameCount);
				AppendValue(Snapshot.Payload, Pair->FeatureTransitionCount);
				AppendValue(Snapshot.Payload, Pair->AcquisitionNormalSpeed);
				AppendValue(Snapshot.Payload, Pair->AccumulatedNormalImpulse);
				AppendValue(Snapshot.Payload, Pair->AccumulatedNormalActivity);
				AppendValue(Snapshot.Payload, Pair->AccumulatedNormalSupportActivity);
				AppendValue(Snapshot.Payload, Pair->PeakNormalActivity);
				AppendValue(Snapshot.Payload, Pair->PeakNormalSupportActivity);
				AppendValue(Snapshot.Payload, Pair->MinimumSeparation);
				AppendValue(Snapshot.Payload, Pair->MaximumSeparation);
				AppendValue(Snapshot.Payload, uint8(Pair->bRollingManifoldReady));
				AppendValue(Snapshot.Payload, uint8(Pair->bActiveContactLastSolve));
			}
		};
		AppendPairs(DynamicContactPairs);
		AppendPairs(PendingRollingContactPairs);
		IAMSPEED_FRAME_PHASE(SnapshotHash);
		Snapshot.StateHash = SimulationBoundary::HashBytes(
			Snapshot.Payload.GetData(), Snapshot.Payload.Num());
		return Snapshot;
	}

	bool FSimulationWorld::RestoreSnapshot(
		const FSimulationSnapshot& Snapshot,
		const uint64 ExpectedInputJournalHash)
	{
		if (bOrderDirty || Snapshot.Payload.IsEmpty() ||
			Snapshot.InputJournalHash != ExpectedInputJournalHash ||
			Snapshot.StateHash == 0 ||
			Snapshot.StateHash != SimulationBoundary::HashBytes(
				Snapshot.Payload.GetData(), Snapshot.Payload.Num()))
		{
			return false;
		}

		struct FRestoredBody
		{
			uint64 StableId = 0;
			ISpeedComponent* Adapter = nullptr;
			SKinematic KinematicState;
			bool bFrozen = false;
			TArray<uint8> MechanicPayload;
		};

		FSnapshotPayloadReader Reader(Snapshot.Payload);
		uint32 SchemaVersion = 0;
		uint64 SerializedFrame = 0;
		uint64 SerializedInputHash = 0;
		uint32 BodyCount = 0;
		if (!Reader.ReadValue(SchemaVersion) ||
			!Reader.ReadValue(SerializedFrame) ||
			!Reader.ReadValue(SerializedInputHash) ||
			!Reader.ReadValue(BodyCount) ||
			SchemaVersion != SimulationSnapshotSchemaVersion ||
			SerializedFrame != Snapshot.NumFrame ||
			SerializedInputHash != Snapshot.InputJournalHash ||
			SerializedFrame > TNumericLimits<unsigned int>::Max() ||
			BodyCount != static_cast<uint32>(Bodies.Num()))
		{
			return false;
		}

		TArray<FRestoredBody> RestoredBodies;
		RestoredBodies.Reserve(BodyCount);
		TSet<uint64> RestoredBodyIds;
		for (uint32 BodyIndex = 0; BodyIndex < BodyCount; ++BodyIndex)
		{
			FRestoredBody& Body = RestoredBodies.AddDefaulted_GetRef();
			FVector Location;
			FVector Velocity;
			FVector Acceleration;
			FVector AngularVelocity;
			FVector AngularAcceleration;
			uint8 bFrozen = 0;
			uint32 MechanicPayloadSize = 0;
			if (!Reader.ReadValue(Body.StableId) ||
				!Reader.ReadVector(Location) ||
				!Reader.ReadVector(Velocity) ||
				!Reader.ReadVector(Acceleration) ||
				!Reader.ReadQuat(Body.KinematicState.Rotation) ||
				!Reader.ReadVector(AngularVelocity) ||
				!Reader.ReadVector(AngularAcceleration) ||
				!Reader.ReadValue(bFrozen) || bFrozen > 1 ||
				!Reader.ReadValue(MechanicPayloadSize) ||
				!Reader.ReadBytes(MechanicPayloadSize, Body.MechanicPayload) ||
				RestoredBodyIds.Contains(Body.StableId))
			{
				return false;
			}
			Body.Adapter = AdaptersByStableId.FindRef(Body.StableId);
			if (!Body.Adapter ||
				!Body.Adapter->CanRestoreSimulationSnapshot(Body.MechanicPayload))
			{
				return false;
			}
			RestoredBodyIds.Add(Body.StableId);
			Body.KinematicState.Location = Location;
			Body.KinematicState.Velocity = Velocity;
			Body.KinematicState.Acceleration = Acceleration;
			Body.KinematicState.AngularVelocity = AngularVelocity;
			Body.KinematicState.AngularAcceleration = AngularAcceleration;
			Body.bFrozen = bFrozen != 0;
		}

		TSet<uint64> RestoredPairKeys;
		const auto ReadPairs = [this, &Reader, &RestoredPairKeys](
			TArray<FDynamicContactPair>& OutPairs)
		{
			uint32 PairCount = 0;
			if (!Reader.ReadValue(PairCount) ||
				PairCount > static_cast<uint32>(Reader.RemainingBytes() / (2 * sizeof(uint64))))
			{
				return false;
			}
			OutPairs.Reserve(PairCount);
			for (uint32 PairIndex = 0; PairIndex < PairCount; ++PairIndex)
			{
				uint64 BodyAId = 0;
				uint64 BodyBId = 0;
				uint8 bRollingManifoldReady = 0;
				uint8 bActiveContactLastSolve = 0;
				FDynamicContactPair& Pair = OutPairs.AddDefaulted_GetRef();
				if (!Reader.ReadValue(BodyAId) || !Reader.ReadValue(BodyBId) ||
					!Reader.ReadVector(Pair.LocalAnchorA) ||
					!Reader.ReadVector(Pair.LocalAnchorB) ||
					!Reader.ReadVector(Pair.LocalNormalB) ||
					!Reader.ReadVector(Pair.LastCOMA) ||
					!Reader.ReadVector(Pair.LastCOMB) ||
					!Reader.ReadValue(Pair.FirstSeenFrame) ||
					!Reader.ReadValue(Pair.LastSeenFrame) ||
					!Reader.ReadValue(Pair.LastSolvedFrame) ||
					!Reader.ReadValue(Pair.SupportedFrameCount) ||
					!Reader.ReadValue(Pair.ActiveContactFrameCount) ||
					!Reader.ReadValue(Pair.ReactionFrameCount) ||
					!Reader.ReadValue(Pair.FeatureTransitionCount) ||
					!Reader.ReadValue(Pair.AcquisitionNormalSpeed) ||
					!Reader.ReadValue(Pair.AccumulatedNormalImpulse) ||
					!Reader.ReadValue(Pair.AccumulatedNormalActivity) ||
					!Reader.ReadValue(Pair.AccumulatedNormalSupportActivity) ||
					!Reader.ReadValue(Pair.PeakNormalActivity) ||
					!Reader.ReadValue(Pair.PeakNormalSupportActivity) ||
					!Reader.ReadValue(Pair.MinimumSeparation) ||
					!Reader.ReadValue(Pair.MaximumSeparation) ||
					!Reader.ReadValue(bRollingManifoldReady) ||
					!Reader.ReadValue(bActiveContactLastSolve) ||
					bRollingManifoldReady > 1 || bActiveContactLastSolve > 1)
				{
					return false;
				}
				USolidSubBody* BodyA = SolidSubBodiesByStableId.FindRef(BodyAId);
				USolidSubBody* BodyB = SolidSubBodiesByStableId.FindRef(BodyBId);
				if (!BodyA || !BodyB || BodyA == BodyB)
				{
					return false;
				}
				Pair.PairKey = MakePairKey(*BodyA, *BodyB);
				if (RestoredPairKeys.Contains(Pair.PairKey))
				{
					return false;
				}
				RestoredPairKeys.Add(Pair.PairKey);
				Pair.BodyA = BodyA;
				Pair.BodyB = BodyB;
				Pair.bRollingManifoldReady = bRollingManifoldReady != 0;
				Pair.bActiveContactLastSolve = bActiveContactLastSolve != 0;
			}
			return true;
		};

		TArray<FDynamicContactPair> RestoredDynamicPairs;
		TArray<FDynamicContactPair> RestoredPendingPairs;
		if (!ReadPairs(RestoredDynamicPairs) ||
			!ReadPairs(RestoredPendingPairs) || !Reader.IsAtEnd())
		{
			return false;
		}

		// All parsing and component-specific validation is complete before any live
		// state is changed. Component restore hooks must therefore be infallible.
		for (const FRestoredBody& Body : RestoredBodies)
		{
			Body.Adapter->RestoreSimulationSnapshot(
				Body.KinematicState, Body.bFrozen, Body.MechanicPayload);
		}
		DynamicContactPairs = MoveTemp(RestoredDynamicPairs);
		PendingRollingContactPairs = MoveTemp(RestoredPendingPairs);
		DynamicContactPairIndices.Reset();
		PendingRollingContactPairIndices.Reset();
		DynamicContactPairKeysByBody.Reset();
		for (int32 PairIndex = 0; PairIndex < DynamicContactPairs.Num(); ++PairIndex)
		{
			const uint64 PairKey = DynamicContactPairs[PairIndex].PairKey;
			DynamicContactPairIndices.Add(PairKey, PairIndex);
			IndexDynamicContactPair(PairKey);
		}
		for (int32 PairIndex = 0; PairIndex < PendingRollingContactPairs.Num(); ++PairIndex)
		{
			PendingRollingContactPairIndices.Add(
				PendingRollingContactPairs[PairIndex].PairKey, PairIndex);
		}
		CurrentStepFrame = static_cast<unsigned int>(Snapshot.NumFrame);
		return true;
	}

	FDynamicContactPair* FSimulationWorld::FindDynamicContactPair(const uint64 PairKey)
	{
		const int32* Index = DynamicContactPairIndices.Find(PairKey);
		return Index ? &DynamicContactPairs[*Index] : nullptr;
	}

	const FDynamicContactPair* FSimulationWorld::FindDynamicContactPair(const uint64 PairKey) const
	{
		const int32* Index = DynamicContactPairIndices.Find(PairKey);
		return Index ? &DynamicContactPairs[*Index] : nullptr;
	}

	FDynamicContactPair* FSimulationWorld::FindPendingRollingContactPair(const uint64 PairKey)
	{
		const int32* Index = PendingRollingContactPairIndices.Find(PairKey);
		return Index ? &PendingRollingContactPairs[*Index] : nullptr;
	}

	FDynamicContactPair& FSimulationWorld::AddDynamicContactPair(
		const uint64 PairKey, USolidSubBody& BodyA, USolidSubBody& BodyB)
	{
		if (FDynamicContactPair* Existing = FindDynamicContactPair(PairKey))
		{
			return *Existing;
		}
		const int32 Index = DynamicContactPairs.AddDefaulted();
		FDynamicContactPair& Pair = DynamicContactPairs[Index];
		Pair.PairKey = PairKey;
		Pair.BodyA = &BodyA;
		Pair.BodyB = &BodyB;
		DynamicContactPairIndices.Add(PairKey, Index);
		IndexDynamicContactPair(PairKey);
		return Pair;
	}

	FDynamicContactPair& FSimulationWorld::AddPendingRollingContactPair(
		const uint64 PairKey, USolidSubBody& BodyA, USolidSubBody& BodyB)
	{
		if (FDynamicContactPair* Existing = FindPendingRollingContactPair(PairKey))
		{
			return *Existing;
		}
		const int32 Index = PendingRollingContactPairs.AddDefaulted();
		FDynamicContactPair& Pair = PendingRollingContactPairs[Index];
		Pair.PairKey = PairKey;
		Pair.BodyA = &BodyA;
		Pair.BodyB = &BodyB;
		PendingRollingContactPairIndices.Add(PairKey, Index);
		return Pair;
	}

	void FSimulationWorld::RemoveDynamicContactPairAtSwap(const int32 PairIndex)
	{
		if (!DynamicContactPairs.IsValidIndex(PairIndex)) return;
		const uint64 RemovedKey = DynamicContactPairs[PairIndex].PairKey;
		DynamicContactPairs.RemoveAtSwap(PairIndex, 1, EAllowShrinking::No);
		DynamicContactPairIndices.Remove(RemovedKey);
		UnindexDynamicContactPair(RemovedKey);
		if (DynamicContactPairs.IsValidIndex(PairIndex))
		{
			DynamicContactPairIndices.FindChecked(DynamicContactPairs[PairIndex].PairKey) = PairIndex;
		}
	}

	void FSimulationWorld::RemovePendingRollingContactPairAtSwap(const int32 PairIndex)
	{
		if (!PendingRollingContactPairs.IsValidIndex(PairIndex)) return;
		const uint64 RemovedKey = PendingRollingContactPairs[PairIndex].PairKey;
		PendingRollingContactPairs.RemoveAtSwap(PairIndex, 1, EAllowShrinking::No);
		PendingRollingContactPairIndices.Remove(RemovedKey);
		if (PendingRollingContactPairs.IsValidIndex(PairIndex))
		{
			PendingRollingContactPairIndices.FindChecked(
				PendingRollingContactPairs[PairIndex].PairKey) = PairIndex;
		}
	}

	bool FSimulationWorld::ActivatePendingRollingContactPair(const uint64 PairKey)
	{
		const int32* PendingIndex = PendingRollingContactPairIndices.Find(PairKey);
		if (!PendingIndex) return false;
		FDynamicContactPair PendingPair = MoveTemp(PendingRollingContactPairs[*PendingIndex]);
		RemovePendingRollingContactPairAtSwap(*PendingIndex);
		if (!FindDynamicContactPair(PairKey))
		{
			const int32 ActiveIndex = DynamicContactPairs.Add(MoveTemp(PendingPair));
			DynamicContactPairIndices.Add(PairKey, ActiveIndex);
			IndexDynamicContactPair(PairKey);
		}
		return true;
	}

	void FSimulationWorld::MergePendingRollingContactPairs()
	{
		for (FDynamicContactPair& PendingPair : PendingRollingContactPairs)
		{
			if (!FindDynamicContactPair(PendingPair.PairKey))
			{
				const uint64 PairKey = PendingPair.PairKey;
				const int32 ActiveIndex = DynamicContactPairs.Add(MoveTemp(PendingPair));
				DynamicContactPairIndices.Add(PairKey, ActiveIndex);
				IndexDynamicContactPair(PairKey);
			}
		}
		PendingRollingContactPairs.Reset();
		PendingRollingContactPairIndices.Reset();
	}

	const TSet<uint64>* FSimulationWorld::FindDynamicContactPairKeys(
		const uint32 SubBodyId) const
	{
		return DynamicContactPairKeysByBody.Find(SubBodyId);
	}

	int32 FSimulationWorld::FindDynamicContactPairIndex(const uint64 PairKey) const
	{
		const int32* Index = DynamicContactPairIndices.Find(PairKey);
		return Index ? *Index : INDEX_NONE;
	}

	void FSimulationWorld::IndexDynamicContactPair(const uint64 PairKey)
	{
		const uint32 LowBodyId = static_cast<uint32>(PairKey >> 32);
		const uint32 HighBodyId = static_cast<uint32>(PairKey);
		DynamicContactPairKeysByBody.FindOrAdd(LowBodyId).Add(PairKey);
		DynamicContactPairKeysByBody.FindOrAdd(HighBodyId).Add(PairKey);
	}

	void FSimulationWorld::UnindexDynamicContactPair(const uint64 PairKey)
	{
		const uint32 BodyIds[] = {
			static_cast<uint32>(PairKey >> 32), static_cast<uint32>(PairKey) };
		for (const uint32 BodyId : BodyIds)
		{
			TSet<uint64>* PairKeys = DynamicContactPairKeysByBody.Find(BodyId);
			if (!PairKeys) continue;
			PairKeys->Remove(PairKey);
			if (PairKeys->IsEmpty())
			{
				DynamicContactPairKeysByBody.Remove(BodyId);
			}
		}
	}
}
