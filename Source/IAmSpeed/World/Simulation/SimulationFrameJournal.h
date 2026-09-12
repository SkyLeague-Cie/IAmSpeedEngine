#pragma once

#include "CoreMinimal.h"
#include "HAL/CriticalSection.h"
#include "IAmSpeed/Base/Kinematic.h"

/**
 * Small, engine-agnostic message used at the simulation boundary.  The
 * payload is deliberately opaque: game adapters serialize their own input
 * type, while IAmSpeed owns frame addressing, sealing and hashing.
 */
struct IAMSPEED_API FSimulationInputCommand
{
	uint64 ActivationFrame = 0;
	uint64 TargetStableId = 0;
	TArray<uint8> Payload;
	uint64 PayloadHash = 0;
};

/** Optional values-only presentation data; excluded from the canonical byte hash. */
struct IAMSPEED_API FSimulationPresentationBody
{
	uint64 StableId = 0;
	SKinematic COMState;
	FVector CenterOfMassLocal = FVector::ZeroVector;
	TArray<uint8> Extension;
	FVector OriginLocation() const { return COMState.Location - COMState.Rotation.RotateVector(CenterOfMassLocal); }
	FVector OriginVelocity() const
	{
		return COMState.Velocity - FVector::CrossProduct(COMState.AngularVelocity,
			COMState.Rotation.RotateVector(CenterOfMassLocal));
	}
};

/** Opaque game-owned output; addressing/serial are owned by the common publisher. */
struct IAMSPEED_API FSimulationPresentationOutput
{
	uint64 OwnerStableId = 0;
	uint32 Channel = 0;
	uint64 NumFrame = 0;
	uint64 PublicationSerial = 0;
	TArray<uint8> Payload;
};

struct IAMSPEED_API FSimulationSnapshot
{
	uint64 NumFrame = 0;
	uint64 InputJournalHash = 0;
	uint64 StateHash = 0;
	TArray<uint8> Payload;
	uint64 PublicationSerial = 0;
	TArray<FSimulationPresentationBody> PresentationBodies;
	/** Derived presentation is committed under the SAME publication lock as bodies. */
	TArray<FSimulationPresentationOutput> PresentationOutputs;
};

/** Values-only producer, retained by shared ownership across a canonical call.
 * Registration never transfers a UObject to the worker. Rollback invalidation is
 * explicit until a producer implements its own complete restoration protocol.
 */
class IAMSPEED_API ISimulationPresentationProducer
{
public:
	virtual ~ISimulationPresentationProducer() = default;
	virtual uint64 OwnerStableId() const = 0;
	virtual uint32 Channel() const = 0;
	virtual void Produce(const FSimulationSnapshot& Bodies, FSimulationPresentationOutput& Out) = 0;
	virtual void InvalidateTimeline() = 0;
};

/** Exact publication selected by a presentation consumer, scoped to one simulation. */
struct IAMSPEED_API FSimulationPoseConsumption
{
	uint64 NumFrame = 0;
	uint64 PublicationSerial = 0;
	uint64 StateHash = 0;
	uint64 InputJournalHash = 0;
	FSimulationPresentationBody Body;
	bool IsValid() const { return PublicationSerial != 0 && Body.StableId != 0; }
};

/** Canonical camera pose emitted by a physics-frame evaluator. */
struct IAMSPEED_API FCameraCanonicalSample
{
	uint64 NumFrame = 0;
	uint64 PublicationSerial = 0;
	uint64 StateHash = 0;
	uint64 InputJournalHash = 0;
	uint64 CarBodyStableId = 0;
	uint64 BallBodyStableId = 0;
	uint8 EvaluatorKind = 0; // 0: static CarCam, 1: high-ball look-at.
	FVector Position = FVector::ZeroVector;
	FQuat Rotation = FQuat::Identity;
	bool IsValid() const
	{
		return PublicationSerial != 0 && StateHash != 0 && InputJournalHash != 0 &&
			CarBodyStableId != 0 && BallBodyStableId != 0 &&
			!Position.ContainsNaN() && Rotation.IsNormalized();
	}
};

struct IAMSPEED_API FSimulationHashDivergence
{
	uint64 NumFrame = 0;
	uint64 ExpectedHash = 0;
	uint64 ActualHash = 0;
};

/** Exact address of one input command inside a simulation run. */
struct FSimulationInputAddress
{
	uint64 ActivationFrame = 0;
	uint64 TargetStableId = 0;

	bool operator==(const FSimulationInputAddress& Other) const
	{
		return ActivationFrame == Other.ActivationFrame &&
			TargetStableId == Other.TargetStableId;
	}

	friend uint32 GetTypeHash(const FSimulationInputAddress& Address)
	{
		return HashCombineFast(
			GetTypeHash(Address.ActivationFrame),
			GetTypeHash(Address.TargetStableId));
	}
};

/** Contiguous command range for one frame in the sealed, sorted journal. */
struct FSimulationInputFrameRange
{
	int32 StartIndex = 0;
	int32 Count = 0;
};

namespace Speed::SimulationBoundary
{
	/** Stable FNV-1a hash for serialized boundary messages. */
	IAMSPEED_API uint64 HashBytes(const void* Data, SIZE_T Size);

	/**
	 * Bounded, frame-addressed input journal.  Appends are rejected after
	 * Seal() and duplicate {frame,target} addresses are rejected, making the sealed
	 * journal safe to consume repeatedly by real-time and fast drivers.
	 */
	class IAMSPEED_API FInputJournal final
	{
	public:
		explicit FInputJournal(uint32 InCapacity = 4096);
		/** Queues one component payload; the {frame,target} pair must be unique. */
		bool Append(uint64 ActivationFrame, uint64 TargetStableId, TConstArrayView<uint8> Payload);
		void Seal();
		void Reset();
		bool IsSealed() const { return bSealed; }
		uint64 StableHash() const { return JournalHash; }
		int32 Num() const { return Commands.Num(); }
		/** Finds the command addressed to one component at an exact frame. */
		const FSimulationInputCommand* Find(uint64 ActivationFrame, uint64 TargetStableId) const;
		/** Returns only the contiguous commands addressed to one exact frame. */
		TConstArrayView<FSimulationInputCommand> GetCommandsForFrame(uint64 ActivationFrame) const;
		const TArray<FSimulationInputCommand>& GetCommands() const { return Commands; }

	private:
		uint32 Capacity = 0;
		bool bSealed = false;
		uint64 JournalHash = 1469598103934665603ull;
		TArray<FSimulationInputCommand> Commands;
		TMap<FSimulationInputAddress, int32> CommandIndices;
		TMap<uint64, FSimulationInputFrameRange> FrameRanges;
	};

	/** Two-slot publication buffer. Readers receive a complete immutable copy. */
	class IAMSPEED_API FSnapshotBuffer final
	{
	public:
		explicit FSnapshotBuffer(uint32 InMaxPayloadBytes = 64 * 1024);
		bool Publish(const FSimulationSnapshot& Snapshot);
		bool ReadLatest(FSimulationSnapshot& OutSnapshot) const;
		uint64 PublishedFrame() const;
		uint64 PublishedSerial() const;

	private:
		uint32 MaxPayloadBytes = 0;
		mutable FCriticalSection Mutex;
		FSimulationSnapshot Slots[2];
		int32 PublishedSlot = INDEX_NONE;
		uint64 PublicationSerial = 0;
	};

	/** Bounded exact-frame history for physics-side camera samples. */
	class IAMSPEED_API FCameraSampleBuffer final
	{
	public:
		explicit FCameraSampleBuffer(uint32 InCapacity = 4096);
		bool Publish(const FCameraCanonicalSample& Sample);
		bool ReadFrame(uint64 NumFrame, FCameraCanonicalSample& Out) const;
		bool ReadRange(uint64 FirstFrame, uint64 LastFrame,
			TArray<FCameraCanonicalSample>& Out) const;
		void Reset();
		int32 Num() const;
	private:
		uint32 Capacity = 0;
		mutable FCriticalSection Mutex;
		TArray<FCameraCanonicalSample> Samples;
	};

	/** Single-consumer frame latch: all actors on one game frame see one publication. */
	class IAMSPEED_API FPresentationFrameLatch final
	{
	public:
		bool ReadBody(uint64 GameFrame, const FSnapshotBuffer& Buffer, uint64 StableId, FSimulationPoseConsumption& Out);
		bool ReadOutput(uint64 GameFrame, const FSnapshotBuffer& Buffer, uint64 StableId,
			uint32 Channel, FSimulationPresentationOutput& Out);
	private:
		uint64 LatchedGameFrame = MAX_uint64;
		FSimulationSnapshot Snapshot;
		TMap<uint64, int32> BodyIndices;
	};

	/** Bounded per-frame hash history used for Fast/RealTime equivalence checks. */
	class IAMSPEED_API FFrameHashJournal final
	{
	public:
		explicit FFrameHashJournal(uint32 InCapacity = 1000000);
		/** Appends a strictly increasing frame/hash pair. */
		bool Append(uint64 NumFrame, uint64 StateHash);
		/** Removes the restored frame and every later hash before resimulation. */
		void RemoveFrom(uint64 NumFrame);
		void Reset() { Entries.Reset(); }
		int32 Num() const { return Entries.Num(); }
		/** Returns the first missing or different frame between two completed runs. */
		TOptional<FSimulationHashDivergence> FirstDivergence(const FFrameHashJournal& Other) const;

	private:
		uint32 Capacity = 0;
		TArray<TPair<uint64, uint64>> Entries;
	};
}
