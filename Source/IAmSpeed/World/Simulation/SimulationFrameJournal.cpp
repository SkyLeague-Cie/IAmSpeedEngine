#include "SimulationFrameJournal.h"

#include "Algo/BinarySearch.h"

namespace Speed::SimulationBoundary
{
	static constexpr uint64 FnvOffset = 1469598103934665603ull;
	static constexpr uint64 FnvPrime = 1099511628211ull;
	static constexpr uint64 FnvPrime2 = FnvPrime * FnvPrime;
	static constexpr uint64 FnvPrime4 = FnvPrime2 * FnvPrime2;
	static constexpr uint64 FnvPrime8 = FnvPrime4 * FnvPrime4;

	uint64 HashBytes(const void* Data, const SIZE_T Size)
	{
		uint64 Hash = FnvOffset;
		const uint8* Bytes = static_cast<const uint8*>(Data);
		SIZE_T Index = 0;
		for (; Size - Index >= sizeof(uint64); Index += sizeof(uint64))
		{
			uint64 Block;
			FMemory::Memcpy(&Block, Bytes + Index, sizeof(Block));
			// For eight zero bytes FNV-1a is exactly H * prime^8 modulo 2^64.
			// Snapshots contain many such blocks. No alignment/endian assumption,
			// padding read, altered byte order or cached simulation state is needed.
			if (Block == 0)
			{
				Hash *= FnvPrime8;
			}
			else
			{
				for (SIZE_T Byte = 0; Byte < sizeof(Block); ++Byte)
				{
					Hash ^= Bytes[Index + Byte];
					Hash *= FnvPrime;
				}
			}
		}
		for (; Index < Size; ++Index)
		{
			Hash ^= Bytes[Index];
			Hash *= FnvPrime;
		}
		return Hash == 0 ? 1 : Hash;
	}

	FInputJournal::FInputJournal(const uint32 InCapacity)
		: Capacity(FMath::Max(1u, InCapacity))
	{
		Commands.Reserve(Capacity);
		CommandIndices.Reserve(Capacity);
	}

	bool FInputJournal::Append(const uint64 ActivationFrame, const uint64 TargetStableId,
		const TConstArrayView<uint8> Payload)
	{
		const FSimulationInputAddress Address{ ActivationFrame, TargetStableId };
		if (bSealed || TargetStableId == 0 || Commands.Num() >= static_cast<int32>(Capacity) ||
			CommandIndices.Contains(Address))
		{
			return false;
		}
		const int32 CommandIndex = Commands.Num();
		FSimulationInputCommand& Command = Commands.AddDefaulted_GetRef();
		Command.ActivationFrame = ActivationFrame;
		Command.TargetStableId = TargetStableId;
		Command.Payload.Append(Payload.GetData(), Payload.Num());
		Command.PayloadHash = HashBytes(Command.Payload.GetData(), Command.Payload.Num());
		CommandIndices.Add(Address, CommandIndex);
		return true;
	}

	void FInputJournal::Seal()
	{
		if (bSealed) return;
		Commands.Sort([](const FSimulationInputCommand& A, const FSimulationInputCommand& B)
		{
			return A.ActivationFrame != B.ActivationFrame
				? A.ActivationFrame < B.ActivationFrame
				: A.TargetStableId < B.TargetStableId;
		});
		CommandIndices.Reset();
		FrameRanges.Reset();
		CommandIndices.Reserve(Commands.Num());
		FrameRanges.Reserve(Commands.Num());
		JournalHash = FnvOffset;
		for (int32 CommandIndex = 0; CommandIndex < Commands.Num(); ++CommandIndex)
		{
			const FSimulationInputCommand& Command = Commands[CommandIndex];
			CommandIndices.Add(
				{ Command.ActivationFrame, Command.TargetStableId }, CommandIndex);
			FSimulationInputFrameRange& Range = FrameRanges.FindOrAdd(Command.ActivationFrame);
			if (Range.Count == 0)
			{
				Range.StartIndex = CommandIndex;
			}
			++Range.Count;
			JournalHash = HashBytes(&Command.ActivationFrame, sizeof(Command.ActivationFrame)) ^
				HashBytes(&Command.TargetStableId, sizeof(Command.TargetStableId)) ^
				(Command.PayloadHash + FnvPrime + (JournalHash << 6) + (JournalHash >> 2));
		}
		if (JournalHash == 0) JournalHash = 1;
		bSealed = true;
	}

	void FInputJournal::Reset()
	{
		Commands.Reset();
		CommandIndices.Reset();
		FrameRanges.Reset();
		bSealed = false;
		JournalHash = FnvOffset;
	}

	const FSimulationInputCommand* FInputJournal::Find(
		const uint64 ActivationFrame, const uint64 TargetStableId) const
	{
		const int32* CommandIndex = CommandIndices.Find({ ActivationFrame, TargetStableId });
		return CommandIndex ? &Commands[*CommandIndex] : nullptr;
	}

	TConstArrayView<FSimulationInputCommand> FInputJournal::GetCommandsForFrame(
		const uint64 ActivationFrame) const
	{
		if (!bSealed)
		{
			return {};
		}
		const FSimulationInputFrameRange* Range = FrameRanges.Find(ActivationFrame);
		return Range
			? MakeArrayView(Commands.GetData() + Range->StartIndex, Range->Count)
			: TConstArrayView<FSimulationInputCommand>();
	}

	FSnapshotBuffer::FSnapshotBuffer(const uint32 InMaxPayloadBytes)
		: MaxPayloadBytes(FMath::Max(1u, InMaxPayloadBytes))
	{
	}

	bool FSnapshotBuffer::Publish(const FSimulationSnapshot& Snapshot)
	{
		if (Snapshot.Payload.Num() > static_cast<int32>(MaxPayloadBytes))
		{
			return false;
		}
		uint64 PresentationBytes = uint64(Snapshot.PresentationBodies.Num()) * sizeof(FSimulationPresentationBody);
		for (const FSimulationPresentationBody& Body : Snapshot.PresentationBodies)
			PresentationBytes += Body.Extension.Num();
		TSet<TPair<uint64, uint32>> Addresses;
		for (const FSimulationPresentationOutput& Output : Snapshot.PresentationOutputs)
		{
			if (!Output.OwnerStableId || !Output.Channel || Output.NumFrame != Snapshot.NumFrame ||
				Addresses.Contains({Output.OwnerStableId, Output.Channel})) return false;
			Addresses.Add({Output.OwnerStableId, Output.Channel});
			PresentationBytes += sizeof(FSimulationPresentationOutput) + Output.Payload.Num() + Output.StatePayload.Num();
		}
		if (PresentationBytes > MaxPayloadBytes) return false;
		FScopeLock Lock(&Mutex);
		const int32 NextSlot = PublishedSlot == INDEX_NONE ? 0 : 1 - PublishedSlot;
		Slots[NextSlot] = Snapshot;
		Slots[NextSlot].PublicationSerial = ++PublicationSerial;
		for (FSimulationPresentationOutput& Output : Slots[NextSlot].PresentationOutputs)
			Output.PublicationSerial = PublicationSerial;
		PublishedSlot = NextSlot;
		return true;
	}

	bool FSnapshotBuffer::ReadLatest(FSimulationSnapshot& OutSnapshot) const
	{
		FScopeLock Lock(&Mutex);
		if (PublishedSlot == INDEX_NONE) return false;
		OutSnapshot = Slots[PublishedSlot];
		return true;
	}

	uint64 FSnapshotBuffer::PublishedFrame() const
	{
		FScopeLock Lock(&Mutex);
		return PublishedSlot == INDEX_NONE ? MAX_uint64 : Slots[PublishedSlot].NumFrame;
	}

	uint64 FSnapshotBuffer::PublishedSerial() const
	{
		FScopeLock Lock(&Mutex);
		return PublishedSlot == INDEX_NONE ? 0 : Slots[PublishedSlot].PublicationSerial;
	}

	FCameraSampleBuffer::FCameraSampleBuffer(const uint32 InCapacity)
		: Capacity(FMath::Max(1u, InCapacity))
	{
		Samples.Reserve(Capacity);
	}

	bool FCameraSampleBuffer::Publish(const FCameraCanonicalSample& Sample)
	{
		if (!Sample.IsValid()) return false;
		FScopeLock Lock(&Mutex);
		if (!Samples.IsEmpty() && Samples.Last().NumFrame >= Sample.NumFrame) return false;
		if (Samples.Num() >= static_cast<int32>(Capacity))
		{
			Samples.RemoveAt(0, 1, EAllowShrinking::No);
		}
		Samples.Add(Sample);
		return true;
	}

	bool FCameraSampleBuffer::ReadFrame(const uint64 NumFrame,
		FCameraCanonicalSample& Out) const
	{
		FScopeLock Lock(&Mutex);
		for (int32 Index = Samples.Num() - 1; Index >= 0; --Index)
		{
			if (Samples[Index].NumFrame == NumFrame)
			{
				Out = Samples[Index];
				return true;
			}
		}
		Out = FCameraCanonicalSample();
		return false;
	}

	bool FCameraSampleBuffer::ReadRange(const uint64 FirstFrame, const uint64 LastFrame,
		TArray<FCameraCanonicalSample>& Out) const
	{
		Out.Reset();
		if (FirstFrame > LastFrame) return false;
		FScopeLock Lock(&Mutex);
		for (const FCameraCanonicalSample& Sample : Samples)
		{
			if (Sample.NumFrame >= FirstFrame && Sample.NumFrame <= LastFrame)
			{
				Out.Add(Sample);
			}
		}
		return !Out.IsEmpty();
	}

	void FCameraSampleBuffer::Reset()
	{
		FScopeLock Lock(&Mutex);
		Samples.Reset();
	}

	int32 FCameraSampleBuffer::Num() const
	{
		FScopeLock Lock(&Mutex);
		return Samples.Num();
	}

	bool FPresentationFrameLatch::ReadBody(const uint64 GameFrame, const FSnapshotBuffer& Buffer,
		const uint64 StableId, FSimulationPoseConsumption& Out)
	{
		Out = FSimulationPoseConsumption();
		if (LatchedGameFrame != GameFrame)
		{
			LatchedGameFrame = GameFrame;
			BodyIndices.Reset();
			Snapshot = FSimulationSnapshot();
			if (Buffer.ReadLatest(Snapshot))
			{
				for (int32 Index = 0; Index < Snapshot.PresentationBodies.Num(); ++Index)
				{
					const uint64 Id = Snapshot.PresentationBodies[Index].StableId;
					if (Id == 0 || BodyIndices.Contains(Id)) { BodyIndices.Reset(); return false; }
					BodyIndices.Add(Id, Index);
				}
			}
		}
		const int32* Index = BodyIndices.Find(StableId);
		if (!Index) return false;
		Out.NumFrame = Snapshot.NumFrame;
		Out.PublicationSerial = Snapshot.PublicationSerial;
		Out.StateHash = Snapshot.StateHash;
		Out.InputJournalHash = Snapshot.InputJournalHash;
		Out.Body = Snapshot.PresentationBodies[*Index];
		return Out.IsValid();
	}

	FFrameHashJournal::FFrameHashJournal(const uint32 InCapacity)
		: Capacity(FMath::Max(1u, InCapacity))
	{
		Entries.Reserve(FMath::Min(Capacity, 1000000u));
	}

	bool FPresentationFrameLatch::ReadOutput(const uint64 GameFrame, const FSnapshotBuffer& Buffer,
		const uint64 StableId, const uint32 Channel, FSimulationPresentationOutput& Out)
	{
		Out = FSimulationPresentationOutput();
		FSimulationPoseConsumption Body;
		if (!ReadBody(GameFrame, Buffer, StableId, Body)) return false;
		for (const FSimulationPresentationOutput& Output : Snapshot.PresentationOutputs)
		{
			if (Output.OwnerStableId != StableId || Output.Channel != Channel) continue;
			if (Output.NumFrame != Snapshot.NumFrame || Output.PublicationSerial != Snapshot.PublicationSerial) return false;
			Out = Output;
			return true;
		}
		return false;
	}

	bool FFrameHashJournal::Append(const uint64 NumFrame, const uint64 StateHash)
	{
		if (StateHash == 0 || Entries.Num() >= static_cast<int32>(Capacity) ||
			(!Entries.IsEmpty() && Entries.Last().Key >= NumFrame))
		{
			return false;
		}
		Entries.Emplace(NumFrame, StateHash);
		return true;
	}

	void FFrameHashJournal::RemoveFrom(const uint64 NumFrame)
	{
		const int32 FirstRemovedIndex = Algo::LowerBoundBy(
			Entries, NumFrame, [](const TPair<uint64, uint64>& Entry)
			{
				return Entry.Key;
			});
		if (FirstRemovedIndex < Entries.Num())
		{
			Entries.RemoveAt(
				FirstRemovedIndex,
				Entries.Num() - FirstRemovedIndex,
				EAllowShrinking::No);
		}
	}

	TOptional<FSimulationHashDivergence> FFrameHashJournal::FirstDivergence(
		const FFrameHashJournal& Other) const
	{
		const int32 SharedCount = FMath::Min(Entries.Num(), Other.Entries.Num());
		for (int32 Index = 0; Index < SharedCount; ++Index)
		{
			if (Entries[Index] != Other.Entries[Index])
			{
				return FSimulationHashDivergence{
					FMath::Min(Entries[Index].Key, Other.Entries[Index].Key),
					Entries[Index].Value,
					Other.Entries[Index].Value };
			}
		}
		if (Entries.Num() == Other.Entries.Num()) return {};
		const TPair<uint64, uint64>& Remaining = Entries.Num() > SharedCount
			? Entries[SharedCount] : Other.Entries[SharedCount];
		return Entries.Num() > SharedCount
			? FSimulationHashDivergence{ Remaining.Key, Remaining.Value, 0 }
			: FSimulationHashDivergence{ Remaining.Key, 0, Remaining.Value };
	}
}
