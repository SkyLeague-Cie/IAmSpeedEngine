#include "SpeedCarCameraCommandJournal.h"
#include "IAmSpeed/World/Simulation/SimulationFrameJournal.h"
#include "Serialization/MemoryWriter.h"
#include "Misc/ScopeLock.h"

FSpeedCarCameraCommandJournal::FSpeedCarCameraCommandJournal(const uint64 InFirstFrame, const uint32 InCapacity)
	: FirstFrame(InFirstFrame), Capacity(FMath::Clamp(InCapacity, 1u, 4096u)),
	  NextConsumeFrame(InFirstFrame), EarliestEnqueueFrame(InFirstFrame)
{
	PrefixHashes.Add(1469598103934665603ull);
}

bool FSpeedCarCameraCommandJournal::AppendLocked(const FSpeedCarCameraCommand& Command)
{
	if (Command.Frame == MAX_uint64 || Commands.Num() >= int32(Capacity) || Command.Serial != uint64(Commands.Num()) + 1 ||
		Command.Frame < EarliestEnqueueFrame || !FMath::IsFinite(Command.Value) ||
		uint8(Command.Kind) > uint8(ESpeedCarCameraCommandKind::InvertSwivel) ||
		(Command.Kind == ESpeedCarCameraCommandKind::InvertSwivel && Command.Value != 0 && Command.Value != 1) ||
		(!Commands.IsEmpty() && Command.Frame < Commands.Last().Frame)) return false;
	Commands.Add(Command);
	TArray<uint8> Bytes;
	FMemoryWriter Writer(Bytes);
	auto Copy = Command;
	uint8 Kind = uint8(Copy.Kind);
	Writer << Copy.Frame << Copy.Serial << Kind << Copy.Value;
	const uint64 Previous = PrefixHashes.Last();
	uint64 Hash = Previous ^ (Speed::SimulationBoundary::HashBytes(Bytes.GetData(), Bytes.Num()) +
		1099511628211ull + (Previous << 6) + (Previous >> 2));
	PrefixHashes.Add(Hash ? Hash : 1);
	return true;
}

bool FSpeedCarCameraCommandJournal::QueueNext(const ESpeedCarCameraCommandKind Kind, const float Value, FSpeedCarCameraCommand* Assigned)
{
	FScopeLock Lock(&Mutex);
	const uint64 Frame = Commands.IsEmpty() ? EarliestEnqueueFrame
		: FMath::Max(EarliestEnqueueFrame, Commands.Last().Frame);
	const FSpeedCarCameraCommand Command{Frame, uint64(Commands.Num()) + 1, Kind, Value};
	if (!AppendLocked(Command)) return false;
	if (Assigned) *Assigned = Command;
	return true;
}

bool FSpeedCarCameraCommandJournal::CopyForReplacement(FSpeedCarCameraCommandJournal& Out) const
{
	if (&Out == this) return false;
	FScopeLock Lock(&Mutex);
	if (NextConsumeFrame != Out.FirstFrame || Commands.Num() > int32(Out.Capacity)) return false;
	Out.Commands = Commands; Out.PrefixHashes = PrefixHashes; Out.Cursor = Cursor;
	Out.NextConsumeFrame = NextConsumeFrame; Out.EarliestEnqueueFrame = EarliestEnqueueFrame;
	return true;
}

bool FSpeedCarCameraCommandJournal::QueueAt(const FSpeedCarCameraCommand& Command)
{
	FScopeLock Lock(&Mutex);
	return AppendLocked(Command);
}

bool FSpeedCarCameraCommandJournal::Consume(const uint64 Frame, TArray<FSpeedCarCameraCommand>& Out)
{
	Out.Reset();
	FScopeLock Lock(&Mutex);
	if (Frame == MAX_uint64 || Frame != NextConsumeFrame || (Cursor < uint32(Commands.Num()) && Commands[Cursor].Frame < Frame))
		return false;
	while (Cursor < uint32(Commands.Num()) && Commands[Cursor].Frame == Frame) Out.Add(Commands[Cursor++]);
	NextConsumeFrame = Frame + 1;
	EarliestEnqueueFrame = FMath::Max(EarliestEnqueueFrame, NextConsumeFrame);
	return true;
}

uint64 FSpeedCarCameraCommandJournal::PrefixHashLocked(const uint32 Count) const
{
	return PrefixHashes[Count];
}

void FSpeedCarCameraCommandJournal::GetCheckpoint(uint64& Frame, uint32& Count, uint64& Hash) const
{
	FScopeLock Lock(&Mutex);
	Frame = NextConsumeFrame - 1; Count = Cursor; Hash = PrefixHashLocked(Cursor);
}

bool FSpeedCarCameraCommandJournal::CanRestore(const uint64 Frame, const uint32 Count, const uint64 Hash) const
{
	FScopeLock Lock(&Mutex);
	if (Frame < FirstFrame || Frame >= NextConsumeFrame || Count > uint32(Commands.Num()) ||
		(Count && Commands[Count - 1].Frame > Frame) ||
		(Count < uint32(Commands.Num()) && Commands[Count].Frame <= Frame)) return false;
	return Hash == PrefixHashLocked(Count);
}

void FSpeedCarCameraCommandJournal::RestoreValidated(const uint64 Frame, const uint32 Count, const uint64 ReplayThrough)
{
	FScopeLock Lock(&Mutex);
	Cursor = Count;
	NextConsumeFrame = Frame + 1;
	// New GT input can never splice itself into the replay window.
	EarliestEnqueueFrame = FMath::Max(EarliestEnqueueFrame, ReplayThrough + 1);
}
