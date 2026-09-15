#pragma once
#include "SpeedCarCameraCommand.h"
#include "HAL/CriticalSection.h"

/** Append-only, bounded run history. GT appends complete commands; only the
 * canonical owner consumes/restores the cursor. A restored prefix must match
 * this exact run history, as with IAmSpeed's sealed physical input journal. */
class IAMSPEED_API FSpeedCarCameraCommandJournal
{
public:
	explicit FSpeedCarCameraCommandJournal(uint64 FirstFrame, uint32 Capacity = 4096);
	bool QueueNext(ESpeedCarCameraCommandKind Kind, float Value, FSpeedCarCameraCommand* Assigned = nullptr);
	/** Called only at the acknowledged boundary, before Out is registered. */
	bool CopyForReplacement(FSpeedCarCameraCommandJournal& Out) const;
	bool QueueAt(const FSpeedCarCameraCommand& Command);
	bool Consume(uint64 Frame, TArray<FSpeedCarCameraCommand>& Out);
	void GetCheckpoint(uint64& ConsumedFrame, uint32& Count, uint64& Hash) const;
	bool CanRestore(uint64 Frame, uint32 Count, uint64 Hash) const;
	void RestoreValidated(uint64 Frame, uint32 Count, uint64 ReplayThrough);
private:
	bool AppendLocked(const FSpeedCarCameraCommand& Command);
	uint64 PrefixHashLocked(uint32 Count) const;
	const uint64 FirstFrame;
	const uint32 Capacity;
	mutable FCriticalSection Mutex;
	TArray<FSpeedCarCameraCommand> Commands;
	TArray<uint64> PrefixHashes;
	uint32 Cursor = 0;
	uint64 NextConsumeFrame;
	uint64 EarliestEnqueueFrame;
};
