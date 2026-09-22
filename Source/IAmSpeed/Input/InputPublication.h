#pragma once
#include "InputFrameV2.h"

namespace Speed::Input::V2
{
struct FPublicationCursor { FStreamEpoch Epoch; std::uint64_t Serial = 0; };
struct FPublishedFrame { FInputFrame Frame; std::uint64_t Serial = 0; };
enum class EReadStatus : std::uint8_t { NoChange, Batch, Overflow, Detached, InvalidCursor };
struct FPublishedBatch
{
	EReadStatus Status = EReadStatus::Detached;
	FPublicationCursor Next;
	std::vector<FPublishedFrame> Frames;
};
class IInputPublicationSource
{
public:
    virtual ~IInputPublicationSource() = default;
    virtual const std::shared_ptr<const FInputActionContract>& GetContract() const = 0;
    virtual FStreamEpoch GetEpoch() const = 0;
    virtual bool CanConfigure() const = 0;
    virtual bool IsActive() const = 0;
    virtual void Deactivate() = 0;
    virtual FPublishedBatch ReadPublishedSince(FPublicationCursor Cursor) const = 0;
    virtual std::optional<FPublishedFrame> ReadLatest() const = 0;
};
}
