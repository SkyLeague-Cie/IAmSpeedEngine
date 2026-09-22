#pragma once
#include "ControlActionReader.h"
namespace Speed::Input::V2
{
enum class EControlApplication : std::uint8_t { Rejected, Dispatched, Applied, NoChange };
struct FControlApplicationReceipt { FControlRequest Request; EControlApplication Result = EControlApplication::Rejected; };
struct FControlApplicationBatch
{
	bool Gap = false, InvalidCursor = false;
	std::uint64_t Next = 0;
	std::vector<FControlApplicationReceipt> Receipts;
};
// GT-only ledger. Dispatched to Blueprint does not claim an applied effect.
class FControlApplicationJournal
{
public:
	bool Record(const FControlRequest& Request, EControlApplication Result)
	{
		if (Serial == std::numeric_limits<std::uint64_t>::max()) return false;
		History[(++Serial) % Capacity] = {Request, Result}; return true;
	}
	FControlApplicationBatch Read(std::uint64_t Cursor) const
	{
		FControlApplicationBatch Out; Out.Next = Serial;
		if (Cursor > Serial) { Out.InvalidCursor = true; return Out; }
		if (Serial - Cursor > Capacity) { Out.Gap = true; return Out; }
		for (auto I = Cursor; I < Serial; ++I) Out.Receipts.push_back(History[(I + 1) % Capacity]);
		return Out;
	}
private:
	static constexpr std::size_t Capacity = 256;
	std::array<FControlApplicationReceipt, Capacity> History{};
	std::uint64_t Serial = 0;
};
}
