#include "WindowsGameInputFakes.h"
using Speed::Input::Windows::FGameInputReadCursor;
using Speed::Input::Windows::EReadBatchStatus;

int main()
{
	for (unsigned Count : {63u, 64u, 65u})
	{
		ComPtr<FakeApi> Api; Api.Attach(new FakeApi);
		ComPtr<FakeDevice> Device; Device.Attach(new FakeDevice);
		FGameInputReadCursor Cursor;
		for (unsigned I = 0; I < Count; ++I) Api->Reading(0.5f, I % 2 == 0, 100 + I);
		const auto Batch = Cursor.PollRaw(*Api.Get(), Device.Get(), GameInputKindGamepad);
		Check(Batch.FreshBaseline, "first raw traversal marks baseline");
		if (Count <= 64)
		{
			Check(Batch.Result.Status == EReadBatchStatus::Updated && Batch.Count == Count, "raw batch retains every reading at capacity");
			for (unsigned I = 0; I < Count; ++I)
				Check(Batch.States[I].TimestampMicroseconds == 100 + I && Batch.States[I].Axes[1] == 0.5f
					&& bool(Batch.States[I].GamepadButtons & GameInputGamepadA) == (I % 2 == 0), "raw order and values unchanged");
			const auto Empty = Cursor.PollRaw(*Api.Get(), Device.Get(), GameInputKindGamepad);
			Check(Empty.Result.Status == EReadBatchStatus::NoChange && !Empty.Count && !Empty.FreshBaseline, "no-change is not synthetic reading");
		}
		else
		{
			Check(Batch.Result.Status == EReadBatchStatus::Resynchronize && Batch.Count == 0, "overflow rejects entire raw batch");
			Api->Reading(1, true, 200);
			const auto Fresh = Cursor.PollRaw(*Api.Get(), Device.Get(), GameInputKindGamepad);
			Check(Fresh.FreshBaseline && Fresh.Count == 1 && Fresh.States[0].TimestampMicroseconds == 200, "overflow restarts current reading baseline");
		}
	}
	std::cout << "PASS WindowsRawReadBatchProbe checks=" << Checks << '\n';
}
