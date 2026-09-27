#include "WindowsGameInputFakes.h"
using Speed::Input::Windows::FGameInputReadCursor;
using Speed::Input::Windows::EReadBatchStatus;

int main()
{
	{
		ComPtr<FakeApi> Api; Api.Attach(new FakeApi);
		ComPtr<FakeDevice> Device; Device.Attach(new FakeDevice);
		FGameInputReadCursor Cursor;
		ComPtr<FakeReading> Held; Held.Attach(new FakeReading);
		Held->Stamp = 100; Held->ObservedDevice = Device;
		Api->Responses.push_back({S_OK, Held});
		const auto Initial = Cursor.PollRaw(*Api.Get(), Device.Get(), GameInputKindGamepad);
		Check(Initial.Result.Status == EReadBatchStatus::Updated && Initial.Count == 1, "initial singleton reading");
		Api->Error(GAMEINPUT_E_REFERENCE_READING_TOO_OLD);
		Api->Responses.push_back({S_OK, Held});
		Speed::Input::Windows::FTooOldReadDiagnostic QuietDiagnostic;
		const auto Quiet = Cursor.PollRaw(*Api.Get(), Device.Get(), GameInputKindGamepad, &QuietDiagnostic);
		Check(Quiet.Result.Status == EReadBatchStatus::NoChange && Quiet.Count == 0,
			"expired reference to same current singleton is not a lost input");
		Check(!QuietDiagnostic.Observed, "same singleton does not emit a true-gap diagnostic");
		Api->Reading(1, true, 200);
		Api->Responses.back().Reading->ObservedDevice = Device;
		const auto Next = Cursor.PollRaw(*Api.Get(), Device.Get(), GameInputKindGamepad);
		Check(Next.Result.Status == EReadBatchStatus::Updated && Next.Count == 1 && !Next.FreshBaseline,
			"quiet singleton preserves ordered cursor for next transition");
		Api->Error(GAMEINPUT_E_REFERENCE_READING_TOO_OLD);
		Api->Reading(0, false, 300);
		Api->Responses.back().Reading->ObservedDevice = Device;
		Speed::Input::Windows::FTooOldReadDiagnostic GapDiagnostic;
		const auto Lost = Cursor.PollRaw(*Api.Get(), Device.Get(), GameInputKindGamepad, &GapDiagnostic);
		Check(Lost.Result.Status == EReadBatchStatus::Error && Lost.Result.Error == GAMEINPUT_E_REFERENCE_READING_TOO_OLD
			&& Lost.Count == 0, "distinct current singleton still requires resynchronization");
		Check(GapDiagnostic.Observed && GapDiagnostic.HasCurrent && GapDiagnostic.CurrentResult == S_OK
			&& GapDiagnostic.PreviousTimestamp == 200 && GapDiagnostic.CurrentTimestamp == 300
			&& GapDiagnostic.ReadCallOrdinal == 1 && GapDiagnostic.HasPreviousDeviceId
			&& GapDiagnostic.HasCurrentDeviceId && GapDiagnostic.PreviousDeviceId == GapDiagnostic.CurrentDeviceId,
			"true gap records prior/current reading provenance without accepting an edge");
		Api->Reading(0, false, 301);
		const auto Fresh = Cursor.PollRaw(*Api.Get(), Device.Get(), GameInputKindGamepad);
		Check(Fresh.FreshBaseline && Fresh.Count == 1, "true gap restarts with fresh baseline");
	}
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
