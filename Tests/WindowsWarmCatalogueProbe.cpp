#include "WindowsDiscoveryFakes.h"
#include "IAmSpeed/Input/Windows/GameInputWarmCatalogue.h"
#include "IAmSpeed/Input/Windows/GameInputWarmContext.h"
#include <chrono>
#include <condition_variable>
#include <future>

using Speed::Input::Windows::FGameInputWarmCatalogue;
using Speed::Input::Windows::FGameInputWarmContext;

struct FBlockedWarmSubscriber
{
    std::mutex Gate;
    std::condition_variable Changed;
    bool Entered = false;
    bool Release = false;
    static bool OnRecord(void* Context, const FGameInputWarmCatalogue::FRecord&) noexcept
    {
        auto& Self = *static_cast<FBlockedWarmSubscriber*>(Context);
        std::unique_lock<std::mutex> Lock(Self.Gate);
        Self.Entered = true;
        Self.Changed.notify_all();
        Self.Changed.wait(Lock, [&] { return Self.Release; });
        return true;
    }
    static void OnFailure(void*, HRESULT) noexcept {}
};

int main()
{
    ComPtr<DiscoveryApi> Api;
    Api.Attach(new DiscoveryApi);
    const auto Pad = MakeDevice(1);
    const auto Replacement = MakeDevice(1);
    const auto Second = MakeDevice(2);
    Api->Initial = {{Pad, 1, true}};
    std::shared_ptr<FGameInputWarmCatalogue> Warm = FGameInputWarmCatalogue::Create(Api.Get());
    Check(bool(Warm) && Api->AllKinds, "one blocking OS enumeration");

    const std::array<bool, ActionCount> Digital{};
    auto First = FGameInputDiscovery::Create(Warm, 101, Digital);
    Check(bool(First) && Warm->SubscriberCount() == 1, "fresh first view subscribes");
    auto Snapshot = First->Snapshot();
    Check(Snapshot.size() == 1 && Snapshot[0].Id == Key(1) && Snapshot[0].Revision == 1,
        "first view seeded without its own SDK callback");
    Check(First->Select(std::make_pair(Key(1), EDeviceKind::Gamepad)), "first view can select pad");
    auto OldLease = First->AcquireSelected();
    Check(OldLease && OldLease->Device.Get() == Pad.Get(), "first view acquired initial COM identity");

    Api->FireDevice(Pad.Get(), 2, false);
    Check(!First->AcquireSelected(), "disconnect invalidates first selected view");
    Api->FireDevice(Replacement.Get(), 3, true);
    Snapshot = First->Snapshot();
    Check(Snapshot.size() == 1 && Snapshot[0].Connected && Snapshot[0].Revision == 3,
        "replacement identity arrives through warm callback");
    Check(!First->Submit(*OldLease, 1, {}), "old COM ticket cannot submit after replacement");

    Check(First->Shutdown(), "first view detaches without unregistering persistent SDK callback");
    Check(Warm->SubscriberCount() == 0 && Api->UnregisterCalls == 0,
        "respawn leaves warm callback alive");
    First.reset();
    auto NewSession = FGameInputDiscovery::Create(Warm, 102, Digital);
    Check(bool(NewSession) && !NewSession->AcquireSelected(),
        "new session starts with fresh selection state, not previous selection");
    Snapshot = NewSession->Snapshot();
    Check(Snapshot.size() == 1 && Snapshot[0].Revision == 3,
        "new session sees current replacement without blocking enumeration");
    Api->FireDevice(Second.Get(), 4, true);
    Snapshot = NewSession->Snapshot();
    Check(Snapshot.size() == 2 && Snapshot[1].Id == Key(2),
        "hotplug after respawn reaches only current view");

    Check(!Warm->Shutdown(), "warm callback cannot close while a view is live");
    Check(NewSession->Shutdown(), "new view detaches");
    NewSession.reset();
    Check(Warm->Shutdown() && Api->UnregisterCalls == 1,
        "controller closes one persistent callback after all sessions");

    {
        ComPtr<DiscoveryApi> FailedApi;
        FailedApi.Attach(new DiscoveryApi);
        const auto FailedPad = MakeDevice(7);
        FailedApi->Initial = {{FailedPad, 10, true}};
        std::shared_ptr<FGameInputWarmCatalogue> FailedWarm = FGameInputWarmCatalogue::Create(FailedApi.Get());
        Check(bool(FailedWarm), "terminal event fixture catalogue created");
        auto A = FGameInputDiscovery::Create(FailedWarm, 201, Digital);
        auto B = FGameInputDiscovery::Create(FailedWarm, 202, Digital);
        Check(bool(A) && bool(B) && FailedWarm->SubscriberCount() == 2,
            "two independent session views subscribed");
        FailedApi->FireDevice(FailedPad.Get(), 10, false);
        Check(FailedWarm->GetError() == E_UNEXPECTED
            && A->GetLastError() == E_UNEXPECTED && B->GetLastError() == E_UNEXPECTED,
            "divergent same-timestamp event fails every subscribed view");
        Check(!FGameInputDiscovery::Create(FailedWarm, 203, Digital),
            "terminal catalogue refuses future sessions");
        Check(A->Shutdown() && B->Shutdown(), "terminal views detach");
        A.reset(); B.reset();
        Check(FailedWarm->Shutdown(), "terminal catalogue unregisters after views");
    }

    {
        ComPtr<DiscoveryApi> TieApi;
        TieApi.Attach(new DiscoveryApi);
        const auto TiePad = MakeDevice(11);
        const auto TieReplacement = MakeDevice(11);
        TieApi->Initial = {{TiePad, 20, true}};
        auto TieWarm = FGameInputWarmCatalogue::Create(TieApi.Get());
        Check(bool(TieWarm), "tie fixture catalogue created");
        TieApi->FireDevice(TiePad.Get(), 21, false);
        TieApi->FireDevice(TieReplacement.Get(), 21, true);
        Check(TieWarm->GetError() == E_UNEXPECTED,
            "same-timestamp replacement cannot bypass a retained tombstone");
        Check(TieWarm->Shutdown(), "tie fixture closes");
    }

    {
        ComPtr<DiscoveryApi> QueuedApi;
        QueuedApi.Attach(new DiscoveryApi);
        const auto Existing = MakeDevice(21);
        const auto Queued = MakeDevice(22);
        QueuedApi->Initial = {{Existing, 50, true}};
        std::shared_ptr<FGameInputWarmCatalogue> QueuedWarm =
            FGameInputWarmCatalogue::Create(QueuedApi.Get());
        Check(bool(QueuedWarm), "queued-hotplug fixture catalogue created");
        // The OS observed this arrival before the fresh view subscribed, but
        // has not yet dispatched its already registered callback.
        const DiscoveryApi::Event Deferred{Queued, 51, true};
        auto View = FGameInputDiscovery::Create(QueuedWarm, 250, Digital);
        Check(bool(View) && View->Snapshot().size() == 1,
            "new view seeds before a previously queued callback dispatches");
        QueuedApi->FireDevice(Deferred.Device.Get(), Deferred.Stamp, Deferred.Connected);
        Check(View->Snapshot().size() == 2 && View->GetLastError() == S_OK,
            "queued pre-subscription arrival reaches the newly installed view");
        Check(View->Shutdown(), "queued-hotplug view detaches");
        View.reset();
        Check(QueuedWarm->Shutdown(), "queued-hotplug fixture closes");
    }

    {
        ComPtr<DiscoveryApi> OrderApi;
        OrderApi.Attach(new DiscoveryApi);
        const auto FirstPad = MakeDevice(13);
        const auto EarlierPad = MakeDevice(14);
        OrderApi->Initial = {{FirstPad, 40, true}};
        auto OrderWarm = FGameInputWarmCatalogue::Create(OrderApi.Get());
        Check(bool(OrderWarm), "callback-order fixture catalogue created");
        OrderApi->FireDevice(EarlierPad.Get(), 39, true);
        Check(OrderWarm->GetError() == E_UNEXPECTED,
            "globally older callback fails closed instead of evading tombstone order");
        Check(OrderWarm->Shutdown(), "callback-order fixture closes");
    }

    {
        ComPtr<DiscoveryApi> ChurnApi;
        ChurnApi.Attach(new DiscoveryApi);
        std::shared_ptr<FGameInputWarmCatalogue> ChurnWarm = FGameInputWarmCatalogue::Create(ChurnApi.Get());
        Check(bool(ChurnWarm), "churn fixture catalogue created");
        auto OldView = FGameInputDiscovery::Create(ChurnWarm, 301, Digital);
        std::unique_ptr<FGameInputDiscovery> RecentView;
        for (unsigned I = 1; I <= 260; ++I)
        {
            if (I == 248) RecentView = FGameInputDiscovery::Create(ChurnWarm, 302, Digital);
            auto Device = MakeDevice(0);
            auto* Bytes = reinterpret_cast<std::uint8_t*>(&Device->Info.deviceId);
            Bytes[0] = static_cast<std::uint8_t>(I);
            Bytes[1] = static_cast<std::uint8_t>(I >> 8);
            ChurnApi->FireDevice(Device.Get(), static_cast<std::uint64_t>(I) * 2, true);
            ChurnApi->FireDevice(Device.Get(), static_cast<std::uint64_t>(I) * 2 + 1, false);
        }
        Check(bool(RecentView) && OldView->GetLastError() == E_OUTOFMEMORY,
            "old session alone fails closed after 256 distinct devices");
        Check(ChurnWarm->GetError() == S_OK && RecentView->GetLastError() == S_OK,
            "a failed old session does not poison the warm catalogue or recent session");
        auto FreshView = FGameInputDiscovery::Create(ChurnWarm, 303, Digital);
        Check(bool(FreshView) && FreshView->GetLastError() == S_OK
            && FreshView->Snapshot().size() == 1,
            "new session seeds only the latest tombstone after long device churn");
        Check(OldView->Shutdown() && RecentView->Shutdown() && FreshView->Shutdown(),
            "churn views detach independently");
        OldView.reset(); RecentView.reset(); FreshView.reset();
        Check(ChurnWarm->Shutdown(), "churn fixture closes one callback");
    }

    {
        using namespace std::chrono_literals;
        ComPtr<DiscoveryApi> RaceApi;
        RaceApi.Attach(new DiscoveryApi);
        auto RaceWarm = FGameInputWarmCatalogue::Create(RaceApi.Get());
        FBlockedWarmSubscriber Blocked;
        const auto Subscription = RaceWarm->Subscribe(&Blocked,
            &FBlockedWarmSubscriber::OnRecord, &FBlockedWarmSubscriber::OnFailure);
        Check(Subscription != 0, "concurrent fixture subscribed");
        const auto RacePad = MakeDevice(12);
        auto Callback = std::async(std::launch::async, [&] { RaceApi->FireDevice(RacePad.Get(), 30, true); });
        bool Entered = false;
        {
            std::unique_lock<std::mutex> Lock(Blocked.Gate);
            Entered = Blocked.Changed.wait_for(Lock, 2s, [&] { return Blocked.Entered; });
        }
        if (Entered)
        {
            std::promise<void> AttemptingUnsubscribe;
            auto AttemptStarted = AttemptingUnsubscribe.get_future();
            auto UnregisterView = std::async(std::launch::async,
                [&] {
                    AttemptingUnsubscribe.set_value();
                    return RaceWarm->Unsubscribe(Subscription);
                });
            AttemptStarted.wait();
            const bool Fenced = UnregisterView.wait_for(20ms) == std::future_status::timeout;
            {
                std::lock_guard<std::mutex> Lock(Blocked.Gate);
                Blocked.Release = true;
            }
            Blocked.Changed.notify_all();
            Callback.get();
            Check(Fenced && UnregisterView.get(),
                "unsubscribe waits for an in-flight callback without lock inversion");
        }
        else
        {
            {
                std::lock_guard<std::mutex> Lock(Blocked.Gate);
                Blocked.Release = true;
            }
            Blocked.Changed.notify_all();
            Callback.get();
            Check(false, "concurrent callback entered the subscriber");
            Check(RaceWarm->Unsubscribe(Subscription), "concurrent fixture unsubscribes");
        }
        Check(RaceWarm->Shutdown(), "concurrent fixture closes after callback fence");
    }

    {
        ComPtr<DiscoveryApi> RetryApi;
        RetryApi.Attach(new DiscoveryApi);
        auto RetryWarm = FGameInputWarmCatalogue::Create(RetryApi.Get());
        Check(bool(RetryWarm), "shutdown retry fixture catalogue created");
        RetryApi->FailUnregister = true;
        Check(!RetryWarm->Shutdown() && RetryApi->UnregisterCalls == 1,
            "failed SDK unregister retains callback ownership for retry");
        RetryApi->FailUnregister = false;
        Check(RetryWarm->Shutdown() && RetryWarm->Shutdown() && RetryApi->UnregisterCalls == 2,
            "warm callback shutdown retries and then becomes idempotent");
    }

    {
        ComPtr<DiscoveryApi> ContextApi;
        ContextApi.Attach(new DiscoveryApi);
        const auto ContextPad = MakeDevice(31);
        ContextApi->Initial = {{ContextPad, 70, true}};
        FGameInputWarmContext Context;
        Check(!Context.GetOrCreate(nullptr), "inert context does not create GameInput on caller thread");
        auto Catalogue = Context.GetOrCreate(ContextApi.Get());
        Check(bool(Catalogue) && Catalogue == Context.GetOrCreate(nullptr),
            "one owner returns the same warm catalogue to later sessions");
        auto FirstView = FGameInputDiscovery::Create(Catalogue, 401, Digital);
        Check(bool(FirstView) && FirstView->Snapshot().size() == 1,
            "first context session sees enumerated device");
        Check(!Context.Close(), "context refuses teardown while a session view is live");
        Check(FirstView->Shutdown(), "first context view detaches");
        FirstView.reset();
        auto SecondView = FGameInputDiscovery::Create(Context.GetOrCreate(nullptr), 402, Digital);
        Check(bool(SecondView) && SecondView->Snapshot().size() == 1
            && ContextApi->UnregisterCalls == 0,
            "replacement session uses retained callback and device without enumeration");
        Check(SecondView->Shutdown(), "replacement context view detaches");
        SecondView.reset();
        ContextApi->FailUnregister = true;
        Check(!Context.Close() && ContextApi->UnregisterCalls == 1,
            "context retains failed callback teardown for a bounded retry");
        ContextApi->FailUnregister = false;
        Check(Context.Close() && Context.Close() && ContextApi->UnregisterCalls == 2,
            "persistent owner retries once and closes after all sessions join");
        Check(!Context.GetOrCreate(ContextApi.Get()), "closed context cannot resurrect callback");
    }

    std::cout << "PASS WindowsWarmCatalogueProbe checks=" << Checks << '\n';
}
