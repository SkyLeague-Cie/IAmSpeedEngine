#pragma once
#include "GameInputDiscovery.h"
#include "GameInputCanonicalControls.h"
#include "../DesktopPublicationValidation.h"
#include <deque>
#include <set>

namespace Speed::Input::Windows
{
// Acquisition-thread collector. Creation, Prepare, Commit and Close belong to
// that same thread. SDK callbacks only retain bounded reading identities.
// Does not select a gamepad, publish a journal or create a physical frame.
class FGameInputDesktopSource final
{
    using FApi=GameInput::v3::IGameInput;
    using FReading=GameInput::v3::IGameInputReading;
    template<class T> using TComPtr=Microsoft::WRL::ComPtr<T>;
    struct FReader;
    struct FQueued {std::shared_ptr<FReader> Reader;TComPtr<FReading> Reading;std::uint64_t Arrival=0;};
    struct FAccepted
    {
        std::shared_ptr<FReader> Reader;
        V2::FDesktopFullObservation Observation;
        TComPtr<IUnknown> Identity;
    };
public:
    enum class EStatus {NoChange,Prepared,Unavailable,Rejected};
    class FPrepared
    {
        friend class FGameInputDesktopSource;
        const FGameInputDesktopSource* Owner=nullptr;
        std::uint64_t Registry=0;
        FGameInputDiscovery::FDesktopSnapshot Catalogue;
        std::unique_ptr<V2::FDesktopInputSource::FPrepared> Source;
        std::shared_ptr<const V2::FDesktopPublication> Publication;
        std::vector<FAccepted> Accepted;
        std::vector<std::uint64_t> Consumed;
        bool Activity=false;
        FPrepared()=default;
    public:
        FPrepared(const FPrepared&)=delete;
        FPrepared& operator=(const FPrepared&)=delete;
        std::shared_ptr<const V2::FDesktopPublication> Inspect()const{return Publication;}
        bool MouseActivity()const{return Activity;}
    };
    struct FPreparation
    {
        EStatus Status=EStatus::NoChange;
        std::unique_ptr<FPrepared> Prepared;
        HRESULT Error=S_OK;
    };
    static std::unique_ptr<FGameInputDesktopSource> Create(FApi* Api,FGameInputDiscovery* Discovery,std::uint64_t ProducerSession)
    {
        if(!Api||!Discovery||!ProducerSession||FPresentationInputScope::IsActive())return {};
        return std::unique_ptr<FGameInputDesktopSource>(new FGameInputDesktopSource(Api,Discovery,ProducerSession));
    }
    ~FGameInputDesktopSource(){if(!Close())std::terminate();}
    FPreparation Prepare(bool RequireFreshBaseline)
    {
        using namespace V2;
        if(!IsOwner()||Stopping||FPresentationInputScope::IsActive())return Fail(EStatus::Rejected,E_UNEXPECTED);
        auto Snapshot=Discovery->CaptureDesktop();
        if(!Snapshot)return Fail(EStatus::Rejected,Discovery->GetLastError());
        if(Snapshot->Devices.size()>FDesktopInputSource::MaxMembers)return Fail(EStatus::Rejected,E_OUTOFMEMORY);
        if(RequireFreshBaseline)NeedsFresh=true;
        if(!SameCatalogue(*Snapshot))
        {
            if(!RefreshReaders(*Snapshot))return Fail(SetupStatus,LastError);
            Captured=*Snapshot;NeedsFresh=true;
        }
        std::vector<FQueued> Observed;
        {
            std::lock_guard<std::mutex> Lock(Gate);
            if(CallbackInvalid)return Fail(EStatus::Rejected,E_UNEXPECTED);
            if(Overflow)return Fail(EStatus::Unavailable,E_BOUNDS);
            // Repeated callback identities already accepted carry no new input.
            for(auto I=Pending.begin();I!=Pending.end();)
            {
                TComPtr<IUnknown> Identity;
                if(FAILED(I->Reading.As(&Identity)))return Fail(EStatus::Rejected,E_UNEXPECTED);
                if(Seen(*I->Reader,Identity.Get()))I=Pending.erase(I);else ++I;
            }
            Observed.assign(Pending.begin(),Pending.end());
        }
        const bool Fresh=NeedsFresh;
        if(!Fresh&&Observed.empty())return {};
        auto Result=std::unique_ptr<FPrepared>(new FPrepared);
        Result->Owner=this;Result->Registry=RegistryRevision;Result->Catalogue=*Snapshot;
        FDesktopBatch Batch;Batch.Mode=Fresh?EDesktopBatchMode::FreshBaseline:EDesktopBatchMode::Poll;
        Batch.RecoverOverflow=RecoverNotices;
        if(Fresh)
        {
            for(const auto& R:Readers)
            {
                TComPtr<FReading> Reading;std::uint64_t Arrival=0;
                for(const auto& Q:Observed)if(Q.Reader==R){Reading=Q.Reading;Arrival=Q.Arrival;break;}
                if(!Reading)
                {
                    const auto H=Api->GetCurrentReading(Kind(*R),R->Lease.Device.Get(),Reading.GetAddressOf());
                    if(H==GAMEINPUT_E_READING_NOT_FOUND)return {};
                    if(FAILED(H)||!Reading)return Fail(H==E_INVALIDARG||(SUCCEEDED(H)&&!Reading)?EStatus::Rejected:EStatus::Unavailable,FAILED(H)?H:E_UNEXPECTED);
                    // Callbacks may arrive during the current snapshot. Their
                    // earliest retained state, not the later current snapshot,
                    // establishes baseline; later entries remain for next poll.
                    std::lock_guard<std::mutex> Lock(Gate);
                    if(CallbackInvalid)return Fail(EStatus::Rejected,E_UNEXPECTED);
                    if(Overflow)return Fail(EStatus::Unavailable,E_BOUNDS);
                    for(const auto& Q:Pending)if(Q.Reader==R)
                    {Reading=Q.Reading;Arrival=Q.Arrival;break;}
                }
                FAccepted A;const auto H=Decode(R,Reading.Get(),R->Sequence+1,A);
                if(FAILED(H))return Fail(EStatus::Rejected,H);
                Batch.Observations.push_back(A.Observation);Result->Accepted.push_back(std::move(A));
                if(Arrival)Result->Consumed.push_back(Arrival);
            }
        }
        else
        {
            for(const auto& Q:Observed)
            {
                TComPtr<IUnknown> Identity;
                if(FAILED(Q.Reading.As(&Identity)))return Fail(EStatus::Rejected,E_UNEXPECTED);
                Result->Consumed.push_back(Q.Arrival);
                bool Duplicate=Seen(*Q.Reader,Identity.Get());
                std::uint64_t Sequence=Q.Reader->Sequence,Stamp=Q.Reader->Timestamp;
                for(const auto& Before:Result->Accepted)if(Before.Reader==Q.Reader)
                {Sequence=Before.Observation.LocalSequence;Stamp=Before.Observation.OsTimestampUs;if(Before.Identity==Identity)Duplicate=true;}
                if(Duplicate)continue;
                FAccepted A;const auto H=Decode(Q.Reader,Q.Reading.Get(),Sequence+1,A);
                if(FAILED(H)||A.Observation.OsTimestampUs<Stamp)return Fail(EStatus::Rejected,FAILED(H)?H:E_UNEXPECTED);
                Batch.Observations.push_back(A.Observation);Result->Accepted.push_back(std::move(A));
            }
            if(Batch.Observations.empty())return {};
        }
        if(Fresh)std::sort(Batch.Observations.begin(),Batch.Observations.end(),
            [](const auto& A,const auto& B){return A.Source<B.Source;});
        const auto Ticket=Source.CapturePump();
        if(!Ticket)return Fail(EStatus::Rejected,E_UNEXPECTED);
        auto Prepared=Source.Prepare(*Ticket,Batch);
        if(Prepared.Status!=EDesktopAdmission::Ready||!Prepared.Prepared)
            return Fail(Prepared.Status==EDesktopAdmission::Capacity?EStatus::Unavailable:EStatus::Rejected,E_UNEXPECTED);
        // Retain an immutable downstream DTO independently of the portable
        // prepared handle, whose internal publication swaps during CommitWith.
        Result->Publication=std::make_shared<const FDesktopPublication>(*Prepared.Prepared->Inspect());
        if(!ValidateDesktopPublication(*Result->Publication))return Fail(EStatus::Rejected,E_UNEXPECTED);
        for(const auto& C:Result->Publication->Changes)
            if(C.Cause==EIngressCause::Hardware&&C.Origin.Control.Kind==ERawControlKind::MouseButton&&C.Value==1.f)Result->Activity=true;
        Result->Source=std::move(Prepared.Prepared);
        return {EStatus::Prepared,std::move(Result),S_OK};
    }
    // Sink is fully prepared: no allocation, catalogue/source reentry or
    // gameplay callback. One lock order: discovery -> collector -> source -> sink.
    template<class TInstall> bool Commit(FPrepared& Prepared,TInstall&& Install)
    {
        static_assert(std::is_nothrow_invocable_r_v<bool,TInstall>,"desktop sink must be noexcept bool");
        if(!IsOwner()||Stopping||Prepared.Owner!=this||Prepared.Registry!=RegistryRevision||!Prepared.Source)return false;
        return Discovery->CommitDesktop(Prepared.Catalogue,[&]()noexcept
        {
            std::lock_guard<std::mutex> Lock(Gate);
            if(CallbackInvalid||Overflow||Prepared.Registry!=RegistryRevision)return false;
            const auto Status=Source.CommitWith(*Prepared.Source,std::forward<TInstall>(Install));
            if(Status!=V2::EDesktopAdmission::Ready)return false;
            for(auto& A:Prepared.Accepted)
            {
                A.Reader->Sequence=A.Observation.LocalSequence;A.Reader->Timestamp=A.Observation.OsTimestampUs;
                A.Reader->Admitted=true;
                if(!Seen(*A.Reader,A.Identity.Get()))
                {
                    // Storage was reserved at reader creation. Erase only old
                    // accepted identities; an unaccepted callback is never evicted.
                    if(A.Reader->Seen.size()==SeenCapacity)A.Reader->Seen.erase(A.Reader->Seen.begin());
                    A.Reader->Seen.push_back(std::move(A.Identity));
                }
            }
            for(auto I=Pending.begin();I!=Pending.end();)
            {
                if(std::find(Prepared.Consumed.begin(),Prepared.Consumed.end(),I->Arrival)!=Prepared.Consumed.end())I=Pending.erase(I);else ++I;
            }
            NeedsFresh=false;RecoverNotices=false;Prepared.Owner=nullptr;return true;
        });
    }
    bool Close()
    {
        if(!IsOwner())return false;
        {std::lock_guard<std::mutex> Lock(Gate);Stopping=true;}
        for(const auto& R:Readers)if(!StopReader(*R))return false;
        std::lock_guard<std::mutex> Lock(Gate);Pending.clear();Readers.clear();return true;
    }
private:
    static constexpr std::size_t QueueCapacity=256,SeenCapacity=512;
    struct FReader
    {
        FGameInputDesktopSource* Owner=nullptr;
        FGameInputDiscovery::FDesktopLease Lease;
        V2::FDesktopSubsource Source;
        std::uint64_t Sequence=0,Timestamp=0;
        GameInput::v3::GameInputCallbackToken Token=0;
        bool Registered=false,Stopped=false,Admitted=false;
        std::uint32_t MouseButtons=0;
        TComPtr<IUnknown> DeviceIdentity;
        std::weak_ptr<FReader> Self;
        std::vector<TComPtr<IUnknown>> Seen;
    };
    FGameInputDesktopSource(FApi* In,FGameInputDiscovery* D,std::uint64_t Session)
        :Api(In),Discovery(D),Source(Session,Session),Thread(std::this_thread::get_id()){}
    bool IsOwner()const{return Thread==std::this_thread::get_id();}
    static FPreparation Fail(EStatus S,HRESULT H){return {S,{},H};}
    static GameInput::v3::GameInputKind Kind(const FReader& R)
    {return R.Source.Kind==V2::EDesktopSubsourceKind::Keyboard?GameInput::v3::GameInputKindKeyboard:GameInput::v3::GameInputKindMouse;}
    static bool Seen(const FReader& R,IUnknown* Identity)
    {for(const auto& I:R.Seen)if(I.Get()==Identity)return true;return false;}
    bool SameCatalogue(const FGameInputDiscovery::FDesktopSnapshot& S)const
    {
        if(S.Revision!=Captured.Revision||S.Devices.size()!=Readers.size())return false;
        for(const auto& L:S.Devices)
        {
            bool Found=false;for(const auto& R:Readers)
                if(R->Lease.Id==L.Id&&R->Lease.Kind==L.Kind&&R->Lease.Revision==L.Revision&&R->Lease.Device==L.Device)Found=true;
            if(!Found)return false;
        }
        return true;
    }
    bool StopReader(FReader& R)
    {
        {std::lock_guard<std::mutex> Lock(Gate);R.Stopped=true;}
        // SDK unregister joins active callbacks. Gate must never be held here.
        if(R.Registered&&!Api->UnregisterCallback(R.Token)){LastError=E_FAIL;return false;}
        R.Registered=false;return true;
    }
    bool RefreshReaders(const FGameInputDiscovery::FDesktopSnapshot& S)
    {
        using namespace V2;SetupStatus=EStatus::Rejected;
        if(RegistryRevision==UINT64_MAX){LastError=E_UNEXPECTED;return false;}++RegistryRevision;
        std::vector<std::shared_ptr<FReader>> Next;
        for(const auto& Old:Readers)
        {
            bool Retained=false;
            for(const auto& L:S.Devices)if(Old->Lease.Id==L.Id&&Old->Lease.Kind==L.Kind&&Old->Lease.Revision==L.Revision&&Old->Lease.Device==L.Device)Retained=true;
            if(!Retained)
            {
                if(!StopReader(*Old)){SetupStatus=EStatus::Unavailable;return false;}
                bool Replaced=false;for(const auto& L:S.Devices)if(L.Id==Old->Lease.Id&&L.Kind==Old->Lease.Kind)Replaced=true;
                if(!Source.EnqueueLifecycle({Old->Source,Replaced?EIngressCause::Replacement:EIngressCause::Disconnect}))RecoverNotices=true;
            }
        }
        {
            std::lock_guard<std::mutex> Lock(Gate);
            for(auto I=Pending.begin();I!=Pending.end();)if(I->Reader->Stopped)I=Pending.erase(I);else ++I;
        }
        for(const auto& L:S.Devices)
        {
            std::shared_ptr<FReader> R;
            for(const auto& Old:Readers)if(!Old->Stopped&&Old->Lease.Id==L.Id&&Old->Lease.Kind==L.Kind&&Old->Lease.Revision==L.Revision&&Old->Lease.Device==L.Device)R=Old;
            if(!R)
            {
                R=std::make_shared<FReader>();R->Owner=this;R->Lease=L;R->Self=R;R->Seen.reserve(SeenCapacity);
                auto I=Ids.find(L.Id);if(I==Ids.end())
                {
                    if(Ids.size()==FDeviceDiscovery::Capacity||NextId==UINT64_MAX){LastError=E_OUTOFMEMORY;return false;}
                    I=Ids.emplace(L.Id,++NextId).first;
                }
                R->Source={L.Kind,I->second,0};
                if(FAILED(L.Device.As(&R->DeviceIdentity))){LastError=E_UNEXPECTED;return false;}
                if(L.Kind==EDesktopSubsourceKind::Mouse)
                {
                    const GameInput::v3::GameInputDeviceInfo* Info=nullptr;
                    LastError=L.Device->GetDeviceInfo(&Info);
                    if(FAILED(LastError)||!Info||!Info->mouseInfo){LastError=E_UNEXPECTED;return false;}
                    R->MouseButtons=static_cast<std::uint32_t>(Info->mouseInfo->supportedButtons);
                }
                Next.push_back(R);
                Readers.push_back(R); // retain even if a later setup step fails
                LastError=Api->RegisterReadingCallback(L.Device.Get(),Kind(*R),R.get(),&OnReading,&R->Token);
                if(FAILED(LastError))
                {SetupStatus=LastError==E_INVALIDARG?EStatus::Rejected:EStatus::Unavailable;return false;}
                R->Registered=true;
            }
            else Next.push_back(R);
            if(!R->Admitted||RecoverNotices)
            {
                if(Generation==UINT64_MAX){LastError=E_UNEXPECTED;return false;}
                R->Source.Generation=++Generation;
            }
        }
        Readers=std::move(Next);
        std::sort(Readers.begin(),Readers.end(),[](const auto& A,const auto& B){return A->Source<B->Source;});
        return true;
    }
    static void CALLBACK OnReading(GameInput::v3::GameInputCallbackToken,void* Context,FReading* Reading)
    {
        auto& R=*static_cast<FReader*>(Context);auto& Self=*R.Owner;
        std::lock_guard<std::mutex> Lock(Self.Gate);
        if(Self.Stopping||R.Stopped||Self.Overflow)return;
        const auto Retained=R.Self.lock();
        if(!Reading||!Retained||Self.Arrival==UINT64_MAX){Self.CallbackInvalid=true;return;}
        if(Self.Pending.size()==QueueCapacity){Self.Overflow=true;return;}
        try{Self.Pending.push_back({Retained,TComPtr<FReading>(Reading),++Self.Arrival});}
        catch(...){Self.Overflow=true;}
    }
    HRESULT Decode(const std::shared_ptr<FReader>& R,FReading* Reading,std::uint64_t Sequence,FAccepted& Out)const
    {
        using namespace V2;using namespace GameInput::v3;
        if(!Reading||!Sequence||!(Reading->GetInputKind()&Kind(*R)))return E_UNEXPECTED;
        TComPtr<IGameInputDevice> Device;Reading->GetDevice(Device.GetAddressOf());
        TComPtr<IUnknown> DeviceIdentity,Identity;
        if(!Device||FAILED(Device.As(&DeviceIdentity))||DeviceIdentity!=R->DeviceIdentity||FAILED(Reading->QueryInterface(IID_PPV_ARGS(Identity.GetAddressOf()))))return E_UNEXPECTED;
        FDesktopFullObservation O;O.Source=R->Source;O.LocalSequence=Sequence;O.OsTimestampUs=Reading->GetTimestamp();
        if(R->Admitted&&O.OsTimestampUs<R->Timestamp)return E_UNEXPECTED;
        if(R->Source.Kind==EDesktopSubsourceKind::Keyboard)
        {
            std::array<GameInputKeyState,256> Keys{};const auto Count=Reading->GetKeyCount();
            if(Count>Keys.size()||Reading->GetKeyState(static_cast<std::uint32_t>(Keys.size()),Keys.data())!=Count)return E_UNEXPECTED;
            FDeviceState Raw;Raw.KeyCount=Count;for(std::size_t I=0;I<Count;++I)Raw.ScanCodes[I]=Keys[I].scanCode;
            FCanonicalDeviceState State;if(!Canonicalize(Raw,ERawDeviceKind::Keyboard,State))return E_UNEXPECTED;
            O.State.assign(State.Values.begin(),State.Values.begin()+State.Count);
        }
        else
        {
            GameInputMouseState State{};if(!Reading->GetMouseState(&State))return E_UNEXPECTED;
            const GameInputMouseButtons Bits[]={GameInputMouseLeftButton,GameInputMouseRightButton,GameInputMouseMiddleButton};
            static_assert(sizeof(Bits)/sizeof(Bits[0])==static_cast<std::size_t>(EMouseButton::Count));
            for(std::uint16_t I=0;I<std::uint16_t(EMouseButton::Count);++I)if(R->MouseButtons&Bits[I])
                O.State.push_back({{ERawControlKind::MouseButton,I},(State.buttons&Bits[I])?1.f:0.f});
            if(O.State.empty())return E_UNEXPECTED;
        }
        Out={R,std::move(O),std::move(Identity)};return S_OK;
    }
    TComPtr<FApi> Api;FGameInputDiscovery* Discovery;
    V2::FDesktopInputSource Source;const std::thread::id Thread;
    FGameInputDiscovery::FDesktopSnapshot Captured;
    std::vector<std::shared_ptr<FReader>> Readers;
    std::map<FDeviceId,std::uint64_t> Ids;
    std::mutex Gate;std::deque<FQueued> Pending;
    std::uint64_t NextId=0,Generation=0,RegistryRevision=0,Arrival=0;
    HRESULT LastError=S_OK;
    EStatus SetupStatus=EStatus::Rejected;
    bool NeedsFresh=true,RecoverNotices=false,Overflow=false,CallbackInvalid=false,Stopping=false;
};
}
