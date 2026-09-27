#define SPEED_DESKTOP_INPUT_PROBE 1
#include "IAmSpeed/Input/DesktopInputSource.h"
#include <cstdlib>
#include <iostream>
#include <thread>

using namespace Speed::Input::V2;
namespace Speed::Input::V2
{
// White-box arithmetic boundary setup only. No alternative acceptance path.
struct FDesktopInputSourceTestAccess
{
    static void ExhaustSequence(FDesktopInputSource& S) { S.State.Sequence = UINT64_MAX; }
    static void ExhaustObservation(FDesktopInputSource& S) { S.State.Observation = UINT64_MAX; }
};
}
static unsigned Checks = 0;
static void Check(bool V, const char* What)
{ ++Checks; if (!V) { std::cerr << "FAIL " << What << '\n'; std::exit(1); } }
static FDesktopSubsource Key(std::uint64_t Id = 1, std::uint64_t Generation = 1)
{ return {EDesktopSubsourceKind::Keyboard, Id, Generation}; }
static FDesktopSubsource Mouse(std::uint64_t Id = 1, std::uint64_t Generation = 2)
{ return {EDesktopSubsourceKind::Mouse, Id, Generation}; }
static FDesktopFullObservation Read(FDesktopSubsource S, std::uint64_t Local, bool Held,
    std::uint64_t Timestamp = 0)
{
    FRawControl C = S.Kind == EDesktopSubsourceKind::Keyboard
        ? FRawControl{ERawControlKind::KeyboardUsage, 4} : FRawControl{ERawControlKind::MouseButton, 0};
    return {S, Local, Timestamp, {{C, Held ? 1.0f : 0.0f}}};
}
static FDesktopBatch Fresh(std::initializer_list<FDesktopFullObservation> Reads)
{ FDesktopBatch B; B.Mode = EDesktopBatchMode::FreshBaseline; B.Observations = Reads; return B; }
static FDesktopBatch Poll(std::initializer_list<FDesktopFullObservation> Reads)
{ FDesktopBatch B; B.Observations = Reads; return B; }
static std::shared_ptr<const FDesktopPublication> Commit(FDesktopInputSource& S, const FDesktopBatch& B)
{
    const auto T = S.CapturePump(); Check(T.has_value(), "owner captures ticket");
    auto P = S.Prepare(*T, B); Check(P.Status == EDesktopAdmission::Ready, "prepare accepted");
    Check(P.Prepared->Inspect() != nullptr, "prepared publication inspectable before commit");
    Check(S.Commit(*P.Prepared) == EDesktopAdmission::Ready, "commit accepted");
    Check(P.Prepared->Inspect() == nullptr, "consumed token cannot expose old or null publication as a reference");
    Check(S.Commit(*P.Prepared) == EDesktopAdmission::Stale, "prepared token single use");
    return S.ReadPublished();
}
static void RejectUnchanged(FDesktopInputSource& S, FDesktopBatch B, EDesktopAdmission Expected)
{
    const auto Before = S.ReadPublished();
    const auto P = S.Prepare(*S.CapturePump(), B);
    Check(P.Status == Expected && !P.Prepared, "batch rejected explicitly");
    Check(S.ReadPublished() == Before, "rejected preparation preserves published identity");
}
// Test-only canonical serializer, independent of object layout/padding. This
// verifies deterministic provenance bytes, not production replay admission.
static std::vector<std::uint8_t> Bytes(const FDesktopPublication& P)
{
    std::vector<std::uint8_t> Out;
    auto U = [&](std::uint64_t V) { for (unsigned I = 0; I < 8; ++I) Out.push_back(static_cast<std::uint8_t>(V >> (8 * I))); };
    auto Source = [&](FDesktopSubsource S) { U(static_cast<std::uint64_t>(S.Kind)); U(S.DeviceId); U(S.Generation); };
    auto Control = [&](FRawControl C) { U(static_cast<std::uint64_t>(C.Kind)); U(C.Code); };
    auto Value = [&](FRawValue V) { Control(V.Control); U(V.Value == 1 ? 1 : 0); };
    U(P.Schema); U(P.Ordering); U(P.Fingerprint);
    U(P.Baseline.ProducerId); U(P.Baseline.StreamEpoch); U(P.Baseline.CompositeGeneration);
    U(P.Baseline.BaselineRevision); U(P.Baseline.AcceptedThrough);
    U(P.Baseline.Members.size()); for (const auto& M : P.Baseline.Members) Source(M);
    U(P.Baseline.Contributions.size()); for (const auto& C : P.Baseline.Contributions)
    { Source(C.Origin.Source); Value({C.Origin.Control, C.Value}); U(C.LastGlobalSequence); }
    U(P.Changes.size()); for (const auto& C : P.Changes)
    {
        Source(C.Origin.Source); Value({C.Origin.Control, C.Value}); U(C.GlobalSequence);
        U(C.LocalSequence); U(C.OsTimestampUs); U(C.ObservationGroup); U(C.WithinObservation); U(static_cast<std::uint64_t>(C.Cause));
    }
    U(P.Observations.size()); for (const auto& O : P.Observations)
    { Source(O.Source); U(O.LocalSequence); U(O.OsTimestampUs); U(O.ObservationGroup); U(O.AcceptanceSequence); U(O.FirstChange); U(O.ChangeCount); }
    U(P.RawTerminals.size()); for (const auto& T : P.RawTerminals)
    { Source(T.Origin.Source); Control(T.Origin.Control); U(T.ActivationSequence); U(T.TerminalSequence); U(static_cast<std::uint64_t>(T.Cause)); }
    U(P.PreviousUnion.size()); for (const auto& V : P.PreviousUnion) Value(V);
    U(P.Sample.DeviceId); U(P.Sample.Generation.Value); U(static_cast<std::uint64_t>(P.Sample.Kind));
    U(P.Sample.Sequence); U(static_cast<std::uint64_t>(P.Sample.Status));
    U(P.Sample.FinalState.size()); for (const auto& V : P.Sample.FinalState) Value(V);
    U(P.Sample.Changes.size()); for (const auto& C : P.Sample.Changes)
    { Value(C.State); U(C.Order.Sequence); U(C.Order.WithinSequence); U(C.AtomicGroup); }
    U(P.RequiresLifecycleEnvelope); U(P.OverflowRecovery); U(P.LifecycleWatermark); U(P.RejectedWatermark);
    return Out;
}
int main()
{
    {
        FDesktopInputSource S(80, 1);
        Commit(S, Fresh({Read(Key(1, 1), 1, true)}));
        Check(S.EnqueueLifecycle({Key(1, 2), EIngressCause::Disconnect}), "newer disconnected generation supersedes old live member");
        RejectUnchanged(S, Fresh({Read(Key(1, 1), 2, true)}), EDesktopAdmission::Stale);
        FDesktopBatch Pause; Pause.Mode = EDesktopBatchMode::Pause;
        const auto P = Commit(S, Pause);
        Check(P->Baseline.Members.empty() && P->RawTerminals.size() == 1
            && P->RawTerminals[0].Cause == EIngressCause::Disconnect,
            "captured device lifecycle cause precedes pause and invalidates older generation");
        RejectUnchanged(S, Fresh({Read(Key(1, 1), 3, true)}), EDesktopAdmission::Stale);
        const auto New = Commit(S, Fresh({Read(Key(1, 3), 1, true)}));
        Check(New->Sample.FinalState[0].Value == 1 && New->Changes.empty(), "fresh later generation resumes held without a press");
    }
    {
        FDesktopInputSource A(8, 3), B(8, 3);
        const auto Baseline = Fresh({Read(Key(), 1, false), Read(Mouse(), 1, false)});
        Check(Bytes(*Commit(A, Baseline)) == Bytes(*Commit(B, Baseline)), "same baseline serializes identically");
        const auto Events = Poll({Read(Key(), 2, true, 7), Read(Mouse(), 2, true, 7), Read(Mouse(), 3, false, 1)});
        const auto PA = Commit(A, Events), PB = Commit(B, Events);
        Check(Bytes(*PA) == Bytes(*PB), "same accepted history produces identical canonical provenance bytes");
        Check(PA->Changes[0].OsTimestampUs == PA->Changes[1].OsTimestampUs
            && PA->Changes[0].GlobalSequence < PA->Changes[1].GlobalSequence, "equal timestamps retain fixed member order");
        Check(A.EnqueueLifecycle({Mouse(), EIngressCause::Disconnect}) && B.EnqueueLifecycle({Mouse(), EIngressCause::Disconnect}), "same lifecycle enqueue");
        const auto Replace = Fresh({Read(Key(), 3, true), Read(Mouse(1, 3), 1, true)});
        Check(Bytes(*Commit(A, Replace)) == Bytes(*Commit(B, Replace)), "atomic registry replacement serializes identically");
    }
    {
        FDesktopInputSource S(9, 42);
        RejectUnchanged(S, Poll({}), EDesktopAdmission::NeedsFreshBaseline);
        auto P = Commit(S, Fresh({Read(Key(), 1, false), Read(Mouse(), 1, false)}));
        Check(P->RequiresLifecycleEnvelope && P->Changes.empty() && P->Sample.Changes.empty(), "baseline has no synthetic edges");
        Check(P->Sample.IsValid() && P->Sample.Kind == ERawDeviceKind::Desktop, "desktop sample structurally valid");
        Check(P->Baseline.StreamEpoch == 42 && P->Baseline.ProducerId == 9, "epoch and producer bound");
        P = Commit(S, Poll({Read(Key(), 2, true, 100), Read(Mouse(), 2, true, 2), Read(Mouse(), 3, false, 1)}));
        Check(P->Changes.size() == 3 && P->Sample.Changes.size() == 3, "short click and keyboard retained");
        Check(P->Changes[0].Origin.Source.Kind == EDesktopSubsourceKind::Keyboard
            && P->Changes[1].OsTimestampUs == 2 && P->Changes[2].OsTimestampUs == 1, "owner order ignores inverted timestamps");
        Check(P->Observations[0].ObservationGroup < P->Observations[1].ObservationGroup
            && P->Observations[1].ObservationGroup < P->Observations[2].ObservationGroup, "observation identities monotone across devices");
        Check(P->Changes[1].GlobalSequence < P->Changes[2].GlobalSequence, "ingress watermark orders click");
        const auto LastGroup = P->Observations.back().ObservationGroup;
        const auto LastWatermark = P->Baseline.AcceptedThrough;
        P = Commit(S, Poll({Read(Key(), 3, true)}));
        Check(P->Changes.empty() && P->Observations.size() == 1 && P->Sample.Changes.empty(), "unchanged observation retains provenance only");
        Check(P->Observations[0].ObservationGroup > LastGroup && P->Baseline.AcceptedThrough > LastWatermark, "nochange advances markers");
        auto Bad = Poll({Read(Mouse(), 4, false), Read(Key(), 4, true)});
        RejectUnchanged(S, Bad, EDesktopAdmission::Invalid);
        RejectUnchanged(S, Poll({Read(Key(), 3, true)}), EDesktopAdmission::Stale);
        RejectUnchanged(S, Poll({Read(Key(), 5, true)}), EDesktopAdmission::Stale);
        Bad = Poll({Read(Key(), 4, true)}); Bad.Ordering = 2;
        RejectUnchanged(S, Bad, EDesktopAdmission::Invalid);
        Bad.Ordering = 1; Bad.Schema = 2; RejectUnchanged(S, Bad, EDesktopAdmission::Invalid);
        Bad.Schema = 1; Bad.Fingerprint ^= 1; RejectUnchanged(S, Bad, EDesktopAdmission::Invalid);
        Bad = Poll({Read(Key(), 4, true)}); Bad.Observations[0].State[0].Control.Kind = ERawControlKind::MouseButton;
        RejectUnchanged(S, Bad, EDesktopAdmission::Invalid);
        Bad = Poll({Read(Mouse(), 4, false)}); Bad.Observations[0].State[0].Control.Code = 3;
        RejectUnchanged(S, Bad, EDesktopAdmission::Invalid);
        Bad = Poll({Read(Key(), 4, true)}); Bad.Observations[0].State[0].Value = 0.5f;
        RejectUnchanged(S, Bad, EDesktopAdmission::Invalid);
        Bad = Poll({Read(Key(), 4, true)}); Bad.Observations[0].State.push_back(Bad.Observations[0].State.front());
        RejectUnchanged(S, Bad, EDesktopAdmission::Invalid);
    }
    {
        FDesktopInputSource S(10, 1);
        Commit(S, Fresh({Read(Key(1, 1), 1, false), Read(Key(2, 2), 1, false)}));
        auto P = Commit(S, Poll({Read(Key(1, 1), 2, true), Read(Key(2, 2), 2, true)}));
        Check(P->Changes.size() == 2 && P->Sample.Changes.size() == 1, "second contribution creates no aggregate raw press");
        P = Commit(S, Poll({Read(Key(1, 1), 3, false)}));
        Check(P->Changes.size() == 1 && P->Sample.Changes.empty() && P->Sample.FinalState[0].Value == 1, "OR keeps other keyboard hold");
        Check(S.EnqueueLifecycle({Key(2, 2), EIngressCause::Disconnect}), "disconnect queued");
        auto B = Fresh({Read(Key(1, 1), 4, false), Read(Key(2, 3), 1, true)});
        const auto Old = S.ReadPublished();
        B.RawTerminalCapacity = 0; RejectUnchanged(S, B, EDesktopAdmission::Capacity);
        B.RawTerminalCapacity = 1; P = Commit(S, B);
        Check(P->RawTerminals.size() == 1 && P->RawTerminals[0].Origin.Source == Key(2, 2), "departing origin terminal retained");
        Check(P->RawTerminals[0].Cause == EIngressCause::Disconnect && P->RawTerminals[0].ActivationSequence != 0, "terminal attributable to raw activation");
        Check(P->PreviousUnion[0].Value == 1 && P->Sample.FinalState[0].Value == 1
            && P->Sample.Changes.empty() && P->RequiresLifecycleEnvelope, "atomic held replacement without intermediate raw edges");
        Check(Old->Baseline.CompositeGeneration + 1 == P->Baseline.CompositeGeneration, "membership advances generation");
        RejectUnchanged(S, Poll({Read(Key(2, 2), 3, true)}), EDesktopAdmission::Stale);
        RejectUnchanged(S, Fresh({Read(Key(2, 2), 4, true)}), EDesktopAdmission::Stale);
        Check(S.EnqueueLifecycle({Key(2, 3), EIngressCause::Disconnect}), "second disconnect queued");
        P = Commit(S, Fresh({Read(Key(1, 1), 5, false)}));
        Check(P->PreviousUnion[0].Value == 1 && P->Sample.FinalState[0].Value == 0
            && P->RawTerminals.size() == 1, "final union neutral and terminal one");
    }
    {
        FDesktopInputSource S(11, 1);
        auto P = Commit(S, Fresh({Read(Key(), 1, true), Read(Mouse(), 1, true)}));
        FDesktopBatch Pause; Pause.Mode = EDesktopBatchMode::Pause; Pause.RawTerminalCapacity = 1;
        RejectUnchanged(S, Pause, EDesktopAdmission::Capacity);
        Pause.RawTerminalCapacity = 2; P = Commit(S, Pause);
        Check(P->RawTerminals.size() == 2 && P->RawTerminals[0].Cause == EIngressCause::Pause, "pause closes both baseline activations");
        Check(P->Sample.FinalState[0].Value == 0 && P->Sample.FinalState[1].Value == 0, "pause neutral union");
        P = Commit(S, Pause); Check(P->RawTerminals.empty(), "repeated pause never duplicates terminals");
        RejectUnchanged(S, Poll({Read(Key(), 2, true)}), EDesktopAdmission::NeedsFreshBaseline);
        P = Commit(S, Fresh({Read(Key(), 2, true), Read(Mouse(), 2, true)}));
        Check(P->Sample.FinalState[0].Value == 1 && P->Changes.empty(), "fresh-held resume immediate without press");
        FDesktopBatch Reset; Reset.Mode = EDesktopBatchMode::Reset;
        P = Commit(S, Reset); Check(P->RawTerminals.size() == 2 && P->RawTerminals[0].Cause == EIngressCause::Reset, "reset terminal cause");
        FDesktopBatch Retire; Retire.Mode = EDesktopBatchMode::Retire;
        Commit(S, Retire);
        RejectUnchanged(S, Fresh({Read(Key(), 3, false)}), EDesktopAdmission::Retired);
    }
    {
        FDesktopInputSource S(12, 1);
        Commit(S, Fresh({Read(Key(), 1, true)}));
        const auto Ticket = *S.CapturePump();
        auto Prepared = S.Prepare(Ticket, Poll({Read(Key(), 2, false)}));
        Check(Prepared.Status == EDesktopAdmission::Ready, "staged before lifecycle event");
        const auto Before = S.ReadPublished();
        Check(S.EnqueueLifecycle({Key(), EIngressCause::Disconnect}), "late lifecycle accepted");
        Check(S.Commit(*Prepared.Prepared) == EDesktopAdmission::Stale && S.ReadPublished() == Before, "late generation notice prevents partial commit");
        for (std::size_t I = 1; I < FDesktopInputSource::MaxNotices; ++I)
            Check(S.EnqueueLifecycle({Key(99, 99), EIngressCause::Disconnect}), "bounded queue fills");
        Check(!S.EnqueueLifecycle({Key(100, 100), EIngressCause::Disconnect}), "queue overflow signaled");
        Check(!S.ReadPublished(), "overflow cannot expose old snapshot as valid");
        auto B = Fresh({Read(Key(2, 102), 1, true)});
        auto Result = S.Prepare(*S.CapturePump(), B);
        Check(Result.Status == EDesktopAdmission::OverflowFence, "ordinary fresh read cannot clear overflow");
        B.RecoverOverflow = true;
        Result = S.Prepare(*S.CapturePump(), B); Check(Result.Status == EDesktopAdmission::Ready, "explicit fresh enumeration prepares recovery");
        Check(!S.EnqueueLifecycle({Key(101, 101), EIngressCause::Disconnect}), "late failed enqueue still fences recovery");
        Check(S.Commit(*Result.Prepared) == EDesktopAdmission::Stale, "recovery cannot lose post-capture overflow");
        auto P = Commit(S, B);
        Check(P->OverflowRecovery && P->RejectedWatermark && P->RawTerminals.size() == 1, "overflow receipt retains rejected watermark and held terminal");
    }
    {
        FDesktopInputSource S(13, 1);
        Commit(S, Fresh({Read(Key(), 1, false)}));
        auto First = S.Prepare(*S.CapturePump(), Poll({Read(Key(), 2, true)}));
        auto Second = S.Prepare(*S.CapturePump(), Poll({Read(Key(), 2, false)}));
        Check(S.Commit(*First.Prepared) == EDesktopAdmission::Ready, "first competing preparation wins");
        Check(S.Commit(*Second.Prepared) == EDesktopAdmission::Stale, "second competing preparation rejected");
        FDesktopInputSource Other(14, 1);
        auto Foreign = S.Prepare(*S.CapturePump(), Poll({Read(Key(), 3, false)}));
        Check(Other.Commit(*Foreign.Prepared) == EDesktopAdmission::Stale, "foreign prepared owner rejected");
        const auto Ticket = *S.CapturePump();
        bool Wrong = false;
        std::thread T([&]() { Wrong = !S.CapturePump() && S.Prepare(Ticket, Poll({})).Status == EDesktopAdmission::WrongOwner; });
        T.join(); Check(Wrong, "nonowner cannot capture or prepare ingress");
        // No commit: dropping an allocated preparation is an abort with no mutation.
        const auto Before = S.ReadPublished(); Foreign.Prepared.reset(); Check(S.ReadPublished() == Before, "abort preparation has no partial state");
    }
    {
        FDesktopInputSource S(15, 1);
        Commit(S, Fresh({Read(Key(), 1, false)}));
        FDesktopBatch B;
        for (std::uint64_t I = 0; I < 256; ++I) B.Observations.push_back(Read(Key(), I + 2, (I % 2) == 0));
        auto P = Commit(S, B); Check(P->Changes.size() == 256 && P->Sample.Changes.size() == 256, "256 edges admitted intact");
        B.Observations.push_back(Read(Key(), 258, true));
        RejectUnchanged(S, B, EDesktopAdmission::Capacity);
        FDesktopBatch TooMany; TooMany.Mode = EDesktopBatchMode::FreshBaseline;
        for (std::uint64_t I = 0; I < 17; ++I) TooMany.Observations.push_back(Read(Key(I + 10, I + 10), 1, false));
        RejectUnchanged(S, TooMany, EDesktopAdmission::Capacity);
    }
    {
        FDesktopInputSource S(16, 1);
        auto Base = Read(Key(), 1, false); Base.State.clear();
        for (std::uint16_t Code = 4; Code < 133; ++Code) Base.State.push_back({{ERawControlKind::KeyboardUsage, Code}, 0});
        Commit(S, Fresh({Base}));
        auto On = Base; On.LocalSequence = 2;
        for (auto& V : On.State) V.Value = 1;
        auto Off = Base; Off.LocalSequence = 3; Off.State.back().Value = 1;
        RejectUnchanged(S, Poll({On, Off}), EDesktopAdmission::Capacity); // 129 + 128 changes.
        // The rejected prefix did not consume either local observation identity.
        On.State.back().Value = 0; Off.State.back().Value = 0;
        auto P = Commit(S, Poll({On, Off}));
        Check(P->Changes.size() == 256, "retry after full-batch overflow retained old cursors");
        Check(P->Changes[127].WithinObservation == 127 && P->Changes[128].WithinObservation == 0, "within-observation order restarts exactly once");
    }
    Check(FDesktopInputSource::CanAdvanceSequence(UINT64_MAX - 1), "last available identity admitted");
    Check(!FDesktopInputSource::CanAdvanceSequence(UINT64_MAX), "sequence cannot wrap to zero");
    Check(!FDesktopInputSource::CanAdvanceSequence(UINT64_MAX - 1, 2), "count arithmetic cannot overflow");
    {
        FDesktopInputSource S(19, 1);
        Commit(S, Fresh({Read(Key(), 1, true)}));
        Check(S.EnqueueLifecycle({Key(), EIngressCause::Disconnect}), "disconnect before pause queued");
        FDesktopBatch Pause; Pause.Mode = EDesktopBatchMode::Pause;
        const auto P = Commit(S, Pause);
        Check(P->RawTerminals.size() == 1 && P->Baseline.Members.empty(), "pause drains terminal and invalidated membership together");
        RejectUnchanged(S, Fresh({Read(Key(), 2, true)}), EDesktopAdmission::Stale);
        Commit(S, Fresh({Read(Key(1, 2), 1, true)}));
    }
    {
        FDesktopInputSource S(20, 1);
        Commit(S, Fresh({Read(Key(), 1, false)}));
        Check(S.EnqueueLifecycle({Mouse(1, 8), EIngressCause::Disconnect}), "unadmitted generation departure queued");
        FDesktopBatch Pause; Pause.Mode = EDesktopBatchMode::Pause;
        Commit(S, Pause);
        RejectUnchanged(S, Fresh({Read(Key(), 2, false), Read(Mouse(1, 8), 1, true)}), EDesktopAdmission::Stale);
        Commit(S, Fresh({Read(Key(), 2, false), Read(Mouse(1, 9), 1, true)}));
    }
    {
        FDesktopInputSource S(17, 1);
        Commit(S, Fresh({Read(Key(), 1, true)}));
        auto Pending = S.Prepare(*S.CapturePump(), Poll({Read(Key(), 2, false)}));
        FDesktopInputSourceTestAccess::ExhaustSequence(S);
        auto R = S.Prepare(*S.CapturePump(), Poll({}));
        Check(R.Status == EDesktopAdmission::Exhausted && !S.ReadPublished(), "sequence exhaustion fences previous held publication");
        Check(S.Commit(*Pending.Prepared) == EDesktopAdmission::Exhausted, "exhaustion fences already prepared commit");
        Check(S.Prepare(*S.CapturePump(), Fresh({Read(Key(), 2, false)})).Status == EDesktopAdmission::Exhausted,
            "ordinary baseline cannot unretire exhausted epoch");
    }
    {
        FDesktopInputSource S(18, 1);
        Commit(S, Fresh({Read(Key(), 1, true)}));
        FDesktopInputSourceTestAccess::ExhaustObservation(S);
        Check(S.Prepare(*S.CapturePump(), Poll({Read(Key(), 2, true)})).Status == EDesktopAdmission::Exhausted,
            "unchanged observation cannot wrap group identity");
        Check(!S.ReadPublished(), "observation exhaustion is sticky neutral admission fence");
    }
    {
        std::uint64_t H = 14695981039346656037ULL;
        for (const auto Word : DesktopContractWords) for (unsigned Byte = 0; Byte < 8; ++Byte)
        { H ^= (Word >> (Byte * 8)) & 255; H *= 1099511628211ULL; }
        Check(H == FDesktopPublication::ContractFingerprint, "canonical little-endian contract descriptor fingerprint");
        Check(DesktopContractWords[2] == FDesktopInputSource::MaxMembers
            && DesktopContractWords[3] == FDesktopInputSource::MaxContributions
            && DesktopContractWords[4] == FDesktopInputSource::MaxChanges
            && DesktopContractWords[5] == FDesktopInputSource::MaxUnion
            && DesktopContractWords[6] == FDesktopInputSource::MaxNotices
            && DesktopContractWords[7] == FDesktopInputSource::MaxObservations, "fingerprint descriptor carries actual bounds");
    }
    std::cout << "PASS DesktopInputSourceProbe " << Checks << " checks (portable foundation only)\n";
}
