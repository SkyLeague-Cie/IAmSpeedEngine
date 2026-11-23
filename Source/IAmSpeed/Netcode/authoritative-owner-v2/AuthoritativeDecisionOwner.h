#pragma once
#include "../consumed-command-v2/OwnerSnapshotBridge.h"
#include <thread>

namespace Speed::Netcode::Authoritative
{
namespace C = Consumed;
namespace V = Input::V2;
enum class ERole : std::uint8_t { OwnerClient = 1, RemoteClient = 2 };
enum class EPhase : std::uint8_t { Idle, Prepared, Installed, Stepping, Faulted, Exhausted };
struct FReceiverBinding
{
    U64 ReceiverId = 0, ReceiverPeer = 0, WorkerGeneration = 0, LocalEpoch = 0;
    ERole Role = ERole::RemoteClient;
    C::FTrustedContext Origin;
    std::shared_ptr<const V::FInputActionContract> Contract;
    V::FContractFingerprint DestinationFingerprint;
};
struct FRestoreAnchor
{
    U64 Next = 0, TimelineRevision = 0, LocalWorldPublication = 0, LocalWorldHash = 0;
    U64 AuthorityTimelineRevision = 0, AuthorityBeforeWorldPublication = 0;
    C::FWorldIdentity World;
    std::optional<FPacket> PreviousOriginal;
    // This state comes from the complete world+input restore transaction.
    std::optional<std::array<std::int16_t, ActionCount>> AppliedBefore;
};
class IReceiverRestoreEvidence
{
public:
    virtual ~IReceiverRestoreEvidence() = default;
    virtual bool Confirms(const FReceiverBinding&, const FRestoreAnchor&) const noexcept { return false; }
};
// Implemented only by the actual LOCAL complete-world publication ledger. A
// remote authority's completed frame cannot certify a local completed frame.
class ILocalWorldPublication
{
public:
    virtual ~ILocalWorldPublication() = default;
    virtual bool Confirms(U64, U64, U64, U64, const C::FPreparedAuthoritativeInput&) const noexcept { return false; }
};
struct FLocalReceipt
{
    U64 ReceiverId = 0, LocalEpoch = 0, WorkerGeneration = 0, RegistryVersion = 0;
    U64 TimelineRevision = 0, Frame = 0, Serial = 0, LocalWorldPublication = 0;
    std::shared_ptr<const C::FPreparedAuthoritativeInput> AuthoritativeEntry;
};
// Explicit decision endpoint, semantically separate from IInputProducer. It
// neither relabels ExactRemote nor calls FSameFrameInputOwner::Process. This
// ordinary core has no native registry adapter, world publisher or actor writes.
class FPublishedDecisionOwner final
{
public:
    class FToken
    {
        friend class FPublishedDecisionOwner;
        std::shared_ptr<const std::uint8_t> Identity;
        explicit FToken(std::shared_ptr<const std::uint8_t> P) noexcept : Identity(std::move(P)) {}
    public:
        FToken() = default; FToken(const FToken&) = delete; FToken& operator=(const FToken&) = delete;
        FToken(FToken&&) noexcept = default; FToken& operator=(FToken&&) noexcept = default;
    };
    static std::unique_ptr<FPublishedDecisionOwner> Create(const FReceiverBinding& B, const FRestoreAnchor& A,
        const IReceiverRestoreEvidence& Evidence)
    {
        if (Input::FPresentationInputScope::IsActive()) return {};
        try
        {
            auto Binding = std::make_shared<const FReceiverBinding>(B); auto Anchor = std::make_shared<const FRestoreAnchor>(A);
            const auto& Origin = Binding->Origin;
            if (!Binding->ReceiverId || !Binding->ReceiverPeer || !Binding->WorkerGeneration || !Binding->LocalEpoch
                || !Binding->Contract || Binding->DestinationFingerprint.empty() || !Origin.Binding.Valid()
                || !Origin.RegistryVersion || !Origin.AuthorityEpoch || Origin.Frame != Anchor->Next || !Anchor->TimelineRevision
                || !Anchor->World.Valid() || Anchor->World != Origin.World
                || !Anchor->AuthorityTimelineRevision || Anchor->AuthorityTimelineRevision != Origin.AuthorityTimelineRevision
                || !Anchor->AuthorityBeforeWorldPublication || Anchor->AuthorityBeforeWorldPublication != Origin.BeforeWorldPublication
                || !Anchor->LocalWorldPublication || !Anchor->LocalWorldHash || !C::ValidTargets(Origin.Targets)
                || Origin.Actions != C::Policies(*Binding->Contract) || !C::ValidProcessing(Origin.Processing, Origin.Actions)
                || Origin.Processing.BindingFingerprint != C::OriginBindingFingerprintFor(*Binding->Contract, Origin)
                || Anchor->AppliedBefore != Origin.Processing.Before
                || Binding->ReceiverPeer == Origin.Binding.ServerPeer
                || (Binding->Role == ERole::OwnerClient ? Binding->ReceiverPeer != Origin.Binding.OwnerPeer
                    : Binding->Role != ERole::RemoteClient || Binding->ReceiverPeer == Origin.Binding.OwnerPeer)) return {};
            if (Anchor->PreviousOriginal && (!Valid(*Anchor->PreviousOriginal)
                || !Origin.Binding.Matches(*Anchor->PreviousOriginal) || Anchor->PreviousOriginal->Kind != EKind::OwnerInput
                || Anchor->PreviousOriginal->Input.ProducerId != Origin.Binding.SourceProducer
                || Anchor->PreviousOriginal->Input.ProducerKind != Origin.Binding.SourceKind
                || Anchor->PreviousOriginal->Frame == UINT64_MAX || Anchor->PreviousOriginal->Frame + 1 != Anchor->Next)) return {};
            if (Anchor->PreviousOriginal)
            {
                if (!C::OriginalFrameToV2(*Anchor->PreviousOriginal, Binding->Contract)) return {};
                if (Origin.Processing.Origin == C::EOrigin::Device && Anchor->PreviousOriginal->Input.Reset
                    && Anchor->AppliedBefore != std::optional<std::array<std::int16_t, ActionCount>>(Anchor->PreviousOriginal->Input.Values)) return {};
            }
            if (!Evidence.Confirms(*Binding, *Anchor)) return {};
            return std::unique_ptr<FPublishedDecisionOwner>(new FPublishedDecisionOwner(std::move(Binding), std::move(Anchor)));
        }
        catch (...) { return {}; }
    }
    EPhase GetPhase() const noexcept { return Lane() ? Phase : EPhase::Faulted; }
    U64 GetNext() const noexcept { return Lane() ? Next : UINT64_MAX; }
    std::shared_ptr<const FLocalReceipt> ReadLatest() const noexcept { return Lane() && Phase != EPhase::Faulted ? Latest : nullptr; }
    std::shared_ptr<const FLocalReceipt> ReadCompleted(U64 Frame) const noexcept
    { if (!Lane()) return {}; const auto& R = History[Frame % History.size()]; return R && R->Frame == Frame ? R : nullptr; }
    std::optional<FToken> PrepareAuthoritativeAtBoundary(U64 TrustedSender, const C::FDecision& D,
        const C::FTrustedContext& Context, const C::IPublishedWorldEvidence& Evidence, U64 PlannedLocalWorldPublication)
    {
        if (!Lane() || Phase != EPhase::Idle || Pending || Serial == UINT64_MAX || !PlannedLocalWorldPublication
            || PlannedLocalWorldPublication <= LastLocalPublication) return {};
        // Block reentrant mutation while a provider validates origin evidence.
        FCallbackScope Guard(InCallback);
        try
        {
            const auto Trusted = std::make_shared<const C::FTrustedContext>(Context);
            const auto& Expected = Binding->Origin;
            if (Trusted->World != Expected.World || Trusted->AuthorityTimelineRevision != Expected.AuthorityTimelineRevision || Trusted->BeforeWorldPublication != AuthorityBeforePublication
                || Trusted->Frame != Next || Trusted->RegistryVersion != Expected.RegistryVersion || Trusted->AuthorityEpoch != Expected.AuthorityEpoch
                || Trusted->Targets != Expected.Targets || Trusted->Processing.Origin != Expected.Processing.Origin
                || Trusted->Processing.Step != Expected.Processing.Step || Trusted->Processing.BindingFingerprint != Expected.Processing.BindingFingerprint
                || Trusted->Actions != Expected.Actions || !SameBinding(Trusted->Binding, Expected.Binding)
                || Trusted->Processing.Before != Applied) return {};
            auto Entry = C::PrepareLocalDecisionEntry(TrustedSender, D, *Trusted, Evidence, Binding->Contract, Binding->DestinationFingerprint);
            if (!Entry || Entry->AcceptedDecision->Publication.Serial <= AuthorityBeforePublication || !Continuous(Entry->AcceptedDecision->Original)) return {};
            auto Identity = std::make_shared<const std::uint8_t>(std::uint8_t{0});
            auto Receipt = std::make_shared<const FLocalReceipt>(FLocalReceipt{Binding->ReceiverId, Binding->LocalEpoch,
                Binding->WorkerGeneration, Expected.RegistryVersion, Anchor->TimelineRevision, Next, Serial + 1,
                PlannedLocalWorldPublication, std::move(Entry)});
            // All immutable receipt/history storage is allocated before install.
            Pending.emplace(FPending{std::move(Identity), std::move(Receipt)}); Phase = EPhase::Prepared;
            return FToken(Pending->Identity);
        }
        catch (...) { return {}; }
    }
    bool Install(const FToken& T) noexcept { if (!Allowed(T, EPhase::Prepared)) return false; Phase = EPhase::Installed; return true; }
    std::shared_ptr<const C::FPreparedAuthoritativeInput> ReadInstalled(const FToken& T) const noexcept
    { return Lane() && Matches(T) && (Phase == EPhase::Installed || Phase == EPhase::Stepping) ? Pending->Receipt->AuthoritativeEntry : nullptr; }
    bool BeginStep(const FToken& T, U64 Frame) noexcept
    { if (!Allowed(T, EPhase::Installed) || Frame != Next) return false; Phase = EPhase::Stepping; return true; }
    bool CompleteStepAfterWorldPublication(const FToken& T, const ILocalWorldPublication& Publication) noexcept
    {
        if (!Allowed(T, EPhase::Stepping)) return false;
        const auto Receipt = Pending->Receipt;
        bool Confirmed = false;
        { FCallbackScope Guard(InCallback); Confirmed = Publication.Confirms(Binding->ReceiverId, Receipt->Frame,
            Receipt->TimelineRevision, Receipt->LocalWorldPublication, *Receipt->AuthoritativeEntry); }
        if (!Confirmed) return false; // no local success inferred from origin publication
        Latest = Receipt; History[Next % History.size()] = Receipt; Serial = Receipt->Serial;
        LastLocalPublication = Receipt->LocalWorldPublication; AuthorityBeforePublication = Receipt->AuthoritativeEntry->AcceptedDecision->Publication.Serial; Previous = Receipt->AuthoritativeEntry->AcceptedDecision;
        if (Binding->Origin.Processing.Origin == C::EOrigin::Device) Applied = Previous->Applied;
        Pending.reset(); if (Next == UINT64_MAX) Phase = EPhase::Exhausted; else { ++Next; Phase = EPhase::Idle; }
        return true;
    }
    bool AbortBeforeStep(const FToken& T) noexcept
    {
        if (!Lane() || !Matches(T) || (Phase != EPhase::Prepared && Phase != EPhase::Installed)) return false;
        Pending.reset(); Phase = EPhase::Idle; return true;
    }
    bool FaultAfterStep(const FToken& T) noexcept
    { if (!Allowed(T, EPhase::Stepping)) return false; Pending.reset(); Phase = EPhase::Faulted; return true; }
    // Correction requires a new complete restore anchor/timeline revision. This
    // endpoint intentionally has no "ignore Applied mismatch" or silent rebind.
private:
    struct FCallbackScope { bool& Flag; explicit FCallbackScope(bool& F) noexcept : Flag(F) { Flag = true; } ~FCallbackScope() { Flag = false; } };
    struct FPending { std::shared_ptr<const std::uint8_t> Identity; std::shared_ptr<const FLocalReceipt> Receipt; };
    FPublishedDecisionOwner(std::shared_ptr<const FReceiverBinding> B, std::shared_ptr<const FRestoreAnchor> A)
        : Binding(std::move(B)), Anchor(std::move(A)), Worker(std::this_thread::get_id()), Next(Anchor->Next),
          LastLocalPublication(Anchor->LocalWorldPublication), AuthorityBeforePublication(Anchor->AuthorityBeforeWorldPublication), Applied(Anchor->AppliedBefore) {}
    bool Lane() const noexcept { return Worker == std::this_thread::get_id() && !InCallback && !Input::FPresentationInputScope::IsActive(); }
    bool Matches(const FToken& T) const noexcept { return Pending && T.Identity && T.Identity == Pending->Identity; }
    bool Allowed(const FToken& T, EPhase P) const noexcept { return Lane() && Matches(T) && Phase == P; }
    static bool SameBinding(const FBinding& A, const FBinding& B) noexcept
    { return A.Session == B.Session && A.Epoch == B.Epoch && A.Actor == B.Actor && A.Incarnation == B.Incarnation
        && A.OwnerPeer == B.OwnerPeer && A.ServerPeer == B.ServerPeer && A.SourceProducer == B.SourceProducer
        && A.SourceKind == B.SourceKind && A.Contract == B.Contract; }
    bool Continuous(const FPacket& P) const noexcept
    {
        const auto* Old = Previous ? &Previous->Original : (Anchor->PreviousOriginal ? &*Anchor->PreviousOriginal : nullptr);
        if (!Old) return P.Input.Reset;
        if (P.Sequence <= Old->Sequence) return false; // distinct wire frame cannot reuse a prior delivery identity
        const auto& A = Old->Input; const auto& B = P.Input;
        if (B.SourceSequence < A.SourceSequence) return false;
        if (B.Reset) return B.DeviceGeneration >= A.DeviceGeneration && B.SourceSequence > A.SourceSequence;
        if (B.DeviceGeneration != A.DeviceGeneration) return false;
        if (B.SourceSequence == A.SourceSequence) return B.Values == A.Values && B.ActiveMask == A.ActiveMask && B.Edges.empty();
        auto Active = A.ActiveMask;
        for (const auto& E : B.Edges)
        {
            const auto Bit = std::uint32_t{1} << E.Action; const bool Started = E.State == 0;
            if (E.SourceSequence <= A.SourceSequence || bool(Active & Bit) == Started) return false;
            if (Started) Active |= Bit; else Active &= ~Bit;
        }
        return Active == B.ActiveMask;
    }
    std::shared_ptr<const FReceiverBinding> Binding;
    std::shared_ptr<const FRestoreAnchor> Anchor;
    const std::thread::id Worker;
    U64 Next = 0, Serial = 0, LastLocalPublication = 0, AuthorityBeforePublication = 0;
    std::optional<std::array<std::int16_t, ActionCount>> Applied;
    std::shared_ptr<const C::FDecision> Previous;
    std::array<std::shared_ptr<const FLocalReceipt>, Window> History{};
    std::shared_ptr<const FLocalReceipt> Latest;
    std::optional<FPending> Pending;
    EPhase Phase = EPhase::Idle;
    bool InCallback = false;
};
}
