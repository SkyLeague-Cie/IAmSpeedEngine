#pragma once
#include "../transport/CanonicalActorState.h"
#include "../consumed-command-v2/ConsumedCommand.h"
#include <deque>
#include <limits>

namespace Speed::Netcode::Prediction
{
// Values-only historical comparison. Does NOT admit a client physics world,
// authenticate received authority, restore components or acknowledge a rollback.
// The host must supply completed local and authenticated server publications.
struct FTimeline
{
    Consumed::FWorldIdentity World;
    U64 AuthorityPeer=0, AuthorityEpoch=0, Branch=0, RegistryGeneration=0;
    std::array<std::uint8_t,32> Schema{};
    bool Valid() const noexcept
    {
        bool Nonzero=false; for (auto B:Schema) Nonzero|=B!=0;
        return World.Valid() && AuthorityPeer && AuthorityEpoch && Branch && RegistryGeneration && Nonzero;
    }
    bool operator==(const FTimeline& B) const noexcept
    {
        return World==B.World && AuthorityPeer==B.AuthorityPeer && AuthorityEpoch==B.AuthorityEpoch
            && Branch==B.Branch && RegistryGeneration==B.RegistryGeneration && Schema==B.Schema;
    }
};
struct FCompletedState
{
    FTimeline Timeline;
    FWorldState State;
};
struct FActorError
{
    U64 Actor=0, Incarnation=0;
    double Position=0, Velocity=0, RotationRadians=0, AngularVelocity=0;
    bool CanonicalBytesDifferent=false;
};
enum class EComparison { Equal, Divergent, MissingHistory, FutureAuthority, WrongTimeline, InvalidState, RosterMismatch };
struct FComparison
{
    EComparison Result=EComparison::InvalidState;
    U64 Frame=0;
    bool WorldBytesDifferent=false;
    std::vector<FActorError> Actors;
};
inline bool Valid(const FCompletedState& S)
{
    if (!S.Timeline.Valid() || !ValidState(S.State)
        || S.Timeline.RegistryGeneration!=S.State.RegistryGeneration) return false;
    for (const auto& A:S.State.Actors)
        if (A.Lifecycle==ELifecycle::Alive && !DecodeCanonicalActor(A.Payload)) return false;
    return true;
}
inline double Distance3(const std::array<double,19>& A,const std::array<double,19>& B,std::size_t At)
{
    return std::hypot(std::hypot(A[At]-B[At],A[At+1]-B[At+1]),A[At+2]-B[At+2]);
}
inline std::optional<double> OrientationError(const std::array<double,19>& A,const std::array<double,19>& B)
{
    const double NA=std::hypot(std::hypot(A[9],A[10]),std::hypot(A[11],A[12]));
    const double NB=std::hypot(std::hypot(B[9],B[10]),std::hypot(B[11],B[12]));
    if (!(NA>0) || !(NB>0) || !std::isfinite(NA) || !std::isfinite(NB)) return {};
    // q and -q encode the same orientation. This normalization is ONLY for
    // diagnostic angle measurement; no source state or solver input is changed.
    double Dot=0; for (std::size_t I=9;I<13;++I) Dot+=(A[I]/NA)*(B[I]/NB);
    const double D=std::min(1.0,std::abs(Dot));
    return 2*std::acos(D);
}
inline FComparison Compare(const FCompletedState& Local,const FCompletedState& Authority)
{
    FComparison Out; Out.Frame=Authority.State.Frame;
    if (!Valid(Local) || !Valid(Authority)) return Out;
    if (!(Local.Timeline==Authority.Timeline)) {Out.Result=EComparison::WrongTimeline;return Out;}
    if (Local.State.Frame!=Authority.State.Frame) return Out;
    const auto& L=Local.State; const auto& A=Authority.State;
    if (L.Actors.size()!=A.Actors.size()) {Out.Result=EComparison::RosterMismatch;return Out;}
    // Validate the entire roster before producing any partial comparison.
    for (std::size_t I=0;I<L.Actors.size();++I)
    {
        const auto& X=L.Actors[I]; const auto& Y=A.Actors[I];
        if (X.Actor!=Y.Actor || X.Incarnation!=Y.Incarnation || X.Type!=Y.Type
            || X.SchemaVersion!=Y.SchemaVersion || X.Lifecycle!=Y.Lifecycle)
        {Out.Result=EComparison::RosterMismatch;return Out;}
    }
    Out.WorldBytesDifferent=L.WorldPayload!=A.WorldPayload;
    bool Different=Out.WorldBytesDifferent;
    for (std::size_t I=0;I<L.Actors.size();++I)
    {
        const auto& X=L.Actors[I]; const auto& Y=A.Actors[I];
        FActorError E; E.Actor=X.Actor; E.Incarnation=X.Incarnation;
        E.CanonicalBytesDifferent=X.Payload!=Y.Payload; Different|=E.CanonicalBytesDifferent;
        if (X.Lifecycle==ELifecycle::Alive)
        {
            const auto SX=DecodeCanonicalActor(X.Payload), SY=DecodeCanonicalActor(Y.Payload);
            if (!SX || !SY || SX->HasBody!=SY->HasBody) return FComparison{};
            if (SX->HasBody)
            {
                const auto Angle=OrientationError(SX->Kinematic,SY->Kinematic);
                if (!Angle) return FComparison{};
                E.Position=Distance3(SX->Kinematic,SY->Kinematic,0);
                E.Velocity=Distance3(SX->Kinematic,SY->Kinematic,3);
                E.RotationRadians=*Angle;
                E.AngularVelocity=Distance3(SX->Kinematic,SY->Kinematic,13);
                if (!std::isfinite(E.Position) || !std::isfinite(E.Velocity) || !std::isfinite(E.AngularVelocity)) return FComparison{};
            }
        }
        Out.Actors.push_back(E);
    }
    // Exact state mismatch triggers reconciliation even when visible pose is
    // equal. A visual smoothing threshold must never hide mechanical divergence.
    Out.Result=Different?EComparison::Divergent:EComparison::Equal;
    return Out;
}
enum class EStore { Stored, Invalid, WrongTimeline, Nonconsecutive, CapacityExceeded };
class FHistory
{
    struct FEntry {FCompletedState State;std::size_t Bytes=0;};
    FTimeline Timeline;
    std::deque<FEntry> Entries;
    std::size_t Frames=0,ByteLimit=0,Bytes=0;
    static std::size_t Size(const FCompletedState& S)
    {
        std::size_t N=S.State.WorldPayload.size();
        for (const auto& A:S.State.Actors) N+=A.Payload.size()+sizeof(FActorState);
        return N+sizeof(FCompletedState);
    }
public:
    // The host freezes explicit retention budgets at admission. No implicit
    // rebind, history reset, future-authority catch-up or pose-only fallback.
    FHistory(FTimeline In,std::size_t FrameLimit,std::size_t MaxBytes)
        :Timeline(In),Frames(FrameLimit),ByteLimit(MaxBytes) {}
    EStore Store(FCompletedState S)
    {
        if (!Timeline.Valid() || !Frames || Frames>4096 || !ByteLimit || !Valid(S)) return EStore::Invalid;
        if (!(Timeline==S.Timeline)) return EStore::WrongTimeline;
        if (!Entries.empty() && (Entries.back().State.State.Frame==std::numeric_limits<U64>::max()
            || S.State.Frame!=Entries.back().State.State.Frame+1)) return EStore::Nonconsecutive;
        const auto N=Size(S); if (N>ByteLimit) return EStore::CapacityExceeded;
        // Once validated, evict whole oldest frames until both budgets fit.
        while (!Entries.empty() && (Entries.size()>=Frames || N>ByteLimit-Bytes))
        {Bytes-=Entries.front().Bytes;Entries.pop_front();}
        Entries.push_back({std::move(S),N});Bytes+=N;return EStore::Stored;
    }
    FComparison CompareAuthority(const FCompletedState& S) const
    {
        FComparison R;R.Frame=S.State.Frame;
        if (!Valid(S)) return R;
        if (!(Timeline==S.Timeline)) {R.Result=EComparison::WrongTimeline;return R;}
        if (!Entries.empty() && S.State.Frame>Entries.back().State.State.Frame)
        {R.Result=EComparison::FutureAuthority;return R;}
        for (const auto& E:Entries) if (E.State.State.Frame==S.State.Frame) return Compare(E.State,S);
        R.Result=EComparison::MissingHistory;return R;
    }
    // Caller may truncate only after the native transaction has restored its
    // completed publication. This is historical storage, not a restore receipt.
    void TruncateFrom(U64 Frame)
    {while (!Entries.empty() && Entries.back().State.State.Frame>=Frame) {Bytes-=Entries.back().Bytes;Entries.pop_back();}}
    std::optional<U64> LastFrame() const noexcept
    {return Entries.empty()?std::optional<U64>{}:std::optional<U64>{Entries.back().State.State.Frame};}
    std::size_t Count() const noexcept {return Entries.size();}
    std::size_t RetainedBytes() const noexcept {return Bytes;}
};
}
