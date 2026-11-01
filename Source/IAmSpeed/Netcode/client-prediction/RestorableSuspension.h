#pragma once
#include "SuspensionSystem.h"
#include "IAmSpeed/World/Simulation/ReplayFieldArchive.h"
#include <optional>
#include <memory>
#include "IAmSpeed/World/Simulation/PreparedAccessRanges.h"
#include <cmath>

namespace Speed::Netcode::Prediction
{
// Actual derived object, not a cast of an existing base object. This backend
// preserves the Engine algorithm and owns its configuration backing storage.
// It is not installed on legacy components by this header. The predicted-world
// owner must exclude all live access, pin the native graph and validate frame /
// world / topology identities before preparing or restoring any child state.
class FRestorableSuspension final : public Chaos::FSimpleSuspensionSim
{
public:
    struct FState
    {
        float Displacement=0,LastDisplacement=0;
        FVector LocalVelocity=FVector::ZeroVector;
        float Force=0;
        FVector LocalOffset=FVector::ZeroVector;
        float SpringDisplacement=0;
        uint32 SpringIndex=0;
        float LastSpringLength=0;
        float Averaging[NUM_SUS_AVERAGING]{};
        int32 AveragingCount=0,AveragingNum=0;
    };
private:
    Chaos::FSimpleSuspensionConfig OwnedConfiguration,FrozenConfiguration;
    uint64 RestoreRevision=1;
    const std::shared_ptr<const uint8> LifetimeIdentity=std::make_shared<const uint8>(uint8(0));
    template<class Archive,class Config> static void ConfigurationFields(Archive& A,Config& S)
    {
        A.Vector(S.SuspensionAxis);A.Vector(S.SuspensionForceOffset);
        A.Scalar(S.SuspensionMaxRaise);A.Scalar(S.SuspensionMaxDrop);A.Scalar(S.MaxLength);
        A.Scalar(S.SpringRate);A.Scalar(S.SpringPreload);A.Scalar(S.CompressionDamping);A.Scalar(S.ReboundDamping);
        A.Scalar(S.RestingForce);A.Scalar(S.Swaybar);A.Scalar(S.DampingRatio);A.Scalar(S.WheelLoadRatio);
        A.Scalar(S.RaycastSafetyMargin);A.Scalar(S.SuspensionSmoothing);
    }
    template<class Archive,class State> static void StateFields(Archive& A,State& S)
    {
        A.Scalar(S.Displacement);A.Scalar(S.LastDisplacement);A.Vector(S.LocalVelocity);A.Scalar(S.Force);
        A.Vector(S.LocalOffset);A.Scalar(S.SpringDisplacement);A.Scalar(S.SpringIndex);A.Scalar(S.LastSpringLength);
        A.Array(S.Averaging);A.Scalar(S.AveragingCount);A.Scalar(S.AveragingNum);
    }
    template<class T> static bool Bits(const T& A,const T& B) noexcept
    {static_assert(std::is_arithmetic_v<T>);return std::memcmp(&A,&B,sizeof(T))==0;}
    static bool VectorBits(const FVector& A,const FVector& B) noexcept
    {return Bits(A.X,B.X)&&Bits(A.Y,B.Y)&&Bits(A.Z,B.Z);}
    static bool ValidConfiguration(const Chaos::FSimpleSuspensionConfig& C) noexcept
    {
        const float Values[]={C.SuspensionMaxRaise,C.SuspensionMaxDrop,C.MaxLength,C.SpringRate,C.SpringPreload,
            C.CompressionDamping,C.ReboundDamping,C.RestingForce,C.Swaybar,C.DampingRatio,C.WheelLoadRatio,C.RaycastSafetyMargin};
        for(float V:Values)if(!std::isfinite(V))return false;
        return !C.SuspensionAxis.ContainsNaN()&&!C.SuspensionForceOffset.ContainsNaN()
            &&C.SuspensionSmoothing>=0&&C.SuspensionSmoothing<=NUM_SUS_AVERAGING;
    }
    static bool SameConfiguration(const Chaos::FSimpleSuspensionConfig& A,const Chaos::FSimpleSuspensionConfig& B) noexcept
    {
        return VectorBits(A.SuspensionAxis,B.SuspensionAxis)&&VectorBits(A.SuspensionForceOffset,B.SuspensionForceOffset)
            &&Bits(A.SuspensionMaxRaise,B.SuspensionMaxRaise)&&Bits(A.SuspensionMaxDrop,B.SuspensionMaxDrop)&&Bits(A.MaxLength,B.MaxLength)
            &&Bits(A.SpringRate,B.SpringRate)&&Bits(A.SpringPreload,B.SpringPreload)&&Bits(A.CompressionDamping,B.CompressionDamping)
            &&Bits(A.ReboundDamping,B.ReboundDamping)&&Bits(A.RestingForce,B.RestingForce)&&Bits(A.Swaybar,B.Swaybar)
            &&Bits(A.DampingRatio,B.DampingRatio)&&Bits(A.WheelLoadRatio,B.WheelLoadRatio)&&Bits(A.RaycastSafetyMargin,B.RaycastSafetyMargin)
            &&Bits(A.SuspensionSmoothing,B.SuspensionSmoothing);
    }
    static bool SameState(const FState& A,const FState& B) noexcept
    {
        if(!Bits(A.Displacement,B.Displacement)||!Bits(A.LastDisplacement,B.LastDisplacement)||!VectorBits(A.LocalVelocity,B.LocalVelocity)
            ||!Bits(A.Force,B.Force)||!VectorBits(A.LocalOffset,B.LocalOffset)||!Bits(A.SpringDisplacement,B.SpringDisplacement)
            ||A.SpringIndex!=B.SpringIndex||!Bits(A.LastSpringLength,B.LastSpringLength)||A.AveragingCount!=B.AveragingCount||A.AveragingNum!=B.AveragingNum)return false;
        for(int32 I=0;I<NUM_SUS_AVERAGING;++I)if(!Bits(A.Averaging[I],B.Averaging[I]))return false;
        return true;
    }
    static bool ValidState(const FState& S) noexcept
    {
        if(!std::isfinite(S.Displacement)||!std::isfinite(S.LastDisplacement)||S.LocalVelocity.ContainsNaN()
            ||!std::isfinite(S.Force)||S.LocalOffset.ContainsNaN()||!std::isfinite(S.SpringDisplacement)||!std::isfinite(S.LastSpringLength)
            ||S.AveragingCount<0||S.AveragingCount>=NUM_SUS_AVERAGING||S.AveragingNum<0||S.AveragingNum>NUM_SUS_AVERAGING)return false;
        for(float V:S.Averaging)if(!std::isfinite(V))return false;
        return true;
    }
    bool ConfigurationIntact() const noexcept
    {return SetupPtr==&OwnedConfiguration&&ValidConfiguration(OwnedConfiguration)&&SameConfiguration(OwnedConfiguration,FrozenConfiguration);}
    FState CaptureFields() const noexcept
    {
        FState S;S.Displacement=DisplacementInput;S.LastDisplacement=LastDisplacement;S.LocalVelocity=LocalVelocity;S.Force=SuspensionForce;
        S.LocalOffset=LocalOffset;S.SpringDisplacement=SpringDisplacement;S.SpringIndex=SpringIndex;S.LastSpringLength=LastSpringLength;
        for(int32 I=0;I<NUM_SUS_AVERAGING;++I)S.Averaging[I]=AveragingLength[I];
        S.AveragingCount=AveragingCount;S.AveragingNum=AveragingNum;return S;
    }
public:
    class FPreparedState
    {
        friend class FRestorableSuspension;
        const FRestorableSuspension* Origin=nullptr;
        uint64 Revision=0;
        std::shared_ptr<const uint8> Lifetime;
        FState Before,After;
        FPreparedState(const FRestorableSuspension* O,uint64 R,std::shared_ptr<const uint8> I,FState B,FState A):Origin(O),Revision(R),Lifetime(std::move(I)),Before(B),After(A){}
    public:
        FPreparedState(const FPreparedState&)=default;
    };
    explicit FRestorableSuspension(const Chaos::FSimpleSuspensionConfig& Configuration)
        :FSimpleSuspensionSim(&Configuration),OwnedConfiguration(Configuration),FrozenConfiguration(Configuration)
    {SetupPtr=&OwnedConfiguration;}
    explicit FRestorableSuspension(const Chaos::FSimpleSuspensionSim& Source)
        :FSimpleSuspensionSim(Source),OwnedConfiguration(Source.Setup()),FrozenConfiguration(Source.Setup())
    {SetupPtr=&OwnedConfiguration;}
    FRestorableSuspension(const FRestorableSuspension&)=delete;
    FRestorableSuspension& operator=(const FRestorableSuspension&)=delete;
    std::optional<Replay::FBytes> CaptureCheckpoint() const
    {
        const auto S=CaptureFields();if(!ConfigurationIntact()||!ValidState(S))return {};
        Replay::FFieldWriter W;W.Scalar(uint32(0x31535349));W.Scalar(uint16(1));W.Scalar(uint16(NUM_SUS_AVERAGING));
        ConfigurationFields(W,OwnedConfiguration);StateFields(W,S);return W.Bytes();
    }
    std::optional<FPreparedState> PrepareRestore(const Replay::FBytes& Bytes) const
    {
        if(!ConfigurationIntact()||RestoreRevision==MAX_uint64)return {};
        try
        {
            Replay::FFieldReader R(Bytes);uint32 Magic=0;uint16 Version=0,Count=0;R.Scalar(Magic);R.Scalar(Version);R.Scalar(Count);
            if(Magic!=0x31535349||Version!=1||Count!=NUM_SUS_AVERAGING)return {};
            Chaos::FSimpleSuspensionConfig Configuration;ConfigurationFields(R,Configuration);
            FState State;StateFields(R,State);
            if(!R.AtEnd()||!SameConfiguration(Configuration,FrozenConfiguration)||!ValidState(State))return {};
            const auto Before=CaptureFields();if(!ValidState(Before))return {};
            return FPreparedState(this,RestoreRevision,LifetimeIdentity,Before,State);
        }
        catch(const Replay::FFieldError&){return {};}
    }
    // Child-level preconditions only; not a native world lease or frame proof.
    bool CanCommit(const FPreparedState& P) const noexcept
    {return P.Lifetime==LifetimeIdentity&&P.Origin==this&&P.Revision==RestoreRevision&&ConfigurationIntact()&&SameState(P.Before,CaptureFields());}
    // Exact child ranges for the native world's global alias validation.
    // The parent must hold actual object/config/lifecycle pins; declarations
    // alone never certify exclusion of escaped mutable consumers.
    bool AppendPreparedAccess(const FPreparedState& P,uint64 Participant,
        std::vector<Speed::PreparedAccess::FParticipantAccess>& Out) const
    {
        if(!Participant||!CanCommit(P))return false;
        Speed::PreparedAccess::FParticipantAccess A;A.Participant=Participant;
        const auto Write=[&A](const auto& V){A.Writes.push_back({reinterpret_cast<std::uintptr_t>(&V),sizeof(V)});};
        const auto Read=[&A](const auto& V){A.ReadOnly.push_back({reinterpret_cast<std::uintptr_t>(&V),sizeof(V)});};
        Write(DisplacementInput);Write(LastDisplacement);Write(LocalVelocity);Write(SuspensionForce);Write(LocalOffset);
        Write(SpringDisplacement);Write(SpringIndex);Write(LastSpringLength);Write(AveragingLength);Write(AveragingCount);Write(AveragingNum);Write(RestoreRevision);
        Read(OwnedConfiguration);Read(FrozenConfiguration);Read(SetupPtr);Read(LifetimeIdentity);
        Out.push_back(std::move(A));return true;
    }
    bool CommitPrepared(const FPreparedState& P) noexcept
    {
        if(!CanCommit(P))return false;
        const auto& S=P.After;DisplacementInput=S.Displacement;LastDisplacement=S.LastDisplacement;LocalVelocity=S.LocalVelocity;
        SuspensionForce=S.Force;LocalOffset=S.LocalOffset;SpringDisplacement=S.SpringDisplacement;SpringIndex=S.SpringIndex;
        LastSpringLength=S.LastSpringLength;for(int32 I=0;I<NUM_SUS_AVERAGING;++I)AveragingLength[I]=S.Averaging[I];
        AveragingCount=S.AveragingCount;AveragingNum=S.AveragingNum;++RestoreRevision;return true;
    }
};
}
