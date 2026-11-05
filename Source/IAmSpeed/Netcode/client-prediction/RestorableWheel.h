#pragma once
#include "WheelSystem.h"
#include "IAmSpeed/World/Simulation/ReplayFieldArchive.h"
#include "IAmSpeed/World/Simulation/PreparedAccessRanges.h"
#include <optional>
#include <memory>
#include <cmath>

namespace Speed::Netcode::Prediction
{
// Genuine native Engine wheel with owned, frozen configuration. The current
// codec admits the empty slip-graph recipe only: Engine FGraph exposes no exact
// point enumeration. A nonempty graph is refused, never sampled or approximated.
// The world owner still supplies actual lifecycle/exclusive-boundary protection.
class FRestorableWheel final : public Chaos::FSimpleWheelSim
{
    using FSimpleWheelConfig=Chaos::FSimpleWheelConfig;
    struct FState
    {
        bool BrakeEnabled=0;
        bool HandbrakeEnabled=0;
        bool SteeringEnabled=0;
        bool EngineEnabled=0;
        bool TractionControlEnabled=0;
        bool ABSEnabled=0;
        float FrictionMultiplier=0;
        float LateralSlipGraphMultiplier=0;
        float CorneringStiffness=0;
        float MaxSteeringAngle=0;
        float MaxBrakeTorque=0;
        float HandbrakeTorque=0;
        FSimpleWheelConfig::EExternalTorqueCombineMethod ExternalTorqueCombineMethod{};
        float Re=0;
        float Omega=0;
        float Sx=0;
        float Inertia=0;
        float DriveTorque=0;
        float BrakeTorque=0;
        float ForceIntoSurface=0;
        FVector GroundVelocityVector{};
        float AngularPosition=0;
        float SteeringAngle=0;
        float SurfaceFriction=0;
        float MaxOmega=0;
        float ExternalDriveTorque=0;
        float ExternalBrakeTorque=0;
        FVector ForceFromFriction{};
        float MassPerWheel=0;
        float SlipVelocity=0;
        float SlipAngle=0;
        bool bInContact=0;
        uint32 WheelIndex=0;
        bool bEngineBraking=0;
        float AppliedLinearDriveForce=0;
        float AppliedLinearBrakeForce=0;
        float LongitudinalAdhesiveLimit=0;
        float LateralAdhesiveLimit=0;
        float SideSlipModifier=0;
        float Spin=0;
        float AvailableGrip=0;
        FVector InputForces{};
        bool bClipping=0;
        bool bABSActivated=0;
    };
    FSimpleWheelConfig OwnedConfiguration;
    Replay::FBytes FrozenConfiguration;
    uint64 Revision=1;
    const std::shared_ptr<const uint8> Lifetime=std::make_shared<const uint8>(uint8(0));
    template<class Archive,class Config> static void ConfigurationFields(Archive& A,Config& S)
    {
        A.Vector(S.Offset);
        A.Scalar(S.WheelMass);
        A.Scalar(S.WheelRadius);
        A.Scalar(S.WheelWidth);
        A.Scalar(S.MaxSteeringAngle);
        A.Scalar(S.MaxBrakeTorque);
        A.Scalar(S.HandbrakeTorque);
        A.Scalar(S.ABSEnabled);
        A.Scalar(S.BrakeEnabled);
        A.Scalar(S.HandbrakeEnabled);
        A.Scalar(S.SteeringEnabled);
        A.Scalar(S.EngineEnabled);
        A.Scalar(S.TractionControlEnabled);
        A.Scalar(S.TorqueRatio);
        A.Scalar(S.AxleType);
        A.Scalar(S.FrictionCombineMethod);
        A.Scalar(S.ExternalTorqueCombineMethod);
        A.Scalar(S.FrictionMultiplier);
        A.Scalar(S.LateralSlipGraphMultiplier);
        A.Scalar(S.CorneringStiffness);
        A.Scalar(S.SideSlipModifier);
        A.Scalar(S.SlipThreshold);
        A.Scalar(S.SkidThreshold);
        A.Scalar(S.MaxSpinRotation);
    }
    template<class Archive,class State> static void StateFields(Archive& A,State& S)
    {
        A.Scalar(S.BrakeEnabled);
        A.Scalar(S.HandbrakeEnabled);
        A.Scalar(S.SteeringEnabled);
        A.Scalar(S.EngineEnabled);
        A.Scalar(S.TractionControlEnabled);
        A.Scalar(S.ABSEnabled);
        A.Scalar(S.FrictionMultiplier);
        A.Scalar(S.LateralSlipGraphMultiplier);
        A.Scalar(S.CorneringStiffness);
        A.Scalar(S.MaxSteeringAngle);
        A.Scalar(S.MaxBrakeTorque);
        A.Scalar(S.HandbrakeTorque);
        A.Scalar(S.ExternalTorqueCombineMethod);
        A.Scalar(S.Re);
        A.Scalar(S.Omega);
        A.Scalar(S.Sx);
        A.Scalar(S.Inertia);
        A.Scalar(S.DriveTorque);
        A.Scalar(S.BrakeTorque);
        A.Scalar(S.ForceIntoSurface);
        A.Vector(S.GroundVelocityVector);
        A.Scalar(S.AngularPosition);
        A.Scalar(S.SteeringAngle);
        A.Scalar(S.SurfaceFriction);
        A.Scalar(S.MaxOmega);
        A.Scalar(S.ExternalDriveTorque);
        A.Scalar(S.ExternalBrakeTorque);
        A.Vector(S.ForceFromFriction);
        A.Scalar(S.MassPerWheel);
        A.Scalar(S.SlipVelocity);
        A.Scalar(S.SlipAngle);
        A.Scalar(S.bInContact);
        A.Scalar(S.WheelIndex);
        A.Scalar(S.bEngineBraking);
        A.Scalar(S.AppliedLinearDriveForce);
        A.Scalar(S.AppliedLinearBrakeForce);
        A.Scalar(S.LongitudinalAdhesiveLimit);
        A.Scalar(S.LateralAdhesiveLimit);
        A.Scalar(S.SideSlipModifier);
        A.Scalar(S.Spin);
        A.Scalar(S.AvailableGrip);
        A.Vector(S.InputForces);
        A.Scalar(S.bClipping);
        A.Scalar(S.bABSActivated);
    }
    static bool Valid(const FState& S) noexcept
    {
        if(!std::isfinite(S.FrictionMultiplier))return false;
        if(!std::isfinite(S.LateralSlipGraphMultiplier))return false;
        if(!std::isfinite(S.CorneringStiffness))return false;
        if(!std::isfinite(S.MaxSteeringAngle))return false;
        if(!std::isfinite(S.MaxBrakeTorque))return false;
        if(!std::isfinite(S.HandbrakeTorque))return false;
        if(!std::isfinite(S.Re))return false;
        if(!std::isfinite(S.Omega))return false;
        if(!std::isfinite(S.Sx))return false;
        if(!std::isfinite(S.Inertia))return false;
        if(!std::isfinite(S.DriveTorque))return false;
        if(!std::isfinite(S.BrakeTorque))return false;
        if(!std::isfinite(S.ForceIntoSurface))return false;
        if(S.GroundVelocityVector.ContainsNaN())return false;
        if(!std::isfinite(S.AngularPosition))return false;
        if(!std::isfinite(S.SteeringAngle))return false;
        if(!std::isfinite(S.SurfaceFriction))return false;
        if(!std::isfinite(S.MaxOmega))return false;
        if(!std::isfinite(S.ExternalDriveTorque))return false;
        if(!std::isfinite(S.ExternalBrakeTorque))return false;
        if(S.ForceFromFriction.ContainsNaN())return false;
        if(!std::isfinite(S.MassPerWheel))return false;
        if(!std::isfinite(S.SlipVelocity))return false;
        if(!std::isfinite(S.SlipAngle))return false;
        if(!std::isfinite(S.AppliedLinearDriveForce))return false;
        if(!std::isfinite(S.AppliedLinearBrakeForce))return false;
        if(!std::isfinite(S.LongitudinalAdhesiveLimit))return false;
        if(!std::isfinite(S.LateralAdhesiveLimit))return false;
        if(!std::isfinite(S.SideSlipModifier))return false;
        if(!std::isfinite(S.Spin))return false;
        if(!std::isfinite(S.AvailableGrip))return false;
        if(S.InputForces.ContainsNaN())return false;
        return S.ExternalTorqueCombineMethod>=FSimpleWheelConfig::EExternalTorqueCombineMethod::None
            &&S.ExternalTorqueCombineMethod<=FSimpleWheelConfig::EExternalTorqueCombineMethod::Additive;
    }
    static Replay::FBytes ConfigurationBytes(const FSimpleWheelConfig& C)
    {
        if(!C.LateralSlipGraph.IsEmpty())return {};
        if(C.Offset.ContainsNaN())return {};
        if(!std::isfinite(C.WheelMass))return {};
        if(!std::isfinite(C.WheelRadius))return {};
        if(!std::isfinite(C.WheelWidth))return {};
        if(!std::isfinite(C.MaxBrakeTorque))return {};
        if(!std::isfinite(C.HandbrakeTorque))return {};
        if(!std::isfinite(C.TorqueRatio))return {};
        if(!std::isfinite(C.FrictionMultiplier))return {};
        if(!std::isfinite(C.LateralSlipGraphMultiplier))return {};
        if(!std::isfinite(C.CorneringStiffness))return {};
        if(!std::isfinite(C.SideSlipModifier))return {};
        if(!std::isfinite(C.SlipThreshold))return {};
        if(!std::isfinite(C.SkidThreshold))return {};
        if(!std::isfinite(C.MaxSpinRotation))return {};
        if(C.AxleType>FSimpleWheelConfig::EAxleType::Rear
            ||C.FrictionCombineMethod<FSimpleWheelConfig::EFrictionCombineMethod::Multiply
            ||C.FrictionCombineMethod>FSimpleWheelConfig::EFrictionCombineMethod::Average
            ||C.ExternalTorqueCombineMethod>FSimpleWheelConfig::EExternalTorqueCombineMethod::Additive)return {};
        Replay::FFieldWriter W;ConfigurationFields(W,C);return W.Bytes();
    }
    FState CaptureFields() const noexcept
    {
        FState S;
        S.BrakeEnabled=BrakeEnabled;
        S.HandbrakeEnabled=HandbrakeEnabled;
        S.SteeringEnabled=SteeringEnabled;
        S.EngineEnabled=EngineEnabled;
        S.TractionControlEnabled=TractionControlEnabled;
        S.ABSEnabled=ABSEnabled;
        S.FrictionMultiplier=FrictionMultiplier;
        S.LateralSlipGraphMultiplier=LateralSlipGraphMultiplier;
        S.CorneringStiffness=CorneringStiffness;
        S.MaxSteeringAngle=MaxSteeringAngle;
        S.MaxBrakeTorque=MaxBrakeTorque;
        S.HandbrakeTorque=HandbrakeTorque;
        S.ExternalTorqueCombineMethod=ExternalTorqueCombineMethod;
        S.Re=Re;
        S.Omega=Omega;
        S.Sx=Sx;
        S.Inertia=Inertia;
        S.DriveTorque=DriveTorque;
        S.BrakeTorque=BrakeTorque;
        S.ForceIntoSurface=ForceIntoSurface;
        S.GroundVelocityVector=GroundVelocityVector;
        S.AngularPosition=AngularPosition;
        S.SteeringAngle=SteeringAngle;
        S.SurfaceFriction=SurfaceFriction;
        S.MaxOmega=MaxOmega;
        S.ExternalDriveTorque=ExternalDriveTorque;
        S.ExternalBrakeTorque=ExternalBrakeTorque;
        S.ForceFromFriction=ForceFromFriction;
        S.MassPerWheel=MassPerWheel;
        S.SlipVelocity=SlipVelocity;
        S.SlipAngle=SlipAngle;
        S.bInContact=bInContact;
        S.WheelIndex=WheelIndex;
        S.bEngineBraking=bEngineBraking;
        S.AppliedLinearDriveForce=AppliedLinearDriveForce;
        S.AppliedLinearBrakeForce=AppliedLinearBrakeForce;
        S.LongitudinalAdhesiveLimit=LongitudinalAdhesiveLimit;
        S.LateralAdhesiveLimit=LateralAdhesiveLimit;
        S.SideSlipModifier=SideSlipModifier;
        S.Spin=Spin;
        S.AvailableGrip=AvailableGrip;
        S.InputForces=InputForces;
        S.bClipping=bClipping;
        S.bABSActivated=bABSActivated;
        return S;
    }
    static Replay::FBytes StateBytes(const FState& S)
    {Replay::FFieldWriter W;StateFields(W,S);return W.Bytes();}
    bool ConfigurationIntact() const
    {return SetupPtr==&OwnedConfiguration&&!FrozenConfiguration.empty()&&ConfigurationBytes(OwnedConfiguration)==FrozenConfiguration;}
public:
    class FPreparedState
    {
        friend class FRestorableWheel;
        const FRestorableWheel* Origin=nullptr;
        std::shared_ptr<const uint8> Identity;
        uint64 Revision=0;
        Replay::FBytes Before;
        FState After;
    public:
        FPreparedState()=default;
    };
    // Source must have completed the native factory's initialization; no reads
    // of an incompletely initialized base wheel are permitted.
    explicit FRestorableWheel(const Chaos::FSimpleWheelSim& Source)
        :FSimpleWheelSim(Source),OwnedConfiguration(Source.Setup()),FrozenConfiguration(ConfigurationBytes(Source.Setup()))
    {SetupPtr=&OwnedConfiguration;}
    FRestorableWheel(const FRestorableWheel&)=delete;
    FRestorableWheel& operator=(const FRestorableWheel&)=delete;
    std::optional<Replay::FBytes> CaptureCheckpoint() const
    {
        if(!ConfigurationIntact())return {};
        const auto S=CaptureFields();if(!Valid(S))return {};
        Replay::FFieldWriter W;W.Scalar(uint32(0x31575349));W.Scalar(uint16(1));
        ConfigurationFields(W,OwnedConfiguration);StateFields(W,S);return W.Bytes();
    }
    std::optional<FPreparedState> PrepareRestore(const Replay::FBytes& Bytes) const
    {
        if(!ConfigurationIntact()||Revision==MAX_uint64)return {};
        try
        {
            Replay::FFieldReader R(Bytes);uint32 Magic=0;uint16 Version=0;R.Scalar(Magic);R.Scalar(Version);
            if(Magic!=0x31575349||Version!=1)return {};
            FSimpleWheelConfig Config;ConfigurationFields(R,Config);
            FState After;StateFields(R,After);
            if(!R.AtEnd()||ConfigurationBytes(Config)!=FrozenConfiguration||!Valid(After))return {};
            const auto Before=CaptureFields();if(!Valid(Before))return {};
            FPreparedState P;P.Origin=this;P.Identity=Lifetime;P.Revision=Revision;P.Before=StateBytes(Before);P.After=After;return P;
        }
        catch(const Replay::FFieldError&){return {};}
    }
    bool CanCommit(const FPreparedState& P) const
    {
        return P.Identity==Lifetime&&P.Origin==this&&P.Revision==Revision&&ConfigurationIntact()
            &&StateBytes(CaptureFields())==P.Before;
    }
private:
    void FinalizePrepared(const FPreparedState& P) noexcept
    {
        const auto& S=P.After;
        BrakeEnabled=S.BrakeEnabled;
        HandbrakeEnabled=S.HandbrakeEnabled;
        SteeringEnabled=S.SteeringEnabled;
        EngineEnabled=S.EngineEnabled;
        TractionControlEnabled=S.TractionControlEnabled;
        ABSEnabled=S.ABSEnabled;
        FrictionMultiplier=S.FrictionMultiplier;
        LateralSlipGraphMultiplier=S.LateralSlipGraphMultiplier;
        CorneringStiffness=S.CorneringStiffness;
        MaxSteeringAngle=S.MaxSteeringAngle;
        MaxBrakeTorque=S.MaxBrakeTorque;
        HandbrakeTorque=S.HandbrakeTorque;
        ExternalTorqueCombineMethod=S.ExternalTorqueCombineMethod;
        Re=S.Re;
        Omega=S.Omega;
        Sx=S.Sx;
        Inertia=S.Inertia;
        DriveTorque=S.DriveTorque;
        BrakeTorque=S.BrakeTorque;
        ForceIntoSurface=S.ForceIntoSurface;
        GroundVelocityVector=S.GroundVelocityVector;
        AngularPosition=S.AngularPosition;
        SteeringAngle=S.SteeringAngle;
        SurfaceFriction=S.SurfaceFriction;
        MaxOmega=S.MaxOmega;
        ExternalDriveTorque=S.ExternalDriveTorque;
        ExternalBrakeTorque=S.ExternalBrakeTorque;
        ForceFromFriction=S.ForceFromFriction;
        MassPerWheel=S.MassPerWheel;
        SlipVelocity=S.SlipVelocity;
        SlipAngle=S.SlipAngle;
        bInContact=S.bInContact;
        WheelIndex=S.WheelIndex;
        bEngineBraking=S.bEngineBraking;
        AppliedLinearDriveForce=S.AppliedLinearDriveForce;
        AppliedLinearBrakeForce=S.AppliedLinearBrakeForce;
        LongitudinalAdhesiveLimit=S.LongitudinalAdhesiveLimit;
        LateralAdhesiveLimit=S.LateralAdhesiveLimit;
        SideSlipModifier=S.SideSlipModifier;
        Spin=S.Spin;
        AvailableGrip=S.AvailableGrip;
        InputForces=S.InputForces;
        bClipping=S.bClipping;
        bABSActivated=S.bABSActivated;
        ++Revision;
    }
public:
    // Child convenience operation only, not an atomic multi-participant commit.
    bool CommitPrepared(const FPreparedState& P)
    {if(!CanCommit(P))return false;FinalizePrepared(P);return true;}
    bool AppendPreparedAccess(const FPreparedState& P,uint64 Participant,
        std::vector<Speed::PreparedAccess::FParticipantAccess>& Out) const
    {
        if(!Participant||!CanCommit(P))return false;
        Speed::PreparedAccess::FParticipantAccess A;A.Participant=Participant;
        const auto Write=[&A](const auto& V){A.Writes.push_back({reinterpret_cast<std::uintptr_t>(&V),sizeof(V)});};
        const auto Read=[&A](const auto& V){A.ReadOnly.push_back({reinterpret_cast<std::uintptr_t>(&V),sizeof(V)});};
        Write(BrakeEnabled);
        Write(HandbrakeEnabled);
        Write(SteeringEnabled);
        Write(EngineEnabled);
        Write(TractionControlEnabled);
        Write(ABSEnabled);
        Write(FrictionMultiplier);
        Write(LateralSlipGraphMultiplier);
        Write(CorneringStiffness);
        Write(MaxSteeringAngle);
        Write(MaxBrakeTorque);
        Write(HandbrakeTorque);
        Write(ExternalTorqueCombineMethod);
        Write(Re);
        Write(Omega);
        Write(Sx);
        Write(Inertia);
        Write(DriveTorque);
        Write(BrakeTorque);
        Write(ForceIntoSurface);
        Write(GroundVelocityVector);
        Write(AngularPosition);
        Write(SteeringAngle);
        Write(SurfaceFriction);
        Write(MaxOmega);
        Write(ExternalDriveTorque);
        Write(ExternalBrakeTorque);
        Write(ForceFromFriction);
        Write(MassPerWheel);
        Write(SlipVelocity);
        Write(SlipAngle);
        Write(bInContact);
        Write(WheelIndex);
        Write(bEngineBraking);
        Write(AppliedLinearDriveForce);
        Write(AppliedLinearBrakeForce);
        Write(LongitudinalAdhesiveLimit);
        Write(LateralAdhesiveLimit);
        Write(SideSlipModifier);
        Write(Spin);
        Write(AvailableGrip);
        Write(InputForces);
        Write(bClipping);
        Write(bABSActivated);
        Write(Revision);Read(OwnedConfiguration);Read(SetupPtr);Read(Lifetime);
        Read(P.After);
        Out.push_back(std::move(A));return true;
    }
};
}
