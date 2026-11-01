#pragma once
#include "RestorableWheel.h"
#include "RestorableSuspension.h"
#include "SimpleVehicle.h"
#include <type_traits>

namespace Speed::Netcode::Prediction
{
// Complete wheel/suspension STORAGE participant for the fixed four-wheel native
// recipe. It restores the actual base objects used by production subbody
// pointers; no downcast of a base object and no rebinding to a temporary clone.
// This participant does NOT cover component, subbody, input or world state.
// Its parent must retain the actual native lifecycle/exclusive boundary lease
// before every call and through publication. Pointer checks cannot supply it.
class FNativeVehicleStorageCheckpoint
{
    struct FFlags
    {
        bool Suspension=false,Mechanical=false,Friction=false,LegacyPosition=false;
        uint32 Driven=0;
    };
    static bool Supported(const Chaos::FSimpleWheeledVehicle& V) noexcept
    {
        return V.Wheels.Num()==4&&V.Suspension.Num()==4
            &&V.Engine.IsEmpty()&&V.Transmission.IsEmpty()&&V.Differential.IsEmpty()
            &&V.Steering.IsEmpty()&&V.Aerodynamics.IsEmpty()&&V.Aerofoils.IsEmpty()
            &&V.Thrusters.IsEmpty()&&V.Axles.IsEmpty()&&V.TorqueControlSim.IsEmpty()
            &&V.TargetRotationControlSim.IsEmpty()&&V.StabilizeControlSim.IsEmpty()
            &&V.NumDrivenWheels<=4;
    }
    static FFlags Flags(const Chaos::FSimpleWheeledVehicle& V) noexcept
    {return {V.bSuspensionEnabled,V.bMechanicalSimEnabled,V.bWheelFrictionEnabled,V.bLegacyWheelFrictionPosition,V.NumDrivenWheels};}
    template<class Archive,class Values> static void FlagFields(Archive& A,Values& V)
    {A.Scalar(V.Suspension);A.Scalar(V.Mechanical);A.Scalar(V.Friction);A.Scalar(V.LegacyPosition);A.Scalar(V.Driven);}
    static void WriteBlob(Replay::FFieldWriter& W,const Replay::FBytes& B)
    {W.Scalar(uint32(B.size()));for(uint8 V:B)W.Scalar(V);}
    static Replay::FBytes ReadBlob(Replay::FFieldReader& R)
    {uint32 N=0;R.Scalar(N);if(N>65536)throw Replay::FFieldError("native child bound");Replay::FBytes B(N);for(uint8& V:B)R.Scalar(V);return B;}
public:
    class FPrepared
    {
        friend class FNativeVehicleStorageCheckpoint;
        const Chaos::FSimpleWheeledVehicle* Origin=nullptr;
        const Chaos::FSimpleWheelSim* Wheels=nullptr;
        const Chaos::FSimpleSuspensionSim* Suspensions=nullptr;
        const Chaos::FSimpleWheelConfig* WheelConfig[4]{};
        const Chaos::FSimpleSuspensionConfig* SuspensionConfig[4]{};
        std::unique_ptr<FRestorableWheel> AfterWheels[4];
        std::unique_ptr<FRestorableSuspension> AfterSuspensions[4];
        Replay::FBytes Before;
        FFlags After;
    public:
        FPrepared()=default;
        FPrepared(const FPrepared&)=delete;
        FPrepared& operator=(const FPrepared&)=delete;
    };
    static std::optional<Replay::FBytes> Capture(const Chaos::FSimpleWheeledVehicle& V)
    {
        if(!Supported(V))return {};
        Replay::FFieldWriter W;W.Scalar(uint32(0x31565349));W.Scalar(uint16(1));const auto F=Flags(V);FlagFields(W,F);
        for(int32 I=0;I<4;++I)
        {
            const FRestorableWheel Wheel(V.Wheels[I]);const FRestorableSuspension Suspension(V.Suspension[I]);
            const auto WB=Wheel.CaptureCheckpoint(),SB=Suspension.CaptureCheckpoint();if(!WB||!SB)return {};
            WriteBlob(W,*WB);WriteBlob(W,*SB);
        }
        return W.Bytes();
    }
    static std::unique_ptr<FPrepared> Prepare(const Chaos::FSimpleWheeledVehicle& V,const Replay::FBytes& Bytes)
    {
        const auto Before=Capture(V);if(!Before)return {};
        try
        {
            Replay::FFieldReader R(Bytes);uint32 Magic=0;uint16 Version=0;R.Scalar(Magic);R.Scalar(Version);
            if(Magic!=0x31565349||Version!=1)return {};
            auto P=std::make_unique<FPrepared>();FlagFields(R,P->After);if(P->After.Driven>4)return {};
            P->Origin=&V;P->Wheels=V.Wheels.GetData();P->Suspensions=V.Suspension.GetData();P->Before=*Before;
            for(int32 I=0;I<4;++I)
            {
                P->WheelConfig[I]=V.Wheels[I].SetupPtr;P->SuspensionConfig[I]=V.Suspension[I].SetupPtr;
                P->AfterWheels[I]=std::make_unique<FRestorableWheel>(V.Wheels[I]);
                P->AfterSuspensions[I]=std::make_unique<FRestorableSuspension>(V.Suspension[I]);
                const auto WB=ReadBlob(R),SB=ReadBlob(R);
                const auto WP=P->AfterWheels[I]->PrepareRestore(WB);const auto SP=P->AfterSuspensions[I]->PrepareRestore(SB);
                if(!WP||!SP||!P->AfterWheels[I]->CommitPrepared(*WP)||!P->AfterSuspensions[I]->CommitPrepared(*SP))return {};
            }
            if(!R.AtEnd())return {};
            return P;
        }
        catch(const Replay::FFieldError&){return {};}
    }
    static bool CanCommit(const Chaos::FSimpleWheeledVehicle& V,const FPrepared& P)
    {
        if(P.Origin!=&V||!Supported(V)||P.Wheels!=V.Wheels.GetData()||P.Suspensions!=V.Suspension.GetData())return false;
        for(int32 I=0;I<4;++I)
            if(V.Wheels[I].SetupPtr!=P.WheelConfig[I]||V.Suspension[I].SetupPtr!=P.SuspensionConfig[I])return false;
        return Capture(V)==std::optional<Replay::FBytes>(P.Before);
    }
    static bool AppendPreparedAccess(const Chaos::FSimpleWheeledVehicle& V,const FPrepared& P,uint64 Identity,
        std::vector<Speed::PreparedAccess::FParticipantAccess>& Out)
    {
        if(!Identity||!CanCommit(V,P))return false;
        Speed::PreparedAccess::FParticipantAccess A;A.Participant=Identity;
        auto Write=[&](const auto& X){A.Writes.push_back({reinterpret_cast<std::uintptr_t>(&X),sizeof(X)});};
        auto Read=[&](const auto& X){A.ReadOnly.push_back({reinterpret_cast<std::uintptr_t>(&X),sizeof(X)});};
        for(int32 I=0;I<4;++I)
        {
            Write(V.Wheels[I]);Write(V.Suspension[I]);Read(*P.WheelConfig[I]);Read(*P.SuspensionConfig[I]);
            Read(*P.AfterWheels[I]);Read(*P.AfterSuspensions[I]);
        }
        Write(V.bSuspensionEnabled);Write(V.bMechanicalSimEnabled);Write(V.bWheelFrictionEnabled);
        Write(V.bLegacyWheelFrictionPosition);Write(V.NumDrivenWheels);Read(V.Wheels);Read(V.Suspension);Read(P.After);
        Write(P.Origin);Read(P.WheelConfig);Read(P.SuspensionConfig);Read(P.AfterWheels);Read(P.AfterSuspensions);
        Out.push_back(std::move(A));return true;
    }
    // Child convenience commit. A whole-world transaction must stage/revalidate
    // every domain first; this API alone does not publish an atomic world.
    static bool Commit(Chaos::FSimpleWheeledVehicle& V,FPrepared& P)
    {
        if(!CanCommit(V,P))return false;
        static_assert(std::is_nothrow_copy_assignable_v<Chaos::FSimpleWheelSim>);
        static_assert(std::is_nothrow_copy_assignable_v<Chaos::FSimpleSuspensionSim>);
        for(int32 I=0;I<4;++I)
        {
            // Legal, complete native base-subobject assignment including all
            // protected suspension history. Preserve ORIGINAL configuration
            // backing addresses so no pointer survives into a retired plan.
            V.Wheels[I]=static_cast<const Chaos::FSimpleWheelSim&>(*P.AfterWheels[I]);
            V.Wheels[I].SetupPtr=P.WheelConfig[I];
            V.Suspension[I]=static_cast<const Chaos::FSimpleSuspensionSim&>(*P.AfterSuspensions[I]);
            V.Suspension[I].SetupPtr=P.SuspensionConfig[I];
        }
        V.bSuspensionEnabled=P.After.Suspension;V.bMechanicalSimEnabled=P.After.Mechanical;
        V.bWheelFrictionEnabled=P.After.Friction;V.bLegacyWheelFrictionPosition=P.After.LegacyPosition;V.NumDrivenWheels=P.After.Driven;
        P.Origin=nullptr;return true;
    }
};
}
