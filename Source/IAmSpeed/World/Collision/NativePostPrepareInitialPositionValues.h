#pragma once
#include "NativePreparationPrefix.h"
#include "WorldStaticSphereGTInspection.h"
#include "CanonicalPreparedFrameClock.h"
namespace Speed {class IStaticCollisionWorld;}
namespace Speed::PreparationPrefix
{
// Member readback values only. Context is an observed fact, never a transferable
// setter capability. The private consumer must reread the live native context.
struct FPostPrepareInitialPositionSphereValues
{
    FContext Context;
    PreparedExecution::FCanonicalPreparedFrameClock Clock;
    const FPreparedNativeOwnerBinding* Binding=nullptr;
    const USSubBody* Body=nullptr;
    const FKinematicState* OwnerStorage=nullptr;
    const FKinematicState* OwnerFrameStorage=nullptr;
    const FKinematicState* BodyStorage=nullptr;
    const IStaticCollisionWorld* StaticWorld=nullptr;
    FWorldStaticSphereGTInspection Shape;
    FVector OwnerCenterOfMassLocal=FVector::ZeroVector;
    float StaticSupportFrameHorizon=0;
    bool OwnerPrescribed=false,OwnerSleeping=false,ConstraintsEmpty=false;
    FNativePrepareGTFields ExactFields;
    static uint64 DoubleBits(double value)
    {uint64 bits=0;FMemory::Memcpy(&bits,&value,8);return bits;}
    static uint32 FloatBits(float value)
    {uint32 bits=0;FMemory::Memcpy(&bits,&value,4);return bits;}
    bool SameValues(const FPostPrepareInitialPositionSphereValues& x) const
    {
        // Epoch changes at After. Value comparison is deliberately independent
        // of context authentication, which is performed fresh by the reader.
        return Clock==x.Clock&&Binding==x.Binding&&Body==x.Body&&OwnerStorage==x.OwnerStorage&&
            OwnerFrameStorage==x.OwnerFrameStorage&&BodyStorage==x.BodyStorage&&StaticWorld==x.StaticWorld&&
            Shape.ExactInitialOperationBits()==x.Shape.ExactInitialOperationBits()&&
            DoubleBits(OwnerCenterOfMassLocal.X)==DoubleBits(x.OwnerCenterOfMassLocal.X)&&
            DoubleBits(OwnerCenterOfMassLocal.Y)==DoubleBits(x.OwnerCenterOfMassLocal.Y)&&
            DoubleBits(OwnerCenterOfMassLocal.Z)==DoubleBits(x.OwnerCenterOfMassLocal.Z)&&
            FloatBits(StaticSupportFrameHorizon)==FloatBits(x.StaticSupportFrameHorizon)&&
            OwnerPrescribed==x.OwnerPrescribed&&OwnerSleeping==x.OwnerSleeping&&ConstraintsEmpty==x.ConstraintsEmpty&&
            ExactFields.Same(x.ExactFields);
    }
    bool HasSameContext(const FContext& x) const
    {
        const auto& a=Context;
        return a.Host==x.Host&&a.Bridge==x.Bridge&&a.Worker==x.Worker&&a.World==x.World&&a.Owner==x.Owner&&
            a.Generation==x.Generation&&a.Frame==x.Frame&&a.StepBefore==x.StepBefore&&a.WindowSerial==x.WindowSerial&&
            a.Nonce==x.Nonce&&a.Ordinal==x.Ordinal&&a.Epoch==x.Epoch&&a.ThreadId==x.ThreadId&&
            a.DeltaBits==x.DeltaBits&&a.TimeBits==x.TimeBits&&a.Effect==x.Effect&&a.Pass==x.Pass&&
            a.PublicationSerial==x.PublicationSerial&&a.FrameHashCount==x.FrameHashCount&&
            a.CallbackSucceeded==x.CallbackSucceeded&&a.NativeReturnValue==x.NativeReturnValue;
    }
};
}
