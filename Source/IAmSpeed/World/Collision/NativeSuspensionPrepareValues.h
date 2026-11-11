#pragma once
#include "NativePrepareGTInspection.h"
#include "SuspensionSystem.h"
#include <type_traits>
namespace Speed::PreparationValues
{
// A real copy of the base subobject, never a downcast of the live simulation.
// No GetSpringLength/GetSuspensionOffset: those getters mutate its smoothing ring.
class FSuspensionCopy final:public Chaos::FSimpleSuspensionSim
{
public:
    explicit FSuspensionCopy(const Chaos::FSimpleSuspensionSim& source):Chaos::FSimpleSuspensionSim(source){}
    bool Append(const FString& prefix,FNativePrepareGTFields& out,Analytic::Certified::FDistanceBudget& budget)const
    {
        if(!SetupPtr)return false;
        bool ok=true;
        const auto scalar=[&](const TCHAR* n,auto v){ok=out.Scalar(prefix+n,v,budget)&&ok;};
        scalar(TEXT(".displacement_input"),DisplacementInput);scalar(TEXT(".last_displacement"),LastDisplacement);
        scalar(TEXT(".force"),SuspensionForce);scalar(TEXT(".spring_displacement"),SpringDisplacement);
        scalar(TEXT(".spring_index"),SpringIndex);scalar(TEXT(".last_spring_length"),LastSpringLength);
        scalar(TEXT(".averaging_count"),AveragingCount);scalar(TEXT(".averaging_num"),AveragingNum);
        ok=out.Vector(prefix+TEXT(".local_velocity"),LocalVelocity,budget)&&ok;
        ok=out.Vector(prefix+TEXT(".local_offset"),LocalOffset,budget)&&ok;
        scalar(TEXT(".averaging_length_count"),UE_ARRAY_COUNT(AveragingLength));
        for(int32 i=0;i<int32(UE_ARRAY_COUNT(AveragingLength));++i)
            ok=out.Float(prefix+FString::Printf(TEXT(".averaging_length[%d]"),i),AveragingLength[i],budget)&&ok;
        const auto& s=Setup();
        ok=out.Vector(prefix+TEXT(".setup.axis"),s.SuspensionAxis,budget)&&ok;
        ok=out.Vector(prefix+TEXT(".setup.force_offset"),s.SuspensionForceOffset,budget)&&ok;
        scalar(TEXT(".setup.max_raise"),s.SuspensionMaxRaise);
        scalar(TEXT(".setup.max_drop"),s.SuspensionMaxDrop);
        scalar(TEXT(".setup.max_length"),s.MaxLength);
        scalar(TEXT(".setup.spring_rate"),s.SpringRate);
        scalar(TEXT(".setup.spring_preload"),s.SpringPreload);
        scalar(TEXT(".setup.compression_damping"),s.CompressionDamping);
        scalar(TEXT(".setup.rebound_damping"),s.ReboundDamping);
        scalar(TEXT(".setup.resting_force"),s.RestingForce);
        scalar(TEXT(".setup.swaybar"),s.Swaybar);
        scalar(TEXT(".setup.damping_ratio"),s.DampingRatio);
        scalar(TEXT(".setup.wheel_load_ratio"),s.WheelLoadRatio);
        scalar(TEXT(".setup.raycast_safety_margin"),s.RaycastSafetyMargin);
        scalar(TEXT(".setup.smoothing"),s.SuspensionSmoothing);
        return ok;
    }
};
static_assert(std::is_copy_constructible_v<Chaos::FSimpleSuspensionSim>);
static_assert(std::is_trivially_destructible_v<FSuspensionCopy>);
}
