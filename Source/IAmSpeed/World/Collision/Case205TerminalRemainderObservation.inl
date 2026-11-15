// Private refusal observer. Included by the existing Prepared execution TU.
#if !UE_BUILD_SHIPPING
#include "IAmSpeed/World/Simulation/FrameFloatingPointContext.h"

namespace Speed::Analytic::Certified::Case205RemainderObservation
{
namespace A=::AnalyticCertified;
namespace P=Speed::PreparedExecution;
static TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.Case205TerminalRemainderBits"),0,
    TEXT("Private diagnostic of native rounded Box vertices after an already terminal case 205 refusal. 0=off, 1=short, 2=finite band."));
struct FRecord
{
    FGeneralBoxMotionInput Motion;
    P::FStepIdentity Identity;
    int32 Mode=0;
};
thread_local TOptional<FRecord> Pending;

bool MatchesCapture(const FGeneralBoxMotionInput& input)
{
    const auto& owner=input.Owner;
    return KinematicCenterDetail::Bits(input.Horizon)==957529968u&&
        owner.Location==FVector3d(-1217.30712391004,3998.8815367020279,157.59399096002531)&&
        owner.Rotation==FQuat4d(-0.0093013589099853813,0.0093074773807651969,0.71029765770529874,0.70377844031030146)&&
        owner.Velocity==FVector3d(14.954081368986765,2271.2479333116753,128.13044612107942)&&
        owner.Acceleration==FVector3d(0,0,-650)&&
        owner.AngularVelocity==FVector3d(-0.016972486017302446,-0.0088031547932911156,0.0025884968657810606)&&
        owner.AngularAcceleration==FVector3d(0.065239776136567104,6.6850512676875059e-05,-0.020333432274189628)&&
        input.LocalCOM==FVector3d(-14.008570000000001,0,17.415800000000001)&&
        input.LocalOffset==FVector3d(-5,0,29.510000000000002)&&input.LocalRotation==FQuat4d::Identity&&
        input.PhysicalHalfExtent==FVector3d(65.659999999999997,43.5852,15.9472)&&
        input.QueryHalfExtent==FVector3d(65.660003662109375,43.585201263427734,15.947199821472168);
}

void Capture(const FGeneralBoxMotionInput& input,const AffineReferenceDetail::FProduct& product,
    float first,float last) noexcept
{
    const int32 mode=Enabled.GetValueOnAnyThread();
    if((mode!=1&&mode!=2)||Pending.IsSet()||!MatchesCapture(input)||
        KinematicCenterDetail::Bits(first)!=0x2f343170u||KinematicCenterDetail::Bits(last)!=0x2f343171u)return;
    P::FStepIdentity step;
    Speed::FPreparedWorldSnapshotPtr snapshot;
    if(!P::ReadCurrentStep(step)||(step.Frame!=219&&(mode!=2||step.Frame!=6386))||
        step.Iteration!=2||!step.QuerySerial||(mode==2&&step.QuerySerial!=2)||
        step.ThreadId!=FPlatformTLS::GetCurrentThreadId()||!P::ReadCurrentSnapshot(snapshot)||!snapshot||
        !snapshot->GetGeometrySelection()||!snapshot->GetGeometrySelection()->HasValidAliases())return;
    const auto& reference=snapshot->GetReference();
    if(!reference||reference->GetExternalPacketIdentity()!=TEXT("090d41158d13f1127827b720ff467a51e415668aa205209a67ed8afcb6b10dc0"))return;
    for(const auto& fact:reference->GetExactAffineExtrusions(1407))
    {
        if(&fact.Product==&product&&fact.Provider==1407&&fact.Product.FreeWorldAxis==0&&fact.Product.FreeAlongU)
        {
            Pending.Emplace(FRecord{input,step,mode});
            return;
        }
    }
}

// Run only after the ordinary step teardown has independently made poison
// sticky and detached StepScope while an original exception is unwinding.
// Successful/fallback steps discard the capture without running arithmetic.
void Drain(const P::FStepIdentity& step,bool terminal) noexcept
{
    if(!Pending.IsSet())return;
    const FRecord saved=Pending.GetValue();
    Pending.Reset();
    const auto& key=saved.Identity;
    if(!terminal||key.Frame!=step.Frame||key.StepSerial!=step.StepSerial||key.QuerySerial!=step.QuerySerial||
        key.QueryEpoch!=step.QueryEpoch||key.Iteration!=step.Iteration||key.WindowNonce!=step.WindowNonce||
        key.Provider!=step.Provider||key.Worker!=step.Worker||key.ThreadId!=step.ThreadId||
        step.ThreadId!=FPlatformTLS::GetCurrentThreadId()||key.NumericControl!=step.NumericControl)return;
    const auto unavailable=[&](const TCHAR* stage) noexcept
    {
        try{UE_LOG(LogTemp,Display,TEXT("[Case205TerminalRemainderBits] Stage=%s Frame=%llu Serial=%llu Result=Unavailable"),
            stage,key.Frame,key.QuerySerial);}catch(...){}
    };
    try
    {
        const auto numeric=[&]()
        {
            const auto fp=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
            return fp.Status==EAnalyticNumericStatus::Complete&&fp.ControlFingerprint()==key.NumericControl;
        };
        const auto fp=CaptureFrameFloatingPointContext();
        if(!fp.bValid||!numeric()){unavailable(TEXT("numeric_precheck"));return;}
        FScopedFrameFloatingPointContext restore(fp);
        if(!restore.IsApplied()){unavailable(TEXT("fp_scope"));return;}
        const uint32 first=saved.Mode==2?0x2f343200u:0x2f343418u;
        const uint32 last=saved.Mode==2?0x2f343425u:0x2f343426u;
        struct FRow {FVector Center,Corner1,Corner5;FQuat Rotation;};
        TArray<FRow> rows;
        rows.SetNum(int32(last-first+1));
        for(uint32 word=first;word<=last;++word)
        {
            if(!numeric()){unavailable(TEXT("numeric_before_sample"));return;}
            const auto& input=saved.Motion;
            auto pose=input.Owner.Integrate(KinematicCenterDetail::FromBits(word));
            pose.Location-=pose.Rotation.RotateVector(input.LocalCOM);
            pose.Location+=pose.Rotation.RotateVector(input.LocalOffset);
            pose.Rotation=pose.Rotation*input.LocalRotation;
            const auto& extent=input.PhysicalHalfExtent;
            auto& row=rows[word-first];
            row.Center=pose.Location;row.Rotation=pose.Rotation;
            row.Corner1=pose.Location+pose.Rotation.RotateVector(FVector(extent.X,-extent.Y,-extent.Z));
            row.Corner5=pose.Location+pose.Rotation.RotateVector(FVector(extent.X,-extent.Y,extent.Z));
            if(!numeric()||row.Center.ContainsNaN()||row.Rotation.ContainsNaN()||
                row.Corner1.ContainsNaN()||row.Corner5.ContainsNaN())
            {unavailable(TEXT("numeric_after_sample"));return;}
        }
        if(!restore.Restore()||!numeric()){unavailable(TEXT("fp_restore"));return;}
        // No partial sample rows are emitted on a numeric or arithmetic failure.
        if(saved.Mode==2)
            UE_LOG(LogTemp,Display,TEXT("[Case205TerminalRemainderBits] Stage=range Frame=%llu Serial=%llu FirstBits=%08x ReferenceClearBits=2f343419 LastBits=%08x Numeric=%016llx"),
                key.Frame,key.QuerySerial,first,last,key.NumericControl);
        for(uint32 word=first;word<=last;++word)
        {
            const auto& r=rows[word-first];
            UE_LOG(LogTemp,Display,TEXT("[Case205TerminalRemainderBits] Stage=sample Frame=%llu Serial=%llu TimeBits=%08x Center=%016llx,%016llx,%016llx Rotation=%016llx,%016llx,%016llx,%016llx Corner1=%016llx,%016llx,%016llx Corner5=%016llx,%016llx,%016llx"),
                key.Frame,key.QuerySerial,word,A::Bits(r.Center.X),A::Bits(r.Center.Y),A::Bits(r.Center.Z),
                A::Bits(r.Rotation.X),A::Bits(r.Rotation.Y),A::Bits(r.Rotation.Z),A::Bits(r.Rotation.W),
                A::Bits(r.Corner1.X),A::Bits(r.Corner1.Y),A::Bits(r.Corner1.Z),A::Bits(r.Corner5.X),A::Bits(r.Corner5.Y),A::Bits(r.Corner5.Z));
        }
        UE_LOG(LogTemp,Display,TEXT("[Case205TerminalRemainderBits] Stage=complete Frame=%llu Serial=%llu Samples=%u"),
            key.Frame,key.QuerySerial,last-first+1);
    }
    catch(...){unavailable(TEXT("native_evaluation_exception"));}
}
}
#endif
