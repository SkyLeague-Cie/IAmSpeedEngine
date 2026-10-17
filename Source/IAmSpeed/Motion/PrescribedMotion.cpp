#include "PrescribedMotion.h"
#include <limits>
namespace Speed::Motion
{
bool FPrescribedMotionTrack::Configure(TConstArrayView<FPrescribedMotionKey> InKeys,const FVector3d& LocalAxis,bool bInLoop)
{
    // Transactional: invalid replacement leaves the prior admitted track intact.
    if(InKeys.Num()<2||InKeys.Num()>256||LocalAxis.ContainsNaN()||FMath::Abs(LocalAxis.SizeSquared()-1.)>1.e-10)return false;
    if(InKeys[0].Seconds!=0.)return false;
    for(int32 I=0;I<InKeys.Num();++I)
    {
        const auto& K=InKeys[I];
        if(!FMath::IsFinite(K.Seconds)||K.Seconds<0.||K.Seconds>86400.||K.Translation.ContainsNaN()||K.Translation.GetAbsMax()>200000.||!FMath::IsFinite(K.AngleRadians)||FMath::Abs(K.AngleRadians)>10000.)return false;
        if(I&&K.Seconds-InKeys[I-1].Seconds<1.e-6)return false;
    }
    if(bInLoop)
    {
        const auto& A=InKeys[0];const auto& B=InKeys.Last();
        if(!A.Translation.Equals(B.Translation,1.e-8)||FMath::Abs(FMath::Sin((B.AngleRadians-A.AngleRadians)*.5))>1.e-10)return false;
    }
    Keys.Reset(InKeys.Num());Keys.Append(InKeys.GetData(),InKeys.Num());Axis=LocalAxis;bLoop=bInLoop;return true;
}
bool FPrescribedMotionTrack::HasCanonicalKeyTimes() const
{
    if(!IsConfigured())return false;
    for(const auto& K:Keys)if(FMath::Abs(K.Seconds*300.-FMath::RoundToDouble(K.Seconds*300.))>1.e-8)return false;
    return true;
}
bool FPrescribedMotionTrack::Evaluate(double Seconds,FPrescribedMotionSample& Out) const
{
    Out={};if(!IsConfigured()||!FMath::IsFinite(Seconds)||Seconds<0.)return false;
    const double Duration=Keys.Last().Seconds;
    double T=bLoop?FMath::Fmod(Seconds,Duration):FMath::Min(Seconds,Duration);
    if(!bLoop&&T>=Duration)
    {Out.Translation=Keys.Last().Translation;Out.Rotation=FQuat4d(Axis,Keys.Last().AngleRadians);Out.SecondsToBoundary=std::numeric_limits<double>::max();return true;}
    int32 Lo=0,Hi=Keys.Num()-1;
    while(Lo+1<Hi){const int32 Mid=(Lo+Hi)/2;if(Keys[Mid].Seconds<=T)Lo=Mid;else Hi=Mid;}
    const auto& A=Keys[Lo];const auto& B=Keys[Lo+1];const double D=B.Seconds-A.Seconds,U=(T-A.Seconds)/D;
    Out.Translation=A.Translation+(B.Translation-A.Translation)*U;
    Out.Velocity=(B.Translation-A.Translation)/D;
    Out.Rotation=FQuat4d(Axis,A.AngleRadians+(B.AngleRadians-A.AngleRadians)*U);
    Out.AngularVelocity=Axis*((B.AngleRadians-A.AngleRadians)/D);
    Out.SecondsToBoundary=B.Seconds-T;return true;
}
bool FPrescribedMotionTrack::EvaluateCanonicalTicks(double ActiveTicks,FPrescribedMotionSample& Out) const
{
    Out={};if(!HasCanonicalKeyTimes()||!FMath::IsFinite(ActiveTicks)||ActiveTicks<0.||ActiveTicks>double(std::numeric_limits<uint32>::max()))return false;
    // Key addresses were admitted on integer300Hz ticks. Reduce the integer
    // phase before division: seconds/fmod can land just below a velocity key
    // after many loops and make CCD predict the wrong direction for a frame.
    const double Duration=FMath::RoundToDouble(Keys.Last().Seconds*300.);
    const double T=bLoop?FMath::Fmod(ActiveTicks,Duration):FMath::Min(ActiveTicks,Duration);
    if(!bLoop&&T>=Duration)
    {Out.Translation=Keys.Last().Translation;Out.Rotation=FQuat4d(Axis,Keys.Last().AngleRadians);Out.SecondsToBoundary=std::numeric_limits<double>::max();return true;}
    int32 Lo=0,Hi=Keys.Num()-1;
    while(Lo+1<Hi){const int32 Mid=(Lo+Hi)/2;if(FMath::RoundToDouble(Keys[Mid].Seconds*300.)<=T)Lo=Mid;else Hi=Mid;}
    const auto& A=Keys[Lo];const auto& B=Keys[Lo+1];const double Start=FMath::RoundToDouble(A.Seconds*300.),End=FMath::RoundToDouble(B.Seconds*300.);
    if(End<=Start)return false;const double U=(T-Start)/(End-Start),Rate=300./(End-Start);
    Out.Translation=A.Translation+(B.Translation-A.Translation)*U;Out.Velocity=(B.Translation-A.Translation)*Rate;
    Out.Rotation=FQuat4d(Axis,A.AngleRadians+(B.AngleRadians-A.AngleRadians)*U);Out.AngularVelocity=Axis*((B.AngleRadians-A.AngleRadians)*Rate);Out.SecondsToBoundary=(End-T)/300.;return true;
}
bool FStageMotionClock::Advance(uint64 Frame,bool bCarInside,bool bPaused)
{
    if(bHasFrame&&(LastFrame==std::numeric_limits<uint64>::max()||Frame!=LastFrame+1))return false;
    if(bCarInside&&!bPaused&&ActiveTicks==std::numeric_limits<uint64>::max())return false;
    LastFrame=Frame;bHasFrame=true;bInside=bCarInside;
    if(bCarInside&&!bPaused)++ActiveTicks;
    return true;
}
}
