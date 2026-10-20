#pragma once
#include "CoreMinimal.h"
namespace Speed::Motion
{
/** Numeric, worker-owned trajectory. Translation and a single local-axis angle
 * are linear between keys. Unwrapped angles preserve full revolutions. */
struct FPrescribedMotionKey { double Seconds=0.; FVector3d Translation=FVector3d::ZeroVector; double AngleRadians=0.; };
struct FPrescribedMotionSample
{
    FVector3d Translation=FVector3d::ZeroVector, Velocity=FVector3d::ZeroVector;
    FQuat4d Rotation=FQuat4d::Identity;
    FVector3d AngularVelocity=FVector3d::ZeroVector;
    double SecondsToBoundary=0.;
};
class IAMSPEED_API FPrescribedMotionTrack
{
public:
    bool Configure(TConstArrayView<FPrescribedMotionKey> InKeys,const FVector3d& LocalAxis,bool bInLoop);
    bool Evaluate(double Seconds,FPrescribedMotionSample& Out) const;
    bool HasCanonicalKeyTimes() const;
    bool EvaluateCanonicalTicks(double ActiveTicks,FPrescribedMotionSample& Out) const;
    bool IsConfigured() const { return Keys.Num()>=2; }
    FBox3d GetTranslationBounds() const { FBox3d Bounds(ForceInit);for(const auto& Key:Keys)Bounds+=Key.Translation;return Bounds; }
private:
    TArray<FPrescribedMotionKey> Keys;
    FVector3d Axis=FVector3d(0.,0.,1.);
    bool bLoop=false;
};
/** No wall-clock, actor tick or UObject dependency. Entry/exit are canonical
 * frame inputs. Exit and pause hold the phase; re-entry resumes that phase. */
struct IAMSPEED_API FStageMotionClock
{
    uint64 ActiveTicks=0, LastFrame=0;
    bool bHasFrame=false, bInside=false;
    bool Advance(uint64 Frame,bool bCarInside,bool bPaused);
    double Seconds() const { return double(ActiveTicks)/300.; }
};
}
