#pragma once
#include "CoreMinimal.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SphereSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "IAmSpeed/SubBodies/Configs/SubBodyConfig.h"
#include "IAmSpeed/SubBodies/Configs/WheelSubBodyConfig.h"
#include <memory>
#include "PrivateRoadBirthFixtureObjects.generated.h"

// Private real UCLASS fixtures. They are never admitted into the sealed native
// road graph and expose no native operation, original part or restore authority.
struct FIASBirthFixtureInverseObservation
{
    float Mass=0;FVector Shape=FVector::ZeroVector;FMatrix Returned=FMatrix::Identity;
};
struct FIASBirthFixtureTrace
{
    TArray<FString> Events;
    TArray<FIASBirthFixtureInverseObservation> Inverse;
    uint32 SubBodyQueries=0,WheelQueries=0;
    FMatrix Observe(float Mass,const FVector& Shape)
    {
        Events.Add(TEXT("inverse"));FMatrix Matrix=FMatrix::Identity;
        Matrix.M[0][0]=double(Inverse.Num()+11); // distinguish actual virtual calls
        Inverse.Add({Mass,Shape,Matrix});return Matrix;
    }
};
UCLASS(Transient,NotBlueprintable)
class UIASBirthFixtureComponent final : public USpeedWheeledComponent
{
    GENERATED_BODY()
public:
    explicit UIASBirthFixtureComponent(const FObjectInitializer& Initializer=FObjectInitializer::Get()):Super(Initializer){}
    std::shared_ptr<FIASBirthFixtureTrace> Trace;
    SubBodyConfig BodyConfig;
    WheelSubBodyConfig WheelConfig;
    SubBodyConfig GetSubBodyConfig(const USSubBody&) const override
    {if(Trace){++Trace->SubBodyQueries;Trace->Events.Add(TEXT("subbody"));}return BodyConfig;}
    WheelSubBodyConfig GetWheelSubBodyConfig(const USWheelSubBody&) const override
    {if(Trace){++Trace->WheelQueries;Trace->Events.Add(TEXT("wheel"));}return WheelConfig;}
    void SetBodiesForFixture(USSubBody* Body){SubBodiesWheeledDomainStorage().Reset();if(Body)SubBodiesWheeledDomainStorage().Add(Body);}
};
UCLASS(Transient,NotBlueprintable)
class UIASBirthFixtureBox final : public UBoxSubBody
{
    GENERATED_BODY()
    FMatrix InitInvInertiaTensor() const override
    {return Trace?Trace->Observe(MassSolidStorage(),GetBoxExtent()):FMatrix::Identity;}
public:
    explicit UIASBirthFixtureBox(const FObjectInitializer& Initializer=FObjectInitializer::Get()):Super(Initializer){}
    std::shared_ptr<FIASBirthFixtureTrace> Trace;
    FMatrix ReadInverseForFixture() const {return InvInertiaLocalSolidStorage();}
    float ReadMassForFixture() const {return MassSolidStorage();}
    int32 ReadIgnoredCountForFixture() const {return AlwaysIgnoredComponentsBodyDomainStorage().Num();}
};
UCLASS(Transient,NotBlueprintable)
class UIASBirthFixtureSphere final : public USphereSubBody
{
    GENERATED_BODY()
    FMatrix InitInvInertiaTensor() const override
    {return Trace?Trace->Observe(MassSolidStorage(),FVector(GetRadius(),0,0)):FMatrix::Identity;}
public:
    explicit UIASBirthFixtureSphere(const FObjectInitializer& Initializer=FObjectInitializer::Get()):Super(Initializer){}
    std::shared_ptr<FIASBirthFixtureTrace> Trace;
    FMatrix ReadInverseForFixture() const {return InvInertiaLocalSolidStorage();}
    float ReadMassForFixture() const {return MassSolidStorage();}
    int32 ReadIgnoredCountForFixture() const {return AlwaysIgnoredComponentsBodyDomainStorage().Num();}
};
UCLASS(Transient,NotBlueprintable)
class UIASBirthFixtureWheel final : public USWheelSubBody
{
    GENERATED_BODY()
    FMatrix InitInvInertiaTensor() const override
    {return Trace?Trace->Observe(MassSolidStorage(),FVector(GetRadius(),0,0)):FMatrix::Identity;}
public:
    explicit UIASBirthFixtureWheel(const FObjectInitializer& Initializer=FObjectInitializer::Get()):Super(Initializer){}
    std::shared_ptr<FIASBirthFixtureTrace> Trace;
    FMatrix ReadInverseForFixture() const {return InvInertiaLocalSolidStorage();}
    float ReadMassForFixture() const {return MassSolidStorage();}
    // Actual ordinary legacy fixture value COPY only. This final test class
    // is never admitted as a private native road body and grants no native
    // operation, mutable borrow, source/lifetime or restore authority.
    Speed::FKinematicState CopyKinematicsForFixture() const {return KinematicsStorage();}
    void AuthorWheelBindingFixture(float InWheelMass,float InMaxRaise,float InMaxDrop)
    {WheelMassWheelDomainStorage()=InWheelMass;SuspensionMaxRaiseOverrideWheelDomainStorage()=InMaxRaise;SuspensionMaxDropOverrideWheelDomainStorage()=InMaxDrop;}

    int32 ReadIgnoredCountForFixture() const {return AlwaysIgnoredComponentsBodyDomainStorage().Num();}
};
