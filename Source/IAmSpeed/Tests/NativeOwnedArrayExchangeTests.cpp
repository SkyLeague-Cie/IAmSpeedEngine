#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Netcode/Common/NativeOwnedArrayExchange.h"
#include "IAmSpeed/Netcode/Common/NativeCommonComponentAccess.h"
#include "IAmSpeed/Base/PhysicalContactConstraint.h"
#include "Misc/AutomationTest.h"

namespace NativeOwnedArrayExchangeTestDetail
{
struct FRetirementWitness
{
    int32& Retirements;
    const int32 Value;
    FRetirementWitness(int32& In,int32 V):Retirements(In),Value(V){}
    ~FRetirementWitness(){++Retirements;}
};
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASNativeOwnedArrayExchange,
    "IAmSpeed.Netcode.Prediction.NativeOwnedArrayExchange",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASNativeOwnedArrayExchange::RunTest(const FString&)
{
    using Speed::NativeState::ExchangeOwnedDefaultArrays;
    TestFalse(TEXT("this exact UE native TArray does not annotate the standard swap path noexcept"),std::is_nothrow_swappable_v<TArray<FPhysicalContactConstraint>>);
    TArray<FPhysicalContactConstraint> Actual,Prepared;Actual.Reserve(17);Prepared.Reserve(29);
    for(int32 N=0;N<2;++N){auto& C=Actual.AddDefaulted_GetRef();C.PairKey=101+N;C.PenetrationDepth=float(N+1);}
    for(int32 N=0;N<3;++N){auto& C=Prepared.AddDefaulted_GetRef();C.PairKey=201+N;C.PenetrationDepth=float(N+10);}
    const auto* OldData=Actual.GetData();const auto* NewData=Prepared.GetData();
    const auto OldMax=Actual.Max(),NewMax=Prepared.Max();const auto OldBytes=Actual.GetAllocatedSize(),NewBytes=Prepared.GetAllocatedSize();
    const TConstArrayView<FPhysicalContactConstraint> OldView(Actual);
    ExchangeOwnedDefaultArrays(Actual,Prepared);
    TestTrue(TEXT("actual UE native constraint buffers exchange ownership without reconstruction"),Actual.GetData()==NewData&&Prepared.GetData()==OldData&&Actual.Num()==3&&Prepared.Num()==2);
    TestTrue(TEXT("original native capacities and allocations travel with their own buffers"),Actual.Max()==NewMax&&Prepared.Max()==OldMax&&Actual.GetAllocatedSize()==NewBytes&&Prepared.GetAllocatedSize()==OldBytes);
    TestTrue(TEXT("old raw values view stays backed by retained old storage"),OldView.Num()==2&&OldView[0].PairKey==101&&OldView[1].PenetrationDepth==2);
    TestTrue(TEXT("new current values are the prepared native values"),Actual[0].PairKey==201&&Actual[2].PenetrationDepth==12);
    ExchangeOwnedDefaultArrays(Actual,Actual);
    TestTrue(TEXT("self exchange does not retire or replace the native buffer"),Actual.GetData()==NewData&&Actual.Num()==3&&Actual[2].PairKey==203);
    int32 Retirements=0;
    {
        using FSentinel=NativeOwnedArrayExchangeTestDetail::FRetirementWitness;
        TArray<TSharedPtr<FSentinel>> Current,Staged;Current.Reserve(5);Staged.Reserve(7);
        Current.Add(MakeShared<FSentinel>(Retirements,11));Current.Add(MakeShared<FSentinel>(Retirements,12));
        Staged.Add(MakeShared<FSentinel>(Retirements,21));
        const auto* OldOwners=Current.GetData();const auto* NewOwners=Staged.GetData();
        TWeakPtr<FSentinel> OldWitness=Current[0];const int32 BeforeRefs=Current[0].GetSharedReferenceCount();
        ExchangeOwnedDefaultArrays(Current,Staged);
        TestTrue(TEXT("exchange invokes no actual element retirement or refcount ownership loss"),Retirements==0&&OldWitness.IsValid()&&Staged.GetData()==OldOwners&&Current.GetData()==NewOwners&&Staged[0].GetSharedReferenceCount()==BeforeRefs);
        TestTrue(TEXT("old prepared owner retains the exact original objects"),Staged[0]->Value==11&&Staged[1]->Value==12&&Current[0]->Value==21);
        Staged.Reset();
        TestTrue(TEXT("old values retire only when their retained owner is explicitly retired later"),Retirements==2&&!OldWitness.IsValid()&&Current[0]->Value==21);
    }
    TestEqual(TEXT("all remaining native test owners eventually retire"),Retirements,3);
    if(HasAnyErrors())return false;
    AddInfo(FString::Printf(TEXT("[NativeOwnedArrayExchange] actualNative=1 constraintBuffersPreserved=1 oldValuesViewBacked=1 elementRetirementsDuringExchange=0 stdNothrowArraySwap=%d worldLeaseCertificate=0 completeComponentProvider=0"),std::is_nothrow_swappable_v<TArray<FPhysicalContactConstraint>>?1:0));
    AddInfo(TEXT("Native default-heap TArray storage ownership only. Keeping old storage alive does NOT authorize an escaped mutable reader. Real component/configuration/source/lifetime leases and global access validation remain required; no complete restore or game admission."));
    return true;
}
#endif
