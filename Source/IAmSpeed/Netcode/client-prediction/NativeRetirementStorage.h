#pragma once
// Original native storage lifetime: memory lifetime only, NEVER a native/source/restore lease.
#include "CoreMinimal.h"
#include "WheelSystem.h"
#include "SuspensionSystem.h"
#include "PrivateRoadNativeObjects.h"
#include "NativeConfigurationLifetimeDiagnostics.h"
#include <memory>
#include <tuple>
#include <type_traits>
#include <utility>
namespace Speed::Netcode::Prediction::RetirementDraft {
enum class EPhase : uint8 {
    Open, AdmissionClosed, RegistryReturned, WorldCallbacksEntered,
    WorldCallbacksReturned, SurvivingFacadesDetached, NativeReferencesReleased,
    Retired, Interrupted
};
struct FVehicleConfiguration final {
    std::shared_ptr<FNativeConfigurationLifetimeTrace> Lifetime=std::make_shared<FNativeConfigurationLifetimeTrace>();
    Chaos::FSimpleWheelConfig Wheel;
    Chaos::FSimpleSuspensionConfig Suspension;
    ~FVehicleConfiguration()
    {
        if(Lifetime->NativeConstructionReturns.load()!=Lifetime->NativeBaseDestructionReturns.load())
            Lifetime->ConfigurationDestructionBeforeNativeReturn.store(true);
        ++Lifetime->ConfigurationDestructionEntries;
    }
};
// Supply the actual private FOriginalWheeledDomain from the owner friend context.
// Domain owns native VehicleSimulationPT. Config must outlive that destructor.
template<class TDomain> struct TVehicleRoot final {
    std::shared_ptr<FVehicleConfiguration> Configuration; // destroyed LAST
    std::shared_ptr<TDomain> Domain;                      // destroyed FIRST
    TVehicleRoot(std::shared_ptr<FVehicleConfiguration> C,std::shared_ptr<TDomain> D)
      :Configuration(std::move(C)),Domain(std::move(D)) {}
};
// Existing private allocations are MOVED here, not copied from facade/CDO.
// UObject fields become aliasing shared_ptr into this root. Any surviving body
// alias retains its actual backend/config even if the component UObject dies.
template<class TVehicle,class... TPart> struct TBodyRoot final {
    std::shared_ptr<TVehicle> Vehicle;                     // destroyed LAST
    std::tuple<std::shared_ptr<TPart>...> Parts;           // destroyed FIRST
    TBodyRoot(std::shared_ptr<TVehicle> V,std::shared_ptr<TPart>... P)
      :Vehicle(std::move(V)),Parts(std::move(P)...) {}
};
template<class TRoot,class TPart>
std::shared_ptr<TPart> AliasExistingPart(const std::shared_ptr<TRoot>& Root,
                                      const std::shared_ptr<TPart>& Part) {
    if(!Root||!Part)return {};
    return std::shared_ptr<TPart>(Root,Part.get()); // same REAL root control block
}
template<size_t Index,class TRoot>
auto AliasBodyPart(const std::shared_ptr<TRoot>& Root) {
    return AliasExistingPart(Root,std::get<Index>(Root->Parts));
}
// For the reference/facade simulation still uniquely owned by its UObject:
// config must remain alive THROUGH the native base destructor. A derived
// shared_ptr MEMBER alone is destroyed too early. Base order is intentional.
template<class TConfig> struct TConfigurationLifetime {
    std::shared_ptr<TConfig> ActualConfiguration;
    bool ActualNativeConstructionReturned=false;
    explicit TConfigurationLifetime(std::shared_ptr<TConfig> C):ActualConfiguration(std::move(C)) {}
    ~TConfigurationLifetime()
    {
        // This base is destroyed AFTER the actual native simulation base.
        // A failed native constructor never records a destructor return.
        if(ActualNativeConstructionReturned)++ActualConfiguration->Lifetime->NativeBaseDestructionReturns;
    }
};
template<class TNativeSimulation,class TConfig>
class TSimulationWithConfiguration final
  : private TConfigurationLifetime<TConfig>, public TNativeSimulation {
    static_assert(std::has_virtual_destructor_v<TNativeSimulation>,
                  "Base unique_ptr deletion must execute the real derived destructor");
public:
    template<class... TArg>
    TSimulationWithConfiguration(std::shared_ptr<TConfig> C,TArg&&... Arg)
      :TConfigurationLifetime<TConfig>(std::move(C)),TNativeSimulation(std::forward<TArg>(Arg)...)
    {
        this->ActualNativeConstructionReturned=true;
        ++this->ActualConfiguration->Lifetime->NativeConstructionReturns;
    }
    // reverse BASE destruction: native simulation first, configuration last.
};
// These helpers mutate ONLY retained C++ parts, after callbacks returned.
// False means missing/wrong backing, never a fabricated drain observation.
template<class TWheelRoot,class TVehicleRoot>
bool DisconnectWheelRoot(TWheelRoot& Root,const std::shared_ptr<TVehicleRoot>& Vehicle) {
    if(!Vehicle||Root.Vehicle!=Vehicle)return false;
    auto& Body=std::get<0>(Root.Parts);auto& Wheel=std::get<3>(Root.Parts);
    if(!Body||!Wheel)return false;
    Wheel->PWheel=nullptr;Wheel->PSuspension=nullptr;Wheel->ChaosWheel=nullptr;
    Wheel->WheelComponent=nullptr;Body->ParentComponent=nullptr;
    Body->AlwaysIgnoredComponents.Empty();Body->IgnoredComponents.Empty();
    return true;
}
template<class TBoxRoot,class TVehicleRoot>
bool DisconnectBoxRoot(TBoxRoot& Root,const std::shared_ptr<TVehicleRoot>& Vehicle) {
    if(!Vehicle||Root.Vehicle!=Vehicle)return false;
    auto& Body=std::get<0>(Root.Parts);if(!Body)return false;
    Body->ParentComponent=nullptr;Body->AlwaysIgnoredComponents.Empty();Body->IgnoredComponents.Empty();
    return true;
}
template<class TVehicleRoot>
bool ReleaseNativeVehicle(TVehicleRoot& Root) {
    if(!Root.Domain||!Root.Configuration)return false;
    Root.Domain->SkySimulation=nullptr;
    Root.Domain->VehicleSimulationPT.Reset(); // actual native destructor, configs still owned
    return true;
}

}
