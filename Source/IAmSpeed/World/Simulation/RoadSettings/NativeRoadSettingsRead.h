#pragma once
#include "CoreMinimal.h"
#include "DeclaredRoadSettingsRecipe.h"
#include <stdexcept>
namespace Speed::Netcode::Prediction {class FPrivateRoadPhysicsWorld;}
namespace Speed::RoadSettings
{
// Immutable value selection only. This scope grants no lifetime exclusion,
// network authority, complete configuration coverage or restoration lease.
IAMSPEED_API const FDeclaredRecipe* SelectedOriginalRoadSettings() noexcept;
class IAMSPEED_API FOriginalRoadSettingsScope final
{
    friend class Speed::Netcode::Prediction::FPrivateRoadPhysicsWorld;
    std::shared_ptr<const FDeclaredRecipe> Recipe;
    const FDeclaredRecipe* Previous=nullptr;
    explicit FOriginalRoadSettingsScope(std::shared_ptr<const FDeclaredRecipe> In);
public:
    ~FOriginalRoadSettingsScope();
    FOriginalRoadSettingsScope(const FOriginalRoadSettingsScope&)=delete;
    FOriginalRoadSettingsScope& operator=(const FOriginalRoadSettingsScope&)=delete;
};
template<std::size_t Index,class FLegacy> auto ReadOriginalOrLegacy(FLegacy&& Legacy)
{
    static_assert(Index<RequiredSlots);
    using T=std::remove_cvref_t<decltype(Legacy())>;
    static_assert(std::is_same_v<T,int32>||std::is_same_v<T,float>);
    if(const auto* Recipe=SelectedOriginalRoadSettings())
    {
        const auto Value=Recipe->Read<T>(Catalog[Index]);
        if(!Value)throw std::logic_error("original native road settings slot unavailable");
        return *Value; // NEVER fall back to live ConsoleManager in an epoch
    }
    return Legacy(); // original legacy expression remains lazy and unchanged
}
}
