#include "NativeRoadSettingsRead.h"
#include <utility>
namespace Speed::RoadSettings
{
namespace {thread_local const FDeclaredRecipe* OriginalSettings=nullptr;}
const FDeclaredRecipe* SelectedOriginalRoadSettings() noexcept {return OriginalSettings;}
FOriginalRoadSettingsScope::FOriginalRoadSettingsScope(std::shared_ptr<const FDeclaredRecipe> In)
    :Recipe(std::move(In)),Previous(OriginalSettings)
{
    if(!IsInGameThread()||(Previous&&Previous!=Recipe.get()))
        throw std::logic_error("original road settings scope conflicts with native owner");
    OriginalSettings=Recipe.get();
}
FOriginalRoadSettingsScope::~FOriginalRoadSettingsScope(){OriginalSettings=Previous;}
}
