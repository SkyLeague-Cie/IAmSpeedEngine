#pragma once

// Source-only prototype: persistent Windows acquisition owner, independent of
// the short-lived car session. The first GetOrCreate runs on the acquisition
// worker; subsequent sessions retrieve the same callback catalogue.

#include "GameInputWarmCatalogue.h"
#include <exception>
#include <memory>
#include <mutex>

namespace Speed::Input::Windows
{
class FGameInputWarmContext final
{
    using FApi = GameInput::v3::IGameInput;
public:
    FGameInputWarmContext() = default;
    ~FGameInputWarmContext() { if (!Close()) std::terminate(); }
    FGameInputWarmContext(const FGameInputWarmContext&) = delete;
    FGameInputWarmContext& operator=(const FGameInputWarmContext&) = delete;

    std::shared_ptr<FGameInputWarmCatalogue> GetOrCreate(FApi* Api)
    {
        std::lock_guard<std::mutex> Lock(Gate);
        if (Closed) return {};
        if (!Catalogue)
        {
            if (!Api) return {};
            auto Created = FGameInputWarmCatalogue::Create(Api);
            if (!Created) return {};
            Catalogue = std::move(Created);
        }
        return Catalogue;
    }

    bool Close()
    {
        std::lock_guard<std::mutex> Lock(Gate);
        if (Closed) return true;
        if (Catalogue && !Catalogue->Shutdown()) return false;
        Catalogue.reset(); Closed = true; return true;
    }

private:
    std::mutex Gate;
    std::shared_ptr<FGameInputWarmCatalogue> Catalogue;
    bool Closed = false;
};
}
