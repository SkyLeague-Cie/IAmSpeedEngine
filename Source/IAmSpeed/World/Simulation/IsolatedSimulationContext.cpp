#include "IsolatedSimulationContext.h"
#include "FastSimulation.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "IAmSpeed/World/Analytic/StaticWorldQueryAudit.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "Components/BoxComponent.h"

TUniquePtr<FIsolatedSimulationContext> FIsolatedSimulationContext::Create(const FIsolatedStaticWorldSeed& Seed)
{
    check(IsInGameThread());
    if (!GEngine || !Seed.Geometry || Seed.Sources.Num() > 512 ||
        !Speed::Analytic::FStaticWorldQueryAudit::IsSurfaceAnalyticBackend()) return nullptr;
    TUniquePtr<FIsolatedSimulationContext> Context(new FIsolatedSimulationContext);
    const auto Options = UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(false)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false)
        .ShouldSimulatePhysics(false).SetTransactional(false);
    UWorld* W = UWorld::CreateWorld(EWorldType::EditorPreview, false, NAME_None, nullptr, true,
        ERHIFeatureLevel::Num, &Options);
    if (!W) return nullptr;
    Context->World.Reset(W);
    GEngine->CreateNewWorldContext(EWorldType::EditorPreview).SetCurrentWorld(W);
    AActor* ProxyOwner = W->SpawnActor<AActor>();
    if (!ProxyOwner) return nullptr;
    for (const FIsolatedStaticSource& Source : Seed.Sources)
    {
        if (!Source.SourceId || Context->Sources.Contains(Source.SourceId) || Source.Transform.ContainsNaN() || Source.ObjectType>=ECC_MAX) return nullptr;
        UBoxComponent* Proxy = NewObject<UBoxComponent>(ProxyOwner);
        if (!Proxy) return nullptr;
        ProxyOwner->AddInstanceComponent(Proxy);
        Proxy->SetMobility(EComponentMobility::Static);
        Proxy->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Proxy->SetCollisionObjectType(ECollisionChannel(Source.ObjectType));
        Proxy->SetWorldTransform(Source.Transform);
        Proxy->RegisterComponent();
        Context->Sources.Add(Source.SourceId, Proxy);
        Context->SourceIds.Add(Proxy, Source.SourceId);
    }
    auto* Bridge = W->GetSubsystem<USpeedWorldSubsystem>();
    if (!Bridge || !Bridge->InstallIsolatedStaticWorldSeed(Seed, Context->Sources)) return nullptr;
    Context->Driver = W->SpawnActor<AFastSimulation>();
    if (!Context->Driver || !Context->Driver->InitializeIsolatedCanonicalHost()) return nullptr;
    return Context;
}

FIsolatedSimulationContext::~FIsolatedSimulationContext() { Close(); }
void FIsolatedSimulationContext::Close()
{
    check(IsInGameThread()); // caller has suspended/joined the only canonical owner
    if (!World.IsValid()) return;
    UWorld* W = World.Get();
    Driver = nullptr;
    Sources.Reset(); SourceIds.Reset();
    W->DestroyWorld(false);
    if (GEngine) GEngine->DestroyWorldContext(W);
    W->RemoveFromRoot();
    World.Reset();
}
UPrimitiveComponent* FIsolatedSimulationContext::ResolveStaticSource(uint64 SourceId) const
{
    const auto* Found = Sources.Find(SourceId);
    return Found ? Found->Get() : nullptr;
}
uint64 FIsolatedSimulationContext::ResolveStaticId(const UPrimitiveComponent* Component) const
{
    const uint64* Found = SourceIds.Find(Component);
    return Found ? *Found : 0;
}
