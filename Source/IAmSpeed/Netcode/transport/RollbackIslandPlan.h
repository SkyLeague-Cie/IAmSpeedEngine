#pragma once
#include "NetInputTransport.h"
#include <algorithm>

namespace Speed::Netcode
{
// Planning only. A complete causal graph must be supplied by the physics owner,
// including swept candidates/new contacts, constraints and gameplay dependencies.
// An observed old contact graph alone is insufficient to assert CompleteCoverage.
enum class EPlanResult { Ready, Invalid, NeedFullWorldCoverage, MissingHistory, BudgetExceeded };
struct FRetainedActor
{
    U64 Actor = 0, Incarnation = 0, SnapshotFrame = 0;
    U64 InputFirst = 0, InputLast = 0;
    bool Mechanical = false, ContactState = false, GameplayAndRng = false, EventLedger = false;
};
struct FCausalDependency
{
    U64 A = 0, B = 0, FirstFrame = 0, LastFrame = 0;
    // Both endpoints join the island: conservative closure for contacts AND
    // directed gameplay reads; no assumption that the reader is physically inert.
};
struct FIslandPlan
{
    EPlanResult Result = EPlanResult::Invalid;
    U64 RestoreFrame = 0, TargetFrame = 0, RegistryGeneration = 0;
    std::vector<U64> Actors;
};
inline FIslandPlan PlanRollbackIsland(const std::vector<FRetainedActor>& Actors,
    const std::vector<FCausalDependency>& Dependencies, const std::vector<U64>& Seeds,
    U64 RestoreFrame, U64 TargetFrame, U64 RegistryGeneration, bool CompleteCoverage,
    std::size_t ActorBudget = 256, U64 FrameBudget = 256)
{
    FIslandPlan Plan; Plan.RestoreFrame = RestoreFrame; Plan.TargetFrame = TargetFrame;
    Plan.RegistryGeneration = RegistryGeneration;
    if (!RegistryGeneration || Seeds.empty() || Actors.empty() || Actors.size() > 256
        || Dependencies.size() > 8192 || RestoreFrame >= TargetFrame) return Plan;
    if (TargetFrame - RestoreFrame > FrameBudget) { Plan.Result = EPlanResult::BudgetExceeded; return Plan; }
    std::vector<U64> All;
    for (const auto& A : Actors)
    {
        if (!A.Actor || !A.Incarnation || std::find(All.begin(), All.end(), A.Actor) != All.end()) return Plan;
        All.push_back(A.Actor);
    }
    const auto Exists = [&](U64 Id) { return std::find(All.begin(), All.end(), Id) != All.end(); };
    for (const auto& D : Dependencies)
        if (!Exists(D.A) || !Exists(D.B) || D.FirstFrame > D.LastFrame) return Plan;
    for (auto Id : Seeds)
    {
        if (!Exists(Id)) return Plan;
        if (std::find(Plan.Actors.begin(), Plan.Actors.end(), Id) == Plan.Actors.end()) Plan.Actors.push_back(Id);
    }
    if (!CompleteCoverage)
    {
        // No reduced island accepted with unknown influences. The caller must
        // obtain a complete full-world transaction, not run a guessed island.
        Plan.Actors = All; Plan.Result = EPlanResult::NeedFullWorldCoverage; return Plan;
    }
    bool Changed;
    do
    {
        Changed = false;
        for (const auto& D : Dependencies)
        {
            if (D.LastFrame <= RestoreFrame || D.FirstFrame > TargetFrame) continue;
            const bool A = std::find(Plan.Actors.begin(), Plan.Actors.end(), D.A) != Plan.Actors.end();
            const bool B = std::find(Plan.Actors.begin(), Plan.Actors.end(), D.B) != Plan.Actors.end();
            if (A != B) { Plan.Actors.push_back(A ? D.B : D.A); Changed = true; }
        }
    } while (Changed);
    std::sort(Plan.Actors.begin(), Plan.Actors.end());
    if (Plan.Actors.size() > ActorBudget) { Plan.Result = EPlanResult::BudgetExceeded; return Plan; }
    for (const auto& A : Actors)
    {
        if (std::find(Plan.Actors.begin(), Plan.Actors.end(), A.Actor) == Plan.Actors.end()) continue;
        if (A.SnapshotFrame != RestoreFrame || A.InputFirst > RestoreFrame + 1 || A.InputLast < TargetFrame
            || !A.Mechanical || !A.ContactState || !A.GameplayAndRng || !A.EventLedger)
        { Plan.Result = EPlanResult::MissingHistory; return Plan; }
    }
    Plan.Result = EPlanResult::Ready;
    return Plan;
}
}
