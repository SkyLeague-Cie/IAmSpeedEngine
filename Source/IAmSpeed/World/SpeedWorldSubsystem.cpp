// Fill out your copyright notice in the Description page of Project Settings.

// USpeedWorldSubsystem.cpp

#include "SpeedWorldSubsystem.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "GameFramework/Actor.h"
#include "Engine/World.h"
#include "Algo/Sort.h"

void USpeedWorldSubsystem::RegisterSpeedComponent(ISpeedComponent* Comp)
{
    if (!Comp) return;
    FScopeLock Lock(&PendingCS);
    PendingOps.Add({ true, Comp });
}

void USpeedWorldSubsystem::UnregisterSpeedComponent(ISpeedComponent* Comp)
{
    if (!Comp) return;
    FScopeLock Lock(&PendingCS);
    PendingOps.Add({ false, Comp });
}

void USpeedWorldSubsystem::ApplyPendingOps()
{
    TArray<FPendingOp> Local;
    {
        FScopeLock Lock(&PendingCS);
        Local = MoveTemp(PendingOps);
        PendingOps.Reset();
    }

    if (Local.Num() == 0) return;

    for (const FPendingOp& Op : Local)
    {
        if (!Op.Comp) continue;

        if (Op.bAdd)
        {
            bool bExists = false;
            for (auto& W : Components) { if (W == Op.Comp) { bExists = true; break; } }
			if (!bExists) AddComponent(*Op.Comp);
        }
        else
        {
			RemoveComponent(*Op.Comp);
        }
    }

    bDirtyOrder = true;
}

void USpeedWorldSubsystem::RebuildSortedIfNeeded()
{
    if (!bDirtyOrder)
        return;

	// Clean invalids
    Components.RemoveAll([](const ISpeedComponent* W)
        {
            return !W;
        });

    ComponentsSorted = Components;

    // Deterministic order:
    // - first Owner->GetUniqueID (stable during a run)
    // - then Object UniqueID of the component
    // Commented for the moment since there is a crash happening in the sort
    /*Algo::Sort(ComponentsSorted, [](const ISpeedComponent* A, const ISpeedComponent* B)
        {
            const UObject* OA = reinterpret_cast<const UObject*>(A);
            const UObject* OB = reinterpret_cast<const UObject*>(B);
            if (!OA || !OB) return OA != nullptr; // valid ones first

            AActor* AA = Cast<AActor>(OA->GetOuter());
            AActor* AB = Cast<AActor>(OB->GetOuter());

            const uint32 AOwnerId = AA ? (uint32)AA->GetUniqueID() : 0u;
            const uint32 BOwnerId = AB ? (uint32)AB->GetUniqueID() : 0u;

            if (AOwnerId != BOwnerId)
                return AOwnerId < BOwnerId;

            return OA->GetUniqueID() < OB->GetUniqueID();
        });*/

    bDirtyOrder = false;
}

void USpeedWorldSubsystem::AddComponent(ISpeedComponent& Comp)
{
    // Add New component to all existing ones
    for (ISpeedComponent* Component : Components)
    {
		Component->AddExternalSubBodies(Comp.GetSubBodies());
    }

	// Add existing ones to the new component
    for (ISpeedComponent* Component : Components)
	{
        Comp.AddExternalSubBodies(Component->GetSubBodies());
	}

	// Finally add to list
    Components.Add(&Comp);
}

void USpeedWorldSubsystem::RemoveComponent(ISpeedComponent& Comp)
{
	// Remove component from list
    Components.RemoveAll([&](const ISpeedComponent* W)
    {
        return !W || W == &Comp;
    });

	// Remove sub-bodies of the removed component from all existing ones
	for (ISpeedComponent* Component : Components)
    {
        Component->RemoveExternalSubBodies(Comp.GetSubBodies());
	}
}

void USpeedWorldSubsystem::Step(const float& Dt, const float& SimTime, const unsigned int& Frame)
{
    ApplyPendingOps();
    RebuildSortedIfNeeded();

    if (Dt <= 0.f || ComponentsSorted.Num() == 0)
        return;

    // ------------------------------------------------------------
    // 1) Reset frame
    // ------------------------------------------------------------
    for (ISpeedComponent* Comp : ComponentsSorted)
    {
        if (!Comp) continue;

		Comp->UpdateSubBodiesKinematics();
        Comp->ResetForFrame(Dt);
    }

    // Anti double-resolve (pair) on the frame
    TSet<uint64> ResolvedPairs;
    ResolvedPairs.Reserve(128);

    float TimePassed = 0.f;

    // Very important: if you leave 0, you risk an infinite loop on TOI==0
	// -> No it should not, SubBody ignores component once it hit it during the frame, so it should just continue to the next hit.
    // const float MinStep = 1e-6f;
    const float MinStep = 0.0f;
    const int32 MaxIter = 24;
    float LastSubDelta = Dt;

    int32 Iter = 0;
    while (TimePassed < Dt && Iter++ < MaxIter)
    {
        const float Remaining = Dt - TimePassed;
        if (Remaining <= MinStep)
            break;

        // ------------------------------------------------------------
        // 2) Find global earliest TOI
        // ------------------------------------------------------------
        SComponentTOI Best;
        Best.bHit = false;
        Best.TOI = Remaining;

        for (ISpeedComponent* Comp : ComponentsSorted)
        {
            if (!Comp) continue;

            const SComponentTOI Ctoi = Comp->SweepTOISubBodies(Remaining, LastSubDelta);

            // TOI sanity
            if (!Ctoi.bHit)
                continue;

            const float T = FMath::Clamp(Ctoi.TOI, 0.f, Remaining);

            // Deterministic tie-break if equal
            // (super important: two hits can have very close TOI)
            if (T < Best.TOI - 1e-9f)
            {
                Best = Ctoi;
                Best.TOI = T;
            }
            else if (FMath::IsNearlyEqual(T, Best.TOI, 1e-9f))
            {
                // tie-break : PairKey then resolver unique id
                if (Ctoi.PairKey < Best.PairKey)
                {
                    Best = Ctoi;
                    Best.TOI = T;
                }
                else if (Ctoi.PairKey == Best.PairKey)
                {
                    USSubBody* R1 = Ctoi.Resolver.Get();
                    USSubBody* R2 = Best.Resolver.Get();
                    const uint32 Id1 = R1 ? (uint32)R1->GetUniqueID() : 0u;
                    const uint32 Id2 = R2 ? (uint32)R2->GetUniqueID() : 0u;
                    if (Id1 < Id2)
                    {
                        Best = Ctoi;
                        Best.TOI = T;
                    }
                }
            }
        }

        // ------------------------------------------------------------
        // 3) Integrate everyone up to TOI (or full remaining if no hit or Iter >= MaxIter)
        // ------------------------------------------------------------
        const bool bWillResolve = Best.bHit && Best.Resolver.IsValid() && Iter < MaxIter;

        float SubDelta = bWillResolve ? Best.TOI : Remaining;

        // clamp to avoid zero-step loop
        if (SubDelta < MinStep)
        {
            // If TOI ~ 0, advance a little, or force resolution without integrating (but this can explode)
            SubDelta = MinStep;
        }
        SubDelta = FMath::Clamp(SubDelta, 0.f, Remaining);
		LastSubDelta = SubDelta;

        // Advance all components to that time
        for (ISpeedComponent* Comp : ComponentsSorted)
        {
            if (!Comp) continue;
            Comp->IntegrateKinematics(SubDelta);
        }
        TimePassed += SubDelta;

        // No hit => End
        if (!bWillResolve)
            break;

        // ------------------------------------------------------------
        // 4) Resolve hit (single per pair)
        // ------------------------------------------------------------
        USSubBody* Resolver = Best.Resolver.Get();
        if (!Resolver)
            continue;

        // Anti-double resolve : if this pair has already been resolved in the frame, skip
        // and continue the loop (re-sweep on updated Remaining).
        if (ResolvedPairs.Contains(Best.PairKey))
            continue;

        // Handoff the chosen hit to the resolver (very important)
        // Resolver->SetFutureHit(Best.Hit);
        Resolver->AcceptHit();
        if (Resolver->ComponentHasBeenIgnored(*Resolver->GetHit().Component.Get()))
        {
            ResolvedPairs.Add(Best.PairKey);
        }

        // Resolve at current substep time
        Resolver->ResolveCurrentHit(SubDelta, SimTime);

        // Optional: post update at each substep (useful if certain gameplay sensors need to react "immediately")
        /*
        for (ISpeedComponent* Comp : ComponentsSorted)
        {
            if (!Comp) continue;
            Comp->PostPhysicsUpdate();
        }
        */
    }

    // ------------------------------------------------------------
    // 5) Final post update
    // ------------------------------------------------------------
    for (ISpeedComponent* Comp : ComponentsSorted)
    {
        if (!Comp) continue;
        Comp->PostPhysicsUpdate(Dt);
    }
}