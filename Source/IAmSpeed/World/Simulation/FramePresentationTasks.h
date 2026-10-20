#pragma once

#include "IAmSpeed/World/Simulation/FrameParallelExecutor.h"
#include "IAmSpeed/World/Simulation/SimulationFrameJournal.h"
#include "IAmSpeed/World/Simulation/FrameParallelSettings.h"
#include "IAmSpeed/World/Simulation/FrameSpeculationSafety1213.h"
#include "HAL/PlatformTLS.h"
#include <exception>
#include <stdexcept>

namespace Speed::FrameParallel
{
inline bool SamePresentationNumericContext1327(const FFrameFloatingPointContext& A,
    const FFrameFloatingPointContext& B) noexcept
{
    return A.bValid && B.bValid && A.VectorControl==B.VectorControl
        && FMemory::Memcmp(&A.Environment,&B.Environment,sizeof(A.Environment))==0;
}
/**
 * Produce the completed frame's presentation and validate owner-only input state.
 *
 * LaneLimit includes the calling canonical owner. A single certified endpoint may
 * use one worker while the owner runs the independent validation. Both finish
 * before output assembly, physical publication, commit, rollback or retirement.
 * No physical frame is shifted and no UObject enters the worker closure.
 *
 * ValidateOwner may only inspect state independent of the endpoint and must
 * preserve its complete floating-point environment, on success and exception.
 * This numeric-purity contract is checked around the actual callback. In the
 * canonical driver this is FInputSessionRegistry::ValidateComplete, whose token
 * and input-owner checks stay on their original owning thread. In particular,
 * ValidateCanonicalFrameCommit is NOT such work: game adapters can prepare a
 * camera receipt lock there, and must run after this function has joined.
 *
 * The serial route preserves the original producer/validation ordering. Other
 * producers and multi-producer lists retain that route until separately audited.
 */
/** Begin runs before capturing the producer FP entry; owner observers may
 * change that entry as on the serial route. End must not throw. It runs on owner
 * after the producer has joined, including failure; its bool describes Produce,
 * not the independent validation. Its wall span includes join/overlap and must
 * never be added to simultaneous owner work as exclusive camera compute. */
class FPresentationOwnerObservation final
{
public:
    FPresentationOwnerObservation(const ISimulationPresentationProducer& Producer,
        TFunctionRef<void(const ISimulationPresentationProducer&)> Begin,
        TFunctionRef<void(bool)> End) : Finish(End) { Begin(Producer); }
    ~FPresentationOwnerObservation() { if (bActive) Finish(false); }
    void MarkReturned() { bActive = false; Finish(true); }
    void Complete(bool bReturned) { bActive = false; Finish(bReturned); }
    FPresentationOwnerObservation(const FPresentationOwnerObservation&) = delete;
    FPresentationOwnerObservation& operator=(const FPresentationOwnerObservation&) = delete;
private:
    TFunctionRef<void(bool)> Finish;
    bool bActive = true;
};

inline bool ProduceFramePresentationWithOwnerValidationAndObservation(
    FSimulationSnapshot& Snapshot,
    TConstArrayView<TSharedRef<ISimulationPresentationProducer, ESPMode::ThreadSafe>> Producers,
    int32 LaneLimit,
    bool bHasIndependentOwnerWork,
    TFunctionRef<bool()> ValidateOwner,
    TFunctionRef<void(const ISimulationPresentationProducer&)> BeginOnOwner,
    TFunctionRef<void(bool)> EndOnOwner)
{
    const auto AppendOutput = [&Snapshot](
        const TSharedRef<ISimulationPresentationProducer, ESPMode::ThreadSafe>& Producer,
        FSimulationPresentationOutput&& Output)
    {
        // Addressing belongs to the common publisher, including on the worker path.
        Output.OwnerStableId = Producer->OwnerStableId();
        Output.Channel = Producer->Channel();
        Output.NumFrame = Snapshot.NumFrame;
        Output.PublicationSerial = 0;
        Snapshot.PresentationOutputs.Add(MoveTemp(Output));
    };

    const bool bMasksSafe1213 = LaneLimit < 2 || Speed::IsInsideFrameParallelTask()
        || Speed::AreCurrentFrameFloatingPointExceptionsMasked1213();
    bool bUseWorker = LaneLimit >= 2 && bMasksSafe1213 && bHasIndependentOwnerWork && !Speed::IsInsideFrameParallelTask()
        && Producers.Num() == 1 && Producers[0]->SupportsIndependentFrameProduction()
        && Speed::EnsurePrivateFrameWorkers(Speed::FFrameParallelPolicy::MaximumLanes - 1) > 0;
    EFrameParallelFallback Fallback = EFrameParallelFallback::SerialPolicy;
    if (LaneLimit >= 2)
        Fallback = IsInsideFrameParallelTask() ? EFrameParallelFallback::NestedTask :
            EFrameParallelFallback::UnsupportedWork;
    if (LaneLimit >= 2 && !bMasksSafe1213)
        Fallback = EFrameParallelFallback::NumericEnvironmentUnavailable;
    if (!bUseWorker)
    {
        RecordFrameParallelStage(EFrameParallelStage::Camera, Producers.Num(),
            Producers.IsEmpty() ? 0 : 1, Producers.IsEmpty() ? 0 : 1, Fallback);
        for (const auto& Producer : Producers)
        {
            FSimulationPresentationOutput Output;
            FPresentationOwnerObservation Observation(Producer.Get(), BeginOnOwner, EndOnOwner);
            Producer->Produce(Snapshot, Output);
            Observation.MarkReturned();
            AppendOutput(Producer, MoveTemp(Output));
        }
        return ValidateOwner();
    }

    // This retained handle protects endpoint lifetime even if GT registration changes.
    const auto Producer = Producers[0];
    FSimulationPresentationOutput Output;
    std::exception_ptr ProducerFailure, ValidationFailure;
    bool bValidated = false;
    bool bProducerReturned = false;
    FFrameFloatingPointContext ProducerNumericExit;
    bool bValidationNumericPure = false;
    FPresentationOwnerObservation Observation(Producer.Get(), BeginOnOwner, EndOnOwner);
    const FFrameFloatingPointContext OwnerNumericContext = CaptureFrameFloatingPointContext();
    if (!OwnerNumericContext.bValid || !Speed::AreCurrentFrameFloatingPointExceptionsMasked1213())
    {
        // An owner observer may have changed the entry environment. Preserve
        // that serial entry and decline migration rather than run unsafe FP.
        RecordFrameParallelStage(EFrameParallelStage::Camera, 1, 1, 1,
            EFrameParallelFallback::NumericEnvironmentUnavailable);
        Producer->Produce(Snapshot, Output);
        Observation.MarkReturned();
        AppendOutput(Producer, MoveTemp(Output));
        return ValidateOwner();
    }
    Speed::FScopedFrameParallelTask OwnerTaskScope(OwnerNumericContext);
    if (!OwnerTaskScope.IsNumericContextApplied())
        throw std::runtime_error("IAmSpeed camera owner numeric context unavailable");
    uint32 WorkerThreadId = 0;
    EFrameExecutorBackend Backend = EFrameExecutorBackend::SerialQueueFallback;
    try
    {
        Backend = Speed::DispatchPrivateFramePair(
            [&Snapshot, Producer, &Output, &WorkerThreadId, &ProducerNumericExit, &bProducerReturned, OwnerNumericContext]()
            {
                Speed::FScopedFrameParallelTask WorkerTaskScope(OwnerNumericContext);
                WorkerThreadId = FPlatformTLS::GetCurrentThreadId();
                std::exception_ptr Failure;
                try
                {
                    if (!WorkerTaskScope.IsNumericContextApplied())
                        throw std::runtime_error("IAmSpeed camera worker numeric context unavailable");
                    Producer->Produce(Snapshot, Output);
                    bProducerReturned = true;
                }
                catch (...) { Failure = std::current_exception(); }
                // Publish the original producer's COMPLETE numeric exit after
                // join, not merely its output bytes. Worker scratch stays private.
                ProducerNumericExit=CaptureFrameFloatingPointContext();
                if (!ProducerNumericExit.bValid)
                    Failure=std::make_exception_ptr(std::runtime_error(
                        "IAmSpeed camera producer numeric exit unavailable"));
                if (!WorkerTaskScope.RestoreNumericContext())
                    Failure = std::make_exception_ptr(std::runtime_error(
                        "IAmSpeed camera worker numeric restoration failed"));
                if (Failure) std::rethrow_exception(Failure);
            },
            [&]
            {
                const auto Entry=CaptureFrameFloatingPointContext();
                try { bValidated = ValidateOwner(); }
                catch (...) { ValidationFailure = std::current_exception(); }
                const auto Exit=CaptureFrameFloatingPointContext();
                bValidationNumericPure=SamePresentationNumericContext1327(Entry,Exit);
            });
    }
    catch (...) { ProducerFailure = std::current_exception(); }

    const bool bOwnerRestored = OwnerTaskScope.RestoreNumericContext();
    const int32 NativeThreads = WorkerThreadId != 0 && WorkerThreadId != FPlatformTLS::GetCurrentThreadId() ? 2 : 1;
    RecordFrameParallelStage(EFrameParallelStage::Camera, 1, NativeThreads, NativeThreads,
        Backend == EFrameExecutorBackend::PrivatePool ? EFrameParallelFallback::None
            : EFrameParallelFallback::UnsupportedWork);
    if (!bOwnerRestored)
    {
        Observation.Complete(bProducerReturned);
        throw std::runtime_error("IAmSpeed camera owner numeric restoration failed");
    }

    // Dispatch/validation infrastructure must not erase camera arithmetic flags.
    // The certified integer-only validation leaves that serial exit unchanged.
    if (!ApplyFrameFloatingPointContext(ProducerNumericExit))
    {
        Observation.Complete(bProducerReturned);
        throw std::runtime_error("IAmSpeed camera producer numeric exit publication failed");
    }
    Observation.Complete(bProducerReturned); // End observes the real serial producer FP exit.
    // Match the original producer-before-validation exception precedence.
    if (ProducerFailure) std::rethrow_exception(ProducerFailure);
    if (!bValidationNumericPure)
        throw std::runtime_error("IAmSpeed camera owner validation changed numeric environment");
    AppendOutput(Producer, MoveTemp(Output));
    if (ValidationFailure) std::rethrow_exception(ValidationFailure);
    return bValidated;
}

inline bool ProduceFramePresentationWithOwnerValidation(
    FSimulationSnapshot& Snapshot,
    TConstArrayView<TSharedRef<ISimulationPresentationProducer, ESPMode::ThreadSafe>> Producers,
    int32 LaneLimit, bool bHasIndependentOwnerWork, TFunctionRef<bool()> ValidateOwner)
{
    return ProduceFramePresentationWithOwnerValidationAndObservation(Snapshot, Producers,
        LaneLimit, bHasIndependentOwnerWork, ValidateOwner,
        [](const ISimulationPresentationProducer&) {}, [](bool) {});
}
}
