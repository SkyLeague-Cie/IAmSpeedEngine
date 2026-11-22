// TEST SHIM DECLARATION, copied API/storage slices, not full native world.
namespace Speed { class FSimulationWorld {
public:
        FSimulationWorld();
        ~FSimulationWorld();
        FSimulationWorld(const FSimulationWorld&) = delete;
        FSimulationWorld& operator=(const FSimulationWorld&) = delete;
        struct FPreparedRestorePublication
        {
            std::shared_ptr<const PreparedWorld::FRestoreEnvelope> Envelope;
            std::shared_ptr<const PreparedWorld::FParsedWorld> Physical;
        };
#include "IAmSpeed/World/Simulation/PreparedRestoreToken.inc"
        std::optional<FPreparedRestoreToken> PrepareSnapshotRestore(
            const FSimulationSnapshot&, std::shared_ptr<const PreparedWorld::FRestoreEnvelope>,
            std::unique_ptr<ISimulationRestoreBoundaryLease>,
            std::unique_ptr<IPreparedSimulationWorldSupplement>);
        std::shared_ptr<const FPreparedRestorePublication> ReadPreparedRestore(const FPreparedRestoreToken&) const noexcept;
        bool PrepareRestoreFinalization(const FPreparedRestoreToken&);
        bool AbortPreparedRestore(const FPreparedRestoreToken&) noexcept;
        void FinalizePreparedRestore(const FPreparedRestoreToken&) noexcept;
        // Caller must already have atomically published its COMMON input/world
        // view containing this EXACT immutable publication. This API alone does
        // not implement that common publisher or establish a native lease.
        void ReleasePreparedRestoreAfterPublication(const FPreparedRestoreToken&,
            const std::shared_ptr<const FPreparedRestorePublication>& Published) noexcept;
        bool IsRestoreBoundaryReserved() const noexcept { return PreparedRestoreEntryActive || PreparedRestore != nullptr; }

        struct FContactStorage
        {
            TArray<FDynamicContactPair> DynamicContactPairs;
            TArray<FDynamicContactPair> PendingRollingContactPairs;
            TMap<uint64, int32> DynamicContactPairIndices;
            TMap<uint64, int32> PendingRollingContactPairIndices;
            TMap<uint32, TSet<uint64>> DynamicContactPairKeysByBody;
        };
        std::unique_ptr<FContactStorage> ContactStorage;
        struct FPreparedWorldRestore;
        std::unique_ptr<FPreparedWorldRestore> PreparedRestore;
        bool PreparedRestoreEntryActive = false; // worker-only: includes cleanup callbacks
        bool MatchesPreparedRestore(const FPreparedRestoreToken&) const noexcept;
        bool CollectPreparedAccess(std::vector<PreparedAccess::FParticipantAccess>&) const;
        uint64 LastPreparedRestoreGeneration = 0, LastPreparedRestorePublication = 0;
    bool bOrderDirty = false;
    unsigned int CurrentStepFrame = 0;
    TArray<FSimulationBodyRecord> Bodies;
    TMap<uint64, ISpeedComponent*> AdaptersByStableId;
    TMap<uint64, USolidSubBody*> SolidSubBodiesByStableId;
    uint64 FindStableId(const ISpeedComponent& C) const;
    uint64 FindStableSubBodyId(const USolidSubBody* C) const;
}; }
