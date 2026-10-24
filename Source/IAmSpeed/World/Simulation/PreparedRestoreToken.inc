// Included inside FSimulationWorld. This definition is also compiled verbatim
// by ordinary transaction tests; no parallel hand-written token declaration.
        class FPreparedRestoreToken
        {
            friend class FSimulationWorld;
            std::shared_ptr<const uint8> Identity;
            const FSimulationWorld* OriginWorld = nullptr;
            std::thread::id OriginWorker{};
            explicit FPreparedRestoreToken(const FSimulationWorld& W, std::shared_ptr<const uint8> I) noexcept
                : Identity(std::move(I)), OriginWorld(&W), OriginWorker(std::this_thread::get_id()) {}
            // Inspect only the token, BEFORE any worker-owned World field.
            // The caller still owes world lifetime ownership and must never
            // move/destroy the token concurrently with a const query.
            bool IsExactOrigin(const FSimulationWorld& W) const noexcept
            { return OriginWorker == std::this_thread::get_id() && OriginWorld == &W && bool(Identity); }
        public:
            FPreparedRestoreToken() = default;
            FPreparedRestoreToken(const FPreparedRestoreToken&) = delete;
            FPreparedRestoreToken& operator=(const FPreparedRestoreToken&) = delete;
            FPreparedRestoreToken(FPreparedRestoreToken&&) noexcept = default;
            FPreparedRestoreToken& operator=(FPreparedRestoreToken&&) noexcept = default;
        };
