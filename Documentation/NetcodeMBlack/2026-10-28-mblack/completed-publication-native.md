# Completed world and consumed-input publication

The default-off `-SpeedPublishCompletedWorldInput` option publishes one immutable object from `ASpeedPhysicsSimulation` after the actual world buffer and produced-input registry complete the same frame. It retains the existing legacy world snapshot, the exact completed `FRegistryFrame`, its exact prepared registry, and the real world-buffer publication serial. Storage is allocated/copied before either commit. Pause, retirement, boundary changes, rollback admission and unsuccessful canonical steps invalidate current access; retained history stays immutable. Base `ASpeedSimulation` and the worker implementation have no Git diff against the base (worktree line-ending conversion is not a source change).

`ReadRegistry()` creates an observation copy. Publication uses `Installed->Registry` retained by `PrepareFrame`; the exact identity guard is preserved. Native coverage distinguishes a later equal-version observation from the retained registry. The test-only empty-contact reset uses current `ContactStorage` and refuses a reserved restore boundary. Physics coefficients, car presets, solver order and native network guards remain unchanged.

## Actual local qualification, 28 October 2026

Evidence root: `E:/SkyLeagueWorktrees/netcode-ias-mblack-20261028`. Candidate revision7 is frozen in `Artifacts/publication-port01/candidate-manifest-v7.json`. `NativeHost017-source04` stages506source/resource files without old DLL/PCH/cache reuse. Build04 completed25actions, exit0 and natural drain460.890s. Input/protected hashes stayed unchanged; only scoped trace/cache/build-history outputs were admitted. Three project DLLs have genuine engine BuildId `38047364-febc-4b9a-9f7b-3af2e496759d`, with no retag.

Query04 produced genuine fresh TargetInfo through direct UBT, exit0/drain1.531s, no unexpected engine writes. Tests04 loaded all three exact hashed DLLs in original Editor PID38704, birth ticks134376900293073545. The observer retained original process/Job handles to actual natural drain20.828s, exit0, with no forced termination. Strict validation found exactly seven unique successes, zero errors and zero warnings. Engine metadata and protected hashes stayed unchanged; no SDK/Build.bat fallback ran. Windows KnownFolder was prevalidated against a complete private profile tree on E. Cache/DDC/environment/UserDir/TEMP/TMP/logs are private on E.

Exact selectors:

- `IAmSpeed.Netcode.CompletedPublication.ValueAndEndpointIdentity`
- `IAmSpeed.Netcode.CompletedPublication.CanonicalLifecycle`
- `IAmSpeed.Netcode.DrivingActions.NativeComponentTargets`
- `IAmSpeed.Simulation.DeviceV2Lifecycle2443`
- `IAmSpeed.Simulation.ProducedDeviceLifecycle`
- `IAmSpeed.Simulation.ProducedInputWorkerOrder`
- `IAmSpeed.Simulation.ProducedWheeledInputBoundary`

The common lifecycle case executes16actual canonical component/world frames on the physical worker, two controller session lifecycles and six pause/resume cycles. It checks world bytes/hashes/frame/serial against exact consumed endpoints and retained registries, current invalidation and history after retirement. Device reports are authored; analytical collision world is empty. Component targets use the unchanged stock profile for ten action/sign cases over five frames each. Hardware, physical force/contact trajectories and multiplayer are not accepted by these tests.

## Preserved failures and remaining gates

Build01 failed obsolete contact-storage members. Tests01 failed its three-action fixture against the stock seven-action car, then required explicit termination of its own CLR auxiliary Job members after Editor exit; the362generated engine-output incident remains FAIL/HOLD. Tests02 reached six successes and failed preparation by comparing an observation copy to the prepared registry. Tests02 and03 each changed five generated Editor settings files; tests03 had seven functional successes but remains overall FAIL. None is relabelled by tests04. Per-attempt raw logs, canonical summaries and manifests remain private;244historical FAIL/UNKNOWN entries are preserved.

Not qualified: actual BeginPlay flag/bootstrap, terminal producer-failure fixtures, fullstatev3/current device input/world branches, native restore/causal rollback, force/contact trajectories, Demo migration, packaging/server build, or two simultaneous real clients on one dedicated server. CI on this commit is not yet observed. SL integration/gitlinks belong to the peer. This is a bounded partial API/test delivery, not match acceptance.
