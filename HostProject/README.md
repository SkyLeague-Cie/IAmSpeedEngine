# IAmSpeed HostProject

This minimal project belongs to the IAmSpeed repository. It exists only to
compile and execute public IAmSpeed contracts without a demo or product host.
The contract runner creates a temporary link under `ExternalPlugins`, which is
the only `AdditionalPluginDirectories` root. This prevents sibling plugins in a
host checkout from being discovered or enabled. It uses no product-specific
module, content, coefficient, fixture, or vocabulary and removes the link when
the run ends.

The repository workflow `.github/workflows/physical-contracts.yml` builds the
Editor target and runs `IAmSpeed.AnalyticWorld` through `Scripts/RunContracts.ps1`.
The next harness slice adds a repository-owned
persistent test world per worker and executes sealed FastSimulation cases at a
fixed `PhysicalDeltaTime = 1/300 s`. Workers remain single-lane; no per-case
physics tasks are permitted in this release.

Pure query tests run before Actor/component harness tests. The harness topology
is `Actor -> SpeedComponent -> BoxSubBody/SphereSubBody`, with backend matrix
cells for `UnrealLegacy` and `Analytic`. Backend comparisons classify hit, TOI,
point, normal, feature, transition, semantic end state, determinism, and cost.
Bit identity across backends is required only for exact shared representations.

The host is intentionally not an example game and does not ship.
