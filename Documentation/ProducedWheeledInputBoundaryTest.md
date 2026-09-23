# W4: actual wheeled input boundary automation, pending Unreal qualification

This source-only test is not yet compiled or executed in Unreal. It adds one
test-only friend and one automation test. No player route, V2 activation or
production physical behavior changes. The opt-in manifest in `Tests` is not
added to a scheduled CI suite before qualification.

The test owns a real `USpeedWheeledComponent` through `TStrongObjectPtr`, attaches
the actual W1 sealed producer and `FInputStream`, and invokes the component's
`UpdateInputs`. Production code performs the canonical-frame conversion,
`ReadDrivingInputTargets` and wheel slew. A forwarding counter only counts
producer calls. There is no substitute component or simulated consumer.

## Expected assertions

- Attach at canonical C0/local frame 1; neutral targets and physical axes,
  reset consumed, no legacy queue or dirty live mailbox. Test override is false.
- Scenario keys 0/2/4/6 take effect at C1/3/5/7. Literal expected targets cover
  throttle/brake quantization to 128 and signed steering -63/+64.
- Literal physical values cover steps of 16, reaching a target inside a step,
  direction reversal, holds and a fully neutral C10. No expected values call
  the implementation's quantizer or slew helper.
- Every frame calls the actual consumer twice: same-frame history prevents
  a second producer call, and the slew frame guard prevents double application.
- After C0, inject one due bypass queue command and dirty live mailbox with
  distinct poison values. Targets/physical axes must match the producer, while
  the old queue's frame/payload/bypass flag and live values/mask remain unchanged.
  This tests observable non-consumption; it is not a hardware memory-read trace.
- Consumption records but does not publish. The real completion callback is
  invoked twice per frame, publishing exactly once. Recorded frame clocks,
  identity, actions, reset flag and edge count are checked.
- Presentation scopes read at strides 1/3/7 without changing producer count or
  physical state. The first recorded frame survives later frames.
- Detach deactivates the stream: further consume/publish calls fail, including
  a cached frame. Object lifetime remains owned through teardown.

The test calls the completion hook itself; it does not run the simulation
worker, integration transaction, forces/contact solver, controller Tick,
DeviceInputProducer, raw mapping, hotplug or real devices. It is a physical input
consumer boundary test, not an end-to-end gameplay or full CI migration result.
CanMove, bypass profiles, initial nonneutral inputs, Sky actions and reactive
scenarios are outside W1 and remain unqualified.

## Future execution, only after the manager grants the UE lease

Freeze a clean parent SHA and all gitlinks containing this test. The manager's
lease preflight must resolve and pin `$QualifiedUERoot` to a clean UE checkout
at `16d75d84714512edfb744e1fd0a59e9c74d57873`, and attest that its engine binary
was built from that exact source. Merely hashing an existing binary does not
prove its source provenance. Record the attestation with executable, module
and source identities; an existing project DLL is not evidence for new source.

`D:\Programs\UnrealEngine\5.8.2` is dirty and inadmissible; never fall back to it.
The manager's clean worktree candidate
`D:\UEWorktrees\UnrealEngine-5.8.2-16d75d847-slot2-01` may be selected only if
its required build script and editor binary exist and their provenance has
been attested. This document does not assert that they exist.

From the parent checkout, substitute the lease-qualified path below and run
these guards before building. A placeholder, missing binary, dirty checkout
or different commit aborts the gate. The provenance attestation remains an
additional required lease input; these guards alone cannot provide it.

```powershell
$QualifiedUERoot = '<lease-qualified-engine-root>'
$ErrorActionPreference = 'Stop'
if (-not (Test-Path -LiteralPath $QualifiedUERoot -PathType Container)) { throw 'Unresolved qualified UE root' }
$QualifiedUERoot = (Resolve-Path -LiteralPath $QualifiedUERoot).Path
if ($QualifiedUERoot.TrimEnd('\') -ieq 'D:\Programs\UnrealEngine\5.8.2') { throw 'Inadmissible dirty engine tree' }
$EngineHead = & git -C $QualifiedUERoot rev-parse HEAD
if ($LASTEXITCODE -ne 0 -or $EngineHead -cne '16d75d84714512edfb744e1fd0a59e9c74d57873') { throw 'Unqualified engine commit' }
$EngineDirty = @(& git -C $QualifiedUERoot status --porcelain --untracked-files=normal)
if ($LASTEXITCODE -ne 0 -or $EngineDirty.Count) { throw 'Engine source must be clean' }
$QualifiedBuild = Join-Path $QualifiedUERoot 'Engine\Build\BatchFiles\Build.bat'
$QualifiedEditor = Join-Path $QualifiedUERoot 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
foreach ($Required in @($QualifiedBuild, $QualifiedEditor)) {
    if (-not (Test-Path -LiteralPath $Required -PathType Leaf)) { throw "Required qualified file absent: $Required" }
}
Get-FileHash -LiteralPath $QualifiedBuild, $QualifiedEditor
& $QualifiedBuild SkyLeagueEditor Win64 Development "-Project=$((Get-Location).Path)\SkyLeague.uproject" -WaitMutex -NoHotReload -NoXGE -MaxParallelActions=4
if ($LASTEXITCODE -ne 0) { throw 'Qualified project build failed' }
```

Capture stdout/stderr, exit code and timing in fresh artifacts and run
`Scripts/SummarizeSLRun.py package` before inspecting build logs. Only a passing
build of the frozen tuple permits this exact automation command:

```powershell
& .\Scripts\RunSLNativeAutomationWindows.ps1 -Filter 'IAmSpeed.Simulation.ProducedWheeledInputBoundary' -ExpectedTestsManifest 'Plugins\IAmSpeed\Tests\ProducedWheeledInputBoundaryAutomation.json' -ResultsDirectory 'Artifacts\InputActionContract\CI-Migration-W4\ue-v1' -EditorPath $QualifiedEditor
```

Require exactly the named test, zero failure, matching receipt/source/binary
pins and the runner's canonical summary. Missing binaries, missing test,
unexpected inventory or assertions are failures, not skips or empty passes.
Use a fresh numbered output for retries. Do not run alongside the manager's
Docker workload. Submit source review before requesting the execution lease.

Adding this automation declaration changes the W2/W3 source inventory. Their
frozen results remain historical evidence; regenerate and review the inventory
for a future candidate instead of treating the old pins as current.
