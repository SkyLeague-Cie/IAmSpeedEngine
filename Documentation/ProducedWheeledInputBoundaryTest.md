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

Freeze a clean parent SHA and all gitlinks containing this test. Compile that
exact tuple against the approved UE5.8.2 installation, recording executable,
module and source identities; an existing DLL is not evidence for new source.
From the parent checkout, the exact build command is:

```powershell
& 'D:\Programs\UnrealEngine\5.8.2\Engine\Build\BatchFiles\Build.bat' SkyLeagueEditor Win64 Development "-Project=$((Get-Location).Path)\SkyLeague.uproject" -WaitMutex -NoHotReload -NoXGE -MaxParallelActions=4
```

Capture stdout/stderr, exit code and timing in fresh artifacts and run
`Scripts/SummarizeSLRun.py package` before inspecting build logs. Only a passing
build of the frozen tuple permits this exact automation command:

```powershell
& .\Scripts\RunSLNativeAutomationWindows.ps1 -Filter 'IAmSpeed.Simulation.ProducedWheeledInputBoundary' -ExpectedTestsManifest 'Plugins\IAmSpeed\Tests\ProducedWheeledInputBoundaryAutomation.json' -ResultsDirectory 'Artifacts\InputActionContract\CI-Migration-W4\ue-v1' -EditorPath 'D:\Programs\UnrealEngine\5.8.2\Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
```

Require exactly the named test, zero failure, matching receipt/source/binary
pins and the runner's canonical summary. Missing binaries, missing test,
unexpected inventory or assertions are failures, not skips or empty passes.
Use a fresh numbered output for retries. Do not run alongside the manager's
Docker workload. Submit source review before requesting the execution lease.

Adding this automation declaration changes the W2/W3 source inventory. Their
frozen results remain historical evidence; regenerate and review the inventory
for a future candidate instead of treating the old pins as current.
