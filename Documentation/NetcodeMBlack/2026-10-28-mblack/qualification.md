# MBlack qualification, lot 1 — recovered source and driving INPUT

This is a bounded portable qualification, not native car physics or multiplayer acceptance.

## Exact inputs

- IAS source capsule: `22bbd541b2c8e623c3dc7379fd6282be33ebac62`, derived privately on `codex/netcode-ias-mblack-20261028`.
- IAS 0.17 base: `3e6fb10d2b5e40710ca644b296c5188960a6f855`; nine rebased commits, eight unchanged patches, one append-context adaptation. No old worker restored.
- Actual demo: `SkyLeague-Cie/IAmSpeedDemo` at `fd6f31c34334bf69aa0e2a60854bde21eecf7950`, private `codex/netcode-demo-mblack-20261028`.
- Demo embeds a normal-directory IAS0.4.1 source snapshot under `Plugins/IAmSpeedEngine`; it is not a gitlink. Its `DrivingInput.h` is byte-identical to the IAS capsule: SHA256 `12edfc434d012a6528288aae27962e89f1790c8761bbe2030c0924fd17a160e9`.
- Parent capsule, independently read by SL peer: `ee9f4c16b1d4f968e31d0620ae3677b61c49d439`; Core `dfcb792e37cba1a639a7c26599c3fcffc5226bf6`, Module `df324ebb98e0ad11b1af1cf9f26720575b5f25d3`, SDK `45edc0068500f8f865786a8823df8695babaeb65`. No parent or child gitlink changed in this lot.

The 31 sealed IAS capsule files were individually hashed locally; all match. Manifest SHA256 is `a082e51354151004828db512f8e4959e2f97a361fa651470c0f7cc6e95758f58`. Its 244 historical FAIL/UNKNOWN entries and all pending-publication files remain intact. These are historical evidence, never a new native PASS.

## Active action inventory

`AIAmSpeedDemoCanonicalController : ASpeedDrivingController` uses `Speed::Driving::MakeContract`. The configured GameMode is `IAmSpeedDemoCanonicalGameMode`. Its native pawn is the SpeedCar path; InitGame may select the authored car Blueprint. Saved preferences may remap controls, so the matrix below qualifies stock bindings only.

| ID | Action | Stock keyboard | Stock gamepad | Current portable lot | Native behavior still required |
|---|---|---|---|---|---|
| 0 | Throttle | Up | RT | 255, release0; physical target tuple | Ground acceleration, airborne behavior, all roles/surfaces |
| 1 | Brake | Down | LT | 255, release0; physical target tuple | Braking/reverse and directional/surface cells |
| 2 | Steering | Left/Right | LeftX -/+ | -127/+127, release0; physical target tuple | Actual wheel/trajectory response, signs and speed cells |
| 3 | Pause | P, Escape | Menu | Bool edges and presentation phases; no physical target | Standalone pause; explicit per-client network policy |
| 4 | ResetWorld | R | View | Bool edges and presentation phases; no physical target | Actual teardown/new session; current demo rejects network reset |
| 5 | CameraYaw | J/L | RightX -/+ | -127/+127, release0; no physical target | Published camera behavior and local ownership |
| 6 | CameraPitch | I/K | RightY +/- | -127/+127, release0; no physical target | Published camera behavior and local ownership |

The template `AIAmSpeedDemoPawn` EnhancedInput Handbrake/ToggleCamera path is not the configured canonical controller/pawn path. It must not be claimed as active action coverage. Menu navigation and settings use a separate journal and are outside this lot. Jump/boost/powerslide are not stock Demo actions; SL extensions belong to the SL peer/Core contract.

`DrivingActionsTests.cpp` exercises the actual production contract, mapper, target assembler, sealed input stream and presentation dispatcher: 21 literal binding/sign cells, five frames each (neutral baseline, press, hold, release, held resync). Started/Completed retain edge identity; Triggered is latest held presentation without an edge identity; held resync creates no Started. Repeated observation creates no extra callback. Wrong frame/epoch fails target assembly. No physical step is executed: `ConfirmPhysicalCommit` is an explicit INPUT fixture acknowledgement, not evidence of a world commit.

The wire mode sends these actual mapped frames through production FromV2, Encode/Decode and FExactInbox, then authoritative INPUT to owner and remote inboxes. It checks values, active mask, edges/source ordering, reset, frame, generation, wrong peer, incarnation, duplicate and absent-next-frame behavior. The schema ID, peers and topology are authored fixtures; there is no negotiation, socket, process, physical world, restored state or causal rollback. This is portable input integration only.

## Observed execution

Local toolchain: MSVC `14.38.33130` under VS18 Community, Windows Kits `10.0.22621.0`, C++17 `/W4 /WX /Od /MT`. No Engine/UBT/UHT/Editor or socket peers launched. First run `driving01` compiled/linked, then failed because the fixture omitted required raw control capabilities. It is retained unchanged. `driving02` passed 5250 atomic-input checks and 3381 wire checks, but the legacy English include parser observed zero localized includes; that limitation is retained.

The runner now requests MSVC `/sourceDependencies` JSON, refuses absent/empty successful traces, hashes the discovered headers, and includes driving-wire in not-run accounting. `ordinary03` completed all 37 operations with expected exits, including intentional exit86 probes, all nine suites and the added wire mode. It observed 76 repository includes; all pinned inputs were unchanged and no sibling artifact include was observed. World/common providers remain explicit shims/fixtures. Counts printed by nested existing suites are cumulative, not independent totals.

Reproduce with explicit local paths:

```text
python -B Tests/Netcode/Standalone/run_standalone.py --suite all --out <fresh-private-E-output> --vc "C:/Program Files/Microsoft Visual Studio/18/Community/VC/Tools/MSVC/14.38.33130" --windows-kits "C:/Program Files (x86)/Windows Kits/10" --sdk-version 10.0.22621.0
```

Private MBlack root: `E:/SkyLeagueWorktrees/netcode-ias-mblack-20261028`. Read `checkpoint.json`, `documentation-gate.json`, `Artifacts/ordinary03/receipt.json`, `inputs.json`, `operations.json`, and per-operation `*.observability.json` before raw files. Raw outputs from failed and passing attempts are retained. Cache/DDC/UserDir/TEMP/TMP/logs have distinct private directories. The manager checkout is untouched.

## Native gates and next executable work

1. **Demo host adaptation NOT_IMPLEMENTED:** Server/Client TargetRules already exist. `ServerDefaultMap` is Entry, while GameDefaultMap is FreeplayCircuit. Canonical GameMode BeginPlay requires `NM_Standalone` and fatals otherwise; FinishRestartPlayer also fatals outside standalone. Removing these guards alone would not implement multiplayer. Prepare an explicit network host policy and migration of its embedded snapshot to IAS0.17 without changing accepted presets or copying the old ASpeedSimulation worker.
2. **Atomic publication DEFERRED:** archived seven-file candidate is not applied. Port to ASpeedPhysicsSimulation, preserve ASpeedSimulation value/presentation contract; then real native canonical-frame lifecycle tests. Current-input + consumed-input + world + true branch/fullstatev3 remain unqualified.
3. **Native car actions NOT_RUN:** real component/canonical owner consumption, real force/trajectory assertions and relevant ground/ramp/air contacts, local/server/remote roles. Existing ProducedInputWorkerOrder, ProducedWheeledInputBoundary, ProducedDeviceLifecycle and Driving tests must be rerun on exact native inputs.
4. **Native resource/toolchain UNKNOWN:** normal Win32_OperatingSystem and Win32_Processor reads returned Access denied; no retry, substitution or escalation was used to obtain those measurements. UE3 Replay release19Oct owner receipt is read, but current writer/protected paths/BuildId and native dependencies are not admitted. No physical engine claimed. UE1 remains Experimental.
5. **Network acceptance NOT_RUN:** actual server build/link/cook and startup, two simultaneous real client processes, server possession/incarnation, client→server→remote actions, authoritative consumed states to owner and remotes at the same historical frame; native restore/replay and causal dependency closure. No match is accepted.
6. **CI UNKNOWN:** no current branch native CI PASS. Portable runs do not qualify UE ABI, UHT, packaging, runtime assets or full native CI. No transferred Central binary/cache/grant/UBA assumption.

IAS owns action/protocol/publication hunks; SL peer owns integration/gitlinks and keeps native client guards closed until a qualified contract exists. Child publication must precede parent gitlinks; no force-push. Heartbeat `netcode-iamspeed-reprise-15min` is ACTIVE on the IAS chat only; Netcode Central remains retired.
