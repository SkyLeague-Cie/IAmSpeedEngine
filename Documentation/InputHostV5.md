# Inputs Host V5 source checkpoint

Status: SOURCE_ONLY_PARTIAL. No Unreal build, native gameplay execution, hardware qualification, CI migration or release integration is claimed.

IAmSpeed now has an explicit GameInput host factory with a platform-independent acquisition worker, a single-use session, controller ownership, paused control servicing, fresh-resume admission, weak UObject presentation bindings and the wheeled reservation/prepare/commit/abort hooks. The game must supply mapping, activity thresholds, cadence and the actual first physical frame. Unsupported targets return no device host. The factory is not yet installed by the production game controls loader; legacy reflected assets/bindings remain for migration. Replacement requires explicit unpossess/new epoch/new session/possess; automatic world-reset orchestration remains open.

Core adds the named Sky catalogue and an ordered physical command plan. Jump/Powerslide decisions call the extracted existing mechanical bodies in edge order; forces and slew still run once per physical frame. Each first/second jump, flip or auto-reset arm carries its input edge to the actual impulse call. A bounded ledger stages individual outcomes until canonical commit. Powerslide entry/release remains ordered even if the final held bit is unchanged. No force, tire, eligibility threshold, minimum hold or timing coefficient is tuned.

SwitchCam submits exact-frame identities to the physical camera journal. V2 camera receipts are staged with the input transaction, exposed on canonical commit, and discarded on abort. Abort poisons the advanced camera timeline instead of claiming physical rollback. Two toggles in one physical frame remain two outcomes. The existing HUD latest-mode adapter still needs journal draining; no HUD/pixel qualification is claimed. The endpoint binding handover and lifecycle must be tested in Unreal.

Pause neutralizes input at an acknowledged worker boundary; fresh held resume produces no synthetic Started. Re-pause while waiting for or after a fresh acknowledgment re-fences the raw journal. A cancelled, not-yet-applied flip clears its delay and armed state; an already applied flip is ongoing mechanics. Cancellation receipt continuity across detach/reset is still an open gate.

## Evidence

`offline-v2` compiles and passes all nine standalone C++ probes with MSVC14.38, C++17, /W4 /WX. The new host/plan/receipt probe has 4445 checks; most ledger capacity iterations are not distinct gameplay scenarios. None of these probes compiles the Unreal controller, factory or mechanics. Source hashes and canonical reports are retained. Native `SkyLeague.Input.ProducedBooleanV2Worker` is authored but NOT COMPILED/EXECUTED: sealed TestInputProducer -> real canonical worker -> impulse receipts and COM velocity, ordered powerslide state, neutral second jump versus directional flip. Its empty-world fixture does not replace supported-contact Regression/Gold. CameraModeHUD tests now include staged receipts, commit and abort; also NOT COMPILED/EXECUTED.

## Remaining gates before atomic source/build acceptance

- Finish game production configuration from saved/default mappings, epoch/first-frame ownership and automatic reset/replacement lifecycle. Remove physical UE BindActions/InputMappingContext only with that complete replacement.
- Bind presentation effects through the controller V2 observer, drain every camera application receipt into HUD, and acknowledge the other Boolean effects (BackCam/AirRoll and control-world completion). Blueprint ResetWorld/AutoControl currently report Dispatched, not completed world actions.
- Fix any reviewer/native compile findings; execute actual Jump/Flip eligibility, supported impulses, short/multiple/same-frame presses, Powerslide intermediate release, every SwitchCam toggle, variable observer cadence, pause/reset, rejection/overflow and publication-failure tests.
- Migrate all CI cases with inputs to the sealed producer path, run Regression/Gold without opportunistic reference refresh, qualify player mappings/hardware, prepare clean PRs. Accepted CI migration remains zero cases.

No heavy resource, Docker volume, push, PR or release branch mutation is included in this checkpoint.
