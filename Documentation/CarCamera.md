# Car camera

IAmSpeed owns a generic physical-frame camera pipeline for `ASpeedCar`:
values-only aim and arm calculations, a frame-addressed producer, and an Unreal
component that applies a completed published pose. The solver advances from
canonical physical frames; the rendered camera consumes that result on the game
thread.

The generic path is connected to `ASpeedCar`. Its constructor creates
`USpeedArmComponent`, `BeginPlay` initializes it from the car and movement
component, and controller/possession changes refresh the binding. The older
source-only aim extraction has become part of this runtime path.

## Responsibilities

| Layer | Responsibility |
| --- | --- |
| `FSpeedCarCameraAim` | Ground/aerial heading, rear view and direction history from captured values |
| `FSpeedCarCameraArm` / `FSpeedCarCameraArmState` | One arm/filter/settings timeline and fixed-frame step; optional host targeting/constraint policy |
| `FSpeedCarCameraPresentation` | Consume the completed owner's snapshot and commands, own mutable camera state, emit pose/checkpoint bytes |
| `USpeedArmComponent` | Capture initial configuration and manage Unreal lifecycle/binding; import the published pose into `UCameraComponent` |
| Host specialization | Game-specific target selection, collision authority, targeting/constraint history and payload policy |

The physical producer does not read a live Unreal actor, component, world query
or game-thread configuration. It works from copied frame values and its owned
state. Current APIs still use Unreal value types and component integration;
this boundary does not make the plugin an Unreal-free camera SDK.

The generic producer is owner-only and unconstrained. It does not fabricate a
target or provide the host game's collision policy. A host requiring collision
implements `ISpeedCarCameraFramePolicy` with admitted immutable authority and
returns a failure when that authority is unavailable. It must not fall back to
a live game-thread trace inside the physical producer.

## One physical update per frame

Aim advances from car orientation/velocity, ground/rear input, captured settings
and direction history at fixed `1/300 s`. Rear view preserves the direction
history and applies its view rotation. Aim is a kernel within the arm's complete
timeline; a specialization must not keep a second independent aim timeline or
step the same arm recursively.

`FSpeedCarCameraArm::Step` is the common outer operation. Its nominal/constraint
seam also allows an independent nominal trajectory to be evaluated and then
resolved against the **current** final physical constraint. A nominal candidate
alone is not a physical collision certificate and cannot be published as one.

The presentation endpoint validates the owner, consecutive frame, input payload
and numeric values before advancing. Frame-addressed setting commands are
consumed through its command journal. Pose and checkpoint payloads serialize
explicit fields; the native C++ object layout is not a wire format.

## Lifecycle and game-thread import

Only a local human owner with a simulation that allows camera presentation binds
the generic player camera. Binding and retirement use an acknowledged simulation
boundary. A pause request alone does not authorize mutation of a running
endpoint; a timeout does not silently resume a partial transition.

`USpeedArmComponent` captures supported initial settings, disables its ordinary
component tick for the generic path and applies matching immutable publications.
`ASpeedCar` reads the published owner pose and applies the camera output rather
than stepping a second camera solver on the game thread. Unsupported settings,
invalid addressing and stale/mismatched packets are rejected.

In Normal mode, a certified independent camera endpoint may execute on an
IAmSpeed private worker while the canonical owner validates independent input
state. Both join before publication. Unsupported or multi-producer lists retain
serial execution; a lane request does not prove parallel camera execution.

FastSimulation does not register or execute player camera producers:
`AFastSimulation::AllowsCameraPresentation()` is false and the simulation guards
registration and dispatch. Physics comparisons between Normal and Fast compare
camera output according to their different mode contracts, rather than
requiring a Fast camera to be present.

## Publication and restore

Pose outputs are addressed by owner/channel/frame and share a publication serial
with the body snapshot. The game-thread adapter composes the relative camera
pose with the matching owner publication. It must not combine a new camera
packet with a stale car pose or substitute a live actor transform.

The camera checkpoint includes settings, arm state, timeline and command-journal
information with payload integrity checks. The endpoint implements `CanRestore`
and `RestoreValidated`; the common simulation validates every producer before
physical restore and only then restores admitted presentation state. See
[PresentationBoundary.md](PresentationBoundary.md) for transaction and retirement
rules.

These local checkpoint/replay mechanisms do not establish an authenticated
remote-state protocol or complete network rollback. End-to-end netcode is not
currently operational; restoration is targeted for 0.5.

## Sources and tests

- [Aim](../Source/IAmSpeed/Camera/SpeedCarCameraAim.h),
  [arm and host policy](../Source/IAmSpeed/Camera/SpeedCarCameraArm.h),
  [arm state](../Source/IAmSpeed/Camera/SpeedCarCameraArmState.h)
- [Producer, pose and checkpoint](../Source/IAmSpeed/Camera/SpeedCarCameraPresentation.h)
- [Unreal adapter](../Source/IAmSpeed/Camera/SpeedArmComponent.cpp) and
  [ASpeedCar integration](../Source/IAmSpeed/Actors/SpeedCar.cpp)
- [Presentation dispatch and join](../Source/IAmSpeed/World/Simulation/FramePresentationTasks.h)
- [Nominal-trajectory contract tests](../Source/IAmSpeed/Camera/SpeedCarCameraNominalTests.cpp),
  [input/lifecycle tests](../Source/IAmSpeed/Camera/SpeedCarCameraInputLifecycleTests.cpp),
  [input-boundary tests](../Source/IAmSpeed/Camera/SpeedCarCameraInputBoundaryTests.cpp)
- [Native camera comparison](../Source/IAmSpeed/World/Simulation/NativeCarsCameraTests1217.cpp)
  and [Normal/Fast Analytic cases](../Source/IAmSpeed/World/Simulation/NativeAnalyticContactTests1221.cpp)

Tests cover separate kernels, lifecycle/publication contracts and selected native
comparisons. A synthetic nominal-trajectory test is not complete camera
acceptance, and test sources do not imply a green CI result for every platform,
network configuration or game specialization.
