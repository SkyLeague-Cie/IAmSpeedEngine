# Controller input automation witness

`Input/Testing/ControllerInputTestAccess.h` exists only when
`WITH_DEV_AUTOMATION_TESTS` is enabled. It observes a controller's stream through
a weak pointer to const, copies recorded/latest frames, and reads active state
under the stream mutex. It exposes no stream mutation, controller pointer to a
worker, serialized address, or component internals. The controller lookup is GT
only. This header does not register or activate a producer or change any default.

The Sky League controller/presentation fixture uses this witness to distinguish
consumed-but-unpublished inputs from completed physical frames, and to observe
UnPossess deactivation. Production Tick, possession and input handlers are
unchanged. Source checks and portable tests cannot qualify UObject lifecycle or
the fixture's actual Unreal simulation; a separate exact-tuple UE gate must
prove the automation macro and execute those checks.
