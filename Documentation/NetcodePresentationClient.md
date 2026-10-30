# Presentation-only multiplayer client: source candidate

This is source-only integration, not a native map, Blueprint, connection or render qualification.
The portable gate tests cover identity/role/tuple failure handling, not Unreal lifecycle behavior.

Use the immutable process opt-in `-SpeedExactRemotePresentationClient` before loading the client world.
Without it, the existing Legacy route is unchanged. Pending, Admitted and Failed all suppress IAS
physical bootstrap. No lifecycle path selects ReplayIsolation, Dedicated or Legacy as a fallback.
A pending world has fifteen seconds to establish and admit its original connection. The host must
poll `Query` during connection startup and during the admitted lifetime; timeout or disconnect is a
logical failure, never proof that a process/Job has drained.

`Query` returns the original world generation. Before `Admit`, the host authenticates the entire
trusted tuple from its original human connection: session, epoch, logical controller, producer,
actor/incarnation and canonical contract offer bytes including policy and mode. Pass both the host
approved tuple and received tuple. IAS validates equality, canonical decode, actual NM_Client,
original NetDriver/ServerConnection and a pristine audit. A decoded offer or fingerprint alone
never authenticates the sender. `Matches` checks the retained full tuple/generation on subsequent
traffic; replacements require a new world generation, not a rebind of the failed gate.

The guard blocks IAS component histories/physics setup, subsystem registration, physical worker,
game-mode simulation bootstrap and physical controller/device-host paths. Raw Windows acquisition
for the host's explicit human transport endpoint is deliberately separate from `CreateDeviceInputHost`.
Existing network history registration, pending physical operations or active worker are refused;
no retroactive unregister, shutdown or disabling of unrelated components manufactures admission.
The seamless-travel NetworkPhysics OnRegister hook is covered before its early initialization.

Static Unreal rendering/query geometry is permitted. `Audit` rejects registered simulating primitive
bodies, existing network histories and IAS owners. The host must inventory and reject non-IAS
Blueprint/dynamic spawn paths before they simulate, and call Audit after streaming/spawn before
publishing presentation. This source does not claim to neutralize arbitrary Blueprint physics.
Map-specific spawn/controller hooks remain the host's responsibility.

Consume the original immutable completed publisher's PresentationOutputs, including camera output,
with its real run/generation/session/actor identities. Never reconstruct a local 300Hz solver from
latest poses. No new camera reader is introduced here.

Required native validation remains NOT_RUN: preplaced car/ball/driver, late spawn/streaming and
seamless travel, static query allowance, hostile Blueprint dynamic body, pending connection before
NetDriver, authentic admission and transport capture, wrong/replaced connection, disconnect and
standalone fallback, conflicting Replay/Dedicated intent, and actual two-client dedicated session.
