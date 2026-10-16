# Car bump cooldown and snapshot codec

Compile CarBumpStateTests.cpp in a fresh private directory with MSVC C++17 /W4
/WX and the plugin Source include path. The test executes production
CarBumpCooldown.h and CarBumpSnapshot.h,635 assertions with integer/pointer
identities.60/120/300Hz release the last-target cooldown exactly at250ms frame
boundaries; a different target is eligible. Decode is transactional and rejects
truncation/version/target-timer inconsistency. Successful restore preserves
eligibility and future ticks. No copied cooldown/codec algorithm is used.

This does NOT qualify UE reflection/weak-object lifetime, UPackageMap transport,
registry lookup or complete rollback. Native authored test:
IAmSpeed.Collision.CarBump.StateAndNetEnvelope, UNCOMPILED/UNRUN.
The native test covers last-target semantics, reset, neutral canonical and
network envelopes, missing target id and rejection of active network state
without a real object package map. Positive multi-player transport and canonical
active-pair restore still require full native integration coverage.
