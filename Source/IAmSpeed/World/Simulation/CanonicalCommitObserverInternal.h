#pragma once
#include "CanonicalCommitObserver.h"
// Driver implementation hooks only. Values/snapshot remain borrowed; this
// header is not an alternative public callback or a proof supplied by a replay.
namespace Speed::CanonicalObservers
{
bool HasRegistration(const ASpeedPhysicsSimulation&);
bool EnterActualWorkerPulse(ASpeedPhysicsSimulation&);
EReadiness DriverDecision(ASpeedPhysicsSimulation&);
EReadiness TerminalDecision(const ASpeedPhysicsSimulation&);
bool AllowsPhysicalStep(ASpeedPhysicsSimulation&,uint64 Frame);
EReadiness PostCommitted(ASpeedPhysicsSimulation&,const FSimulationSnapshot&,uint64 PublishedSerial);
bool HasCommittedAt(const ASpeedPhysicsSimulation&,uint64 Frame);
void DriverFailed(ASpeedPhysicsSimulation&);
bool RetireAllAfterActualJoin(ASpeedPhysicsSimulation&);
}
