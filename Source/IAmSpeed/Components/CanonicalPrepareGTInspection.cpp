#include "CoreMinimal.h"
#include "SpeedMovementComponent.h"
#include "SpeedWheeledComponent.h"
#include "ChaosVehicleManager.h"
#include "ChaosVehicleWheel.h"
#include "HAL/IConsoleManager.h"
#include "Misc/ScopeTryLock.h"
#include "IAmSpeed/World/Collision/NativePrepareGTInspection.h"
#include "IAmSpeed/World/Collision/NativeInitialHitCache.h"
#include "IAmSpeed/World/Collision/NativeSuspensionPrepareValues.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
bool USpeedMovementComponent::ReadCanonicalPrepareStateForGTInspection(const Speed::FPreparedNativeOwnerBinding& binding,
    Speed::FNativePrepareGTFields& out,Speed::Analytic::Certified::FDistanceBudget& budget,FString& reason) const
{
    out={};reason.Reset();if(!Speed::MatchesNativePrepareGTBinding(*this,*this,binding))
    {reason=TEXT("Movement prepare diagnostic actual GT binding unavailable");return false;}
    bool ok=true;const auto put=[&](const TCHAR* key,uint64 v){ok=out.Add(key,v,budget)&&ok;};
    put(TEXT("base.frame"),BaseGameState.NumFrame);put(TEXT("base.frozen"),BasePhysicsState.bIsFrozen);
    put(TEXT("base.countdown_started"),BasePhysicsState.bStartCountdown);put(TEXT("base.countdown_frames"),BasePhysicsState.nbFramesbeforeCanMove);
    put(TEXT("base.minimum_frames"),MinNbFramesBeforeCanMove);put(TEXT("base.since_can_move"),uint64(int64(SinceCanMoveFrame)));
    put(TEXT("base.has_test_velocity"),BaseGameState.bHasTestVelocity);put(TEXT("base.simulation_enabled"),bEnableSimulation);
    put(TEXT("base.constraints_count"),GetPhysicalConstraints().Num());
    put(TEXT("base.static_world_private_pointer"),reinterpret_cast<UPTRINT>(GetStaticCollisionWorldForFrame()));
    ok=out.Float(TEXT("base.static_horizon"),GetStaticSupportFrameHorizon(),budget)&&ok;
    put(TEXT("base.authority"),HasAuthority());
#if !UE_BUILD_SHIPPING
    put(TEXT("base.phase_observer_present"),ISpeedComponent::IsPhysicsPhaseObservationEnabled());
#endif
    ok=out.Scalar(TEXT("base.configuration.bEnableGravity"),bEnableGravity,budget)&&ok;
    ok=out.Scalar(TEXT("base.configuration.GravityZ"),GravityZ,budget)&&ok;
    ok=out.Scalar(TEXT("base.configuration.PhysMass"),PhysMass,budget)&&ok;
    ok=out.Scalar(TEXT("base.configuration.PhysMaxSpeed"),PhysMaxSpeed,budget)&&ok;
    ok=out.Scalar(TEXT("base.configuration.PhysMaxAngularSpeed"),PhysMaxAngularSpeed,budget)&&ok;
    ok=out.Scalar(TEXT("base.configuration.PhysDamping"),PhysDamping,budget)&&ok;
    ok=out.Scalar(TEXT("base.configuration.EngineFPS"),EngineFPS,budget)&&ok;
#if !UE_BUILD_SHIPPING
    put(TEXT("base.episode_before_forces"),bEpisodeTestingBeforeForces);
#endif

    ok=out.Vector(TEXT("base.test_velocity"),BaseGameState.TestVelocity,budget)&&ok;
    ok=out.Vector(TEXT("base.test_angular_velocity"),BaseGameState.TestAngularVelocity,budget)&&ok;
    ok=out.Kinematic(TEXT("base.state"),BasePhysicsState.Kinematic,budget)&&ok;
    if(!ok){out={};reason=TEXT("Movement prepare diagnostic budget exhausted");}return ok;
}
bool USpeedWheeledComponent::ReadCanonicalPrepareStateForGTInspection(const Speed::FPreparedNativeOwnerBinding& binding,
    Speed::FNativePrepareGTFields& out,Speed::Analytic::Certified::FDistanceBudget& budget,FString& reason) const
{
    out={};reason.Reset();if(!Speed::MatchesNativePrepareGTBinding(*this,*this,binding))
    {reason=TEXT("Wheeled prepare diagnostic actual GT binding unavailable");return false;}
    const Chaos::FSimpleWheeledVehicle* vehicle=SkySimulation?SkySimulation->PVehicle.Get():nullptr;
    const int32 count=WheelSubBodies.Num();
    if(!vehicle||VehicleSimulationPT.Get()!=SkySimulation||SkySimulation->SpeedWheeledComponent!=this||
        count<=0||count>int32(UE_ARRAY_COUNT(WheeledPhysicsState.SuspensionLastDisplacement))||
        count!=WheelSetups.Num()||count!=Wheels.Num()||count!=vehicle->Wheels.Num()||count!=vehicle->Suspension.Num()||binding.Bodies.Num()!=count+1)
    {reason=TEXT("Wheeled actual vehicle/complete body census unavailable");return false;}
    // Join every wheel to its retained pinned body BEFORE any wheel getter.
    for(int32 i=0;i<count;++i)
    {
        const auto* wheel=WheelSubBodies[i].Get();const Speed::FPreparedNativeBodyBinding* matched=nullptr;
        for(const auto& body:binding.Bodies)if(body.Body==wheel){if(matched){reason=TEXT("Duplicate wheel census identity");return false;}matched=&body;}
        if(!wheel||!matched||!matched->Class||
            (IsInGameThread()&&(wheel->GetClass()!=matched->Class||!matched->Object.HasSameIndexAndSerialNumber(FWeakObjectPtr(wheel)))))
        {reason=TEXT("Wheel is not an exact retained pinned census member");return false;}
    }
    // Same actual associations as ValidateSimulationBindings, with static
    // refusal text: worker observation must not format UObject paths.
    for(int32 i=0;i<count;++i)
    {
        const auto* wheel=WheelSubBodies[i].Get();const auto* config=Wheels[i].Get();
        if(wheel->Idx()!=i||wheel->GetParentComponent()!=this||!config||!WheelSetups[i].WheelClass||
            wheel->GetChaosWheelConfiguration()!=config||wheel->GetWheelSim()!=&vehicle->Wheels[i]||
            wheel->GetSuspensionSim()!=&vehicle->Suspension[i])
        {reason=TEXT("Wheeled retained native associations changed");return false;}
    }
    bool ok=true;const auto put=[&](const TCHAR* key,uint64 v){ok=out.Add(key,v,budget)&&ok;};
    put(TEXT("car.frame"),BaseGameState.NumFrame);put(TEXT("car.frozen"),BasePhysicsState.bIsFrozen);
    // These are the EFFECTIVE wheeled countdown fields, not BasePhysicsState.
    put(TEXT("car.countdown_started"),WheeledPhysicsState.bStartCountdown);put(TEXT("car.countdown_frames"),WheeledPhysicsState.nbFramesbeforeCanMove);
    put(TEXT("car.minimum_frames"),MinNbFramesBeforeCanMove);put(TEXT("car.since_can_move"),uint64(int64(SinceCanMoveFrame)));
    put(TEXT("car.has_test_velocity"),BaseGameState.bHasTestVelocity);put(TEXT("car.startup_requested"),bCanonicalSupportStartupInitializationRequested);
    put(TEXT("car.startup_pose_confirmed"),bCanonicalSupportStartupPoseConfirmed);put(TEXT("car.prepared_input_frame_present"),PreparedCanonicalInputFrame.IsSet());
    if(PreparedCanonicalInputFrame.IsSet())put(TEXT("car.prepared_input_frame"),PreparedCanonicalInputFrame.GetValue());
    put(TEXT("car.canonical_input_pending"),bCanonicalInputPending);put(TEXT("car.input_rejected"),bInputFrameRejectedV2);
    put(TEXT("car.constraints_count"),GetPhysicalConstraints().Num());
    put(TEXT("car.static_world_private_pointer"),reinterpret_cast<UPTRINT>(GetStaticCollisionWorldForFrame()));
    ok=out.Float(TEXT("car.static_horizon"),GetStaticSupportFrameHorizon(),budget)&&ok;
    put(TEXT("car.authority"),HasAuthority());
#if !UE_BUILD_SHIPPING
    put(TEXT("car.phase_observer_present"),ISpeedComponent::IsPhysicsPhaseObservationEnabled());
#endif
    ok=out.Scalar(TEXT("car.configuration.bEnableGravity"),bEnableGravity,budget)&&ok;
    ok=out.Scalar(TEXT("car.configuration.GravityZ"),GravityZ,budget)&&ok;
    ok=out.Scalar(TEXT("car.configuration.PhysMass"),PhysMass,budget)&&ok;
    ok=out.Scalar(TEXT("car.configuration.PhysMaxAngularSpeed"),PhysMaxAngularSpeed,budget)&&ok;
    ok=out.Scalar(TEXT("car.configuration.PhysDamping"),PhysDamping,budget)&&ok;
    ok=out.Scalar(TEXT("car.configuration.EngineFPS"),EngineFPS,budget)&&ok;
    ok=out.Scalar(TEXT("car.WheeledGameState.bGroundState"),WheeledGameState.bGroundState,budget)&&ok;
    ok=out.Scalar(TEXT("car.WheeledGameState.bIsUpsideDown"),WheeledGameState.bIsUpsideDown,budget)&&ok;
    ok=out.Scalar(TEXT("car.WheeledGameState.NbWheelsOnGround"),WheeledGameState.NbWheelsOnGround,budget)&&ok;
    ok=out.Scalar(TEXT("car.WheeledGameState.NbFrameSinceInAir"),WheeledGameState.NbFrameSinceInAir,budget)&&ok;
    ok=out.Scalar(TEXT("car.WheeledPhysicsState.FramesSinceLastImpact"),WheeledPhysicsState.FramesSinceLastImpact,budget)&&ok;
    ok=out.Scalar(TEXT("car.WheeledPhysicsState.CarBumpCooldownFrames"),WheeledPhysicsState.CarBumpCooldownFrames,budget)&&ok;
    ok=out.Scalar(TEXT("car.WheeledPhysicsState.NbFramesSinceGroundContact"),WheeledPhysicsState.NbFramesSinceGroundContact,budget)&&ok;
    put(TEXT("car.bump_victim_explicit_null"),WheeledPhysicsState.LastCarBumpVictim.IsExplicitlyNull());
    ok=out.Vector(TEXT("car.allowed_side_velocity"),WheeledPhysicsState.AllowedSideVelocity,budget)&&ok;
    ok=out.Vector(TEXT("car.allowed_angular_velocity"),WheeledPhysicsState.AllowedAngularVelocity,budget)&&ok;
    ok=out.Scalar(TEXT("car.handbrake.WheelFrameHandbrakeRiseRate"),WheelFrameHandbrakeRiseRate,budget)&&ok;
    ok=out.Scalar(TEXT("car.handbrake.WheelFrameHandbrakeFallRate"),WheelFrameHandbrakeFallRate,budget)&&ok;
    ok=out.Scalar(TEXT("car.handbrake.WheelFrameHandbrakeValue"),WheelFrameHandbrakeValue,budget)&&ok;
    ok=out.Scalar(TEXT("car.handbrake.WheelFrameHandbrakeActiveTime"),WheelFrameHandbrakeActiveTime,budget)&&ok;
    ok=out.Scalar(TEXT("car.handbrake.LastWheelFrameHandbrakeUpdateFrame"),LastWheelFrameHandbrakeUpdateFrame,budget)&&ok;

    // Actual per-component values after any real handling initialization.
    // Keep them separate from console overrides; do not infer a preset or
    // replay its application while reading the current native inputs.
    ok=out.Scalar(TEXT("car.handling.WheelFrameHBContactResponseModel"),WheelFrameHBContactResponseModel,budget)&&ok;
    ok=out.Scalar(TEXT("car.handling.WheelFrameHandbrakeDriveForceScale"),WheelFrameHandbrakeDriveForceScale,budget)&&ok;
    ok=out.Scalar(TEXT("car.handling.WheelFrameHandbrakeHalfInputFraction"),WheelFrameHandbrakeHalfInputFraction,budget)&&ok;
    ok=out.Scalar(TEXT("car.handling.WheelFrameHandbrakeQuarterInputFraction"),WheelFrameHandbrakeQuarterInputFraction,budget)&&ok;
    ok=out.Scalar(TEXT("car.handling.WheelFrameNormalInputFullFraction"),WheelFrameNormalInputFullFraction,budget)&&ok;
    ok=out.Scalar(TEXT("car.handling.WheelFrameNormalHalfInputFraction"),WheelFrameNormalHalfInputFraction,budget)&&ok;
    ok=out.Scalar(TEXT("car.handling.WheelFrameLateralForwardLossScale"),WheelFrameLateralForwardLossScale,budget)&&ok;
    ok=out.Scalar(TEXT("car.handling.WheelFrameLongitudinalScrubScale"),WheelFrameLongitudinalScrubScale,budget)&&ok;
    ok=out.Scalar(TEXT("car.handling.WheelFrameSteeringRearLateralFrictionScale"),WheelFrameSteeringRearLateralFrictionScale,budget)&&ok;

    put(TEXT("car.produced_frame_owned"),bProducedInputFrameOwned);
    put(TEXT("car.input_reservation_present"),InputReservationV2.has_value());
    put(TEXT("car.reserved_input_frame_present"),ReservedInputFrameV2.has_value());
    if(ReservedInputFrameV2)put(TEXT("car.reserved_input_frame"),*ReservedInputFrameV2);
    put(TEXT("car.produced_input_authority"),bProducedInputAuthority.load());
    put(TEXT("car.legacy_remote_input_authority"),bLegacyRemoteInputAuthority.load());
    put(TEXT("car.input_retirement_requested"),bInputRetirementRequested.load());
    put(TEXT("car.input_retirement_completed"),bInputRetirementCompleted.load());
    put(TEXT("car.legacy_owner_present"),bool(LegacyWheeledOwner));
    put(TEXT("car.legacy_retired"),bLegacyWheeledRetired);
    {
        UE::TScopeTryLock<FCriticalSection> lock(FrameInputProducerMutex);
        if(!lock.IsLocked()){out={};reason=TEXT("Wheeled frame producer binding busy");return false;}
        put(TEXT("car.frame_stream_present"),bool(FrameInputStream));put(TEXT("car.frame_stream_v2_present"),bool(FrameInputStreamV2));
        put(TEXT("car.frame_stream_reset"),bResetProducedWheeledInputs);
    }
    put(TEXT("car.consumed_stream_present"),bool(ConsumedFrameInputStream));
    put(TEXT("car.consumed_stream_v2_present"),bool(ConsumedFrameInputStreamV2));
    put(TEXT("car.generic_camera_enabled"),bGenericCameraInputEnabled.load());
    put(TEXT("car.generic_camera_pending_word"),PendingCameraInput.load());
    {
        UE::TScopeTryLock<FCriticalSection> lock(PendingCameraInputMutex);
        if(!lock.IsLocked()||PendingCameraInputCommands.Num()>MaxPendingCameraInputs)
        {out={};reason=TEXT("Wheeled camera input queue busy or outside structural bound");return false;}
        put(TEXT("car.generic_camera_queue_count"),PendingCameraInputCommands.Num());
        put(TEXT("car.generic_camera_last_history"),uint64(int64(LastCameraCaptureHistoryFrame)));
        put(TEXT("car.generic_camera_last_activation"),uint64(int64(LastCameraCaptureActivationFrame)));
        for(int32 i=0;i<PendingCameraInputCommands.Num();++i)
        {
            const auto& v=PendingCameraInputCommands[i];const FString k=FString::Printf(TEXT("car.generic_camera_queue[%d]"),i);
            ok=out.Scalar(k+TEXT(".activation"),v.ActivationFrame,budget)&&ok;
            ok=out.Scalar(k+TEXT(".version"),v.Input.Version,budget)&&ok;ok=out.Scalar(k+TEXT(".flags"),v.Input.Flags,budget)&&ok;
            ok=out.Scalar(k+TEXT(".yaw"),v.Input.Yaw,budget)&&ok;ok=out.Scalar(k+TEXT(".pitch"),v.Input.Pitch,budget)&&ok;
        }
        ok=out.Scalar(TEXT("car.generic_camera_last.version"),LastCapturedCameraInput.Version,budget)&&ok;
        ok=out.Scalar(TEXT("car.generic_camera_last.flags"),LastCapturedCameraInput.Flags,budget)&&ok;
        ok=out.Scalar(TEXT("car.generic_camera_last.yaw"),LastCapturedCameraInput.Yaw,budget)&&ok;
        ok=out.Scalar(TEXT("car.generic_camera_last.pitch"),LastCapturedCameraInput.Pitch,budget)&&ok;
    }
    put(TEXT("car.bSimTimelineWasCanMove"),bSimTimelineWasCanMove);
    put(TEXT("car.bSimTimelineWasGrounded"),bSimTimelineWasGrounded);
    put(TEXT("car.bSimTimelineHasGroundState"),bSimTimelineHasGroundState);
    const auto cvar=[&](const TCHAR* name,bool integer)
    {
        const auto* v=IConsoleManager::Get().FindConsoleVariable(name);
        if(!v){ok=false;return;}
        const FString k=FString(TEXT("car.cvar."))+name;
        ok=out.Scalar(k+TEXT(".flags"),v->GetFlags(),budget)&&ok;
        if(integer)ok=out.Scalar(k,v->GetInt(),budget)&&ok;
        else ok=out.Float(k,v->GetFloat(),budget)&&ok;
    };
    cvar(TEXT("p.IAmSpeed.WheelFrame.NormalInputFullFraction"),false);
    cvar(TEXT("p.IAmSpeed.WheelFrame.NormalHalfInputFraction"),false);
    cvar(TEXT("p.IAmSpeed.WheelFrame.SteeringRearLateralFrictionReverseScale"),false);
    cvar(TEXT("p.IAmSpeed.WheelFrame.UnsteeredAligningYawResidualDamping"),true);
    cvar(TEXT("p.IAmSpeed.Suspension.HullContactContinuousAngularDrag"),true);
    cvar(TEXT("p.SkyLeague.Suspension.ForceDebug"),true);
    cvar(TEXT("p.IAmSpeed.VehicleInertia.CovariantFrame"),true);
    cvar(TEXT("p.IAmSpeed.DebugKinematics"),true);
    cvar(TEXT("p.IAmSpeed.DebugSteering"),true);
    cvar(TEXT("p.IAmSpeed.DebugWheelFriction"),true);
    cvar(TEXT("p.IAmSpeed.DebugWheelFrictionDetailed"),true);
    cvar(TEXT("p.IAmSpeed.WheelFrame.LateralFrictionScale"),false);
    cvar(TEXT("p.IAmSpeed.WheelFrame.ZeroSteerLateralFrictionScale"),false);
    cvar(TEXT("p.IAmSpeed.WheelFrame.UnsteeredAligningYawMaxRate"),false);
    cvar(TEXT("p.IAmSpeed.WheelFrame.UnsteeredAligningYawFullSlipSpeed"),false);
    cvar(TEXT("p.IAmSpeed.WheelFrame.UnsteeredAligningYawMinSlipSpeed"),false);
    cvar(TEXT("p.IAmSpeed.WheelFrame.UnsteeredAligningYawTimeConstant"),false);
    cvar(TEXT("p.IAmSpeed.WheelFrame.SteeringRearLateralFrictionScale"),false);
    cvar(TEXT("p.IAmSpeed.WheelFrame.ClampLateralImpulse"),true);
    cvar(TEXT("p.IAmSpeed.WheelFrame.LongitudinalScrubScale"),false);
    cvar(TEXT("p.IAmSpeed.WheelFrame.HandbrakeDriveModel"),true);
    cvar(TEXT("p.IAmSpeed.WheelFrame.HandbrakeDriveForceScale"),false);
    cvar(TEXT("p.IAmSpeed.WheelFrame.HBContactResponseModel"),true);
    cvar(TEXT("p.IAmSpeed.WheelFrame.HandbrakeHalfInputFraction"),false);
    cvar(TEXT("p.IAmSpeed.WheelFrame.HandbrakeQuarterInputFraction"),false);

    cvar(TEXT("p.IAmSpeed.WheelFrame.HandbrakeDriveForceBlendModel"),true);
    cvar(TEXT("p.IAmSpeed.WheelFrame.HandbrakeSupportProjectionGuard"),true);
    cvar(TEXT("p.IAmSpeed.WheelFrame.HandbrakeSteeringBlendModel"),true);
    cvar(TEXT("p.IAmSpeed.WheelFrame.HandbrakeFullInputCOMSpeedModel"),true);
    cvar(TEXT("p.IAmSpeed.WheelFrame.HandbrakeLateralImpulseScale"),false);
    cvar(TEXT("p.IAmSpeed.WheelFrame.HandbrakeFrontLateralFrictionFactor"),false);
    cvar(TEXT("p.IAmSpeed.WheelFrame.HandbrakeRearLateralFrictionFactor"),false);
    cvar(TEXT("p.IAmSpeed.WheelFrame.SteeringCurvatureScale"),false);
    cvar(TEXT("p.IAmSpeed.WheelFrame.LateralForwardLossScale"),false);
    cvar(TEXT("p.IAmSpeed.WheelFrame.TurnSummaryIntervalFrames"),true);
    cvar(TEXT("p.IAmSpeed.WheelFrame.DebugIntervalFrames"),true);
    cvar(TEXT("p.SkyLeague.Steering.TimeConstant"),false);
    cvar(TEXT("p.SkyLeague.Steering.YawScale"),false);
    cvar(TEXT("p.SkyLeague.Steering.ThrottleTimeConstant"),false);
    cvar(TEXT("p.SkyLeague.Steering.MinForwardSpeed"),false);
    cvar(TEXT("p.SkyLeague.Steering.ReleaseInterpSpeed"),false);
    cvar(TEXT("p.SkyLeague.Suspension.NormalBlend"),false);
    cvar(TEXT("p.SkyLeague.Suspension.InvDotBlend"),false);
    cvar(TEXT("p.SkyLeague.Suspension.Debug"),true);

    ok=out.Vector(TEXT("car.test_velocity"),BaseGameState.TestVelocity,budget)&&ok;
    ok=out.Vector(TEXT("car.test_angular_velocity"),BaseGameState.TestAngularVelocity,budget)&&ok;
    ok=out.Kinematic(TEXT("car.state"),BasePhysicsState.Kinematic,budget)&&ok;
    // These real immutable publication records are reset by terminal abort.
    // Observing their absence/payload closes that reached preparation suffix;
    // no publication or network commit is performed by this reader.
    const auto network=[&](const FString& key,const std::shared_ptr<const FCompletedWheeledInput>& value)
    {
        ok=out.Scalar(key+TEXT(".present"),bool(value),budget)&&ok;
        if(!value)return;
        out.RetainedValues.Add(std::static_pointer_cast<const void>(value));
        ok=out.Scalar(key+TEXT(".private_pointer"),reinterpret_cast<UPTRINT>(value.get()),budget)&&ok;
        ok=out.Scalar(key+TEXT(".frame"),value->LocalFrame,budget)&&ok;
        ok=out.Scalar(key+TEXT(".since_can_move"),value->SinceCanMove,budget)&&ok;
        const auto& input=value->Requested;
        ok=out.Scalar(key+TEXT(".requested.can_move"),input.bCanMove,budget)&&ok;
        ok=out.Scalar(key+TEXT(".requested.throttle"),input.Throttle,budget)&&ok;
        ok=out.Scalar(key+TEXT(".requested.brake"),input.Brake,budget)&&ok;
        ok=out.Scalar(key+TEXT(".requested.steer"),input.Steer,budget)&&ok;
        ok=out.Scalar(key+TEXT(".requested.camera.version"),input.Camera.Version,budget)&&ok;
        ok=out.Scalar(key+TEXT(".requested.camera.flags"),input.Camera.Flags,budget)&&ok;
        ok=out.Scalar(key+TEXT(".requested.camera.yaw"),input.Camera.Yaw,budget)&&ok;
        ok=out.Scalar(key+TEXT(".requested.camera.pitch"),input.Camera.Pitch,budget)&&ok;
    };
    network(TEXT("car.network.prepared"),PreparedWheeledNetworkInput);
    network(TEXT("car.network.published"),PublishedWheeledNetworkInput.load(std::memory_order_seq_cst));
    const auto input=GetCanonicalInputSnapshot();put(TEXT("car.input_snapshot_present"),bool(input));
    if(input)
    {
        out.RetainedInput=input;
        put(TEXT("car.input_snapshot_private_pointer"),uint64(reinterpret_cast<UPTRINT>(input.get())));
        put(TEXT("car.input_consumption_frame"),input->Input.GetData().ConsumptionFrame);put(TEXT("car.input_cutoff"),input->CutoffSequence);
        const auto& data=input->Input.GetData();
        put(TEXT("car.input.version"),data.Version);put(TEXT("car.input.source_sequence"),data.SourceSequence);put(TEXT("car.input.producer_kind"),uint64(data.Producer.Kind));put(TEXT("car.input.producer_id"),data.Producer.Id);
        put(TEXT("car.input.stream_epoch"),data.StreamEpoch.Value);put(TEXT("car.input.device_generation"),data.DeviceGeneration.Value);put(TEXT("car.input.active_mask"),data.ActiveMask);put(TEXT("car.input.reset"),data.Reset);put(TEXT("car.input.transition_count"),uint64(data.Transitions.size()));
        if(data.Transitions.size()>Speed::Input::MaxEdges){out={};reason=TEXT("Input transitions exceed native structural bound");return false;}
        for(int32 i=0;i<int32(data.Values.size());++i)ok=out.Add(FString::Printf(TEXT("car.input.values[%d]"),i),uint64(int64(data.Values[i])),budget)&&ok;
        for(int32 i=0;i<int32(data.Transitions.size());++i)
        {
            const auto& e=data.Transitions[i];const FString k=FString::Printf(TEXT("car.input.transition[%d]"),i);
            ok=out.Add(k+TEXT(".action"),e.Action,budget)&&ok;ok=out.Add(k+TEXT(".state"),uint64(e.State),budget)&&ok;ok=out.Add(k+TEXT(".value"),uint64(int64(e.ValueAtTransition)),budget)&&ok;
            ok=out.Add(k+TEXT(".sequence"),e.Order.Sequence,budget)&&ok;ok=out.Add(k+TEXT(".within_sequence"),e.Order.WithinSequence,budget)&&ok;
        }
        if(input->BindingFingerprint.size()>65536){out={};reason=TEXT("Input fingerprint exceeds diagnostic byte cap");return false;}
        ok=out.Bytes(TEXT("car.input_binding_fingerprint"),input->BindingFingerprint.data(),int32(input->BindingFingerprint.size()),budget)&&ok;
        for(int32 i=0;i<int32(input->Requested.size());++i)
        {ok=out.Add(FString::Printf(TEXT("car.input.requested[%d]"),i),uint64(int64(input->Requested[i])),budget)&&ok;ok=out.Add(FString::Printf(TEXT("car.input.applied[%d]"),i),uint64(int64(input->Applied[i])),budget)&&ok;}
    }
    const auto* hitbox=HitboxSubBody.Get();int32 hitboxMatches=0;
    for(const auto& body:binding.Bodies)if(body.Body==hitbox)++hitboxMatches;
    if(!hitbox||hitboxMatches!=1||hitbox->GetParentComponent()!=this)
    {out={};reason=TEXT("Hitbox is not an exact retained census member");return false;}
    // These getters only return stored values. Never call the seemingly
    // observational HasCurrentExactPlanarContact: it can execute a real sweep.
    put(TEXT("car.hitbox.last_resolved_frame"),uint64(int64(hitbox->GetLastResolvedGroundHitFrame())));
    put(TEXT("car.hitbox.has_persistent_ground_contact"),hitbox->HasPersistentGroundContact());
    put(TEXT("car.hitbox.has_physics_tick_ground_contact"),hitbox->HasPhysicsTickGroundContact());
    ok=out.Hit(TEXT("car.hitbox.current_hit"),hitbox->GetHit(),budget)&&ok;
    ok=out.Vector(TEXT("car.hitbox.ground_plane_normal"),hitbox->GetGroundPlaneNormal(),budget)&&ok;
    ok=out.Float(TEXT("car.hitbox.ground_plane_d"),hitbox->GetGroundPlaneD(),budget)&&ok;
    const TArray<FVector>* contactSets[2]={&hitbox->GetGroundContacts(),&hitbox->GetPhysicsTickGroundContacts()};
    for(int32 pass=0;pass<2;++pass)
    {
        const auto& contacts=*contactSets[pass];const FString k=FString::Printf(TEXT("car.hitbox.contacts[%d]"),pass);
        if(contacts.Num()>4096){out={};reason=TEXT("Hitbox contact list exceeds observation budget bound");return false;}
        ok=out.Scalar(k+TEXT(".count"),contacts.Num(),budget)&&ok;
        for(int32 i=0;i<contacts.Num();++i)ok=out.Vector(k+FString::Printf(TEXT("[%d]"),i),contacts[i],budget)&&ok;
    }
    put(TEXT("car.vehicle.suspension_enabled"),vehicle->bSuspensionEnabled);
    put(TEXT("car.vehicle.mechanical_enabled"),vehicle->bMechanicalSimEnabled);
    put(TEXT("car.vehicle.wheel_friction_enabled"),vehicle->bWheelFrictionEnabled);
    put(TEXT("car.vehicle.driven_wheels"),uint64(int64(vehicle->NumDrivenWheels)));
    put(TEXT("car.vehicle.legacy_friction_position"),vehicle->bLegacyWheelFrictionPosition);
    // Bound output size before walking any caller-sized association list.
    if(vehicle->Axles.Num()>4096){out={};reason=TEXT("Axle list exceeds diagnostic word bound");return false;}
    put(TEXT("car.vehicle.axle_count"),vehicle->Axles.Num());
    for(int32 i=0;i<vehicle->Axles.Num();++i)
    {
        const auto& axle=vehicle->Axles[i].Setup;const FString k=FString::Printf(TEXT("car.vehicle.axle[%d]"),i);
        if(axle.WheelIndex.Num()>4096){out={};reason=TEXT("Axle wheel list exceeds diagnostic word bound");return false;}
        ok=out.Add(k+TEXT(".wheel_index_count"),axle.WheelIndex.Num(),budget)&&ok;
        ok=out.Float(k+TEXT(".rollbar_scaling"),axle.RollbarScaling,budget)&&ok;
        for(int32 j=0;j<axle.WheelIndex.Num();++j)
            ok=out.Add(k+FString::Printf(TEXT(".wheel_index[%d]"),j),axle.WheelIndex[j],budget)&&ok;
        // Keep every association, including duplicates and non-pair axles.
        // Never index a wheel from this diagnostic. Consumers need their own bounds.
    }
    ok=out.Scalar(TEXT("car.max_speed"),PhysMaxSpeed,budget)&&ok;
    ok=out.Scalar(TEXT("car.effective_input.can_move"),WheeledPhysicalInput.bCanMove,budget)&&ok;
    ok=out.Scalar(TEXT("car.effective_input.throttle"),WheeledPhysicalInput.Throttle,budget)&&ok;
    ok=out.Scalar(TEXT("car.effective_input.brake"),WheeledPhysicalInput.Brake,budget)&&ok;
    ok=out.Scalar(TEXT("car.effective_input.steer"),WheeledPhysicalInput.Steer,budget)&&ok;
    ok=out.Scalar(TEXT("car.effective_input.camera.version"),WheeledPhysicalInput.Camera.Version,budget)&&ok;
    ok=out.Scalar(TEXT("car.effective_input.camera.flags"),WheeledPhysicalInput.Camera.Flags,budget)&&ok;
    ok=out.Scalar(TEXT("car.effective_input.camera.yaw"),WheeledPhysicalInput.Camera.Yaw,budget)&&ok;
    ok=out.Scalar(TEXT("car.effective_input.camera.pitch"),WheeledPhysicalInput.Camera.Pitch,budget)&&ok;
    ok=out.Scalar(TEXT("car.restored_input.throttle"),WheeledPhysicsState.PhysicalThrottleInput,budget)&&ok;
    ok=out.Scalar(TEXT("car.restored_input.brake"),WheeledPhysicsState.PhysicalBrakeInput,budget)&&ok;
    ok=out.Scalar(TEXT("car.restored_input.steer"),WheeledPhysicsState.PhysicalSteerInput,budget)&&ok;
    put(TEXT("car.wheel_count"),uint64(count));
    for(int32 i=0;i<count;++i)
    {
        const auto* wheel=WheelSubBodies[i].Get();const FString prefix=FString::Printf(TEXT("car.wheel[%d]"),i);
        ok=out.Add(prefix+TEXT(".bindings_verified"),true,budget)&&ok;
        ok=out.Float(prefix+TEXT(".restore.last_displacement"),WheeledPhysicsState.SuspensionLastDisplacement[i],budget)&&ok;
        if(!wheel->ReadCanonicalPrepareWheelForGTInspection(binding,i,out,budget,reason)){out={};return false;}
        const auto& pw=vehicle->Wheels[i];
        const auto scalar=[&](const TCHAR* n,auto v){ok=out.Scalar(prefix+n,v,budget)&&ok;};
        scalar(TEXT(".wheel_sim.BrakeEnabled"),pw.BrakeEnabled);
        scalar(TEXT(".wheel_sim.HandbrakeEnabled"),pw.HandbrakeEnabled);
        scalar(TEXT(".wheel_sim.SteeringEnabled"),pw.SteeringEnabled);
        scalar(TEXT(".wheel_sim.EngineEnabled"),pw.EngineEnabled);
        scalar(TEXT(".wheel_sim.TractionControlEnabled"),pw.TractionControlEnabled);
        scalar(TEXT(".wheel_sim.ABSEnabled"),pw.ABSEnabled);
        scalar(TEXT(".wheel_sim.FrictionMultiplier"),pw.FrictionMultiplier);
        scalar(TEXT(".wheel_sim.LateralSlipGraphMultiplier"),pw.LateralSlipGraphMultiplier);
        scalar(TEXT(".wheel_sim.CorneringStiffness"),pw.CorneringStiffness);
        scalar(TEXT(".wheel_sim.MaxSteeringAngle"),pw.MaxSteeringAngle);
        scalar(TEXT(".wheel_sim.MaxBrakeTorque"),pw.MaxBrakeTorque);
        scalar(TEXT(".wheel_sim.HandbrakeTorque"),pw.HandbrakeTorque);
        scalar(TEXT(".wheel_sim.ExternalTorqueCombineMethod"),pw.ExternalTorqueCombineMethod);
        scalar(TEXT(".wheel_sim.Re"),pw.Re);
        scalar(TEXT(".wheel_sim.Omega"),pw.Omega);
        scalar(TEXT(".wheel_sim.Sx"),pw.Sx);
        scalar(TEXT(".wheel_sim.Inertia"),pw.Inertia);
        scalar(TEXT(".wheel_sim.DriveTorque"),pw.DriveTorque);
        scalar(TEXT(".wheel_sim.BrakeTorque"),pw.BrakeTorque);
        scalar(TEXT(".wheel_sim.ForceIntoSurface"),pw.ForceIntoSurface);
        scalar(TEXT(".wheel_sim.AngularPosition"),pw.AngularPosition);
        scalar(TEXT(".wheel_sim.SteeringAngle"),pw.SteeringAngle);
        scalar(TEXT(".wheel_sim.SurfaceFriction"),pw.SurfaceFriction);
        scalar(TEXT(".wheel_sim.MaxOmega"),pw.MaxOmega);
        scalar(TEXT(".wheel_sim.ExternalDriveTorque"),pw.ExternalDriveTorque);
        scalar(TEXT(".wheel_sim.ExternalBrakeTorque"),pw.ExternalBrakeTorque);
        scalar(TEXT(".wheel_sim.MassPerWheel"),pw.MassPerWheel);
        scalar(TEXT(".wheel_sim.SlipVelocity"),pw.SlipVelocity);
        scalar(TEXT(".wheel_sim.SlipAngle"),pw.SlipAngle);
        scalar(TEXT(".wheel_sim.bInContact"),pw.bInContact);
        scalar(TEXT(".wheel_sim.WheelIndex"),pw.WheelIndex);
        scalar(TEXT(".wheel_sim.bEngineBraking"),pw.bEngineBraking);
        scalar(TEXT(".wheel_sim.AppliedLinearDriveForce"),pw.AppliedLinearDriveForce);
        scalar(TEXT(".wheel_sim.AppliedLinearBrakeForce"),pw.AppliedLinearBrakeForce);
        scalar(TEXT(".wheel_sim.LongitudinalAdhesiveLimit"),pw.LongitudinalAdhesiveLimit);
        scalar(TEXT(".wheel_sim.LateralAdhesiveLimit"),pw.LateralAdhesiveLimit);
        scalar(TEXT(".wheel_sim.SideSlipModifier"),pw.SideSlipModifier);
        scalar(TEXT(".wheel_sim.Spin"),pw.Spin);
        scalar(TEXT(".wheel_sim.AvailableGrip"),pw.AvailableGrip);
        scalar(TEXT(".wheel_sim.bClipping"),pw.bClipping);
        scalar(TEXT(".wheel_sim.bABSActivated"),pw.bABSActivated);
        ok=out.Vector(prefix+TEXT(".wheel_sim.GroundVelocityVector"),pw.GroundVelocityVector,budget)&&ok;
        ok=out.Vector(prefix+TEXT(".wheel_sim.ForceFromFriction"),pw.ForceFromFriction,budget)&&ok;
        ok=out.Vector(prefix+TEXT(".wheel_sim.InputForces"),pw.InputForces,budget)&&ok;
        const Speed::PreparationValues::FSuspensionCopy suspension(vehicle->Suspension[i]);
        ok=suspension.Append(prefix+TEXT(".suspension"),out,budget)&&ok;
        ok=out.Add(prefix+TEXT(".body_census_index"),uint64(binding.Bodies.IndexOfByPredicate([&](const auto& b){return b.Body==wheel;})),budget)&&ok;
        ok=out.Add(prefix+TEXT(".on_ground"),wheel->IsOnGround(),budget)&&ok;
        ok=out.Add(prefix+TEXT(".in_contact"),wheel->GetWheelSim()->InContact(),budget)&&ok;
        const auto& hit=wheel->GetHit();const bool empty=Speed::IsEmptyNativeInitialHit(hit);
        ok=out.Hit(prefix+TEXT(".current_hit"),hit,budget)&&ok;
        ok=out.Add(prefix+TEXT(".current_hit_empty"),empty,budget)&&ok;
        ok=out.Add(prefix+TEXT(".support_identity_available"),empty,budget)&&ok;
        // A nonempty hit is recorded as UNKNOWN support, never resolved here.
        ok=out.Add(prefix+TEXT(".unresolved_nonempty_support"),!empty,budget)&&ok;
    }
    if(!ok){out={};reason=TEXT("Wheeled prepare diagnostic budget exhausted");}return ok;
}

bool USWheelSubBody::ReadCanonicalPrepareWheelForGTInspection(const Speed::FPreparedNativeOwnerBinding& binding,int32 index,
    Speed::FNativePrepareGTFields& out,Speed::Analytic::Certified::FDistanceBudget& budget,FString& reason) const
{
    if(!binding.BodyCensusCaptured||!binding.Adapter||!WheelComponent||
        !binding.NativeObject||!Speed::MatchesNativePrepareGTBinding(*binding.NativeObject,*WheelComponent,binding)||
        static_cast<const ISpeedComponent*>(WheelComponent)!=binding.Adapter||index!=Idx())
    {reason=TEXT("Wheel prepare diagnostic owner/ordinal unavailable");return false;}
    int32 matches=0;
    for(const auto& body:binding.Bodies)if(body.Body==this)
    {
        if(!body.Class||(IsInGameThread()&&(body.Class!=GetClass()||!body.Object.HasSameIndexAndSerialNumber(FWeakObjectPtr(this)))))
        {reason=TEXT("Wheel prepare diagnostic retained native identity differs");return false;}
        ++matches;
    }
    if(matches!=1||!PWheel||!PSuspension||!ChaosWheel||!PWheel->SetupPtr||!PSuspension->SetupPtr)
    {reason=TEXT("Wheel prepare diagnostic configuration unavailable");return false;}
    const FString k=FString::Printf(TEXT("car.wheel[%d].native"),index);bool ok=true;
    const auto scalar=[&](const TCHAR* n,auto v){ok=out.Scalar(k+n,v,budget)&&ok;};
    scalar(TEXT(".WheelMass"),WheelMass);
    scalar(TEXT(".SuspensionSpringRate"),SuspensionSpringRate);
    scalar(TEXT(".SuspensionDampingCompressionRatio"),SuspensionDampingCompressionRatio);
    scalar(TEXT(".SuspensionDampingReboundRatio"),SuspensionDampingReboundRatio);
    scalar(TEXT(".bUseSuspensionForceModelOverride"),bUseSuspensionForceModelOverride);
    scalar(TEXT(".bClampSuspensionForceToPositive"),bClampSuspensionForceToPositive);
    scalar(TEXT(".bClampPositiveUntilFirstCompression"),bClampPositiveUntilFirstCompression);
    scalar(TEXT(".bUseEffectiveSuspensionSweepRadius"),bUseEffectiveSuspensionSweepRadius);
    scalar(TEXT(".bUseIncreasingDisplacementAsCompression"),bUseIncreasingDisplacementAsCompression);
    scalar(TEXT(".bUseCompressionCrossingContactImpulse"),bUseCompressionCrossingContactImpulse);
    scalar(TEXT(".GroundForceApplicationMode"),GroundForceApplicationMode);
    scalar(TEXT(".ContactImpulseApplicationMode"),ContactImpulseApplicationMode);
    scalar(TEXT(".SuspensionStiffnessForceScale"),SuspensionStiffnessForceScale);
    scalar(TEXT(".SuspensionStiffnessReferenceDisplacement"),SuspensionStiffnessReferenceDisplacement);
    scalar(TEXT(".SuspensionPreRestCompressionDampingScale"),SuspensionPreRestCompressionDampingScale);
    scalar(TEXT(".bClampNegativeSuspensionDisplacement"),bClampNegativeSuspensionDisplacement);
    scalar(TEXT(".bDisableSuspensionBumpStopForce"),bDisableSuspensionBumpStopForce);
    scalar(TEXT(".SuspensionSpringRateCmOverride"),SuspensionSpringRateCmOverride);
    scalar(TEXT(".SuspensionCompressionDampingOverride"),SuspensionCompressionDampingOverride);
    scalar(TEXT(".SuspensionReboundDampingOverride"),SuspensionReboundDampingOverride);
    scalar(TEXT(".SuspensionMaxRaiseOverride"),SuspensionMaxRaiseOverride);
    scalar(TEXT(".SuspensionMaxDropOverride"),SuspensionMaxDropOverride);
    scalar(TEXT(".ConfiguredStaticSpringCompression"),ConfiguredStaticSpringCompression);
    scalar(TEXT(".SuspensionForce"),SuspensionForce);
    scalar(TEXT(".bIsJumping"),bIsJumping);
    scalar(TEXT(".bJumpUnilateralSupport"),bJumpUnilateralSupport);
    scalar(TEXT(".ConsecutiveGroundFrames"),ConsecutiveGroundFrames);
    scalar(TEXT(".SteeringAngle"),SteeringAngle);
    scalar(TEXT(".Omega"),Omega);
    scalar(TEXT(".RollAngle"),RollAngle);
    scalar(TEXT(".bWasOnGroundPrevFrame"),bWasOnGroundPrevFrame);
    scalar(TEXT(".bCountedGroundFrame"),bCountedGroundFrame);
    scalar(TEXT(".bContactVelocityLocked"),bContactVelocityLocked);
    scalar(TEXT(".bCrossedIntoCompressionThisFrame"),bCrossedIntoCompressionThisFrame);
    ok=out.Vector(k+TEXT(".ForwardAxis"),ForwardAxis,budget)&&ok;
    ok=out.Vector(k+TEXT(".RightAxis"),RightAxis,budget)&&ok;
    ok=out.Vector(k+TEXT(".UpAxis"),UpAxis,budget)&&ok;
    ok=out.Vector(k+TEXT(".RenderData.WorldPosition"),RenderData.WorldPosition,budget)&&ok;
    ok=out.Vector(k+TEXT(".RenderData.RightAxis"),RenderData.RightAxis,budget)&&ok;
    ok=out.Vector(k+TEXT(".RenderData.UpAxis"),RenderData.UpAxis,budget)&&ok;
    ok=out.Vector(k+TEXT(".RenderData.ForwardAxis"),RenderData.ForwardAxis,budget)&&ok;
    ok=out.Quat(k+TEXT(".render_roll"),RenderData.RollRotation,budget)&&ok;
    ok=out.Quat(k+TEXT(".render_steer"),RenderData.SteerRotation,budget)&&ok;
    scalar(TEXT(".render_spring_offset"),RenderData.SpringOffset);
    // Effective scalar/offset getters here are pure; never sample SpringLength.
    scalar(TEXT(".effective_radius"),Radius());scalar(TEXT(".effective_chaos_radius"),ChaosEffectiveRadius());
    scalar(TEXT(".effective_max_length"),MaxLength());scalar(TEXT(".effective_max_raise"),SuspensionMaxRaise());
    scalar(TEXT(".effective_max_drop"),SuspensionMaxDrop());
    scalar(TEXT(".configured_rest_length"),ChaosWheel->SuspensionMaxRaise);
    scalar(TEXT(".configured_max_drop"),ChaosWheel->SuspensionMaxDrop);
    scalar(TEXT(".configured_wheel_radius"),ChaosWheel->WheelRadius);
    ok=out.Vector(k+TEXT(".relative_location"),GetLocalOffset(),budget)&&ok;
    ok=out.Quat(k+TEXT(".relative_rotation"),GetLocalRotation(),budget)&&ok;
    if(!ok)reason=TEXT("Wheel prepare diagnostic budget exhausted");return ok;
}

#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/World/Collision/NativeSuspensionCopyProbe.h"
void Speed::PreparationValues::CaptureSuspensionCopyProbeForTesting(FSuspensionCopyProbe& out)
{
    out={};
    Chaos::FSimpleSuspensionConfig config;config.SetSuspensionMaxRaise(2.f);config.SetSuspensionMaxDrop(4.f);
    config.SuspensionSmoothing=4;config.RaycastSafetyMargin=0.f;
    Chaos::FSimpleSuspensionSim live(&config);
    live.SetSuspensionLength(6.f,1.f);live.SetLastDisplacement(7.f);
    live.SetLastSpringLength(2.f);live.SetAveragingCount(3);live.SetAveragingNum(4);
    for(int32 i=0;i<NUM_SUS_AVERAGING;++i)live.SetAveragingLength(i,float(i)+0.25f);
    Analytic::Certified::FDistanceBudget a{4096},b{4096},c{4096},d{4096},exhausted{};
    const FSuspensionCopy first(live);
    out.BeforeCaptured=first.Append(TEXT("test"),out.Before,a);
    FSuspensionCopy separate(live);out.CopyGetterResult=separate.GetSpringLength();
    out.ChangedCopyCaptured=separate.Append(TEXT("test"),out.ChangedCopy,b);
    const FSuspensionCopy second(live);
    out.LiveAfterCaptured=second.Append(TEXT("test"),out.LiveAfter,c);
    FNativePrepareGTFields unavailable;
    out.ExhaustedBudgetCaptureReturned=second.Append(TEXT("test"),unavailable,exhausted);
    const FSuspensionCopy last(live);
    out.LiveAfterBudgetCaptured=last.Append(TEXT("test"),out.LiveAfterBudget,d);
}
#endif

// Values-only predictor for the exact base gravity -> game AIR damping ->
// native speed-cap arithmetic. It calls the same SSBox operations as the live
// implementation, with actual stored state/configuration, and writes no owner.
bool USpeedMovementComponent::PredictFirstAirPreparationAcceleration(const Speed::FPreparedNativeOwnerBinding& binding,
    uint64 canonicalFrame,float delta,float airDamping,Speed::FNativePrepareGTFields& expected,
    Speed::Analytic::Certified::FDistanceBudget& budget,FString& reason) const
{
    expected={};reason.Reset();
    if(!Speed::MatchesNativePrepareGTBinding(*this,*this,binding)||canonicalFrame!=0||BaseGameState.NumFrame!=0||
        !bEnableSimulation||BasePhysicsState.bIsFrozen||BasePhysicsState.bStartCountdown||BaseGameState.bHasTestVelocity||
        !GetPhysicalConstraints().IsEmpty()||!HasAuthority()||!FMath::IsFinite(delta)||delta<=0||
        !FMath::IsFinite(airDamping)||!FMath::IsFinite(GravityZ)||!FMath::IsFinite(PhysMaxSpeed)||PhysMaxSpeed<0||
        !FMath::IsFinite(PhysMaxAngularSpeed)||PhysMaxAngularSpeed<0)
    {reason=TEXT("First AIR base predictor actual owner/domain unavailable");return false;}
#if !UE_BUILD_SHIPPING
    if(bEpisodeTestingBeforeForces||ISpeedComponent::IsPhysicsPhaseObservationEnabled())
    {reason=TEXT("First AIR predictor has an unsupported observer/episode scope");return false;}
#endif
    auto state=BasePhysicsState.Kinematic;
    const auto finite=[](const Speed::FKinematicState& k)
    {return !k.Location.ContainsNaN()&&!k.Velocity.ContainsNaN()&&!k.Acceleration.ContainsNaN()&&
        !k.Rotation.ContainsNaN()&&!k.AngularVelocity.ContainsNaN()&&!k.AngularAcceleration.ContainsNaN();};
    if(!finite(state)){reason=TEXT("First AIR predictor native input is not finite");return false;}
    // Empty constraints make the native delta projection exactly the identity.
    if(bEnableGravity)state.Acceleration=state.Acceleration+FVector(0.0f,0.0f,GravityZ);
    if(airDamping>0.0)
    {
        const FVector velocity=state.Velocity;
        FVector newVelocity=(1-airDamping)*velocity;
        const auto speed=velocity.Size();if(speed<=1.0)newVelocity=FVector::ZeroVector;
        const auto forceToApply=(newVelocity-velocity)/delta;
        state.Acceleration=state.Acceleration+forceToApply;
    }
    // Exact native operation order, no reassociation into a closed-form force.
    const FVector velocity=state.Velocity;
    const FVector advanced=SSBox::AdvanceVelocity(velocity,state.Acceleration,delta);
    const FVector clamped=advanced.GetClampedToMaxSize(PhysMaxSpeed);
    state.Acceleration=(clamped-velocity)/delta;
    const FVector angularVelocity=state.AngularVelocity;
    FVector advancedAngular=SSBox::AdvanceAngularVelocity(angularVelocity,state.AngularAcceleration,delta);
    advancedAngular=advancedAngular.GetClampedToMaxSize(PhysMaxAngularSpeed);
    state.AngularAcceleration=(advancedAngular-angularVelocity)/delta;
    if(!finite(state)){reason=TEXT("First AIR predictor native output is not finite");return false;}
    const bool ok=expected.Scalar(TEXT("base.frame"),canonicalFrame+1,budget)&&expected.Kinematic(TEXT("base.state"),state,budget);
    if(!ok){expected={};reason=TEXT("First AIR predictor budget exhausted");}return ok;
}


namespace
{
bool IsFiniteNativeStepSyncHit(const SHitResult& hit)
{
    return !hit.Location.ContainsNaN()&&!hit.ImpactPoint.ContainsNaN()&&!hit.ImpactNormal.ContainsNaN()&&
        !hit.ContactPointThis.ContainsNaN()&&!hit.ContactPointOther.ContainsNaN()&&
        FMath::IsFinite(hit.TOI)&&FMath::IsFinite(hit.PenetrationDepth)&&FMath::IsFinite(hit.GeometricErrorBoundCm);
}
}
bool USSubBody::ReadCanonicalStepSyncFields(const Speed::FPreparedNativeOwnerBinding& binding,
    const FString& prefix,Speed::FNativePrepareGTFields& out,
    Speed::Analytic::Certified::FDistanceBudget& budget,FString& reason) const
{
    if(!binding.NativeObject||!binding.Adapter||!ParentComponent||ParentComponent!=binding.Adapter||
        !Speed::MatchesNativePrepareGTBinding(*binding.NativeObject,*ParentComponent,binding))
    {reason=TEXT("Step sync capture has no current retained owner binding");return false;}
    int32 matches=0;
    for(const auto& entry:binding.Bodies)if(entry.Body==this)
    {
        if(!entry.Class||(IsInGameThread()&&(entry.Class!=GetClass()||!entry.Object.HasSameIndexAndSerialNumber(FWeakObjectPtr(this)))))
        {reason=TEXT("Step sync capture retained body identity differs");return false;}
        ++matches;
    }
    const auto finite=[](const Speed::FKinematicState& s)
    {return !s.Location.ContainsNaN()&&!s.Velocity.ContainsNaN()&&!s.Acceleration.ContainsNaN()&&
        !s.Rotation.ContainsNaN()&&!s.AngularVelocity.ContainsNaN()&&!s.AngularAcceleration.ContainsNaN();};
    const FVector relative=GetRelativeLocation();const FRotator euler=GetRelativeRotation();
    if(matches!=1||prefix.IsEmpty()||!finite(Kinematics)||relative.ContainsNaN()||euler.ContainsNaN()||
        !IsFiniteNativeStepSyncHit(CurrentHit)||!IsFiniteNativeStepSyncHit(FutureHit)||
        AlwaysIgnoredComponents.Num()>4096||IgnoredComponents.Num()>4096)
    {reason=TEXT("Step sync capture state/association bound unavailable");return false;}
    bool ok=true;const auto put=[&](const TCHAR* key,uint64 value){ok=out.Add(prefix+key,value,budget)&&ok;};
    put(TEXT(".body_address"),reinterpret_cast<UPTRINT>(this));put(TEXT(".parent_address"),reinterpret_cast<UPTRINT>(ParentComponent));
    ok=out.Kinematic(prefix+TEXT(".state"),Kinematics,budget)&&ok;
    ok=out.Vector(prefix+TEXT(".relative_location"),relative,budget)&&ok;
    ok=out.Double(prefix+TEXT(".relative_euler.pitch"),euler.Pitch,budget)&&ok;
    ok=out.Double(prefix+TEXT(".relative_euler.yaw"),euler.Yaw,budget)&&ok;
    ok=out.Double(prefix+TEXT(".relative_euler.roll"),euler.Roll,budget)&&ok;
    ok=out.Hit(prefix+TEXT(".current_hit"),CurrentHit,budget)&&ok;
    ok=out.Hit(prefix+TEXT(".future_hit"),FutureHit,budget)&&ok;
    const TArray<UPrimitiveComponent*>* ignored[2]={&AlwaysIgnoredComponents,&IgnoredComponents};
    for(int32 pass=0;pass<2;++pass)
    {
        const auto& list=*ignored[pass];const FString key=prefix+FString::Printf(TEXT(".ignored[%d]"),pass);
        ok=out.Add(key+TEXT(".count"),list.Num(),budget)&&ok;
        // These are stored pointer identities only. Do not resolve or touch a
        // component from this read-only worker observation.
        for(int32 i=0;i<list.Num();++i)ok=out.Add(key+FString::Printf(TEXT("[%d]"),i),reinterpret_cast<UPTRINT>(list[i]),budget)&&ok;
    }
    if(!ok)reason=TEXT("Step sync capture budget exhausted");return ok;
}
bool UBoxSubBody::ReadCanonicalStepSyncFields(const Speed::FPreparedNativeOwnerBinding& binding,
    const FString& prefix,Speed::FNativePrepareGTFields& out,
    Speed::Analytic::Certified::FDistanceBudget& budget,FString& reason) const
{
    if(!Super::ReadCanonicalStepSyncFields(binding,prefix,out,budget,reason))return false;
    if(!IsFiniteNativeStepSyncHit(GroundHit)){reason=TEXT("Step sync box stored hit is not finite");return false;}
    bool ok=true;const auto put=[&](const TCHAR* key,uint64 value){ok=out.Add(prefix+key,value,budget)&&ok;};
    // The two raw flags close every support-query/pose-correction branch of
    // Box Reset before dispatch. Never call HasRoofSurfaceTraversalSupport
    // here: it can inspect another weak object and native owner state.
    put(TEXT(".box.varying_normal"),GroundHit.bSurfaceNormalMayVary);
    put(TEXT(".box.roof_latched"),bRoofSurfaceTraversalLatched);
    put(TEXT(".box.ground_component_explicit_null"),GroundComp.IsExplicitlyNull());
    put(TEXT(".box.roof_component_explicit_null"),RoofSurfaceComp.IsExplicitlyNull());
    put(TEXT(".box.ground_plane_valid"),bGroundPlaneValid);
    put(TEXT(".box.has_ground_contact"),bHasGroundContact);
    put(TEXT(".box.ground_from_sweep"),bGroundHitFromSweep);
    put(TEXT(".box.fresh_edge_candidate"),bFreshEdgeRecoverCandidate);
    put(TEXT(".box.hit_count"),HitCountThisFrame.Num());
    ok=out.Hit(prefix+TEXT(".box.ground_hit"),GroundHit,budget)&&ok;
    const TArray<FVector>* lists[3]={&CurrentGroundContactsWS,&PreviousFrameGroundContactsWS,&CurrentGroundNormalsWS};
    for(int32 pass=0;pass<3;++pass)
    {
        const auto& list=*lists[pass];const FString key=prefix+FString::Printf(TEXT(".box.contacts[%d]"),pass);
        if(list.Num()>4096){reason=TEXT("Step sync box contact observation bound exceeded");return false;}
        ok=out.Add(key+TEXT(".count"),list.Num(),budget)&&ok;
        for(int32 i=0;i<list.Num();++i)
        {
            if(list[i].ContainsNaN()){reason=TEXT("Step sync box stored contact is not finite");return false;}
            ok=out.Vector(key+FString::Printf(TEXT("[%d]"),i),list[i],budget)&&ok;
        }
    }
    // This witness covers reset without an active support branch. Existing
    // contact arrays remain observations, not an assertion that all are empty.
    // Active support is refused, not resolved or silently omitted.
    if(GroundHit.bSurfaceNormalMayVary||bRoofSurfaceTraversalLatched||!GroundComp.IsExplicitlyNull()||
        !RoofSurfaceComp.IsExplicitlyNull()||bGroundPlaneValid||!HitCountThisFrame.IsEmpty())
    {reason=TEXT("Step sync capture refuses a non-cold box support branch before reset");return false;}
    if(!ok)reason=TEXT("Step sync box capture budget exhausted");return ok;
}
