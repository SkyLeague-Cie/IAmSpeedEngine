#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <type_traits>
#include <vector>

namespace Speed::Netcode::DeclaredGame
{
inline constexpr bool CompleteNativeRestoreQualified = false;
struct FVectorValue { double X=0,Y=0,Z=0; };
struct FAddress
{
    std::uint64_t Frame=0,RegistryGeneration=0,Actor=0,ActorIncarnation=0;
    bool Valid() const noexcept { return RegistryGeneration && Actor && ActorIncarnation; }
};
// These are typed SUBPAYLOADS. World/incarnation, authority branch, full actor
// roster, config/lifetime/topology and common/body/contacts belong to the outer
// complete snapshot transaction. A subpayload alone cannot restore an actor.
struct FCarPhysical
{
    std::uint16_t boostQty{};
    bool bGroundState{};
    std::uint8_t JumpBitField{};
    std::uint8_t JumpPerformingBitField{};
    std::uint8_t JumpTriggerState{};
    std::uint8_t PowerslideBitField{};
    std::uint8_t PowerslideTriggerState{};
    std::uint8_t FlipBitField{};
    std::uint8_t WaveDashClosureAssistState{};
    std::int8_t PostFlipPitchContinuationInput{};
    std::int8_t flipSteeringForward{};
    std::int8_t flipSteeringRight{};
    FVectorValue forwardFlipVector{};
    FVectorValue rightFlipVector{};
    std::uint8_t remainingFramesLongJump{};
    std::uint16_t remainingFramesBetweenJump{};
    std::uint8_t remainingFramesPostPowerslide{};
    std::uint8_t remainingFramesFlipDelay{};
    std::uint8_t remainingFramesCancelDelay{};
    std::uint8_t remainingFramesZVelocityReduction{};
    std::uint8_t remainingFramesFlip{};
    std::uint8_t remainingFramesPostFlip{};
    std::uint8_t remainingFramesPostFlipPitchLock{};
    std::uint8_t remainingFramesAutoReset{};
    std::uint8_t remainingFramesStickyForce{};
    std::uint8_t nbFramesAutoRecoverStart{};
    std::uint8_t WheelAutoRecoverStartLatchMask{};
    std::uint8_t UpsideDownGraceFramesRemaining{};
    std::array<float,4> WheelsOmega{};
    std::array<float,4> WheelsAngularPosition{};
    std::array<std::uint8_t,4> WheelsIsJumping{};
    std::array<std::uint8_t,4> WheelsGroundFrames{};
    std::uint8_t PartialJumpWheelMask{};
    std::array<float,4> FirstJumpWheelWeights{};
    std::array<float,4> ZapArmArrivalWheelWeights{};
    std::uint8_t ZapArmArrivalFramesRemaining{};
    std::uint8_t PreviousJumpContactWheelMask{};
    bool bZapArmIntentionalAirTurnSinceFullSupport{};
    std::uint8_t CoupledWheelSupportMask{};
    FVectorValue CoupledWheelSupportPlaneNormal{};
    FVectorValue CoupledWheelSupportPlanePoint{};
    bool bWheelAutoRecoverBrakingActive{};
    FVectorValue WheelAutoRecoverBrakingLocalAxis{};
    std::uint8_t NbFramesSinceTriggerAction{};
    std::uint8_t NbFramesSincePowerslideSteering{};
    std::uint8_t NbFramesSinceGroundContact{};
    std::uint8_t PhysicalBoostInput{};
    std::int8_t PhysicalAirYawInput{};
    std::int8_t PhysicalAirPitchInput{};
    std::int8_t PhysicalAirRollInput{};
    std::int8_t PhysicalLeftAirRollInput{};
    std::int8_t PhysicalRightAirRollInput{};
    std::int8_t PhysicalCamYawInput{};
    std::int8_t PhysicalCamPitchInput{};
};
struct FCarGame
{
    float CancelSpeed{};
    float TotalAccel{};
    bool bHasInfiniteBoost{};
    FVectorValue flipTorqueVector{};
    FVectorValue autoResetTorqueVector{};
};
struct FBallPhysical
{
    std::uint8_t SurfaceState{};
    bool bIsAffectedByGravity{};
    bool bHasRollingAxis{};
    FVectorValue CachedRollingAxis{};
    bool bHasFlatFloorSupport{};
    std::uint8_t FlatFloorSupportStableFrames{};
    FVectorValue FlatFloorSupportNormal{};
    FVectorValue FlatFloorSupportPoint{};
};
struct FBallGame
{
    std::uint32_t NumFrame{};
    FVectorValue TestInitialVelocity{};
};

namespace Detail
{
template<class T> bool Finite(const T&) noexcept { return true; }
inline bool Finite(float V) noexcept { return std::isfinite(V); }
inline bool Finite(const FVectorValue& V) noexcept { return std::isfinite(V.X)&&std::isfinite(V.Y)&&std::isfinite(V.Z); }
template<class T,std::size_t N> bool Finite(const std::array<T,N>& V) noexcept
{ for(const auto& X:V)if(!Finite(X))return false;return true; }
inline void Put(std::vector<std::uint8_t>& B,std::uint64_t V,std::size_t N)
{ for(std::size_t I=0;I<N;++I)B.push_back(std::uint8_t(V>>(8*I))); }
inline void Write(std::vector<std::uint8_t>& B,bool V) { Put(B,V?1:0,1); }
template<class T,std::enable_if_t<std::is_integral_v<T>&&!std::is_same_v<T,bool>,int> =0>
void Write(std::vector<std::uint8_t>& B,T V)
{ using U=std::make_unsigned_t<T>;U Bits=0;std::memcpy(&Bits,&V,sizeof(T));Put(B,Bits,sizeof(T)); }
template<class T,std::enable_if_t<std::is_floating_point_v<T>,int> =0>
void Write(std::vector<std::uint8_t>& B,T V)
{ static_assert(std::numeric_limits<T>::is_iec559);using U=std::conditional_t<sizeof(T)==4,std::uint32_t,std::uint64_t>;U Bits=0;std::memcpy(&Bits,&V,sizeof(T));Put(B,Bits,sizeof(T)); }
inline void Write(std::vector<std::uint8_t>& B,const FVectorValue& V) { Write(B,V.X);Write(B,V.Y);Write(B,V.Z); }
template<class T,std::size_t N> void Write(std::vector<std::uint8_t>& B,const std::array<T,N>& V)
{ for(const auto& X:V)Write(B,X); }
struct FReader
{
    const std::vector<std::uint8_t>& Bytes;std::size_t At=0;bool Good=true;
    std::uint64_t Get(std::size_t N) noexcept
    { if(N>8 || At>Bytes.size() || N>Bytes.size()-At){Good=false;return 0;}std::uint64_t V=0;for(std::size_t I=0;I<N;++I)V|=std::uint64_t(Bytes[At++])<<(8*I);return V; }
};
inline void Read(FReader& R,bool& V) noexcept { const auto X=R.Get(1);if(X>1)R.Good=false;V=X!=0; }
template<class T,std::enable_if_t<std::is_integral_v<T>&&!std::is_same_v<T,bool>,int> =0>
void Read(FReader& R,T& V) noexcept
{ using U=std::make_unsigned_t<T>;const U Bits=U(R.Get(sizeof(T)));std::memcpy(&V,&Bits,sizeof(T)); }
template<class T,std::enable_if_t<std::is_floating_point_v<T>,int> =0>
void Read(FReader& R,T& V) noexcept
{ using U=std::conditional_t<sizeof(T)==4,std::uint32_t,std::uint64_t>;const U Bits=U(R.Get(sizeof(T)));std::memcpy(&V,&Bits,sizeof(T)); }
inline void Read(FReader& R,FVectorValue& V) noexcept { Read(R,V.X);Read(R,V.Y);Read(R,V.Z); }
template<class T,std::size_t N> void Read(FReader& R,std::array<T,N>& V) noexcept { for(auto& X:V)Read(R,X); }
template<class T> struct TFields;
template<> struct TFields<FCarPhysical>
{
    static constexpr std::uint8_t Id=1;
    static constexpr std::size_t Bytes=237;
    template<class T,class F> static void Visit(T& V,F&& Function)
    {
        Function(V.boostQty);
        Function(V.bGroundState);
        Function(V.JumpBitField);
        Function(V.JumpPerformingBitField);
        Function(V.JumpTriggerState);
        Function(V.PowerslideBitField);
        Function(V.PowerslideTriggerState);
        Function(V.FlipBitField);
        Function(V.WaveDashClosureAssistState);
        Function(V.PostFlipPitchContinuationInput);
        Function(V.flipSteeringForward);
        Function(V.flipSteeringRight);
        Function(V.forwardFlipVector);
        Function(V.rightFlipVector);
        Function(V.remainingFramesLongJump);
        Function(V.remainingFramesBetweenJump);
        Function(V.remainingFramesPostPowerslide);
        Function(V.remainingFramesFlipDelay);
        Function(V.remainingFramesCancelDelay);
        Function(V.remainingFramesZVelocityReduction);
        Function(V.remainingFramesFlip);
        Function(V.remainingFramesPostFlip);
        Function(V.remainingFramesPostFlipPitchLock);
        Function(V.remainingFramesAutoReset);
        Function(V.remainingFramesStickyForce);
        Function(V.nbFramesAutoRecoverStart);
        Function(V.WheelAutoRecoverStartLatchMask);
        Function(V.UpsideDownGraceFramesRemaining);
        Function(V.WheelsOmega);
        Function(V.WheelsAngularPosition);
        Function(V.WheelsIsJumping);
        Function(V.WheelsGroundFrames);
        Function(V.PartialJumpWheelMask);
        Function(V.FirstJumpWheelWeights);
        Function(V.ZapArmArrivalWheelWeights);
        Function(V.ZapArmArrivalFramesRemaining);
        Function(V.PreviousJumpContactWheelMask);
        Function(V.bZapArmIntentionalAirTurnSinceFullSupport);
        Function(V.CoupledWheelSupportMask);
        Function(V.CoupledWheelSupportPlaneNormal);
        Function(V.CoupledWheelSupportPlanePoint);
        Function(V.bWheelAutoRecoverBrakingActive);
        Function(V.WheelAutoRecoverBrakingLocalAxis);
        Function(V.NbFramesSinceTriggerAction);
        Function(V.NbFramesSincePowerslideSteering);
        Function(V.NbFramesSinceGroundContact);
        Function(V.PhysicalBoostInput);
        Function(V.PhysicalAirYawInput);
        Function(V.PhysicalAirPitchInput);
        Function(V.PhysicalAirRollInput);
        Function(V.PhysicalLeftAirRollInput);
        Function(V.PhysicalRightAirRollInput);
        Function(V.PhysicalCamYawInput);
        Function(V.PhysicalCamPitchInput);
    }
};
template<> struct TFields<FCarGame>
{
    static constexpr std::uint8_t Id=2;
    static constexpr std::size_t Bytes=57;
    template<class T,class F> static void Visit(T& V,F&& Function)
    {
        Function(V.CancelSpeed);
        Function(V.TotalAccel);
        Function(V.bHasInfiniteBoost);
        Function(V.flipTorqueVector);
        Function(V.autoResetTorqueVector);
    }
};
template<> struct TFields<FBallPhysical>
{
    static constexpr std::uint8_t Id=3;
    static constexpr std::size_t Bytes=77;
    template<class T,class F> static void Visit(T& V,F&& Function)
    {
        Function(V.SurfaceState);
        Function(V.bIsAffectedByGravity);
        Function(V.bHasRollingAxis);
        Function(V.CachedRollingAxis);
        Function(V.bHasFlatFloorSupport);
        Function(V.FlatFloorSupportStableFrames);
        Function(V.FlatFloorSupportNormal);
        Function(V.FlatFloorSupportPoint);
    }
};
template<> struct TFields<FBallGame>
{
    static constexpr std::uint8_t Id=4;
    static constexpr std::size_t Bytes=28;
    template<class T,class F> static void Visit(T& V,F&& Function)
    {
        Function(V.NumFrame);
        Function(V.TestInitialVelocity);
    }
};
}
template<class T> bool Valid(const T& V) noexcept
{
    bool Good=true;Detail::TFields<T>::Visit(V,[&](const auto& X) noexcept { Good=Good&&Detail::Finite(X); });
    if constexpr(std::is_same_v<T,FBallPhysical>)Good=Good && V.SurfaceState<=2;
    return Good;
}
template<class T> std::optional<std::vector<std::uint8_t>> Encode(FAddress At,const T V)
{
    if(!At.Valid()||!Valid(V))return {};
    std::vector<std::uint8_t> B;B.reserve(40+Detail::TFields<T>::Bytes);
    Detail::Put(B,0x31444753,4);Detail::Put(B,1,2);Detail::Put(B,Detail::TFields<T>::Id,1);Detail::Put(B,0,1);
    for(const auto X:{At.Frame,At.RegistryGeneration,At.Actor,At.ActorIncarnation})Detail::Put(B,X,8);
    Detail::TFields<T>::Visit(V,[&](const auto& X){Detail::Write(B,X);});
    if(B.size()!=40+Detail::TFields<T>::Bytes)return {};return B;
}
template<class T> std::optional<T> Decode(FAddress Expected,const std::vector<std::uint8_t>& B)
{
    if(!Expected.Valid()||B.size()!=40+Detail::TFields<T>::Bytes)return {};
    Detail::FReader R{B};if(R.Get(4)!=0x31444753||R.Get(2)!=1||R.Get(1)!=Detail::TFields<T>::Id||R.Get(1))return {};
    for(const auto X:{Expected.Frame,Expected.RegistryGeneration,Expected.Actor,Expected.ActorIncarnation})if(R.Get(8)!=X)return {};
    T V;Detail::TFields<T>::Visit(V,[&](auto& X)noexcept{Detail::Read(R,X);});
    if(!R.Good||R.At!=B.size()||!Valid(V))return {};return V;
}
// Direct named field copies only, without getters, setter effects, physics
// steps, normalization or NetSerialize quantization. Install is for a PRIVATE
// prepared copy under the complete native transaction, not a live actor API.
template<class TNative> std::optional<FCarPhysical> CaptureCarPhysical(const TNative& Native) noexcept
{
    FCarPhysical Value;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.boostQty)>,std::uint16_t>);
    Value.boostQty=Native.boostQty;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.bGroundState)>,bool>);
    Value.bGroundState=Native.bGroundState;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.JumpBitField)>,std::uint8_t>);
    Value.JumpBitField=Native.JumpBitField;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.JumpPerformingBitField)>,std::uint8_t>);
    Value.JumpPerformingBitField=Native.JumpPerformingBitField;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.JumpTriggerState)>,std::uint8_t>);
    Value.JumpTriggerState=Native.JumpTriggerState;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.PowerslideBitField)>,std::uint8_t>);
    Value.PowerslideBitField=Native.PowerslideBitField;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.PowerslideTriggerState)>,std::uint8_t>);
    Value.PowerslideTriggerState=Native.PowerslideTriggerState;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.FlipBitField)>,std::uint8_t>);
    Value.FlipBitField=Native.FlipBitField;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.WaveDashClosureAssistState)>,std::uint8_t>);
    Value.WaveDashClosureAssistState=Native.WaveDashClosureAssistState;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.PostFlipPitchContinuationInput)>,std::int8_t>);
    Value.PostFlipPitchContinuationInput=Native.PostFlipPitchContinuationInput;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.flipSteeringForward)>,std::int8_t>);
    Value.flipSteeringForward=Native.flipSteeringForward;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.flipSteeringRight)>,std::int8_t>);
    Value.flipSteeringRight=Native.flipSteeringRight;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.forwardFlipVector.X)>,double>);
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.forwardFlipVector.Y)>,double>);
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.forwardFlipVector.Z)>,double>);
    Value.forwardFlipVector={Native.forwardFlipVector.X,Native.forwardFlipVector.Y,Native.forwardFlipVector.Z};
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.rightFlipVector.X)>,double>);
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.rightFlipVector.Y)>,double>);
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.rightFlipVector.Z)>,double>);
    Value.rightFlipVector={Native.rightFlipVector.X,Native.rightFlipVector.Y,Native.rightFlipVector.Z};
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.remainingFramesLongJump)>,std::uint8_t>);
    Value.remainingFramesLongJump=Native.remainingFramesLongJump;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.remainingFramesBetweenJump)>,std::uint16_t>);
    Value.remainingFramesBetweenJump=Native.remainingFramesBetweenJump;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.remainingFramesPostPowerslide)>,std::uint8_t>);
    Value.remainingFramesPostPowerslide=Native.remainingFramesPostPowerslide;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.remainingFramesFlipDelay)>,std::uint8_t>);
    Value.remainingFramesFlipDelay=Native.remainingFramesFlipDelay;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.remainingFramesCancelDelay)>,std::uint8_t>);
    Value.remainingFramesCancelDelay=Native.remainingFramesCancelDelay;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.remainingFramesZVelocityReduction)>,std::uint8_t>);
    Value.remainingFramesZVelocityReduction=Native.remainingFramesZVelocityReduction;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.remainingFramesFlip)>,std::uint8_t>);
    Value.remainingFramesFlip=Native.remainingFramesFlip;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.remainingFramesPostFlip)>,std::uint8_t>);
    Value.remainingFramesPostFlip=Native.remainingFramesPostFlip;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.remainingFramesPostFlipPitchLock)>,std::uint8_t>);
    Value.remainingFramesPostFlipPitchLock=Native.remainingFramesPostFlipPitchLock;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.remainingFramesAutoReset)>,std::uint8_t>);
    Value.remainingFramesAutoReset=Native.remainingFramesAutoReset;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.remainingFramesStickyForce)>,std::uint8_t>);
    Value.remainingFramesStickyForce=Native.remainingFramesStickyForce;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.nbFramesAutoRecoverStart)>,std::uint8_t>);
    Value.nbFramesAutoRecoverStart=Native.nbFramesAutoRecoverStart;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.WheelAutoRecoverStartLatchMask)>,std::uint8_t>);
    Value.WheelAutoRecoverStartLatchMask=Native.WheelAutoRecoverStartLatchMask;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.UpsideDownGraceFramesRemaining)>,std::uint8_t>);
    Value.UpsideDownGraceFramesRemaining=Native.UpsideDownGraceFramesRemaining;
    static_assert(std::extent_v<decltype(Native.WheelsOmega)> ==4);
    static_assert(std::is_same_v<std::remove_extent_t<decltype(Native.WheelsOmega)>,float>);
    for(std::size_t I=0;I<4;++I)Value.WheelsOmega[I]=Native.WheelsOmega[I];
    static_assert(std::extent_v<decltype(Native.WheelsAngularPosition)> ==4);
    static_assert(std::is_same_v<std::remove_extent_t<decltype(Native.WheelsAngularPosition)>,float>);
    for(std::size_t I=0;I<4;++I)Value.WheelsAngularPosition[I]=Native.WheelsAngularPosition[I];
    static_assert(std::extent_v<decltype(Native.WheelsIsJumping)> ==4);
    static_assert(std::is_same_v<std::remove_extent_t<decltype(Native.WheelsIsJumping)>,std::uint8_t>);
    for(std::size_t I=0;I<4;++I)Value.WheelsIsJumping[I]=Native.WheelsIsJumping[I];
    static_assert(std::extent_v<decltype(Native.WheelsGroundFrames)> ==4);
    static_assert(std::is_same_v<std::remove_extent_t<decltype(Native.WheelsGroundFrames)>,std::uint8_t>);
    for(std::size_t I=0;I<4;++I)Value.WheelsGroundFrames[I]=Native.WheelsGroundFrames[I];
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.PartialJumpWheelMask)>,std::uint8_t>);
    Value.PartialJumpWheelMask=Native.PartialJumpWheelMask;
    static_assert(std::extent_v<decltype(Native.FirstJumpWheelWeights)> ==4);
    static_assert(std::is_same_v<std::remove_extent_t<decltype(Native.FirstJumpWheelWeights)>,float>);
    for(std::size_t I=0;I<4;++I)Value.FirstJumpWheelWeights[I]=Native.FirstJumpWheelWeights[I];
    static_assert(std::extent_v<decltype(Native.ZapArmArrivalWheelWeights)> ==4);
    static_assert(std::is_same_v<std::remove_extent_t<decltype(Native.ZapArmArrivalWheelWeights)>,float>);
    for(std::size_t I=0;I<4;++I)Value.ZapArmArrivalWheelWeights[I]=Native.ZapArmArrivalWheelWeights[I];
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.ZapArmArrivalFramesRemaining)>,std::uint8_t>);
    Value.ZapArmArrivalFramesRemaining=Native.ZapArmArrivalFramesRemaining;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.PreviousJumpContactWheelMask)>,std::uint8_t>);
    Value.PreviousJumpContactWheelMask=Native.PreviousJumpContactWheelMask;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.bZapArmIntentionalAirTurnSinceFullSupport)>,bool>);
    Value.bZapArmIntentionalAirTurnSinceFullSupport=Native.bZapArmIntentionalAirTurnSinceFullSupport;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.CoupledWheelSupportMask)>,std::uint8_t>);
    Value.CoupledWheelSupportMask=Native.CoupledWheelSupportMask;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.CoupledWheelSupportPlaneNormal.X)>,double>);
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.CoupledWheelSupportPlaneNormal.Y)>,double>);
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.CoupledWheelSupportPlaneNormal.Z)>,double>);
    Value.CoupledWheelSupportPlaneNormal={Native.CoupledWheelSupportPlaneNormal.X,Native.CoupledWheelSupportPlaneNormal.Y,Native.CoupledWheelSupportPlaneNormal.Z};
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.CoupledWheelSupportPlanePoint.X)>,double>);
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.CoupledWheelSupportPlanePoint.Y)>,double>);
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.CoupledWheelSupportPlanePoint.Z)>,double>);
    Value.CoupledWheelSupportPlanePoint={Native.CoupledWheelSupportPlanePoint.X,Native.CoupledWheelSupportPlanePoint.Y,Native.CoupledWheelSupportPlanePoint.Z};
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.bWheelAutoRecoverBrakingActive)>,bool>);
    Value.bWheelAutoRecoverBrakingActive=Native.bWheelAutoRecoverBrakingActive;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.WheelAutoRecoverBrakingLocalAxis.X)>,double>);
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.WheelAutoRecoverBrakingLocalAxis.Y)>,double>);
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.WheelAutoRecoverBrakingLocalAxis.Z)>,double>);
    Value.WheelAutoRecoverBrakingLocalAxis={Native.WheelAutoRecoverBrakingLocalAxis.X,Native.WheelAutoRecoverBrakingLocalAxis.Y,Native.WheelAutoRecoverBrakingLocalAxis.Z};
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.NbFramesSinceTriggerAction)>,std::uint8_t>);
    Value.NbFramesSinceTriggerAction=Native.NbFramesSinceTriggerAction;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.NbFramesSincePowerslideSteering)>,std::uint8_t>);
    Value.NbFramesSincePowerslideSteering=Native.NbFramesSincePowerslideSteering;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.NbFramesSinceGroundContact)>,std::uint8_t>);
    Value.NbFramesSinceGroundContact=Native.NbFramesSinceGroundContact;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.PhysicalBoostInput)>,std::uint8_t>);
    Value.PhysicalBoostInput=Native.PhysicalBoostInput;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.PhysicalAirYawInput)>,std::int8_t>);
    Value.PhysicalAirYawInput=Native.PhysicalAirYawInput;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.PhysicalAirPitchInput)>,std::int8_t>);
    Value.PhysicalAirPitchInput=Native.PhysicalAirPitchInput;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.PhysicalAirRollInput)>,std::int8_t>);
    Value.PhysicalAirRollInput=Native.PhysicalAirRollInput;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.PhysicalLeftAirRollInput)>,std::int8_t>);
    Value.PhysicalLeftAirRollInput=Native.PhysicalLeftAirRollInput;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.PhysicalRightAirRollInput)>,std::int8_t>);
    Value.PhysicalRightAirRollInput=Native.PhysicalRightAirRollInput;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.PhysicalCamYawInput)>,std::int8_t>);
    Value.PhysicalCamYawInput=Native.PhysicalCamYawInput;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.PhysicalCamPitchInput)>,std::int8_t>);
    Value.PhysicalCamPitchInput=Native.PhysicalCamPitchInput;
    return Valid(Value)?std::optional<FCarPhysical>(Value):std::nullopt;
}
template<class TNative> void InstallCarPhysicalPrivateCopy(TNative& Native,const FCarPhysical Value) noexcept
{
    Native.boostQty=Value.boostQty;
    Native.bGroundState=Value.bGroundState;
    Native.JumpBitField=Value.JumpBitField;
    Native.JumpPerformingBitField=Value.JumpPerformingBitField;
    Native.JumpTriggerState=Value.JumpTriggerState;
    Native.PowerslideBitField=Value.PowerslideBitField;
    Native.PowerslideTriggerState=Value.PowerslideTriggerState;
    Native.FlipBitField=Value.FlipBitField;
    Native.WaveDashClosureAssistState=Value.WaveDashClosureAssistState;
    Native.PostFlipPitchContinuationInput=Value.PostFlipPitchContinuationInput;
    Native.flipSteeringForward=Value.flipSteeringForward;
    Native.flipSteeringRight=Value.flipSteeringRight;
    Native.forwardFlipVector.X=Value.forwardFlipVector.X;Native.forwardFlipVector.Y=Value.forwardFlipVector.Y;Native.forwardFlipVector.Z=Value.forwardFlipVector.Z;
    Native.rightFlipVector.X=Value.rightFlipVector.X;Native.rightFlipVector.Y=Value.rightFlipVector.Y;Native.rightFlipVector.Z=Value.rightFlipVector.Z;
    Native.remainingFramesLongJump=Value.remainingFramesLongJump;
    Native.remainingFramesBetweenJump=Value.remainingFramesBetweenJump;
    Native.remainingFramesPostPowerslide=Value.remainingFramesPostPowerslide;
    Native.remainingFramesFlipDelay=Value.remainingFramesFlipDelay;
    Native.remainingFramesCancelDelay=Value.remainingFramesCancelDelay;
    Native.remainingFramesZVelocityReduction=Value.remainingFramesZVelocityReduction;
    Native.remainingFramesFlip=Value.remainingFramesFlip;
    Native.remainingFramesPostFlip=Value.remainingFramesPostFlip;
    Native.remainingFramesPostFlipPitchLock=Value.remainingFramesPostFlipPitchLock;
    Native.remainingFramesAutoReset=Value.remainingFramesAutoReset;
    Native.remainingFramesStickyForce=Value.remainingFramesStickyForce;
    Native.nbFramesAutoRecoverStart=Value.nbFramesAutoRecoverStart;
    Native.WheelAutoRecoverStartLatchMask=Value.WheelAutoRecoverStartLatchMask;
    Native.UpsideDownGraceFramesRemaining=Value.UpsideDownGraceFramesRemaining;
    for(std::size_t I=0;I<4;++I)Native.WheelsOmega[I]=Value.WheelsOmega[I];
    for(std::size_t I=0;I<4;++I)Native.WheelsAngularPosition[I]=Value.WheelsAngularPosition[I];
    for(std::size_t I=0;I<4;++I)Native.WheelsIsJumping[I]=Value.WheelsIsJumping[I];
    for(std::size_t I=0;I<4;++I)Native.WheelsGroundFrames[I]=Value.WheelsGroundFrames[I];
    Native.PartialJumpWheelMask=Value.PartialJumpWheelMask;
    for(std::size_t I=0;I<4;++I)Native.FirstJumpWheelWeights[I]=Value.FirstJumpWheelWeights[I];
    for(std::size_t I=0;I<4;++I)Native.ZapArmArrivalWheelWeights[I]=Value.ZapArmArrivalWheelWeights[I];
    Native.ZapArmArrivalFramesRemaining=Value.ZapArmArrivalFramesRemaining;
    Native.PreviousJumpContactWheelMask=Value.PreviousJumpContactWheelMask;
    Native.bZapArmIntentionalAirTurnSinceFullSupport=Value.bZapArmIntentionalAirTurnSinceFullSupport;
    Native.CoupledWheelSupportMask=Value.CoupledWheelSupportMask;
    Native.CoupledWheelSupportPlaneNormal.X=Value.CoupledWheelSupportPlaneNormal.X;Native.CoupledWheelSupportPlaneNormal.Y=Value.CoupledWheelSupportPlaneNormal.Y;Native.CoupledWheelSupportPlaneNormal.Z=Value.CoupledWheelSupportPlaneNormal.Z;
    Native.CoupledWheelSupportPlanePoint.X=Value.CoupledWheelSupportPlanePoint.X;Native.CoupledWheelSupportPlanePoint.Y=Value.CoupledWheelSupportPlanePoint.Y;Native.CoupledWheelSupportPlanePoint.Z=Value.CoupledWheelSupportPlanePoint.Z;
    Native.bWheelAutoRecoverBrakingActive=Value.bWheelAutoRecoverBrakingActive;
    Native.WheelAutoRecoverBrakingLocalAxis.X=Value.WheelAutoRecoverBrakingLocalAxis.X;Native.WheelAutoRecoverBrakingLocalAxis.Y=Value.WheelAutoRecoverBrakingLocalAxis.Y;Native.WheelAutoRecoverBrakingLocalAxis.Z=Value.WheelAutoRecoverBrakingLocalAxis.Z;
    Native.NbFramesSinceTriggerAction=Value.NbFramesSinceTriggerAction;
    Native.NbFramesSincePowerslideSteering=Value.NbFramesSincePowerslideSteering;
    Native.NbFramesSinceGroundContact=Value.NbFramesSinceGroundContact;
    Native.PhysicalBoostInput=Value.PhysicalBoostInput;
    Native.PhysicalAirYawInput=Value.PhysicalAirYawInput;
    Native.PhysicalAirPitchInput=Value.PhysicalAirPitchInput;
    Native.PhysicalAirRollInput=Value.PhysicalAirRollInput;
    Native.PhysicalLeftAirRollInput=Value.PhysicalLeftAirRollInput;
    Native.PhysicalRightAirRollInput=Value.PhysicalRightAirRollInput;
    Native.PhysicalCamYawInput=Value.PhysicalCamYawInput;
    Native.PhysicalCamPitchInput=Value.PhysicalCamPitchInput;
}
template<class TNative> std::optional<FCarGame> CaptureCarGame(const TNative& Native) noexcept
{
    FCarGame Value;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.CancelSpeed)>,float>);
    Value.CancelSpeed=Native.CancelSpeed;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.TotalAccel)>,float>);
    Value.TotalAccel=Native.TotalAccel;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.bHasInfiniteBoost)>,bool>);
    Value.bHasInfiniteBoost=Native.bHasInfiniteBoost;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.flipTorqueVector.X)>,double>);
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.flipTorqueVector.Y)>,double>);
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.flipTorqueVector.Z)>,double>);
    Value.flipTorqueVector={Native.flipTorqueVector.X,Native.flipTorqueVector.Y,Native.flipTorqueVector.Z};
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.autoResetTorqueVector.X)>,double>);
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.autoResetTorqueVector.Y)>,double>);
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.autoResetTorqueVector.Z)>,double>);
    Value.autoResetTorqueVector={Native.autoResetTorqueVector.X,Native.autoResetTorqueVector.Y,Native.autoResetTorqueVector.Z};
    return Valid(Value)?std::optional<FCarGame>(Value):std::nullopt;
}
template<class TNative> void InstallCarGamePrivateCopy(TNative& Native,const FCarGame Value) noexcept
{
    Native.CancelSpeed=Value.CancelSpeed;
    Native.TotalAccel=Value.TotalAccel;
    Native.bHasInfiniteBoost=Value.bHasInfiniteBoost;
    Native.flipTorqueVector.X=Value.flipTorqueVector.X;Native.flipTorqueVector.Y=Value.flipTorqueVector.Y;Native.flipTorqueVector.Z=Value.flipTorqueVector.Z;
    Native.autoResetTorqueVector.X=Value.autoResetTorqueVector.X;Native.autoResetTorqueVector.Y=Value.autoResetTorqueVector.Y;Native.autoResetTorqueVector.Z=Value.autoResetTorqueVector.Z;
}
template<class TNative> std::optional<FBallPhysical> CaptureBallPhysical(const TNative& Native) noexcept
{
    FBallPhysical Value;
    static_assert(std::is_enum_v<decltype(Native.SurfaceState)> && sizeof(Native.SurfaceState)==1);
    Value.SurfaceState=std::uint8_t(Native.SurfaceState);
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.bIsAffectedByGravity)>,bool>);
    Value.bIsAffectedByGravity=Native.bIsAffectedByGravity;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.bHasRollingAxis)>,bool>);
    Value.bHasRollingAxis=Native.bHasRollingAxis;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.CachedRollingAxis.X)>,double>);
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.CachedRollingAxis.Y)>,double>);
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.CachedRollingAxis.Z)>,double>);
    Value.CachedRollingAxis={Native.CachedRollingAxis.X,Native.CachedRollingAxis.Y,Native.CachedRollingAxis.Z};
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.bHasFlatFloorSupport)>,bool>);
    Value.bHasFlatFloorSupport=Native.bHasFlatFloorSupport;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.FlatFloorSupportStableFrames)>,std::uint8_t>);
    Value.FlatFloorSupportStableFrames=Native.FlatFloorSupportStableFrames;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.FlatFloorSupportNormal.X)>,double>);
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.FlatFloorSupportNormal.Y)>,double>);
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.FlatFloorSupportNormal.Z)>,double>);
    Value.FlatFloorSupportNormal={Native.FlatFloorSupportNormal.X,Native.FlatFloorSupportNormal.Y,Native.FlatFloorSupportNormal.Z};
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.FlatFloorSupportPoint.X)>,double>);
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.FlatFloorSupportPoint.Y)>,double>);
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.FlatFloorSupportPoint.Z)>,double>);
    Value.FlatFloorSupportPoint={Native.FlatFloorSupportPoint.X,Native.FlatFloorSupportPoint.Y,Native.FlatFloorSupportPoint.Z};
    return Valid(Value)?std::optional<FBallPhysical>(Value):std::nullopt;
}
template<class TNative> void InstallBallPhysicalPrivateCopy(TNative& Native,const FBallPhysical Value) noexcept
{
    Native.SurfaceState=static_cast<decltype(Native.SurfaceState)>(Value.SurfaceState);
    Native.bIsAffectedByGravity=Value.bIsAffectedByGravity;
    Native.bHasRollingAxis=Value.bHasRollingAxis;
    Native.CachedRollingAxis.X=Value.CachedRollingAxis.X;Native.CachedRollingAxis.Y=Value.CachedRollingAxis.Y;Native.CachedRollingAxis.Z=Value.CachedRollingAxis.Z;
    Native.bHasFlatFloorSupport=Value.bHasFlatFloorSupport;
    Native.FlatFloorSupportStableFrames=Value.FlatFloorSupportStableFrames;
    Native.FlatFloorSupportNormal.X=Value.FlatFloorSupportNormal.X;Native.FlatFloorSupportNormal.Y=Value.FlatFloorSupportNormal.Y;Native.FlatFloorSupportNormal.Z=Value.FlatFloorSupportNormal.Z;
    Native.FlatFloorSupportPoint.X=Value.FlatFloorSupportPoint.X;Native.FlatFloorSupportPoint.Y=Value.FlatFloorSupportPoint.Y;Native.FlatFloorSupportPoint.Z=Value.FlatFloorSupportPoint.Z;
}
template<class TNative> std::optional<FBallGame> CaptureBallGame(const TNative& Native) noexcept
{
    FBallGame Value;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.NumFrame)>,std::uint32_t>);
    Value.NumFrame=Native.NumFrame;
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.TestInitialVelocity.X)>,double>);
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.TestInitialVelocity.Y)>,double>);
    static_assert(std::is_same_v<std::remove_cv_t<decltype(Native.TestInitialVelocity.Z)>,double>);
    Value.TestInitialVelocity={Native.TestInitialVelocity.X,Native.TestInitialVelocity.Y,Native.TestInitialVelocity.Z};
    return Valid(Value)?std::optional<FBallGame>(Value):std::nullopt;
}
template<class TNative> void InstallBallGamePrivateCopy(TNative& Native,const FBallGame Value) noexcept
{
    Native.NumFrame=Value.NumFrame;
    Native.TestInitialVelocity.X=Value.TestInitialVelocity.X;Native.TestInitialVelocity.Y=Value.TestInitialVelocity.Y;Native.TestInitialVelocity.Z=Value.TestInitialVelocity.Z;
}
}
