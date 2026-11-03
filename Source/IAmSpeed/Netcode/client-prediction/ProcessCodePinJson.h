#pragma once
#include "NativeProcessCodePins.h"
#include <ostream>
namespace Speed::Netcode::Prediction
{
// Immutable actual diagnostic data only. The fragments grant no admission,
// incarnation, source lifetime, world lease or restoration permission.
inline void AppendProcessCodePinRecords(std::ostream& Out,const FNativeProcessCodePins& Code)
{
    Out<<R"(,"modules":[)";
    bool First=true;for(const auto& R:Code.Records())
    {
        if(!First)Out<<',';First=false;
        Out<<R"({"enumerated_base":)"<<R.EnumeratedBase<<R"(,"returned_base":)"<<R.ReturnedBase
            <<R"(,"attempted":)"<<int(R.Attempted)<<R"(,"pin_returned":)"<<int(R.PinReturned)<<R"(,"error":)"<<R.Error<<'}';
    }
    Out<<']';
}
inline void AppendProcessCodePinPasses(std::ostream& Out,const FNativeProcessCodePins& Code)
{
    Out<<R"(,"pin_pass_count":)"<<Code.PassCount()<<R"(,"pin_passes":[)";
    for(std::size_t I=0;I<Code.PassCount()&&I<Code.Passes().size();++I)
    {
        if(I)Out<<',';const auto& P=Code.Passes()[I];
        Out<<R"({"input_is_initial":)"<<int(P.InputIsInitial)<<R"(,"input_after_pass":)"<<P.InputAfterPass
            <<R"(,"input_succeeded":)"<<int(P.InputSucceeded)<<R"(,"input_bytes":)"<<P.InputBytes
            <<R"(,"input_available":)"<<P.InputAvailable<<R"(,"input_error":)"<<P.InputError
            <<R"(,"after_succeeded":)"<<int(P.AfterSucceeded)<<R"(,"after_bytes":)"<<P.AfterBytes
            <<R"(,"after_available":)"<<P.AfterAvailable<<R"(,"after_error":)"<<P.AfterError
            <<R"(,"new_pin_attempts":)"<<P.NewPinAttempts<<R"(,"transition":)"<<int(P.Transition)
            <<R"(,"window_installed":)"<<int(P.WindowInstalled)<<R"(,"window_signaled":)"<<int(P.WindowSignaled)
            <<R"(,"window_wait_returned":)"<<int(P.WindowWaitReturned)<<R"(,"window_wait_result":)"<<P.WindowWaitResult
            <<R"(,"window_signal_error":)"<<P.WindowSignalError<<R"(,"window_wait_error":)"<<P.WindowWaitError
            <<R"(,"input_bases":[)";
        for(std::size_t J=0;J<P.InputAvailable&&J<P.InputBases.size();++J){if(J)Out<<',';Out<<P.InputBases[J];}
        Out<<R"(],"after_bases":[)";
        for(std::size_t J=0;J<P.AfterAvailable&&J<P.AfterBases.size();++J){if(J)Out<<',';Out<<P.AfterBases[J];}
        Out<<"]}";
    }
    Out<<']';
}
}
