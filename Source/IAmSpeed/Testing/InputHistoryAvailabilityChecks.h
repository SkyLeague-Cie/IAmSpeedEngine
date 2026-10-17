#pragma once
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "Physics/NetworkPhysicsComponent.h"
#include "Serialization/BitReader.h"
#include "Serialization/BitWriter.h"
#include "Serialization/MemoryReader.h"
#include "Serialization/MemoryWriter.h"

// Genuine UE history-container checks. These are local framing regressions,
// never evidence of actual peer transport or a physical rollback transaction.
template<typename DataType,typename SetInput,typename CheckInput>
bool RunInputHistoryAvailabilityChecks(FAutomationTestBase& Test,SetInput Set,CheckInput Check)
{
    for(bool bIncludeAbsent : {false,true})
    {
        TNetRewindHistory<DataType,true> Saved(3,true);
        for(int32 Frame=100;Frame<=102;++Frame)
        {
            DataType Input;
            Input.LocalFrame=Input.ServerFrame=Frame;
            Input.ClientFrame=Frame;
            Set(Input,Frame);
            if(bIncludeAbsent && Frame==101) Input.BuildData(nullptr);
            if(!Test.TestTrue(TEXT("Record real history entry"),Saved.RecordData(Frame,&Input))) return false;
        }
        FBitWriter Writer(65536,true);
        Saved.NetSerialize(Writer,nullptr,nullptr);
        if(!Test.TestFalse(TEXT("Explicitly absent slot keeps history framing valid"),Writer.IsError())) return false;
        TNetRewindHistory<DataType,true> Loaded(3,false);
        FBitReader Reader(Writer.GetData(),Writer.GetNumBits());
        Loaded.NetSerialize(Reader,nullptr,nullptr);
        if(!Test.TestFalse(TEXT("History with absent middle slot decodes"),Reader.IsError())) return false;
        Test.TestEqual(TEXT("No leftover or missing bits"),Reader.GetBitsLeft(),int64(0));
        // The decoded collection is chronological, not the modulo-indexed
        // simulation history. Use the actual receive path before extraction.
        constexpr int32 FrameOffset=7;
        TNetRewindHistory<DataType,true> Received(8,true);
        Received.ReceiveNewData(Loaded,FrameOffset,false,false);
        for(int32 Frame=100;Frame<=102;++Frame)
        {
            DataType Input;
            if(!Test.TestTrue(TEXT("Extract actual received slot"),Received.ExtractData(Frame-FrameOffset,true,&Input,true))) return false;
            Test.TestEqual(TEXT("Receive path maps the real local frame"),int32(Input.LocalFrame),Frame-FrameOffset);
            Test.TestEqual(TEXT("Receive path preserves server frame"),int32(Input.ServerFrame),Frame);
            Test.TestTrue(TEXT("Actual receive path marks every slot received"),Input.bReceivedData);
            const bool bExpected=!(bIncludeAbsent && Frame==101);
            Test.TestEqual(TEXT("Availability survives history serialization"),Input.bInputProjectionValid,bExpected);
            if(bExpected)
            {
                Test.TestEqual(TEXT("Real source frame survives after absent slot"),Input.ClientFrame,uint32(Frame));
                Test.TestTrue(TEXT("Real input payload survives after absent slot"),Check(Input,Frame));
            }
        }
    }
    DataType Packet;
    Packet.LocalFrame=Packet.ServerFrame=100;
    Packet.ClientFrame=100; Set(Packet,100);
    TArray<uint8> Bytes;
    FMemoryWriter Writer(Bytes); bool bSaved=false;
    if(!Test.TestTrue(TEXT("Complete envelope serializes"),Packet.NetSerialize(Writer,nullptr,bSaved) && bSaved && !Writer.IsError())) return false;
    if(!Test.TestTrue(TEXT("Versioned envelope is explicit"),Bytes.Num()>2 && Bytes[0]==1 && Bytes[1]==1)) return false;
    for(int32 Corruption=0;Corruption<3;++Corruption)
    {
        TArray<uint8> Bad=Bytes;
        if(Corruption==0) Bad[0]=2;
        else if(Corruption==1) Bad[1]=2;
        else Bad.SetNum(Bad.Num()-1);
        DataType Destination;
        Destination.LocalFrame=Destination.ServerFrame=3111;
        Destination.ClientFrame=3111; Set(Destination,3111);
        FMemoryReader Reader(Bad); bool bSuccess=true;
        Test.TestFalse(TEXT("Unknown envelope or truncated payload rejected"),Destination.NetSerialize(Reader,nullptr,bSuccess));
        Test.TestTrue(TEXT("Malformed archive remains erroneous"),Reader.IsError() && !bSuccess);
        Test.TestEqual(TEXT("Failed decode preserves source frame"),Destination.ClientFrame,uint32(3111));
        Test.TestEqual(TEXT("Failed decode preserves base frame"),int32(Destination.LocalFrame),int32(3111));
        Test.TestTrue(TEXT("Failed decode preserves requested input"),Check(Destination,3111));
    }
    DataType Available,Absent,Interpolated;
    Available.LocalFrame=Available.ServerFrame=100; Available.ClientFrame=100; Set(Available,100);
    Absent=Available; Absent.LocalFrame=Absent.ServerFrame=102; Absent.BuildData(nullptr);
    Interpolated.LocalFrame=101;
    Interpolated.InterpolateData(Available,Absent);
    Test.TestFalse(TEXT("Interpolation cannot manufacture missing input"),Interpolated.bInputProjectionValid);
    Test.TestTrue(TEXT("Two absent commands compare as absent"),Absent.CompareData(Absent));
    Test.TestFalse(TEXT("Available and absent remain distinct"),Available.CompareData(Absent));
    Test.TestFalse(TEXT("Absent and available remain distinct"),Absent.CompareData(Available));
    Available.MergeData(Absent);
    Test.TestTrue(TEXT("Absent history cannot alter real current input"),Available.bInputProjectionValid && Check(Available,100));
    Absent.MergeData(Available);
    Test.TestFalse(TEXT("Older command cannot manufacture a missing newer command"),Absent.bInputProjectionValid);
    return true;
}
#endif
