#include "IAmSpeed/Base/CarBumpCooldown.h"
#include "IAmSpeed/Base/CarBumpSnapshot.h"
#include <cstdio>
#include <initializer_list>
using namespace Speed::Collision;
static int checks=0,failures=0;
static void Test(bool B,const char* N){++checks;if(!B){if(failures<6)std::printf("FAIL %s\n",N);++failures;}}
int main(){
    for(unsigned rate:{60u,120u,300u}){
        std::uint64_t last=11;std::uint16_t frames=static_cast<std::uint16_t>(rate/4);
        const unsigned duration=frames;
        for(unsigned step=0;step<duration;++step){
            Test(IsCarBumpInCooldown(last,std::uint64_t(11),frames),"same victim held before250ms boundary");
            Test(!IsCarBumpInCooldown(last,std::uint64_t(12),frames),"different victim remains eligible");
            const auto bytes=EncodeCarBumpSnapshot(frames,last);
            std::uint16_t restoredFrames=0;std::uint64_t restoredLast=0;
            Test(DecodeCarBumpSnapshot(bytes.data(),bytes.size(),restoredFrames,restoredLast),"active state decodes");
            Test(restoredFrames==frames&&restoredLast==last,"snapshot restore retains future bump eligibility");
            AdvanceCarBumpCooldown(last,frames);AdvanceCarBumpCooldown(restoredLast,restoredFrames);
            Test(restoredFrames==frames&&restoredLast==last,"replay tick matches uninterrupted state");
        }
        Test(frames==0&&last==0&&!IsCarBumpInCooldown(last,std::uint64_t(11),frames),"250ms boundary releases same victim exactly");
        AdvanceCarBumpCooldown(last,frames);Test(frames==0&&last==0,"neutral state does not underflow");
    }
    const auto neutral=EncodeCarBumpSnapshot(0,0);
    std::uint16_t f=999;std::uint64_t id=888;
    for(std::size_t len=0;len<neutral.size();++len){
        Test(!DecodeCarBumpSnapshot(neutral.data(),len,f,id),"truncated payload rejects");
        Test(f==999&&id==888,"failed decode leaves output transaction unchanged");
    }
    auto bad=neutral;bad[0]=2;Test(!DecodeCarBumpSnapshot(bad.data(),bad.size(),f,id),"unknown schema rejects");
    for(const auto pair:{EncodeCarBumpSnapshot(75,0),EncodeCarBumpSnapshot(0,11)})
        Test(!DecodeCarBumpSnapshot(pair.data(),pair.size(),f,id),"inconsistent target/timer pair rejects");
    Test(!DecodeCarBumpSnapshot(nullptr,neutral.size(),f,id),"null payload rejects");
    Test(DecodeCarBumpSnapshot(neutral.data(),neutral.size(),f,id)&&f==0&&id==0,"neutral state restores");
    int actor=0;int* p=&actor;const int* cp=&actor;
    Test(IsCarBumpInCooldown(p,cp,1),"const and mutable target views share identity");
    std::uint16_t one=1;AdvanceCarBumpCooldown(p,one);Test(!p&&one==0,"pointer target clears at expiry");
    std::printf("BUMP_STATE assertions=%d failures=%d scope=production_cooldown_and_snapshot_codec\n",checks,failures);
    return failures?1:0;
}
