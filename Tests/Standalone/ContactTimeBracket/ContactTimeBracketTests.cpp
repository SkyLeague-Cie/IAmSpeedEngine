#include "IAmSpeed/World/Collision/ContactTimeBracket.h"
#include <cmath>
#include <initializer_list>
#include <cstdio>
#include <limits>
int main(){
    int cases=0,failures=0;double worstOld=0,worstNew=0;
    // Translating156cm-wide box pair: the signed center gap has a known exact
    // root. This is a time-bracket unit fixture, not an alternative OBB solver.
    for(double closing:{300.,1200.,2400.,4600.})for(int phase=1;phase<=99;++phase){
        const float low=0,high=1.f/(300.f*12.f);
        const double root=double(high)*phase/100.;
        const auto Overlap=[&](float t){return double(t)>=root;};
        const auto b=Speed::Collision::RefineFirstOverlap(low,high,Overlap);
        ++cases;
        const bool ok=!Overlap(b.SeparatedTime)&&Overlap(b.OverlapTime)&&
            b.SeparatedTime>=low&&b.OverlapTime<=high&&
            std::nextafter(b.SeparatedTime,std::numeric_limits<float>::infinity())>=b.OverlapTime;
        if(!ok)++failures;
        const double oldPen=closing*(double(high)-root);
        const double newPen=closing*(double(b.SeparatedTime)-root);
        if(oldPen>worstOld)worstOld=oldPen;
        if(newPen>worstNew)worstNew=newPen;
    }
    // Nonmonotonic callback: maintain endpoint truth, without claiming this
    // bisection proves which disconnected overlap interval is first.
    auto r=Speed::Collision::RefineFirstOverlap(0.f,1.f,[](float t){return (t>.1f&&t<.2f)||t>.9f;});
    if(!(r.SeparatedTime>.89f && r.OverlapTime>.9f))++failures; // explicit earlier-island limitation
    std::printf("CCD_BRACKET cases=%d failures=%d ideal_fixture_old_max_pen_cm=%.9g ideal_fixture_new_max_pen_cm=%.9g scope=time_bracket_only\n",cases,failures,worstOld,worstNew);
    return failures?1:0;
}
