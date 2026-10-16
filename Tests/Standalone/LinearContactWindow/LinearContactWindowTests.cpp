#include "IAmSpeed/World/Collision/LinearContactWindow.h"
#include <cstdio>
#include <cmath>
#include <limits>
int main(){
 int cases=0,failures=0;
 auto Check=[&](bool Ok){++cases;if(!Ok)++failures;};
 constexpr double Step=1./(300.*12.);
 for(double Speed:{300.,1200.,4600.}) for(int Phase=0;Phase<12;++Phase) for(double Sign:{-1.,1.}) {
  const double Enter=(Phase+.35)*Step, Exit=(Phase+.4)*Step;
  const double X=156-Speed*Exit,Y=104+Speed*Enter;
  Speed::Collision::FLinearContactWindow W(1./300.);
  Check(W.Clip(Sign*X,Sign*Speed,156));Check(W.Clip(Sign*Y,-Sign*Speed,104));Check(W.Clip(0,0,36));
  Check(std::abs(W.Enter-Enter)<1.e-14&&std::abs(W.Exit-Exit)<1.e-14);
  const double Probe=(W.Enter+W.Exit)*.5;
  auto Inside=[&](double T){return std::abs(X+Speed*T)<=156&&std::abs(Y-Speed*T)<=104;};
  Check(Inside(Probe)); bool OldFound=false;for(int I=0;I<=12;++I)OldFound|=Inside(I*Step);Check(!OldFound);
 }
 Speed::Collision::FLinearContactWindow Parallel(1);Check(!Parallel.Clip(3,0,2));Check(!Parallel.Clip(0,0,2));
 Speed::Collision::FLinearContactWindow Receding(1);Check(!Receding.Clip(3,1,2));
 Speed::Collision::FLinearContactWindow Future(1);Check(!Future.Clip(5,-1,2));
 Speed::Collision::FLinearContactWindow Invalid(1);Check(!Invalid.Clip(0,0,-1));
 Speed::Collision::FLinearContactWindow Nan(1);Check(!Nan.Clip(std::numeric_limits<double>::quiet_NaN(),1,2));
 Speed::Collision::FLinearContactWindow BadDuration(-1);Check(!BadDuration.Clip(0,0,2));
 std::printf("LINEAR_CONTACT_WINDOW assertions=%d failures=%d scope=production_scalar_interval_only native_geometry_unrun\n",cases,failures);return failures?1:0;
}
