#include "TorusGeometry.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include <limits>

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTorusLocalContactsTest,
    "IAmSpeed.Collision.Torus.LocalContacts",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTorusLocalContactsTest::RunTest(const FString& Parameters)
{
    using namespace Speed::Geometry;
    const FTorusGeometry Torus{100., 10.};
    FTorusContact Contact;
    const auto Check = [this,&Torus,&Contact](const TCHAR* Label, const FVector3d& P,
        double Radius, double Expected, const FVector3d& Normal)
    {
        TestTrue(Label, TryTorusSphereContact(Torus, P, Radius, Contact));
        TestTrue(TEXT("Signed separation"), FMath::Abs(Contact.Separation-Expected)<1.e-9);
        TestTrue(TEXT("Outward unit normal"), Contact.Normal.Equals(Normal,1.e-9));
        // Independently verify the returned witness lies on the implicit torus.
        const double Radial=FMath::Sqrt(FMath::Square(Contact.SurfacePoint.Y)+FMath::Square(Contact.SurfacePoint.Z));
        TestTrue(TEXT("Surface witness satisfies torus equation"),
            FMath::Abs(FMath::Square(Contact.SurfacePoint.X)+FMath::Square(Radial-100.)-100.)<1.e-7);
    };
    Check(TEXT("Empty central hole"), FVector3d::ZeroVector, 20., 70., FVector3d(0.,-1.,0.));
    Check(TEXT("Outer contact"), FVector3d(0.,115.,0.), 5., 0., FVector3d(0.,1.,0.));
    Check(TEXT("Inner contact"), FVector3d(0.,85.,0.), 5., 0., FVector3d(0.,-1.,0.));
    Check(TEXT("Axial contact"), FVector3d(15.,100.,0.), 5., 0., FVector3d(1.,0.,0.));
    Check(TEXT("Tube penetration"), FVector3d(0.,100.,0.), 2., -12., FVector3d(1.,0.,0.));
    Check(TEXT("Other radial direction"), FVector3d(0.,0.,115.), 5., 0., FVector3d(0.,0.,1.));
    TestFalse(TEXT("Horn torus rejected"), TryTorusPointContact({10.,10.},FVector3d::ZeroVector,Contact));
    TestTrue(TEXT("Invalid input clears prior witness"), Contact.Normal.IsZero() && Contact.Separation==0.);
    TestFalse(TEXT("Negative sphere radius"), TryTorusSphereContact(Torus,FVector3d::ZeroVector,-1.,Contact));
    TestFalse(TEXT("Nonfinite point"), TryTorusPointContact(Torus,FVector3d(std::numeric_limits<double>::infinity(),0.,0.),Contact));
    return true;
}
#endif
