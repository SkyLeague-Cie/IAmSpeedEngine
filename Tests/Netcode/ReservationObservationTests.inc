#include <type_traits>
using FReservationObservation=V::FInputSessionRegistry::FCompletedBoundaryReservationObservation;
static_assert(!std::is_default_constructible_v<FReservationObservation>);

static void ReservationObservationExactIdentityAndLifetime()
{
    FEmptyFixture F;F.Complete(10);
    const auto C=F.Registry.CaptureCompletedBoundary(10);CHECK(C);
    auto T=F.Registry.ReserveCompletedBoundary(C);CHECK(T);
    Allocations=0;Budget=0;
    const auto O=F.Registry.ObserveCompletedBoundaryReservation(*T);
    const bool Valid=O && F.Registry.ValidateCompletedBoundaryReservationObservation(*T,*O);
    Budget=-1;CHECK(Valid && Allocations==0 && O->Capture()==C);
    CHECK(O->Capture()->Completed()==F.Registry.ReadHistorical(10));
    V::FInputSessionRegistry::FCompletedBoundaryReservation Default;
    CHECK(!F.Registry.ObserveCompletedBoundaryReservation(Default));
    CHECK(!F.Registry.ValidateCompletedBoundaryReservationObservation(Default,*O));
    auto Foreign=std::async(std::launch::async,[&] {
        return !F.Registry.ObserveCompletedBoundaryReservation(*T)
            && !F.Registry.ValidateCompletedBoundaryReservationObservation(*T,*O);
    });
    CHECK(Foreign.get());
    {
        I::FPresentationInputScope Scope;
        CHECK(!F.Registry.ObserveCompletedBoundaryReservation(*T));
        CHECK(!F.Registry.ValidateCompletedBoundaryReservationObservation(*T,*O));
    }
    FEmptyFixture Other;Other.Complete(10);
    auto OtherT=Other.Registry.ReserveCompletedBoundary(Other.Registry.CaptureCompletedBoundary(10));CHECK(OtherT);
    const auto OtherO=Other.Registry.ObserveCompletedBoundaryReservation(*OtherT);CHECK(OtherO);
    CHECK(!F.Registry.ObserveCompletedBoundaryReservation(*OtherT));
    CHECK(!F.Registry.ValidateCompletedBoundaryReservationObservation(*T,*OtherO));
    CHECK(!Other.Registry.ValidateCompletedBoundaryReservationObservation(*OtherT,*O));
    Other.Registry.ReleaseCompletedBoundaryReservation(*OtherT);
    auto Moved=std::move(*T);
    CHECK(!F.Registry.ObserveCompletedBoundaryReservation(*T));
    CHECK(F.Registry.ValidateCompletedBoundaryReservationObservation(Moved,*O));
    CHECK(F.Registry.AbortCompletedBoundaryReservation(Moved));
    CHECK(!F.Registry.ValidateCompletedBoundaryReservationObservation(Moved,*O));
    auto Renewed=F.Registry.ReserveCompletedBoundary(C);CHECK(Renewed);
    CHECK(!F.Registry.ValidateCompletedBoundaryReservationObservation(*Renewed,*O));
    const auto NewO=F.Registry.ObserveCompletedBoundaryReservation(*Renewed);CHECK(NewO);
    Allocations=0;Budget=0;
    const bool NewValid=F.Registry.ValidateCompletedBoundaryReservationObservation(*Renewed,*NewO);
    Budget=-1;CHECK(NewValid && Allocations==0);
    F.Registry.ReleaseCompletedBoundaryReservation(*Renewed);
    CHECK(!F.Registry.ValidateCompletedBoundaryReservationObservation(*Renewed,*NewO));
    CHECK(O->Capture()==C && O->Capture()->Completed()->Frame==10);
}
static void ReservationObservationRetainsDataWithoutFence()
{
    const auto Retained=[] {
        FFixture F(1);
        const auto C=F.Registry.CaptureCompletedBoundary(0);CHECK(C);
        auto T=F.Registry.ReserveCompletedBoundary(C);CHECK(T);
        const auto O=F.Registry.ObserveCompletedBoundaryReservation(*T);CHECK(O);
        CHECK(O->Capture()->Completed()->Inputs.size()==1);
        CHECK(O->Capture()->CurrentBoundary()->Phases[0].Phase==V::ESessionPhase::Paused);
        F.Registry.ReleaseCompletedBoundaryReservation(*T);
        CHECK(!F.Registry.ValidateCompletedBoundaryReservationObservation(*T,*O));
        return *O;
    }();
    // Immutable objects remain readable. No call to the destroyed registry and
    // no live world/branch/restore/publication authority can be derived here.
    CHECK(Retained.Capture()->Completed()->Frame==0);
    CHECK(Retained.Capture()->Completed()->Inputs[0].Snapshot);
    CHECK(Retained.Capture()->CurrentBoundary()->Bindings.size()==1);
}
static void TestReservationObservation()
{
    ReservationObservationExactIdentityAndLifetime();
    ReservationObservationRetainsDataWithoutFence();
}
