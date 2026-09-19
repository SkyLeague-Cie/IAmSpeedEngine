#include "SimulationWorld.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "IAmSpeed/SubBodies/Configs/SubBodyConfig.h"
#include "Misc/AutomationTest.h"
#include <stdexcept>

#if WITH_DEV_AUTOMATION_TESTS
namespace
{
// Values-only adapter fixture: no UObject, world startup, device or car physics.
class FPublicationAdapter final : public ISpeedComponent
{
public:
	mutable unsigned Validations = 0;
	unsigned Commits = 0, Aborts = 0;
	bool Valid = true, ThrowValidation = false, CommitValid = true;
	bool* GlobalVisible = nullptr;
	bool CommitSawGlobal = false;
	ECanonicalFrameAbortReason Reason = ECanonicalFrameAbortReason::PreparationFailed;
	bool ValidateCanonicalFrameCommit(uint64) const override
	{ ++Validations; if (ThrowValidation) throw std::runtime_error("validation"); return Valid; }
	bool CommitCanonicalFrame(uint64) noexcept override
	{ ++Commits; CommitSawGlobal = GlobalVisible && *GlobalVisible; return CommitValid; }
	void AbortCanonicalFrame(uint64, ECanonicalFrameAbortReason InReason) noexcept override
	{ ++Aborts; Reason = InReason; }
	void PrepareCanonicalFrame(const FCanonicalFrameContext&) override {}
	unsigned int NumFrame() const override { return 1; }
	float GetPhysMass() const override { return 1; }
	const FVector& GetPhysCOM() const override { return Zero; }
	const FVector& GetPhysCenterOfMassLocal() const override { return Zero; }
	const TArray<USSubBody*>& GetSubBodies() const override { return SubBodies; }
	TArray<USSubBody*> CreateSubBodies() override { return {}; }
	bool IsFrozen() const override { return false; }
	SubBodyConfig GetSubBodyConfig(const USSubBody&) const override { return {}; }
	SKinematic GetKinematicsOfSubBody(const USSubBody&, const unsigned int&) const override { return State; }
	FMatrix ComputeWorldInvInertiaTensor() const override { return FMatrix::Identity; }
	FMatrix ComputeWorldInvInertiaTensorOfSubBody(const USSubBody&) const override { return FMatrix::Identity; }
	const SKinematic& GetKinematicState() const override { return State; }
	const SKinematic& GetKinematicStateForFrame(const unsigned int&) const override { return State; }
	float GetPhysMaxSpeed() const override { return 0; }
	float GetPhysMaxAngularSpeed() const override { return 0; }
	void SetIsUpsideDown(bool) override {}
	bool IsUpsideDown() const override { return false; }
	bool IsInAutoRecover() const override { return false; }
	bool IsSubBodyInAutoRecoverMode() const override { return false; }
	void RcvImpactOnSubBody(const USSubBody&, const FVector&) override {}
protected:
	void PostPhysicsUpdatePrv(const float&) override {}
	void SetKinematicState(const SKinematic&) override {}
	void SetIsFrozen(bool) override {}
private:
	FVector Zero = FVector::ZeroVector;
	SKinematic State;
	TArray<USSubBody*> SubBodies;
};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCanonicalPublicationTransactionTest,
	"IAmSpeed.Simulation.CanonicalPublicationTransaction",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FCanonicalPublicationTransactionTest::RunTest(const FString&)
{
	for (unsigned Mode = 0; Mode < 7; ++Mode)
	{
		Speed::FSimulationWorld World;
		FPublicationAdapter First, Last;
		bool GlobalVisible = false;
		First.GlobalVisible = Last.GlobalVisible = &GlobalVisible;
		First.Valid = Mode != 1; Last.Valid = Mode != 2;
		Last.ThrowValidation = Mode == 3; Last.CommitValid = Mode != 6;
		TestTrue(TEXT("register both real world participants"), World.AddAdapter(First) && World.AddAdapter(Last));
		World.RebuildOrderedAdapters();
		Speed::SimulationBoundary::FSnapshotBuffer Buffer(128);
		FSimulationSnapshot Previous; Previous.NumFrame = 6;
		TestTrue(TEXT("previous authoritative snapshot"), Buffer.Publish(Previous));
		const uint64 BeforeSerial = Buffer.PublishedSerial();
		FSimulationSnapshot Next; Next.NumFrame = 7;
		if (Mode == 4) Next.Payload.SetNum(129); // Real buffer validation failure.
		unsigned PublisherCalls = 0;
		const auto Result = World.PublishCanonicalFrame(7, [&]()
		{
			++PublisherCalls;
			TestEqual(TEXT("all participants validated before global publication"), First.Validations + Last.Validations, 2u);
			TestEqual(TEXT("no input commits before global publication"), First.Commits + Last.Commits, 0u);
			if (Mode == 5) throw std::runtime_error("publisher before visibility");
			GlobalVisible = Buffer.Publish(Next);
			return GlobalVisible;
		});
		if (Mode >= 1 && Mode <= 3)
		{
			TestTrue(TEXT("validation false/throw rejected"), Result == ECanonicalPublicationResult::ValidationFailed);
			TestEqual(TEXT("failed participant prevents publisher call"), PublisherCalls, 0u);
		}
		else if (Mode == 4 || Mode == 5)
			TestTrue(TEXT("publisher false/throw rejected"), Result == ECanonicalPublicationResult::PublicationFailed);
		else
			TestTrue(TEXT("success or explicit commit invariant failure"), Result == (Mode == 6
				? ECanonicalPublicationResult::CommitInvariantFailed : ECanonicalPublicationResult::Completed));
		const bool Visible = Mode == 0 || Mode == 6;
		TestEqual(TEXT("global visibility and serial are atomic on failure"), Buffer.PublishedSerial(), BeforeSerial + (Visible ? uint64(1) : uint64(0)));
		TestEqual(TEXT("previous snapshot retained on failure"), Buffer.PublishedFrame(), Visible ? uint64(7) : uint64(6));
		if (!Visible)
		{
			TestEqual(TEXT("abort ALL including participants already validated"), First.Aborts + Last.Aborts, 2u);
			TestEqual(TEXT("no partial input publication"), First.Commits + Last.Commits, 0u);
			TestTrue(TEXT("snapshot failure abort reason"), First.Reason == ECanonicalFrameAbortReason::SnapshotPublicationFailed
				&& Last.Reason == ECanonicalFrameAbortReason::SnapshotPublicationFailed);
		}
		else
		{
			TestEqual(TEXT("no abort after authoritative publication"), First.Aborts + Last.Aborts, 0u);
			TestTrue(TEXT("input commit observes published global frame"), First.CommitSawGlobal && Last.CommitSawGlobal);
			TestEqual(TEXT("one finalization per participant"), First.Commits + Last.Commits, 2u);
		}
	}
	return true;
}
#endif
