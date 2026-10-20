#include "IAmSpeed/Concurrency/AtomicSharedPtr.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include <thread>
#include <type_traits>

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedAtomicSharedPtrPublicationTest,
    "IAmSpeed.Concurrency.AtomicSharedPtr.Publication",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedAtomicSharedPtrPublicationTest::RunTest(const FString&)
{
    struct FSnapshot { uint64 Sequence; uint64 Complement; };
    using FSlot = Speed::Concurrency::TAtomicSharedPtr<const FSnapshot>;
    static_assert(!std::is_copy_constructible_v<FSlot>);
    FSlot Published;
    TestFalse(TEXT("Default publication is empty"), bool(Published.load()));
    auto Initial = std::make_shared<const FSnapshot>(FSnapshot{7, ~uint64(7)});
    std::weak_ptr<const FSnapshot> Weak = Initial;
    Published.store(Initial, std::memory_order_seq_cst);
    Initial.reset();
    auto Held = Published.load(std::memory_order_seq_cst);
    auto Retired = Published.exchange({}, std::memory_order_seq_cst);
    TestTrue(TEXT("Exchange returns the published owner"), Held && Held == Retired);
    TestFalse(TEXT("Retirement clears publication"), bool(Published.load()));
    Retired.reset();
    TestFalse(TEXT("Reader keeps retired publication alive"), Weak.expired());
    Held.reset();
    TestTrue(TEXT("Last reader releases the retired object"), Weak.expired());

    std::atomic<bool> Done{false}, Bad{false};
    std::atomic<uint64> Reads{0};
    std::thread Reader([&]
    {
        do
        {
            const auto Value = Published.load(std::memory_order_acquire);
            if (Value)
            {
                ++Reads;
                if (Value->Complement != ~Value->Sequence) Bad.store(true);
            }
        } while (!Done.load(std::memory_order_acquire));
    });
    for (uint64 N = 1; N <= 20000; ++N)
        Published.store(std::make_shared<const FSnapshot>(FSnapshot{N, ~N}), std::memory_order_release);
    Done.store(true, std::memory_order_release);
    Reader.join();
    const auto Last = Published.load(std::memory_order_seq_cst);
    TestFalse(TEXT("Acquire readers observe complete immutable payloads"), Bad.load());
    TestTrue(TEXT("Reader consumed a publication"), Reads.load() > 0);
    TestTrue(TEXT("Final publication is the complete final snapshot"),
        Last && Last->Sequence == 20000 && Last->Complement == ~Last->Sequence);
    return true;
}
#endif
