#pragma once
#include <atomic>
#include <memory>
#include <utility>

namespace Speed::Concurrency
{
#if defined(__cpp_lib_atomic_shared_ptr) && __cpp_lib_atomic_shared_ptr >= 201711L
// Retain the standard specialization on libraries which implement it.
template<class T>
using TAtomicSharedPtr = std::atomic<std::shared_ptr<T>>;
#else
// UE's Linux libc++ lacks the C++20 specialization. Its standard shared_ptr
// atomic operations preserve publication ordering and ownership on every access.
// Value is private: a non-atomic read/reset must never bypass these operations.
template<class T>
class TAtomicSharedPtr
{
public:
    TAtomicSharedPtr() noexcept = default;
    explicit TAtomicSharedPtr(std::shared_ptr<T> Initial) noexcept : Value(std::move(Initial)) {}
    TAtomicSharedPtr(const TAtomicSharedPtr&) = delete;
    TAtomicSharedPtr& operator=(const TAtomicSharedPtr&) = delete;

    std::shared_ptr<T> load(std::memory_order Order = std::memory_order_seq_cst) const noexcept
    { return std::atomic_load_explicit(&Value, Order); }
    void store(std::shared_ptr<T> Desired, std::memory_order Order = std::memory_order_seq_cst) noexcept
    { std::atomic_store_explicit(&Value, std::move(Desired), Order); }
    std::shared_ptr<T> exchange(std::shared_ptr<T> Desired, std::memory_order Order = std::memory_order_seq_cst) noexcept
    { return std::atomic_exchange_explicit(&Value, std::move(Desired), Order); }
private:
    std::shared_ptr<T> Value;
};
#endif
}
