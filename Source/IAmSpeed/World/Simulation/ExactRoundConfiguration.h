#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>

namespace Speed
{
// Local, typed configuration stream. Each adapter supplies its effective
// configuration and an explicit schema tag from its original native fields.
// The stream is bounded and retains neither object references nor padding.
class FExactRoundConfigurationWriter final
{
public:
    static constexpr unsigned MaxValues = 512;

    bool Unsigned(std::uint64_t Value) noexcept { return Add(1, Value, 8); }
    bool Signed(std::int64_t Value) noexcept { return Add(2, static_cast<std::uint64_t>(Value), 8); }
    bool Boolean(bool Value) noexcept { return Add(3, Value ? 1 : 0, 1); }
    bool Float(float Value) noexcept { return AddReal<std::uint32_t>(4, Value); }
    bool Double(double Value) noexcept { return AddReal<std::uint64_t>(5, Value); }
    std::uint64_t Hash() const noexcept { return Valid && Count ? ValueHash : 0; }

private:
    std::uint64_t ValueHash = 14695981039346656037ull;
    unsigned Count = 0;
    bool Valid = true;

    void Mix(std::uint8_t Byte) noexcept { ValueHash = (ValueHash ^ Byte) * 1099511628211ull; }
    bool Add(std::uint8_t Tag, std::uint64_t Value, unsigned Width) noexcept
    {
        if (!Valid || Count == MaxValues) return Valid = false;
        ++Count;
        Mix(Tag);
        for (unsigned I = 0; I < Width; ++I)
        {
            Mix(static_cast<std::uint8_t>(Value));
            Value >>= 8;
        }
        return true;
    }

    template<class TBits, class TReal>
    bool AddReal(std::uint8_t Tag, TReal Value) noexcept
    {
        if (!std::isfinite(Value)) return Valid = false;
        static_assert(sizeof(TBits) == sizeof(TReal));
        TBits Bits;
        std::memcpy(&Bits, &Value, sizeof(Bits));
        return Add(Tag, Bits, sizeof(Bits));
    }
};
}
