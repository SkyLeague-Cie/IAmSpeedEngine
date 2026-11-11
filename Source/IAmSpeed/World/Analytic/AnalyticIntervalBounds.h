#pragma once
// Elementary enclosure kernel; carries no collision authority.
// Entry points/worker lanes MUST call
// RequireCurrentAnalyticNumericContext(Binary64NearestAllowFlush), even if a
// caller world is Unspecified. This header neither establishes nor changes it.
// Scope: current MSVC Win64 SSE2 /fp:precise; no x87/fast-math transfer.
// Encloses elementary binary64 operations under the already guarded RNE/masked
// context, with either FTZ/DAZ mode. Endpoints are normal or zero, so a flushed
// subnormal endpoint cannot silently change a later bound. Not a contact solver.
#include <cstdint>
#include <cstring>
#include <limits>

namespace AnalyticCertified
{
static_assert(sizeof(double) == sizeof(std::uint64_t) &&
    std::numeric_limits<double>::is_iec559, "Requires IEEE binary64");
inline std::uint64_t Bits(double x) { std::uint64_t b; std::memcpy(&b, &x, 8); return b; }
inline double FromBits(std::uint64_t b) { double x; std::memcpy(&x, &b, 8); return x; }
constexpr std::uint64_t Sign = 0x8000000000000000ULL;
constexpr std::uint64_t Magnitude = 0x7fffffffffffffffULL;
constexpr std::uint64_t MinNormal = 0x0010000000000000ULL;
constexpr std::uint64_t Infinity = 0x7ff0000000000000ULL;
inline bool Finite(double x) { return (Bits(x) & Magnitude) < Infinity; }
struct Interval
{
    double Lower = 0.0, Upper = 0.0;
    bool Known = false;
    static Interval Exact(double x)
    {
        const auto b = Bits(x), a = b & Magnitude;
        if (a >= Infinity) return {};
        if (a == 0) return {0.0, 0.0, true};
        if (a < MinNormal)
            return (b & Sign) ? Interval{FromBits(Sign | MinNormal), 0.0, true}
                              : Interval{0.0, FromBits(MinNormal), true};
        return {x, x, true};
    }
};
// Public aggregate construction is accepted only after bitwise validation.
// In particular DAZ can erase a forged subnormal endpoint before multiplication.
inline bool Valid(Interval a)
{
    if (!a.Known) return false;
    const auto lo = Bits(a.Lower) & Magnitude, hi = Bits(a.Upper) & Magnitude;
    if (lo >= Infinity || hi >= Infinity ||
        (lo != 0 && lo < MinNormal) || (hi != 0 && hi < MinNormal)) return false;
    return a.Lower <= a.Upper;
}
// One representable step encloses normal RNE error. A zero or subnormal result
// receives a whole smallest-normal allowance, including a result flushed to zero.
inline bool Outward(double x, bool upward, double& out)
{
    auto b = Bits(x); const auto a = b & Magnitude;
    if (a >= Infinity) return false;
    if (a == 0) { out = FromBits(upward ? MinNormal : (Sign | MinNormal)); return true; }
    if (a < MinNormal)
    {
        out = (b & Sign) ? (upward ? 0.0 : FromBits(Sign | MinNormal))
                         : (upward ? FromBits(MinNormal) : 0.0);
        return true;
    }
    if (((b & Sign) != 0) == upward) --b; else ++b;
    if ((b & Magnitude) >= Infinity) return false;
    if ((b & Magnitude) < MinNormal)
        b = (b & Sign) ? (upward ? 0ULL : Sign | MinNormal)
                       : (upward ? MinNormal : 0ULL);
    out = FromBits(b); return true;
}
inline Interval Add(Interval a, Interval b)
{
    if (!Valid(a) || !Valid(b)) return {};
    volatile double lo = a.Lower + b.Lower;
    volatile double hi = a.Upper + b.Upper;
    Interval r;
    r.Known = Outward(lo, false, r.Lower) && Outward(hi, true, r.Upper);
    return r;
}
inline Interval Negate(Interval a)
{
    if (!Valid(a)) return {};
    return {FromBits(Bits(a.Upper) ^ Sign), FromBits(Bits(a.Lower) ^ Sign), true};
}
inline Interval Subtract(Interval a, Interval b) { return Add(a, Negate(b)); }
inline Interval Multiply(Interval a, Interval b)
{
    if (!Valid(a) || !Valid(b)) return {};
    // Singleton operands repeat identical native products. Under this
    // header's guarded RNE/masked context, one occurrence has exactly the
    // same result and sticky-flag union as its duplicates. Keep the distinct
    // products in their ORIGINAL order, including immediate refusal.
    const bool singletonA=Bits(a.Lower)==Bits(a.Upper);
    const bool singletonB=Bits(b.Lower)==Bits(b.Upper);
    if(singletonA||singletonB)
    {
        volatile double first=a.Lower*b.Lower;const auto firstBits=Bits(first);
        if((firstBits&Magnitude)>=0x7fefffffffffffffULL)return {};
        auto lowerBits=firstBits,upperBits=firstBits;
        if(!(singletonA&&singletonB))
        {
            // A singleton: [aL*bL,aL*bU] repeated. B singleton:
            // [aL*bL,aL*bL,aU*bL,aU*bL]. Nothing is reordered.
            volatile double second=singletonA?a.Lower*b.Upper:a.Upper*b.Lower;
            const auto secondBits=Bits(second);
            if((secondBits&Magnitude)>=0x7fefffffffffffffULL)return {};
            const auto firstKey=(firstBits&Sign)?~firstBits:(firstBits|Sign);
            const auto secondKey=(secondBits&Sign)?~secondBits:(secondBits|Sign);
            if(secondKey<firstKey)lowerBits=secondBits;
            if(secondKey>firstKey)upperBits=secondBits;
        }
        Interval r;
        r.Known=Outward(FromBits(lowerBits),false,r.Lower)&&Outward(FromBits(upperBits),true,r.Upper);
        return r;
    }
    const double av[2] = {a.Lower, a.Upper}, bv[2] = {b.Lower, b.Upper};
    std::uint64_t lowerBits=0,upperBits=0,lowerKey=0,upperKey=0;bool first=true;
    for (double x : av) for (double y : bv)
    {
        // Retain ALL four native products, their order and the original
        // immediate failure point (Outward also refuses +/-MaxFinite).
        // Outward is monotone: only the extrema require its bit expansion.
        volatile double z = x * y;const auto bits=Bits(z);
        if ((bits & Magnitude) >= 0x7fefffffffffffffULL) return {};
        // Native comparisons could erase a subnormal product under DAZ.
        // Integer ordering also keeps the two signed-zero representatives;
        // both expand to exactly the same +/-MinNormal bounds as before.
        const auto key=(bits & Sign)?~bits:(bits | Sign);
        if(first || key<lowerKey){lowerKey=key;lowerBits=bits;}
        if(first || key>upperKey){upperKey=key;upperBits=bits;}
        first=false;
    }
    Interval r;
    r.Known=Outward(FromBits(lowerBits),false,r.Lower) && Outward(FromBits(upperBits),true,r.Upper);
    return r;
}
// Exact specialization of Multiply(a,Exact(.5)) under this header's guarded
// RNE/SSE2/masked contract, including FTZ/DAZ. The generic products are
// [Lower*.5,Lower*.5,Upper*.5,Upper*.5]. Positive halving is monotone; signed
// zeros have identical Outward images. No finite accepted operand can overflow
// here. Removing repeated products preserves the union of masked sticky flags.
inline Interval ScaleHalf(Interval a)
{
    if (!Valid(a)) return {};
    volatile double lo=a.Lower*0.5;
    volatile double hi=a.Upper*0.5;
    Interval r;r.Known=Outward(lo,false,r.Lower)&&Outward(hi,true,r.Upper);
    return r;
}
inline Interval HalfSum(Interval a, Interval b)
{
    // Keep both operand calls; an invalid operand must not suppress the other
    // operand's flags. Outward stages and final Add stay exactly as before.
    return Add(ScaleHalf(a), ScaleHalf(b));
}
inline Interval Square(Interval a)
{
    if (!Valid(a)) return {};
    // Valid endpoints are normal or zero. Each former singleton Multiply
    // performed four identical products; one product has the same masked
    // sticky-flag union and exactly the same Outward enclosure/refusal.
    const auto endpointSquare=[](double endpoint)
    {
        // Match Exact's signed-zero normalization before the native product.
        const double x=(Bits(endpoint)&Magnitude)==0?0.0:endpoint;
        volatile double squared=x*x;
        if((Bits(squared)&Magnitude)>=0x7fefffffffffffffULL)return Interval{};
        Interval r;r.Known=Outward(squared,false,r.Lower)&&Outward(squared,true,r.Upper);
        return r;
    };
    // Keep both calls even if the left endpoint refuses: the right endpoint
    // may still contribute underflow, overflow or inexact sticky flags.
    auto left=endpointSquare(a.Lower);
    auto right=endpointSquare(a.Upper);
    if (!left.Known || !right.Known) return {};
    const bool crossesZero = a.Lower <= 0.0 && a.Upper >= 0.0;
    double lo = crossesZero ? 0.0 : (left.Lower < right.Lower ? left.Lower : right.Lower);
    if (lo < 0.0) lo = 0.0;
    return {lo, left.Upper > right.Upper ? left.Upper : right.Upper, true};
}
}
