#pragma once
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>
namespace Speed::PreparedAccess {
struct FSpan { std::uintptr_t Begin = 0; std::size_t Bytes = 0; };
struct FParticipantAccess
{
    std::uint64_t Participant = 0;
    std::vector<FSpan> Writes, ReadOnly;
};
inline bool ValidSpan(FSpan S) noexcept
{ return S.Begin && S.Bytes && S.Bytes - 1 <= std::numeric_limits<std::uintptr_t>::max() - S.Begin; }
inline bool Overlap(FSpan A, FSpan B) noexcept
{ return A.Begin <= B.Begin ? B.Begin - A.Begin < A.Bytes : A.Begin - B.Begin < B.Bytes; }
inline bool ValidateGlobalAccess(const std::vector<FParticipantAccess>& Plans) noexcept
{
    for (std::size_t I = 0; I < Plans.size(); ++I)
    {
        const auto& P = Plans[I]; if (!P.Participant || P.Writes.empty()) return false;
        for (std::size_t J = 0; J < I; ++J) if (P.Participant == Plans[J].Participant) return false;
        for (const auto S : P.Writes) if (!ValidSpan(S)) return false;
        for (const auto S : P.ReadOnly) if (!ValidSpan(S)) return false;
    }
    for (std::size_t I = 0; I < Plans.size(); ++I)
        for (std::size_t A = 0; A < Plans[I].Writes.size(); ++A)
        {
            const auto Write = Plans[I].Writes[A];
            for (std::size_t J = 0; J < Plans.size(); ++J)
            {
                for (const auto Read : Plans[J].ReadOnly) if (Overlap(Write, Read)) return false;
                for (std::size_t B = 0; B < Plans[J].Writes.size(); ++B)
                    if ((J > I || (J == I && B > A)) && Overlap(Write, Plans[J].Writes[B])) return false;
            }
        }
    return !Plans.empty(); // read-only/read-only sharing is allowed
}
// Adapter copies declarations only; actual owner plans MUST retain all pins.
// Compatible with common FSpan and wheel/subbody Begin/Bytes exports.
template<class Writes, class ReadGroups> bool AppendPlanAccess(std::vector<FParticipantAccess>& Out,
    std::uint64_t Identity, const Writes& WriteRanges, const ReadGroups& ReadOnlyGroups)
{
    if (!Identity) return false;
    FParticipantAccess P; P.Participant = Identity;
    for (const auto& S : WriteRanges) { FSpan V{S.Begin, S.Bytes}; if (!ValidSpan(V)) return false; P.Writes.push_back(V); }
    for (const auto& G : ReadOnlyGroups) for (const auto& S : G)
    { FSpan V{S.Begin, S.Bytes}; if (!ValidSpan(V)) return false; P.ReadOnly.push_back(V); }
    if (P.Writes.empty()) return false; Out.push_back(std::move(P)); return true;
}
inline bool SameDeclarations(const std::vector<FParticipantAccess>& A, const std::vector<FParticipantAccess>& B) noexcept
{
    if (A.size() != B.size()) return false;
    for (std::size_t I = 0; I < A.size(); ++I)
    {
        if (A[I].Participant != B[I].Participant || A[I].Writes.size() != B[I].Writes.size() || A[I].ReadOnly.size() != B[I].ReadOnly.size()) return false;
        for (std::size_t J = 0; J < A[I].Writes.size(); ++J)
            if (A[I].Writes[J].Begin != B[I].Writes[J].Begin || A[I].Writes[J].Bytes != B[I].Writes[J].Bytes) return false;
        for (std::size_t J = 0; J < A[I].ReadOnly.size(); ++J)
            if (A[I].ReadOnly[J].Begin != B[I].ReadOnly[J].Begin || A[I].ReadOnly[J].Bytes != B[I].ReadOnly[J].Bytes) return false;
    }
    return true;
}
}
