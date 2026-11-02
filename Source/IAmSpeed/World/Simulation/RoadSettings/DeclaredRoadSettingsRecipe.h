#pragma once
#include "RoadSettingsCatalog.h"
#include <bit>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>
namespace Speed::RoadSettings
{
using FValue=std::variant<std::int32_t,float>;
struct FDeclaredValue {FKey Key;FValue Value;};
enum class ERefusal : std::uint8_t
{None,Profile,Count,Key,Type,Duplicate,NonFinite,Encoding,Schema,NonCanonical};
// The caller explicitly supplies every actual chosen value. No ConsoleManager,
// default/CDO getter, registration pointer or boolean authority is accepted.
// This immutable declaration still needs real original-session provisioning,
// complete native config/code identity and binding before physical admission.
class FDeclaredRecipe final
{
    std::string Profile;
    std::array<FValue,RequiredSlots> Values;
    std::vector<std::uint8_t> Identity;
    FDeclaredRecipe(std::string P,std::array<FValue,RequiredSlots> V)
        :Profile(std::move(P)),Values(std::move(V)) {Encode();}
    static bool ValidProfile(std::string_view P)
    {
        if(P.empty()||P.size()>96)return false;
        for(unsigned char C:P)if(!((C>='a'&&C<='z')||(C>='A'&&C<='Z')||(C>='0'&&C<='9')||C=='-'||C=='_'||C=='.'))return false;
        return true;
    }
    void Add(std::uint32_t N,unsigned Bytes)
    {for(unsigned I=0;I<Bytes;++I){Identity.push_back(std::uint8_t(N&255));N>>=8;}}
    void Text(std::string_view T)
    {Add(std::uint32_t(T.size()),2);Identity.insert(Identity.end(),T.begin(),T.end());}
    void Encode()
    {
        static_assert(sizeof(float)==4&&std::numeric_limits<float>::is_iec559);
        // Full canonical byte identity, not a truncated/collision-prone digest.
        // It identifies only these declared slots and this explicit profile.
        Text(Coverage);Text(Profile);Add(std::uint32_t(RequiredSlots),2);
        for(const auto& K:Catalog)
        {
            Add(K.Index,2);Add(std::uint8_t(K.Kind),1);Text(K.Name);
            if(K.Kind==EKind::Integer)Add(std::bit_cast<std::uint32_t>(std::get<std::int32_t>(Values[K.Index])),4);
            else Add(std::bit_cast<std::uint32_t>(std::get<float>(Values[K.Index])),4);
        }
    }
public:
    FDeclaredRecipe(const FDeclaredRecipe&)=delete;
    FDeclaredRecipe& operator=(const FDeclaredRecipe&)=delete;
    static std::shared_ptr<const FDeclaredRecipe> Create(std::string Profile,
        const std::vector<FDeclaredValue>& Supplied,ERefusal& Why)
    {
        Why=ERefusal::None;
        const auto Fail=[&](ERefusal R)->std::shared_ptr<const FDeclaredRecipe>{Why=R;return {};};
        if(!ValidProfile(Profile))return Fail(ERefusal::Profile);
        if(Supplied.size()!=RequiredSlots)return Fail(ERefusal::Count);
        std::array<std::optional<FValue>,RequiredSlots> Pending;
        for(const auto& S:Supplied)
        {
            if(S.Key.Index>=RequiredSlots)return Fail(ERefusal::Key);
            const auto& K=Catalog[S.Key.Index];
            if(K.Kind!=S.Key.Kind||K.Name!=S.Key.Name)return Fail(ERefusal::Key);
            if(Pending[K.Index])return Fail(ERefusal::Duplicate);
            if(K.Kind==EKind::Integer)
            {if(!std::get_if<std::int32_t>(&S.Value))return Fail(ERefusal::Type);}
            else
            {
                const auto* V=std::get_if<float>(&S.Value);
                if(!V)return Fail(ERefusal::Type);
                if(!std::isfinite(*V))return Fail(ERefusal::NonFinite);
            }
            Pending[K.Index]=S.Value;
        }
        std::array<FValue,RequiredSlots> Values;
        for(std::size_t I=0;I<RequiredSlots;++I)
        {if(!Pending[I])return Fail(ERefusal::Count);Values[I]=*Pending[I];}
        return std::shared_ptr<const FDeclaredRecipe>(new FDeclaredRecipe(std::move(Profile),std::move(Values)));
    }
    template<class T> std::optional<T> Read(FKey Key) const
    {
        static_assert(std::is_same_v<T,std::int32_t>||std::is_same_v<T,float>);
        if(Key.Index>=RequiredSlots)return {};
        const auto& K=Catalog[Key.Index];
        if(K.Name!=Key.Name||K.Kind!=Key.Kind)return {};
        const auto* V=std::get_if<T>(&Values[Key.Index]);
        return V?std::optional<T>(*V):std::optional<T>{}; // no fallback
    }
    const std::string& ProfileId() const noexcept {return Profile;}
    const std::vector<std::uint8_t>& CanonicalIdentity() const noexcept {return Identity;}
    static std::shared_ptr<const FDeclaredRecipe> Decode(const std::vector<std::uint8_t>& Bytes,ERefusal& Why)
    {
        Why=ERefusal::None;
        const auto Fail=[&](ERefusal R)->std::shared_ptr<const FDeclaredRecipe>{Why=R;return {};};
        if(Bytes.size()>65536)return Fail(ERefusal::Encoding);
        std::size_t At=0;
        const auto Number=[&](unsigned N,std::uint32_t& Out)
        {
            if(N>4||At>Bytes.size()||Bytes.size()-At<N)return false;
            Out=0;for(unsigned I=0;I<N;++I)Out|=std::uint32_t(Bytes[At++])<<(8*I);return true;
        };
        const auto Text=[&](std::string_view& Out)
        {
            std::uint32_t N=0;if(!Number(2,N)||At>Bytes.size()||Bytes.size()-At<N)return false;
            Out=std::string_view(reinterpret_cast<const char*>(Bytes.data()+At),N);At+=N;return true;
        };
        std::string_view Schema,Profile;
        if(!Text(Schema)||!Text(Profile))return Fail(ERefusal::Encoding);
        if(Schema!=Coverage)return Fail(ERefusal::Schema);
        std::uint32_t Count=0;if(!Number(2,Count))return Fail(ERefusal::Encoding);
        if(Count!=RequiredSlots)return Fail(ERefusal::Count);
        std::vector<FDeclaredValue> Values;Values.reserve(RequiredSlots);
        for(std::size_t I=0;I<RequiredSlots;++I)
        {
            std::uint32_t Index=0,Kind=0,Bits=0;std::string_view Name;
            if(!Number(2,Index)||!Number(1,Kind)||!Text(Name)||!Number(4,Bits))return Fail(ERefusal::Encoding);
            if(Index!=I||Kind!=std::uint8_t(Catalog[I].Kind)||Name!=Catalog[I].Name)return Fail(ERefusal::Key);
            FValue V=Kind==std::uint8_t(EKind::Integer)?FValue(std::bit_cast<std::int32_t>(Bits)):FValue(std::bit_cast<float>(Bits));
            Values.push_back({Catalog[I],V});
        }
        if(At!=Bytes.size())return Fail(ERefusal::NonCanonical);
        auto Recipe=Create(std::string(Profile),Values,Why);
        if(Recipe&&Recipe->CanonicalIdentity()!=Bytes)return Fail(ERefusal::NonCanonical);
        return Recipe;
    }
};
}
