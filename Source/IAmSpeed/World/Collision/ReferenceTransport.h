#pragma once
#include "PreparedIntervalScene.h"
#include "ReferenceSphereResponseContract.h"
#include "ReferenceAffineExtrusionDescription.h"
#include "Hash/Blake3.h"
namespace Speed::Analytic::Prepared
{
inline bool ChargeReferenceBytes(C::FDistanceBudget& budget,int64 bytes)
{
    if(bytes<0)return false;
    const int64 blocks=(bytes+63)/64;
    if(blocks>budget.Remaining||uint64(blocks)>MAX_uint64-budget.Used)return false;
    for(int64 i=0;i<blocks;++i)if(!budget.Consume())return false;
    return true;
}
inline bool ChargeReferenceText(C::FDistanceBudget& budget,const FString& text)
{return ChargeReferenceBytes(budget,int64(text.Len())*sizeof(TCHAR));}
struct FReferenceFace
{
    FString OpaqueId,CanonicalRecordIdentity,BoundRecordIdentity;
    bool FromPlane=false;
    TArray<FString> DomainAndChart;
    bool operator==(const FReferenceFace& other) const
    {return OpaqueId==other.OpaqueId&&CanonicalRecordIdentity==other.CanonicalRecordIdentity&&
        BoundRecordIdentity==other.BoundRecordIdentity&&FromPlane==other.FromPlane&&DomainAndChart==other.DomainAndChart;}
};
struct FReferenceTransportData
{
    C::FIndexedIntervalUnionPtr CanonicalUnion;
    FScenePtr NominalScene; // Provenance only; never select a model by SourceHash.
    TArray<FReferenceFace> Faces;
    TArray<int32> ProviderFaceIndices;
    TArray<FReferenceSphereResponseContract> SphereResponseContracts;
    TArray<FString> ProofIdentities;
    TArray<FReferenceAffineExtrusionDescription> ExactAffineExtrusions;
    FString ExactAffineProofIdentity;
    TArray<uint8> Packet;
    FString Model,ExternalPacketIdentity;
};
class FReferenceTransport;
using FReferenceTransportPtr=TSharedPtr<const FReferenceTransport,ESPMode::ThreadSafe>;
class FReferenceTransportProducer;
// Generic immutable mathematical transport. Structure, ownership and a digest
// of the actual retained bytes ONLY. No certificate/asset authentication,
// C2 assertion, physical authority or readiness follows from its existence.
class FReferenceTransport final
{
    FReferenceTransportData Data;
    FBlake3Hash PacketByteIdentity;
    TArray<FIntPoint> ExactAffineProviderRanges;
    explicit FReferenceTransport(FReferenceTransportData&& data):Data(MoveTemp(data)),
        PacketByteIdentity(FBlake3::HashBuffer(Data.Packet.GetData(),uint64(Data.Packet.Num())))
    {
        if(!Data.ExactAffineExtrusions.IsEmpty())
        {
            ExactAffineProviderRanges.Init(FIntPoint(INDEX_NONE,0),Data.CanonicalUnion->Num());
            for(int32 i=0;i<Data.ExactAffineExtrusions.Num();++i)
            {
                auto& range=ExactAffineProviderRanges[Data.ExactAffineExtrusions[i].Provider];
                if(range.X==INDEX_NONE)range.X=i;
                ++range.Y;
            }
        }
    }
    friend class FReferenceTransportProducer;
public:
    const C::FIndexedIntervalUnionPtr& GetCanonicalUnion() const {return Data.CanonicalUnion;}
    const FScenePtr& GetNominalScene() const {return Data.NominalScene;}
    TConstArrayView<FReferenceFace> GetFaces() const {return Data.Faces;}
    TConstArrayView<int32> GetProviderFaceIndices() const {return Data.ProviderFaceIndices;}
    TConstArrayView<FReferenceSphereResponseContract> GetSphereResponseContracts() const {return Data.SphereResponseContracts;}
    TConstArrayView<FString> GetProofIdentities() const {return Data.ProofIdentities;}
    TConstArrayView<uint8> GetPacket() const {return Data.Packet;}
    const FString& GetExactAffineProofIdentity() const {return Data.ExactAffineProofIdentity;}
    TConstArrayView<FReferenceAffineExtrusionDescription> GetExactAffineExtrusions(int32 provider) const
    {
        if(!ExactAffineProviderRanges.IsValidIndex(provider))return {};
        const auto range=ExactAffineProviderRanges[provider];
        if(range.X==INDEX_NONE||range.Y<=0)return {};
        return MakeArrayView(Data.ExactAffineExtrusions).Slice(range.X,range.Y);
    }
    const FString& GetModel() const {return Data.Model;}
    const FString& GetExternalPacketIdentity() const {return Data.ExternalPacketIdentity;}
    const FBlake3Hash& GetPacketByteIdentity() const {return PacketByteIdentity;}
    // Description equality, not a proof of packet->coefficients correspondence.
    // The closed game's producer must establish that relation from actual input.
    bool SameDescription(const FReferenceTransport& other,C::FDistanceBudget& budget) const
    {
        if(!budget.Consume())return false;
        if(PacketByteIdentity!=other.PacketByteIdentity||Data.Packet.Num()!=other.Data.Packet.Num()||
            Data.Faces.Num()!=other.Data.Faces.Num()||Data.ProviderFaceIndices.Num()!=other.Data.ProviderFaceIndices.Num()||
            Data.ProofIdentities.Num()!=other.Data.ProofIdentities.Num()||Data.SphereResponseContracts.Num()!=other.Data.SphereResponseContracts.Num()||
            Data.ExactAffineExtrusions.Num()!=other.Data.ExactAffineExtrusions.Num())return false;
        if(!ChargeReferenceBytes(budget,Data.Packet.Num())||!ChargeReferenceText(budget,Data.Model)||
            !ChargeReferenceText(budget,Data.ExternalPacketIdentity))return false;
        if(Data.Packet!=other.Data.Packet||Data.Model!=other.Data.Model||Data.ExternalPacketIdentity!=other.Data.ExternalPacketIdentity)return false;
        if(!ChargeReferenceText(budget,Data.ExactAffineProofIdentity)||
            Data.ExactAffineProofIdentity!=other.Data.ExactAffineProofIdentity)return false;
        for(int32 i=0;i<Data.ExactAffineExtrusions.Num();++i)
            if(!Data.ExactAffineExtrusions[i].SameDescription(other.Data.ExactAffineExtrusions[i],budget))return false;
        for(int32 i=0;i<Data.ProviderFaceIndices.Num();++i)
            if(!budget.Consume()||Data.ProviderFaceIndices[i]!=other.Data.ProviderFaceIndices[i]||
                !(Data.SphereResponseContracts[i]==other.Data.SphereResponseContracts[i]))return false;
        for(int32 i=0;i<Data.ProofIdentities.Num();++i)
            if(!budget.Consume()||!ChargeReferenceText(budget,Data.ProofIdentities[i])||
                Data.ProofIdentities[i]!=other.Data.ProofIdentities[i])return false;
        for(int32 i=0;i<Data.Faces.Num();++i)
        {
            const auto& a=Data.Faces[i];const auto& b=other.Data.Faces[i];
            if(!budget.Consume()||a.FromPlane!=b.FromPlane||a.DomainAndChart.Num()!=b.DomainAndChart.Num()||
                !ChargeReferenceText(budget,a.OpaqueId)||!ChargeReferenceText(budget,a.CanonicalRecordIdentity)||
                !ChargeReferenceText(budget,a.BoundRecordIdentity))return false;
            if(a.OpaqueId!=b.OpaqueId||a.CanonicalRecordIdentity!=b.CanonicalRecordIdentity||a.BoundRecordIdentity!=b.BoundRecordIdentity)return false;
            for(int32 k=0;k<a.DomainAndChart.Num();++k)
                if(!budget.Consume()||!ChargeReferenceText(budget,a.DomainAndChart[k])||a.DomainAndChart[k]!=b.DomainAndChart[k])return false;
        }
        return true;
    }
};
// Other games may implement their own producer. Protected construction avoids
// accidental direct mixtures; it does NOT authenticate arbitrary subclasses.
class FReferenceTransportProducer
{
protected:
    static bool SealReference(FReferenceTransportData&& data,C::FDistanceBudget& budget,FReferenceTransportPtr& out,FString& reason)
    {
        out.Reset();reason.Reset();RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        auto fail=[&](const TCHAR* message){reason=message;return false;};
        if(!data.CanonicalUnion||!data.NominalScene||data.CanonicalUnion->Num()!=data.NominalScene->GetOwners().Num()||
            data.ProviderFaceIndices.Num()!=data.CanonicalUnion->Num()||data.SphereResponseContracts.Num()!=data.CanonicalUnion->Num()||data.Faces.IsEmpty()||data.Packet.IsEmpty()||
            data.Model.IsEmpty()||data.ExternalPacketIdentity.IsEmpty()||data.ProofIdentities.IsEmpty())
            return fail(TEXT("Reference transport is structurally incomplete"));
        // Bounded integer hash work: one charge per at most64 input bytes;
        // no geometric/gameplay tolerance is introduced.
        if(!budget.Consume()||!ChargeReferenceBytes(budget,data.Packet.Num())||!ChargeReferenceText(budget,data.Model)||
            !ChargeReferenceText(budget,data.ExternalPacketIdentity))return fail(TEXT("Reference packet/label hash budget exhausted"));
        int32 previous=INDEX_NONE;
        TSet<FString> faceIds;
        for(const auto& face:data.Faces)
        {
            if(!budget.Consume()||!ChargeReferenceText(budget,face.OpaqueId)||!ChargeReferenceText(budget,face.CanonicalRecordIdentity)||
                !ChargeReferenceText(budget,face.BoundRecordIdentity)||face.OpaqueId.IsEmpty()||faceIds.Contains(face.OpaqueId)||
                face.CanonicalRecordIdentity.IsEmpty()||face.BoundRecordIdentity.IsEmpty()||face.DomainAndChart.IsEmpty())
                return fail(TEXT("Reference face identity/domain closure differs"));
            faceIds.Add(face.OpaqueId);
            for(const auto& field:face.DomainAndChart)
                if(!budget.Consume()||!ChargeReferenceText(budget,field)||field.IsEmpty())return fail(TEXT("Reference domain field/budget unavailable"));
        }
        for(int32 i=0;i<data.ProviderFaceIndices.Num();++i)
        {
            const int32 face=data.ProviderFaceIndices[i];
            if(!budget.Consume()||!data.Faces.IsValidIndex(face)||!data.SphereResponseContracts[i].StructurallyValid(face)||face<previous||face>previous+1||
                data.Faces[face].FromPlane!=data.NominalScene->GetOwners()[i].FromPlane||
                (face==previous&&!data.Faces[face].FromPlane))
                return fail(TEXT("Reference complete provider/face partition differs"));
            previous=face;
        }
        if(previous!=data.Faces.Num()-1)return fail(TEXT("Reference face partition omits a face"));
        for(const auto& proof:data.ProofIdentities)
            if(!budget.Consume()||!ChargeReferenceText(budget,proof)||proof.IsEmpty())return fail(TEXT("Reference proof identities incomplete"));
        if(data.ExactAffineExtrusions.IsEmpty()!=data.ExactAffineProofIdentity.IsEmpty()||
            int64(data.ExactAffineExtrusions.Num())>int64(data.CanonicalUnion->Num())*6)
            return fail(TEXT("Optional fixed-source affine atlas is incomplete"));
        int32 previousProvider=INDEX_NONE,previousAxis=INDEX_NONE,previousParameter=INDEX_NONE;
        for(const auto& product:data.ExactAffineExtrusions)
        {
            const int32 axis=product.Product.FreeWorldAxis,parameter=int32(product.Product.FreeAlongU);
            const bool after=product.Provider>previousProvider||(product.Provider==previousProvider&&
                (axis>previousAxis||(axis==previousAxis&&parameter>previousParameter)));
            if(!budget.Consume()||!after||!data.ProviderFaceIndices.IsValidIndex(product.Provider)||
                product.SourceFace!=data.ProviderFaceIndices[product.Provider]||!data.Faces.IsValidIndex(product.SourceFace)||
                product.CorrelationProofIdentity!=data.ExactAffineProofIdentity)
                return fail(TEXT("Optional fixed-source affine atlas ownership/order differs"));
            const auto& face=data.Faces[product.SourceFace];
            if(face.FromPlane||product.SourceFaceIdentity!=face.OpaqueId||
                product.CanonicalRecordIdentity!=face.CanonicalRecordIdentity||product.BoundRecordIdentity!=face.BoundRecordIdentity||
                product.OriginalDomains!=face.DomainAndChart)
                return fail(TEXT("Optional fixed-source affine atlas face/domain binding differs"));
            C::FIntervalTensorSurface original;
            if(!data.CanonicalUnion->CopySurfaceForQuery(product.Provider,budget,original)||!product.StructurallyMatches(original,budget))
                return fail(TEXT("Optional fixed-source affine atlas coefficient/budget binding differs"));
            previousProvider=product.Provider;previousAxis=axis;previousParameter=parameter;
        }
        if(!data.ExactAffineExtrusions.IsEmpty())
            for(int32 i=0;i<data.CanonicalUnion->Num();++i)
                if(!budget.Consume())return fail(TEXT("Optional fixed-source affine index budget exhausted"));
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        out=MakeShareable(new FReferenceTransport(MoveTemp(data)));return true;
    }
};
}
