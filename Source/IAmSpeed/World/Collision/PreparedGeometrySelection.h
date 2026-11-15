#pragma once
#include "ReferenceTransport.h"
namespace Speed
{
enum class EPreparedGeometryModel:uint8 { NominalInput,RetainedReference };
class FPreparedGeometrySelection;
using FPreparedGeometrySelectionPtr=TSharedPtr<const FPreparedGeometrySelection,ESPMode::ThreadSafe>;
// Explicit immutable model choice ONLY. Structural aliases and possession do
// not authenticate coefficients against a packet, confer authority or Ready,
// or certify a physical request/trajectory/response. The closed producer must
// independently authenticate the real inputs at their actual World boundary.
class FPreparedGeometrySelection final
{
    EPreparedGeometryModel Model;
    Analytic::Prepared::FScenePtr Nominal;
    Analytic::Prepared::FReferenceTransportPtr Reference;
    Analytic::Certified::FIndexedIntervalUnionPtr Selected;
    FPreparedGeometrySelection(EPreparedGeometryModel model,Analytic::Prepared::FScenePtr nominal,
        Analytic::Prepared::FReferenceTransportPtr reference,Analytic::Certified::FIndexedIntervalUnionPtr selected)
        :Model(model),Nominal(MoveTemp(nominal)),Reference(MoveTemp(reference)),Selected(MoveTemp(selected)){}
public:
    static bool TryCreate(EPreparedGeometryModel model,Analytic::Prepared::FScenePtr nominal,
        Analytic::Prepared::FReferenceTransportPtr reference,FPreparedGeometrySelectionPtr& out,FString& reason)
    {
        out.Reset();reason.Reset();
        auto fail=[&](const TCHAR* message){reason=message;return false;};
        if(!nominal||!nominal->GetUnion()||nominal->GetOwners().Num()!=nominal->GetUnion()->Num())
            return fail(TEXT("Prepared geometry nominal provenance unavailable"));
        Analytic::Certified::FIndexedIntervalUnionPtr selected;
        switch(model)
        {
        case EPreparedGeometryModel::NominalInput:
            if(reference)return fail(TEXT("Nominal selection cannot discard a retained reference"));
            selected=nominal->GetUnion();break;
        case EPreparedGeometryModel::RetainedReference:
            if(!reference||reference->GetNominalScene()!=nominal||!reference->GetCanonicalUnion()||
                reference->GetCanonicalUnion()->Num()!=nominal->GetOwners().Num()||
                reference->GetProviderFaceIndices().Num()!=nominal->GetOwners().Num())
                return fail(TEXT("Retained reference selection is missing, mixed or incomplete"));
            selected=reference->GetCanonicalUnion();break;
        default:return fail(TEXT("Unknown prepared geometry model"));
        }
        // Equal coefficient unions may be legitimate for two explicit models.
        // Neither pointer inequality, SourceHash nor a label selects geometry.
        out=MakeShareable(new FPreparedGeometrySelection(model,MoveTemp(nominal),MoveTemp(reference),MoveTemp(selected)));
        return true;
    }
    bool HasValidAliases() const
    {
        if(!Nominal||!Selected||Selected->Num()!=Nominal->GetOwners().Num())return false;
        switch(Model)
        {
        case EPreparedGeometryModel::NominalInput:return !Reference&&Selected==Nominal->GetUnion();
        case EPreparedGeometryModel::RetainedReference:return Reference&&Reference->GetNominalScene()==Nominal&&
            Selected==Reference->GetCanonicalUnion()&&Reference->GetProviderFaceIndices().Num()==Selected->Num();
        default:return false;
        }
    }
    EPreparedGeometryModel GetModel() const {return Model;}
    const Analytic::Prepared::FScenePtr& GetNominalScene() const {return Nominal;}
    const Analytic::Prepared::FReferenceTransportPtr& GetReference() const {return Reference;}
    const Analytic::Certified::FIndexedIntervalUnionPtr& GetSelectedUnion() const {return Selected;}
};
}
