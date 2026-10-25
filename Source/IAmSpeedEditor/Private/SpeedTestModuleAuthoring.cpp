#include "SpeedTestModuleAuthoring.h"
#include "Engine/StaticMesh.h"
#include "MeshDescription.h"
#include "StaticMeshAttributes.h"
#include "StaticMeshOperations.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "Misc/PackageName.h"
#include "UObject/Package.h"
#include "UObject/SavePackage.h"

UStaticMesh* USpeedTestModuleAuthoring::CreateModuleMesh(const FString& PackageName,const TArray<FVector>& V,const TArray<int32>& T)
{
    if(!PackageName.StartsWith(TEXT("/Game/")) || !FPackageName::IsValidLongPackageName(PackageName) || V.Num()<3 || V.Num()>100000 || T.IsEmpty() || T.Num()%3 || T.Num()>600000)return nullptr;
    for(const auto& P:V)if(P.ContainsNaN())return nullptr;
    for(int32 I:T)if(!V.IsValidIndex(I))return nullptr;
    FMeshDescription M;FStaticMeshAttributes A(M);A.Register();
    auto Positions=A.GetVertexPositions();auto Normals=A.GetVertexInstanceNormals();auto UV=A.GetVertexInstanceUVs();UV.SetNumChannels(1);
    const auto Group=M.CreatePolygonGroup();A.GetPolygonGroupMaterialSlotNames()[Group]=TEXT("Module");TArray<FVertexID> Ids;
    auto Tangents=A.GetVertexInstanceTangents();auto Signs=A.GetVertexInstanceBinormalSigns();auto Colors=A.GetVertexInstanceColors();
    TArray<FVector3f> ExpectedNormals;
    for(const auto& P:V){const auto Id=M.CreateVertex();Positions[Id]=FVector3f(P);Ids.Add(Id);}
    for(int32 I=0;I<T.Num();I+=3)
    {
        const FVector N=FVector::CrossProduct(V[T[I+1]]-V[T[I]],V[T[I+2]]-V[T[I]]).GetSafeNormal();
        if(N.IsNearlyZero())return nullptr;
        ExpectedNormals.Add(FVector3f(N));
        const FVector Tangent=FVector::CrossProduct(FMath::Abs(N.Z)<.99?FVector::UpVector:FVector::RightVector,N).GetSafeNormal();
        TArray<FVertexInstanceID> Face;
        for(int32 J:{0,2,1}) // UE MeshDescription uses Cross(C-A,B-A).
        {const auto Id=M.CreateVertexInstance(Ids[T[I+J]]);Normals[Id]=FVector3f(N);Tangents[Id]=FVector3f(Tangent);Signs[Id]=1;Colors[Id]=FVector4f(1,1,1,1);UV.Set(Id,0,FVector2f(V[T[I+J]].X*.001,V[T[I+J]].Y*.001));Face.Add(Id);}
        M.CreatePolygon(Group,Face);
    }
    FStaticMeshOperations::ComputeTriangleTangentsAndNormals(M);
    int32 NormalIndex=0;
    for(const auto Id:M.Triangles().GetElementIDs())
        if(!ExpectedNormals.IsValidIndex(NormalIndex) || FVector3f::DotProduct(A.GetTriangleNormals()[Id],ExpectedNormals[NormalIndex++])<.999f)return nullptr;
    const FString Name=FPackageName::GetLongPackageAssetName(PackageName);
    UPackage* Package=FPackageName::DoesPackageExist(PackageName)?LoadPackage(nullptr,*PackageName,LOAD_None):CreatePackage(*PackageName);
    if(!Package)return nullptr;Package->FullyLoad();
    auto* Asset=FindObject<UStaticMesh>(Package,*Name);
    if(!Asset){Asset=NewObject<UStaticMesh>(Package,*Name,RF_Public|RF_Standalone);FAssetRegistryModule::AssetCreated(Asset);}
    Asset->GetStaticMaterials().Reset();Asset->GetStaticMaterials().Add(FStaticMaterial(nullptr,TEXT("Module")));
    UStaticMesh::FBuildMeshDescriptionsParams Build;Build.bBuildSimpleCollision=false;Build.bCommitMeshDescription=true;
    if(!Asset->BuildFromMeshDescriptions({&M},Build))return nullptr;
    Asset->MarkPackageDirty();FSavePackageArgs Save;Save.TopLevelFlags=RF_Public|RF_Standalone;
    return UPackage::SavePackage(Package,Asset,*FPackageName::LongPackageNameToFilename(PackageName,FPackageName::GetAssetPackageExtension()),Save)?Asset:nullptr;
}
