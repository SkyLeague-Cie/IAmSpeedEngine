// Shares the bounded staging implementation in SpeedAnalyticAuthoredImport.cpp.
#include "AssetRegistry/AssetRegistryModule.h"
#include "Misc/PackageName.h"
#include "UObject/SavePackage.h"
#include "UObject/UObjectHash.h"

namespace SpeedAuthoredImport
{
	bool FreshStadiumPackage(const FString& Name, FString& Filename, FString& Reason)
	{
		if (!Name.StartsWith(TEXT("/Game/Generated/Analytic/RLDimension/")) ||
			Name.Contains(TEXT("..")) || !FPackageName::IsValidLongPackageName(Name))
			return Fail(Reason, TEXT("Expected a valid project RLDimension package path."));
		Filename = FPaths::ConvertRelativePathToFull(FPackageName::LongPackageNameToFilename(
			Name, FPackageName::GetAssetPackageExtension()));
		const FString Content = FPaths::ConvertRelativePathToFull(FPaths::ProjectContentDir());
		if (!FPaths::IsUnderDirectory(Filename, Content) || FindPackage(nullptr, *Name) ||
			FPackageName::DoesPackageExist(Name) || IFileManager::Get().FileExists(*Filename))
			return Fail(Reason, TEXT("Package must be fresh, unloaded, and inside the current project Content."));
		return true;
	}

	template<class T> T* PersistentCopy(T* Candidate, const FString& PackageName)
	{
		UPackage* Package = CreatePackage(*PackageName);
		FObjectDuplicationParameters Params(Candidate, Package);
		Params.DestName = FName(*FPackageName::GetLongPackageAssetName(PackageName));
		Params.FlagMask = static_cast<EObjectFlags>(RF_AllFlags & ~RF_Transient);
		Params.ApplyFlags = RF_NoFlags;
		T* Copy = Cast<T>(StaticDuplicateObjectEx(Params));
		if (!Copy) return nullptr;
		Copy->ClearFlags(RF_Transient);
		Copy->SetFlags(RF_Public | RF_Standalone);
		// Instanced mesh source descriptions/body setup must not retain transient flags.
		TArray<UObject*> Subobjects;
		GetObjectsWithOuter(Copy, Subobjects, EGetObjectsFlags::IncludeNestedObjects);
		for (UObject* Object : Subobjects) Object->ClearFlags(RF_Transient);
		return Copy;
	}

	bool SaveFreshStadiumAsset(UObject* Asset, const FString& Filename, FString& Reason)
	{
		if (!Asset || Asset->HasAnyFlags(RF_Transient) || IFileManager::Get().FileExists(*Filename))
			return Fail(Reason, TEXT("Persistent asset or fresh output contract failed."));
		if (!IFileManager::Get().MakeDirectory(*FPaths::GetPath(Filename), true))
			return Fail(Reason, TEXT("Cannot create declared asset output directory."));
		UPackage* Package = Asset->GetOutermost();
		Package->MarkPackageDirty();
		Package->GetMetaData();
		FSavePackageArgs Args;
		Args.TopLevelFlags = RF_Public | RF_Standalone;
		Args.SaveFlags = SAVE_NoError;
		if (!UPackage::SavePackage(Package, Asset, *Filename, Args))
			return Fail(Reason, TEXT("Package save failed; retain any partial output and inspect before recovery."));
		FAssetRegistryModule::AssetCreated(Asset);
		return true;
	}
}

void USpeedAnalyticAuthoredImport::PersistStadiumV1(USpeedAnalyticSourceComponent* Source,
	UPrimitiveComponent* Contact, const FString& ExpectedProjectDirectory, const FString& PackageRoot,
	bool& OutSuccess, UStaticMesh*& OutRenderMesh, USpeedAnalyticCollisionAsset*& OutAnalyticAsset,
	FString& OutManifestJson, FString& OutReason)
{
	using namespace SpeedAuthoredImport;
	OutSuccess = false; OutRenderMesh = nullptr; OutAnalyticAsset = nullptr; OutManifestJson.Reset(); OutReason.Reset();
	if (ExpectedProjectDirectory.IsEmpty() || !FPaths::IsSamePath(
		FPaths::ConvertRelativePathToFull(ExpectedProjectDirectory),
		FPaths::ConvertRelativePathToFull(FPaths::ProjectDir())))
		{ Fail(OutReason, TEXT("Current project does not match the explicitly selected private checkout.")); return; }
	const FString MeshPackage = PackageRoot / TEXT("SM_StadiumC2");
	const FString AnalyticPackage = PackageRoot / TEXT("DA_StadiumC2");
	FString MeshFilename, AnalyticFilename;
	if (!FreshStadiumPackage(MeshPackage, MeshFilename, OutReason) ||
		!FreshStadiumPackage(AnalyticPackage, AnalyticFilename, OutReason)) return;
	FString AnalyticManifest, RenderManifest;
	TStrongObjectPtr<USpeedAnalyticCollisionAsset> Analytic(StageStadiumV1(Source, Contact, AnalyticManifest, OutReason));
	if (!Analytic.IsValid()) return;
	TStrongObjectPtr<UStaticMesh> Mesh(StageStadiumRenderV1(RenderManifest, OutReason));
	if (!Mesh.IsValid()) return;
	// Recheck actual registration/identity/policy after render staging, before saving.
	FSpeedAnalyticAuthoredSourceRecord Current;
	if (Analytic->AuthoredSources.Num() != 1 || !Capture(Source, Contact, Current, OutReason) ||
		SourceKey(Current) != SourceKey(Analytic->AuthoredSources[0]) ||
		Current.bQueryCollisionEnabled != Analytic->AuthoredSources[0].bQueryCollisionEnabled)
		{ Fail(OutReason, TEXT("Actual analytic source/contact changed during staging.")); return; }
	// The fast PostLoad path does not restore the per-LOD high precision override.
	// Persist the build setting and use the regular Editor rebuild on duplication
	// and cold load. This operation therefore needs the bounded Editor build profile.
	Mesh->GetSourceModel(0).BuildSettings.bUseHighPrecisionTangentBasis = true;
	Mesh->GetSourceModel(0).BuildSettings.bRemoveDegenerates = false;
	Mesh->bDoFastBuild = false;
	if (!FreshStadiumPackage(MeshPackage, MeshFilename, OutReason) ||
		!FreshStadiumPackage(AnalyticPackage, AnalyticFilename, OutReason)) return;
	TStrongObjectPtr<USpeedAnalyticCollisionAsset> SavedAnalytic(PersistentCopy(Analytic.Get(), AnalyticPackage));
	TStrongObjectPtr<UStaticMesh> SavedMesh(PersistentCopy(Mesh.Get(), MeshPackage));
	if (!SavedAnalytic.IsValid() || !SavedMesh.IsValid() ||
		!SavedAnalytic->ValidateGeneratedData(&OutReason, false) ||
		SavedMesh->GetNumTriangles(0) != RenderTriangles || !SavedMesh->GetMeshDescription(0) ||
		SavedMesh->GetMeshDescription(0)->Vertices().Num() != RenderPositions)
		{ Fail(OutReason, TEXT("Persistent copy failed structural validation; no Blueprint changed.")); return; }
	if (!SavedMesh->GetRenderData() || SavedMesh->GetRenderData()->LODResources.Num() != 1 ||
		!SavedMesh->GetRenderData()->LODResources[0].VertexBuffers.StaticMeshVertexBuffer.GetUseHighPrecisionTangentBasis() ||
		!SavedMesh->GetBodySetup() || SavedMesh->GetBodySetup()->AggGeom.GetElementCount() != 0 ||
		SavedMesh->GetBodySetup()->DefaultInstance.GetCollisionEnabled() != ECollisionEnabled::NoCollision)
		{ Fail(OutReason, TEXT("Persistent render precision or empty collision contract changed.")); return; }
	for (const FStaticMeshSection& Section : SavedMesh->GetRenderData()->LODResources[0].Sections)
		if (Section.bEnableCollision) { Fail(OutReason, TEXT("Persistent mesh unexpectedly enables section collision.")); return; }
	bool bAnalyticSaved = false, bRenderSaved = false;
	auto Receipt = [&]()
	{
		const auto Json = MakeShared<FJsonObject>();
		Json->SetStringField(TEXT("schema"), TEXT("skyleague.stadium-persistent-assets/v1"));
		Json->SetStringField(TEXT("analytic_asset"), SavedAnalytic->GetPathName());
		Json->SetStringField(TEXT("render_asset"), SavedMesh->GetPathName());
		Json->SetStringField(TEXT("analytic_staging_manifest"), AnalyticManifest);
		Json->SetStringField(TEXT("render_staging_manifest"), RenderManifest);
		Json->SetBoolField(TEXT("analytic_saved"), bAnalyticSaved);
		Json->SetBoolField(TEXT("render_saved"), bRenderSaved);
		Json->SetBoolField(TEXT("blueprint_modified"), false);
		Json->SetBoolField(TEXT("physical_authority_promoted"), false);
		Json->SetBoolField(TEXT("cold_load_verified"), false);
		Json->SetBoolField(TEXT("regular_editor_rebuild_required"), true);
		FJsonSerializer::Serialize(Json, TJsonWriterFactory<>::Create(&OutManifestJson));
	};
	bAnalyticSaved = SaveFreshStadiumAsset(SavedAnalytic.Get(), AnalyticFilename, OutReason);
	if (!bAnalyticSaved) { Receipt(); return; }
	bRenderSaved = SaveFreshStadiumAsset(SavedMesh.Get(), MeshFilename, OutReason);
	Receipt();
	if (!bRenderSaved) return;
	OutAnalyticAsset = SavedAnalytic.Get(); OutRenderMesh = SavedMesh.Get();
	OutReason = TEXT("Both assets saved; analytic authority remains unchanged. Blueprint binding and cold-load validation are separate.");
	OutSuccess = true;
}

void USpeedAnalyticAuthoredImport::ValidateStadiumBinding(USpeedAnalyticSourceComponent* Source,
	UPrimitiveComponent* Contact, USpeedAnalyticCollisionAsset* Asset, bool& OutMatches, FString& OutReason)
{
	OutMatches = false; OutReason.Reset();
	if (!IsInGameThread() || !IsValid(Source) || !IsValid(Contact) || !IsValid(Asset) || Asset->AuthoredSources.Num() != 1)
	{ OutReason = TEXT("Expected a real source, contact and one-source asset on the editor thread."); return; }
	OutMatches = Source->ResolveAuthoredContact(Asset->AuthoredSources[0], *Asset, &OutReason) == Contact;
	if (!OutMatches && OutReason.IsEmpty()) OutReason = TEXT("Resolved contact differs from the explicit target.");
}

void USpeedAnalyticAuthoredImport::PersistStadiumAnalyticV1(USpeedAnalyticSourceComponent* Source,
	UPrimitiveComponent* Contact, const FString& ExpectedProjectDirectory, const FString& PackageRoot,
	bool& OutSuccess, USpeedAnalyticCollisionAsset*& OutAnalyticAsset, FString& OutManifestJson, FString& OutReason)
{
	using namespace SpeedAuthoredImport;
	OutSuccess = false; OutAnalyticAsset = nullptr; OutManifestJson.Reset(); OutReason.Reset();
	if (ExpectedProjectDirectory.IsEmpty() || !FPaths::IsSamePath(
		FPaths::ConvertRelativePathToFull(ExpectedProjectDirectory), FPaths::ConvertRelativePathToFull(FPaths::ProjectDir())))
	{ Fail(OutReason, TEXT("Current project differs from the selected private checkout.")); return; }
	const FString PackageName = PackageRoot / TEXT("DA_StadiumC2");
	FString Filename, StagingManifest;
	if (!FreshStadiumPackage(PackageName, Filename, OutReason)) return;
	TStrongObjectPtr<USpeedAnalyticCollisionAsset> Candidate(StageStadiumV1(Source, Contact, StagingManifest, OutReason));
	if (!Candidate.IsValid()) return;
	FSpeedAnalyticAuthoredSourceRecord Current;
	if (Candidate->AuthoredSources.Num() != 1 || !Capture(Source, Contact, Current, OutReason) ||
		SourceKey(Current) != SourceKey(Candidate->AuthoredSources[0]) ||
		Current.bQueryCollisionEnabled != Candidate->AuthoredSources[0].bQueryCollisionEnabled ||
		!FreshStadiumPackage(PackageName, Filename, OutReason)) return;
	TStrongObjectPtr<USpeedAnalyticCollisionAsset> Saved(PersistentCopy(Candidate.Get(), PackageName));
	if (!Saved.IsValid() || !Saved->ValidateGeneratedData(&OutReason, false)) return;
	const bool bSaved = SaveFreshStadiumAsset(Saved.Get(), Filename, OutReason);
	const auto Json = MakeShared<FJsonObject>();
	Json->SetStringField(TEXT("schema"), TEXT("skyleague.stadium-persistent-analytic/v1"));
	Json->SetStringField(TEXT("analytic_asset"), Saved->GetPathName());
	Json->SetStringField(TEXT("analytic_staging_manifest"), StagingManifest);
	Json->SetBoolField(TEXT("analytic_saved"), bSaved);
	Json->SetBoolField(TEXT("render_modified"), false);
	Json->SetBoolField(TEXT("blueprint_modified"), false);
	Json->SetBoolField(TEXT("physical_authority_promoted"), false);
	Json->SetBoolField(TEXT("cold_load_verified"), false);
	FJsonSerializer::Serialize(Json, TJsonWriterFactory<>::Create(&OutManifestJson));
	if (!bSaved) return;
	OutAnalyticAsset = Saved.Get(); OutSuccess = true;
	OutReason = TEXT("Analytic asset saved; render unchanged. Blueprint binding and cold read remain separate.");
}
