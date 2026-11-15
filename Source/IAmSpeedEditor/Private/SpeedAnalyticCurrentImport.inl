#include "Components/ChildActorComponent.h"
// Separate current transport contract. Numeric2 / StageStadiumV1 stay unchanged.
// Included after PersistImport so only the established bounded IO/save helpers are shared.
namespace SpeedAuthoredCurrent
{
    using namespace SpeedAuthoredImport;
    constexpr FInputPin IdentityPin{ TEXT("RLDimension/GoalCorrectedV1/CurrentNumeric2/identity-manifest.json"), 734253,
        TEXT("82c0a775f27cb54449c96f2d7e8066680640b125ab1c8e17f213b1bc0f600dcf") };
    constexpr FInputPin NumericPin{ TEXT("RLDimension/GoalCorrectedV1/CurrentNumeric2/geometry.json"), 5727616,
        TEXT("6a7d4caa177a5daaa6306147ec6fe78c3c7d067081307e156bb04e4c05841648") };
    constexpr FInputPin ConversionPin{ TEXT("RLDimension/GoalCorrectedV1/CurrentNumeric2/conversion-bounds.json"), 5745123,
        TEXT("d2f7ef063c583ad22dc3678549e56f24fd24c279ef136eb393966a57c511b181") };
    constexpr const TCHAR* CurrentRecipeId = TEXT("MEASURED_GOALS_CURRENT_ASSEMBLY1");
    constexpr const TCHAR* CurrentSourceSchema = TEXT("skyleague.stadium.measured-complete-atlas/v1");
    constexpr const TCHAR* ExactSourceSha = TEXT("46cd45f7af71cf745f541c92e1a10066c9dba9cb7fcc27d1fbe0d0ecdcaf31a8");
    constexpr const TCHAR* ExactC2Sha = TEXT("27a91f698bc375f277d4f09deb4ecb4c14f03dc229e9c5dbb28f3347a08f1d55");
    constexpr const TCHAR* LegacyNamespace = TEXT("C2_ASSEMBLED_STADIUM_OCTAGONAL_V2");
    constexpr int32 CurrentTensors = 2636, CurrentPlanes = 18, CurrentDomainVertices = 392;

    FObject Child(const FObject& Object, const TCHAR* Key)
    {
        const FObject* Out = nullptr;
        return Object.IsValid() && Object->TryGetObjectField(Key, Out) && Out ? *Out : nullptr;
    }
    bool BoolIs(const FObject& Object, const TCHAR* Key, bool Expected)
    {
        const auto Value = Object.IsValid() ? Object->TryGetField(Key) : nullptr;
        return Value.IsValid() && Value->Type == EJson::Boolean && Value->AsBool() == Expected;
    }
    bool StrictUint64(const FString& Text, uint64& Out)
    {
        Out = 0;
        if (Text.IsEmpty() || (Text.Len() > 1 && Text[0] == TEXT('0'))) return false;
        for (TCHAR C : Text)
        {
            if (C < TEXT('0') || C > TEXT('9')) return false;
            const uint64 Digit = static_cast<uint64>(C - TEXT('0'));
            if (Out > (MAX_uint64 - Digit) / 10) return false;
            Out = Out * 10 + Digit;
        }
        return true;
    }
    bool Dyadic(const TSharedPtr<FJsonValue>& Value)
    {
        const FValues* Pair = nullptr;
        if (!Value.IsValid() || !Value->TryGetArray(Pair) || !Pair || Pair->Num() != 2 ||
            !(*Pair)[0].IsValid() || (*Pair)[0]->Type != EJson::String ||
            !(*Pair)[1].IsValid() || (*Pair)[1]->Type != EJson::Number) return false;
        FString Mantissa; uint64 M = 0; double E = 0.0;
        if (!(*Pair)[0]->TryGetString(Mantissa) || !StrictUint64(Mantissa, M) ||
            !(*Pair)[1]->TryGetNumber(E) || !FMath::IsFinite(E) || E < -4096 || E > 4096 ||
            E != FMath::FloorToDouble(E)) return false;
        // Do not convert this exact pair to a floating value (which could under/overflow).
        return M != 0 || E == 0;
    }
    bool DyadicArray(const FObject& Object, const TCHAR* Key, int32 Count)
    {
        const FValues* Values = nullptr;
        if (!Array(Object, Key, Values, Count)) return false;
        for (const auto& Value : *Values) if (!Dyadic(Value)) return false;
        return true;
    }
    bool PackedPoint(const TSharedPtr<FJsonValue>& Point, const FString& Hex, FVector& Out)
    {
        const FValues* Values = nullptr;
        if (Hex.Len() != 48 || !Point.IsValid() || !Point->TryGetArray(Values) || !Values || Values->Num() != 3) return false;
        double V[3]{};
        for (int32 K = 0; K < 3; ++K)
        {
            if (!(*Values)[K].IsValid() || (*Values)[K]->Type != EJson::Number ||
                !(*Values)[K]->TryGetNumber(V[K]) || !FMath::IsFinite(V[K]) ||
                DoubleHex(V[K]) != Hex.Mid(K * 16, 16)) return false;
        }
        Out = FVector(V[0], V[1], V[2]); return true;
    }
    bool IsAdaptedCap(const FString& Id)
    {
        return Id == TEXT("C2_MEASURED_GOAL_CAP_V3/sx1sy1/core") ||
            Id == TEXT("C2_MEASURED_GOAL_CAP_V3/sx1sy-1/core") ||
            Id == TEXT("C2_MEASURED_GOAL_CAP_V3/sx-1sy1/core") ||
            Id == TEXT("C2_MEASURED_GOAL_CAP_V3/sx-1sy-1/core");
    }
    bool FaceMetadata(const FObject& R, const FObject& G, const FObject& B, bool bPlane)
    {
        FString Id, Family, Origin;
        if (!R.IsValid() || !R->TryGetStringField(TEXT("id"), Id) ||
            !R->TryGetStringField(TEXT("source_family"), Family) || Family.IsEmpty() ||
            !R->TryGetStringField(TEXT("source_family_origin"), Origin) ||
            !TextIs(G, TEXT("id"), Id) || !TextIs(B, TEXT("id"), Id) ||
            !TextIs(G, TEXT("source_family"), Family) || !TextIs(B, TEXT("source_family"), Family) ||
            !TextIs(G, TEXT("source_family_origin"), Origin) || !TextIs(B, TEXT("source_family_origin"), Origin)) return false;
        if (IsAdaptedCap(Id))
            return bPlane && Origin == TEXT("structural-plane-type/v1") && Family == TEXT("AnalyticPrimitive/BoundedPlane");
        return Origin == TEXT("source-record");
    }
    FString IdentityPrefix(uint64 SourceId)
    {
        return FString::Printf(TEXT("AnalyticStadiumImport/v1|%016llx|%s"),
            static_cast<unsigned long long>(SourceId), LegacyNamespace);
    }
    bool IdentityContract(const FObject& Manifest)
    {
        const auto Ids = Child(Manifest, TEXT("identity_derivation"));
        const auto RoleScope = Child(Ids, TEXT("role_scope"));
        const FValues* Roles = nullptr;
        return TextIs(Ids, TEXT("algorithm"), TEXT("StableStringId/v1")) &&
            TextIs(Ids, TEXT("prefix"), TEXT("AnalyticStadiumImport/v1")) &&
            TextIs(Ids, TEXT("namespace_recipe_id"), LegacyNamespace) &&
            TextIs(Ids, TEXT("source_id"), TEXT("ACTUAL_CAPTURED_COMPONENT_UINT64_NOT_JSON_NUMBER")) &&
            TextIs(Ids, TEXT("face_key"), TEXT("manifest_row.id")) &&
            BoolIs(Ids, TEXT("retained_face_identity_preserved_by_same_namespace_and_literal_ID"), true) &&
            TextIs(RoleScope, TEXT("group"), TEXT("tensor_only")) && Array(Ids, TEXT("roles"), Roles, 4) &&
            StringValueIs((*Roles)[0], TEXT("surface")) && StringValueIs((*Roles)[1], TEXT("feature")) &&
            StringValueIs((*Roles)[2], TEXT("primitive")) && StringValueIs((*Roles)[3], TEXT("group"));
    }
    bool MetadataContract(const FObject& Manifest)
    {
        const FValues* Rows = nullptr;
        if (!Array(Manifest, TEXT("metadata_adaptations"), Rows, 4)) return false;
        TSet<FString> Seen;
        for (int32 I = 0; I < Rows->Num(); ++I)
        {
            const auto Row = ObjectAt(*Rows, I); FString Id;
            if (!Row.IsValid() || !Row->TryGetStringField(TEXT("id"), Id) || !IsAdaptedCap(Id) || Seen.Contains(Id) ||
                !TextIs(Row, TEXT("field"), TEXT("source_family")) || !BoolIs(Row, TEXT("source_key_present"), false) ||
                !TextIs(Row, TEXT("value"), TEXT("AnalyticPrimitive/BoundedPlane")) ||
                !TextIs(Row, TEXT("origin"), TEXT("structural-plane-type/v1"))) return false;
            Seen.Add(Id);
        }
        return Seen.Num() == 4;
    }
    bool ConversionContract(const FObject& Bounds)
    {
        int32 Bits = 0; const FValues* Order = nullptr;
        if (!Int(Bounds, TEXT("upper_bound_significand_bits"), Bits, 64, 64) ||
            !TextIs(Bounds, TEXT("tensor_hex_encoding"), TEXT("packed-binary64-be-xyz-u-major-v-minor/v1")) ||
            !TextIs(Bounds, TEXT("upper_bound_encoding"), TEXT("nonnegative-uint64-decimal-mantissa-times-two-power-signed-exponent/v1")) ||
            !Array(Bounds, TEXT("normalized_jet_order"), Order, 9) ||
            !BoolIs(Bounds, TEXT("json_binary64_bit_roundtrip_verified"), true) ||
            !BoolIs(Bounds, TEXT("rational_proofs_transferred"), false) ||
            !BoolIs(Bounds, TEXT("physical_admission"), false) || !TextIs(Bounds, TEXT("Native"), TEXT("NOT_RUN"))) return false;
        for (int32 I = 0; I < 9; ++I)
        {
            const FValues* Pair = nullptr; double U = 0.0, V = 0.0;
            if (!(*Order)[I].IsValid() || !(*Order)[I]->TryGetArray(Pair) || !Pair || Pair->Num() != 2 ||
                !(*Pair)[0].IsValid() || (*Pair)[0]->Type != EJson::Number || !(*Pair)[0]->TryGetNumber(U) ||
                !(*Pair)[1].IsValid() || (*Pair)[1]->Type != EJson::Number || !(*Pair)[1]->TryGetNumber(V) ||
                U != I / 3 || V != I % 3) return false;
        }
        for (const TCHAR* Key : { TEXT("floating_evaluator_roundoff"), TEXT("shared_physical_chart_delta_jets"),
            TEXT("converted_C0_C2_closure"), TEXT("converted_embedding_and_confinement"),
            TEXT("projected_Jacobian_margin_transfer"), TEXT("normal_unit_vector_error") })
            if (!TextIs(Bounds, Key, TEXT("NOT_ESTABLISHED"))) return false;
        return true;
    }
    bool CaptureCurrent(USpeedAnalyticSourceComponent* Source, UPrimitiveComponent* Contact,
        FSpeedAnalyticAuthoredSourceRecord& Out, FString& Reason)
    {
        if (!Capture(Source, Contact, Out, Reason)) return false;
        // Actual component identity/policy/transforms are unchanged. Source provenance is
        // the complete rational recipe, distinct from the compact Native identity manifest.
        Out.RecipeSchema = CurrentSourceSchema; Out.RecipeSha256 = ExactSourceSha;
        Out.PayloadSha256 = NumericPin.Sha256;
        return Out.IsValid(&Reason);
    }
    bool SameCapture(const FSpeedAnalyticAuthoredSourceRecord& A, const FSpeedAnalyticAuthoredSourceRecord& B)
    {
        return SourceKey(A) == SourceKey(B) && A.bQueryCollisionEnabled == B.bQueryCollisionEnabled &&
            A.RecipeSchema == B.RecipeSchema && A.RecipeSha256 == B.RecipeSha256 && A.PayloadSha256 == B.PayloadSha256;
    }
}

AActor* USpeedAnalyticAuthoredImport::SpawnStadiumCurrentEditorProbe(UBlueprint* Blueprint, UWorld* World, FString& OutReason)
{
    OutReason.Reset();
    if (!IsInGameThread() || !IsValid(World) || World->WorldType != EWorldType::Editor || World->IsGameWorld() ||
        World->HasBegunPlay() || !IsValid(World->PersistentLevel) || !IsValid(Blueprint) || Blueprint->bBeingCompiled ||
        !Blueprint->IsUpToDate() || !IsValid(Blueprint->GeneratedClass) ||
        Blueprint->GeneratedClass->ClassGeneratedBy != Blueprint || !Blueprint->GeneratedClass->IsChildOf(AActor::StaticClass()) ||
        Blueprint->GeneratedClass->HasAnyClassFlags(CLASS_Abstract | CLASS_Deprecated | CLASS_NewerVersionExists))
    { OutReason = TEXT("Expected a compiled actor Blueprint and an inactive Editor world for the current probe."); return nullptr; }
    FActorSpawnParameters Params;
    Params.ObjectFlags = RF_Transient; Params.bTemporaryEditorActor = true; Params.bCreateActorPackage = false;
    Params.OverrideLevel = World->PersistentLevel; Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    Params.bDeferConstruction = true; Params.TransformScaleMethod = ESpawnActorScaleMethod::OverrideRootScale;
    AActor* Actor = World->SpawnActor<AActor>(Blueprint->GeneratedClass, FTransform::Identity, Params);
    if (!Actor) { OutReason = TEXT("Current transient Editor probe spawn failed."); return nullptr; }
    Actor->Tags.AddUnique(TEXT("SLAnalyticStadiumEditorProbe"));
    Actor->Tags.AddUnique(TEXT("Stadium.AnalyticOnly"));
    TArray<UChildActorComponent*> Children;
    Actor->GetComponents(Children);
    // Own transient instance only. Never change a Blueprint template/CDO/map actor.
    // Constructors have already run; this is not a proof of no earlier registration.
    for (UChildActorComponent* Child : Children) if (Child) Child->SetChildActorClass(nullptr);
    Actor->FinishSpawning(FTransform::Identity, true, nullptr, ESpawnActorScaleMethod::OverrideRootScale);
    bool bValid = IsValid(Actor) && Actor->GetWorld() == World && Actor->GetLevel() == World->PersistentLevel &&
        Actor->GetClass() == Blueprint->GeneratedClass && Actor->HasAnyFlags(RF_Transient) &&
        Actor->ActorHasTag(TEXT("SLAnalyticStadiumEditorProbe")) && Actor->ActorHasTag(TEXT("Stadium.AnalyticOnly")) &&
        IsValid(Actor->GetRootComponent()) && Actor->GetRootComponent()->IsRegistered() &&
        Actor->GetActorTransform().Equals(FTransform::Identity, 0.0);
    if (bValid)
    {
        Children.Reset(); Actor->GetComponents(Children);
        for (const UChildActorComponent* Child : Children)
            bValid &= IsValid(Child) && Child->GetOwner() == Actor && !Child->GetChildActorClass() && !Child->GetChildActor();
    }
    if (!bValid)
    {
        if (IsValid(Actor) && !DestroyStadiumEditorProbe(Actor))
        { OutReason = TEXT("Invalid current probe cleanup failed; preserve this actor and stop authoring."); return Actor; }
        OutReason = TEXT("Current deferred construction restored child actors or invalidated probe identity."); return nullptr;
    }
    return Actor;
}

USpeedAnalyticCollisionAsset* USpeedAnalyticAuthoredImport::StageStadiumCurrentV1(
    USpeedAnalyticSourceComponent* Source, UPrimitiveComponent* Contact, FString& OutManifestJson, FString& OutReason)
{
    using namespace SpeedAuthoredImport;
    using namespace SpeedAuthoredCurrent;
    OutManifestJson.Reset(); OutReason.Reset();
    FSpeedAnalyticAuthoredSourceRecord Captured;
    if (!CaptureCurrent(Source, Contact, Captured, OutReason)) return nullptr;
    FObject Identity, Geometry, Bounds;
    if (!ReadPinned(IdentityPin, Identity, OutReason) || !ReadPinned(NumericPin, Geometry, OutReason) ||
        !ReadPinned(ConversionPin, Bounds, OutReason)) return nullptr;
    int32 SourceBytes = 0, CertificateBytes = 0, CountT = 0, CountP = 0, CountV = 0;
    const auto SourceRecipe = Child(Identity, TEXT("source_recipe"));
    const auto SourceCertificate = Child(Identity, TEXT("source_C2_certificate"));
    if (!TextIs(Identity, TEXT("schema"), TEXT("skyleague.analytic-source-identity-manifest/v1")) ||
        !TextIs(Identity, TEXT("recipe_id"), CurrentRecipeId) || !TextIs(Identity, TEXT("units"), TEXT("cm")) ||
        !TextIs(Identity, TEXT("frame"), Frame) || !TextIs(Identity, TEXT("source_recipe_schema"), CurrentSourceSchema) ||
        !TextIs(SourceRecipe, TEXT("sha256"), ExactSourceSha) || !Int(SourceRecipe, TEXT("bytes"), SourceBytes, 21910963, 21910963) ||
        !TextIs(SourceCertificate, TEXT("sha256"), ExactC2Sha) || !Int(SourceCertificate, TEXT("bytes"), CertificateBytes, 1901, 1901) ||
        !Int(Identity, TEXT("tensor_count"), CountT, CurrentTensors, CurrentTensors) ||
        !Int(Identity, TEXT("plane_count"), CountP, CurrentPlanes, CurrentPlanes) ||
        !Int(Identity, TEXT("domain_vertex_count"), CountV, CurrentDomainVertices, CurrentDomainVertices) ||
        !IdentityContract(Identity) || !MetadataContract(Identity) ||
        !TextIs(Geometry, TEXT("schema"), TEXT("skyleague.analytic-binary64-geometry-transfer/v2")) ||
        !TextIs(Geometry, TEXT("recipe_id"), CurrentRecipeId) || !TextIs(Geometry, TEXT("units"), TEXT("cm")) ||
        !TextIs(Geometry, TEXT("frame"), Frame) || !TextIs(Geometry, TEXT("identity_manifest_sha256"), IdentityPin.Sha256) ||
        !TextIs(Geometry, TEXT("source_recipe_sha256"), ExactSourceSha) ||
        !TextIs(Geometry, TEXT("runtime_source_ids_and_policies"), TEXT("UNBOUND")) ||
        !TextIs(Geometry, TEXT("authority_eligibility"), TEXT("UNASSIGNED")) ||
        !TextIs(Bounds, TEXT("schema"), TEXT("skyleague.analytic-coefficient-conversion-bounds/v2")) ||
        !TextIs(Bounds, TEXT("recipe_id"), CurrentRecipeId) || !TextIs(Bounds, TEXT("units"), TEXT("cm")) ||
        !TextIs(Bounds, TEXT("frame"), Frame) || !TextIs(Bounds, TEXT("identity_manifest_sha256"), IdentityPin.Sha256) ||
        !TextIs(Bounds, TEXT("source_recipe_sha256"), ExactSourceSha) ||
        !TextIs(Bounds, TEXT("geometry_payload_sha256"), NumericPin.Sha256) || !ConversionContract(Bounds))
    { OutReason = TEXT("Current identity/geometry/bounds provenance, identity namespace or qualification contract differs."); return nullptr; }
    const FValues *Tensors = nullptr, *Planes = nullptr, *IdentityTensors = nullptr, *IdentityPlanes = nullptr, *BoundTensors = nullptr, *BoundPlanes = nullptr;
    if (!Array(Geometry, TEXT("tensor_patches"), Tensors, CurrentTensors) ||
        !Array(Geometry, TEXT("bounded_planes"), Planes, CurrentPlanes) ||
        !Array(Identity, TEXT("patches"), IdentityTensors, CurrentTensors) ||
        !Array(Identity, TEXT("planes"), IdentityPlanes, CurrentPlanes) ||
        !Array(Bounds, TEXT("patches"), BoundTensors, CurrentTensors) ||
        !Array(Bounds, TEXT("planes"), BoundPlanes, CurrentPlanes))
    { OutReason = TEXT("Current complete ordered face tables differ."); return nullptr; }
    TStrongObjectPtr<USpeedAnalyticCollisionAsset> Candidate(NewObject<USpeedAnalyticCollisionAsset>(GetTransientPackage(), NAME_None, RF_Transient));
    Candidate->AuthoredSourceSchemaVersion = 1; Candidate->AuthoredSources.Add(Captured);
    FIdentityBuilder Ids; Ids.Prefix = IdentityPrefix(Captured.SourceId); Ids.Used.Add(Captured.SourceId);
    for (int32 I = 0; I < CurrentTensors; ++I)
    {
        const auto G = ObjectAt(*Tensors, I), R = ObjectAt(*IdentityTensors, I), B = ObjectAt(*BoundTensors, I);
        FSpeedAnalyticTensorBezierPatchRecord Patch;
        int32 RU = 0, RV = 0, Controls = 0; FString Packed;
        const FValues *Points = nullptr, *UnitDomain = nullptr;
        if (!FaceMetadata(R, G, B, false) ||
            !Int(G, TEXT("degree_u"), Patch.DegreeU, 1, 10) || !Int(G, TEXT("degree_v"), Patch.DegreeV, 1, 10) ||
            !Int(R, TEXT("degree_u"), RU, 1, 10) || !Int(R, TEXT("degree_v"), RV, 1, 10) ||
            RU != Patch.DegreeU || RV != Patch.DegreeV ||
            !Int(R, TEXT("control_count"), Controls, (RU + 1) * (RV + 1), (RU + 1) * (RV + 1)) ||
            !SameStrings(G, TEXT("original_u_domain"), R, TEXT("u_domain"), 2) ||
            !SameStrings(G, TEXT("original_v_domain"), R, TEXT("v_domain"), 2) ||
            !TextIs(G, TEXT("control_order"), TEXT("u-major-v-minor")) ||
            !Array(G, TEXT("runtime_parameter_domain"), UnitDomain, 2) ||
            !StringValueIs((*UnitDomain)[0], TEXT("0")) || !StringValueIs((*UnitDomain)[1], TEXT("1")) ||
            !Array(G, TEXT("control_points_cm"), Points, Controls) ||
            !B->TryGetStringField(TEXT("control_binary64_be_hex"), Packed) || Packed.Len() != 48 * Controls ||
            !DyadicArray(B, TEXT("normalized_jet_l1_upper_dyadic"), 9) ||
            !Dyadic(B->TryGetField(TEXT("normalized_cross_product_delta_l1_upper_dyadic"))))
        { OutReason = FString::Printf(TEXT("Current tensor %d identity, shape, packed bits or dyadic bounds differ."), I); return nullptr; }
        if (!Ids.Face(G, R, B, Patch.SurfaceId, Patch.FeatureId, Patch.PrimitiveId, &Patch.CanonicalGroupId, OutReason)) return nullptr;
        for (int32 J = 0; J < Points->Num(); ++J)
        {
            FVector V;
            if (!PackedPoint((*Points)[J], Packed.Mid(J * 48, 48), V))
            { OutReason = FString::Printf(TEXT("Current tensor %d control %d binary64 bits differ."), I, J); return nullptr; }
            Patch.ControlPoints.Add(V);
        }
        Patch.SourceId = Captured.SourceId; Patch.ObjectType = Captured.ObjectType;
        Patch.BlockingChannels = Captured.BlockingChannels; Patch.bQueryCollisionEnabled = Captured.bQueryCollisionEnabled;
        Patch.bAuthorityEligible = false;
        Candidate->TensorBezierPatches.Add(MoveTemp(Patch));
    }
    int32 Vertices = 0;
    for (int32 I = 0; I < CurrentPlanes; ++I)
    {
        const auto G = ObjectAt(*Planes, I), R = ObjectAt(*IdentityPlanes, I), B = ObjectAt(*BoundPlanes, I);
        FSpeedAnalyticBoundedPlaneRecord Plane;
        int32 IdentityVertices = 0, PreservedVertices = 0; double Half[2]{}; const FValues *Domain = nullptr, *HexDomain = nullptr;
        if (!FaceMetadata(R, G, B, true))
        { OutReason = TEXT("Current finite plane metadata identity differs."); return nullptr; }
        const auto Fields = Child(B, TEXT("field_binary64_be_hex"));
        if (!Fields.IsValid() || !VectorField(G, Fields, TEXT("origin_cm"), Plane.Origin) ||
            !VectorField(G, Fields, TEXT("normal"), Plane.Normal) || !VectorField(G, Fields, TEXT("axis_u"), Plane.AxisU) ||
            !VectorField(G, Fields, TEXT("axis_v"), Plane.AxisV) ||
            !Coordinates(G->TryGetField(TEXT("half_extents_cm")), Fields->TryGetField(TEXT("half_extents_cm")), 2, Half) ||
            !Int(R, TEXT("domain_vertex_count"), IdentityVertices, 3, 76) ||
            !Array(G, TEXT("domain_vertices_cm"), Domain, IdentityVertices) ||
            !Array(B, TEXT("domain_binary64_be_hex"), HexDomain, IdentityVertices) ||
            !Int(B, TEXT("vertex_count_preserved"), PreservedVertices, 3, 76) || IdentityVertices != PreservedVertices ||
            !DyadicArray(B, TEXT("world_vertex_delta_component_upper_dyadic"), 3) ||
            !Dyadic(B->TryGetField(TEXT("world_vertex_delta_l1_upper_dyadic"))) ||
            !Dyadic(B->TryGetField(TEXT("normal_delta_l1_upper_dyadic"))))
        { OutReason = FString::Printf(TEXT("Current plane %d frame/domain/bounds differ."), I); return nullptr; }
        if (!Ids.Face(G, R, B, Plane.SurfaceId, Plane.FeatureId, Plane.PrimitiveId, nullptr, OutReason)) return nullptr;
        Plane.HalfExtents = FVector2D(Half[0], Half[1]);
        for (int32 J = 0; J < Domain->Num(); ++J)
        {
            double V[2]{};
            if (!Coordinates((*Domain)[J], (*HexDomain)[J], 2, V))
            { OutReason = FString::Printf(TEXT("Current plane %d vertex %d bits differ."), I, J); return nullptr; }
            Plane.DomainVertices.Emplace(V[0], V[1]);
        }
        Vertices += Domain->Num(); Plane.SourceId = Captured.SourceId; Plane.ObjectType = Captured.ObjectType;
        Plane.BlockingChannels = Captured.BlockingChannels; Plane.bQueryCollisionEnabled = Captured.bQueryCollisionEnabled;
        Plane.bAuthorityEligible = false; Candidate->BoundedPlanes.Add(MoveTemp(Plane));
    }
    if (Vertices != CurrentDomainVertices) { OutReason = TEXT("Current finite domain vertex total differs."); return nullptr; }
    Candidate->TensorBezierPatches.Sort([](const auto& A, const auto& B) { return A.PrimitiveId < B.PrimitiveId; });
    Candidate->BoundedPlanes.Sort([](const auto& A, const auto& B) { return A.PrimitiveId < B.PrimitiveId; });
    Candidate->SourceHash = Speed::Analytic::StableStringId(TEXT("AnalyticStadiumCurrent/v1|") + Ids.Prefix + TEXT("|") +
        IdentityPin.Sha256 + TEXT("|") + NumericPin.Sha256 + TEXT("|") + ConversionPin.Sha256 + TEXT("|") + SourceKey(Captured));
    if (!Candidate->ValidateGeneratedData(&OutReason, false)) return nullptr;
    FSpeedAnalyticAuthoredSourceRecord AtEnd;
    if (!CaptureCurrent(Source, Contact, AtEnd, OutReason) || !SameCapture(AtEnd, Captured))
    { if (OutReason.IsEmpty()) OutReason = TEXT("Actual source/contact changed during current staging."); return nullptr; }
    const auto Manifest = MakeShared<FJsonObject>();
    Manifest->SetStringField(TEXT("schema"), TEXT("skyleague.authored-current-stadium-staging/v1"));
    Manifest->SetStringField(TEXT("identity_manifest_sha256"), IdentityPin.Sha256);
    Manifest->SetStringField(TEXT("exact_source_recipe_sha256"), ExactSourceSha);
    Manifest->SetNumberField(TEXT("exact_source_recipe_bytes"), SourceBytes);
    Manifest->SetStringField(TEXT("geometry_sha256"), NumericPin.Sha256);
    Manifest->SetStringField(TEXT("conversion_bounds_sha256"), ConversionPin.Sha256);
    Manifest->SetStringField(TEXT("source_identity_and_policy"), SourceKey(Captured));
    Manifest->SetStringField(TEXT("declared_source_id_hex"), FString::Printf(TEXT("%016llx"), static_cast<unsigned long long>(Captured.SourceId)));
    Manifest->SetStringField(TEXT("source_hash"), FString::Printf(TEXT("%016llx"), static_cast<unsigned long long>(Candidate->SourceHash)));
    Manifest->SetStringField(TEXT("world"), Source->GetWorld()->GetPathName()); Manifest->SetStringField(TEXT("owner"), Source->GetOwner()->GetPathName());
    Manifest->SetStringField(TEXT("identity_namespace"), Ids.Prefix); Manifest->SetArrayField(TEXT("faces"), Ids.Rows);
    Manifest->SetNumberField(TEXT("finite_domain_vertices"), Vertices);
    Manifest->SetBoolField(TEXT("authority_eligible"), false); Manifest->SetBoolField(TEXT("source_bound"), false);
    Manifest->SetBoolField(TEXT("saved"), false); Manifest->SetBoolField(TEXT("runtime_world_built"), false);
    Manifest->SetStringField(TEXT("numeric_physical_seams_and_evaluator_qualification"), TEXT("NOT_ESTABLISHED"));
    if (!FJsonSerializer::Serialize(Manifest, TJsonWriterFactory<>::Create(&OutManifestJson)))
    { OutManifestJson.Reset(); OutReason = TEXT("Could not serialize current staging manifest."); return nullptr; }
    OutReason = TEXT("Transient current diagnostic payload; no source binding, save, runtime or authority admission.");
    return Candidate.Get();
}

void USpeedAnalyticAuthoredImport::PersistStadiumAnalyticCurrentV1(USpeedAnalyticSourceComponent* Source,
    UPrimitiveComponent* Contact, const FString& ExpectedProjectDirectory, const FString& PackageRoot,
    bool& OutSuccess, USpeedAnalyticCollisionAsset*& OutAnalyticAsset, FString& OutManifestJson, FString& OutReason)
{
    using namespace SpeedAuthoredImport;
    using namespace SpeedAuthoredCurrent;
    OutSuccess = false; OutAnalyticAsset = nullptr; OutManifestJson.Reset(); OutReason.Reset();
    if (!IsInGameThread() || ExpectedProjectDirectory.IsEmpty() || !FPaths::IsSamePath(
        FPaths::ConvertRelativePathToFull(ExpectedProjectDirectory), FPaths::ConvertRelativePathToFull(FPaths::ProjectDir())))
    { OutReason = TEXT("Current analytic save requires the game thread and the explicitly selected private project."); return; }
    const FString PackageName = PackageRoot / TEXT("DA_StadiumC2");
    FString Filename, StagingManifest;
    if (!FreshStadiumPackage(PackageName, Filename, OutReason)) return;
    TStrongObjectPtr<USpeedAnalyticCollisionAsset> Candidate(StageStadiumCurrentV1(Source, Contact, StagingManifest, OutReason));
    if (!Candidate.IsValid()) return;
    FSpeedAnalyticAuthoredSourceRecord Current;
    if (Candidate->AuthoredSources.Num() != 1 || !CaptureCurrent(Source, Contact, Current, OutReason) ||
        !SameCapture(Current, Candidate->AuthoredSources[0]))
    { if (OutReason.IsEmpty()) OutReason = TEXT("Current source/contact changed before persistence."); return; }
    if (!FreshStadiumPackage(PackageName, Filename, OutReason)) return;
    TStrongObjectPtr<USpeedAnalyticCollisionAsset> Saved(PersistentCopy(Candidate.Get(), PackageName));
    bool bSaveAttempted = false, bSaved = false;
    auto Receipt = [&]()
    {
        const auto Json = MakeShared<FJsonObject>();
        Json->SetStringField(TEXT("schema"), TEXT("skyleague.stadium-persistent-current-analytic/v1"));
        Json->SetStringField(TEXT("package"), PackageName);
        Json->SetStringField(TEXT("analytic_asset"), Saved.IsValid() ? Saved->GetPathName() : FString());
        Json->SetStringField(TEXT("analytic_staging_manifest"), StagingManifest);
        Json->SetBoolField(TEXT("package_loaded"), FindPackage(nullptr, *PackageName) != nullptr);
        Json->SetBoolField(TEXT("save_attempted"), bSaveAttempted); Json->SetBoolField(TEXT("analytic_saved"), bSaved);
        Json->SetBoolField(TEXT("file_exists_after_attempt"), IFileManager::Get().FileExists(*Filename));
        Json->SetBoolField(TEXT("render_modified"), false); Json->SetBoolField(TEXT("blueprint_modified"), false);
        Json->SetBoolField(TEXT("physical_authority_promoted"), false); Json->SetBoolField(TEXT("cold_load_verified"), false);
        Json->SetStringField(TEXT("reason"), OutReason);
        return FJsonSerializer::Serialize(Json, TJsonWriterFactory<>::Create(&OutManifestJson));
    };
    if (!Saved.IsValid() || Saved->AuthoredSources.Num() != 1 || !Saved->ValidateGeneratedData(&OutReason, false))
    { if (OutReason.IsEmpty()) OutReason = TEXT("Current persistent copy failed; preserve any loaded package."); Receipt(); return; }
    if (!CaptureCurrent(Source, Contact, Current, OutReason) || !SameCapture(Current, Saved->AuthoredSources[0]))
    { if (OutReason.IsEmpty()) OutReason = TEXT("Source/contact changed after current asset duplication."); Receipt(); return; }
    bSaveAttempted = true; bSaved = SaveFreshStadiumAsset(Saved.Get(), Filename, OutReason);
    if (bSaved) OutReason = TEXT("Current analytic asset saved; no Blueprint/render changes or authority admission. Cold read and binding are separate.");
    if (!Receipt()) { OutReason = TEXT("Current save receipt serialization failed; inspect the preserved package/output before recovery."); return; }
    if (!bSaved) return;
    OutAnalyticAsset = Saved.Get(); OutSuccess = true;
}

#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSpeedCurrentImportDyadicTest,
    "IAmSpeed.AnalyticImport.Current.DyadicContract", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FSpeedCurrentImportDyadicTest::RunTest(const FString&)
{
    using namespace SpeedAuthoredCurrent;
    uint64 M = 0;
    TestTrue(TEXT("uint64 maximum is exact"), StrictUint64(TEXT("18446744073709551615"), M));
    TestEqual(TEXT("no double conversion at uint64 maximum"), M, uint64(MAX_uint64));
    TestTrue(TEXT("integer beyond binary64 exact range"), StrictUint64(TEXT("9007199254740993"), M));
    TestEqual(TEXT("above2^53 retains low bit"), M, uint64(9007199254740993ull));
    for (const TCHAR* Text : { TEXT(""), TEXT("18446744073709551616"), TEXT("-1"), TEXT("+1"), TEXT("1e3"),
        TEXT("1x"), TEXT(" 1"), TEXT("1 "), TEXT("01") }) TestFalse(TEXT("malformed/overflow decimal refused"), StrictUint64(Text, M));
    auto Pair = [](const FString& Text)
    {
        SpeedAuthoredImport::FObject Parsed;
        FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(TEXT("{\"v\":") + Text + TEXT("}")), Parsed);
        return Parsed.IsValid() ? Parsed->TryGetField(TEXT("v")) : nullptr;
    };
    TestTrue(TEXT("canonical zero"), Dyadic(Pair(TEXT("[\"0\",0]"))));
    TestTrue(TEXT("uint64 dyadic remains encoded"), Dyadic(Pair(TEXT("[\"18446744073709551615\",4096]"))));
    for (const TCHAR* Text : { TEXT("[\"0\",1]"), TEXT("[1,0]"), TEXT("[\"1\",\"0\"]"),
        TEXT("[\"1\",0.5]"), TEXT("[\"1\",4097]"), TEXT("[\"1\",-4097]"), TEXT("[\"1\"]"),
        TEXT("[\"\\u0031\\u0000\",0]"), TEXT("[\"\\u0661\",0]") })
        TestFalse(TEXT("wrong dyadic arity/type/canonicality refused"), Dyadic(Pair(Text)));
    return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSpeedCurrentImportPackedTest,
    "IAmSpeed.AnalyticImport.Current.PackedBinary64", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FSpeedCurrentImportPackedTest::RunTest(const FString&)
{
    using namespace SpeedAuthoredImport;
    using namespace SpeedAuthoredCurrent;
    FObject Parsed;
    if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(TEXT("{\"v\":[0.1,3788.8,420.5946669226287]}")), Parsed)) return false;
    const auto Point = Parsed->TryGetField(TEXT("v")); FVector Out;
    const FString Hex = TEXT("3fb999999999999a40ad99999999999a407a4983c1768b40");
    TestTrue(TEXT("known independent decimal/hex oracle"), PackedPoint(Point, Hex, Out));
    TestFalse(TEXT("one bit mutation"), PackedPoint(Point, TEXT("3fb999999999999b40ad99999999999a407a4983c1768b40"), Out));
    TestFalse(TEXT("XYZ permutation"), PackedPoint(Point, Hex.Mid(16,16) + Hex.Left(16) + Hex.Mid(32), Out));
    TestFalse(TEXT("truncation"), PackedPoint(Point, Hex.Left(47), Out));
    TestFalse(TEXT("suffix"), PackedPoint(Point, Hex + TEXT("0"), Out));
    TestFalse(TEXT("non-hex"), PackedPoint(Point, TEXT("g") + Hex.Mid(1), Out));
    const FValues Zero{ MakeShared<FJsonValueNumber>(-0.0), MakeShared<FJsonValueNumber>(0.0), MakeShared<FJsonValueNumber>(0.0) };
    TestFalse(TEXT("signed zero bits are preserved"), PackedPoint(MakeShared<FJsonValueArray>(Zero), FString::ChrN(48,TEXT('0')), Out));
    return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSpeedCurrentImportIdentityTest,
    "IAmSpeed.AnalyticImport.Current.StableIdentityNamespace", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FSpeedCurrentImportIdentityTest::RunTest(const FString&)
{
    using namespace SpeedAuthoredImport;
    using namespace SpeedAuthoredCurrent;
    // Deliberately synthetic declaration, not the actual mission SourceId or a live actor.
    const uint64 SourceId = 9007199254740993ull;
    const FString Legacy = FString::Printf(TEXT("AnalyticStadiumImport/v1|%016llx|C2_ASSEMBLED_STADIUM_OCTAGONAL_V2"),
        static_cast<unsigned long long>(SourceId));
    TestEqual(TEXT("new recipe does not rename retained namespace"), IdentityPrefix(SourceId), Legacy);
    TestNotEqual(TEXT("adjacent source IDs above2^53 stay distinct"), IdentityPrefix(SourceId), IdentityPrefix(SourceId+1));
    for (const TCHAR* Role : { TEXT("surface"), TEXT("feature"), TEXT("primitive"), TEXT("group") })
    {
        const FString Face = TEXT("C2_OUTER_HIGH_BODY_V1/XPYP/side/FloorContinuation");
        TestEqual(TEXT("retained role hashes match legacy"),
            Speed::Analytic::StableStringId(IdentityPrefix(SourceId) + TEXT("|") + Face + TEXT("|") + Role),
            Speed::Analytic::StableStringId(Legacy + TEXT("|") + Face + TEXT("|") + Role));
    }
    FString Manifest = TEXT("stale"), Reason;
    TestNull(TEXT("no implicit current source/contact fallback"), USpeedAnalyticAuthoredImport::StageStadiumCurrentV1(nullptr,nullptr,Manifest,Reason));
    TestTrue(TEXT("stale output cleared"), Manifest.IsEmpty()); TestFalse(TEXT("explicit rejection"), Reason.IsEmpty());
    return !HasAnyErrors();
}
#endif
