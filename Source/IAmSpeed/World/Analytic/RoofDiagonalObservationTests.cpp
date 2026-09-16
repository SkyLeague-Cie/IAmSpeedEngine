#include "AnalyticWorldQuery.h"

#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include <limits>

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedRoofDiagonalObservationTest,
    "IAmSpeed.AnalyticWorld.RoofDiagonalDerivedObservation",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedRoofDiagonalObservationTest::RunTest(const FString& Parameters)
{
    using namespace Speed::Analytic;
    // Observation only: rounded frame16/attempt102, NOT a bit-exact replay.
    // Capture receipt 04DEACE90EE501BBAE8EC5CA01E3D2067730EB5FC6E6A6861BE12751259CCB33.
    // Inventory D8784985F7A03033712A02FD87817C193DFEF94C9AF865F7132AB0385DF4A83D.
    // A single frozen provider and three mathematical reflections, not the
    // complete stadium or the four original vehicle trajectories. Unlogged
    // query flags below are explicit assumptions, never reconstructed facts.
    AddInfo(TEXT("[RoofDiagonalObservationScope] DerivedRoundedQuery=1 ExactReplay=0 "
        "FullWorld=0 RuntimeMutation=0 MirrorKind=mathematical_reflection "
        "FeatureKind=0:Unknown,1:Face,2:Edge,3:Vertex"));
    FExtrudedQuinticPatch Base;
    Base.SourceId = 0x6B096F692F3206D0ull;
    Base.SurfaceId = 0xD1F2DF807735DBD9ull;
    Base.FeatureId = 0x09C354C9ADDBA88Cull;
    Base.PrimitiveId = 0xD5C7B9B6573745B2ull;
    // Isolated derived provider, no canonical certificate. The captured group
    // 529547B4BF944050 is provenance only, not certification of this fixture.
    Base.CanonicalGroupId = 0;
    Base.ObjectType = 0;
    Base.BlockingChannels = 0xFFFFFEEFull;
    Base.bQueryCollisionEnabled = true;
    Base.SectionControlPoints[0] = FVector3d(3998.050891337107,-3998.051677522742,-0.0035156249068677425);
    Base.SectionControlPoints[1] = FVector3d(4108.262069761709,-4108.262877619516,-0.0035156249068677425);
    Base.SectionControlPoints[2] = FVector3d(4218.473248186312,-4218.4740777162915,-0.0035156249068677425);
    Base.SectionControlPoints[3] = FVector3d(4367.95850864448,-4367.959367569575,230.24849607066363);
    Base.SectionControlPoints[4] = FVector3d(4367.958472241777,-4367.959331166863,370.515780060177);
    Base.SectionControlPoints[5] = FVector3d(4367.958435839072,-4367.959294764152,510.7830640496904);
    Base.ExtrusionAxis = FVector3d(-0.7069229312894242,-0.707290563667015,-0.0001666276214726403);
    Base.MinimumExtrusionCoordinate = -1791.512980255973;
    Base.MaximumExtrusionCoordinate = 381.3725302467989;

    FWorldQuery Rounded;
    Rounded.Shape = EQueryShape::Box;
    Rounded.Start = FVector3d(4838.789,-3841.683,433.509);
    Rounded.End = FVector3d(4838.785,-3841.685,434.865);
    const FQuat4d PrintedRotation(.733653459,-.303795703,.561608315,.232501339);
    Rounded.Rotation = PrintedRotation.GetNormalized();
    Rounded.HalfExtent = FVector3d(78.790,52.300,19.140);
    Rounded.bIncludeCompactPatches = true;
    Rounded.bUseFiniteContactDomain = true;
    Rounded.DomainTolerance = 0.;
    Rounded.InitialOverlapTolerance = .05;
    AddInfo(TEXT("[RoofDiagonalQueryAssumptions] FiniteDomain=1 DomainTolerance=0 "
        "InitialOverlapTolerance=.05 ApplyCollisionFilter=0 AuthorityOnly=0 "
        "SourceSurfaceGroupRestriction=0"));

    // Rounding of each position/extent coordinate <=.0005cm, each quaternion
    // component <=.5e-9, assuming nearest decimal formatting and a unit input.
    // Normalization is Lipschitz bounded by 2*epsilon/(1-epsilon). This bounds
    // input OBB vertices only; feature identity and TOI need not be continuous.
    const double PositionBound = FMath::Sqrt(3.)*.0005;
    const double ExtentBound = PositionBound;
    const double QuatBound = 2.e-9/(1.-1.e-9);
    const double VertexBound = PositionBound+ExtentBound+
        2.*QuatBound*(Rounded.HalfExtent.Length()+ExtentBound);
    AddInfo(FString::Printf(TEXT("[RoofDiagonalQuantization] PositionBoundCm=%.17g "
        "ExtentBoundCm=%.17g NormalizedQuatBound=%.17g VertexBoundCm=%.17g "
        "MotionBoundCm=%.17g FeatureOrTOIBound=unavailable"),
        PositionBound,ExtentBound,QuatBound,VertexBound,2.*PositionBound));

    const FVector3d Reflections[] = {FVector3d(1,1,1),FVector3d(-1,-1,1),
        FVector3d(-1,1,1),FVector3d(1,-1,1)};
    for (int32 Mirror=0; Mirror<UE_ARRAY_COUNT(Reflections); ++Mirror)
    {
        const FVector3d M=Reflections[Mirror];
        FAnalyticWorldData World;
        FExtrudedQuinticPatch P=Base;
        for (auto& Point:P.SectionControlPoints) Point*=M;
        P.ExtrusionAxis*=M;
        // FinalizeAndValidate checks the finite bounds before rebuilding its
        // query approximation. Zero corrections leave this control hull valid.
        P.Bounds=FBox3d(EForceInit::ForceInit);
        for (const auto& Point:P.SectionControlPoints)
        {
            P.Bounds+=Point+P.MinimumExtrusionCoordinate*P.ExtrusionAxis;
            P.Bounds+=Point+P.MaximumExtrusionCoordinate*P.ExtrusionAxis;
        }
        // Isolated test world, no fabricated source/authority certification.
        World.ExtrudedQuinticPatches.Add(P);
        FString Reason;
        if (!TestTrue(TEXT("Frozen reflected provider validates"),World.FinalizeAndValidate(&Reason)))
        { AddError(Reason); return false; }
        const auto& Patch=World.ExtrudedQuinticPatches[0];
        FWorldQuery Q=Rounded;
        Q.Start*=M; Q.End*=M;
        Q.Rotation=FQuat4d(M.Y*Rounded.Rotation.X,M.X*Rounded.Rotation.Y,
            M.X*M.Y*Rounded.Rotation.Z,Rounded.Rotation.W);
        // Keep this historical observation on the implicated RAW facet. The
        // joined provider may now correctly reject its internal-edge wall.
        const int32 Segment=Patch.SectionPolyline.Num()-2;
        FBoundedPlane NativeFacet;
        NativeFacet.SourceId=Patch.SourceId; NativeFacet.SurfaceId=Patch.SurfaceId;
        NativeFacet.FeatureId=Patch.FeatureId;
        NativeFacet.AxisU=(Patch.SectionPolyline[Segment+1]-Patch.SectionPolyline[Segment]).GetSafeNormal();
        NativeFacet.AxisV=Patch.ExtrusionAxis;
        NativeFacet.Normal=FVector3d::CrossProduct(NativeFacet.AxisU,NativeFacet.AxisV).GetSafeNormal();
        NativeFacet.Origin=.5*(Patch.SectionPolyline[Segment]+Patch.SectionPolyline[Segment+1])+
            .5*(Patch.MinimumExtrusionCoordinate+Patch.MaximumExtrusionCoordinate)*Patch.ExtrusionAxis;
        NativeFacet.HalfExtents=FVector2d(.5*(Patch.SectionPolyline[Segment+1]-Patch.SectionPolyline[Segment]).Length(),
            .5*(Patch.MaximumExtrusionCoordinate-Patch.MinimumExtrusionCoordinate));
        FWorldHit Hit=FWorldQueryService::SweepPlane(Q,NativeFacet);
        Hit.PrimitiveId=CombineStableIds(Patch.PrimitiveId,uint64(Segment+1));
        if (!Hit.bHit)
        {
            AddError(FString::Printf(TEXT("[RoofDiagonalUnavailable] Mirror=%d DerivedQueryMiss=1; do not infer f16 feature"),Mirror));
            continue;
        }
        const FVector3d A=Patch.SectionPolyline[Segment];
        const FVector3d B=Patch.SectionPolyline[Segment+1];
        const FVector3d Chord=B-A;
        const double Alpha=FVector3d::DotProduct(Hit.Point-A,Chord)/Chord.SquaredLength();
        // The authored axis is not exactly orthogonal to the section chord.
        // Use the dual basis for geometric boundary coordinates; retain the
        // native projected alpha separately instead of confusing the two.
        const double CC=Chord.SquaredLength(), AA=Patch.ExtrusionAxis.SquaredLength();
        const double CA=FVector3d::DotProduct(Chord,Patch.ExtrusionAxis);
        const double DC=FVector3d::DotProduct(Hit.Point-A,Chord);
        const double DA=FVector3d::DotProduct(Hit.Point-A,Patch.ExtrusionAxis);
        const double Determinant=CC*AA-CA*CA;
        if (!TestTrue(TEXT("Facet dual basis is nonsingular"),Determinant>0.)) continue;
        const double GeometricAlpha=(DC*AA-DA*CA)/Determinant;
        const double E=(DA*CC-DC*CA)/Determinant;
        const double T=FMath::Lerp(Patch.SectionParameters[Segment],Patch.SectionParameters[Segment+1],FMath::Clamp(Alpha,0.,1.));
        FVector3d DerivativeNormal=FVector3d::CrossProduct(Patch.EvaluateSectionDerivative(T),Patch.ExtrusionAxis).GetSafeNormal();
        if (FVector3d::DotProduct(DerivativeNormal,Hit.Normal)<0) DerivativeNormal=-DerivativeNormal;
        FBoundedPlane Face;
        Face.SourceId=Patch.SourceId; Face.SurfaceId=Patch.SurfaceId; Face.FeatureId=Patch.FeatureId;
        Face.ObjectType=Patch.ObjectType; Face.BlockingChannels=Patch.BlockingChannels;
        Face.bQueryCollisionEnabled=true;
        Face.AxisU=Chord.GetSafeNormal();
        Face.Normal=FVector3d::CrossProduct(Face.AxisU,Patch.ExtrusionAxis).GetSafeNormal();
        Face.Origin=.5*(A+B)+.5*(Patch.MinimumExtrusionCoordinate+Patch.MaximumExtrusionCoordinate)*Patch.ExtrusionAxis;
        if (FVector3d::DotProduct(Q.Start-Face.Origin,Face.Normal)<0) Face.Normal=-Face.Normal;
        Face.AxisV=FVector3d::CrossProduct(Face.Normal,Face.AxisU).GetSafeNormal();
        const FVector3d Vertices[]={A+Patch.MinimumExtrusionCoordinate*Patch.ExtrusionAxis,
            B+Patch.MinimumExtrusionCoordinate*Patch.ExtrusionAxis,
            B+Patch.MaximumExtrusionCoordinate*Patch.ExtrusionAxis,
            A+Patch.MaximumExtrusionCoordinate*Patch.ExtrusionAxis};
        FVector3d Reconstructed[4];
        double ReconstructionError=0., CoordinateScale=1.;
        for (int32 V=0; V<4; ++V)
        {
            const FVector3d Relative=Vertices[V]-Face.Origin;
            const FVector2d Local(FVector3d::DotProduct(Relative,Face.AxisU),FVector3d::DotProduct(Relative,Face.AxisV));
            Face.DomainVertices.Add(Local);
            Face.HalfExtents.X=FMath::Max(Face.HalfExtents.X,FMath::Abs(Local.X));
            Face.HalfExtents.Y=FMath::Max(Face.HalfExtents.Y,FMath::Abs(Local.Y));
            Face.Bounds+=Vertices[V];
            Reconstructed[V]=Face.Origin+Local.X*Face.AxisU+Local.Y*Face.AxisV;
            ReconstructionError=FMath::Max(ReconstructionError,(Reconstructed[V]-Vertices[V]).Length());
            CoordinateScale=FMath::Max(CoordinateScale,Vertices[V].Length());
            AddInfo(FString::Printf(TEXT("[RoofDiagonalFacetVertex] Mirror=%d Index=%d "
                "World=(%.17g,%.17g,%.17g) Reconstructed=(%.17g,%.17g,%.17g)"),
                Mirror,V,Vertices[V].X,Vertices[V].Y,Vertices[V].Z,
                Reconstructed[V].X,Reconstructed[V].Y,Reconstructed[V].Z));
        }
        FVector3d Closure=FVector3d::ZeroVector;
        for (int32 V=0; V<4; ++V) Closure+=Reconstructed[(V+1)%4]-Reconstructed[V];
        const double RoundoffBound=64.*std::numeric_limits<double>::epsilon()*CoordinateScale;
        AddInfo(FString::Printf(TEXT("[RoofDiagonalFacetEncoding] Mirror=%d UDotV=%.17g "
            "ReconstructionErrorCm=%.17g ClosureErrorCm=%.17g RoundoffBoundCm=%.17g "
            "Domain=orthonormal_four_vertex_parallelogram RuntimeRectangleReplay=0"),
            Mirror,FVector3d::DotProduct(Face.AxisU,Face.AxisV),ReconstructionError,Closure.Length(),RoundoffBound));
        if (!TestTrue(TEXT("Facet vertices reconstruct within floating-point bound"),ReconstructionError<=RoundoffBound) ||
            !TestTrue(TEXT("Facet polygon closes within floating-point bound"),Closure.Length()<=RoundoffBound) ||
            !TestTrue(TEXT("Orthonormal finite plane validates"),Face.IsValid(&Reason)))
        { AddError(Reason); continue; }
        // Test-only friend observes the unchanged finite-plane kernel on the
        // TRUE finite parallelogram, not the runtime's rectangular encoding.
        // This independent derived query is not injected into runtime selection.
        FWorldQuery FaceQuery=Q;
        FaceQuery.bAuthorityOnly=false; FaceQuery.RequiredCanonicalGroupId=0;
        const FWorldHit Raw=FWorldQueryService::SweepPlane(FaceQuery,Face);
        const FExtrudedFacetEdgeObservation EdgeObservation=
            FWorldQueryService::SweepPlaneWithEdgeObservationForTest(FaceQuery,Face);
        const auto SameHit=[](const FWorldHit& A,const FWorldHit& B)
        {
            return A.bHit==B.bHit && A.bStartPenetrating==B.bStartPenetrating &&
                A.Time==B.Time && A.PenetrationDepth==B.PenetrationDepth &&
                A.GeometricErrorBoundCm==B.GeometricErrorBoundCm &&
                A.AdditionalResidualAgreementAllowanceCm==B.AdditionalResidualAgreementAllowanceCm &&
                A.bSurfaceNormalMayVary==B.bSurfaceNormalMayVary &&
                A.Location==B.Location && A.Point==B.Point && A.QueryPoint==B.QueryPoint &&
                A.Normal==B.Normal && A.QueryFeatureKind==B.QueryFeatureKind &&
                A.SurfaceFeatureKind==B.SurfaceFeatureKind &&
                A.QueryFeatureIndex==B.QueryFeatureIndex && A.SurfaceFeatureIndex==B.SurfaceFeatureIndex &&
                A.SourceId==B.SourceId && A.SurfaceId==B.SurfaceId && A.FeatureId==B.FeatureId &&
                A.ProviderId==B.ProviderId && A.PrimitiveId==B.PrimitiveId &&
                A.CanonicalGroupId==B.CanonicalGroupId && A.MaterialId==B.MaterialId;
        };
        if (!TestTrue(TEXT("Edge observation preserves every public hit field"),SameHit(Raw,EdgeObservation.Hit)))
            continue;
        const EExtrudedFacetEdgeRole EdgeRole=EdgeObservation.bHasTriangleEdgeTags
            ? FWorldQueryService::ClassifyExtrudedFacetEdgeForTest(
                Patch.SectionPolyline.Num()-1,Segment,EdgeObservation.CornerA,EdgeObservation.CornerB)
            : EExtrudedFacetEdgeRole::Unknown;
        TestTrue(TEXT("Last-facet rounded witness is its internal section beginning"),
            EdgeRole==EExtrudedFacetEdgeRole::InternalSectionStart);
        AddInfo(FString::Printf(TEXT("[RoofDiagonalEdgeObservation] Mirror=%d HasTags=%d CornerA=%d CornerB=%d "
            "Role=%u Segment=%d SegmentCount=%d Classification=topology_only"),
            Mirror,EdgeObservation.bHasTriangleEdgeTags,EdgeObservation.CornerA,EdgeObservation.CornerB,
            uint8(EdgeRole),Segment,Patch.SectionPolyline.Num()-1));
        AddInfo(FString::Printf(TEXT("[RoofDiagonalDerivedHit] Mirror=%d Primitive=%016llX "
            "FacetOrdinal=%d FacetCount=%d SurfaceKind=%u SurfaceIndex=%d QueryKind=%u QueryIndex=%d "
            "NativeProjectedAlpha=%.17g GeometricAlpha=%.17g T=%.17g TOI=%.17g Depth=%.17g StartPenetrating=%d "
            "Point=(%.17g,%.17g,%.17g) SelectedNormal=(%.17g,%.17g,%.17g) "
            "DerivativeNormal=(%.17g,%.17g,%.17g) PreviousNeighbor=%d NextNeighbor=%d "
            "DistanceToChordStartCm=%.17g DistanceToChordEndCm=%.17g "
            "DistanceToExtrusionMinCm=%.17g DistanceToExtrusionMaxCm=%.17g"),
            Mirror,Hit.PrimitiveId,Segment+1,Patch.SectionPolyline.Num()-1,
            uint8(Hit.SurfaceFeatureKind),int32(Hit.SurfaceFeatureIndex),uint8(Hit.QueryFeatureKind),int32(Hit.QueryFeatureIndex),
            Alpha,GeometricAlpha,T,Hit.Time,Hit.PenetrationDepth,Hit.bStartPenetrating,
            Hit.Point.X,Hit.Point.Y,Hit.Point.Z,Hit.Normal.X,Hit.Normal.Y,Hit.Normal.Z,
            DerivativeNormal.X,DerivativeNormal.Y,DerivativeNormal.Z,Segment>0,Segment+2<Patch.SectionPolyline.Num(),
            GeometricAlpha*Chord.Length(),(1.-GeometricAlpha)*Chord.Length(),E-Patch.MinimumExtrusionCoordinate,Patch.MaximumExtrusionCoordinate-E));
        AddInfo(FString::Printf(TEXT("[RoofDiagonalRawFacet] Mirror=%d Hit=%d SurfaceKind=%u SurfaceIndex=%d "
            "QueryKind=%u QueryIndex=%d TOI=%.17g Depth=%.17g Normal=(%.17g,%.17g,%.17g) "
            "Point=(%.17g,%.17g,%.17g) Classification=uninterpreted_no_f16_causal_claim"),
            Mirror,Raw.bHit,uint8(Raw.SurfaceFeatureKind),int32(Raw.SurfaceFeatureIndex),
            uint8(Raw.QueryFeatureKind),int32(Raw.QueryFeatureIndex),Raw.Time,Raw.PenetrationDepth,
            Raw.Normal.X,Raw.Normal.Y,Raw.Normal.Z,Raw.Point.X,Raw.Point.Y,Raw.Point.Z));
    }
    return true;
}
#endif
