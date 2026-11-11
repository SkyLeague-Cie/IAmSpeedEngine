// Included in the existing native pure-math automation test translation unit.
// Synthetic elementary products prove arithmetic contracts only, never a
// gameplay profile, physical pose, owner admission or speed result.
namespace AffineConditionalTestDetail
{
using Product=C::AffineReferenceDetail::FProduct;
inline Product MakeProduct(bool negative=false,bool freeAlongU=true)
{
    Product p;p.FreeWorldAxis=0;p.FreeAlongU=freeAlongU;
    p.Start=I::Exact(negative?1.:-1.);p.End=I::Exact(negative?-1.:1.);
    p.Curve.DegreeU=0;p.Curve.DegreeV=1;
    p.Curve.ControlPoints={C::Detail::Point(FVector3d(0,-1,0)),C::Detail::Point(FVector3d(0,1,0))};
    p.ExactCurveDerivativeHull=C::Detail::Point(FVector3d(0,2,0));return p;
}
inline C::FIntervalTensorSurface Original(const Product& p)
{
    C::FIntervalTensorSurface surface;surface.DegreeU=surface.DegreeV=1;
    for(int32 u=0;u<2;++u)for(int32 v=0;v<2;++v)
    {
        const int32 f=p.FreeAlongU?u:v,c=p.FreeAlongU?v:u;
        const double x=f?p.End.Lower:p.Start.Lower;
        surface.ControlPoints.Add(C::Detail::Point(FVector3d(x,c?1.:-1.,0)));
    }
    return surface;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FExactAffineProjectionLiftTest,
    "IAmSpeed.AnalyticWorld.TensorContact.ExactAffineConditionalProjectionLift",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FExactAffineProjectionLiftTest::RunTest(const FString&)
{
    namespace R=C::AffineReferenceDetail;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE product fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(int32 kind=0;kind<2;++kind)
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        auto product=AffineConditionalTestDetail::MakeProduct(kind!=0,kind==0);
        const auto surface=AffineConditionalTestDetail::Original(product);
        C::FIntervalBezierCenterPath path;
        if(!TestTrue(TEXT("original fixed linear path"),C::CenterDetail::Linear(FVector3d(.25,.2,3),FVector3d(.25,.2,-1),path)))return false;
        C::FDistanceBudget work{100000};R::FProjection projection;
        if(!TestTrue(TEXT("complete fixed product projection"),R::ProjectCompleteCandidateUnion(MakeArrayView(&product,1),path,work,projection)==R::EOutcome::Complete))return false;
        TestEqual(TEXT("degree unchanged by coordinate projection"),projection.Path.Degree,path.Degree);
        TestEqual(TEXT("curve retains original nonfree polynomial degree"),projection.Curves[0].DegreeV,1);
        for(int32 i=0;i<path.ControlPoints.Num();++i)
        {
            TestTrue(TEXT("source free coordinate was not overwritten"),path.ControlPoints[i].Axis[0].Lower==.25&&path.ControlPoints[i].Axis[0].Upper==.25);
            TestTrue(TEXT("only projected free coefficient is exact zero"),projection.Path.ControlPoints[i].Axis[0].Lower==0&&projection.Path.ControlPoints[i].Axis[0].Upper==0);
            for(int32 d=1;d<3;++d)TestTrue(TEXT("other path coefficient bits unchanged"),
                A::Bits(projection.Path.ControlPoints[i].Axis[d].Lower)==A::Bits(path.ControlPoints[i].Axis[d].Lower)&&
                A::Bits(projection.Path.ControlPoints[i].Axis[d].Upper)==A::Bits(path.ControlPoints[i].Axis[d].Upper));
        }
        C::FTensorSphereContactEnclosure lifted;int32 calls=0;
        const auto check=[&](const C::FTensorSphereContactEnclosure& reduced)
        {
            ++calls;
            const auto status=R::LiftFixedMasterContact(MakeArrayView(&product,1),MakeArrayView(&surface,1),path,reduced,
                .00025,.001,work,lifted);
            return status==R::EOutcome::Complete?C::EContactEnclosureDecision::Accept:
                status==R::EOutcome::NeedsRefinement?C::EContactEnclosureDecision::Refine:C::EContactEnclosureDecision::Refuse;
        };
        const C::FContactEnclosureFilter filter(check);
        const auto reduced=C::BoundTensorSphereFirstContact(MakeArrayView(projection.Curves),projection.Path,1.,work,
            .00001,0.,8192,.001,.001,24,&filter);
        if(!TestTrue(TEXT("actual reduced kernel and lifted fixed source contact accepted"),calls>0&&
            reduced.Status==C::ETensorSphereContactStatus::FirstContactEnclosed&&lifted.Status==C::ETensorSphereContactStatus::FirstContactEnclosed))return false;
        TestTrue(TEXT("known first time enclosed"),lifted.LowerTime<=.5&&lifted.UpperTime>=.5);
        TestTrue(TEXT("world free point restored"),lifted.SurfacePoint.Axis[0].Lower<=.25&&lifted.SurfacePoint.Axis[0].Upper>=.25);
        TestTrue(TEXT("fixed product radial free component exactly zero"),lifted.RadialUnitNormal.Axis[0].Lower==0&&lifted.RadialUnitNormal.Axis[0].Upper==0);
        for(const auto& leaf:lifted.Leaves)
        {
            C::FReferenceDyadicCell u,v;C::FIntervalPoint chart;
            TestTrue(TEXT("original interior dyadic cells retained"),M::DecodeLeafDyadic(leaf.MinU,leaf.MaxU,u)&&M::DecodeLeafDyadic(leaf.MinV,leaf.MaxV,v)&&
                leaf.MinU>0&&leaf.MaxU<1&&leaf.MinV>0&&leaf.MaxV<1);
            TestTrue(TEXT("source rank conclusion encloses positive local orientation"),R::ProveFixedProductLeafRank(product,u,v,work,chart)&&chart.Axis[2].Lower>0);
            const double freeParameter=kind?.375:.625,curveParameter=.6;
            const double lo=product.FreeAlongU?leaf.MinU:leaf.MinV,hi=product.FreeAlongU?leaf.MaxU:leaf.MaxV;
            TestTrue(TEXT("every emitted closed axial cell intersects covered parameter"),lo<=freeParameter&&hi>=freeParameter);
            const double curveLo=product.FreeAlongU?leaf.MinV:leaf.MinU,curveHi=product.FreeAlongU?leaf.MaxV:leaf.MaxU;
            TestTrue(TEXT("curve parameter mapping is not swapped twice"),curveLo<=curveParameter&&curveHi>=curveParameter);
        }
        auto broad=reduced;broad.LowerTime=0;broad.UpperTime=1;
        C::FIntervalBezierCenterPath wide;
        TestTrue(TEXT("same projected path may hide broad original free motion"),C::CenterDetail::Linear(FVector3d(-.5,.2,3),FVector3d(.5,.2,-1),wide));
        C::FTensorSphereContactEnclosure refused;
        TestTrue(TEXT("wide lifted global point asks time refinement before consumer"),
            R::LiftFixedMasterContact(MakeArrayView(&product,1),MakeArrayView(&surface,1),wide,broad,.00025,.001,work,refused)==R::EOutcome::NeedsRefinement&&refused.Leaves.IsEmpty());
    }
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FExactAffineDyadicCoverageTest,
    "IAmSpeed.AnalyticWorld.TensorContact.ExactAffineDyadicCoverageAndBudget",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FExactAffineDyadicCoverageTest::RunTest(const FString&)
{
    namespace R=C::AffineReferenceDetail;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE cover fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        auto p=AffineConditionalTestDetail::MakeProduct();p.Start=I::Exact(0);p.End=I::Exact(1);
        for(double parameter:{.25,.5,.9})
        {
            C::FDistanceBudget work{1000};TArray<C::FReferenceDyadicCell> cells;
            TestTrue(TEXT("constant parameter finds deeper interior cover without time refinement"),R::CoverFreeParameter(p,I::Exact(parameter),1.,work,cells)==R::EOutcome::Complete&&!cells.IsEmpty());
            for(const auto& cell:cells){const auto range=cell.GetRange();TestTrue(TEXT("closed strict interior parameter cover"),range.Lower>0&&range.Upper<1&&range.Lower<=parameter&&range.Upper>=parameter);}
        }
        const I across{.49999,.50001,true};TArray<C::FReferenceDyadicCell> cells;
        C::FDistanceBudget work{1000};
        TestTrue(TEXT("cross-grid interval gets complete closed coverage"),R::CoverFreeParameter(p,across,.001,work,cells)==R::EOutcome::Complete&&cells.Num()==2&&cells[0].GetRange().Lower<=across.Lower&&cells.Last().GetRange().Upper>=across.Upper);
        C::FDistanceBudget exhausted{11};
        TestTrue(TEXT("failed second cell publishes no partial cover or refund"),R::CoverFreeParameter(p,across,.001,exhausted,cells)==R::EOutcome::Unknown&&cells.IsEmpty()&&exhausted.Used==11&&exhausted.Remaining==0);
        C::FIntervalPoint center=C::Detail::Point(FVector3d(.25,0,0));I parameter;
        p.Start=I::Exact(1);p.End=I::Exact(-1);
        TestTrue(TEXT("negative slope quotient includes exact original parameter"),R::FreeParameter(p,center,parameter)&&parameter.Lower<=.375&&parameter.Upper>=.375);
    }
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FExactAffineScopeAndRoundingTest,
    "IAmSpeed.AnalyticWorld.TensorContact.ExactAffineCompleteScopeAndNumericRefusal",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FExactAffineScopeAndRoundingTest::RunTest(const FString&)
{
    namespace R=C::AffineReferenceDetail;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE scope fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    auto p=AffineConditionalTestDetail::MakeProduct();C::FIntervalBezierCenterPath path;
    TestTrue(TEXT("boundary-crossing original path built"),C::CenterDetail::Linear(FVector3d(-2,0,3),FVector3d(0,0,-1),path));
    C::FDistanceBudget work{1000};R::FProjection projection;
    TestTrue(TEXT("fullpath outside free interval is not an applicable source reduction"),R::ProjectCompleteCandidateUnion(MakeArrayView(&p,1),path,work,projection)==R::EOutcome::NotApplicable&&projection.Curves.IsEmpty());
    TestTrue(TEXT("non-RNE test context installed"),std::fesetround(FE_UPWARD)==0);
    bool parameterRefused=false,coverRefused=false,liftRefused=false;
    try{I v;R::FreeParameter(p,C::Detail::Point(FVector3d(0,0,0)),v);}catch(const Speed::Analytic::FAnalyticNumericUnknown&){parameterRefused=true;}
    try{TArray<C::FReferenceDyadicCell> cells;R::CoverFreeParameter(p,I::Exact(.5),.001,work,cells);}catch(const Speed::Analytic::FAnalyticNumericUnknown&){coverRefused=true;}
    try{C::FTensorSphereContactEnclosure r,o;auto s=AffineConditionalTestDetail::Original(p);R::LiftFixedMasterContact(MakeArrayView(&p,1),MakeArrayView(&s,1),path,r,.001,.001,work,o);}catch(const Speed::Analytic::FAnalyticNumericUnknown&){liftRefused=true;}
    TestTrue(TEXT("each public conditional enclosure helper refuses unsupported rounding"),parameterRefused&&coverRefused&&liftRefused);
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}


IMPLEMENT_SIMPLE_AUTOMATION_TEST(FExactAffineStrictDominanceTest,
    "IAmSpeed.AnalyticWorld.TensorContact.ExactAffineStrictDominanceAndScope",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FExactAffineStrictDominanceTest::RunTest(const FString&)
{
    namespace R=C::AffineReferenceDetail;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE dominance fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 controls=VectorGetControlRegister();
    const FString group=TEXT("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})for(int32 reversed=0;reversed<2;++reversed)
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        auto left=AffineConditionalTestDetail::MakeProduct();left.Start=I::Exact(-2);left.End=I::Exact(0);
        auto right=AffineConditionalTestDetail::MakeProduct();right.Start=I::Exact(2);right.End=I::Exact(0);
        if(reversed)
        {
            Swap(right.Curve.ControlPoints[0],right.Curve.ControlPoints[1]);
            right.ExactCurveDerivativeHull=C::Detail::Point(FVector3d(0,-2,0));
        }
        TArray<R::FProduct> products{left,right};TArray<FString> ids{group,group};
        C::FIntervalBezierCenterPath path;
        TestTrue(TEXT("full path stays in original left chart"),C::CenterDetail::Linear(FVector3d(-.5,.2,3),FVector3d(-.25,.2,-1),path));
        C::FDistanceBudget work{10000};TArray<int32> kept,representatives;
        TestTrue(TEXT("strictly separated exact same/reversed sibling is dominated"),R::SelectStrictDominantProductCover(MakeArrayView(products),MakeArrayView(ids),path,work,kept,representatives)==R::EOutcome::Complete&&kept.Num()==1&&kept[0]==0&&representatives.Num()==2&&representatives[0]==0&&representatives[1]==0);
        TestTrue(TEXT("original source tensors were not overwritten"),products[0].Start.Lower==-2&&products[1].Start.Lower==2&&products[1].Curve.ControlPoints[0].Axis[1].Lower==(reversed?1.:-1.));
        auto missing=ids;missing[1].Reset();
        TestTrue(TEXT("box equality alone never authenticates omitted source"),R::SelectStrictDominantProductCover(MakeArrayView(products),MakeArrayView(missing),path,work,kept,representatives)==R::EOutcome::NotApplicable&&kept.IsEmpty()&&representatives.IsEmpty());
        auto malformed=products;malformed[1].Curve.ControlPoints[0].Axis[1]=I::Exact(2);
        TestTrue(TEXT("same ID with different bound curve is refused before search"),R::SelectStrictDominantProductCover(MakeArrayView(malformed),MakeArrayView(ids),path,work,kept,representatives)==R::EOutcome::NotApplicable&&kept.IsEmpty());
        C::FIntervalBezierCenterPath seam,cross;
        C::CenterDetail::Linear(FVector3d(0,.2,3),FVector3d(0,.2,-1),seam);
        C::CenterDetail::Linear(FVector3d(-.5,.2,3),FVector3d(.5,.2,-1),cross);
        TestTrue(TEXT("exact original seam keeps generic path"),R::SelectStrictDominantProductCover(MakeArrayView(products),MakeArrayView(ids),seam,work,kept,representatives)==R::EOutcome::NotApplicable&&kept.IsEmpty());
        TestTrue(TEXT("whole-path crossing cannot borrow one interior instant"),R::SelectStrictDominantProductCover(MakeArrayView(products),MakeArrayView(ids),cross,work,kept,representatives)==R::EOutcome::NotApplicable&&kept.IsEmpty());
        TArray<R::FProduct> overlap{left,left};
        TestTrue(TEXT("all interior owners retained despite same curve"),R::SelectStrictDominantProductCover(MakeArrayView(overlap),MakeArrayView(ids),path,work,kept,representatives)==R::EOutcome::Complete&&kept.Num()==2&&representatives[0]==0&&representatives[1]==1);
        C::FDistanceBudget exhausted{2};
        TestTrue(TEXT("partial consumed work is Unknown without cover/refund"),R::SelectStrictDominantProductCover(MakeArrayView(products),MakeArrayView(ids),path,exhausted,kept,representatives)==R::EOutcome::Unknown&&kept.IsEmpty()&&representatives.IsEmpty()&&exhausted.Used==2&&exhausted.Remaining==0);
    }
    C::FIntervalBezierCenterPath numericPath;
    TestTrue(TEXT("numeric refusal path built before rounding change"),C::CenterDetail::Linear(FVector3d(0,0,3),FVector3d(0,0,-1),numericPath));
    TestTrue(TEXT("non-RNE dominance context installed"),std::fesetround(FE_UPWARD)==0);
    bool refused=false;
    try
    {
        auto p=AffineConditionalTestDetail::MakeProduct();TArray<R::FProduct> products{p};TArray<FString> ids{group};TArray<int32> a,b;
        C::FDistanceBudget budget{1000};
        R::SelectStrictDominantProductCover(MakeArrayView(products),MakeArrayView(ids),numericPath,budget,a,b);
    }
    catch(const Speed::Analytic::FAnalyticNumericUnknown&){refused=true;}
    TestTrue(TEXT("unsupported numeric context refused"),refused);
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}


namespace AffineConditionalTestDetail
{
// Structurally valid synthetic transport, never an authenticated stadium or
// gameplay pose. AuthorityEligible remains false for every owner.
class FSubsetProducer:public Speed::Analytic::Prepared::FReferenceTransportProducer
{
public:
    static bool Build(C::FDistanceBudget& budget,Speed::Analytic::Prepared::FReferenceTransportPtr& out,bool changedGroup=false)
    {
        namespace P=Speed::Analytic::Prepared;
        TArray<Product> products;
        for(int32 i=0;i<5;++i)
        {
            auto p=MakeProduct();p.Start=I::Exact(i==0?-20.:(i==2?2.:-2.));p.End=I::Exact(i==0?-18.:0.);
            if(i==3)for(auto& cp:p.Curve.ControlPoints)cp.Axis[2]=I::Exact(20);
            products.Add(p);
        }
        TArray<C::FIntervalTensorSurface> surfaces;
        for(const auto& p:products)surfaces.Add(Original(p));
        // Original() is the elementary z=0 fixture. This distant independent
        // provider must be absent from the actual candidate set.
        for(auto& cp:surfaces[3].ControlPoints)cp.Axis[2]=I::Exact(20);
        C::FIndexedIntervalUnionPtr net;
        if(!C::FIndexedIntervalUnion::Build(surfaces,budget,net))return false;
        TArray<P::FOwner> owners;
        for(int32 i=0;i<5;++i)
        {
            P::FOwner owner;owner.SourceId=1;owner.SurfaceId=uint64(100+i);owner.FeatureId=uint64(200+i);owner.PrimitiveId=uint64(300+i);
            owner.QueryEnabled=true;owner.CompactOptIn=true;owner.BlockingChannels=1;owners.Add(owner);
        }
        FSpeedAnalyticAuthoredSourceRecord source;source.Kind=ESpeedAnalyticSourceKind::AuthoredAnalytic;source.SourceId=1;
        source.RecipeSchema=TEXT("unit.structure.only");source.RecipeSha256=FString::ChrN(64,TCHAR('1'));source.PayloadSha256=FString::ChrN(64,TCHAR('2'));
        source.SourceComponentPath=TEXT("Source");source.ContactComponentPath=TEXT("Contact");source.CollisionEnabled=uint8(ECollisionEnabled::QueryOnly);
        source.bQueryCollisionEnabled=true;source.BlockingChannels=1;
        P::FScenePtr scene;FString reason;
        if(!P::FScene::Build(net,owners,MakeArrayView(&source,1),{},1,budget,scene,reason))return false;
        P::FReferenceTransportData data;data.CanonicalUnion=net;data.NominalScene=scene;data.Packet={1};
        data.Model=TEXT("unit.structure");data.ExternalPacketIdentity=TEXT("unit.not.authenticated");data.ProofIdentities={TEXT("unit.structure.not.certificate")};
        data.ExactAffineProofIdentity=TEXT("unit.synthetic.fixed.products");
        for(int32 i=0;i<5;++i)
        {
            P::FReferenceFace face;face.OpaqueId=FString::Printf(TEXT("unit.face.%d"),i);face.CanonicalRecordIdentity=FString::Printf(TEXT("unit.net.%d"),i);
            face.BoundRecordIdentity=FString::Printf(TEXT("unit.bound.%d"),i);face.DomainAndChart={TEXT("u0"),TEXT("u1"),TEXT("v0"),TEXT("v1")};
            data.Faces.Add(face);data.ProviderFaceIndices.Add(i);data.SphereResponseContracts.Add(P::FReferenceSphereResponseContract::RadialReacquisitionV1(i));
            P::FReferenceAffineExtrusionDescription fact;fact.Provider=fact.SourceFace=i;fact.OriginalDegreeU=fact.OriginalDegreeV=1;
            fact.SourceFaceIdentity=face.OpaqueId;fact.CanonicalRecordIdentity=face.CanonicalRecordIdentity;fact.BoundRecordIdentity=face.BoundRecordIdentity;
            fact.CorrelationProofIdentity=data.ExactAffineProofIdentity;fact.ExactProjectedCurveIdentity=FString::ChrN(64,TCHAR(i==3||(changedGroup&&i==2)?'b':'a'));
            fact.OriginalDomains=face.DomainAndChart;fact.Product=products[i];data.ExactAffineExtrusions.Add(fact);
        }
        return SealReference(MoveTemp(data),budget,out,reason);
    }
};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FExactAffineSubsetMappingTest,
    "IAmSpeed.AnalyticWorld.TensorContact.ExactAffineDominantSubsetMapping",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FExactAffineSubsetMappingTest::RunTest(const FString&)
{
    namespace P=Speed::Analytic::Prepared;namespace R=C::AffineReferenceDetail;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE subset fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        C::FDistanceBudget work{1000000};P::FReferenceTransportPtr reference,changed;
        if(!TestTrue(TEXT("complete synthetic original transport sealed"),AffineConditionalTestDetail::FSubsetProducer::Build(work,reference)&&AffineConditionalTestDetail::FSubsetProducer::Build(work,changed,true)))return false;
        TestFalse(TEXT("curve identity mutation fails full transport description equality"),reference->SameDescription(*changed,work));
        C::FIntervalBezierCenterPath path;
        if(!TestTrue(TEXT("subset fixed path"),C::CenterDetail::Linear(FVector3d(-.25,.2,3),FVector3d(-.25,.2,-1),path)))return false;
        C::FReferenceCandidateSetPtr candidates;
        if(!TestTrue(TEXT("actual complete candidate selection"),C::FReferenceCandidateSet::Select(reference->GetCanonicalUnion(),path,1.,work,candidates)))return false;
        const auto originalIndices=candidates->GetCandidateIndices();
        TestTrue(TEXT("noncontiguous original globals include dominated middle member"),originalIndices.Num()==3&&originalIndices.Contains(1)&&originalIndices.Contains(2)&&originalIndices.Contains(4));
        P::FReferenceAffineContactPlanPtr plan;
        if(!TestTrue(TEXT("conditional plan uses strict dominance"),P::FReferenceAffineContactPlan::Build(reference,candidates,work,plan)==R::EOutcome::Complete&&plan&&plan->GetDominatedProviderCountForInspection()==1))return false;
        TestTrue(TEXT("diagnostic provider map is the retained subset"),plan->MapRetainedProviderForInspection(0)==1&&plan->MapRetainedProviderForInspection(1)==4&&plan->MapRetainedProviderForInspection(2)==INDEX_NONE);
        bool checked=false;
        const auto consume=[&](const C::FTensorSphereContactEnclosure& bounds)
        {
            bool first=false,last=false;
            for(const auto& leaf:bounds.Leaves)
            {
                if(leaf.SurfaceIndex!=1&&leaf.SurfaceIndex!=4)return C::EContactEnclosureDecision::Refuse;
                if(!(leaf.MinU>0&&leaf.MaxU<1&&leaf.MinV>0&&leaf.MaxV<1))return C::EContactEnclosureDecision::Refuse;
                C::FIntervalPoint normal;if(!plan->ProveFixedMasterLeafRank(leaf,work,normal))return C::EContactEnclosureDecision::Refuse;
                first|=leaf.SurfaceIndex==1;last|=leaf.SurfaceIndex==4;
            }
            checked=first&&last;return checked?C::EContactEnclosureDecision::Accept:C::EContactEnclosureDecision::Refuse;
        };
        const C::FContactEnclosureFilter filter(consume);
        const auto fixed=plan->Contact(work,.00001,0.,8192,.001,.001,filter);
        const auto* bounds=fixed?fixed->InspectForSameFixedSource(reference,candidates):nullptr;
        TestTrue(TEXT("same fixed source contact lifted and remapped exactly once"),checked&&bounds&&bounds->Status==C::ETensorSphereContactStatus::FirstContactEnclosed&&bounds->LowerTime<=.5&&bounds->UpperTime>=.5);
        const auto repeated=plan->Contact(work,.00001,0.,8192,.001,.001,filter);
        const auto* second=repeated?repeated->InspectForSameFixedSource(reference,candidates):nullptr;
        TestTrue(TEXT("consumed plan cannot restart"),second&&second->Status==C::ETensorSphereContactStatus::Unknown);
    }
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}


namespace AffineSubset86TestDetail
{
// Complete synthetic source; provider1 deliberately has NO affine fact.
class FProducer final:public Speed::Analytic::Prepared::FReferenceTransportProducer
{
public:
    static bool Build(C::FDistanceBudget& budget,Speed::Analytic::Prepared::FReferenceTransportPtr& out,int32 kind=0)
    {
        namespace P=Speed::Analytic::Prepared;
        const auto product=AffineConditionalTestDetail::MakeProduct();
        TArray<C::FIntervalTensorSurface> surfaces{AffineConditionalTestDetail::Original(product)};
        C::FIntervalTensorSurface blocker;blocker.DegreeU=0;blocker.DegreeV=kind==0?2:1;
        if(kind==0)blocker.ControlPoints={C::Detail::Point(FVector3d(0,-3,0)),C::Detail::Point(FVector3d(0,0,20)),C::Detail::Point(FVector3d(0,3,0))};
        else {const double x=kind==2?1.25:0.;blocker.ControlPoints={C::Detail::Point(FVector3d(x,-1,0)),C::Detail::Point(FVector3d(x,1,0))};}
        surfaces.Add(blocker);C::FIndexedIntervalUnionPtr net;
        if(!C::FIndexedIntervalUnion::Build(surfaces,budget,net))return false;
        TArray<P::FOwner> owners;
        for(int32 i=0;i<2;++i){P::FOwner owner;owner.SourceId=1;owner.SurfaceId=100+i;owner.FeatureId=200+i;owner.PrimitiveId=300+i;owner.QueryEnabled=true;owner.CompactOptIn=true;owner.BlockingChannels=1;owners.Add(owner);}
        FSpeedAnalyticAuthoredSourceRecord source;source.Kind=ESpeedAnalyticSourceKind::AuthoredAnalytic;source.SourceId=1;
        source.RecipeSchema=TEXT("unit.subset86.only");source.RecipeSha256=FString::ChrN(64,TCHAR('1'));source.PayloadSha256=FString::ChrN(64,TCHAR('2'));
        source.SourceComponentPath=TEXT("Source");source.ContactComponentPath=TEXT("Contact");source.CollisionEnabled=uint8(ECollisionEnabled::QueryOnly);source.bQueryCollisionEnabled=true;source.BlockingChannels=1;
        P::FScenePtr scene;FString reason;if(!P::FScene::Build(net,owners,MakeArrayView(&source,1),{},1,budget,scene,reason))return false;
        P::FReferenceTransportData data;data.CanonicalUnion=net;data.NominalScene=scene;data.Packet={86};data.Model=TEXT("unit.structure");data.ExternalPacketIdentity=TEXT("unit.not.authenticated");data.ProofIdentities={TEXT("unit.not.certificate")};data.ExactAffineProofIdentity=TEXT("unit.synthetic.subset86");
        for(int32 i=0;i<2;++i)
        {
            P::FReferenceFace face;face.OpaqueId=FString::Printf(TEXT("face.%d"),i);face.CanonicalRecordIdentity=FString::Printf(TEXT("net.%d"),i);face.BoundRecordIdentity=FString::Printf(TEXT("bound.%d"),i);face.DomainAndChart={TEXT("u0"),TEXT("u1"),TEXT("v0"),TEXT("v1")};
            data.Faces.Add(face);data.ProviderFaceIndices.Add(i);data.SphereResponseContracts.Add(P::FReferenceSphereResponseContract::RadialReacquisitionV1(i));
            if(i)continue;
            P::FReferenceAffineExtrusionDescription fact;fact.Provider=fact.SourceFace=0;fact.OriginalDegreeU=fact.OriginalDegreeV=1;fact.SourceFaceIdentity=face.OpaqueId;fact.CanonicalRecordIdentity=face.CanonicalRecordIdentity;fact.BoundRecordIdentity=face.BoundRecordIdentity;fact.CorrelationProofIdentity=data.ExactAffineProofIdentity;fact.ExactProjectedCurveIdentity=FString::ChrN(64,TCHAR('a'));fact.OriginalDomains=face.DomainAndChart;fact.Product=product;data.ExactAffineExtrusions.Add(fact);
        }
        return SealReference(MoveTemp(data),budget,out,reason);
    }
};
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FExactAffineSeparatedBlockerTest,
    "IAmSpeed.AnalyticWorld.TensorContact.ExactAffineSeparatedBlockerCoverage",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FExactAffineSeparatedBlockerTest::RunTest(const FString&)
{
    namespace P=Speed::Analytic::Prepared;namespace E=C::AffineReferenceDetail;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE subset86 fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);C::FDistanceBudget work{1000000};P::FReferenceTransportPtr reference;
        if(!TestTrue(TEXT("synthetic blocker transport"),AffineSubset86TestDetail::FProducer::Build(work,reference)))return false;
        C::FIntervalBezierCenterPath path;C::CenterDetail::Linear(FVector3d(.25,.2,3),FVector3d(.25,.2,-1),path);C::FReferenceCandidateSetPtr candidates;
        if(!TestTrue(TEXT("both original providers selected by hull"),C::FReferenceCandidateSet::Select(reference->GetCanonicalUnion(),path,1.,work,candidates)&&candidates->GetCandidateIndices().Num()==2))return false;
        const uint64 before=work.Used;const uint32 flags=VectorGetControlRegister();const int exceptions=std::fetestexcept(FE_ALL_EXCEPT);
        P::FReferenceAffineBuildObservation86 observation;P::FReferenceAffineContactPlanPtr plan;
        if(!TestTrue(TEXT("only certified separated blocker permits subset"),P::FReferenceAffineContactPlan::Build(reference,candidates,work,plan,&observation)==E::EOutcome::Complete&&plan))return false;
        TestTrue(TEXT("proof costs preserved and separate from Contact"),work.Used>before&&observation.SubsetProofWork>0&&observation.BlockerTests==1&&observation.BlockerSeparated==1&&observation.SubsetPlans==1);
        TestTrue(TEXT("separated is never counted as dominated"),plan->GetStrictlySeparatedProviderCountForInspection86()==1&&plan->GetDominatedProviderCountForInspection()==0&&plan->MapRetainedProviderForInspection(0)==0&&plan->MapRetainedProviderForInspection(1)==INDEX_NONE);
        TestTrue(TEXT("new optional proof restores all floating state"),flags==VectorGetControlRegister()&&exceptions==std::fetestexcept(FE_ALL_EXCEPT));
        bool checked=false;const auto consume=[&](const C::FTensorSphereContactEnclosure& bounds)
        {
            for(const auto& leaf:bounds.Leaves){C::FIntervalPoint normal;if(leaf.SurfaceIndex!=0||!plan->ProveFixedMasterLeafRank(leaf,work,normal))return C::EContactEnclosureDecision::Refuse;}
            checked=!bounds.Leaves.IsEmpty();return checked?C::EContactEnclosureDecision::Accept:C::EContactEnclosureDecision::Refuse;
        };
        const C::FContactEnclosureFilter filter(consume);const auto fixed=plan->Contact(work,.00001,0.,8192,.001,.001,filter);
        const auto* bounds=fixed?fixed->InspectForSameFixedSource(reference,candidates):nullptr;
        TestTrue(TEXT("same original candidate seal and one global mapping"),checked&&bounds&&bounds->Status==C::ETensorSphereContactStatus::FirstContactEnclosed&&bounds->LowerTime<=.5&&bounds->UpperTime>=.5);
        const auto repeated=plan->Contact(work,.00001,0.,8192,.001,.001,filter);const auto* second=repeated?repeated->InspectForSameFixedSource(reference,candidates):nullptr;
        TestTrue(TEXT("subset attempt cannot restart"),second&&second->Status==C::ETensorSphereContactStatus::Unknown);
    }
    TestTrue(TEXT("fixture restore"),restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FExactAffineSeparatedBlockerRefusalTest,
    "IAmSpeed.AnalyticWorld.TensorContact.ExactAffineSeparatedBlockerRefusalAndBudget",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FExactAffineSeparatedBlockerRefusalTest::RunTest(const FString&)
{
    namespace P=Speed::Analytic::Prepared;namespace E=C::AffineReferenceDetail;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE refusal fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    for(int32 kind=0;kind<3;++kind)
    {
        C::FDistanceBudget setup{1000000};P::FReferenceTransportPtr reference;
        if(!TestTrue(TEXT("refusal source"),AffineSubset86TestDetail::FProducer::Build(setup,reference,kind)))return false;
        C::FIntervalBezierCenterPath path;C::CenterDetail::Linear(FVector3d(.25,.2,3),FVector3d(.25,.2,-1),path);C::FReferenceCandidateSetPtr candidates;
        if(!TestTrue(TEXT("all possible competitors kept"),C::FReferenceCandidateSet::Select(reference->GetCanonicalUnion(),path,1.,setup,candidates)&&candidates->GetCandidateIndices().Num()==2))return false;
        C::FDistanceBudget work{100000};P::FReferenceAffineBuildObservation86 observation;P::FReferenceAffineContactPlanPtr plan;
        const auto status=P::FReferenceAffineContactPlan::Build(reference,candidates,work,plan,&observation,kind==0?1:8192);
        TestTrue(TEXT("unproved/real/tangent blocker never dropped"),status==E::EOutcome::NotApplicable&&!plan&&observation.BlockerUnresolved==1&&observation.SubsetPlans==0&&work.Used>0);
        C::FDistanceBudget exhausted{4};P::FReferenceAffineBuildObservation86 limited;
        TestTrue(TEXT("actual exhausted work is terminal Unknown without partial plan/refund"),P::FReferenceAffineContactPlan::Build(reference,candidates,exhausted,plan,&limited)==E::EOutcome::Unknown&&!plan&&exhausted.Used==4&&exhausted.Remaining==0&&limited.SubsetAttempts==1&&limited.SubsetUnknown==1);
    }
    TestTrue(TEXT("fixture restore"),restore.Restore());return !HasAnyErrors();
}
