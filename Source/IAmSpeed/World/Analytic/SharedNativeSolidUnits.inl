namespace
{
FAutoConsoleCommand SharedNativeSolidUnits(TEXT("Speed.Analytic.RunSharedNativeSolidUnits"),
 TEXT("Run the six added/extended shared query/solid Source units; no World or physical authority."),
 FConsoleCommandDelegate::CreateLambda([]()
 {
    static bool used=false;auto& framework=FAutomationTestFramework::Get();
    if(!IsRunningCommandlet()||!FParse::Param(FCommandLine::Get(),TEXT("AnalyticSharedNativeSolidUnits"))||used||
        GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||framework.GetCurrentTest()||!framework.IsLatentCommandQueueEmpty())
    {UE_LOG(LogTemp,Error,TEXT("Shared native solid units refused"));return;}
    used=true;
    const TCHAR* classes[]={TEXT("FIAmSpeedCanonicalFinitePlaneGeometryTest"),TEXT("FAnalyticFixedQueryBoundsTest"),
        TEXT("FPreparedPhysicalExecutionTest"),TEXT("FFloorNativeSolidSweepBoundsTest"),TEXT("FPreparedNativeAnalyticContactValuesTest"),TEXT("FInitialFloorSphereGeometryTest")};
    const TCHAR* names[]={TEXT("IAmSpeed.AnalyticWorld.CanonicalFinitePlane.GeometryShapesAndRefusals"),
        TEXT("IAmSpeed.AnalyticWorld.CompleteUnion.FixedRaySphereBoxBounds"),
        TEXT("IAmSpeed.AnalyticWorld.Prepared.NativeExecutionAndSphereCaptureRefusals"),
        TEXT("IAmSpeed.AnalyticWorld.NativeSolid.FloorArrivalAndWholePrefix"),
        TEXT("IAmSpeed.AnalyticWorld.NativeSolid.ContactTokenAndExactValues"),
        TEXT("IAmSpeed.AnalyticWorld.NativeSolid.InitialCapturedSphereSupportAndSkin")};
    for(int32 i=0;i<UE_ARRAY_COUNT(classes);++i)
    {
        if(GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||framework.GetCurrentTest()||
            !framework.IsLatentCommandQueueEmpty()||!framework.ContainsTest(classes[i]))
        {UE_LOG(LogTemp,Error,TEXT("Shared native solid unit ownership refused"));return;}
        framework.SetRequestedTestFilter(EAutomationTestFlags::EngineFilter);framework.StartTestByName(classes[i],0,names[i]);
        if(!GIsAutomationTesting||!framework.GetCurrentTest()||framework.GetCurrentTestFullPath()!=names[i]||!framework.IsLatentCommandQueueEmpty())
        {UE_LOG(LogTemp,Error,TEXT("Shared native solid synchronous ownership refused"));return;}
        FAutomationTestExecutionInfo info;const bool stopped=framework.StopTest(info);
        if(GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||framework.GetCurrentTest()||!framework.IsLatentCommandQueueEmpty())
        {UE_LOG(LogTemp,Error,TEXT("Shared native solid final ownership refused"));return;}
        const bool passed=stopped&&info.bSuccessful&&info.GetErrorTotal()==0&&info.GetWarningTotal()==0;
        UE_LOG(LogTemp,Display,TEXT("ANALYTIC_SHARED_SOLID_UNIT name=%s passed=%d errors=%d warnings=%d"),
            names[i],passed,info.GetErrorTotal(),info.GetWarningTotal());
    }
 }));
}
