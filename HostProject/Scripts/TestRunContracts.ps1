$ErrorActionPreference = 'Stop'
$ScriptsRoot = $PSScriptRoot
$HelperPath = Join-Path $ScriptsRoot 'RunContracts.Helpers.ps1'
$RunnerPath = Join-Path $ScriptsRoot 'RunContracts.ps1'
. $HelperPath

function Assert-True {
    param([bool]$Condition, [string]$Message)
    if (-not $Condition) { throw "ASSERT: $Message" }
}

function Assert-Throws {
    param([scriptblock]$Action, [string]$Message)
    try { & $Action }
    catch { return }
    throw "ASSERT: expected failure: $Message"
}

$TempBase = (Resolve-Path -LiteralPath $env:TEMP).Path.TrimEnd('\')
$FixtureRoot = Join-Path $TempBase ('IAmSpeedRunContracts-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $FixtureRoot | Out-Null
try {
    $EngineFixture = Join-Path $FixtureRoot 'Engine'
    $BuildPath = Join-Path $EngineFixture 'Engine\Build\BatchFiles\Build.bat'
    $EditorPath = Join-Path $EngineFixture 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
    New-Item -ItemType Directory -Path (Split-Path -Parent $BuildPath) -Force | Out-Null
    New-Item -ItemType Directory -Path (Split-Path -Parent $EditorPath) -Force | Out-Null
    [IO.File]::WriteAllText((Join-Path $EngineFixture 'Engine\Build\Build.version'), (@{
        MajorVersion=5; MinorVersion=8; PatchVersion=2; CompatibleChangelist=55116800
    } | ConvertTo-Json))
    [IO.File]::WriteAllText($BuildPath, '@echo off')
    [IO.File]::WriteAllText($EditorPath, '')

    Assert-Throws { Assert-IAmSpeedEngineRoot -EngineRoot '' } 'missing engine root'
    Assert-Throws { Assert-IAmSpeedEngineRoot -EngineRoot (Join-Path $FixtureRoot 'Missing') } 'missing engine directory'
    $engine = Assert-IAmSpeedEngineRoot -EngineRoot $EngineFixture
    Assert-True ($engine.Version -ceq '5.8.2' -and $engine.CompatibleChangelist -eq 55116800) 'exact UE 5.8.2 pin is accepted'

    $rulesSeedRoot = $env:IAMSPEED_RULES_ENGINE_ROOT
    if ([string]::IsNullOrWhiteSpace($rulesSeedRoot)) {
        throw 'IAMSPEED_RULES_ENGINE_ROOT is required to validate the real pinned Rules seed returned to the runner.'
    }
    $rulesSeed = Assert-IAmSpeedRulesSeedRoot -EngineRoot $rulesSeedRoot
    $expectedRulesSeedRoot = (Resolve-Path -LiteralPath $rulesSeedRoot).Path.TrimEnd('\')
    $expectedRulesSeedBuild = Join-Path $expectedRulesSeedRoot 'Engine\Build\BatchFiles\Build.bat'
    Assert-True ($rulesSeed.Root -ceq $expectedRulesSeedRoot) 'Rules seed object pins the resolved Engine root'
    Assert-True ($rulesSeed.Build -ceq $expectedRulesSeedBuild) 'Rules seed object returns the Build.bat path used by QueryTargets'
    $rulesSeedBuildCommand = Get-Command -Name $rulesSeed.Build -CommandType Application -ErrorAction Stop
    Assert-True ($rulesSeedBuildCommand.Source -ceq $expectedRulesSeedBuild) 'Rules seed object Build path resolves as an invocable command'

    $PluginFixture = Join-Path $FixtureRoot 'PluginCheckout'
    $PluginProject = Join-Path $PluginFixture 'HostProject'
    New-Item -ItemType Directory -Path $PluginProject -Force | Out-Null
    $privateOutside = Assert-IAmSpeedPrivateRoot -PrivateRoot (Join-Path $FixtureRoot 'Private') `
        -EngineRoot $EngineFixture -PluginRoot $PluginFixture
    Assert-True ($privateOutside -eq (Join-Path $FixtureRoot 'Private')) 'private output outside plugin checkout is accepted'
    Assert-Throws { Assert-IAmSpeedPrivateRoot -PrivateRoot (Join-Path $PluginProject 'Private') `
        -EngineRoot $EngineFixture -PluginRoot $PluginFixture } 'private output anywhere inside plugin checkout is rejected'
    Assert-Throws { Assert-IAmSpeedPrivateRoot -PrivateRoot (Join-Path $PluginFixture 'HostProject\ExternalPlugins\IAmSpeed\Private') `
        -EngineRoot $EngineFixture -PluginRoot $PluginFixture } 'private output under the project plugin-link path is rejected'

    Assert-True ($script:IAmSpeedExpectedTbbFallbackSha256 -ceq 'af20d7ca563e542432b856f6628d9481247197d1853bd4057caaf6c449749d42') 'pinned TBB fallback SHA is exact'
    $TbbBinaryDirectory = Join-Path $EngineFixture 'Engine\Binaries\Win64'
    New-Item -ItemType Directory -Path $TbbBinaryDirectory -Force | Out-Null
    $TbbBinary = Join-Path $TbbBinaryDirectory 'tbbmalloc.dll'
    [IO.File]::WriteAllText($TbbBinary, 'engine-binary-tbb')
    $binaryLoader = Resolve-IAmSpeedTbbLoader -EngineRoot $EngineFixture
    Assert-True ($binaryLoader.Source -ceq 'EngineBinaries' -and $binaryLoader.DllPath -eq $TbbBinary) 'engine binary TBB loader is preferred'

    Remove-Item -LiteralPath $TbbBinary -Force
    $TbbFallback = Join-Path $EngineFixture 'Engine\Source\ThirdParty\Intel\TBB\Deploy\oneTBB-2022.3.0\VS2015\x64\bin\tbbmalloc.dll'
    New-Item -ItemType Directory -Path (Split-Path -Parent $TbbFallback) -Force | Out-Null
    [IO.File]::WriteAllText($TbbFallback, 'fixture-fallback-tbb')
    $fixtureFallbackSha = (Get-FileHash -LiteralPath $TbbFallback -Algorithm SHA256).Hash.ToLowerInvariant()
    $fallbackLoader = Resolve-IAmSpeedTbbLoader -EngineRoot $EngineFixture -ExpectedFallbackSha256 $fixtureFallbackSha
    Assert-True ($fallbackLoader.Source -ceq 'PinnedThirdPartyFallback' -and $fallbackLoader.FallbackShaValidated -and
        $fallbackLoader.Sha256 -ceq $fixtureFallbackSha) 'fallback TBB loader requires and records its qualified SHA'
    Assert-Throws { Resolve-IAmSpeedTbbLoader -EngineRoot $EngineFixture } 'fallback TBB with a nonqualified SHA is rejected'

    $originalPath = [Environment]::GetEnvironmentVariable('PATH', 'Process')
    $pathResult = Invoke-IAmSpeedWithProcessTbbPath -LoaderDirectory $TbbBinaryDirectory -Action {
        [Environment]::GetEnvironmentVariable('PATH', 'Process')
    }
    Assert-True ($pathResult.StartsWith($TbbBinaryDirectory + ';', [StringComparison]::OrdinalIgnoreCase)) 'private TBB directory is process-PATH prefixed'
    Assert-True ([Environment]::GetEnvironmentVariable('PATH', 'Process') -ceq $originalPath) 'process PATH is restored after successful action'
    Assert-Throws { Invoke-IAmSpeedWithProcessTbbPath -LoaderDirectory $TbbBinaryDirectory -Action { throw 'fixture action failure' } } 'TBB action exception is propagated'
    Assert-True ([Environment]::GetEnvironmentVariable('PATH', 'Process') -ceq $originalPath) 'process PATH is restored after failed action'

    $SnapshotFixtureRoot = Join-Path $FixtureRoot 'RulesSnapshotEngine'
    $SnapshotRulesDirectory = Join-Path $SnapshotFixtureRoot 'Engine\Intermediate\Build\BuildRules'
    New-Item -ItemType Directory -Path $SnapshotRulesDirectory -Force | Out-Null
    $SnapshotNames = @('UE5Rules.dll', 'UE5Rules.pdb', 'UE5RulesManifest.json',
        'UE5ProgramRules.dll', 'UE5ProgramRules.pdb', 'UE5ProgramRulesManifest.json')
    foreach ($name in $SnapshotNames) { [IO.File]::WriteAllText((Join-Path $SnapshotRulesDirectory $name), "before-$name") }
    $snapshotBefore = Get-IAmSpeedRulesSnapshot -EngineRoot $SnapshotFixtureRoot
    Assert-True (Assert-IAmSpeedRulesSnapshotUnchanged -Before $snapshotBefore -After (Get-IAmSpeedRulesSnapshot -EngineRoot $SnapshotFixtureRoot) -Label 'fixture') 'unchanged six-file rules snapshot is accepted'
    $changedDll = Join-Path $SnapshotRulesDirectory 'UE5Rules.dll'
    [IO.File]::WriteAllText($changedDll, 'changed-bytes')
    $changedBytesSnapshot = Get-IAmSpeedRulesSnapshot -EngineRoot $SnapshotFixtureRoot
    Assert-Throws { Assert-IAmSpeedRulesSnapshotUnchanged -Before $snapshotBefore -After $changedBytesSnapshot -Label 'fixture bytes' } 'Engine rules byte change is rejected without rebaseline'
    [IO.File]::WriteAllText($changedDll, 'before-UE5Rules.dll')
    $originalDllTime = [DateTime]::new([long]$snapshotBefore.files[0].lastWriteUtcTicks, [DateTimeKind]::Utc)
    (Get-Item -LiteralPath $changedDll).LastWriteTimeUtc = $originalDllTime
    (Get-Item -LiteralPath $changedDll).LastWriteTimeUtc = $originalDllTime.AddSeconds(1)
    $changedTimeSnapshot = Get-IAmSpeedRulesSnapshot -EngineRoot $SnapshotFixtureRoot
    Assert-Throws { Assert-IAmSpeedRulesSnapshotUnchanged -Before $snapshotBefore -After $changedTimeSnapshot -Label 'fixture timestamp' } 'Engine rules timestamp-only change is rejected'
    (Get-Item -LiteralPath $changedDll).LastWriteTimeUtc = $originalDllTime
    Remove-Item -LiteralPath (Join-Path $SnapshotRulesDirectory 'UE5Rules.pdb') -Force
    Assert-Throws { Get-IAmSpeedRulesSnapshot -EngineRoot $SnapshotFixtureRoot } 'missing Engine rules file is rejected'

    $buildArgs = New-IAmSpeedBuildArguments -ProjectFile 'D:\Private\HostProject.uproject' `
        -LogPath 'D:\Private\Logs\build.log' -UbaRoot 'D:\Private\UBA' -MaxParallelActions 4 -UbaMaxWorkers 4
    foreach ($required in @('-WaitMutex', '-NoHotReload', '-UsePrecompiled', '-NoEngineChanges',
            '-NoXGE', '-NoFASTBuild', '-NoSNDBS', '-MaxParallelActions=4',
            '-UBAStoreCapacityGb=4', '-UBAMaxWorkers=4', '-UBADisableRemote',
            '-UBADisableHorde', '-SLPrivateProjectResources',
            '-UBARootDir=D:\Private\UBA')) {
        Assert-True ($buildArgs -ccontains $required) "build args contain $required"
    }
    Assert-True ($buildArgs -cnotcontains '-NoUBA') 'private local UBA remains enabled'
    Assert-True ($buildArgs -cnotcontains '-SkipRulesCompile') 'default build can compile fresh project rules'
    $skipBuildArgs = New-IAmSpeedBuildArguments -ProjectFile 'D:\Private\HostProject.uproject' `
        -LogPath 'D:\Private\Logs\build.log' -UbaRoot 'D:\Private\UBA' -SkipRulesCompile
    Assert-True ($skipBuildArgs -ccontains '-SkipRulesCompile') 'target build can load the already staged project rules without compiling Engine rules'

    $rulesQueryArgs = New-IAmSpeedProjectRulesQueryArguments -ProjectFile 'D:\Private\HostProject\IAmSpeedHostProject.uproject' `
        -OutputPath 'D:\Private\Automation\TargetInfo.json' -LogPath 'D:\Private\Logs\RulesQuery.log'
    Assert-True ($rulesQueryArgs -ccontains '-Mode=QueryTargets') 'rules preparation uses UBT QueryTargets only'
    Assert-True ($rulesQueryArgs -ccontains '-UsePrecompiled') 'rules preparation loads the pinned seed Engine rules'
    Assert-True ($rulesQueryArgs -cnotcontains '-SkipRulesCompile') 'QueryTargets compiles the fresh private project rules'
    Assert-True (@($rulesQueryArgs | Where-Object { $_ -match '^-Output=D:\\Private\\' }).Count -eq 1) 'QueryTargets output is private'

    $editorArgs = New-IAmSpeedEditorArguments -ProjectFile 'D:\Private\HostProject.uproject' `
        -TestFilter 'IAmSpeed.AnalyticWorld' -LogPath 'D:\Private\Logs\automation.log' `
        -ReportPath 'D:\Private\Automation'
    Assert-True ($editorArgs -ccontains '-TestExit=Automation Test Queue Empty') 'Editor waits for queue completion'
    Assert-True ($editorArgs -ccontains '-ExecCmds=Automation RunTests IAmSpeed.AnalyticWorld') 'Editor runs the intended suite'
    Assert-True (@($editorArgs | Where-Object { $_ -match 'Quit' }).Count -eq 0) 'Editor is not told to quit before tests finish'

    $ReportPath = Join-Path $FixtureRoot 'index.json'
    $success = @{
        succeeded=2; succeededWithWarnings=0; failed=0; notRun=0; inProcess=0
        tests=@(
            @{ fullTestPath='IAmSpeed.AnalyticWorld.BoundedPlane'; state='Success' },
            @{ fullTestPath='IAmSpeed.AnalyticWorld.TriangleFaceBvh'; state='Success' }
        )
    }
    [IO.File]::WriteAllText($ReportPath, ($success | ConvertTo-Json -Depth 5))
    Assert-Throws { Assert-IAmSpeedAutomationResult -ExitCode 0 -StandardOutput 'startup only' `
        -ReportPath $ReportPath -TestFilter 'IAmSpeed.AnalyticWorld' } 'missing queue marker'
    Assert-Throws { Assert-IAmSpeedAutomationResult -ExitCode 0 `
        -StandardOutput 'Automation Test Queue Empty 3 tests performed.' `
        -ReportPath $ReportPath -TestFilter 'IAmSpeed.AnalyticWorld' } 'queue/report count mismatch'
    Assert-Throws { Assert-IAmSpeedAutomationResult -ExitCode 1 `
        -StandardOutput 'Automation Test Queue Empty 2 tests performed.' `
        -ReportPath $ReportPath -TestFilter 'IAmSpeed.AnalyticWorld' } 'nonzero Editor exit'

    $success.tests[1].state = 'Fail'
    $success.failed = 1
    $success.succeeded = 1
    [IO.File]::WriteAllText($ReportPath, ($success | ConvertTo-Json -Depth 5))
    Assert-Throws { Assert-IAmSpeedAutomationResult -ExitCode 0 `
        -StandardOutput 'Automation Test Queue Empty 2 tests performed.' `
        -ReportPath $ReportPath -TestFilter 'IAmSpeed.AnalyticWorld' } 'failed test report'

    $success.tests[1].state = 'Success'
    $success.failed = 0
    $success.succeeded = 2
    [IO.File]::WriteAllText($ReportPath, ($success | ConvertTo-Json -Depth 5))
    $pass = Assert-IAmSpeedAutomationResult -ExitCode 0 `
        -StandardOutput 'Automation Test Queue Empty 2 tests performed.' `
        -ReportPath $ReportPath -TestFilter 'IAmSpeed.AnalyticWorld'
    Assert-True ($pass.TestsPerformed -eq 2 -and $pass.TestsSucceeded -eq 2) 'strict complete report is accepted'

    foreach ($path in @($HelperPath, $RunnerPath, $MyInvocation.MyCommand.Path)) {
        $tokens = $null
        $parseErrors = $null
        [System.Management.Automation.Language.Parser]::ParseFile($path, [ref]$tokens, [ref]$parseErrors) | Out-Null
        Assert-True ($parseErrors.Count -eq 0) "PowerShell 5 parser accepts $path"
    }
    Write-Output 'PASS split UBT rules preparation, six-file Engine-rule immutability, process-only TBB, bounded build, strict reports, and PATH restoration'
}
finally {
    $resolvedFixture = [IO.Path]::GetFullPath($FixtureRoot).TrimEnd('\')
    if (-not $resolvedFixture.StartsWith($TempBase + '\', [StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing to remove fixture outside TEMP: $resolvedFixture"
    }
    Remove-Item -LiteralPath $resolvedFixture -Recurse -Force
}
