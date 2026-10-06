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
    Write-Output 'PASS exact engine pin, whole-checkout private-root exclusion, bounded/private build arguments, queue/report rejection, and strict automation pass'
}
finally {
    $resolvedFixture = [IO.Path]::GetFullPath($FixtureRoot).TrimEnd('\')
    if (-not $resolvedFixture.StartsWith($TempBase + '\', [StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing to remove fixture outside TEMP: $resolvedFixture"
    }
    Remove-Item -LiteralPath $resolvedFixture -Recurse -Force
}
