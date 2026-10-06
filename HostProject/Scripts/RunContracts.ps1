[CmdletBinding()]
param(
    [string]$EngineRoot = $env:IAMSPEED_UE_ROOT,
    [string]$PrivateRoot = $env:IAMSPEED_PRIVATE_ROOT,
    [string]$TestFilter = 'IAmSpeed.AnalyticWorld',
    [ValidateRange(1, 8)] [int]$MaxParallelActions = 4,
    [ValidateRange(1, 8)] [int]$UBAMaxWorkers = 4
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'RunContracts.Helpers.ps1')

$RunRoot = $null
$PluginLink = $null
$EnvironmentNames = @(
    'TEMP', 'TMP', 'LOCALAPPDATA', 'APPDATA', 'PATH',
    'UE-LocalDataCachePath', 'UE-SharedDataCachePath', 'UE_SKIP_UBT_SDK_SETUP'
)
$OriginalEnvironment = @{}
$TbbLoader = $null
foreach ($name in $EnvironmentNames) {
    $OriginalEnvironment[$name] = [Environment]::GetEnvironmentVariable($name, 'Process')
}

try {
    $Engine = Assert-IAmSpeedEngineRoot -EngineRoot $EngineRoot
    if ([string]::IsNullOrWhiteSpace($TestFilter) -or
        $TestFilter -notmatch '^IAmSpeed\.AnalyticWorld(?:\.[A-Za-z0-9_.]+)?$') {
        throw "Unsupported physical-contract test filter: $TestFilter"
    }

    $ProjectRoot = Split-Path -Parent $PSScriptRoot
    $PluginRoot = Split-Path -Parent $ProjectRoot
    $PrivateParent = Assert-IAmSpeedPrivateRoot -PrivateRoot $PrivateRoot -EngineRoot $Engine.Root -PluginRoot $PluginRoot
    $TbbLoader = Resolve-IAmSpeedTbbLoader -EngineRoot $Engine.Root
    if (-not (Test-Path -LiteralPath $PrivateParent -PathType Container)) {
        New-Item -ItemType Directory -Path $PrivateParent -Force | Out-Null
    }
    $PrivateItem = Get-Item -LiteralPath $PrivateParent -Force
    if (($PrivateItem.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0) {
        throw 'Private output root cannot be a reparse point.'
    }

    $RunName = 'IAmSpeed-' + [DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ') + '-' + $PID
    $RunRoot = Join-Path $PrivateParent $RunName
    if (Test-Path -LiteralPath $RunRoot) { throw "Private run root already exists: $RunRoot" }
    New-Item -ItemType Directory -Path $RunRoot | Out-Null

    $PrivateProject = Join-Path $RunRoot 'HostProject'
    New-Item -ItemType Directory -Path $PrivateProject | Out-Null
    Copy-Item -LiteralPath (Join-Path $ProjectRoot 'IAmSpeedHostProject.uproject') -Destination $PrivateProject
    foreach ($directory in @('Config', 'Source')) {
        $source = Join-Path $ProjectRoot $directory
        if (Test-Path -LiteralPath $source -PathType Container) {
            Copy-Item -LiteralPath $source -Destination $PrivateProject -Recurse
        }
    }
    $ExternalPluginRoot = Join-Path $PrivateProject 'ExternalPlugins'
    New-Item -ItemType Directory -Path $ExternalPluginRoot | Out-Null
    $PluginLink = Join-Path $ExternalPluginRoot 'IAmSpeed'
    New-Item -ItemType Junction -Path $PluginLink -Target $PluginRoot | Out-Null

    $Logs = Join-Path $RunRoot 'Logs'
    $AutomationReport = Join-Path $RunRoot 'Automation'
    $UbaRoot = Join-Path $RunRoot 'UBA'
    $LocalDataCache = Join-Path $RunRoot 'DDC'
    $PrivateLocalAppData = Join-Path $RunRoot 'UserData\LocalAppData'
    $PrivateAppData = Join-Path $RunRoot 'UserData\Roaming'
    $PrivateTemp = Join-Path $RunRoot 'Temp'
    foreach ($directory in @($Logs, $AutomationReport, $UbaRoot, $LocalDataCache, $PrivateLocalAppData, $PrivateAppData, $PrivateTemp)) {
        New-Item -ItemType Directory -Path $directory -Force | Out-Null
    }

    [Environment]::SetEnvironmentVariable('TEMP', $PrivateTemp, 'Process')
    [Environment]::SetEnvironmentVariable('TMP', $PrivateTemp, 'Process')
    [Environment]::SetEnvironmentVariable('LOCALAPPDATA', $PrivateLocalAppData, 'Process')
    [Environment]::SetEnvironmentVariable('APPDATA', $PrivateAppData, 'Process')
    [Environment]::SetEnvironmentVariable('UE-LocalDataCachePath', $LocalDataCache, 'Process')
    [Environment]::SetEnvironmentVariable('UE-SharedDataCachePath', 'None', 'Process')
    [Environment]::SetEnvironmentVariable('UE_SKIP_UBT_SDK_SETUP', '1', 'Process')

    $ProjectFile = Join-Path $PrivateProject 'IAmSpeedHostProject.uproject'
    $Build = Join-Path $Engine.Root 'Engine\Build\BatchFiles\Build.bat'
    $Editor = Join-Path $Engine.Root 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
    $BuildLog = Join-Path $Logs 'Build.log'
    $BuildConsoleLog = Join-Path $Logs 'Build.console.log'
    $EditorLog = Join-Path $Logs 'Automation.log'
    $EditorOutputLog = Join-Path $Logs 'Automation.stdout.log'
    $EditorErrorLog = Join-Path $Logs 'Automation.stderr.log'
    $ReportIndex = Join-Path $AutomationReport 'index.json'

    $BuildArguments = New-IAmSpeedBuildArguments `
        -ProjectFile $ProjectFile -LogPath $BuildLog -UbaRoot $UbaRoot `
        -MaxParallelActions $MaxParallelActions -UbaMaxWorkers $UBAMaxWorkers
    $EditorArguments = New-IAmSpeedEditorArguments `
        -ProjectFile $ProjectFile -TestFilter $TestFilter -LogPath $EditorLog `
        -ReportPath $AutomationReport
    $Execution = Invoke-IAmSpeedWithProcessTbbPath -LoaderDirectory $TbbLoader.LoaderDirectory -Action {
        & $Build @BuildArguments *> $BuildConsoleLog
        $BuildExitCode = $LASTEXITCODE
        if ($BuildExitCode -ne 0) {
            throw "HostProject editor build failed with exit code $BuildExitCode. See $BuildLog."
        }

        $EditorOutput = & $Editor @EditorArguments 2> $EditorErrorLog
        $EditorExitCode = $LASTEXITCODE
        [IO.File]::WriteAllText($EditorOutputLog, (($EditorOutput | ForEach-Object { [string]$_ }) -join [Environment]::NewLine), [Text.UTF8Encoding]::new($false))
        $Automation = Assert-IAmSpeedAutomationResult `
            -ExitCode $EditorExitCode -StandardOutput (Get-Content -Raw -LiteralPath $EditorOutputLog) `
            -ReportPath $ReportIndex -TestFilter $TestFilter
        [pscustomobject]@{
            BuildExitCode = $BuildExitCode
            EditorExitCode = $EditorExitCode
            Automation = $Automation
        }
    }
    $BuildExitCode = $Execution.BuildExitCode
    $EditorExitCode = $Execution.EditorExitCode
    $Automation = $Execution.Automation

    $Result = [pscustomobject]@{
        status = 'passed'
        engine_root = $Engine.Root
        engine_version = $Engine.Version
        compatible_changelist = $Engine.CompatibleChangelist
        tbb_loader_dll = $TbbLoader.DllPath
        tbb_loader_directory = $TbbLoader.LoaderDirectory
        tbb_loader_source = $TbbLoader.Source
        tbb_loader_sha256 = $TbbLoader.Sha256
        tbb_fallback_sha_validated = $TbbLoader.FallbackShaValidated
        tbb_loader_path_scope = 'process-only; restored after build and automation'
        private_run_root = $RunRoot
        host_project = $ProjectFile
        test_filter = $TestFilter
        tests_performed = $Automation.TestsPerformed
        tests_succeeded = $Automation.TestsSucceeded
        tests_failed = 0
        build_exit_code = $BuildExitCode
        editor_exit_code = $EditorExitCode
    }
    $Result | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $Logs 'result.json') -Encoding UTF8
    Write-Output ($Result | ConvertTo-Json -Depth 4 -Compress)
}
catch {
    if ($RunRoot -and (Test-Path -LiteralPath (Join-Path $RunRoot 'Logs') -PathType Container)) {
        $failure = [pscustomobject]@{
            status='failed'
            error=$_.Exception.Message
            private_run_root=$RunRoot
            tbb_loader_dll=if ($TbbLoader) { $TbbLoader.DllPath } else { $null }
            tbb_loader_directory=if ($TbbLoader) { $TbbLoader.LoaderDirectory } else { $null }
            tbb_loader_source=if ($TbbLoader) { $TbbLoader.Source } else { $null }
            tbb_loader_sha256=if ($TbbLoader) { $TbbLoader.Sha256 } else { $null }
            tbb_fallback_sha_validated=if ($TbbLoader) { $TbbLoader.FallbackShaValidated } else { $false }
            tbb_loader_path_scope='process-only; restored by finally'
        }
        $failure | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $RunRoot 'Logs\result.json') -Encoding UTF8
    }
    throw
}
finally {
    if ($PluginLink -and (Test-Path -LiteralPath $PluginLink)) {
        [IO.Directory]::Delete($PluginLink)
    }
    foreach ($name in $EnvironmentNames) {
        [Environment]::SetEnvironmentVariable($name, $OriginalEnvironment[$name], 'Process')
    }
}
