[CmdletBinding()]
param(
    [string]$EngineRoot = $env:IAMSPEED_UE_ROOT,
    [string]$RulesEngineRoot = $env:IAMSPEED_RULES_ENGINE_ROOT,
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
    'UE-LocalDataCachePath', 'UE-SharedDataCachePath', 'UE_SKIP_UBT_SDK_SETUP',
    'DOTNET_CLI_HOME', 'DOTNET_CLI_TELEMETRY_OPTOUT', 'DOTNET_SKIP_FIRST_TIME_EXPERIENCE'
)
$OriginalEnvironment = @{}
$TbbLoader = $null
$RulesSeedTbbLoader = $null
$RulesEvidence = $null
foreach ($name in $EnvironmentNames) {
    $OriginalEnvironment[$name] = [Environment]::GetEnvironmentVariable($name, 'Process')
}

try {
    $Engine = Assert-IAmSpeedEngineRoot -EngineRoot $EngineRoot
    if ([string]::IsNullOrWhiteSpace($RulesEngineRoot)) {
        throw 'IAMSPEED_RULES_ENGINE_ROOT is required for isolated project-rules preparation.'
    }
    $RulesSeedEngine = Assert-IAmSpeedRulesSeedRoot -EngineRoot $RulesEngineRoot
    $DirectUbtRuntime = Assert-IAmSpeedDirectUbtRuntime -EngineRoot $Engine.Root -PrivateManifestPath $env:IAMSPEED_PRIVATE_UBT_MANIFEST -PrivateManifestSha256 $env:IAMSPEED_PRIVATE_UBT_MANIFEST_SHA256
    if ($RulesSeedEngine.Root -ieq $Engine.Root) {
        throw 'Rules seed root must be a separate qualified slot from the target Engine root.'
    }
    $PrecompiledEngineRules = Assert-IAmSpeedPrecompiledRules -EngineRoot $Engine.Root
    if ([string]::IsNullOrWhiteSpace($TestFilter) -or
        $TestFilter -notmatch '^IAmSpeed\.AnalyticWorld(?:\.[A-Za-z0-9_.]+)?$') {
        throw "Unsupported physical-contract test filter: $TestFilter"
    }

    $ProjectRoot = Split-Path -Parent $PSScriptRoot
    $PluginRoot = Split-Path -Parent $ProjectRoot
    $PrivateParent = Assert-IAmSpeedPrivateRoot -PrivateRoot $PrivateRoot -EngineRoot $Engine.Root -PluginRoot $PluginRoot
    $TbbLoader = Resolve-IAmSpeedTbbLoader -EngineRoot $Engine.Root
    $RulesSeedTbbLoader = Resolve-IAmSpeedTbbLoader -EngineRoot $RulesSeedEngine.Root
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
    $PrivateDotNetHome = Join-Path $RunRoot 'DotNetCliHome'
    foreach ($directory in @($Logs, $AutomationReport, $UbaRoot, $LocalDataCache, $PrivateLocalAppData, $PrivateAppData, $PrivateTemp, $PrivateDotNetHome)) {
        New-Item -ItemType Directory -Path $directory -Force | Out-Null
    }

    [Environment]::SetEnvironmentVariable('TEMP', $PrivateTemp, 'Process')
    [Environment]::SetEnvironmentVariable('TMP', $PrivateTemp, 'Process')
    [Environment]::SetEnvironmentVariable('LOCALAPPDATA', $PrivateLocalAppData, 'Process')
    [Environment]::SetEnvironmentVariable('APPDATA', $PrivateAppData, 'Process')
    [Environment]::SetEnvironmentVariable('UE-LocalDataCachePath', $LocalDataCache, 'Process')
    [Environment]::SetEnvironmentVariable('UE-SharedDataCachePath', 'None', 'Process')
    [Environment]::SetEnvironmentVariable('UE_SKIP_UBT_SDK_SETUP', '1', 'Process')
    [Environment]::SetEnvironmentVariable('DOTNET_CLI_HOME', $PrivateDotNetHome, 'Process')
    [Environment]::SetEnvironmentVariable('DOTNET_CLI_TELEMETRY_OPTOUT', '1', 'Process')
    [Environment]::SetEnvironmentVariable('DOTNET_SKIP_FIRST_TIME_EXPERIENCE', '1', 'Process')

    $ProjectFile = Join-Path $PrivateProject 'IAmSpeedHostProject.uproject'
    $Build = Join-Path $Engine.Root 'Engine\Build\BatchFiles\Build.bat'
    $Editor = Join-Path $Engine.Root 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
    $BuildLog = Join-Path $Logs 'Build.log'
    $BuildConsoleLog = Join-Path $Logs 'Build.console.log'
    $EditorLog = Join-Path $Logs 'Automation.log'
    $EditorOutputLog = Join-Path $Logs 'Automation.stdout.log'
    $EditorErrorLog = Join-Path $Logs 'Automation.stderr.log'
    $ReportIndex = Join-Path $AutomationReport 'index.json'
    $RulesQueryLog = Join-Path $Logs 'ProjectRules.QueryTargets.log'
    $RulesQueryConsoleLog = Join-Path $Logs 'ProjectRules.QueryTargets.console.log'
    $RulesTargetInfo = Join-Path $AutomationReport 'ProjectRules.TargetInfo.json'
    $RulesAssemblyDirectory = Join-Path $PrivateProject 'Intermediate\Build\BuildRules'
    $RulesAssemblyPath = Join-Path $RulesAssemblyDirectory 'IAmSpeedHostProjectModuleRules.dll'
    $RulesPdbPath = Join-Path $RulesAssemblyDirectory 'IAmSpeedHostProjectModuleRules.pdb'
    $RulesManifestPath = Join-Path $RulesAssemblyDirectory 'IAmSpeedHostProjectModuleRulesManifest.json'
    $SeedRulesBeforeQuery = Get-IAmSpeedRulesSnapshot -EngineRoot $RulesSeedEngine.Root
    $TargetRulesBeforeQuery = Get-IAmSpeedRulesSnapshot -EngineRoot $Engine.Root
    $RulesEvidence = [ordered]@{
        seed_engine_root = $RulesSeedEngine.Root
        target_engine_root = $Engine.Root
        seed_direct_ubt_runtime = [pscustomobject]@{ dotnet=$RulesSeedEngine.DotNetPath; dotnet_directory=$RulesSeedEngine.DotNetDirectory; dotnet_sha256=$RulesSeedEngine.DotNetSha256; ubt=$RulesSeedEngine.UbtPath; ubt_sha256=$RulesSeedEngine.UbtSha256; working_directory=$RulesSeedEngine.WorkingDirectory }
        target_direct_ubt_runtime = [pscustomobject]@{ dotnet=$DirectUbtRuntime.DotNetPath; dotnet_directory=$DirectUbtRuntime.DotNetDirectory; dotnet_sha256=$DirectUbtRuntime.DotNetSha256; ubt=$DirectUbtRuntime.UbtPath; ubt_sha256=$DirectUbtRuntime.UbtSha256; working_directory=$DirectUbtRuntime.WorkingDirectory }
        seed_before_query = $SeedRulesBeforeQuery
        target_before_query = $TargetRulesBeforeQuery
    }

    $RulesQueryArguments = New-IAmSpeedProjectRulesQueryArguments `
        -ProjectFile $ProjectFile -OutputPath $RulesTargetInfo -LogPath $RulesQueryLog
    $RulesQueryArguments += @('-NoXGE', '-NoFASTBuild', '-NoSNDBS', '-MaxParallelActions=1')
    $RulesQueryInvocation = New-IAmSpeedDirectUbtInvocation -Runtime $RulesSeedEngine -Arguments $RulesQueryArguments
    $RulesQueryInvocationArguments = [string[]]$RulesQueryInvocation.Arguments
    $RulesEvidence.query_invocation = $RulesQueryInvocation
    $RulesEvidence.query_session = $RulesQueryInvocation.SessionId
    $RulesEvidence.query_trace_policy = 'explicit -Session argument suppresses UBT default Engine-side Trace.uba creation before environment parsing'
    $RulesQueryExecution = Invoke-IAmSpeedWithProcessTbbPath -LoaderDirectory $RulesSeedTbbLoader.LoaderDirectory -DotNetDirectory $RulesQueryInvocation.DotNetDirectory -Action {
        Push-Location -LiteralPath $RulesQueryInvocation.WorkingDirectory
        try {
            & $RulesQueryInvocation.Executable @RulesQueryInvocationArguments *> $RulesQueryConsoleLog
            [pscustomobject]@{ ExitCode=$LASTEXITCODE }
        }
        finally { Pop-Location }
    }
    $SeedRulesAfterQuery = Get-IAmSpeedRulesSnapshot -EngineRoot $RulesSeedEngine.Root
    $TargetRulesAfterQuery = Get-IAmSpeedRulesSnapshot -EngineRoot $Engine.Root
    $RulesEvidence.seed_after_query = $SeedRulesAfterQuery
    $RulesEvidence.target_after_query = $TargetRulesAfterQuery
    Assert-IAmSpeedRulesSnapshotUnchanged -Before $SeedRulesBeforeQuery -After $SeedRulesAfterQuery -Label 'Rules-seed QueryTargets'
    Assert-IAmSpeedRulesSnapshotUnchanged -Before $TargetRulesBeforeQuery -After $TargetRulesAfterQuery -Label 'Target Engine during QueryTargets'
    $RulesQueryExitCode = $RulesQueryExecution.ExitCode
    if ($RulesQueryExitCode -ne 0) {
        throw "Private project-rules QueryTargets failed with exit code $RulesQueryExitCode. See $RulesQueryLog."
    }
    if (-not (Test-Path -LiteralPath $RulesTargetInfo -PathType Leaf) -or
        -not (Select-String -LiteralPath $RulesTargetInfo -SimpleMatch 'IAmSpeedHostProjectEditor' -Quiet)) {
        throw "QueryTargets did not report IAmSpeedHostProjectEditor: $RulesTargetInfo"
    }
    foreach ($path in @($RulesAssemblyPath, $RulesPdbPath, $RulesManifestPath)) {
        if (-not (Test-Path -LiteralPath $path -PathType Leaf) -or (Get-Item -LiteralPath $path -Force).Length -le 0) {
            throw "QueryTargets did not emit a private project-rules artifact: $path"
        }
    }
    $RulesManifest = Get-Content -LiteralPath $RulesManifestPath -Raw | ConvertFrom-Json
    if ($RulesManifest.EngineVersion -cne '5.8.2' -or @($RulesManifest.SourceFiles).Count -lt 2) {
        throw "Private project-rules manifest is invalid: $RulesManifestPath"
    }
    $PrivateProjectPrefix = [IO.Path]::GetFullPath($PrivateProject).TrimEnd('\') + '\'
    $PluginSourcePrefix = [IO.Path]::GetFullPath($PluginRoot).TrimEnd('\') + '\'
    foreach ($source in $RulesManifest.SourceFiles) {
        $fullSource = [IO.Path]::GetFullPath([string]$source)
        $isPrivateProjectSource = $fullSource.StartsWith($PrivateProjectPrefix, [StringComparison]::OrdinalIgnoreCase)
        $isAllowedPluginSource = $fullSource.StartsWith($PluginSourcePrefix, [StringComparison]::OrdinalIgnoreCase)
        if ((-not $isPrivateProjectSource -and -not $isAllowedPluginSource) -or
            -not (Test-Path -LiteralPath $fullSource -PathType Leaf)) {
            throw "Project-rules manifest source escapes the private HostProject and its linked IAmSpeed plugin source: $fullSource"
        }
    }
    $RulesAssemblyEvidence = [pscustomobject]@{
        query_engine_root = $RulesSeedEngine.Root
        query_ubt_sha256 = $RulesSeedEngine.UbtSha256
        target_info = $RulesTargetInfo
        rules_assembly = [pscustomobject]@{ path=$RulesAssemblyPath; bytes=(Get-Item $RulesAssemblyPath).Length; sha256=(Get-FileHash $RulesAssemblyPath -Algorithm SHA256).Hash.ToLowerInvariant() }
        rules_pdb = [pscustomobject]@{ path=$RulesPdbPath; bytes=(Get-Item $RulesPdbPath).Length; sha256=(Get-FileHash $RulesPdbPath -Algorithm SHA256).Hash.ToLowerInvariant() }
        rules_manifest = [pscustomobject]@{ path=$RulesManifestPath; sha256=(Get-FileHash $RulesManifestPath -Algorithm SHA256).Hash.ToLowerInvariant(); source_count=@($RulesManifest.SourceFiles).Count }
        engine_rules_inputs = $PrecompiledEngineRules.Files
    }

    $BuildArguments = New-IAmSpeedBuildArguments `
        -ProjectFile $ProjectFile -LogPath $BuildLog -UbaRoot $UbaRoot `
        -MaxParallelActions $MaxParallelActions -UbaMaxWorkers $UBAMaxWorkers -SkipRulesCompile
    $BuildInvocation = New-IAmSpeedDirectUbtInvocation -Runtime $DirectUbtRuntime -Arguments $BuildArguments
    $BuildInvocationArguments = [string[]]$BuildInvocation.Arguments
    $RulesEvidence.build_invocation = $BuildInvocation
    $RulesEvidence.private_ubt_manifest = $DirectUbtRuntime.PrivateManifestPath
    $RulesEvidence.private_ubt_manifest_sha256 = $DirectUbtRuntime.PrivateManifestSha256
    $RulesEvidence.private_engine_policy = $DirectUbtRuntime.PrivateEnginePolicyPath
    $EditorArguments = New-IAmSpeedEditorArguments `
        -ProjectFile $ProjectFile -TestFilter $TestFilter -LogPath $EditorLog `
        -ReportPath $AutomationReport
    $RulesEvidence.build_session = $BuildInvocation.SessionId
    $RulesEvidence.build_trace_policy = 'explicit -Session argument suppresses UBT default Engine-side Trace.uba creation before environment parsing'
    $BuildExecution = Invoke-IAmSpeedWithProcessTbbPath -LoaderDirectory $TbbLoader.LoaderDirectory -DotNetDirectory $BuildInvocation.DotNetDirectory -Action {
        Push-Location -LiteralPath $BuildInvocation.WorkingDirectory
        try {
            Invoke-IAmSpeedPrivateEnginePolicy -Runtime $DirectUbtRuntime -Action {
                & $BuildInvocation.Executable @BuildInvocationArguments *> $BuildConsoleLog
                [pscustomobject]@{ ExitCode=$LASTEXITCODE }
            }
        }
        finally { Pop-Location }
    }
    $BuildExitCode = $BuildExecution.ExitCode
    $SeedRulesAfterBuild = Get-IAmSpeedRulesSnapshot -EngineRoot $RulesSeedEngine.Root
    $TargetRulesAfterBuild = Get-IAmSpeedRulesSnapshot -EngineRoot $Engine.Root
    $RulesEvidence.seed_after_build = $SeedRulesAfterBuild
    $RulesEvidence.target_after_build = $TargetRulesAfterBuild
    Assert-IAmSpeedRulesSnapshotUnchanged -Before $SeedRulesBeforeQuery -After $SeedRulesAfterBuild -Label 'Rules seed during target build'
    Assert-IAmSpeedRulesSnapshotUnchanged -Before $TargetRulesBeforeQuery -After $TargetRulesAfterBuild -Label 'Target Engine during target build'
    if ($BuildExitCode -ne 0) {
        throw "HostProject editor build failed with exit code $BuildExitCode. See $BuildLog."
    }

    $EditorExecution = Invoke-IAmSpeedWithProcessTbbPath -LoaderDirectory $TbbLoader.LoaderDirectory -Action {
        $EditorOutput = & $Editor @EditorArguments 2> $EditorErrorLog
        $EditorExitCode = $LASTEXITCODE
        [IO.File]::WriteAllText($EditorOutputLog, (($EditorOutput | ForEach-Object { [string]$_ }) -join [Environment]::NewLine), [Text.UTF8Encoding]::new($false))
        [pscustomobject]@{ ExitCode=$EditorExitCode }
    }
    $EditorExitCode = $EditorExecution.ExitCode
    $SeedRulesAfterEditor = Get-IAmSpeedRulesSnapshot -EngineRoot $RulesSeedEngine.Root
    $TargetRulesAfterEditor = Get-IAmSpeedRulesSnapshot -EngineRoot $Engine.Root
    $RulesEvidence.seed_after_editor = $SeedRulesAfterEditor
    $RulesEvidence.target_after_editor = $TargetRulesAfterEditor
    Assert-IAmSpeedRulesSnapshotUnchanged -Before $SeedRulesBeforeQuery -After $SeedRulesAfterEditor -Label 'Rules seed during Automation'
    Assert-IAmSpeedRulesSnapshotUnchanged -Before $TargetRulesBeforeQuery -After $TargetRulesAfterEditor -Label 'Target Engine during Automation'
    $Automation = Assert-IAmSpeedAutomationResult `
        -ExitCode $EditorExitCode -StandardOutput (Get-Content -Raw -LiteralPath $EditorOutputLog) `
        -ReportPath $ReportIndex -TestFilter $TestFilter

    $Result = [pscustomobject]@{
        status = 'passed'
        engine_root = $Engine.Root
        engine_version = $Engine.Version
        compatible_changelist = $Engine.CompatibleChangelist
        project_rules_precompile = $RulesAssemblyEvidence
        engine_rule_write_guard = $RulesEvidence
        tbb_loader_dll = $TbbLoader.DllPath
        tbb_loader_directory = $TbbLoader.LoaderDirectory
        tbb_loader_source = $TbbLoader.Source
        tbb_loader_sha256 = $TbbLoader.Sha256
        tbb_fallback_sha_validated = $TbbLoader.FallbackShaValidated
        tbb_loader_path_scope = 'process-only; restored after build and automation'
        rules_seed_engine_root = $RulesSeedEngine.Root
        rules_seed_tbb_loader_dll = $RulesSeedTbbLoader.DllPath
        rules_seed_tbb_loader_sha256 = $RulesSeedTbbLoader.Sha256
        private_run_root = $RunRoot
        host_project = $ProjectFile
        test_filter = $TestFilter
        tests_performed = $Automation.TestsPerformed
        tests_succeeded = $Automation.TestsSucceeded
        tests_failed = 0
        build_exit_code = $BuildExitCode
        editor_exit_code = $EditorExitCode
    }
    $Result | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath (Join-Path $Logs 'result.json') -Encoding UTF8
    Write-Output ($Result | ConvertTo-Json -Depth 12 -Compress)
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
            engine_rule_write_guard=$RulesEvidence
            rules_seed_engine_root=if ($RulesSeedEngine) { $RulesSeedEngine.Root } else { $null }
            rules_seed_tbb_loader_dll=if ($RulesSeedTbbLoader) { $RulesSeedTbbLoader.DllPath } else { $null }
            rules_seed_tbb_loader_sha256=if ($RulesSeedTbbLoader) { $RulesSeedTbbLoader.Sha256 } else { $null }
        }
        $failure | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath (Join-Path $RunRoot 'Logs\result.json') -Encoding UTF8
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
