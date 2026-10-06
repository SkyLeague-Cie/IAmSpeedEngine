function Assert-IAmSpeedPrivateRoot {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory=$true)] [AllowNull()] [string]$PrivateRoot,
        [Parameter(Mandatory=$true)] [string]$EngineRoot,
        [Parameter(Mandatory=$true)] [string]$PluginRoot
    )

    if ([string]::IsNullOrWhiteSpace($PrivateRoot)) {
        throw 'IAMSPEED_PRIVATE_ROOT is required; use a fresh RM-approved private output root.'
    }
    $privatePath = [IO.Path]::GetFullPath($PrivateRoot).TrimEnd('\')
    $enginePath = [IO.Path]::GetFullPath($EngineRoot).TrimEnd('\')
    $pluginPath = [IO.Path]::GetFullPath($PluginRoot).TrimEnd('\')
    if ($privatePath -ieq $enginePath -or $privatePath.StartsWith($enginePath + '\', [StringComparison]::OrdinalIgnoreCase)) {
        throw 'Private output root must be outside the installed Engine.'
    }
    if ($privatePath -ieq $pluginPath -or $privatePath.StartsWith($pluginPath + '\', [StringComparison]::OrdinalIgnoreCase)) {
        throw "Private output root must be outside the entire plugin source checkout: $pluginPath"
    }
    return $privatePath
}

function Assert-IAmSpeedEngineRoot {
    [CmdletBinding()]
    param([AllowNull()] [string]$EngineRoot)

    if ([string]::IsNullOrWhiteSpace($EngineRoot)) {
        throw 'IAMSPEED_UE_ROOT is required; Unreal Engine fallback/bootstrap is disabled.'
    }
    if (-not (Test-Path -LiteralPath $EngineRoot -PathType Container)) {
        throw "The requested Unreal Engine root does not exist: $EngineRoot"
    }
    $root = (Resolve-Path -LiteralPath $EngineRoot).Path.TrimEnd('\')
    $rootItem = Get-Item -LiteralPath $root -Force
    if (($rootItem.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0) {
        throw "The requested Unreal Engine root must be a physical directory: $root"
    }

    $versionPath = Join-Path $root 'Engine\Build\Build.version'
    $build = Join-Path $root 'Engine\Build\BatchFiles\Build.bat'
    $editor = Join-Path $root 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
    if (-not (Test-Path -LiteralPath $versionPath -PathType Leaf) -or
        -not (Test-Path -LiteralPath $build -PathType Leaf) -or
        -not (Test-Path -LiteralPath $editor -PathType Leaf)) {
        throw "The requested Unreal Engine root is incomplete: $root"
    }
    try { $version = Get-Content -Raw -LiteralPath $versionPath | ConvertFrom-Json }
    catch { throw "The requested Unreal Engine Build.version is invalid: $versionPath" }
    $actual = '{0}.{1}.{2}' -f $version.MajorVersion, $version.MinorVersion, $version.PatchVersion
    if ($actual -cne '5.8.2' -or [int]$version.CompatibleChangelist -ne 55116800) {
        throw "Expected Unreal Engine 5.8.2 CL 55116800; found $actual CL $($version.CompatibleChangelist)."
    }

    return [pscustomobject]@{
        Root = $root
        Version = $actual
        CompatibleChangelist = [int]$version.CompatibleChangelist
        Build = $build
        Editor = $editor
    }
}

function New-IAmSpeedBuildArguments {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory=$true)] [string]$ProjectFile,
        [Parameter(Mandatory=$true)] [string]$LogPath,
        [Parameter(Mandatory=$true)] [string]$UbaRoot,
        [ValidateRange(1, 8)] [int]$MaxParallelActions = 4,
        [ValidateRange(1, 8)] [int]$UbaMaxWorkers = 4
    )
    return [string[]]@(
        'IAmSpeedHostProjectEditor', 'Win64', 'Development', "-Project=$ProjectFile",
        '-WaitMutex', '-NoHotReload', '-UsePrecompiled', '-NoEngineChanges',
        '-NoXGE', '-NoFASTBuild', '-NoSNDBS', "-MaxParallelActions=$MaxParallelActions",
        "-UBARootDir=$UbaRoot", '-UBAStoreCapacityGb=4', "-UBAMaxWorkers=$UbaMaxWorkers",
        '-UBADisableRemote', '-UBADisableHorde', '-SLPrivateProjectResources', "-Log=$LogPath"
    )
}

function New-IAmSpeedEditorArguments {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory=$true)] [string]$ProjectFile,
        [Parameter(Mandatory=$true)] [string]$TestFilter,
        [Parameter(Mandatory=$true)] [string]$LogPath,
        [Parameter(Mandatory=$true)] [string]$ReportPath
    )
    return [string[]]@(
        $ProjectFile, '-unattended', '-nop4', '-nosplash', '-NullRHI', '-NoSound',
        '-stdout', '-FullStdOutLogOutput', '-nosteam', '-NoEOS', '-Locale=en-US',
        "-AbsLog=$LogPath", "-ReportExportPath=$ReportPath",
        '-TestExit=Automation Test Queue Empty', "-ExecCmds=Automation RunTests $TestFilter"
    )
}

function Assert-IAmSpeedAutomationResult {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory=$true)] [int]$ExitCode,
        [Parameter(Mandatory=$true)] [AllowEmptyString()] [string]$StandardOutput,
        [Parameter(Mandatory=$true)] [string]$ReportPath,
        [Parameter(Mandatory=$true)] [string]$TestFilter
    )
    if ($ExitCode -ne 0) { throw "UnrealEditor-Cmd exited with code $ExitCode." }
    $queueMatches = [regex]::Matches($StandardOutput, 'Automation Test Queue Empty (\d+) tests performed\.')
    if ($queueMatches.Count -ne 1) {
        throw 'Automation completion marker is missing or ambiguous.'
    }
    $queueCount = [int]$queueMatches[0].Groups[1].Value
    if ($queueCount -le 0) { throw 'Automation completed without running any tests.' }
    if (-not (Test-Path -LiteralPath $ReportPath -PathType Leaf)) {
        throw "Automation report index is missing: $ReportPath"
    }
    try { $report = Get-Content -Raw -LiteralPath $ReportPath | ConvertFrom-Json }
    catch { throw "Automation report index is invalid: $ReportPath" }
    if ($report.PSObject.Properties.Name -notcontains 'tests') {
        throw 'Automation report has no tests array.'
    }
    $tests = @($report.tests)
    if ($tests.Count -ne $queueCount) {
        throw "Automation queue/report count differs: queue=$queueCount report=$($tests.Count)."
    }
    if (@($tests | Where-Object { $_.state -cne 'Success' }).Count -ne 0) {
        throw 'Automation report contains failed, warning, skipped, or incomplete tests.'
    }
    $paths = @($tests | ForEach-Object { [string]$_.fullTestPath })
    if (@($paths | Where-Object { [string]::IsNullOrWhiteSpace($_) -or $_ -notlike "$TestFilter.*" }).Count -ne 0) {
        throw "Automation report contains tests outside the requested filter '$TestFilter'."
    }
    if (@($paths | Group-Object | Where-Object { $_.Count -ne 1 }).Count -ne 0) {
        throw 'Automation report contains duplicate test paths.'
    }
    foreach ($name in @('succeeded', 'succeededWithWarnings', 'failed', 'notRun', 'inProcess')) {
        if ($report.PSObject.Properties.Name -notcontains $name) {
            throw "Automation report is missing aggregate field '$name'."
        }
    }
    if ([int]$report.succeeded -ne $queueCount -or [int]$report.succeededWithWarnings -ne 0 -or
        [int]$report.failed -ne 0 -or [int]$report.notRun -ne 0 -or [int]$report.inProcess -ne 0) {
        throw 'Automation aggregate counts do not describe a complete, warning-free pass.'
    }
    return [pscustomobject]@{ TestsPerformed=$queueCount; TestsSucceeded=[int]$report.succeeded }
}
