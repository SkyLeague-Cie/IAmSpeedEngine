$script:IAmSpeedExpectedTbbFallbackSha256 = 'af20d7ca563e542432b856f6628d9481247197d1853bd4057caaf6c449749d42'
$script:IAmSpeedRulesSeedFiles = @(
    @{ Name='UE5Rules.dll'; Bytes=964096; Sha256='a4a337f37541b8a8fdd53f96490de7b13e16e2978de41ac5ff18facc4fe2327c'; Manifest='UE5RulesManifest.json'; ManifestBytes=364846; ManifestSha256='ffd4524d047d2ce6ad5ac8b1abe77a3fd4f43ac61406cfe3a5c81888045e2cec' },
    @{ Name='UE5ProgramRules.dll'; Bytes=123904; Sha256='08c8be5f5d77d7e6f50c36e5c3c0e58b0d3f6f72b5677e3fe5e8f60a369cef80'; Manifest='UE5ProgramRulesManifest.json'; ManifestBytes=29913; ManifestSha256='ff4fe1365b438c8b2c8de0efbe5ce03896042d1e9273a9e8b8099735a44d22bf' }
)
$script:IAmSpeedRulesSeedPdbs = @(
    @{ Name='UE5Rules.pdb'; Bytes=475812; Sha256='1c4fbf0c19e866059d80819bda5237f9b5f628494bf69c1b2752e58cb7febe10' },
    @{ Name='UE5ProgramRules.pdb'; Bytes=47436; Sha256='b3fef02dd82a98308d7a577963b090dbb605f7fc9b6056156a268a16923ce46e' }
)

function Assert-IAmSpeedRulesSeedRoot {
    [CmdletBinding()]
    param([Parameter(Mandatory=$true)] [string]$EngineRoot)

    $engine = Assert-IAmSpeedEngineRoot -EngineRoot $EngineRoot
    $rulesDirectory = Join-Path $engine.Root 'Engine\Intermediate\Build\BuildRules'
    $ubtPath = Join-Path $engine.Root 'Engine\Binaries\DotNET\UnrealBuildTool\UnrealBuildTool.dll'
    if (Test-Path -LiteralPath (Join-Path $engine.Root 'Engine\Build\InstalledBuild.txt')) {
        throw 'Rules seed must use the exact pinned non-installed slot1 rules source; installed-engine roots use a different UBT skip path.'
    }
    if (-not (Test-Path -LiteralPath $ubtPath -PathType Leaf)) { throw "Rules seed UBT is missing: $ubtPath" }
    $ubtSha = (Get-FileHash -LiteralPath $ubtPath -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($ubtSha -cne '0c3adf01933fd31497971cbcc3f66315fabe061bbb4c56ac143cd809192e208d') {
        throw "Rules seed UBT SHA-256 is not the qualified 5.8.2 tool: $ubtSha"
    }
    $ubt = Get-Item -LiteralPath $ubtPath -Force
    $assemblyProof = @()
    foreach ($pin in $script:IAmSpeedRulesSeedFiles) {
        $assemblyPath = Join-Path $rulesDirectory $pin.Name
        $manifestPath = Join-Path $rulesDirectory $pin.Manifest
        if (-not (Test-Path -LiteralPath $assemblyPath -PathType Leaf) -or -not (Test-Path -LiteralPath $manifestPath -PathType Leaf)) {
            throw "Rules seed assembly or manifest is missing: $($pin.Name)"
        }
        $assembly = Get-Item -LiteralPath $assemblyPath -Force
        $assemblySha = (Get-FileHash -LiteralPath $assemblyPath -Algorithm SHA256).Hash.ToLowerInvariant()
        $manifest = Get-Item -LiteralPath $manifestPath -Force
        $manifestSha = (Get-FileHash -LiteralPath $manifestPath -Algorithm SHA256).Hash.ToLowerInvariant()
        if ($assembly.Length -ne $pin.Bytes -or $assemblySha -cne $pin.Sha256 -or
            $manifest.Length -ne $pin.ManifestBytes -or $manifestSha -cne $pin.ManifestSha256) {
            throw "Rules seed assembly/manifest differs from its qualified pin: $($pin.Name)"
        }
        try { [Reflection.AssemblyName]::GetAssemblyName($assemblyPath) | Out-Null }
        catch { throw "Rules seed assembly metadata cannot be read: $assemblyPath" }
        if ($ubt.LastWriteTimeUtc -gt $assembly.LastWriteTimeUtc) {
            throw "Rules seed would rebuild $($pin.Name): UBT is newer than the assembly."
        }
        $manifestData = Get-Content -LiteralPath $manifestPath -Raw | ConvertFrom-Json
        if ($manifestData.EngineVersion -cne '5.8.2' -or @($manifestData.SourceFiles).Count -eq 0) {
            throw "Rules seed manifest is invalid or belongs to another Engine version: $manifestPath"
        }
        $expectedSourceRoots = @('Source', 'Platforms', 'Plugins', 'Shaders') | ForEach-Object { Join-Path $engine.Root (Join-Path 'Engine' $_) }
        foreach ($sourcePath in $manifestData.SourceFiles) {
            $sourceFull = [IO.Path]::GetFullPath([string]$sourcePath)
            $underExpectedRoot = $false
            foreach ($expectedSourceRoot in $expectedSourceRoots) {
                if ($sourceFull.StartsWith($expectedSourceRoot.TrimEnd('\') + '\', [StringComparison]::OrdinalIgnoreCase)) { $underExpectedRoot = $true; break }
            }
            if (-not $underExpectedRoot) {
                throw "Rules seed manifest source escapes its pinned Engine root: $sourceFull"
            }
            if (-not (Test-Path -LiteralPath $sourceFull -PathType Leaf)) { throw "Rules seed source is missing: $sourceFull" }
            if ((Get-Item -LiteralPath $sourceFull -Force).LastWriteTimeUtc -gt $assembly.LastWriteTimeUtc) {
                throw "Rules seed would rebuild $($pin.Name): source is newer than the assembly ($sourceFull)."
            }
        }
        $assemblyProof += [pscustomobject]@{ path=$assemblyPath; bytes=$assembly.Length; sha256=$assemblySha; manifest=$manifestPath; manifestSha256=$manifestSha; sourceCount=@($manifestData.SourceFiles).Count; lastWriteTimeUtc=$assembly.LastWriteTimeUtc.ToString('o') }
    }
    foreach ($pin in $script:IAmSpeedRulesSeedPdbs) {
        $path = Join-Path $rulesDirectory $pin.Name
        if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Rules seed PDB is missing: $path" }
        $item = Get-Item -LiteralPath $path -Force
        $sha = (Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash.ToLowerInvariant()
        if ($item.Length -ne $pin.Bytes -or $sha -cne $pin.Sha256) { throw "Rules seed PDB differs from its qualified pin: $($pin.Name)" }
    }
    $marketplaceRoot = Join-Path $engine.Root 'Engine\Plugins\Marketplace'
    $marketplaceDescriptors = @()
    if (Test-Path -LiteralPath $marketplaceRoot -PathType Container) {
        $marketplaceDescriptors = @(Get-ChildItem -LiteralPath $marketplaceRoot -Filter '*.uplugin' -Recurse -File -ErrorAction SilentlyContinue)
    }
    if ($marketplaceDescriptors.Count -gt 0) {
        throw 'Marketplace rules would add another Engine-side assembly; this bounded rules seed does not allow it.'
    }
    return [pscustomobject]@{ Root=$engine.Root; Build=$engine.Build; UbtPath=$ubtPath; UbtSha256=$ubtSha; RulesDirectory=$rulesDirectory; Assemblies=$assemblyProof }
}

function Assert-IAmSpeedPrecompiledRules {
    [CmdletBinding()]
    param([Parameter(Mandatory=$true)] [string]$EngineRoot)

    $engine = Assert-IAmSpeedEngineRoot -EngineRoot $EngineRoot
    $directory = Join-Path $engine.Root 'Engine\Intermediate\Build\BuildRules'
    $files = @()
    foreach ($pin in $script:IAmSpeedRulesSeedFiles) {
        foreach ($name in @($pin.Name, $pin.Manifest)) {
            $path = Join-Path $directory $name
            if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Precompiled Engine rules file is missing: $path" }
            $item = Get-Item -LiteralPath $path -Force
            $sha = (Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash.ToLowerInvariant()
            $expectedBytes = if ($name -eq $pin.Name) { $pin.Bytes } else { $pin.ManifestBytes }
            $expectedSha = if ($name -eq $pin.Name) { $pin.Sha256 } else { $pin.ManifestSha256 }
            if ($item.Length -ne $expectedBytes -or $sha -cne $expectedSha) { throw "Precompiled Engine rules file does not match the qualified assembly: $path" }
            $files += [pscustomobject]@{ path=$path; bytes=$item.Length; sha256=$sha }
        }
    }
    foreach ($pin in $script:IAmSpeedRulesSeedPdbs) {
        $path = Join-Path $directory $pin.Name
        if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Precompiled Engine rules PDB is missing: $path" }
        $item = Get-Item -LiteralPath $path -Force
        $sha = (Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash.ToLowerInvariant()
        if ($item.Length -ne $pin.Bytes -or $sha -cne $pin.Sha256) { throw "Precompiled Engine rules PDB does not match the qualified assembly: $path" }
        $files += [pscustomobject]@{ path=$path; bytes=$item.Length; sha256=$sha }
    }
    return [pscustomobject]@{ Root=$engine.Root; RulesDirectory=$directory; Files=$files }
}

function Get-IAmSpeedRulesSnapshot {
    [CmdletBinding()]
    param([Parameter(Mandatory=$true)] [string]$EngineRoot)

    $root = [IO.Path]::GetFullPath($EngineRoot).TrimEnd('\')
    $directory = Join-Path $root 'Engine\Intermediate\Build\BuildRules'
    $names = @('UE5Rules.dll', 'UE5Rules.pdb', 'UE5RulesManifest.json',
        'UE5ProgramRules.dll', 'UE5ProgramRules.pdb', 'UE5ProgramRulesManifest.json')
    $files = @()
    foreach ($name in $names) {
        $path = Join-Path $directory $name
        if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Engine rules snapshot file is missing: $path" }
        $item = Get-Item -LiteralPath $path -Force
        if (($item.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0) { throw "Engine rules snapshot file is a reparse point: $path" }
        $files += [pscustomobject]@{
            name = $name
            path = $item.FullName
            bytes = [long]$item.Length
            sha256 = (Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash.ToLowerInvariant()
            lastWriteUtcTicks = [long]$item.LastWriteTimeUtc.Ticks
            lastWriteTimeUtc = $item.LastWriteTimeUtc.ToString('o')
        }
    }
    return [pscustomobject]@{ engineRoot=$root; rulesDirectory=$directory; files=$files; capturedUtc=[DateTime]::UtcNow.ToString('o') }
}

function Assert-IAmSpeedRulesSnapshotUnchanged {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory=$true)] [psobject]$Before,
        [Parameter(Mandatory=$true)] [psobject]$After,
        [Parameter(Mandatory=$true)] [string]$Label
    )
    if ($Before.engineRoot -ine $After.engineRoot -or @($Before.files).Count -ne 6 -or @($After.files).Count -ne 6) {
        throw "$Label Engine-rules snapshot root or file count changed. No automatic rebaseline or restore is permitted."
    }
    foreach ($beforeFile in $Before.files) {
        $afterFile = @($After.files | Where-Object { $_.name -ceq $beforeFile.name })
        if ($afterFile.Count -ne 1) { throw "$Label Engine-rules snapshot lost or duplicated $($beforeFile.name). No automatic rebaseline or restore is permitted." }
        $current = $afterFile[0]
        if ($beforeFile.path -ine $current.path -or [long]$beforeFile.bytes -ne [long]$current.bytes -or
            $beforeFile.sha256 -cne $current.sha256 -or [long]$beforeFile.lastWriteUtcTicks -ne [long]$current.lastWriteUtcTicks) {
            throw "$Label Engine-rules file changed during the owned operation: $($beforeFile.path). No automatic rebaseline or restore is permitted."
        }
    }
    return $true
}

function New-IAmSpeedProjectRulesQueryArguments {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory=$true)] [string]$ProjectFile,
        [Parameter(Mandatory=$true)] [string]$OutputPath,
        [Parameter(Mandatory=$true)] [string]$LogPath
    )
    return [string[]]@(
        '-Mode=QueryTargets', "-Project=$ProjectFile", "-Output=$OutputPath",
        '-UsePrecompiled', '-NoEngineChanges', '-SLPrivateProjectResources', "-Log=$LogPath"
    )
}

function Resolve-IAmSpeedTbbLoader {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory=$true)] [string]$EngineRoot,
        [string]$ExpectedFallbackSha256 = $script:IAmSpeedExpectedTbbFallbackSha256
    )

    $root = [IO.Path]::GetFullPath($EngineRoot).TrimEnd('\')
    $binaryPath = Join-Path $root 'Engine\Binaries\Win64\tbbmalloc.dll'
    if (Test-Path -LiteralPath $binaryPath -PathType Leaf) {
        $file = Get-Item -LiteralPath $binaryPath -Force
        return [pscustomobject]@{
            LoaderDirectory = Split-Path -Parent $binaryPath
            DllPath = $binaryPath
            Sha256 = (Get-FileHash -LiteralPath $binaryPath -Algorithm SHA256).Hash.ToLowerInvariant()
            Source = 'EngineBinaries'
            FallbackShaValidated = $false
        }
    }

    $fallbackPath = Join-Path $root 'Engine\Source\ThirdParty\Intel\TBB\Deploy\oneTBB-2022.3.0\VS2015\x64\bin\tbbmalloc.dll'
    if (-not (Test-Path -LiteralPath $fallbackPath -PathType Leaf)) {
        throw "tbbmalloc.dll was not found in Engine Binaries or the pinned oneTBB fallback: $fallbackPath"
    }
    $fallback = Get-Item -LiteralPath $fallbackPath -Force
    $fallbackSha = (Get-FileHash -LiteralPath $fallbackPath -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($fallbackSha -cne $ExpectedFallbackSha256.ToLowerInvariant()) {
        throw "The oneTBB fallback tbbmalloc.dll SHA-256 is not the qualified payload: $fallbackSha"
    }
    return [pscustomobject]@{
        LoaderDirectory = Split-Path -Parent $fallbackPath
        DllPath = $fallbackPath
        Sha256 = $fallbackSha
        Source = 'PinnedThirdPartyFallback'
        FallbackShaValidated = $true
    }
}

function New-IAmSpeedProcessPath {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory=$true)] [string]$LoaderDirectory,
        [AllowNull()] [string]$ExistingPath
    )
    $directory = [IO.Path]::GetFullPath($LoaderDirectory).TrimEnd('\')
    if ([string]::IsNullOrWhiteSpace($ExistingPath)) { return $directory }
    return $directory + ';' + $ExistingPath
}

function Invoke-IAmSpeedWithProcessTbbPath {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory=$true)] [string]$LoaderDirectory,
        [Parameter(Mandatory=$true)] [scriptblock]$Action
    )
    $originalPath = [Environment]::GetEnvironmentVariable('PATH', 'Process')
    try {
        [Environment]::SetEnvironmentVariable('PATH', (New-IAmSpeedProcessPath -LoaderDirectory $LoaderDirectory -ExistingPath $originalPath), 'Process')
        & $Action
    }
    finally {
        [Environment]::SetEnvironmentVariable('PATH', $originalPath, 'Process')
    }
}

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
        [ValidateRange(1, 8)] [int]$UbaMaxWorkers = 4,
        [switch]$SkipRulesCompile
    )
    $arguments = [System.Collections.Generic.List[string]]::new()
    $arguments.AddRange([string[]]@(
        'IAmSpeedHostProjectEditor', 'Win64', 'Development', "-Project=$ProjectFile",
        '-WaitMutex', '-NoHotReload', '-UsePrecompiled', '-NoEngineChanges',
        '-NoXGE', '-NoFASTBuild', '-NoSNDBS', "-MaxParallelActions=$MaxParallelActions",
        "-UBARootDir=$UbaRoot", '-UBAStoreCapacityGb=4', "-UBAMaxWorkers=$UbaMaxWorkers",
        '-UBADisableRemote', '-UBADisableHorde', '-SLPrivateProjectResources', "-Log=$LogPath"
    ))
    if ($SkipRulesCompile) { $arguments.Add('-SkipRulesCompile') }
    return [string[]]$arguments.ToArray()
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
