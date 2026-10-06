$script:IAmSpeedExpectedTbbFallbackSha256 = 'af20d7ca563e542432b856f6628d9481247197d1853bd4057caaf6c449749d42'
$script:IAmSpeedDotNetSha256 = 'c1809e1f7fc603c2096efdfbc3f98c2123a398d3dff331096fdaebc3071ac32d'
$script:IAmSpeedUbtSha256 = '0c3adf01933fd31497971cbcc3f66315fabe061bbb4c56ac143cd809192e208d'
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
    $runtime = Assert-IAmSpeedDirectUbtRuntime -EngineRoot $engine.Root
    if (Test-Path -LiteralPath (Join-Path $engine.Root 'Engine\Build\InstalledBuild.txt')) {
        throw 'Rules seed must use the exact pinned non-installed slot1 rules source; installed-engine roots use a different UBT skip path.'
    }
    $ubtPath = $runtime.UbtPath
    $ubtSha = $runtime.UbtSha256
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
    return [pscustomobject]@{
        Root=$engine.Root; Build=$engine.Build; DotNetPath=$runtime.DotNetPath; DotNetDirectory=$runtime.DotNetDirectory
        DotNetVersion=$runtime.DotNetVersion; DotNetArchitecture=$runtime.DotNetArchitecture; DotNetSha256=$runtime.DotNetSha256
        UbtPath=$ubtPath; UbtSha256=$ubtSha; WorkingDirectory=$runtime.WorkingDirectory
        RulesDirectory=$rulesDirectory; Assemblies=$assemblyProof
    }
}


function Assert-IAmSpeedPrivateRulesRoot {
    param([Parameter(Mandatory=$true)][psobject]$Runtime)
    if (-not $Runtime.PrivateUbt) { throw 'Private Query requires a manifest-bound private UBT runtime.' }
    $manifest = Assert-IAmSpeedPrivateUbtManifest -Path $Runtime.PrivateManifestPath -Sha256 $Runtime.PrivateManifestSha256 -EngineRoot $Runtime.Root
    $engine = Assert-IAmSpeedEngineRoot -EngineRoot $Runtime.Root
    if (Test-Path -LiteralPath (Join-Path $engine.Root 'Engine\Build\InstalledBuild.txt')) { throw 'Private Query requires the exact non-installed source Engine.' }
    $policy = Get-Content -LiteralPath $manifest.policy_path -Raw | ConvertFrom-Json
    $records = @($policy.engine_rules_records)
    if ($records.Count -ne 6) { throw 'Private Query requires all six exact Engine Rules artifacts.' }
    $rulesDirectory = Join-Path $engine.Root 'Engine\Intermediate\Build\BuildRules'
    $expected = @{}
    foreach ($pin in $script:IAmSpeedRulesSeedFiles) { $expected[$pin.Name]=$pin.Sha256; $expected[$pin.Manifest]=$pin.ManifestSha256 }
    foreach ($pin in $script:IAmSpeedRulesSeedPdbs) { $expected[$pin.Name]=$pin.Sha256 }
    $seen = @{}
    foreach ($record in $records) {
        $name = Split-Path -Leaf $record.path
        $path = Join-Path $rulesDirectory $name
        if (-not $expected.ContainsKey($name) -or $seen.ContainsKey($name) -or [IO.Path]::GetFullPath($record.path) -ine $path -or $record.sha256 -cne $expected[$name]) { throw 'Private Query Rules inventory or provenance differs.' }
        Assert-IAmSpeedPhysicalFile $path
        $item = Get-Item -LiteralPath $path
        if ($item.Length -ne $record.bytes -or $item.LastWriteTimeUtc.Ticks -ne $record.mtime_ticks -or (Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash.ToLowerInvariant() -cne $record.sha256) { throw 'Private Query Rules artifact drift.' }
        $seen[$name]=$true
    }
    $marketplace = Join-Path $engine.Root 'Engine\Plugins\Marketplace'
    if ((Test-Path -LiteralPath $marketplace) -and @(Get-ChildItem -LiteralPath $marketplace -Recurse -File -Filter '*.uplugin').Count -gt 0) { throw 'Unpinned Marketplace Rules are forbidden.' }
    # Absolute E1 source paths remain immutable provenance in the pinned manifests.
    # Private UBT loads these exact DLLs and rejects every Engine compilation fallback.
    $Runtime | Add-Member -NotePropertyName RulesDirectory -NotePropertyValue $rulesDirectory -Force
    $Runtime | Add-Member -NotePropertyName Assemblies -NotePropertyValue $records -Force
    $Runtime | Add-Member -NotePropertyName Build -NotePropertyValue $engine.Build -Force
    return $Runtime
}

function Assert-IAmSpeedPhysicalFile {
    param([Parameter(Mandatory=$true)][string]$Path)
    $current = [IO.Path]::GetFullPath($Path)
    while (-not [string]::IsNullOrWhiteSpace($current)) {
        if (-not (Test-Path -LiteralPath $current)) { throw "Missing physical runtime input: $current" }
        if ((Get-Item -LiteralPath $current -Force).Attributes -band [IO.FileAttributes]::ReparsePoint) { throw "Runtime input alias: $current" }
        $current = [IO.Path]::GetDirectoryName($current)
    }
}

function Assert-IAmSpeedPrivateUbtManifest {
    param([Parameter(Mandatory=$true)][string]$Path, [Parameter(Mandatory=$true)][string]$Sha256, [Parameter(Mandatory=$true)][string]$EngineRoot)
    Assert-IAmSpeedPhysicalFile $Path
    if ((Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant() -cne $Sha256) { throw 'Private UBT manifest SHA differs.' }
    $manifest = Get-Content -LiteralPath $Path -Raw | ConvertFrom-Json
    if ($manifest.schema -cne 'sl.private-ubt-runtime/v1' -or [IO.Path]::GetFullPath($manifest.engine_root).TrimEnd('\') -ine [IO.Path]::GetFullPath($EngineRoot).TrimEnd('\')) { throw 'Private UBT physical Engine binding differs.' }
    $runtimeRoot = [IO.Path]::GetFullPath($manifest.runtime_root).TrimEnd('\')
    $enginePrefix = [IO.Path]::GetFullPath($EngineRoot).TrimEnd('\') + '\'
    if ($runtimeRoot.StartsWith($enginePrefix, [StringComparison]::OrdinalIgnoreCase)) { throw 'Private UBT runtime must be outside Engine.' }
    $seen = @{}
    foreach ($entry in $manifest.files) {
        $file = [IO.Path]::GetFullPath((Join-Path $runtimeRoot ([string]$entry.relative)))
        if (-not $file.StartsWith($runtimeRoot + '\', [StringComparison]::OrdinalIgnoreCase) -or $seen.ContainsKey($file)) { throw 'Private UBT file scope or inventory differs.' }
        Assert-IAmSpeedPhysicalFile $file
        $item = Get-Item -LiteralPath $file
        if ($item.Length -ne $entry.bytes -or (Get-FileHash -LiteralPath $file -Algorithm SHA256).Hash.ToLowerInvariant() -cne $entry.sha256) { throw "Private UBT file drift: $file" }
        $seen[$file] = $true
    }
    if (@(Get-ChildItem -LiteralPath $runtimeRoot -Recurse -File -Force).Count -ne $seen.Count) { throw 'Private UBT extra runtime file.' }
    $ubt = Join-Path $runtimeRoot 'UnrealBuildTool.dll'
    if ([IO.Path]::GetFullPath($manifest.ubt_path) -ine $ubt -or -not $seen.ContainsKey($ubt) -or (Get-FileHash -LiteralPath $ubt -Algorithm SHA256).Hash.ToLowerInvariant() -cne $manifest.ubt_sha256) { throw 'Private UBT entry assembly differs.' }
    Assert-IAmSpeedPhysicalFile $manifest.policy_path
    if ((Get-FileHash -LiteralPath $manifest.policy_path -Algorithm SHA256).Hash.ToLowerInvariant() -cne $manifest.policy_sha256) { throw 'Private Engine policy SHA differs.' }
    $policy = Get-Content -LiteralPath $manifest.policy_path -Raw | ConvertFrom-Json
    if ([IO.Path]::GetFullPath($policy.engine_root).TrimEnd('\') -ine (Join-Path $EngineRoot 'Engine') -or [IO.Path]::GetFullPath($policy.project_private_parent).TrimEnd('\') -ine [IO.Path]::GetFullPath($manifest.baseline_project_parent).TrimEnd('\')) { throw 'Private Engine policy binding differs.' }
    $traceContract = 'Bound root Build only; private Trace required by real UBA non-detour executor. Query and recursive helper modes retain Session trace suppression.'
    if ($manifest.private_root_trace_contract -cne $traceContract -or $policy.private_root_trace_contract -cne $traceContract) { throw 'Private root Build trace contract differs.' }
    return $manifest
}

function Invoke-IAmSpeedPrivateEnginePolicy {
    param([Parameter(Mandatory=$true)][psobject]$Runtime,[Parameter(Mandatory=$true)][scriptblock]$Action)
    $original = [Environment]::GetEnvironmentVariable('SL_PRIVATE_ENGINE_METADATA_POLICY','Process')
    try {
        $policy = $null
        if ($Runtime.PrivateUbt) { $policy = [string]$Runtime.PrivateEnginePolicyPath }
        [Environment]::SetEnvironmentVariable('SL_PRIVATE_ENGINE_METADATA_POLICY',$policy,'Process')
        & $Action
    }
    finally { [Environment]::SetEnvironmentVariable('SL_PRIVATE_ENGINE_METADATA_POLICY',$original,'Process') }
}

function Assert-IAmSpeedDirectUbtRuntime {
    [CmdletBinding()]
    param([Parameter(Mandatory=$true)] [string]$EngineRoot, [string]$PrivateManifestPath, [string]$PrivateManifestSha256)

    $engine = Assert-IAmSpeedEngineRoot -EngineRoot $EngineRoot
    $dotnet = Join-Path $engine.Root 'Engine\Binaries\ThirdParty\DotNet\10.0\win-x64\dotnet.exe'
    $ubt = Join-Path $engine.Root 'Engine\Binaries\DotNET\UnrealBuildTool\UnrealBuildTool.dll'
    $private = $null
    if (-not [string]::IsNullOrWhiteSpace($PrivateManifestPath)) {
        $private = Assert-IAmSpeedPrivateUbtManifest -Path $PrivateManifestPath -Sha256 $PrivateManifestSha256 -EngineRoot $engine.Root
        $ubt = [string]$private.ubt_path
    }
    foreach ($path in @($dotnet, $ubt)) {
        if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Pinned direct UBT runtime file is missing: $path" }
        if ((Get-Item -LiteralPath $path -Force).Attributes -band [IO.FileAttributes]::ReparsePoint) { throw "Pinned direct UBT runtime file is a reparse point: $path" }
    }
    $dotnetSha = (Get-FileHash -LiteralPath $dotnet -Algorithm SHA256).Hash.ToLowerInvariant()
    $ubtSha = (Get-FileHash -LiteralPath $ubt -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($dotnetSha -cne $script:IAmSpeedDotNetSha256) { throw "Pinned .NET host SHA-256 differs: $dotnetSha" }
    if ($null -eq $private -and $ubtSha -cne $script:IAmSpeedUbtSha256) { throw "Pinned UnrealBuildTool SHA-256 differs: $ubtSha" }
    $workingDirectory = Join-Path $engine.Root 'Engine\Source'
    if (-not (Test-Path -LiteralPath $workingDirectory -PathType Container)) { throw "Direct UBT working directory is missing: $workingDirectory" }
    return [pscustomobject]@{ Root=$engine.Root; WorkingDirectory=$workingDirectory; DotNetPath=$dotnet; DotNetDirectory=(Split-Path -Parent $dotnet); DotNetVersion='10.0'; DotNetArchitecture='win-x64'; DotNetSha256=$dotnetSha; UbtPath=$ubt; UbtSha256=$ubtSha; PrivateUbt=($null -ne $private); PrivateManifestPath=$PrivateManifestPath; PrivateManifestSha256=$PrivateManifestSha256; PrivateEnginePolicyPath=$private.policy_path }
}

function New-IAmSpeedDirectUbtInvocation {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory=$true)] [psobject]$Runtime,
        [Parameter(Mandatory=$true)] [string[]]$Arguments
    )
    $root = [IO.Path]::GetFullPath([string]$Runtime.Root).TrimEnd('\')
    $runtimePaths = @([string]$Runtime.DotNetPath)
    if ($Runtime.PrivateUbt) {
        $private = Assert-IAmSpeedPrivateUbtManifest -Path $Runtime.PrivateManifestPath -Sha256 $Runtime.PrivateManifestSha256 -EngineRoot $root
        if ($Runtime.UbtPath -ine $private.ubt_path -or $Runtime.UbtSha256 -cne $private.ubt_sha256) { throw 'Private UBT invocation entry drift.' }
        if ($Arguments -notcontains '-NoEngineChanges' -or $Arguments -notcontains '-UsePrecompiled' -or $Arguments -notcontains '-NoUBA') { throw 'Private UBT requires protected precompiled non-detour build flags.' }
    } else { $runtimePaths += [string]$Runtime.UbtPath }
    foreach ($path in $runtimePaths) {
        $full = [IO.Path]::GetFullPath($path)
        if (-not $full.StartsWith($root + '\', [StringComparison]::OrdinalIgnoreCase)) { throw "Direct UBT runtime path escapes its pinned Engine root: $full" }
    }
    $expectedDotNetDirectory = Split-Path -Parent ([string]$Runtime.DotNetPath)
    if ([IO.Path]::GetFullPath([string]$Runtime.DotNetDirectory).TrimEnd('\') -ine [IO.Path]::GetFullPath($expectedDotNetDirectory).TrimEnd('\') -or
        [string]$Runtime.DotNetVersion -cne '10.0' -or [string]$Runtime.DotNetArchitecture -cne 'win-x64' -or
        [string]$Runtime.DotNetSha256 -cne $script:IAmSpeedDotNetSha256 -or (-not $Runtime.PrivateUbt -and [string]$Runtime.UbtSha256 -cne $script:IAmSpeedUbtSha256)) {
        throw 'Direct UBT runtime metadata differs from the pinned bundled .NET and UBT pair.'
    }
    if (@($Arguments | Where-Object { $_ -cmatch '^-Session=' }).Count -gt 0) { throw 'Direct UBT caller cannot override or duplicate its private UBT session.' }
    $expectedWorkingDirectory = Join-Path $root 'Engine\Source'
    if ([IO.Path]::GetFullPath([string]$Runtime.WorkingDirectory).TrimEnd('\') -ine $expectedWorkingDirectory) { throw 'Direct UBT working directory must match Build.bat Engine\Source context.' }
    $rootArguments = @()
    if ($Runtime.PrivateUbt) {
        if (@($Arguments | Where-Object { $_ -imatch '^-RootDirectory=' }).Count -gt 0) { throw 'Private UBT caller cannot override physical Engine root.' }
        $rootArguments = @("-RootDirectory=$root")
    }
    $session = [guid]::NewGuid().ToString('B')
    return [pscustomobject]@{
        EngineRoot=$root
        WorkingDirectory=$expectedWorkingDirectory
        Executable=[string]$Runtime.DotNetPath
        Arguments=[string[]](@([string]$Runtime.UbtPath) + @($Arguments) + $rootArguments + @("-Session=$session"))
        SessionId=$session
        DotNetDirectory=[string]$Runtime.DotNetDirectory
        DotNetVersion=[string]$Runtime.DotNetVersion
        DotNetArchitecture=[string]$Runtime.DotNetArchitecture
        DotNetSha256=[string]$Runtime.DotNetSha256
        UbtSha256=[string]$Runtime.UbtSha256
    }
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
        '-UsePrecompiled', '-NoEngineChanges', '-NoUBA', '-SLPrivateProjectResources', "-Log=$LogPath"
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
        [string[]]$AdditionalDirectories = @(),
        [AllowNull()] [string]$ExistingPath
    )
    $directory = [IO.Path]::GetFullPath($LoaderDirectory).TrimEnd('\')
    $directories = [System.Collections.Generic.List[string]]::new()
    $directories.Add($directory)
    foreach ($additional in $AdditionalDirectories) {
        if (-not [string]::IsNullOrWhiteSpace($additional)) { $directories.Add([IO.Path]::GetFullPath($additional).TrimEnd('\')) }
    }
    if (-not [string]::IsNullOrWhiteSpace($ExistingPath)) { $directories.Add($ExistingPath) }
    return [string]::Join(';', $directories.ToArray())
}

function Invoke-IAmSpeedWithProcessTbbPath {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory=$true)] [string]$LoaderDirectory,
        [string]$DotNetDirectory,
        [Parameter(Mandatory=$true)] [scriptblock]$Action
    )
    $environmentNames = @('PATH', 'UE_DOTNET_VERSION', 'UE_DOTNET_ARCH', 'UE_DOTNET_DIR', 'DOTNET_ROOT', 'DOTNET_MULTILEVEL_LOOKUP', 'DOTNET_ROLL_FORWARD')
    $originalEnvironment = @{}
    foreach ($name in $environmentNames) { $originalEnvironment[$name] = [Environment]::GetEnvironmentVariable($name, 'Process') }
    try {
        $additionalDirectories = @()
        if (-not [string]::IsNullOrWhiteSpace($DotNetDirectory)) {
            $dotnetDirectoryFull = [IO.Path]::GetFullPath($DotNetDirectory).TrimEnd('\')
            $additionalDirectories = @($dotnetDirectoryFull)
            [Environment]::SetEnvironmentVariable('UE_DOTNET_VERSION', '10.0', 'Process')
            [Environment]::SetEnvironmentVariable('UE_DOTNET_ARCH', 'win-x64', 'Process')
            [Environment]::SetEnvironmentVariable('UE_DOTNET_DIR', $dotnetDirectoryFull, 'Process')
            [Environment]::SetEnvironmentVariable('DOTNET_ROOT', $dotnetDirectoryFull, 'Process')
            [Environment]::SetEnvironmentVariable('DOTNET_MULTILEVEL_LOOKUP', '0', 'Process')
            [Environment]::SetEnvironmentVariable('DOTNET_ROLL_FORWARD', 'LatestMajor', 'Process')
        }
        [Environment]::SetEnvironmentVariable('PATH', (New-IAmSpeedProcessPath -LoaderDirectory $LoaderDirectory -AdditionalDirectories $additionalDirectories -ExistingPath $originalEnvironment['PATH']), 'Process')
        & $Action
    }
    finally {
        foreach ($name in $environmentNames) { [Environment]::SetEnvironmentVariable($name, $originalEnvironment[$name], 'Process') }
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
        '-NoXGE', '-NoFASTBuild', '-NoSNDBS', '-NoUBA', "-MaxParallelActions=$MaxParallelActions",
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
