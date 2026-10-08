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
    if ([string]::IsNullOrWhiteSpace($rulesSeedRoot)) { throw 'Explicit private Query Engine root required.' }
    $rulesSeedRuntime = Assert-IAmSpeedDirectUbtRuntime -EngineRoot $rulesSeedRoot -PrivateManifestPath $env:IAMSPEED_PRIVATE_UBT_MANIFEST -PrivateManifestSha256 $env:IAMSPEED_PRIVATE_UBT_MANIFEST_SHA256
    $rulesSeed = Assert-IAmSpeedPrivateRulesRoot -Runtime $rulesSeedRuntime
    $expectedRulesSeedRoot = (Resolve-Path -LiteralPath $rulesSeedRoot).Path.TrimEnd('\')
    $expectedRulesSeedBuild = Join-Path $expectedRulesSeedRoot 'Engine\Build\BatchFiles\Build.bat'
    Assert-True ($rulesSeed.Root -ceq $expectedRulesSeedRoot -and $rulesSeed.Build -ceq $expectedRulesSeedBuild -and @($rulesSeed.Assemblies).Count -eq 6) 'Private same-root Query validates all six immutable Rules artifacts'
    $expectedDotNet = Join-Path $expectedRulesSeedRoot 'Engine\Binaries\ThirdParty\DotNet\10.0\win-x64\dotnet.exe'
    $expectedUbt = $rulesSeedRuntime.UbtPath
    Assert-True ($rulesSeedRuntime.DotNetPath -ceq $expectedDotNet -and $rulesSeedRuntime.PrivateUbt) 'Private Query uses manifest-bound runtime'
    Assert-Throws { Assert-IAmSpeedPrivateRulesRoot -Runtime (Assert-IAmSpeedDirectUbtRuntime -EngineRoot $rulesSeedRoot) } 'Stock Query cannot bypass private writer confinement'
    $foreign = $rulesSeedRuntime | Select-Object *
    $foreign.Root=$EngineFixture
    Assert-Throws { Assert-IAmSpeedPrivateRulesRoot -Runtime $foreign } 'Foreign Engine cannot reuse D Rules policy'

    $targetEngineRoot = $env:IAMSPEED_UE_ROOT
    if ([string]::IsNullOrWhiteSpace($targetEngineRoot)) {
        throw 'IAMSPEED_UE_ROOT is required to validate the real target Engine direct UBT runtime.'
    }
    $targetRuntime = Assert-IAmSpeedDirectUbtRuntime -EngineRoot $targetEngineRoot -PrivateManifestPath $env:IAMSPEED_PRIVATE_UBT_MANIFEST -PrivateManifestSha256 $env:IAMSPEED_PRIVATE_UBT_MANIFEST_SHA256
    $targetExpectedRoot = (Resolve-Path -LiteralPath $targetEngineRoot).Path.TrimEnd('\')
    $targetExpectedUbt = $targetRuntime.UbtPath
    $targetBuildArguments = New-IAmSpeedBuildArguments -ProjectFile 'D:\Private\HostProject.uproject' `
        -LogPath 'D:\Private\Logs\build.log' -UbaRoot 'D:\Private\UBA' -SkipRulesCompile
    $targetInvocation = New-IAmSpeedDirectUbtInvocation -Runtime $targetRuntime -Arguments $targetBuildArguments
    Assert-True ($targetInvocation.EngineRoot -ceq $targetExpectedRoot -and $targetInvocation.WorkingDirectory -ceq (Join-Path $targetExpectedRoot 'Engine\Source')) 'target UBT working directory matches Build.bat Engine\Source context'
    Assert-True ($targetInvocation.Executable -ceq $targetRuntime.DotNetPath -and $targetInvocation.Arguments[0] -ceq $targetExpectedUbt) 'target invocation uses pinned dotnet with UBT DLL as its first argument'
    Assert-True ($targetInvocation.Arguments -ccontains '-NoEngineChanges' -and $targetInvocation.Arguments -ccontains '-UsePrecompiled' -and $targetInvocation.Arguments -ccontains '-SkipRulesCompile') 'target invocation preserves protected Engine and precompiled project-rules flags'
    Assert-True (@($targetInvocation.Arguments | Where-Object { $_ -cmatch '^-Session=\{[0-9a-fA-F-]{36}\}$' }).Count -eq 1) 'target invocation supplies an explicit UBT session before trace initialization'
    Assert-Throws { New-IAmSpeedDirectUbtInvocation -Runtime ([pscustomobject]@{
        Root=$targetExpectedRoot; WorkingDirectory=$targetExpectedRoot; DotNetPath='C:\fake\dotnet.exe';
        UbtPath=$targetExpectedUbt; DotNetSha256=$script:IAmSpeedDotNetSha256; UbtSha256=$script:IAmSpeedUbtSha256
    }) -Arguments $targetBuildArguments } 'direct invocation rejects runtime paths outside the pinned Engine root'


    if (-not [string]::IsNullOrWhiteSpace($env:IAMSPEED_PRIVATE_UBT_MANIFEST)) {
        $privateRuntime = Assert-IAmSpeedDirectUbtRuntime -EngineRoot $targetEngineRoot -PrivateManifestPath $env:IAMSPEED_PRIVATE_UBT_MANIFEST -PrivateManifestSha256 $env:IAMSPEED_PRIVATE_UBT_MANIFEST_SHA256
        $privateManifest = Get-Content -LiteralPath $env:IAMSPEED_PRIVATE_UBT_MANIFEST -Raw | ConvertFrom-Json
        $privatePolicy = Get-Content -LiteralPath $privateRuntime.PrivateEnginePolicyPath -Raw | ConvertFrom-Json
        $invalidPolicy = $privatePolicy | ConvertTo-Json -Depth 100 | ConvertFrom-Json
        $invalidPolicy.preserved_runtime_copies[0].target = Join-Path $targetEngineRoot 'Engine\Binaries\Win64\AgentInterface.dll'
        $invalidPolicyPath = Join-Path $FixtureRoot 'policy-engine-target.json'
        [IO.File]::WriteAllText($invalidPolicyPath, ($invalidPolicy | ConvertTo-Json -Depth 100), [Text.UTF8Encoding]::new($false))
        $invalidManifest = $privateManifest | ConvertTo-Json -Depth 100 | ConvertFrom-Json
        $invalidManifest.policy_path = $invalidPolicyPath
        $invalidManifest.policy_sha256 = (Get-FileHash -LiteralPath $invalidPolicyPath -Algorithm SHA256).Hash.ToLowerInvariant()
        $invalidManifestPath = Join-Path $FixtureRoot 'manifest-engine-target.json'
        [IO.File]::WriteAllText($invalidManifestPath, ($invalidManifest | ConvertTo-Json -Depth 100), [Text.UTF8Encoding]::new($false))
        $invalidManifestSha = (Get-FileHash -LiteralPath $invalidManifestPath -Algorithm SHA256).Hash.ToLowerInvariant()
        Assert-Throws { Assert-IAmSpeedPrivateUbtManifest -Path $invalidManifestPath -Sha256 $invalidManifestSha -EngineRoot $targetEngineRoot } 'private runtime DLL target on D is rejected'
        $d3d12ApprovedTargets = @(
            (Join-Path $targetEngineRoot 'Engine\Binaries\Win64\D3D12\x64\D3D12Core.dll'),
            (Join-Path $targetEngineRoot 'Engine\Binaries\Win64\D3D12\x64\d3d12SDKLayers.dll')
        )
        $d3d12ApprovedCopies = @($privatePolicy.preserved_runtime_copies | Where-Object { $_.target -in $d3d12ApprovedTargets })
        Assert-True ($d3d12ApprovedCopies.Count -eq 2) 'private policy pins exactly the two pre-existing D3D12 Engine inputs'
        foreach ($copy in $d3d12ApprovedCopies) {
            Assert-True ($copy.source_sha256 -ceq $copy.target_sha256 -and $copy.target -in $d3d12ApprovedTargets) 'D3D12 Engine copy pair is byte-identical and precisely scoped'
        }
        $eosTarget = Join-Path $targetEngineRoot 'Engine\Binaries\Win64\EOSSDK-Win64-Shipping.dll'
        $eosSource = Join-Path $targetEngineRoot 'Engine\Source\ThirdParty\EOSSDK\SDK\Bin\EOSSDK-Win64-Shipping.dll'
        $eosApprovedCopies = @($privatePolicy.preserved_runtime_copies | Where-Object { $_.target -ceq $eosTarget -and $_.source -ceq $eosSource })
        Assert-True ($eosApprovedCopies.Count -eq 1 -and $eosApprovedCopies[0].source_sha256 -ceq $eosApprovedCopies[0].target_sha256 -and $eosApprovedCopies[0].source_mtime_ticks -eq $eosApprovedCopies[0].target_mtime_ticks) 'EOS Engine copy is an exact immutable input already pinned to E'
        $nneTarget = Join-Path $targetEngineRoot 'Engine\Binaries\Win64\NNEEditorOnnxTools.dll'
        $nneSource = Join-Path $targetEngineRoot 'Engine\Source\Editor\NNEEditor\Bin\Win64\NNEEditorOnnxTools.dll'
        $nneCopies = @($privatePolicy.preserved_runtime_copies | Where-Object { $_.target -ceq $nneTarget -and $_.source -ceq $nneSource })
        Assert-True ($nneCopies.Count -eq 1 -and $nneCopies[0].source_sha256 -ceq $nneCopies[0].target_sha256 -and $nneCopies[0].source_mtime_ticks -eq $nneCopies[0].target_mtime_ticks) 'NNE editor tools D runtime pair is byte-identical and precisely scoped'
        $invalidPolicy.preserved_runtime_copies = @($privatePolicy.preserved_runtime_copies | ForEach-Object { $_ | ConvertTo-Json -Depth 20 | ConvertFrom-Json })
        $invalidPolicy.preserved_runtime_copies | Where-Object { $_.target -ceq $eosTarget } | ForEach-Object { $_.source = Join-Path $targetEngineRoot 'Engine\Binaries\Win64\AgentInterface.dll' }
        [IO.File]::WriteAllText($invalidPolicyPath, ($invalidPolicy | ConvertTo-Json -Depth 100), [Text.UTF8Encoding]::new($false))
        $invalidManifest.policy_sha256 = (Get-FileHash -LiteralPath $invalidPolicyPath -Algorithm SHA256).Hash.ToLowerInvariant()
        [IO.File]::WriteAllText($invalidManifestPath, ($invalidManifest | ConvertTo-Json -Depth 100), [Text.UTF8Encoding]::new($false))
        $invalidManifestSha = (Get-FileHash -LiteralPath $invalidManifestPath -Algorithm SHA256).Hash.ToLowerInvariant()
        Assert-Throws { Assert-IAmSpeedPrivateUbtManifest -Path $invalidManifestPath -Sha256 $invalidManifestSha -EngineRoot $targetEngineRoot } 'EOS target with mismatched source is rejected'
        $invalidPolicy.preserved_runtime_copies = @($privatePolicy.preserved_runtime_copies | ForEach-Object { $_ | ConvertTo-Json -Depth 20 | ConvertFrom-Json })
        $invalidPolicy.preserved_runtime_copies | Where-Object { $_.target -ceq $d3d12ApprovedTargets[0] } | ForEach-Object { $_.source = Join-Path $targetEngineRoot 'Engine\Binaries\Win64\AgentInterface.dll' }
        [IO.File]::WriteAllText($invalidPolicyPath, ($invalidPolicy | ConvertTo-Json -Depth 100), [Text.UTF8Encoding]::new($false))
        $invalidManifest.policy_sha256 = (Get-FileHash -LiteralPath $invalidPolicyPath -Algorithm SHA256).Hash.ToLowerInvariant()
        [IO.File]::WriteAllText($invalidManifestPath, ($invalidManifest | ConvertTo-Json -Depth 100), [Text.UTF8Encoding]::new($false))
        $invalidManifestSha = (Get-FileHash -LiteralPath $invalidManifestPath -Algorithm SHA256).Hash.ToLowerInvariant()
        Assert-Throws { Assert-IAmSpeedPrivateUbtManifest -Path $invalidManifestPath -Sha256 $invalidManifestSha -EngineRoot $targetEngineRoot } 'D3D12 Engine target with a mismatched source is rejected'
        $invalidPolicy.preserved_runtime_copies[0].target = Join-Path ([string]$privateManifest.private_root) 'Foreign\AgentInterface.dll'
        [IO.File]::WriteAllText($invalidPolicyPath, ($invalidPolicy | ConvertTo-Json -Depth 100), [Text.UTF8Encoding]::new($false))
        $invalidManifest.policy_sha256 = (Get-FileHash -LiteralPath $invalidPolicyPath -Algorithm SHA256).Hash.ToLowerInvariant()
        [IO.File]::WriteAllText($invalidManifestPath, ($invalidManifest | ConvertTo-Json -Depth 100), [Text.UTF8Encoding]::new($false))
        $invalidManifestSha = (Get-FileHash -LiteralPath $invalidManifestPath -Algorithm SHA256).Hash.ToLowerInvariant()
        Assert-Throws { Assert-IAmSpeedPrivateUbtManifest -Path $invalidManifestPath -Sha256 $invalidManifestSha -EngineRoot $targetEngineRoot } 'private runtime DLL target outside EnginePrivate is rejected'
        $expectedCache = $privatePolicy.private_cache_root
        $privateCacheBinding = Assert-IAmSpeedPrivateCacheBinding -Runtime $privateRuntime -ExpectedCacheRoot $expectedCache
        Assert-True ($privateCacheBinding.ManifestParent -ceq $privateCacheBinding.PolicyParent) 'private UBT manifest and policy bind the same private root'
        Assert-Throws { Assert-IAmSpeedPrivateCacheBinding -Runtime $privateRuntime -ExpectedCacheRoot (Join-Path $env:TEMP 'wrong-private-cache') } 'stale private UBT cache root is rejected before execution'
        $privateInvocation = New-IAmSpeedDirectUbtInvocation -Runtime $privateRuntime -Arguments $targetBuildArguments
        Assert-True ($privateRuntime.PrivateUbt -and $privateInvocation.Arguments[0] -ceq $privateRuntime.UbtPath -and $privateInvocation.Arguments -ccontains '-NoUBA' -and $privateInvocation.Arguments -ccontains "-RootDirectory=$targetExpectedRoot") 'private UBT preserves physical Engine root, NoUBA and exact private entry'
        Assert-Throws { Assert-IAmSpeedPrivateUbtManifest -Path $env:IAMSPEED_PRIVATE_UBT_MANIFEST -Sha256 ('0' * 64) -EngineRoot $targetEngineRoot } 'private runtime manifest digest drift'
        Assert-Throws { New-IAmSpeedDirectUbtInvocation -Runtime $privateRuntime -Arguments @($targetBuildArguments + '-RootDirectory=C:\Foreign') } 'private root override'
        Assert-Throws { New-IAmSpeedDirectUbtInvocation -Runtime $privateRuntime -Arguments @($targetBuildArguments | Where-Object {$_ -cne '-NoUBA'}) } 'private route requires non-detour UBA'
        $originalPolicy = [Environment]::GetEnvironmentVariable('SL_PRIVATE_ENGINE_METADATA_POLICY','Process')
        $seenPolicy = Invoke-IAmSpeedPrivateEnginePolicy -Runtime $privateRuntime -Action { [Environment]::GetEnvironmentVariable('SL_PRIVATE_ENGINE_METADATA_POLICY','Process') }
        Assert-True ($seenPolicy -ceq $privateRuntime.PrivateEnginePolicyPath) 'D build policy enabled in process scope'
        Assert-True ([Environment]::GetEnvironmentVariable('SL_PRIVATE_ENGINE_METADATA_POLICY','Process') -ceq $originalPolicy) 'policy restored after success'
        Assert-Throws { Invoke-IAmSpeedPrivateEnginePolicy -Runtime $privateRuntime -Action {throw 'fixture stop'} } 'policy action failure'
        Assert-True ([Environment]::GetEnvironmentVariable('SL_PRIVATE_ENGINE_METADATA_POLICY','Process') -ceq $originalPolicy) 'policy restored after failure'
        $seedPolicy = Invoke-IAmSpeedPrivateEnginePolicy -Runtime $rulesSeedRuntime -Action { [Environment]::GetEnvironmentVariable('SL_PRIVATE_ENGINE_METADATA_POLICY','Process') }
        Assert-True ($seedPolicy -ceq $privateRuntime.PrivateEnginePolicyPath) 'private Query and build use the same exact D policy'
    }

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

    Assert-Throws { Resolve-IAmSpeedEmbreeLoader -EngineRoot $EngineFixture } 'missing Embree runtime is rejected'
    $EmbreeBinary = Join-Path $TbbBinaryDirectory 'embree4.dll'
    [IO.File]::WriteAllText($EmbreeBinary, 'engine-binary-embree')
    $binaryEmbreeSha = (Get-FileHash -LiteralPath $EmbreeBinary -Algorithm SHA256).Hash.ToLowerInvariant()
    $binaryEmbree = Resolve-IAmSpeedEmbreeLoader -EngineRoot $EngineFixture -ExpectedFallbackSha256 $binaryEmbreeSha
    Assert-True ($binaryEmbree.Source -ceq 'EngineBinaries' -and $binaryEmbree.DllPath -eq $EmbreeBinary -and
        $binaryEmbree.Sha256Pinned -and $binaryEmbree.Bytes -eq [IO.FileInfo]::new($EmbreeBinary).Length) 'engine binary Embree runtime is hash-pinned'
    [IO.File]::AppendAllText($EmbreeBinary, '-mutated')
    Assert-Throws { Assert-IAmSpeedResolvedRuntimeDllUnchanged -Loader $binaryEmbree } 'Embree runtime drift before/after Editor is rejected'
    [IO.File]::WriteAllText($EmbreeBinary, 'engine-binary-embree')
    Remove-Item -LiteralPath $EmbreeBinary -Force
    $EmbreeFallback = Join-Path $EngineFixture 'Engine\Source\ThirdParty\Intel\Embree\Deploy\embree-4.3.3\VS2015\x64\bin\embree4.dll'
    New-Item -ItemType Directory -Path (Split-Path -Parent $EmbreeFallback) -Force | Out-Null
    [IO.File]::WriteAllText($EmbreeFallback, 'fixture-fallback-embree')
    $fixtureEmbreeSha = (Get-FileHash -LiteralPath $EmbreeFallback -Algorithm SHA256).Hash.ToLowerInvariant()
    $fallbackEmbree = Resolve-IAmSpeedEmbreeLoader -EngineRoot $EngineFixture -ExpectedFallbackSha256 $fixtureEmbreeSha
    Assert-True ($fallbackEmbree.Source -ceq 'PinnedThirdPartyFallback' -and $fallbackEmbree.FallbackShaValidated -and
        $fallbackEmbree.Sha256Pinned -and $fallbackEmbree.Sha256 -ceq $fixtureEmbreeSha) 'fallback Embree runtime requires and records its qualified SHA'
    Assert-True ((Assert-IAmSpeedResolvedRuntimeDllUnchanged -Loader $fallbackEmbree).sha256 -ceq $fixtureEmbreeSha) 'pinned fallback Embree runtime is unchanged at load boundaries'
    Assert-Throws { Resolve-IAmSpeedEmbreeLoader -EngineRoot $EngineFixture } 'fallback Embree runtime with a nonqualified SHA is rejected'

    $originalPath = [Environment]::GetEnvironmentVariable('PATH', 'Process')
    $OriginalEnvironment = @{}
    foreach ($variable in @('UE_DOTNET_VERSION', 'UE_DOTNET_ARCH', 'UE_DOTNET_DIR', 'DOTNET_ROOT', 'DOTNET_MULTILEVEL_LOOKUP', 'DOTNET_ROLL_FORWARD')) {
        $OriginalEnvironment[$variable] = [Environment]::GetEnvironmentVariable($variable, 'Process')
    }
    $pathResult = Invoke-IAmSpeedWithProcessTbbPath -LoaderDirectory $TbbBinaryDirectory -Action {
        [Environment]::GetEnvironmentVariable('PATH', 'Process')
    }
    Assert-True ($pathResult.StartsWith($TbbBinaryDirectory + ';', [StringComparison]::OrdinalIgnoreCase)) 'private TBB directory is process-PATH prefixed'
    Assert-True ([Environment]::GetEnvironmentVariable('PATH', 'Process') -ceq $originalPath) 'process PATH is restored after successful action'
    $embreePathResult = Invoke-IAmSpeedWithProcessTbbPath -LoaderDirectory $TbbBinaryDirectory -AdditionalDirectories @((Split-Path -Parent $EmbreeFallback)) -Action {
        [Environment]::GetEnvironmentVariable('PATH', 'Process')
    }
    Assert-True ($embreePathResult.StartsWith($TbbBinaryDirectory + ';' + (Split-Path -Parent $EmbreeFallback) + ';', [StringComparison]::OrdinalIgnoreCase)) 'pinned Embree runtime directory is process-PATH scoped after TBB'
    Assert-True ([Environment]::GetEnvironmentVariable('PATH', 'Process') -ceq $originalPath) 'process PATH is restored after Embree action'
    Assert-Throws { Invoke-IAmSpeedWithProcessTbbPath -LoaderDirectory $TbbBinaryDirectory -Action { throw 'fixture action failure' } } 'TBB action exception is propagated'
    Assert-True ([Environment]::GetEnvironmentVariable('PATH', 'Process') -ceq $originalPath) 'process PATH is restored after failed action'
    $dotnetEnvironment = Invoke-IAmSpeedWithProcessTbbPath -LoaderDirectory $TbbBinaryDirectory -DotNetDirectory $TbbBinaryDirectory -Action {
        [pscustomobject]@{
            Path=[Environment]::GetEnvironmentVariable('PATH', 'Process')
            Version=[Environment]::GetEnvironmentVariable('UE_DOTNET_VERSION', 'Process')
            Architecture=[Environment]::GetEnvironmentVariable('UE_DOTNET_ARCH', 'Process')
            Root=[Environment]::GetEnvironmentVariable('DOTNET_ROOT', 'Process')
            Multilevel=[Environment]::GetEnvironmentVariable('DOTNET_MULTILEVEL_LOOKUP', 'Process')
            RollForward=[Environment]::GetEnvironmentVariable('DOTNET_ROLL_FORWARD', 'Process')
        }
    }
    Assert-True ($dotnetEnvironment.Path.StartsWith($TbbBinaryDirectory + ';' + $TbbBinaryDirectory + ';', [StringComparison]::OrdinalIgnoreCase)) 'direct UBT PATH preserves TBB-first and bundled .NET directory ordering'
    Assert-True ($dotnetEnvironment.Version -ceq '10.0' -and $dotnetEnvironment.Architecture -ceq 'win-x64' -and
        $dotnetEnvironment.Root -ceq $TbbBinaryDirectory -and $dotnetEnvironment.Multilevel -ceq '0' -and
        $dotnetEnvironment.RollForward -ceq 'LatestMajor') 'direct UBT process reproduces Build.bat bundled .NET environment'
    Assert-True ([Environment]::GetEnvironmentVariable('PATH', 'Process') -ceq $originalPath) 'PATH is restored after direct .NET environment action'
    foreach ($variable in @('UE_DOTNET_VERSION', 'UE_DOTNET_ARCH', 'UE_DOTNET_DIR', 'DOTNET_ROOT', 'DOTNET_MULTILEVEL_LOOKUP', 'DOTNET_ROLL_FORWARD')) {
        $beforeValue = $OriginalEnvironment[$variable]
        $afterValue = [Environment]::GetEnvironmentVariable($variable, 'Process')
        $sameValue = ($null -eq $beforeValue -and [string]::IsNullOrEmpty($afterValue)) -or
            ($null -eq $afterValue -and [string]::IsNullOrEmpty($beforeValue)) -or
            ([string]$afterValue -ceq [string]$beforeValue)
        Assert-True $sameValue "direct .NET variable $variable is restored"
    }

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
    # UBA remains the actual executor with -NoUBA; the manifest-bound private runtime provides its root Build trace under -Session.
    Assert-True ($buildArgs -ccontains '-NoUBA') 'protected session route requires UBA non-detour mode'
    Assert-True ($buildArgs -cnotcontains '-SkipRulesCompile') 'default build can compile fresh project rules'
    $skipBuildArgs = New-IAmSpeedBuildArguments -ProjectFile 'D:\Private\HostProject.uproject' `
        -LogPath 'D:\Private\Logs\build.log' -UbaRoot 'D:\Private\UBA' -SkipRulesCompile
    Assert-True ($skipBuildArgs -ccontains '-SkipRulesCompile') 'target build can load the already staged project rules without compiling Engine rules'

    $rulesQueryArgs = New-IAmSpeedProjectRulesQueryArguments -ProjectFile 'D:\Private\HostProject\IAmSpeedHostProject.uproject' `
        -OutputPath 'D:\Private\HostProject\Intermediate\TargetInfo.json' -LogPath 'D:\Private\Logs\RulesQuery.log'
    Assert-True ($rulesQueryArgs -ccontains '-Mode=QueryTargets') 'rules preparation uses UBT QueryTargets only'
    Assert-True ($rulesQueryArgs -ccontains '-UsePrecompiled') 'rules preparation loads the pinned seed Engine rules'
    Assert-True ($rulesQueryArgs -cnotcontains '-SkipRulesCompile') 'QueryTargets compiles the fresh private project rules'
    Assert-True ($rulesQueryArgs -ccontains '-IncludeAllTargets') 'rules query includes all project targets like the Editor fallback'
    Assert-True ($rulesQueryArgs -ccontains '-DontIncludeParentAssembly') 'rules query excludes parent assembly like the Editor fallback'
    Assert-True ($rulesQueryArgs -ccontains '-DontIncludeProgramTargets') 'rules query excludes programs like the Editor fallback'
    Assert-True ($rulesQueryArgs -ccontains '-Output=D:\Private\HostProject\Intermediate\TargetInfo.json') 'QueryTargets writes directly to the Editor cache path'
    $queryInvocation = New-IAmSpeedDirectUbtInvocation -Runtime $rulesSeed -Arguments $rulesQueryArgs
    Assert-True ($queryInvocation.Executable -ceq $expectedDotNet -and $queryInvocation.EngineRoot -ceq $expectedRulesSeedRoot -and $queryInvocation.Arguments[0] -ceq $expectedUbt) 'QueryTargets uses the exact private runtime and D Engine root'
    Assert-True ($queryInvocation.WorkingDirectory -ceq (Join-Path $expectedRulesSeedRoot 'Engine\Source')) 'QueryTargets retains Build.bat Engine\Source working directory'
    Assert-True (@($queryInvocation.Arguments | Where-Object { $_ -cmatch '^-Session=\{[0-9a-fA-F-]{36}\}$' }).Count -eq 1) 'QueryTargets supplies an explicit UBT session before trace initialization'
    Assert-True ($queryInvocation.SessionId -cne $targetInvocation.SessionId) 'QueryTargets and target build have distinct private UBT sessions'
    Assert-True ($queryInvocation.Arguments -ccontains '-Mode=QueryTargets' -and $queryInvocation.Arguments -ccontains '-UsePrecompiled' -and $queryInvocation.Arguments -ccontains '-NoEngineChanges' -and $queryInvocation.Arguments -cnotcontains '-SkipRulesCompile') 'QueryTargets keeps private rules compilation while protecting Engine rules'

    $TargetInfoFixtureRoot = Join-Path $FixtureRoot 'TargetInfoProject'
    $TargetInfoSource = Join-Path $TargetInfoFixtureRoot 'Source'
    $TargetInfoIntermediate = Join-Path $TargetInfoFixtureRoot 'Intermediate'
    New-Item -ItemType Directory -Path $TargetInfoSource,$TargetInfoIntermediate -Force | Out-Null
    $TargetInfoGameSource = Join-Path $TargetInfoSource 'FixtureGame.Target.cs'
    $TargetInfoEditorSource = Join-Path $TargetInfoSource 'FixtureGameEditor.Target.cs'
    $TargetInfoFixturePath = Join-Path $TargetInfoIntermediate 'TargetInfo.json'
    [IO.File]::WriteAllText($TargetInfoGameSource, '// fixture target')
    [IO.File]::WriteAllText($TargetInfoEditorSource, '// fixture target')
    [IO.File]::WriteAllText($TargetInfoFixturePath, (@{ Targets=@(
        @{ Name='FixtureGame'; Path='..\Source\FixtureGame.Target.cs'; Type='Game' },
        @{ Name='FixtureGameEditor'; Path='..\Source\FixtureGameEditor.Target.cs'; Type='Editor' }
    ) } | ConvertTo-Json -Depth 5))
    (Get-Item -LiteralPath $TargetInfoGameSource).LastWriteTimeUtc = [DateTime]::UtcNow.AddMinutes(-2)
    (Get-Item -LiteralPath $TargetInfoEditorSource).LastWriteTimeUtc = [DateTime]::UtcNow.AddMinutes(-2)
    (Get-Item -LiteralPath $TargetInfoFixturePath).LastWriteTimeUtc = [DateTime]::UtcNow
    $targetInfoFresh = Assert-IAmSpeedEditorTargetInfoFresh -TargetInfoPath $TargetInfoFixturePath -ProjectRoot $TargetInfoFixtureRoot
    Assert-True ($targetInfoFresh.target_count -eq 2 -and $targetInfoFresh.project_target_source_count -eq 2) 'fresh Editor TargetInfo resolves both project targets'
    (Get-Item -LiteralPath $TargetInfoEditorSource).LastWriteTimeUtc = [DateTime]::UtcNow.AddMinutes(1)
    Assert-Throws { Assert-IAmSpeedEditorTargetInfoFresh -TargetInfoPath $TargetInfoFixturePath -ProjectRoot $TargetInfoFixtureRoot } 'newer target source invalidates Editor TargetInfo'
    (Get-Item -LiteralPath $TargetInfoEditorSource).LastWriteTimeUtc = [DateTime]::UtcNow.AddMinutes(-2)
    [IO.File]::WriteAllText($TargetInfoFixturePath, (@{ Targets=@(
        @{ Name='MissingTarget'; Path='..\Source\FixtureGame.Target.cs'; Type='Game' }
    ) } | ConvertTo-Json -Depth 5))
    (Get-Item -LiteralPath $TargetInfoFixturePath).LastWriteTimeUtc = [DateTime]::UtcNow
    Assert-Throws { Assert-IAmSpeedEditorTargetInfoFresh -TargetInfoPath $TargetInfoFixturePath -ProjectRoot $TargetInfoFixtureRoot } 'unknown Editor target invalidates TargetInfo'

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
    $runnerSource = Get-Content -LiteralPath $RunnerPath -Raw
    Assert-True ($runnerSource.Contains('Intermediate\TargetInfo.json')) 'Editor QueryTargets output is primed at DesktopPlatform TargetInfo path'
    Assert-True ($runnerSource.Contains('Editor ignored the fresh private TargetInfo.json')) 'Editor fallback to Engine Build.bat is rejected'
Write-Output 'PASS split UBT rules preparation, six-file Engine-rule immutability, process-only TBB and Embree runtime resolution, bounded build, strict reports, and PATH restoration'
}
finally {
    $resolvedFixture = [IO.Path]::GetFullPath($FixtureRoot).TrimEnd('\')
    if (-not $resolvedFixture.StartsWith($TempBase + '\', [StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing to remove fixture outside TEMP: $resolvedFixture"
    }
    Remove-Item -LiteralPath $resolvedFixture -Recurse -Force
}
