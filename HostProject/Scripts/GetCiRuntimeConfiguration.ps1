function Get-IAmSpeedCiRuntimeConfiguration {
    [CmdletBinding()]
    param(
        [AllowNull()] [string]$EngineRoot = $env:IAMSPEED_UE_ROOT,
        [AllowNull()] [string]$RulesEngineRoot = $env:IAMSPEED_RULES_ENGINE_ROOT,
        [AllowNull()] [string]$PrivateRoot = $env:IAMSPEED_PRIVATE_ROOT,
        [AllowNull()] [string]$PrivateUbtManifest = $env:IAMSPEED_PRIVATE_UBT_MANIFEST,
        [AllowNull()] [string]$PrivateUbtManifestSha256 = $env:IAMSPEED_PRIVATE_UBT_MANIFEST_SHA256
    )
    # Repository variables select an already qualified runner tuple. This helper
    # checks location/pin admission only; RunContracts retains all native gates.
    $paths = [ordered]@{
        IAMSPEED_UE_ROOT=$EngineRoot
        IAMSPEED_RULES_ENGINE_ROOT=$RulesEngineRoot
        IAMSPEED_PRIVATE_ROOT=$PrivateRoot
        IAMSPEED_PRIVATE_UBT_MANIFEST=$PrivateUbtManifest
    }
    foreach ($name in $paths.Keys) {
        $value = [string]$paths[$name]
        if ([string]::IsNullOrWhiteSpace($value) -or
            $value -cnotmatch '^[A-Za-z]:[\\/]' -or
            $value -match '(^|[\\/])\.\.([\\/]|$)' -or
            $value.Substring(2).Contains(':')) {
            throw "Configure $name with an absolute local path without traversal or alternate streams."
        }
    }
    if ($PrivateUbtManifestSha256 -cnotmatch '^[0-9a-fA-F]{64}$') {
        throw 'Configure IAMSPEED_PRIVATE_UBT_MANIFEST_SHA256 with exactly 64 hexadecimal characters.'
    }
    if (-not (Test-Path -LiteralPath $PrivateUbtManifest -PathType Leaf)) {
        throw "The configured private UBT manifest is missing: $PrivateUbtManifest"
    }
    $expected = $PrivateUbtManifestSha256.ToLowerInvariant()
    if ((Get-FileHash -LiteralPath $PrivateUbtManifest -Algorithm SHA256).Hash.ToLowerInvariant() -cne $expected) {
        throw 'The runner private UBT manifest differs from the repository SHA-256 pin.'
    }
    return [pscustomobject]@{
        EngineRoot=[IO.Path]::GetFullPath($EngineRoot)
        RulesEngineRoot=[IO.Path]::GetFullPath($RulesEngineRoot)
        PrivateRoot=[IO.Path]::GetFullPath($PrivateRoot)
        PrivateUbtManifest=[IO.Path]::GetFullPath($PrivateUbtManifest)
        PrivateUbtManifestSha256=$expected
        AdmissionOnly=$true
        NativeCIQualified=$false
    }
}
