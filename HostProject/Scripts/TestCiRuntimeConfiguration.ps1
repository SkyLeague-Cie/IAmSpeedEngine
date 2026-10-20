[CmdletBinding()]
param([string]$TestOutputRoot = (Join-Path ([IO.Path]::GetTempPath()) 'IAmSpeed-CiConfigurationTests'))
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'GetCiRuntimeConfiguration.ps1')
$root = Join-Path $TestOutputRoot ([Guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $root | Out-Null
$manifest = Join-Path $root 'manifest.json'
Set-Content -LiteralPath $manifest -Value '{"status":"fixture-only"}' -Encoding UTF8
$good = @{ EngineRoot=(Join-Path $root 'Engine'); RulesEngineRoot=(Join-Path $root 'Engine'); PrivateRoot=(Join-Path $root 'Private'); PrivateUbtManifest=$manifest; PrivateUbtManifestSha256=(Get-FileHash -LiteralPath $manifest -Algorithm SHA256).Hash }
$cases = [System.Collections.Generic.List[string]]::new()
function Expect-Rejection([string]$Name, [hashtable]$Arguments) {
    $rejected = $false
    try { $null = Get-IAmSpeedCiRuntimeConfiguration @Arguments } catch { $rejected = $true }
    if (-not $rejected) { throw "Invalid runner configuration accepted: $Name" }
    $cases.Add($Name)
}
$result = Get-IAmSpeedCiRuntimeConfiguration @good
if (-not $result.AdmissionOnly -or $result.NativeCIQualified -or $result.PrivateUbtManifest -cne $manifest) { throw 'Configuration admission must not qualify native CI.' }
$cases.Add('admission_is_not_native_ci')
foreach ($key in $good.Keys) {
    $bad = $good.Clone(); $bad[$key] = ''; Expect-Rejection ('missing_' + $key) $bad
}
$bad=$good.Clone(); $bad.PrivateUbtManifestSha256='not-a-sha'; Expect-Rejection 'malformed_manifest_pin' $bad
$bad=$good.Clone(); $bad.PrivateUbtManifestSha256=('0'*64); Expect-Rejection 'manifest_pin_drift' $bad
$bad=$good.Clone(); $bad.PrivateUbtManifest=(Join-Path $root 'missing.json'); Expect-Rejection 'missing_manifest' $bad
foreach ($value in @('relative\output', 'C:\good\..\bad', 'C:\good:alternate', '\\server\share\output')) {
    $bad=$good.Clone(); $bad.PrivateRoot=$value; Expect-Rejection ('invalid_output_' + $cases.Count) $bad
}
$report = [ordered]@{ status='PASS_CI_CONFIGURATION_ADMISSION_ONLY'; native_execution='NOT_EXECUTED'; cases=@($cases); helper_sha256=(Get-FileHash -LiteralPath (Join-Path $PSScriptRoot 'GetCiRuntimeConfiguration.ps1') -Algorithm SHA256).Hash.ToLowerInvariant(); raw_fixture_root=$root }
$report | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $root 'result.json') -Encoding UTF8
$report | ConvertTo-Json -Depth 6 -Compress
