param(
    [Parameter(Mandatory = $true)][string]$VcVars64,
    [Parameter(Mandatory = $true)][string]$OutputDirectory,
    [Parameter(Mandatory = $true)][string]$Summarizer,
    [string]$GameInputIncludePath,
    [ValidateSet('DeviceDiscoveryProbe', 'DeviceActivityProbe', 'WindowsDeviceDiscoveryProbe', 'WindowsSelectedSourceProbe', 'WindowsActivitySourceProbe', 'WindowsCallbackReadingsProbe', 'WindowsWarmCatalogueProbe')][string]$ProbeName = 'DeviceDiscoveryProbe',
    [string]$Python = 'python'
)
$ErrorActionPreference = 'Stop'
$moduleRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$sdkFlags = ''
if ($ProbeName.StartsWith('Windows')) {
    if (!(Test-Path -LiteralPath (Join-Path $GameInputIncludePath 'GameInput.h'))) { throw 'Windows probe requires GameInput v3 headers' }
    $sdkFlags = "/external:I`"$GameInputIncludePath`" /external:W0"
}
$outputPath = [IO.Path]::GetFullPath($OutputDirectory)
if (Test-Path -LiteralPath $outputPath) { throw "Use a fresh output directory: $outputPath" }
New-Item -ItemType Directory -Path $outputPath | Out-Null
$batch = @"
@echo off
call "$VcVars64" -vcvars_ver=14.38
if errorlevel 1 exit /b %errorlevel%
cl /nologo /std:c++17 /EHsc /W4 /WX /O2 /I"$moduleRoot\Source" $sdkFlags "$PSScriptRoot\${ProbeName}.cpp" "$moduleRoot\Source\IAmSpeed\Input\InputPresentationScope.cpp" /Fe:"$outputPath\${ProbeName}.exe" /link /STACK:16777216
if errorlevel 1 exit /b %errorlevel%
"$outputPath\${ProbeName}.exe"
exit /b %errorlevel%
"@
$commandFile = Join-Path $outputPath 'probe.cmd'
[IO.File]::WriteAllText($commandFile, $batch)
$watch = [Diagnostics.Stopwatch]::StartNew()
Push-Location $outputPath
try {
    & cmd.exe /d /c $commandFile 1> 'probe.stdout.raw.log' 2> 'probe.stderr.raw.log'
    $probeExit = $LASTEXITCODE
} finally { Pop-Location }
$watch.Stop()
$metadata = @{
    status = $(if ($probeExit -eq 0) { 'passed' } else { 'failed' })
    total_seconds = $watch.Elapsed.TotalSeconds
    operation = 'standalone_device_discovery_fake_probe_no_hardware'
    exit_code = $probeExit
    compiler = 'MSVC 14.38 (vcvars_ver=14.38)'
    command_file = 'probe.cmd'
    gameinput_header_sha256 = $(if ($sdkFlags) { (Get-FileHash (Join-Path $GameInputIncludePath 'GameInput.h') -Algorithm SHA256).Hash } else { 'not_applicable' })
    fixture_sha256 = (Get-FileHash (Join-Path $PSScriptRoot "${ProbeName}.cpp") -Algorithm SHA256).Hash
    source_sha = (& git -c "safe.directory=$($moduleRoot.Replace('\','/'))" -C $moduleRoot rev-parse HEAD)
}
$metadata | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $outputPath 'metadata.json') -Encoding utf8
& $Python $Summarizer package --stdout-log (Join-Path $outputPath 'probe.stdout.raw.log') --stderr-log (Join-Path $outputPath 'probe.stderr.raw.log') --metadata (Join-Path $outputPath 'metadata.json') --output (Join-Path $outputPath 'observability.json')
if ($LASTEXITCODE -ne 0) { throw 'Observability summarizer failed' }
if ($probeExit -ne 0) { throw "Native probe failed (exit $probeExit). Inspect observability.json before raw output." }
Write-Output "[SLSUMMARY] native_probe=passed exit=0 artifacts=$outputPath"
