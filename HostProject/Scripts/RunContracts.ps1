[CmdletBinding()]
param(
    [string]$EngineRoot = $env:UE_ROOT,
    [string]$TestFilter = "IAmSpeed.AnalyticWorld"
)

$ErrorActionPreference = "Stop"
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$ProjectFile = Join-Path $ProjectRoot "IAmSpeedHostProject.uproject"
$PluginRoot = Split-Path -Parent $ProjectRoot
$ExternalPluginRoot = Join-Path $ProjectRoot "ExternalPlugins"
$PluginLink = Join-Path $ExternalPluginRoot "IAmSpeed"
if ([string]::IsNullOrWhiteSpace($EngineRoot)) {
    throw "EngineRoot or UE_ROOT is required."
}
$Build = Join-Path $EngineRoot "Engine\Build\BatchFiles\Build.bat"
$Editor = Join-Path $EngineRoot "Engine\Binaries\Win64\UnrealEditor-Cmd.exe"
if (-not (Test-Path -LiteralPath $Build) -or
    -not (Test-Path -LiteralPath $Editor)) {
    throw "The requested Unreal Engine root is incomplete: $EngineRoot"
}

New-Item -ItemType Directory -Force -Path $ExternalPluginRoot | Out-Null
if (Test-Path -LiteralPath $PluginLink) {
    throw "HostProject plugin link already exists: $PluginLink"
}
try {
    $LinkType = if ($IsWindows -or $env:OS -eq "Windows_NT") {
        "Junction"
    } else {
        "SymbolicLink"
    }
    New-Item -ItemType $LinkType -Path $PluginLink -Target $PluginRoot | Out-Null

    & $Build IAmSpeedHostProjectEditor Win64 Development "-Project=$ProjectFile" `
        -WaitMutex -NoHotReload
    if ($LASTEXITCODE -ne 0) { throw "HostProject build failed." }

    & $Editor $ProjectFile -unattended -nop4 -nosplash -NullRHI `
        "-ExecCmds=Automation RunTests $TestFilter;Quit" `
        "-TestExit=Automation Test Queue Empty"
    if ($LASTEXITCODE -ne 0) { throw "HostProject contract tests failed." }
}
finally {
    if (Test-Path -LiteralPath $PluginLink) {
        Remove-Item -LiteralPath $PluginLink -Force
    }
}
