[CmdletBinding()]
param([string]$TestOutputRoot = (Join-Path $PSScriptRoot 'CoverageResults'))
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'RunContracts.Helpers.ps1')
$taskCaseRoot = Join-Path $TestOutputRoot ([DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ') + '-' + [Guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $taskCaseRoot | Out-Null
$script:PassedCases = [System.Collections.Generic.List[string]]::new()
$pathA = 'IAmSpeed.AnalyticWorld.BoundedPlane'
$pathB = 'IAmSpeed.Simulation.ParallelFrame.WorkerPolicy'
$reportPath = Join-Path $taskCaseRoot 'index.json'

function Write-Report($Paths, $State = 'Success', $Warnings = 0, $NotRun = 0) {
    $tests = @($Paths | ForEach-Object { @{ fullTestPath=$_; state=$State } })
    $report = @{ succeeded=$tests.Count; succeededWithWarnings=$Warnings; failed=0; notRun=$NotRun; inProcess=0; tests=$tests }
    $report | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $reportPath -Encoding UTF8
}
function Expect-Pass([string]$Name, [scriptblock]$Action) {
    $null = & $Action
    $script:PassedCases.Add($Name)
}
function Expect-Fail([string]$Name, [scriptblock]$Action) {
    $rejected = $false
    try { $null = & $Action } catch { $rejected = $true }
    if (-not $rejected) { throw "Malformed automation result was accepted: $Name" }
    $script:PassedCases.Add($Name)
}
function Check-Report([string]$Filter, [string[]]$Required, [string]$Output = 'Automation Test Queue Empty 2 tests performed.') {
    Assert-IAmSpeedAutomationResult -ExitCode 0 -StandardOutput $Output -ReportPath $reportPath -TestFilter $Filter -RequiredTestPaths $Required
}

Write-Report @($pathA, $pathB)
Expect-Pass 'root_filter_requires_real_parallel_test' { Check-Report 'IAmSpeed' @($pathA, $pathB) }
Expect-Fail 'successful_subset_missing_required_test' { Check-Report 'IAmSpeed' @($pathA, $pathB, 'IAmSpeed.Parallel.StaticQueryBatch.AllHitFields') }
Expect-Fail 'missing_completion_marker' { Check-Report 'IAmSpeed' @($pathB) 'startup only' }
Expect-Fail 'ambiguous_completion_marker' { Check-Report 'IAmSpeed' @($pathB) 'Automation Test Queue Empty 2 tests performed. Automation Test Queue Empty 2 tests performed.' }
Expect-Fail 'queue_report_count_mismatch' { Check-Report 'IAmSpeed' @($pathB) 'Automation Test Queue Empty 3 tests performed.' }

Write-Report @($pathB)
Expect-Pass 'exact_leaf_filter_is_valid' { Check-Report $pathB @($pathB) 'Automation Test Queue Empty 1 tests performed.' }
Expect-Fail 'case_changed_required_name_is_not_the_test' { Check-Report 'IAmSpeed' @('IAmSpeed.Simulation.ParallelFrame.workerpolicy') 'Automation Test Queue Empty 1 tests performed.' }
Expect-Fail 'wrong_prefix_cannot_satisfy_leaf_filter' { Check-Report 'IAmSpeed.Parallel' @() 'Automation Test Queue Empty 1 tests performed.' }
Expect-Fail 'nonzero_native_exit_is_preserved' { Assert-IAmSpeedAutomationResult -ExitCode 1 -StandardOutput 'Automation Test Queue Empty 1 tests performed.' -ReportPath $reportPath -TestFilter 'IAmSpeed' -RequiredTestPaths @($pathB) }

Write-Report @($pathB, $pathB)
Expect-Fail 'duplicate_successful_test_is_not_coverage' { Check-Report 'IAmSpeed' @($pathB) }
Write-Report @($pathA, 'OtherPlugin.Success')
Expect-Fail 'outside_plugin_result_rejected' { Check-Report 'IAmSpeed' @($pathA) }
foreach ($state in @('Fail', 'NotRun', 'InProcess', 'SuccessWithWarnings')) {
    Write-Report @($pathA, $pathB) $state
    Expect-Fail ('non_success_state_' + $state) { Check-Report 'IAmSpeed' @($pathA, $pathB) }
}
Write-Report @($pathA, $pathB) 'Success' 1 0
Expect-Fail 'aggregate_warning_cannot_be_hidden' { Check-Report 'IAmSpeed' @($pathB) }
Write-Report @($pathA, $pathB) 'Success' 0 1
Expect-Fail 'aggregate_notrun_cannot_be_hidden' { Check-Report 'IAmSpeed' @($pathB) }
Set-Content -LiteralPath $reportPath -Value '{' -Encoding UTF8
Expect-Fail 'malformed_report_is_not_empty_pass' { Check-Report 'IAmSpeed' @($pathB) }

$receipt = [ordered]@{
    status='PASS_AUTOMATION_RESULT_READER_ONLY';
    native_execution='NOT_EXECUTED';
    cases=@($script:PassedCases);
    helper_sha256=(Get-FileHash -LiteralPath (Join-Path $PSScriptRoot 'RunContracts.Helpers.ps1') -Algorithm SHA256).Hash.ToLowerInvariant();
    raw_fixture_root=$taskCaseRoot
}
$receipt | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath (Join-Path $taskCaseRoot 'result.json') -Encoding UTF8
$receipt | ConvertTo-Json -Depth 8 -Compress
