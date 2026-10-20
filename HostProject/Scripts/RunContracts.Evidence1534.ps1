# Exact adapters for the native observer and retained Rules admission schemas.
# These checks qualify evidence consumption only, never plugin CI or physics.
function Assert-IAmSpeedEvidenceFields1534 {
    param($Value,[string[]]$Fields)
    if ($null -eq $Value) { throw 'Missing evidence object.' }
    foreach ($name in $Fields) {
        if ($null -eq $Value.PSObject.Properties[$name] -or $null -eq $Value.$name) { throw "Missing evidence field: $name" }
    }
}

function Assert-IAmSpeedEvidenceSame1534 {
    param($Expected,$Actual)
    if ($null -eq $Expected -or $null -eq $Actual) {
        if ($null -ne $Expected -or $null -ne $Actual) { throw 'Evidence null/value mismatch.' }; return
    }
    if ($Expected -is [System.Array]) {
        if ($Actual -isnot [System.Array] -or $Expected.Count -ne $Actual.Count) { throw 'Evidence array mismatch.' }
        for ($j=0;$j -lt $Expected.Count;$j++) { Assert-IAmSpeedEvidenceSame1534 $Expected[$j] $Actual[$j] }; return
    }
    if ($Expected -is [pscustomobject]) {
        if ($Actual -isnot [pscustomobject]) { throw 'Evidence object mismatch.' }
        $names=@($Expected.PSObject.Properties.Name)
        if ($names.Count -ne @($Actual.PSObject.Properties.Name).Count) { throw 'Evidence property inventory mismatch.' }
        foreach ($name in $names) {
            if ($null -eq $Actual.PSObject.Properties[$name]) { throw "Evidence property absent: $name" }
            Assert-IAmSpeedEvidenceSame1534 $Expected.$name $Actual.$name
        }; return
    }
    if (($Expected -is [bool]) -ne ($Actual -is [bool]) -or ($Expected -is [string]) -ne ($Actual -is [string]) -or $Expected -cne $Actual) { throw 'Evidence scalar mismatch.' }
}

function Read-IAmSpeedEvidence1534 {
    param($Reference)
    Assert-IAmSpeedEvidenceFields1534 $Reference @('path','sha256')
    if ($Reference.sha256 -cnotmatch '^[0-9a-f]{64}$') { throw 'Invalid evidence SHA256.' }
    Assert-IAmSpeedPhysicalFile $Reference.path
    if ((Get-FileHash -LiteralPath $Reference.path -Algorithm SHA256).Hash.ToLowerInvariant() -cne $Reference.sha256) { throw 'Bound evidence bytes changed.' }
    Get-Content -LiteralPath $Reference.path -Raw | ConvertFrom-Json
}

function Assert-IAmSpeedNativeObservation1534 {
    param($Observation,[int]$ExpectedExit,[long]$ExpectedMemoryBytes,[int]$ExpectedCpuRate)
    Assert-IAmSpeedEvidenceFields1534 $Observation @('schema','status','natural_drain','original_exit','memory_bytes','cpu_rate','deadline_seconds','termination_requested','observations')
    $expectedStatus='failed_naturally_drained'; if ($ExpectedExit -eq 0) { $expectedStatus='naturally_drained' }
    if ($Observation.schema -cne 'skyleague.native-observation/v1' -or $Observation.status -cne $expectedStatus -or
        $Observation.natural_drain -isnot [bool] -or $Observation.natural_drain -ne $true -or
        $Observation.termination_requested -isnot [bool] -or $Observation.termination_requested -ne $false -or
        $Observation.original_exit -ne $ExpectedExit -or $ExpectedMemoryBytes -le 0 -or $ExpectedCpuRate -le 0 -or $ExpectedCpuRate -gt 10000 -or
        $Observation.memory_bytes -ne $ExpectedMemoryBytes -or $Observation.cpu_rate -ne $ExpectedCpuRate -or $Observation.deadline_seconds -le 0 -or
        $Observation.observations -isnot [System.Array] -or $Observation.observations.Count -lt 2) { throw 'Native observation schema/exit/caps/drain differs.' }
    $tail=@($Observation.observations | Select-Object -Last 2)
    foreach ($row in $tail) {
        Assert-IAmSpeedEvidenceFields1534 $row @('active','total','terminated','flags','memory_bytes','cpu_flags','cpu_rate','elapsed_seconds','original_exit')
        if ($null -ne $row.PSObject.Properties['assigned'] -or $null -ne $row.PSObject.Properties['listed'] -or
            $row.active -ne 0 -or $row.total -lt 1 -or $row.terminated -ne 0 -or $row.original_exit -ne $ExpectedExit -or
            $row.flags -ne 512 -or $row.cpu_flags -ne 5 -or $row.memory_bytes -ne $ExpectedMemoryBytes -or $row.cpu_rate -ne $ExpectedCpuRate -or
            [double]::IsNaN([double]$row.elapsed_seconds) -or [double]::IsInfinity([double]$row.elapsed_seconds) -or $row.elapsed_seconds -lt 0) { throw 'Native final two samples are ambiguous, active, terminated, or mismatched.' }
    }
    if ($tail[1].elapsed_seconds -le $tail[0].elapsed_seconds) { throw 'Native drain samples must be two distinct ordered observations.' }
}

function Assert-IAmSpeedBootstrapObservation1534 {
    param($Terminal)
    Assert-IAmSpeedEvidenceFields1534 $Terminal @('status','exit','timeout','Engine_and_source_unchanged','actual_zero_and_drain','original_job_zero','observation_path','observation_sha256')
    if ($Terminal.status -cne 'PASS' -or $Terminal.exit -ne 0 -or $Terminal.timeout -ne $false -or $Terminal.Engine_and_source_unchanged -ne $true -or $Terminal.actual_zero_and_drain -ne $true) { throw 'Bootstrap execution is not qualified and drained.' }
    $ob=Read-IAmSpeedEvidence1534 ([pscustomobject]@{path=$Terminal.observation_path;sha256=$Terminal.observation_sha256})
    Assert-IAmSpeedNativeObservation1534 $ob 0 6442450944 2000
    Assert-IAmSpeedEvidenceSame1534 $ob.observations[-1] $Terminal.original_job_zero
}

function Assert-IAmSpeedRulesDerivedAdmission1534 {
    param($Negative,$Loaded,[string]$EngineRoot)
    Assert-IAmSpeedEvidenceFields1534 $Negative @('schema','status','outer_Engine_unchanged','cases','original_roots','derived_audit','source_binding','producer','native_rerun','native_CI','full_physics','gain')
    if ($Negative.schema -cne 'sl.rules-derived-admission/v1' -or $Negative.status -cne 'ADMITTED_TEN_EXISTING_RULES_OBSERVATIONS' -or
        $Negative.outer_Engine_unchanged -ne $true -or $Negative.native_rerun -ne $false -or $Negative.native_CI -ne $false -or $Negative.full_physics -ne $false -or $Negative.gain -ne $false -or
        @($Negative.cases).Count -ne 10 -or @($Negative.original_roots).Count -ne 2) { throw 'Unknown retained Rules admission schema or qualification claim.' }
    foreach ($r in $Negative.original_roots) { if ($r.actual_original_exit -ne 1) { throw 'Retained original failed wrapper exit changed.' } }
    $audit=Read-IAmSpeedEvidence1534 $Negative.derived_audit
    if ($audit.schema -cne 'sl.rules-existing-ten-audit/v1' -or $audit.status -cne 'AUTHENTICATED_TEN_EXISTING_RULES_OBSERVATIONS_ONLY' -or $audit.native_rerun -ne $false -or @($audit.cases).Count -ne 10) { throw 'Retained ten-observation audit differs.' }
    Assert-IAmSpeedEvidenceSame1534 $audit.original_roots $Negative.original_roots
    # The producer is evidence, never executed by the consumer.
    Assert-IAmSpeedPhysicalFile $Negative.producer.path
    if ((Get-FileHash -LiteralPath $Negative.producer.path).Hash.ToLowerInvariant() -cne $Negative.producer.sha256) { throw 'Rules admission producer changed.' }
    $names=@('good_sealed','source_sha_drift','rules_sha_drift','missing_rules','alias_rules_root','unknown_stage','unknown_rules_file','profile_sha_drift','wrong_engine','missing_profile')
    $extras=@('source_case_set','derived_admission','outer_Engine_unchanged','outer_Engine_evidence_scope','source_original_root_exit','source_case_terminal','native_observation','native_rerun')
    for ($j=0;$j -lt 10;$j++) {
        $row=$Negative.cases[$j];$a=$audit.cases[$j];$name=$names[$j];$set='Negatives1488';if ($j -lt 4) {$set='Negatives'}
        if ($row.case -cne $name -or $a.case -cne $name -or $row.source_case_set -cne $set -or $row.derived_admission -cne 'AUTHENTICATED_EXISTING_RULES_OBSERVATION' -or
            $row.outer_Engine_unchanged -ne $true -or $row.source_original_root_exit -ne 1 -or $row.native_rerun -ne $false -or $a.native_rerun -ne $false -or $a.original_root_exit -ne 1) { throw 'Retained Rules row identity/scope changed.' }
        Assert-IAmSpeedEvidenceSame1534 $a.terminal $row.source_case_terminal
        Assert-IAmSpeedEvidenceSame1534 $a.observation $row.native_observation
        $source=Read-IAmSpeedEvidence1534 $row.source_case_terminal
        if (@($row.PSObject.Properties.Name).Count -ne @($source.PSObject.Properties.Name).Count+$extras.Count) { throw 'Derived Rules row property inventory changed.' }
        foreach ($p in $source.PSObject.Properties) {
            if ($null -eq $row.PSObject.Properties[$p.Name]) { throw 'Original Rules property removed.' }
            Assert-IAmSpeedEvidenceSame1534 $p.Value $row.($p.Name)
        }
        foreach ($extra in $extras) { if ($null -eq $row.PSObject.Properties[$extra]) { throw 'Derived Rules property removed.' } }
        if ($a.original_status -cne $source.status -or $a.original_native_exit -ne $source.exit -or $source.case -cne $name -or $source.Source_unchanged -ne $true -or $source.timeout -ne $false) { throw 'Original Rules FAIL/status/source/exit changed.' }
        $exit=6;if ($j -eq 0) {$exit=0}
        if ($source.exit -ne $exit -or $row.native_observation.sha256 -cne $source.observation_sha256) { throw 'Rules native refusal/success differs.' }
        $ob=Read-IAmSpeedEvidence1534 $row.native_observation
        Assert-IAmSpeedNativeObservation1534 $ob $exit 4294967296 2000
        Assert-IAmSpeedEvidenceSame1534 $ob.observations[-2..-1] $source.Job0
        $rawRoot=Split-Path -Parent $row.native_observation.path
        foreach ($stream in @('stdout','stderr')) {
            $raw=Join-Path $rawRoot ($stream+'.raw.log');Assert-IAmSpeedPhysicalFile $raw
            if ((Get-FileHash -LiteralPath $raw).Hash.ToLowerInvariant() -cne $source.($stream+'_sha256')) { throw 'Original native raw stream changed.' }
        }
        if ($j -eq 0) {
            if (@($source.loaded).Count -ne 2) { throw 'Sealed success lacks two actual assembly loads.' }
            foreach ($l in $source.loaded) {
                $name=Split-Path -Leaf $l.path
                if (-not $Loaded.ContainsKey($name) -or $l.stage -cne 'SEALED_LOAD_ONLY' -or $l.recompiled -ne $false -or $l.sha256 -cne $Loaded[$name].sha256 -or $l.mvid -cne $Loaded[$name].mvid) { throw 'Sealed load fallback or assembly mismatch.' }
            }
        } elseif (@($source.loaded).Count -ne 0) { throw 'Refusal loaded or recompiled Rules.' }
    }
    $engine=[IO.Path]::GetFullPath((Join-Path $EngineRoot 'Engine')).TrimEnd('\')+'\';$inventorySha=$null;$inventoryCount=$null
    foreach ($root in $Negative.original_roots) {
        $proof=Read-IAmSpeedEvidence1534 $root.proof;$launch=Read-IAmSpeedEvidence1534 $root.launch;$term=Read-IAmSpeedEvidence1534 $root.terminal
        if ($proof.source -cne 'tools.write_stdin' -or $proof.original_session_retained -ne $true -or $proof.actual_exit -ne 1 -or $proof.all_children_naturally_drained -ne $true -or
            $launch.source -cne 'tools.exec_command' -or $launch.original_session_retained -ne $true -or $launch.original_session_id -ne $proof.original_session_id -or $launch.owner_pid -ne $proof.owner_pid -or
            $proof.original_session_id -le 0 -or $proof.owner_pid -le 0 -or $launch.controller -cne $proof.controller -or $launch.controller_sha256 -cne $proof.controller_sha256 -or
            $proof.terminal_path -cne $root.terminal.path -or $proof.terminal_sha256 -cne $root.terminal.sha256 -or $term.Engine_and_inputs_unchanged -ne $true -or $term.unresolved_child -ne $false) { throw 'Actual original root1/launch/owner/drain proof differs.' }
        $exitFields=@($proof.PSObject.Properties | Where-Object {$_.Name -cmatch '^original_ci\d+_exit$'})
        if ($exitFields.Count -ne 1 -or $exitFields[0].Value -ne 1) { throw 'Original root exit is absent or ambiguous.' }
        Assert-IAmSpeedPhysicalFile $proof.controller
        if ((Get-FileHash -LiteralPath $proof.controller).Hash.ToLowerInvariant() -cne $proof.controller_sha256 -or $proof.original_launch_sha256 -cne $root.launch.sha256) { throw 'Original controller or launch bytes changed.' }
        $scope=Split-Path -Parent $root.terminal.path;$owner=Get-Content -LiteralPath (Join-Path $scope 'owner.json') -Raw | ConvertFrom-Json
        if ($owner.pid -ne $proof.owner_pid) { throw 'Original owner identity differs.' }
        if ($proof.Engine_after_sha256 -cne $root.engine_after.sha256) { throw 'Original exhaustive Engine SHA differs.' }
        $map=Read-IAmSpeedEvidence1534 $root.engine_after;$count=@($map.PSObject.Properties.Name).Count
        if ($count -le 0) { throw 'Empty outer Engine inventory.' }
        foreach ($p in $map.PSObject.Properties) {
            if (-not $p.Name.StartsWith($engine,[StringComparison]::OrdinalIgnoreCase) -or $p.Value -isnot [System.Array] -or $p.Value.Count -ne 3 -or $p.Value[0] -lt 0 -or $p.Value[1] -le 0 -or $p.Value[2] -cnotmatch '^[0-9a-f]{64}$') { throw 'Outer exhaustive Engine inventory shape differs.' }
        }
        $before=Join-Path $scope 'Before\Result\engine.json';Assert-IAmSpeedPhysicalFile $before
        if ((Get-FileHash -LiteralPath $before).Hash.ToLowerInvariant() -cne $root.engine_after.sha256) { throw 'Original exhaustive before/after Engine inventories differ.' }
        if ($null -ne $inventorySha -and ($inventorySha -cne $root.engine_after.sha256 -or $inventoryCount -ne $count)) { throw 'Original phases describe different Engine inventories.' }
        $inventorySha=$root.engine_after.sha256;$inventoryCount=$count
    }
    $binding=Read-IAmSpeedEvidence1534 $Negative.source_binding;$sourceRoot=Join-Path (Split-Path -Parent $Negative.source_binding.path) 'source';$seen=@{}
    if (@($binding.files).Count -le 0) { throw 'Empty historical source binding.' }
    foreach ($file in $binding.files) {
        $full=[IO.Path]::GetFullPath((Join-Path $sourceRoot $file.path))
        if (-not $full.StartsWith([IO.Path]::GetFullPath($sourceRoot).TrimEnd('\')+'\',[StringComparison]::OrdinalIgnoreCase) -or $seen.ContainsKey($full)) { throw 'Historical source escape or duplicate.' }
        $seen[$full]=$true;Assert-IAmSpeedPhysicalFile $full
        if ((Get-FileHash -LiteralPath $full).Hash.ToLowerInvariant() -cne $file.sha256) { throw 'Historical source bytes changed.' }
    }
}
