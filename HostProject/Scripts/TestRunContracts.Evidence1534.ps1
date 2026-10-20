function Test-IAmSpeedEvidenceContracts1534 {
    $sample=[pscustomobject]@{active=0;total=1;terminated=0;flags=512;memory_bytes=4294967296;cpu_flags=5;cpu_rate=2000;elapsed_seconds=1.0;original_exit=6}
    $last=$sample | ConvertTo-Json | ConvertFrom-Json;$last.elapsed_seconds=1.1
    $good=[pscustomobject]@{schema='skyleague.native-observation/v1';status='failed_naturally_drained';natural_drain=$true;original_exit=6;memory_bytes=4294967296;cpu_rate=2000;deadline_seconds=600.0;termination_requested=$false;observations=@($sample,$last)}
    Assert-IAmSpeedNativeObservation1534 $good 6 4294967296 2000
    foreach ($mutation in @('schema','status','one_sample','active','exit','missing_active','ambiguous_assigned','terminated','cpu','memory','elapsed','termination','drain')) {
        $bad=$good | ConvertTo-Json -Depth 10 | ConvertFrom-Json
        switch ($mutation) {
            'schema' {$bad.schema='unknown/v1'}
            'status' {$bad.status='naturally_drained'}
            'one_sample' {$bad.observations=@($bad.observations[1])}
            'active' {$bad.observations[0].active=1}
            'exit' {$bad.observations[1].original_exit=0}
            'missing_active' {$bad.observations[1].PSObject.Properties.Remove('active')}
            'ambiguous_assigned' {$bad.observations[1] | Add-Member -NotePropertyName assigned -NotePropertyValue 0}
            'terminated' {$bad.observations[0].terminated=1}
            'cpu' {$bad.observations[1].cpu_rate=4000}
            'memory' {$bad.memory_bytes=2147483648}
            'elapsed' {$bad.observations[1].elapsed_seconds=1.0}
            'termination' {$bad.termination_requested=$true}
            'drain' {$bad.natural_drain='true'}
        }
        Assert-Throws { Assert-IAmSpeedNativeObservation1534 $bad 6 4294967296 2000 } ("Native observation refuses $mutation")
    }
    $original=[pscustomobject]@{status='FAIL';exit=6;Engine_unchanged=$null;Job0=@($sample,$last)}
    foreach ($mutation in @('status','missing','null','sample_order')) {
        $bad=$original | ConvertTo-Json -Depth 10 | ConvertFrom-Json
        switch ($mutation) {
            'status' {$bad.status='PASS'}
            'missing' {$bad.PSObject.Properties.Remove('Engine_unchanged')}
            'null' {$bad.Engine_unchanged=$true}
            'sample_order' {$bad.Job0=@($bad.Job0[1],$bad.Job0[0])}
        }
        Assert-Throws { Assert-IAmSpeedEvidenceSame1534 $original $bad } ("Original evidence refuses $mutation rewrite")
    }
    Write-Output 'PASS native-observation schema, two-sample drain/caps, and exact retained FAIL/null evidence adversarial fixtures'
}

function Test-IAmSpeedDerivedEvidenceContracts1534 {
    param($Manifest,[string]$EngineRoot)
    $receipt=Read-IAmSpeedEvidence1534 ([pscustomobject]@{path=$Manifest.rules_qualification_path;sha256=$Manifest.rules_qualification_sha256})
    $good=Read-IAmSpeedEvidence1534 $receipt.negative_terminal
    if ($good.schema -cne 'sl.rules-derived-admission/v1') { throw 'Current runtime requires the retained observation schema.' }
    $loaded=@{}
    foreach ($line in Get-Content -LiteralPath $receipt.bootstrap_loaded.path) {
        $row=$line | ConvertFrom-Json;$loaded[(Split-Path -Leaf $row.path)]=$row
    }
    foreach ($mutation in @('schema','claim','root_exit','root_order','case_identity','source_status','retained_fail','terminal_sha','observation_sha','raw_sha','null_rewrite')) {
        $bad=$good | ConvertTo-Json -Depth 100 | ConvertFrom-Json
        switch ($mutation) {
            'schema' {$bad.schema='unknown/v1'}
            'claim' {$bad.native_CI=$true}
            'root_exit' {$bad.original_roots[0].actual_original_exit=0}
            'root_order' {$bad.original_roots=@($bad.original_roots[1],$bad.original_roots[0])}
            'case_identity' {$bad.cases[0].case='missing_rules'}
            'source_status' {$bad.cases[0].status='FAIL'}
            'retained_fail' {$bad.cases[9].status='PASS'}
            'terminal_sha' {$bad.cases[0].source_case_terminal.sha256='0'*64}
            'observation_sha' {$bad.cases[0].native_observation.sha256='0'*64}
            'raw_sha' {$bad.cases[0].stdout_sha256='0'*64}
            'null_rewrite' {$bad.cases[0].Engine_unchanged=$true}
        }
        Assert-Throws { Assert-IAmSpeedRulesDerivedAdmission1534 -Negative $bad -Loaded $loaded -EngineRoot $EngineRoot } ("Retained Rules admission refuses $mutation")
    }
    $bootstrap=Read-IAmSpeedEvidence1534 $receipt.bootstrap_terminal
    $bootstrap.actual_zero_and_drain=$false
    Assert-Throws { Assert-IAmSpeedBootstrapObservation1534 $bootstrap } 'bootstrap false drain claim'
    Write-Output 'PASS retained root1/FAIL/null status, exact original refs/raw SHA, and derived admission adversarial fixtures'
}
