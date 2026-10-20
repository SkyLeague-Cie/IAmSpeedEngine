function Assert-IAmSpeedSourceGraph1495 {
    [CmdletBinding()]
    param([Parameter(Mandatory=$true)][string]$Path,
          [Parameter(Mandatory=$true)][string]$ConsolePath,
          [Parameter(Mandatory=$true)][int]$ExitCode,
          [Parameter(Mandatory=$true)][string]$EngineRoot,
          [Parameter(Mandatory=$true)][string[]]$AllowedOutputRoots)
    if ($ExitCode -ne 6) { throw "Source graph diagnostic must stop with its intentional exit 6; received $ExitCode." }
    Assert-IAmSpeedPhysicalFile $Path
    Assert-IAmSpeedPhysicalFile $ConsolePath
    if (-not (Select-String -LiteralPath $ConsolePath -SimpleMatch 'Private source graph diagnostic completed: action execution disabled' -Quiet)) { throw 'Source graph intentional cutoff is missing.' }
    $graph = Get-Content -LiteralPath $Path -Raw | ConvertFrom-Json
    $engine = [IO.Path]::GetFullPath((Join-Path $EngineRoot 'Engine')).TrimEnd('\')
    if ($graph.state -cne 'REAL_TARGET_GRAPH_CAPTURED_BEFORE_ACTION_EXECUTION' -or
        [IO.Path]::GetFullPath([string]$graph.engine).TrimEnd('\') -ine $engine -or
        -not $graph.no_cpp_actions_executed -or $graph.engine_output_count -ne 0 -or
        $graph.action_count -ne @($graph.actions).Count -or $graph.action_count -le 0) { throw 'Source graph scope or action inventory differs.' }
    $roots = @($AllowedOutputRoots | ForEach-Object { [IO.Path]::GetFullPath($_).TrimEnd('\') + '\' })
    if ($roots.Count -lt 3) { throw 'Source graph requires explicit project, plugin and cache output roots.' }
    $compileCount = 0; $linkCount = 0; $outputCount = 0
    foreach ($action in @($graph.actions)) {
        foreach ($field in @('produced','delete_items','prerequisite_items','engine_outputs')) {
            if ($null -eq $action.PSObject.Properties[$field] -or $action.$field -isnot [System.Array]) { throw "Missing source graph array: $field" }
        }
        foreach ($reference in (@($action.executable,$action.working_directory,$action.arguments) + @($action.prerequisite_items) + @($action.produced) + @($action.delete_items))) {
            $normalized = ([string]$reference).Replace('/','\').ToLowerInvariant()
            foreach ($forbidden in @('e:\documents\unreal projects\skyleague','e:\programs\unrealengine\5.8.2-slot5','\physicsfinal\','\e4\')) {
                if ($normalized.Contains($forbidden)) { throw 'Source graph references a forbidden external checkout or Engine.' }
            }
        }

        if (@($action.engine_outputs).Count -ne 0) { throw 'Source graph produces Engine outputs.' }
        if ($action.type -ceq 'Compile') { $compileCount++ }
        if ($action.type -ceq 'Link') { $linkCount++ }
        foreach ($pathValue in (@($action.produced) + @($action.delete_items))) {
            if (-not [IO.Path]::IsPathRooted([string]$pathValue)) { throw 'Relative source graph output is refused.' }
            $output = [IO.Path]::GetFullPath([string]$pathValue)
            if ($output.StartsWith($engine + '\', [StringComparison]::OrdinalIgnoreCase)) { throw 'Source graph writes or deletes Engine files.' }
            $allowed = $false
            foreach ($prefix in $roots) { if ($output.StartsWith($prefix, [StringComparison]::OrdinalIgnoreCase)) { $allowed = $true; break } }
            if (-not $allowed) { throw "Source graph output escapes private roots: $output" }
            $outputCount++
        }
    }
    if ($compileCount -le 0 -or $linkCount -le 0) { throw 'Fresh source CI must contain actual C++ compile and link actions.' }
    [pscustomobject]@{status='ACTUAL_GRAPH_CUTOFF_ONLY_CPP_NOT_EXECUTED';path=$Path;sha256=(Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant();console_sha256=(Get-FileHash -LiteralPath $ConsolePath -Algorithm SHA256).Hash.ToLowerInvariant();native_exit=$ExitCode;action_count=$graph.action_count;compile_count=$compileCount;link_count=$linkCount;private_output_count=$outputCount;Engine_outputs=0;native_CI_qualified=$false}
}