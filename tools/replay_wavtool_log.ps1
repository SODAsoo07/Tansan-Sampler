param(
    [Parameter(Mandatory = $true)]
    [string]$LogPath,

    [Parameter(Mandatory = $true)]
    [string]$WavtoolPath,

    [int]$Max = 10
)

if (!(Test-Path -LiteralPath $LogPath)) {
    Write-Error "Log file not found: $LogPath"
    exit 1
}

if (!(Test-Path -LiteralPath $WavtoolPath)) {
    Write-Error "Wavtool executable not found: $WavtoolPath"
    exit 1
}

$count = 0
Get-Content -LiteralPath $LogPath | ForEach-Object {
    if ($count -ge $Max) { return }
    $line = $_.Trim()
    if ([string]::IsNullOrWhiteSpace($line)) { return }

    try {
        $obj = $line | ConvertFrom-Json
    } catch {
        return
    }

    if ($obj.event -ne "invoke") { return }
    if (-not $obj.args -or $obj.args.Count -lt 1) { return }

    $args = @()
    for ($i = 1; $i -lt $obj.args.Count; $i++) {
        $args += [string]$obj.args[$i]
    }

    Write-Host ("[{0}] Replaying: {1} {2}" -f $count, $WavtoolPath, ($args -join " "))
    & $WavtoolPath @args
    if ($LASTEXITCODE -ne 0) {
        Write-Warning "Replay command failed with exit code $LASTEXITCODE"
    }
    $count++
}

Write-Host "Replay done. invoke records replayed: $count"
