param(
    [string]$WavtoolPath = "build\Release\V_wavtool.exe",
    [string]$InputWav = "external\World\test\vaiueo2d.wav",
    [string]$OutPath = "build\wavtool_test.wav",
    [string]$DebugLog = "build\wavtool_debug.jsonl"
)

function Get-WavInfo {
    param([Parameter(Mandatory = $true)][string]$Path)

    $fs = [System.IO.File]::OpenRead($Path)
    try {
        $br = New-Object System.IO.BinaryReader($fs)
        $riff = [System.Text.Encoding]::ASCII.GetString($br.ReadBytes(4))
        [void]$br.ReadUInt32()
        $wave = [System.Text.Encoding]::ASCII.GetString($br.ReadBytes(4))
        if ($riff -ne "RIFF" -or $wave -ne "WAVE") {
            throw "Not a RIFF/WAVE file: $Path"
        }

        $sampleRate = 0
        $channels = 0
        $bits = 0
        $dataSize = 0

        while ($fs.Position -lt $fs.Length) {
            $id = [System.Text.Encoding]::ASCII.GetString($br.ReadBytes(4))
            $size = $br.ReadUInt32()
            $next = $fs.Position + $size
            if ($id -eq "fmt ") {
                [void]$br.ReadUInt16()
                $channels = $br.ReadUInt16()
                $sampleRate = $br.ReadUInt32()
                [void]$br.ReadUInt32()
                [void]$br.ReadUInt16()
                $bits = $br.ReadUInt16()
            } elseif ($id -eq "data") {
                $dataSize = [int]$size
            }
            $fs.Position = $next
        }

        $bytesPerFrame = [Math]::Max(1, $channels * [int]($bits / 8))
        [pscustomobject]@{
            SampleRate = $sampleRate
            Channels = $channels
            Bits = $bits
            DataBytes = $dataSize
            Samples = [int]($dataSize / $bytesPerFrame)
        }
    } finally {
        $fs.Dispose()
    }
}

function Complete-WavtoolOutput {
    param([Parameter(Mandatory = $true)][string]$Path)

    if ((Test-Path -LiteralPath "$Path.whd") -and
        (Test-Path -LiteralPath "$Path.dat")) {
        $bytes = [System.IO.File]::ReadAllBytes("$Path.whd") +
            [System.IO.File]::ReadAllBytes("$Path.dat")
        [System.IO.File]::WriteAllBytes($Path, $bytes)
    }
}

if (!(Test-Path -LiteralPath $WavtoolPath)) {
    throw "Wavtool executable not found: $WavtoolPath"
}
if (!(Test-Path -LiteralPath $InputWav)) {
    throw "Input WAV not found: $InputWav"
}

if (Test-Path -LiteralPath $OutPath) {
    Remove-Item -LiteralPath $OutPath -Force
}
if (Test-Path -LiteralPath "$OutPath.whd") {
    Remove-Item -LiteralPath "$OutPath.whd" -Force
}
if (Test-Path -LiteralPath "$OutPath.dat") {
    Remove-Item -LiteralPath "$OutPath.dat" -Force
}
if (Test-Path -LiteralPath "$OutPath.wtstate") {
    Remove-Item -LiteralPath "$OutPath.wtstate" -Force
}
if (Test-Path -LiteralPath $DebugLog) {
    Remove-Item -LiteralPath $DebugLog -Force
}

$env:WT_DEBUG = "1"
$env:WT_DEBUG_LOG = $DebugLog

$envArgs = @("0", "5", "35", "0", "100", "100", "0", "50", "0", "100", "100")
& $WavtoolPath $OutPath $InputWav "0" "480@120+0" @envArgs
if ($LASTEXITCODE -ne 0) {
    throw "First wavtool call failed with exit code $LASTEXITCODE"
}
& $WavtoolPath $OutPath $InputWav "0" "480@120+0" @envArgs
if ($LASTEXITCODE -ne 0) {
    throw "Second wavtool call failed with exit code $LASTEXITCODE"
}

Complete-WavtoolOutput -Path $OutPath

$info = Get-WavInfo -Path $OutPath
if ($info.Samples -le 0 -or $info.SampleRate -le 0) {
    throw "Invalid wav output: $OutPath"
}

$twoCallSamples = $info.Samples

& $WavtoolPath $OutPath $InputWav "0" "480@120+0" @envArgs
if ($LASTEXITCODE -ne 0) {
    throw "Stale-session wavtool call failed with exit code $LASTEXITCODE"
}

Complete-WavtoolOutput -Path $OutPath
$resetInfo = Get-WavInfo -Path $OutPath
if ($resetInfo.Samples -ge $twoCallSamples) {
    throw ("Stale session was not reset: before={0}, after={1}" -f `
        $twoCallSamples, $resetInfo.Samples)
}

Write-Host ("OK wavtool output: {0} Hz, {1} ch, {2} bit, {3} samples" -f `
    $resetInfo.SampleRate, $resetInfo.Channels, $resetInfo.Bits, $resetInfo.Samples)
if (Test-Path -LiteralPath $DebugLog) {
    Write-Host "Debug log: $DebugLog"
}
