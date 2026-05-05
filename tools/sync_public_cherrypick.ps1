<#
.SYNOPSIS
프라이빗 브랜치 커밋을 공개 브랜치에 cherry-pick 하고 필요 시 push까지 수행합니다.

.EXAMPLE
.\tools\sync_public_cherrypick.ps1 -Commits a1b2c3d e4f5g6h

.EXAMPLE
.\tools\sync_public_cherrypick.ps1 -RangeStart a1b2c3d -RangeEnd e4f5g6h -Squash

.EXAMPLE
.\tools\sync_public_cherrypick.ps1 -RangeStart HEAD~3 -RangeEnd HEAD -DryRun
#>
[CmdletBinding(DefaultParameterSetName = "Commits")]
param(
    [Parameter(Mandatory = $true, ParameterSetName = "Commits")]
    [string[]]$Commits,

    [Parameter(Mandatory = $true, ParameterSetName = "Range")]
    [string]$RangeStart,

    [Parameter(Mandatory = $true, ParameterSetName = "Range")]
    [string]$RangeEnd,

    [string]$PrivateBranch = "master",
    [string]$PublicBranch = "public-main",
    [string]$PublicRemote = "public",
    [string]$PublicRemoteBranch = "main",

    [switch]$Squash,
    [switch]$NoPush,
    [switch]$DryRun
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Invoke-Git {
    param(
        [Parameter(Mandatory = $true)]
        [string[]]$Args,
        [switch]$Capture
    )

    if ($Capture) {
        $output = & git @Args 2>&1
        if ($LASTEXITCODE -ne 0) {
            throw ("git {0}`n{1}" -f ($Args -join " "), ($output -join "`n"))
        }
        return @($output)
    }

    & git @Args
    if ($LASTEXITCODE -ne 0) {
        throw ("git {0}" -f ($Args -join " "))
    }
}

function Assert-CleanWorktree {
    $status = Invoke-Git -Args @("status", "--porcelain", "-uall") -Capture
    if ($status.Count -gt 0) {
        throw "작업 트리가 깨끗하지 않습니다. 변경사항을 커밋/스태시 후 다시 실행하세요."
    }
}

function Assert-RemoteExists {
    param([Parameter(Mandatory = $true)][string]$RemoteName)

    $remotes = Invoke-Git -Args @("remote") -Capture
    if (-not ($remotes | Where-Object { $_.Trim() -eq $RemoteName })) {
        throw ("remote '{0}' 가 없습니다. 먼저 remote를 추가하세요." -f $RemoteName)
    }
}

function Assert-RefExists {
    param([Parameter(Mandatory = $true)][string]$Ref)
    Invoke-Git -Args @("rev-parse", "--verify", $Ref) -Capture | Out-Null
}

function Get-CommitsFromRange {
    param(
        [Parameter(Mandatory = $true)][string]$Start,
        [Parameter(Mandatory = $true)][string]$End
    )

    $range = "{0}^..{1}" -f $Start, $End
    $items = Invoke-Git -Args @("rev-list", "--reverse", $range) -Capture
    if ($items.Count -eq 0) {
        throw ("범위에 커밋이 없습니다: {0}" -f $range)
    }
    return @($items)
}

$originalBranch = (Invoke-Git -Args @("rev-parse", "--abbrev-ref", "HEAD") -Capture)[0].Trim()
$picked = @()

try {
    Assert-CleanWorktree
    Assert-RemoteExists -RemoteName $PublicRemote
    Assert-RefExists -Ref $PrivateBranch

    Write-Host ("[1/6] fetch {0}" -f $PublicRemote)
    Invoke-Git -Args @("fetch", $PublicRemote)

    $remoteRef = "refs/remotes/{0}/{1}" -f $PublicRemote, $PublicRemoteBranch
    Assert-RefExists -Ref $remoteRef

    $localPublicRef = "refs/heads/{0}" -f $PublicBranch
    $localPublicExists = $true
    try {
        Assert-RefExists -Ref $localPublicRef
    } catch {
        $localPublicExists = $false
    }

    Write-Host ("[2/6] checkout {0}" -f $PublicBranch)
    if ($localPublicExists) {
        Invoke-Git -Args @("switch", $PublicBranch)
    } else {
        Invoke-Git -Args @("switch", "-c", $PublicBranch, "--track", "{0}/{1}" -f $PublicRemote, $PublicRemoteBranch)
    }

    Write-Host ("[3/6] update {0} from {1}/{2} (ff-only)" -f $PublicBranch, $PublicRemote, $PublicRemoteBranch)
    Invoke-Git -Args @("merge", "--ff-only", "{0}/{1}" -f $PublicRemote, $PublicRemoteBranch)

    if ($PSCmdlet.ParameterSetName -eq "Range") {
        $Commits = Get-CommitsFromRange -Start $RangeStart -End $RangeEnd
    }

    if (-not $Commits -or $Commits.Count -eq 0) {
        throw "적용할 커밋이 비어 있습니다."
    }

    foreach ($commit in $Commits) {
        $resolved = (Invoke-Git -Args @("rev-parse", "--verify", $commit) -Capture)[0].Trim()
        $picked += $resolved
    }

    if ($DryRun) {
        Write-Host "[4/6] dry-run mode"
        Write-Host ("- target: {0}/{1}" -f $PublicRemote, $PublicRemoteBranch)
        Write-Host ("- pick count: {0}" -f $picked.Count)
        $picked | ForEach-Object { Write-Host ("  - {0}" -f $_) }
        Write-Host "[5/6] skipped (dry-run)"
        return
    }

    Write-Host "[4/6] cherry-pick"
    if ($Squash) {
        foreach ($commit in $picked) {
            Write-Host ("  - apply(no-commit): {0}" -f $commit)
            try {
                Invoke-Git -Args @("cherry-pick", "--no-commit", "-x", $commit)
            } catch {
                Write-Error ("충돌 발생: {0}" -f $commit)
                Write-Host "cherry-pick 중단/복구를 수행합니다."
                Invoke-Git -Args @("cherry-pick", "--abort")
                throw
            }
        }

        $msg = "sync(public): cherry-pick {0} commit(s) from {1}" -f $picked.Count, $PrivateBranch
        Invoke-Git -Args @("commit", "-m", $msg)
    } else {
        foreach ($commit in $picked) {
            Write-Host ("  - pick: {0}" -f $commit)
            try {
                Invoke-Git -Args @("cherry-pick", "-x", $commit)
            } catch {
                Write-Error ("충돌 발생: {0}" -f $commit)
                Write-Host "cherry-pick 중단/복구를 수행합니다."
                Invoke-Git -Args @("cherry-pick", "--abort")
                throw
            }
        }
    }

    if (-not $NoPush) {
        Write-Host ("[5/6] push -> {0}/{1}" -f $PublicRemote, $PublicRemoteBranch)
        Invoke-Git -Args @("push", $PublicRemote, "{0}:{1}" -f $PublicBranch, $PublicRemoteBranch)
    } else {
        Write-Host "[5/6] push skipped (-NoPush)"
    }
}
finally {
    Write-Host ("[6/6] restore branch: {0}" -f $originalBranch)
    try {
        Invoke-Git -Args @("switch", $originalBranch)
    } catch {
        Write-Warning ("원래 브랜치 복귀 실패: {0}" -f $originalBranch)
    }
}

Write-Host ""
Write-Host "완료:"
Write-Host ("- source branch: {0}" -f $PrivateBranch)
Write-Host ("- public branch: {0}" -f $PublicBranch)
Write-Host ("- picked count : {0}" -f $picked.Count)
if ($picked.Count -gt 0) {
    Write-Host ("- first commit : {0}" -f $picked[0])
    Write-Host ("- last commit  : {0}" -f $picked[$picked.Count - 1])
}
