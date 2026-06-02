#!/usr/bin/env pwsh
# Reference-oracle harness (P0 skeleton).
#
# Builds Instant Meshes (BSD-3) and QuadriFlow (MIT) locally and generates golden outputs for the
# test corpus. These tools are an OFFLINE TEST ORACLE ONLY -- never shipped or linked. See
# Agents/Docs/06-csharp-portability-analysis.md section 6.6.
#
# This is a skeleton: the corpus arrives in P1 and field/singularity dumps are wired per stage from
# P3+. Until then the script clones/builds the tools (with -Build) and reports what is pending.

[CmdletBinding()]
param(
    [switch]$Build,      # clone + cmake-build the reference tools into _external/ and build/
    [switch]$Generate    # run the tools over tests/corpus and write tests/golden (P1+)
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

# --- Pinned references (bump deliberately; keeps goldens reproducible) --------------------------
$References = @(
    [pscustomobject]@{
        Name    = 'instant-meshes'
        Repo    = 'https://github.com/wjakob/instant-meshes.git'
        Commit  = 'REPLACE_WITH_PINNED_COMMIT'   # TODO(P3): pin when first generating field goldens
        License = 'BSD-3-Clause'
    },
    [pscustomobject]@{
        Name    = 'quadriflow'
        Repo    = 'https://github.com/hjwdzh/QuadriFlow.git'
        Commit  = 'REPLACE_WITH_PINNED_COMMIT'   # TODO(P6): pin when first generating pure-quad goldens
        License = 'MIT'
    }
)

$root        = Split-Path -Parent $PSCommandPath
$externalDir = Join-Path $root '_external'
$buildDir    = Join-Path $root 'build'
$corpusDir   = Join-Path $root '..\..\tests\corpus'
$goldenDir   = Join-Path $root '..\..\tests\golden'

function Require-Tool([string]$tool) {
    if (-not (Get-Command $tool -ErrorAction SilentlyContinue)) {
        throw "Required tool '$tool' not found on PATH. Install it before running the reference oracle."
    }
}

function Invoke-Build {
    Require-Tool git
    Require-Tool cmake
    New-Item -ItemType Directory -Force -Path $externalDir | Out-Null
    foreach ($ref in $References) {
        if ($ref.Commit -eq 'REPLACE_WITH_PINNED_COMMIT') {
            Write-Warning "[$($ref.Name)] commit not pinned yet - skipping (pin it when the consuming phase begins)."
            continue
        }
        $dest = Join-Path $externalDir $ref.Name
        if (-not (Test-Path $dest)) {
            Write-Host "[$($ref.Name)] cloning $($ref.Repo)"
            git clone --recursive $ref.Repo $dest
        }
        Write-Host "[$($ref.Name)] checking out $($ref.Commit)"
        git -C $dest checkout $ref.Commit
        git -C $dest submodule update --init --recursive
        # cmake configure + build into a per-tool build dir.
        $toolBuild = Join-Path $buildDir $ref.Name
        cmake -S $dest -B $toolBuild -DCMAKE_BUILD_TYPE=Release
        cmake --build $toolBuild --config Release
    }
}

function Invoke-Generate {
    $haveCorpus = (Test-Path $corpusDir) -and `
        (Get-ChildItem -Path $corpusDir -Filter *.obj -ErrorAction SilentlyContinue)
    if (-not $haveCorpus) {
        Write-Warning 'No corpus meshes found under tests/corpus (these arrive in P1). Nothing to generate yet.'
        return
    }
    New-Item -ItemType Directory -Force -Path $goldenDir | Out-Null
    # TODO(P3+): for each corpus mesh, invoke each built tool deterministically (fixed seed/params),
    # capture quad mesh + field/singularity dumps, write under tests/golden/<tool>/<mesh>/.
    Write-Warning 'Golden generation is wired per stage from P3 onward; nothing to do at P0.'
}

if ($Build)    { Invoke-Build }
if ($Generate) { Invoke-Generate }
if (-not $Build -and -not $Generate) {
    Write-Host 'Reference-oracle harness (P0 skeleton). Use -Build to build the tools, -Generate to produce goldens.'
    Write-Host 'See tools/reference-oracle/README.md.'
}
