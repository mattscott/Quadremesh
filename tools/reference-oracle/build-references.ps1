#!/usr/bin/env pwsh
# Reference-oracle harness. Builds QuadriFlow (MIT) - and optionally Instant Meshes (BSD) - from
# source and runs them over the test corpus to emit golden quad meshes for comparison. These tools
# are an OFFLINE TEST ORACLE ONLY: never shipped, never linked. See
# Agents/Docs/06-csharp-portability-analysis.md section 6.6.
#
# Usage:
#   pwsh tools/reference-oracle/build-references.ps1 -Build       # discover toolchain, clone, build
#   pwsh tools/reference-oracle/build-references.ps1 -Generate    # run over tests/corpus -> tests/golden
#   pwsh tools/reference-oracle/build-references.ps1 -Build -Generate

[CmdletBinding()]
param(
    [switch]$Build,
    [switch]$Generate,
    [switch]$IncludeInstantMeshes   # Instant Meshes pulls heavy GUI deps; off by default
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$root        = Split-Path -Parent $PSCommandPath
$externalDir = Join-Path $root '_external'
$buildRoot   = Join-Path $root 'build'
$corpusDir   = Resolve-Path (Join-Path $root '..\..\tests\corpus') -ErrorAction SilentlyContinue
$goldenDir   = Join-Path $root '..\..\tests\golden'

# --- Pinned references (bump deliberately; keeps goldens reproducible) --------------------------
$References = @(
    [pscustomobject]@{
        Name    = 'quadriflow'
        Repo    = 'https://github.com/hjwdzh/QuadriFlow.git'
        Commit  = '810b7a0967c35b0dc85b4464e3835e26a756c967'  # master, 2024
        License = 'MIT'
        Enabled = $true
    },
    [pscustomobject]@{
        Name    = 'instant-meshes'
        Repo    = 'https://github.com/wjakob/instant-meshes.git'
        Commit  = '7b3160864a2e1025af498c84cfed91cbfb613698'  # master
        License = 'BSD-3-Clause'
        Enabled = [bool]$IncludeInstantMeshes
    }
)

function Find-Toolchain {
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (-not (Test-Path $vswhere)) {
        throw "vswhere not found. Install Visual Studio with the C++ workload, or build on a box with cmake on PATH."
    }

    $vs = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath 2>$null
    if (-not $vs) {
        throw "No Visual Studio install with the C++ tools (VC.Tools.x86.x64) was found."
    }

    $cmake = Join-Path $vs 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
    if (-not (Test-Path $cmake)) {
        $cmake = (Get-Command cmake -ErrorAction SilentlyContinue).Source
    }
    if (-not $cmake) {
        throw "cmake not found (neither bundled with VS nor on PATH)."
    }

    Write-Host "Visual Studio: $vs"
    Write-Host "cmake: $cmake"
    return [pscustomobject]@{ Cmake = $cmake }
}

function Invoke-Build {
    Require-Tool git
    $tc = Find-Toolchain
    New-Item -ItemType Directory -Force -Path $externalDir | Out-Null

    foreach ($ref in $References | Where-Object { $_.Enabled }) {
        $dest = Join-Path $externalDir $ref.Name
        if (-not (Test-Path (Join-Path $dest '.git'))) {
            Write-Host "[$($ref.Name)] cloning $($ref.Repo)"
            git clone --recursive $ref.Repo $dest
        }

        Write-Host "[$($ref.Name)] checking out $($ref.Commit)"
        git -C $dest fetch --depth 1 origin $ref.Commit 2>$null
        git -C $dest checkout --force $ref.Commit
        git -C $dest submodule update --init --recursive

        $bld = Join-Path $buildRoot $ref.Name
        Write-Host "[$($ref.Name)] configuring"
        & $tc.Cmake -S $dest -B $bld -DCMAKE_BUILD_TYPE=Release
        if ($LASTEXITCODE -ne 0) { throw "[$($ref.Name)] cmake configure failed" }

        Write-Host "[$($ref.Name)] building (Release)"
        & $tc.Cmake --build $bld --config Release
        if ($LASTEXITCODE -ne 0) { throw "[$($ref.Name)] build failed" }
    }

    Write-Host 'Build complete.'
}

function Find-Executable([string]$buildDir, [string]$stem) {
    Get-ChildItem -Path $buildDir -Recurse -Filter "$stem*.exe" -ErrorAction SilentlyContinue |
        Where-Object { $_.Name -match "^$stem(\.exe)?$" -or $_.BaseName -ieq $stem } |
        Select-Object -First 1 -ExpandProperty FullName
}

function Invoke-Generate {
    if (-not $corpusDir -or -not (Get-ChildItem -Path $corpusDir -Filter *.obj -ErrorAction SilentlyContinue)) {
        Write-Warning "No corpus meshes under tests/corpus."
        return
    }

    # QuadriFlow: quadriflow -i in.obj -o out.obj -f <target faces>
    $qfExe = Find-Executable (Join-Path $buildRoot 'quadriflow') 'quadriflow'
    if (-not $qfExe) {
        Write-Warning "quadriflow executable not found - run with -Build first."
        return
    }

    $outDir = Join-Path $goldenDir 'quadriflow'
    New-Item -ItemType Directory -Force -Path $outDir | Out-Null

    # The procedural primitives plus any high-density benchmark meshes downloaded into external/
    # (tools/reference-oracle/download-corpus.ps1). The latter are gitignored; their goldens are committed.
    $meshes = @(Get-ChildItem -Path $corpusDir -Filter *.obj)
    $externalDir = Join-Path $corpusDir 'external'
    if (Test-Path $externalDir) {
        $meshes += Get-ChildItem -Path $externalDir -Filter *.obj
    }

    foreach ($mesh in $meshes) {
        if ($mesh.BaseName -eq 'non-manifold') { continue }  # QuadriFlow expects manifold input
        $out = Join-Path $outDir "$($mesh.BaseName).obj"
        Write-Host "[quadriflow] $($mesh.Name) -> $out"
        # -mcf = the min-cost-flow integer solve (the production extractor); -seed for determinism.
        & $qfExe -mcf -seed 0 -i $mesh.FullName -o $out -f 5000
    }

    Write-Host "Goldens written to $outDir"
}

function Require-Tool([string]$tool) {
    if (-not (Get-Command $tool -ErrorAction SilentlyContinue)) {
        throw "Required tool '$tool' not found on PATH."
    }
}

if ($Build)    { Invoke-Build }
if ($Generate) { Invoke-Generate }
if (-not $Build -and -not $Generate) {
    Write-Host 'Reference-oracle harness. Use -Build to build the tools, -Generate to produce goldens.'
    Write-Host 'See tools/reference-oracle/README.md.'
}
