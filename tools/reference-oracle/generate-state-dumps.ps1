#!/usr/bin/env pwsh
# Faithful-port validation harness (docs/plans/07-field-stack-report.md, Phase 0).
#
# Produces intermediate field-stack dumps from the QuadriFlow reference so the C# StateMatchTests can
# diff our port stage-by-stage against ground truth (goldens alone are insufficient — intermediate
# states regress them). The dumps are large binaries and gitignored (tests/golden/quadriflow/state/);
# they regenerate in seconds for primitives. The instrumentation itself is the committed patch
# quadriflow-dump-instrumentation.patch (the reference tree under _external/ is cloned + gitignored,
# so the patch is how the `-dump` flag survives a re-clone).
#
# Prereqs: run build-references.ps1 -Build first (clones + cmake-configures the reference with conda
# Eigen+Boost). See memory reference-oracle-quadriflow for the toolchain recipe.
#
# Usage:
#   pwsh tools/reference-oracle/generate-state-dumps.ps1                  # sphere + torus (fast)
#   pwsh tools/reference-oracle/generate-state-dumps.ps1 -Meshes fandisk  # add a dense mesh (minutes)

[CmdletBinding()]
param(
    [string[]]$Meshes = @('sphere', 'torus'),
    [int]$Faces = 5000,
    [string]$BoostBin = 'C:\ProgramData\miniconda3\envs\quadref\Library\bin'
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$root    = Split-Path -Parent $PSCommandPath
$qfTree  = Join-Path $root '_external\quadriflow'
$patch   = Join-Path $root 'quadriflow-dump-instrumentation.patch'
$build   = Join-Path $qfTree 'build'
$exe     = Join-Path $build 'Release\quadriflow.exe'
$corpus  = Join-Path $root '..\..\tests\corpus'
$extDir  = Join-Path $corpus 'external'
$stateDir = Join-Path $root '..\..\tests\golden\quadriflow\state'

if (-not (Test-Path $qfTree)) { throw "Reference tree missing. Run build-references.ps1 -Build first." }

# 1. Apply the -dump instrumentation if not already present.
if (-not (Select-String -Path (Join-Path $qfTree 'src\main.cpp') -Pattern '-dump' -Quiet)) {
    Write-Host "Applying dump instrumentation patch..."
    git -C $qfTree apply $patch
} else {
    Write-Host "Dump instrumentation already present."
}

# 2. (Re)build quadriflow incrementally using the bundled VS cmake.
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$vs = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
$cmake = Join-Path $vs 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
if (-not (Test-Path $cmake)) { $cmake = (Get-Command cmake).Source }
Write-Host "Building quadriflow (incremental)..."
& $cmake --build $build --config Release --target quadriflow
if ($LASTEXITCODE -ne 0) { throw "build failed" }

# 3. Run the instrumented oracle with -dump for each mesh (same args as the committed goldens).
if (Test-Path $BoostBin) { $env:PATH = "$BoostBin;$env:PATH" }
foreach ($m in $Meshes) {
    $in = Join-Path $corpus "$m.obj"
    if (-not (Test-Path $in)) { $in = Join-Path $extDir "$m.obj" }
    if (-not (Test-Path $in)) { Write-Warning "input $m.obj not found; skipping"; continue }

    $dir = Join-Path $stateDir $m
    New-Item -ItemType Directory -Force -Path $dir | Out-Null
    Write-Host "[$m] dumping field-stack state..."
    & $exe -mcf -seed 0 -dump $dir -i $in -o (Join-Path $env:TEMP "$m-ref.obj") -f $Faces
}

Write-Host "State dumps written under $stateDir (gitignored). StateMatchTests will pick them up."
