#!/usr/bin/env pwsh
# Downloads the high-density benchmark meshes into tests/corpus/external/ (gitignored). These are
# real, diverse, public research meshes used to benchmark our quad output against the QuadriFlow
# oracle on inputs far denser than the procedural primitives. Run once, then:
#   pwsh tools/reference-oracle/build-references.ps1 -Generate   # produces tests/golden/quadriflow/*
#
# The meshes are dev/CI-only (never shipped, never linked into Quadremesh.Core). Provenance and
# attributions are in tests/corpus/external/PROVENANCE.md. Source: alecjacobson/common-3d-test-models
# (pinned commit), which mirrors the classic graphics test meshes the QuadriFlow paper benchmarks on.

$ErrorActionPreference = 'Stop'

# Pinned commit keeps the downloaded meshes reproducible (bump deliberately).
$commit = '8a4f8642acaf43f9cd7b67858a1502e1055ef202'
$base = "https://raw.githubusercontent.com/alecjacobson/common-3d-test-models/$commit/data"

# axis covered ->                         mesh
$meshes = @(
    'fandisk'        # hard-surface CAD, sharp 90/45 creases (P7 crease target)
    'rocker-arm'     # mechanical, genus-1 through-hole, smooth+sharp mix
    'armadillo'      # organic with limbs/extremities, genus 0
    'happy'          # Happy Buddha: high genus, thin interleaved features
    'xyzrgb_dragon'  # very high-density organic scan (scale / robustness)
)

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$outDir = Join-Path $root '..\..\tests\corpus\external'
New-Item -ItemType Directory -Force -Path $outDir | Out-Null

foreach ($m in $meshes) {
    $url = "$base/$m.obj"
    $out = Join-Path $outDir "$m.obj"
    Write-Host "[download] $m.obj"
    Invoke-WebRequest -Uri $url -OutFile $out
}

Write-Host "Downloaded $($meshes.Count) meshes to $outDir"
Write-Host "Next: pwsh tools/reference-oracle/build-references.ps1 -Generate"
