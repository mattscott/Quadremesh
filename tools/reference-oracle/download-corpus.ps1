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
    # Extra high-poly meshes (no QuadriFlow golden; used by NewCorpus_search_and_remesh for the auto-budget
    # search + remesh). horse/lucy/bimba are CLOSED genus-0 (watertight tests); max-planck/ogre are OPEN
    # scans (boundary handling / robustness — the search reports found=False for them).
    'horse'          # organic animal, closed genus 0 (~97k tris)
    'lucy'           # intricate winged statue, closed genus 0 (~100k tris)
    'bimba'          # bust statue, closed genus 0 (~225k tris)
    'max-planck'     # head-bust scan, OPEN (~100k tris)
    'ogre'           # creature scan, OPEN/multi-boundary (~124k tris)
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
