# 07 — Faithful re-port of the FIELD + EXTRACTION stack

Status: **spec / plan** (no production code yet). Read with the prior plans
`04-quadriflow-integer-port.md`, `05-topology-safe-singularities.md`, `06-cdcl-flipsat-scoping.md`.

This plan is built on a stage-by-stage audit of our implementation against the read-only MIT reference
(`tools/reference-oracle/_external/quadriflow/src/`). The per-stage divergence detail is in §A (appendix);
the strategy, validation architecture, and phasing are in §1–§7.

---

## 1. The one-paragraph thesis

The dense-mesh holes are **not** a back-end bug and **not** a single missing stage. Six independent
audits confirm the memory meta-finding: **every stage of our field+extraction pipeline diverges from
QuadriFlow in a coupled way**, and our pipeline is internally balanced around its *own* (different)
choices — which is exactly why all four prior single-piece ports (position optimizer, hierarchical flow,
FixFlipSat/CDCL, priority coarsening) either regressed a primitive or moved holes sideways. The fix is a
**whole-stack faithful re-port**, but the way to make that *tractable and validatable* is the key
strategic decision of this plan: **build the faithful stack bottom-up behind a flag, validating each
stage numerically against ground-truth state dumped from the reference binary** — never against the
production goldens (which intermediate states are *proven* to regress). When stage *N* receives the same
inputs the reference had, the port of stage *N* finally matches — dissolving the coupling problem that
doomed isolated attempts. Production stays green on the current pipeline the entire time; we flip the
default only when the faithful stack hits goldens on primitives **and** closes the dense holes.

---

## 2. The whole-stack divergence map

Condensed from §A. "Coupled-to" names the stages a divergence interacts with — the reason it cannot be
ported alone. Severity = estimated contribution to the dense-mesh hole/quality gap.

| # | Stage | Our divergence (vs reference) | Coupled-to | Severity |
|---|-------|-------------------------------|-----------|----------|
| D0 | Pre-pass | full isotropic remesh (split+collapse+flip+relax) vs ref **split-only subdivide** to `scale/2`, guarded. Different starting mesh ⇒ dirtier field. | everything | **High** |
| D1 | Finest attributes | Newell **area-weighted normals** + **barycentric area** vs ref **angle-weighted normals** + **Voronoi mixed area** | D2, D3 | Med |
| D2 | Adjacency order | undirected-edge-id order vs ref **dedge angular circulation** (boundary-first); no `V2E`/`E2E` tables | D3, D4 | Med (ordering feeds stable-sort + coloring) |
| D3 | Hierarchy coarsen | **first-unmatched-neighbour** match vs ref **priority `normalDot×areaRatio` + stable_sort**; plain normal sum vs **area-weighted** merge; no `mToUpper`; dictionary-order coarse adj vs ref scratch-sort | D1, D2, D5, D6 | **High** |
| D4 | Graph coloring | natural-order greedy, recomputed per solve vs ref **PCG32-shuffled**, computed once at build | D5, D6 (GS order) | Med |
| D5 | Orientation field | **flat average** (self-weight 1, project/normalize once at end) vs ref **running-reference recurrence** (self-as-reference-only, weight_sum starts 0, **per-neighbour project+normalize**); 20 sweeps vs **6**; **missing** final fine→coarse propagation; `compat` fixes `best_a=0`, no `1e-6` tie hysteresis | D3, D4, D6, D7 | **High** |
| D6 | Singularities | counts **±2** as singular, single-sided `MatchingIndex` vs ref `value.second−value.first` with **odd-only (mod∈{1,3})** filter ⇒ inflated count | D5 | Med (count gap) |
| D7 | Scale field | single isotropic scalar; `optimize_scale` **never run** vs ref two-component per-vertex `mS`, scale solve always run (no-op `(1,1)` in uniform mode but interface threaded) | D8 | Low (latent under uniform ℓ) |
| D8 | Position field | **fixed-reference round-wrap-to-`v_i`**, self-weight 1, project once; `LatticeRepresentative` = **ROUND in `p_i`'s frame**; **MISSING** `middle_point`, `position_floor_4`, `position_round_4`, the 4×4 corner search; **level-skipping** (`SelectStartLevel`); **offset-only prolong**; init `p_i=v_i` (no random) vs ref running-reference + midpoint-floor + 4-search + `position_round_4` + full hierarchy + absolute prolong + random init | D3, D5, D7, D9 | **Critical** (prime foldover generator) |
| D9 | Integer solve | **single-level** flow + **global** capacity growth vs ref **hierarchical** coarse→fine with **level-0-only** growth; **always MCF cost-1** vs ref default **max-flow** (MCF opt-in, coarse only); **no `allow_changes==2`** half-lock; Balance X+Y joint vs per-dim. Ref **Balances once at finest before downsampling** (no per-level Balance) — that ordering keeps the torus handle routable | D8, D10 | **High** |
| D10 | Subdivide/extract | **leading FixFlip** added + **FixFlipSat omitted** + **early-stopping subdivide guard** (leaves `|diff|>1`) vs ref `Sub→FixFlip→Sub→FixFlipSat` with `exit(0)` guarantee of `±1`; **extractor re-coarsens with ad-hoc UnionFind instead of `DownsampleEdgeGraph.back()`** — the direct boundary-loop (hole) source; **missing** `PushDownwardFlip` depth descent; **missing** post-solver `optimize_positions_sharp/_fixed` | D9, D8 | **Critical** (the hole emerges here) |
| D11 | Orchestrator | no `NormalizeMesh`/un-normalize; default-on feature pinning (ref `-sharp` gated); `ProjectToSurface` snap (ref has none) | glue | Low/keep |

**Faithful pieces already in place (do not touch):** `Lattice2D.Rshift90`/`Match`/`Rank` (bit-identical),
`subdivide_edgeDiff` core, `FixFlip`/`CheckShrink`/`PropagateEdge`, `BuildTriangleManifold` fan+colour-split,
`FixValence` + `compute_direct_graph_quad`, `QuadHoleFiller`/`QuadEnergy`, the CDCL/`FlipSat`/`LocalSat`
foundation. **These are assets the port reuses** — notably the extractor fix (D10) just means *wiring the
existing faithful `EdgeGraphHierarchy.DownsampleEdgeGraph` into the extractor* instead of the UnionFind.

---

## 3. Why incremental failed — and the resolution that makes the re-port work

The investigation's hard-won lesson (memory): four faithful single-stage ports each regressed a golden or
moved holes sideways, because each ran on **inputs our other stages produce, which differ from what the
ported stage assumes**. Porting `optimize_positions` made foldovers worse because the *orientation field*
it consumed was ours, not ref's; area-weighted coarsening regressed the torus because the *normals* it
weighted were Newell, not angle-weighted; hierarchical flow stalled because Balance ran in the wrong place.

The resolution is structural, not heroic:

1. **Port bottom-up.** Mesh attributes → adjacency → hierarchy → coloring → orientation → singularities →
   scale → position → integer → subdivide/extract. Each stage is ported only after every stage it consumes
   is already faithful. Then the prior failures *cannot recur*: the position solver finally receives ref's
   orientation field and ref's hierarchy, so ref's `optimize_positions` produces ref's positions.

2. **Validate against ground truth, not goldens.** The reference binary is functional on this box
   (`reference-oracle-quadriflow`) and the reference has `serialize.hpp`. We **dump the reference's
   intermediate arrays** (`mAdj`/`mToLower`/`mToUpper` per level, `mQ` orientation field, singularity set,
   `mS`, `mO` position field, `edge_diff` pre/post-flow, the edge-graph levels) on the *same* input mesh,
   and diff ours against them numerically. This converts the unverifiable "validated together" into a
   strong **per-stage equivalence gate**. A stage is "done" when its array matches the reference dump
   within tolerance — long before the full pipeline runs end-to-end.

3. **Parallel pipeline behind a flag.** The current pipeline stays the default and stays green
   (148 tests, primitives at goldens) throughout. The faithful stack is built alongside it
   (`QuadRemesherOptions.FaithfulPort`, or a sibling `FaithfulParametrizer`), so no intermediate phase can
   regress production. Production goldens gate **only** the final default-flip (Phase 6).

This is the difference between this plan and every prior attempt: prior attempts mutated the *one* live
pipeline and were judged by goldens that the mutation necessarily disturbed. Here, correctness is judged
stage-locally against the oracle, and production is insulated until the whole faithful stack is proven.

---

## 4. Validation architecture (the linchpin — Phase 0 builds it)

Without this, the re-port is an un-bisectable big bang. With it, every phase has a sharp, automatable gate.

**4a. Reference state dumps.** Add a debug serialization path to our *copy* of the reference (it's MIT,
read-only for algorithm fidelity but we may add instrumentation in a build branch, or write a tiny
companion `dump.cpp`). Using `serialize.hpp`, dump after each reference stage, for a fixed input set
{sphere, torus, fandisk, plus one of happy/rocker}:
- `Hierarchy`: per level `mV`, `mN`, `mA`, `mAdj` (with order), `mToLower`, `mToUpper`, `mPhases` (coloring).
- After `optimize_orientations`: `mQ[0]` (+ per-level), and the final fine→coarse propagated `mQ`.
- After `ComputeOrientationSingularities`: the singularity index map.
- After `optimize_scale`: `mS`.
- After `optimize_positions`: `mO[0]`.
- `ComputeIndexMap`: `edge_diff` immediately after `BuildIntegerConstraints` (pre-flow) and after
  `ComputeMaxFlow` (post-flow); the edge-graph levels (`mF2E/mE2F/mEdgeDiff/mFQ/mToUpperEdges`).

Run once per input, commit the dumps under `tests/golden/quadriflow/state/` (small for primitives).

**4b. C# loader + per-stage diff harness.** Extend `FoldoverDiagnosticTests`/add `StateMatchTests`:
load a reference dump, run our faithful stack up to the same stage on the same input, and assert
equivalence with stage-appropriate tolerances and **invariance to representative choice** (4-RoSy fields
match modulo 90° rotation; positions modulo lattice translation; hierarchy match modulo a node-permutation
recovered from `mToLower`). Report worst-case per-element deviation.

**4c. Keep the existing instruments.** Cached post-solve grids (`QUADREMESH_DIAG=1`), `negArea`/χ/boundary
counters, `BenchmarkComparisonTests` (`QUADREMESH_BENCH=1`), and the `QuadriFlowSolverTests` goldens remain
the *end-to-end* gates (Phase 5/6). The state-diff harness is the *per-stage* gate (Phases 1–4).

**Tolerance note.** Bit-identity is *not* the goal (and is unattainable — ref uses PCG32/mt19937 RNG and
`std::shuffle` we will seed differently). The goal is **algorithmic equivalence**: same singularity set,
same field up to representative, same `edge_diff` up to the flow's documented non-uniqueness. Where ref
uses RNG (coloring shuffle, position init, Balance shuffle), match the *algorithm* and accept a different
deterministic seed; assert the *invariant* (valid coloring, converged field, balanced divergence), not the
exact array.

---

## 5. The phases

Each phase is independently buildable and testable (flag-gated; production untouched). "Gate" is the
pass condition. "Reuse" notes existing faithful code. Files are the primary touch-points.

### Phase 0 — Validation harness + parallel-pipeline scaffold + de-risking spike
**Goal:** make every later phase bisectable; prove the diagnosis cheaply before committing to the field rewrite.
- Build §4a reference state dumps and §4b C# diff harness. This is the bulk of Phase 0.
- Add `FaithfulPort` flag / `FaithfulParametrizer` skeleton that initially delegates to the current pipeline,
  so we can swap stages in one at a time.
- **Spike (cheap, high information):** on the *current* pipeline, point the extractor at the existing
  `EdgeGraphHierarchy.DownsampleEdgeGraph(...).back()` instead of the ad-hoc UnionFind coarse graph (D10).
  The subdivide auditor flags this as an independent hole source ("either alone reduces holes"), and unlike
  the prior failed field tweaks it is a **back-end consumption change**, not a field change — lower risk of
  the "holes move sideways" trap. Measure boundary edges on the cached dense grids.
- **Gate:** harness loads ref dumps and diffs a trivially-equal stage; primitives still green. Record the
  spike's hole delta (informational — informs whether D10-extractor is a quick partial win to land early).
- **Risk:** low. Worst case the spike regresses and we revert it (flag-gated); the harness is pure win.

### Phase 1 — Faithful `Initialize`: pre-pass + attributes + adjacency + hierarchy + coloring (D0,D1,D2,D3,D4)
**Goal:** reproduce ref's `Parametrizer::Initialize` + `Hierarchy::Initialize` state exactly.
- `NormalizeMesh` (+ un-normalize at output) — D11, cheap, do it here so coordinates match ref dumps.
- Pre-pass: replace full isotropic remesh with ref's guarded **split-only subdivide** to
  `min(scale/2, avg_edge*2)`, refine-only (D0). Keep the isotropic remesher available for the non-faithful
  default pipeline.
- Finest attributes: angle-weighted normals (Thürmer–Wüthrich) + Voronoi mixed area (D1).
- `V2E`/`E2E` directed-edge tables + angular-order `Neighbors` (D2). Our `HalfEdgeMesh` already stores
  twin/next/origin, so the dedge ring is reconstructable.
- `DownsampleGraph`: `Entry(dp*ratio)` priority + stable descending sort + greedy pair merge;
  area-weighted normal merge; add `mToUpper`; per-parent scratch-sort+dedupe coarse adjacency (D3).
- `generate_graph_coloring_deterministic` with the PCG32 shuffle, cached at build (D4).
- **Reuse:** CSR adjacency layout is fine (sort each slice ascending by id + accumulate dup weights).
- **Files:** `Mesh/MeshAttributes.cs`, `Mesh/HalfEdgeMesh.cs` (dedge tables), `Hierarchy/MultiresHierarchy.cs`,
  `Hierarchy/GraphLevel.cs` (+ `ToUpper`), `Fields/GraphColoring.cs`, new pre-pass in `QuadRemesher`.
- **Gate (§4b):** per level, `mToLower`/`mToUpper`/`mAdj` (order included), `mV`/`mN`/`mA` match the ref
  dump on sphere/torus/fandisk modulo the `mToLower` node-permutation. Coloring is a valid partition with
  the same class count.
- **Risk:** Med. This is the bedrock; if it's off, everything downstream drifts. The state-diff harness
  catches it here, not 4 phases later. Note: area-weighted normals regressed the torus *in isolation* on
  the old pipeline — here it's validated against the ref dump, not the torus golden, so that trap is gone.

### Phase 2 — Orientation field + singularities (D5, D6)
**Goal:** reproduce `mQ` and the singularity set.
- Re-port the update rule: `sum = sum*weight_sum + rep*weight`, self-as-reference-only (`weight_sum`
  starts 0), **per-neighbour project+normalize** (D5). Full `compat_orientation_extrinsic_4` 2×2 search
  returning the pair, propagate `value.first`, add `+1e-6` tie hysteresis.
- 6 iterations/level (not 20); add the final fine→coarse propagation pass.
- Crease constraints: swap the hard-pin for ref's `CQ/CQw` soft-blend-after-accumulation (only needed for
  the feature path; can stub for primitives and complete with P7).
- Singularities: `value.second−value.first` accumulation, **odd-only (mod∈{1,3})** filter; stop counting ±2.
- **Files:** `Fields/OrientationFieldSolver.cs`, `Fields/Rosy.cs`, `Fields/SingularityDetector.cs`.
- **Gate:** `mQ` matches ref dump modulo 90° per vertex (cosine of best representative ≥ 1−tol); singularity
  set matches the ref index map exactly on sphere(8)/torus/fandisk.
- **Risk:** High leverage, but with Phase 1 faithful the inputs are ref's, so the port should converge to
  ref's field. The singularity-count gap (memory: 13–68× QuadriFlow) is expected to collapse here.

### Phase 3 — Scale + position field (D7, D8)
**Goal:** reproduce `mO`; drive residual foldovers to ref levels (~0).
- Scale: thread the two-component per-vertex interface (`scale_x`,`scale_y`); run `optimize_scale` but in
  non-adaptive mode it sets `(1,1)` — a no-op for the lattice math, so this is interface plumbing, not a
  full solve. (Adaptive sizing stays deferred to P7.)
- Position: port the whole quartet **together** (the audit is explicit that piecemeal breaks coherence):
  `middle_point` + `position_floor_4` + `position_round_4` + the 4×4 corner search in
  `compat_position_extrinsic_4`; the running-reference inner loop with per-neighbour tangent reprojection;
  `position_round_4` final write (keep the integer cell — do **not** wrap to `v_i`); **absolute** prolong
  (re-project the coarse anchor, don't transfer offset-only); **remove `SelectStartLevel`** (solve the full
  hierarchy coarse→fine); random tangential init seeded deterministically.
- **Files:** `Fields/PositionFieldSolver.cs`, `Fields/ISizingField.cs`, new lattice helpers (likely in
  `Fields/Rosy.cs` or a new `LatticeMath`).
- **Gate:** `mO` matches ref dump modulo lattice translation on sphere/torus/fandisk; residual negative-area
  cell count on the extracted lattice matches ref (≈0). This is the **decisive** gate — it directly measures
  the hole precursor.
- **Risk:** Critical stage, but this is where the bottom-up strategy pays off most. The prior
  position-optimizer port failed *because* orientation/hierarchy differed; with Phases 1–2 faithful, that
  cause is removed. If `mO` still diverges with faithful inputs, the harness localizes it to a specific
  helper (floor vs round, midpoint, prolong) rather than "the field is wrong somewhere."

### Phase 4 — Integer solve: hierarchical flow (D9)
**Goal:** reproduce `edge_diff` post-flow; keep `maxDiff` bounded; route the torus handle.
- Restructure `Solve()` to hierarchical: `Balance()` **once at the finest level** (ref does no per-level
  Balance), then `DownsampleEdgeGraph` to build the flow's edge-graph levels (reuse the existing
  `EdgeGraphHierarchy` port), then flow **coarsest→fine** with capacity fixed at 2 except a level-0 growth
  loop, propagating down via `nEdgeDiff[i]=rshift90(EdgeDiff[toUpper[i]],(4−orient)%4)`.
- Decide the flow backend (open question §7): ref's *default* uses cost-blind max-flow at all levels with
  MCF opt-in at the coarsest only; our `MinCostFlow` (cost-1 SSP) is arguably better-behaved but is **not**
  what stock QuadriFlow does, so it selects a different (valid) integer solution. To *match the oracle*,
  replicate the max-flow-at-coarse + optional-MCF-at-level-0 selection. To *beat* it, keep MCF — but then
  the state-diff gate for this stage must be relaxed to "balanced + bounded `maxDiff`," not "matches dump."
- Add the `allow_changes==2` half-lock state (feature path) and the per-`sharp_colors` BFS balance.
- **Reuse:** `EdgeGraphHierarchy.DownsampleEdgeGraph/FixFlip/PropagateEdge`, `Lattice2D.Rshift90`,
  the existing branch `hierarchical-integer-flow` as a starting point (it had the Balance-ordering bug).
- **Files:** `Extraction/QuadriFlowSolver.cs`, `Acceleration/MinCostFlow.cs`, `Extraction/EdgeGraphHierarchy.cs`.
- **Gate:** torus handle routes to divergence 0 without OOM (the prior stall is explained: it came from
  Balancing in the wrong place — ref Balances once at finest *before* downsampling); `edge_diff` matches
  the ref post-flow dump (or, if keeping MCF, `maxDiff` ≤ ref's and divergence 0) on sphere/torus/fandisk.
- **Risk:** High. The torus genus-1 stall is now root-caused (Balance-once-at-finest), which de-risks the
  prior dead-end. The max-flow-vs-MCF decision is a genuine fork — see §7.

### Phase 5 — Faithful back end: subdivide / FixFlip / FixFlipSat / post-solver / extract (D10)
**Goal:** watertight on primitives **and** dense corpus.
- Restore ref ordering `Sub→FixFlip→Sub→FixFlipSat`; **drop the leading FixFlip** and the **early-stopping
  subdivide guard** (with a faithful field, ref's `exit(0)`-on-`|diff|>1` invariant should hold; if it
  fires we have a real upstream bug to fix, not paper over).
- **Wire FixFlipSat** into the pipeline using the existing CDCL backend, with `PushDownwardFlip` multi-depth
  5→0 descent and the threshold 1→4 outer loop (the SAT machinery is built; this adds the descent driver).
- **Extractor → `DownsampleEdgeGraph.back()`** for `F2E/E2F/EdgeDiff/FQ`, keeping the UnionFind/DisjointTree
  only for the geometry merge (`Vset`/`O_compact`) — exactly ref's two-role split. This is the direct hole
  fix (D10) and may already be partly landed from the Phase 0 spike.
- Add the post-solver `optimize_positions_sharp` + `optimize_positions_fixed` (re-optimize positions on the
  subdivided lattice before extract). The orchestrator audit flags this as the most consequential omission,
  coupled to the (now corrected) sweep count.
- **Reuse:** `subdivide_edgeDiff`, `FixFlip`/`CheckShrink`, `BuildTriangleManifold`, `FixValence`,
  `QuadHoleFiller`, `Cdcl`/`FlipSat`/`LocalSat` — all faithful already.
- **Files:** `QuadRemesher.cs` (`AlignLatticeForExtraction` ordering), `Extraction/QuadriFlowExtractor.cs`,
  `Extraction/EdgeGraphHierarchy.cs` (`PushDownwardFlip` + depth driver), `Extraction/IntegerGridMesh.cs`.
- **Gate:** sphere/torus/box watertight at goldens **on the faithful stack**; dense corpus (fandisk, happy,
  dragon, armadillo) watertight (χ matches input, 0 boundary) under `QUADREMESH_BENCH=1`, with irregular
  counts at-or-near QuadriFlow. FixHoles should revert to a near-no-op (its workload = genuine singularities
  only), confirming the field is now clean.
- **Risk:** Medium *if* Phases 1–4 land (the holes should already be largely gone before FixHoles); the
  remaining work is faithful ordering + wiring existing components.

### Phase 6 — Flip the default, cleanup, benchmark
- Make `FaithfulPort` the default once primitives stay at goldens and dense meshes are watertight.
- Decide the fate of the old pipeline: keep `QuadDominant` as the fast/robust fallback (it never holed);
  delete the now-dead divergent field code (old `LatticeRepresentative` wrap, `SelectStartLevel`, leading
  FixFlip, ad-hoc extractor coarsener, `IntegerOffsetSolver`/`OrientationComb` if still unused).
- Full `QUADREMESH_BENCH=1` re-measure vs goldens; update `ReferenceComparisonTests` expectations; refresh
  memory (`p6-quadriflow-status`, `foldover-holes-diagnosis`, `benchmark-corpus`).
- **Gate:** 148+ tests green, 0 warnings; primitives at goldens; dense corpus watertight; perf acceptable
  (perf is a separate P8 concern — dragon ~8 min).

---

## 6. Recommended sequencing

1. **Phase 0 first, always.** The harness is the safety net for everything; the spike either gives an early
   partial win (extractor→`DownsampleEdgeGraph`) or cheaply confirms it's not sufficient alone (consistent
   with the meta-finding) — either way it informs scope before the expensive field work.
2. **Strict bottom-up 1→5.** Do **not** skip ahead. The entire thesis is that stage *N* is only portable
   once 1..*N*−1 are faithful. Each phase merges to the faithful branch only when its §4b state-diff gate
   passes. The current pipeline stays default-green throughout.
3. **Phase 5 before Phase 6.** Don't flip the default until the dense corpus is watertight on the faithful
   stack; the goldens are the canary but the dense corpus is the actual goal.
4. **Decision gate after Phase 3.** Phase 3's foldover-count gate is the make-or-break: if a faithful field
   (Phases 1–3) drives residual foldovers to ~0 on the dense grids, the rest is faithful plumbing and the
   port is de-risked. If foldovers persist *with faithful inputs*, stop and re-examine — but the audits
   give no reason to expect that (the failures were all input-mismatch, now eliminated).
5. **Feature/crease soft-blend (D5 crease, D9 `allow_changes==2`)** can lag — primitives don't exercise it.
   Fold into P7 once the closed-mesh stack is watertight.

**Rough effort:** Phase 0 ~1 session (harness is the work). Phases 1–2 ~1–2 sessions each (mechanical but
must match dumps). Phase 3 ~1–2 sessions (the lattice quartet is fiddly; the gate is sharp). Phase 4
~1–2 sessions (restructure + the backend fork). Phase 5 ~1 session (mostly wiring faithful parts). Phase 6
~0.5 session. Total: a focused multi-session effort, **bounded and de-riskable at the Phase 0 spike and the
Phase 3 gate** — the two cheap places to confirm the thesis before paying for the rest.

---

## 7. Risks & open questions

- **Reference instrumentation cost.** Phase 0 needs the reference to emit state dumps. `serialize.hpp`
  exists, but wiring dumps means a local build tweak. Risk: the oracle build is finicky
  (`reference-oracle-quadriflow`: conda Eigen+Boost, VS 2026, CRT flags). Mitigation: a minimal companion
  `dump_main.cpp` linking the existing objects, or `#ifdef`-guarded dumps in our read-only copy on a build
  branch (algorithm stays untouched — we only *observe*).
- **Max-flow vs MCF fork (Phase 4).** The single biggest "match the oracle vs do better" decision. Ref's
  default is cost-blind max-flow (MCF opt-in at coarsest only); our cost-1 MCF is L1-minimal and arguably
  *better*, but selects a different integer solution, so it won't bit-match the ref `edge_diff` dump.
  Recommendation: **first match ref** (max-flow-at-coarse) so the state-diff gate is exact and we *know*
  the port is faithful; **then** optionally A/B MCF as a quality improvement once watertight. Don't conflate
  "faithful" with "improved" in the same step.
- **RNG seeds.** Coloring (PCG32), position init, Balance (mt19937 `std::shuffle`) are seeded RNG in ref.
  We will seed differently (harness-forbidden non-determinism aside, we want reproducibility). Gates must
  assert *invariants* (valid coloring, converged field modulo representative, balanced divergence), not the
  exact RNG-dependent array. Risk: a divergence hides behind "well, the RNG differs." Mitigation: where
  feasible, replicate ref's exact PRNG (PCG32 is ~40 lines, deterministic) so coloring/init *do* match.
- **The Phase 3 thesis could be wrong.** The whole plan bets that faithful inputs make the position port
  match. The audits strongly support this (every prior failure was input-mismatch), but it's a hypothesis.
  The Phase 3 foldover gate settles it cheaply — this is the designed kill-point.
- **Pre-pass (D0) interaction with our remesher.** Switching to split-only subdivide may *coarsen* less /
  preserve detail differently than our isotropic remesh; downstream field quality could shift. The state
  dumps are taken on the *same* pre-processed mesh ref produces, so as long as Phase 1 reproduces ref's
  pre-pass, this is controlled — but it means our nice isotropic remesher is bypassed in the faithful path
  (kept for `QuadDominant`).
- **Scope creep into perf.** The faithful stack (full hierarchy position solve, hierarchical flow,
  PushDownwardFlip descent) may be slower than today on dense meshes. Perf is explicitly **P8** — do not
  optimize during the port; just don't regress catastrophically (watch the armadillo cascade, now expected
  to vanish with a clean field + `exit(0)` invariant restored).
- **Keep the partial wins regardless.** `QuadHoleFiller` (shipped) and the extractor→`DownsampleEdgeGraph`
  spike are net-positive even if the full port stalls. The CDCL foundation is reused in Phase 5.

---

## A. Appendix — per-stage divergence detail (audit citations)

The six stage audits (file:line on both sides) are the source of §2. Key load-bearing citations, by stage:

- **D0/D11 orchestrator:** ref pipeline `main.cpp:47-128` + `parametrizer.cpp:31-128`
  (`Initialize`→orient→singular→**scale**→positions→ComputeIndexMap→`subdivide→FixFlip→subdivide→FixFlipSat→
  optimize_positions_sharp→optimize_positions_fixed→AdvancedExtractQuad(→FixHoles)→FixValence`); ours
  `QuadRemesher.cs:65-125` + `AlignLatticeForExtraction:148-154` (`FixFlip→Sub→FixFlip→Sub`, no SAT, no
  post-solver, no scale, no normalize). Resolution `×0.5` is faithful (`scale/2` field mesh,
  `parametrizer-mesh.cpp:60-71`). Pre-pass: ref split-only subdivide vs our full isotropic remesh.
- **D1/D2/D3/D4 hierarchy:** adjacency order `adjacent-matrix.cpp:8-33` (dedge ring) vs CSR by edge-id
  (`HalfEdgeMesh.cs:303-313`); coarsen priority `hierarchy.cpp:242-291` (`Entry dp*ratio`, `stable_sort`,
  `:250,:280,:286-291`) vs first-unmatched (`MultiresHierarchy.cs:102-134`); area-weighted normal merge
  `hierarchy.cpp:311` vs plain sum (`MultiresHierarchy.cs:145`); `mToUpper` (`hierarchy.cpp:298,315,328`)
  absent in ours; coloring PCG32 `hierarchy.cpp:195-239`. Finest normals angle-weighted
  `parametrizer-mesh.cpp:225-288`, Voronoi area `:298-322` vs Newell/barycentric (`MeshAttributes.cs:16-64`).
- **D5/D6 orientation:** update loop `optimizer.cpp:55-91` (running-reference `sum=value.first*weight_sum+
  value.second*weight`, per-neighbour project+normalize `:68-72`, `levelIterations=6` `:41`, final
  fine→coarse `:113-142`) vs flat average (`OrientationFieldSolver.cs:158-188`, 20 sweeps);
  `compat_orientation_extrinsic_4` `field-math.hpp:111-134` (2×2 + `1e-6` `:124`) vs B-side-only
  (`Rosy.cs:17-44`); singularities `parametrizer-sing.cpp:7-29` (`value.second-value.first`, odd-only
  `:22`) vs ±2-counting single-sided (`SingularityDetector.cs:27-77`).
- **D7/D8 position/scale:** `optimize_positions` inner loop `optimizer.cpp:290-349` (running `sum`,
  `compat_position_extrinsic_4` `:325`, per-neighbour reproject `:332`, `position_round_4` final `:347`),
  `compat_position_extrinsic_4`/`middle_point`/`position_floor_4`/`position_round_4`
  `field-math.hpp:136-208`, `optimize_scale` `:147-266` vs ours fixed-reference ROUND-wrap
  (`PositionFieldSolver.cs:177-233`, `SelectStartLevel:99-114`, offset-only prolong `:235-249`); the three
  lattice helpers are **missing** in ours.
- **D9 integer:** hierarchical descent `optimizer.cpp:1218-1349` (`:1223` coarsest-first, `:1328` level-0
  capacity growth, `:1338-1348` `rshift90` propagate), `DownsampleEdgeGraph` `hierarchy.cpp:417-638`,
  Balance `parametrizer-int.cpp:169-417` (once, at finest, per-`sharp_colors`), flow backends `flow.hpp`
  (NetworkSimplex `:111-192`, EC max-flow `:268-371`, Boykov) selected `optimizer.cpp:1284-1298` vs single
  level + global capacity (`QuadriFlowSolver.cs:456-534`), always cost-1 MCF (`MinCostFlow.cs:78-201`),
  no `allow_changes==2`. Torus stall root cause: Balance must run once at finest *before* downsample.
- **D10 subdivide/extract:** order `parametrizer.cpp:83,101,102,105` vs `QuadRemesher.cs:148-154`;
  extractor `AdvancedExtractQuad` uses `DownsampleEdgeGraph.back()` `parametrizer-flip.cpp:200,211-214`
  + DisjointTree for geometry `:205-262` vs ad-hoc UnionFind (`QuadriFlowExtractor.cs:55-142`);
  `PushDownwardFlip` `hierarchy.cpp:888-921` absent; `FixFlip`/`CheckShrink`/`BuildTriangleManifold`/
  `FixValence`/`FixHoles`/`QuadEnergy` all confirmed faithful. Hole mechanism: residual foldover +
  missing-SAT + early-stop subdivide guard → missing-twin boundary dedge → odd loop → unfillable.
