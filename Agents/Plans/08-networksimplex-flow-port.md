# 08 — Faithful integer FLOW: port LEMON NetworkSimplex to reach QuadriFlow parity on complex meshes

Status: **plan / thinking-through** (no code yet). Follows `07-field-stack-report.md` (the field+extraction
re-port, MERGED). Read with memory `faithful-integer-backend` (the happy diagnosis that motivates this) and
`foldover-holes-diagnosis`.

This plan exists because of a **surgical, fully-evidenced diagnosis**: after the faithful field + edge_diff +
comb + extraction port, the *only* remaining divergence from QuadriFlow on `happy` (the last non-watertight
mesh) is the **integer flow's choice of solution** among min-cost optima. This plan is about closing that
last gap — but it opens with a trade-off decision that must be made deliberately, because the obvious target
("bit-match QuadriFlow") is **not strictly "best."**

---

## 1. What we proved (the foundation this plan stands on)

Generated `happy` reference state-dumps and ran per-stage gates (`StateMatchTests`, `tests/golden/quadriflow/
state/happy/`). On happy, against the reference:
- **Orientation + singularities, position field, raw edge_diff, comb (faceEdgeOrients): ALL bit-exact.**
- **`ReferencePostFlowExtraction`** (decisive): the reference's *exact* post-flow edge_diff run through **our**
  extraction → happy **WATERTIGHT, χ=−10 = golden genus 6**. ⇒ **our extraction is faithful**; the holes are
  not an extraction defect and the post-solver is the wrong lever.
- **`OurFlowOnReferenceInput`**: fed the identical faithful raw integer state, **our** SSP min-cost-flow
  reaches divergence 0, maxDiff 2, and **lower total L1 than the reference** (happy 98275 < 98821) — but a
  *different* optimum (96% of edges identical). On happy that 4% flips extraction to **genus-7 + two 3-edge
  (odd) boundary loops** that `QuadHoleFiller` (even loops only) cannot quad-fill.

**Conclusion:** the field/edge_diff/comb/extraction are all faithful. The sole remaining lever is the flow's
**solution selection among equal-class optima**. The clean-extracting solution is *not* the one cost selects
(ours is cheaper yet extracts worse), so the only way to land on the reference's specific solution is to
**replicate LEMON NetworkSimplex** (the `-mcf` backend the goldens + dumps were generated with).

---

## 2. THE TRADE-OFF — read this before committing (it reframes the goal)

"Bit-match QuadriFlow's flow" makes our **entire pipeline bit-identical to QuadriFlow** (field→edge_diff→
comb→flow→[faithful extraction]). That is the strongest possible *robustness* guarantee: we inherit
QuadriFlow's proven behavior across the long tail of complex meshes. **But it is not uniformly "better"** —
it is uniformly *equal to golden*:

| mesh | our CURRENT flow (SSP MCF) | golden (= bit-matched target) | bit-match effect |
|------|------|------|------|
| happy | **holes**, genus 7 | **watertight genus 6** | **FIX** ✅ |
| armadillo | 146 (==golden already) | 146 | no change |
| sphere | 8 (==golden) | 8 | no change |
| **rocker-arm** | **56** | 70 | **REGRESS** 56→70 ✗ |
| **torus** | **16** | 22 | **REGRESS** 16→22 ✗ |
| fandisk | 56 | 44 | (improves 56→44, but crease path — see §6) |

Our current flow is a *different* min-cost optimum that, on some meshes (rocker, torus), yields **fewer
singularities than QuadriFlow** — a genuine quality win — but on happy yields holes. QuadriFlow's
NetworkSimplex is the opposite: not singularity-minimal, but battle-tested watertight across a wide corpus.

**So there are two distinct goals, and #1 (port NetworkSimplex) serves the first, not strictly the second:**

- **Goal A — Robustness / QuadriFlow parity.** Watertight + correct genus on the widest range of complex
  meshes, *proven* by matching the reference standard. Accepts golden-level singularity counts (loses the
  opportunistic rocker/torus wins). **This is what bit-matching NetworkSimplex delivers.**
- **Goal B — Pareto-best.** Watertight everywhere AND minimal singularities — keep our wins where our flow
  already beats golden, and fix only where it holes (happy). No single reference to validate against; needs a
  *hybrid* or a *repair* strategy, and is higher-risk/unbounded.

**Recommendation:** pursue **Goal A first** (this plan), because (a) "widest support of complex meshes" is
fundamentally a *robustness* property and QuadriFlow is the proven standard; (b) it is bounded and
bit-validatable against dumps; (c) it fixes the one hard correctness gap (happy). Then, as a *follow-on*
(§7, optional P9), layer a **hybrid** that opportunistically keeps our cheaper solution **when it extracts
watertight**, falling back to the NetworkSimplex solution only when ours holes — recovering the rocker/torus
wins without sacrificing robustness. Build the proven floor first; chase Pareto-best second.

> Decision needed from the user before Phase 2: **accept the rocker/torus singularity regression** as the
> price of proven robustness (Goal A), or require the hybrid (Goal B) up front. The plan assumes Goal A with
> the hybrid deferred; the hybrid only *adds* a selection step on top of a working NetworkSimplex, so
> committing to A does not foreclose B.

---

## 3. The precise target (file:line)

Replicate, bit-for-bit, the reference's `-mcf` flow so our post-flow `edge_diff` equals `state_int_post.bin`:

- **Driver:** `optimizer.cpp:1218 optimize_integer_constraints` — single level for us (`ComputeMaxFlow`
  calls `DownsampleEdgeGraph(...,1)`, `parametrizer-int.cpp:420`). Per capacity iteration: build
  `edge_to_constraints` + `initial` divergence; build **arcs in this exact order** — (1) conservation arcs
  in **edge-component order** (`edge_to_constraints` index), then (2) supply/demand arcs in **face-constraint
  order** (`initial` index); `resize(initial.size()+2, arc_count)`; add arcs; `compute()`; `applyTo`. Grow
  `edge_capacity` (start 2) **only at level 0**, ≤10 iters. Arc insertion order = LEMON arc IDs ⇒ it is
  load-bearing for the pivot rule and MUST match.
- **Backend:** `flow.hpp:111 NetworkSimplexFlowHelper`. `compute()` = `Preflow` (push-relabel) to get the
  scalar **maxflow value**, then `NetworkSimplex.costMap(cost).upperMap(capacity).stSupply(src, sink,
  maxflow).run()` (default `BLOCK_SEARCH`). `applyTo`: `edge_diff[var/2][var%2] -= sgn * flow[arc]`.
- **The min-cost-max-flow VALUE is unique**, so Preflow can be ANY correct max-flow (reuse our existing
  max-flow / `MinCostFlow` for the scalar). **The SOLUTION among optima is determined entirely by LEMON's**:
  - **`BlockSearchPivotRule`** (`network_simplex.h:342-411`): `block_size = max(sqrt(search_arc_num), 10)`;
    reduced cost `c = state[e]*(cost[e]+pi[source[e]]-pi[target[e]])`; circular block scan with `_next_arc`
    persisted across pivots; pick the most-negative `c` in the first block that contains a negative.
  - **`init()`** (`:1059-1247`): the artificial-root initial spanning tree (artificial arcs appended AFTER
    the real arcs, so their IDs are `m..m+n-1`; `_search_arc_num`, `_all_arc_num`), the
    `_source/_target/_cap/_cost/_supply/_state/_pi` arrays, `_parent/_pred/_thread/_rev_thread/_succ_num/
    _last_succ` tree threads.
  - **`start()`** (`:1573`), **`findJoinNode`** (`:1248`), **`findLeavingArc`** (`:1263`, includes the
    tie-break that picks among equal-ratio leaving candidates — load-bearing for the solution),
    **`changeFlow`** (`:1320`), **`updateTreeStructure`** (`:1343`), **`updatePotential`** (`:1469`).

LEMON 1.3.1 is vendored (`3rd/lemon-1.3.1/`, **Boost Software License** — permissive; read for
understanding, clean-room reimplement).

---

## 4. Validation architecture (already mostly built)

The Phase-0 harness from plan 07 is the gate; this plan extends it:
- **Dumps:** `generate-state-dumps.ps1 -Meshes <m>` (run with **Windows PowerShell 5.1** — no `pwsh` on this
  box; the script's incremental cmake build + oracle run work under 5.1). Have sphere/torus/happy; generate
  **fandisk, rocker-arm, armadillo, xyzrgb_dragon** for full-corpus flow validation.
- **The gate (sharpen to bit-exact):** `StateMatchTests.OurFlowOnReferenceInput` already feeds our flow the
  reference's faithful raw integer state and reports `% edges identical to ref post`. **Phase target: assert
  100%** (currently sphere 94.7 / torus 93.5 / happy 96.0 with SSP). When NetworkSimplex is faithful this
  becomes a hard `Assert.Equal(refPost[e], ours[e])` per edge.
- **End-to-end:** `ReferencePostFlowExtraction` (ref post-flow → our extract → watertight golden genus) is
  the *extraction* control; once the flow matches, `FaithfulBenchmark` should reproduce **every golden's
  topology** (happy watertight genus-6 included). Production `QuadriFlowSolverTests` goldens unaffected
  (faithful pipeline is parallel).

**Invariance note:** unlike plan 07's field stages (matched modulo gauge), the flow target is **literal
bit-identity** of `edge_diff` — because we proved the *specific* solution determines extraction topology, the
non-uniqueness that justified "don't bit-match" earlier is exactly what we must now pin down.

---

## 5. Phases (bottom-up, each bit-validatable; production untouched throughout)

### Phase 0 — De-risk + lock the target (cheap, high-information)
- **Confirm the graph construction order** our `QuadriFlowSolver.Solve()` builds matches the reference's arc
  order (conservation edge-component-order, then supply/demand). If not, align it — this alone changes which
  arcs get low IDs and is a prerequisite for any pivot-rule match.
- **Generate the dense dumps** (fandisk/rocker/armadillo/dragon) so the flow is validated on the corpus, not
  just primitives.
- **Smallest-first kill-point:** decide to validate **sphere** first (184320 edges, genus 0 — the simplest
  flow). If a faithful NetworkSimplex reproduces sphere's `state_int_post.bin` bit-for-bit, the port is
  sound and happy will follow; if sphere can't be matched, stop and re-examine before paying for the rest.
- **Gate:** dumps present; `OurFlowOnReferenceInput` instrumented to report per-edge bit-diff. **Risk: low.**

### Phase 1 — Faithful graph construction + capacity loop
- Restructure the flow setup to mirror `optimize_integer_constraints` exactly: arc order, node indices
  (0=source, 1..C=constraints, C+1=sink), caps `max(0,c±cap)`, cost 1 everywhere, the `allow_changes==2`
  half-lock asymmetric caps (`c>0 ? [c-1, -c+cap] : [c+cap, -c-1]`, arc_id negative), and the
  level-0-only capacity-growth `while(!fullFlow)` loop. Keep this as a NEW faithful flow path
  (`QuadriFlowSolver` already has the faithful constructor); do not disturb the existing SSP `Solve()`.
- **Reuse:** the existing `BuildConstraintGraph`/`Balance` (Balance is a no-op on closed meshes — keep it).
- **Gate:** the constructed (node,arc,cap,cost) lists match a dumped trace of the reference's graph on
  sphere (add a one-off reference instrumentation if needed, or infer from edge_to_constraints).
- **Files:** `Extraction/QuadriFlowSolver.cs` (new `SolveFaithful`), `Faithful/FaithfulPipeline.cs`.
- **Risk:** Med — the arc ordering is fiddly but mechanical and directly checkable.

### Phase 2 — Port LEMON NetworkSimplex core (the heart)
**SCOPED (read of `network_simplex.h` complete). The bit-match hinges on replicating these EXACTLY:**
- **Arc mixing** (`:939`, on by default): `skip = max(arc_num/node_num, 3)`; iterate arcs in **`ArcIt`
  order = REVERSE insertion** (SmartDigraph newest→oldest); assign internal id `i` scattering by `skip`
  (`i += skip; if (i>=arc_num) i = ++j`). The pivot rule scans these *mixed* internal ids — so the
  permutation is load-bearing. Replicate from the (controlled) insertion order + arc_num/node_num.
- **`init()` EQ branch only** (`:1119`): our `stSupply(src,+k; sink,−k)` ⇒ `_sum_supply==0`. Artificial root
  `_root=node_num`; one artificial arc per node (ids `arc_num..arc_num+node_num−1`), `_search_arc_num=arc_num`,
  `_all_arc_num=arc_num+node_num`. Supply≥0 node → DIR_UP, π=0, art arc u→root cost 0; else DIR_DOWN,
  π=ART_COST, root→u cost ART_COST (`ART_COST = max(|Cost|)/2+1` for exact int).
- **`initialPivots()`** (`:1483`): EQ, supply_nodes=[src], demand_nodes=[sink] (size 1 each) ⇒ the
  **reverse graph search from sink to source via `InArcIt`** (reverse-insertion in-arc order at each node),
  collecting arcs with `_cap[j] >= total` (total=maxflow), then heuristic-pivoting them. Then the main loop.
- **`BlockSearchPivotRule`** (`:342`), **`findJoinNode`/`findLeavingArc`/`changeFlow`/`updateTreeStructure`/
  `updatePotential`/`start`** (`:1248-1591`): port verbatim. Note `findLeavingArc` uses **`<` on the
  first-half scan and `<=` on the second-half** (the tie-break that picks the leaving arc — load-bearing).
- **SmartDigraph iteration order** is the subtle dependency: `ArcIt` and `InArcIt` both iterate
  **reverse-insertion**. Model in-arc lists per node (insertion order, iterate reversed).
- Clean-room reimplement `NetworkSimplex` (BLOCK_SEARCH only) in `Acceleration/NetworkSimplex.cs`:
  the `init()` artificial-root basis, the tree threads (`_parent/_pred/_thread/_rev_thread/_succ_num/
  _last_succ/_state/_pi/_flow`), `BlockSearchPivotRule.findEnteringArc`, `findJoinNode`, `findLeavingArc`
  (with LEMON's exact leaving-candidate tie-break), `changeFlow`, `updateTreeStructure`, `updatePotential`,
  `start`. Int costs/caps (`Weight=Capacity=int`); supply = stSupply(src,+maxflow; sink,−maxflow).
- **Determinism checklist (where bit-matching lives):** arc IDs = insertion order; artificial arcs appended
  after; block size formula; circular `_next_arc`; strict-`<` vs `≤` in entering/leaving selection;
  `_next_arc` reset rules; potential update sign. Each is a place a subtle deviation silently picks a
  different optimum — caught by the sphere dump diff.
- **Gate:** standalone unit tests vs hand-checked small instances; then **sphere `state_int_post.bin`
  bit-exact** through `OurFlowOnReferenceInput` (the kill-point).
- **Files:** `Acceleration/NetworkSimplex.cs`, tests `NetworkSimplexTests.cs`.
- **Risk:** **High** — this is the load-bearing port. Mitigation: the dump diff localizes the first diverging
  pivot; bisect by logging entering/leaving arcs against an instrumented reference run if needed.

### Phase 3 — Wire Preflow value + integrate
- Compute the maxflow scalar via existing max-flow (value is unique). Run NetworkSimplex with that supply.
  `applyTo` → updated `edge_diff`. Loop capacity at level 0.
- **Gate:** **sphere + torus** post-flow bit-exact (100% in `OurFlowOnReferenceInput`).
- **Risk:** Low once Phase 2 lands.

### Phase 4 — Validate the corpus + flip the faithful flow on
- Bit-exact gate on **happy** (the target) + **fandisk/rocker/armadillo/dragon** (regenerated dumps). Then
  point `FaithfulPipeline` at `SolveFaithful` and run `FaithfulBenchmark`: expect **happy watertight
  genus-6**, and every mesh's topology == golden (accept the rocker/torus singularity change per §2).
- **Gate:** all dumped meshes bit-exact post-flow; `FaithfulBenchmark` 7/7 watertight, genus == golden.
- **Risk:** Med — a dense mesh may expose an unhandled NetworkSimplex path (e.g., degeneracy/anti-cycling)
  the primitives didn't; the per-edge dump diff localizes it.

### Phase 5 — (Optional, geometry) post-solver
- Port `optimize_positions_sharp` / `optimize_positions_fixed` (`optimizer.cpp`) for vertex-position fidelity
  on the subdivided lattice. **Topology is already golden from Phase 4**; this is geometry polish (matches
  the reference's vertex placement before `ProjectToSurface`). Lower priority.

### Phase 6 — Productionize
- Decide whether `FaithfulPipeline` becomes the default `PureQuad` path; wire through `QuadRemesher`/CLI;
  retire the old SSP flow path (or keep behind a flag for the §7 hybrid); refresh benchmarks + memory.

---

## 6. Crease / feature meshes (widest-support requires this)
The `allow_changes==2` half-lock + sharp-edge integer locks (`parametrizer.cpp:74-86`,
`QuadriFlowSolver.ApplySharpConstraints`, memory `p7-features-status`) ride along the flow. fandisk's 56-vs-44
gap and the broader CAD story need the orientation `CQ/CQw` soft-blend (`optimizer.cpp:75-86`) too. The flow
port must thread `allow_changes` (Phase 1 does); full crease quality is a sibling effort, not blocked by this.

---

## 7. Goal-B follow-on (optional P9) — recover the rocker/torus wins without losing robustness
Once NetworkSimplex is faithful, add a **hybrid selector**: run BOTH our cheaper SSP solution and the
NetworkSimplex solution; **extract each (cheaply, topology-only) and keep the one that is watertight with
fewer singularities** — defaulting to NetworkSimplex when ours holes. This is Pareto-best (happy from NS,
rocker/torus from SSP) at ~2× flow cost. Gate on watertightness, never ship a holed result. Strictly additive
on top of Phase 4; do not attempt before the faithful floor exists.

---

## 8. Risks & open questions
- **Bit-matching LEMON is the central risk.** A faithful reimplementation IS deterministic and WILL match
  given identical arc IDs + pivot rule + tie-breaks — but the tie-breaks are subtle (strict vs non-strict
  comparisons, `_next_arc` carry, leaving-arc selection among equal ratios). Mitigation: the **sphere
  kill-point** + per-edge dump diff make it bisectable, exactly like the field port.
- **Goal-A regresses rocker/torus singularity counts.** Surfaced in §2; needs a user decision. The §7 hybrid
  recovers them later.
- **Preflow scalar.** Only the maxflow VALUE is needed (unique) — low risk; reuse existing max-flow.
- **Capacity-growth interaction.** The reference re-runs Preflow+NS from scratch each capacity iteration;
  replicate (don't warm-start) to match.
- **Performance.** Managed NetworkSimplex on dragon (250k v, ~225k edges) may be slower than LEMON's; the
  reference's index-map took ~60s on happy. Perf is **P-perf**, not correctness — don't optimize during the
  port, but watch for pathological pivoting on dense meshes.
- **Non-goal:** *beating* QuadriFlow's flow (a provably extraction-optimal flow) is research and explicitly
  out of scope here — Goal A targets parity; the §7 hybrid is the bounded step toward Pareto-best.

---

## 9. Recommended sequencing
1. **Phase 0 + the sphere kill-point first, always** — cheapest confirmation the port is achievable before
   paying for the full NetworkSimplex.
2. Strict bottom-up 1→4; merge each phase to a branch only when its bit-exact gate passes; production stays
   default-green.
3. **User decision gate before Phase 4 flips the default:** accept Goal-A's rocker/torus regression, or
   require the §7 hybrid in the same release.
4. Phase 5 (geometry) and §7 (hybrid) are independent follow-ons.

**Effort:** Phase 0 ~0.5 session; Phase 1 ~1; Phase 2 ~2–3 (the real work — NetworkSimplex + bit-match
debugging); Phase 3 ~0.5; Phase 4 ~1 (+ dump regen); Phase 5/§7 ~1 each. The kill-point (sphere bit-exact)
de-risks the whole thing within Phase 2.
