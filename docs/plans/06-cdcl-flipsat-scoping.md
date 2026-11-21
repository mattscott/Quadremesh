# 06 — Scoping a CDCL SAT solver for FixFlipSat (clearing residual foldovers)

Status: **scoping / research** (no code yet). Author handoff doc. Read with
[[../../memory]] `foldover-holes-diagnosis` and `04-quadriflow-integer-port.md`.

## 0. Why this, why now

Dense/organic PureQuad outputs leave **holes** (fandisk 6, happy 38, dragon 80 boundary edges after
FixHoles). The whole investigation converged on one cause:
**residual parametrization foldovers** — negative-area lattice cells that survive `FixFlip`. They are
locally consistent (divergence 0) but geometrically inverted, so the extractor can't build a quad there →
boundary loop → hole. ~99% sit in smooth regions (not at field cones), i.e. they are an artifact of the
integer map, not inherent singularities.

What we ruled out (all in memory, don't repeat):
- **extended CheckShrink** (FixFlip max_len 2→4): residual foldovers are a *fixed point* of edge-ring
  shrinking — no change.
- **position-optimizer port / hierarchical flow**: don't target foldovers (and regress torus/happy).
- **FixFlipSat with our naive DPLL**: the *right* mechanism (QuadriFlow uses it), but the hole-causing
  foldovers sit in **large** flexible groups (≈4230 edges at happy's coarsest level) that a naive
  O(clauses)-propagation DPLL can't solve in reasonable time, so we cap group size and skip them — and
  skipping the big groups is exactly skipping the holes. **CDCL removes that wall.**

QuadriFlow's `Parametrizer::FixFlipSat` (hierarchy.cpp:640) + `SolveSatProblem` (localsat.cpp) is the
reference. It shells out to **minisat**. We can't take a process dependency (Core is zero-dep, single
managed artifact), so we need an **in-process CDCL solver**. We already ported everything *around* it
(`FlipSat.cs` marking/grouping/constraint-build, `LocalSat.cs` CNF encoding + a DPLL). **This plan scopes
replacing the DPLL backend with a real CDCL solver.**

## 1. The exact problem the solver must handle

`LocalSat.Solve` already builds the CNF (faithful to `SolveSatProblem`). Per flip-fix group:

- **Variables**: one per edge-diff component, each ∈ {−1, 0, +1}. Encoded as **3 boolean SAT variables**
  per integer variable (value-booleans `b₋₁, b₀, b₊₁`) with hard clauses for **exactly-one**
  (`at-most-one` pairwise + `at-least-one`).
- **Equality constraints** (face divergence = 0), 2 per face: enumerate the 27 (v₀,v₁,v₂) combos, forbid
  those with `cst·v ≠ 0` (a 3-literal clause each).
- **Inequality constraints** (face lattice area ≥ 0), 1 per face: enumerate the 81 (v₀,v₁,v₂,v₃) combos,
  forbid `cst₀·v₀v₁ − cst₁·v₂v₃ < 0` (a 4-literal clause each).
- **Fixed boundary variables**: edges adjacent to the group but outside it are pinned (`at-least-one` of
  the single current value).

Instance scale (worst observed, happy coarsest level): ~4230 flexible edges → ~8460 int vars → **~25k
boolean vars**, and ~3000 faces × (~27 eq + ~81 ge clauses) → **~300k clauses**. This is *tiny* for CDCL
(industrial solvers do 10⁶–10⁷ vars), but **out of reach for naive DPLL** with linear-scan propagation.

Important properties that shape the design:
- **Mostly satisfiable & structured** — the field is already consistent; we want a flip-free assignment
  *near the current one*. So the solver should be **minimal-change biased** (phase-saving toward current
  values) — a satisfying assignment that flips many edges would disturb good regions.
- **Pure SAT, not MaxSAT** — the reference writes *all* clauses (eq + ge) to minisat as hard; on UNSAT
  the group is left unchanged (foldover stays). We match that. (`sat_ishard` is tracked but unused in
  `RunCNF`.) MaxSAT — satisfy eq hard, *maximize* ge satisfied — is a possible future enhancement (see §7).
- **Many independent groups per pass**, across thresholds 1–4. Groups are independent → **one-shot solver
  per group**, no cross-group incrementality needed. Keep it simple.
- **Determinism is mandatory** — no `Date.now`/`random` (harness-forbidden anyway). Break VSIDS ties by
  index; fixed restart schedule.

## 2. Reference material & licensing

Clean-room, same as the rest of the QuadriFlow port. Read for understanding, implement ourselves.
- **MiniSat — "An Extensible SAT-solver", Eén & Sörensson (2003)** — the canonical, complete spec
  (search/propagate/analyze/VSIDS). MiniSat is **MIT** → safe to study. This is what QuadriFlow actually
  invokes, so matching its behavior is the faithful replacement.
  http://minisat.se/ , paper: lara.epfl.ch/w/_media/projects:minisat-anextensiblesatsolver.pdf
- **Marijn Heule's microSAT** (~200 lines C, MIT) and **microsat-cpp** (stbrumme) — minimal reference for
  the single-array data-structure style; good for sizing.
- CDCL handbook chapter (Marques-Silva/Lynce/Malik) + Aalto course notes — algorithm spec for 1-UIP.
- AVOID `MiniSAT.cs` (pjt33) — no visible license.

## 3. Algorithm design (the components, with our choices)

Standard CDCL = DPLL + (a) two-watched-literal propagation, (b) 1-UIP clause learning, (c) VSIDS, (d)
restarts, (e) phase saving. Our choices:

1. **Two-watched-literal unit propagation** — *the* essential upgrade (our DPLL's O(clauses)-per-fixpoint
   scan is the actual bottleneck). Each variable keeps a list of clauses watching one of its literals;
   only the negated-literal's watchers are visited on assignment; move the watch or propagate/conflict.
2. **1-UIP conflict analysis** — on conflict, resolve the conflicting clause against antecedents of the
   most-recently-assigned current-level variables (walk the trail backward) until exactly one current-level
   literal remains; that's the learned clause. Backjump to the **second-highest** decision level in it.
   Antecedent (reason) pointer + decision level per assigned variable; assignment trail with level marks.
3. **VSIDS** decision heuristic — per-variable activity, bumped for vars in learned clauses, ×0.95 decay
   per conflict; pick the highest-activity unassigned var (ties → lowest index, for determinism).
4. **Restarts** — Luby or geometric (start ~100 conflicts, ×1.5). Keeps hard groups from getting stuck.
5. **Phase saving / minimal-change polarity** — initialize and save each boolean's preferred phase toward
   the **current** integer value (the reference's "init to current, flip only if needed"). This gives the
   minimal-change behavior we need without a full optimization objective.
6. **Budget & graceful failure** — a conflict budget per group; on exceed return `Timeout`, leave the
   group unchanged (foldover stays). Same contract as today's `LocalSat.Status`.

Deliberately **omit** (industrial extras, not needed at our scale): clause-DB reduction/forgetting,
clause minimization, inprocessing, blocked-clause elimination. Add only if profiling demands.

## 4. Data structures (zero-dep managed C#)

- Boolean vars `1..n`; literal `= var<<1 | sign` (or signed int, as today).
- `int[] assign` (var → 0/1/unset), `int[] level`, `int[] reason` (clause id or −1 for decision/none).
- `List<int> trail`, `List<int> trailLim` (decision-level boundaries).
- Clauses: flat `int[]` per clause in a `List<int[]>` (learned clauses appended). Watches: `List<int>[]`
  indexed by literal (clause ids). (microSAT's single-big-array arena is an optional later optimization;
  start with the simple array-of-arrays — clearer, fast enough.)
- `double[] activity`, a cheap max-scan or binary heap for VSIDS (max-scan is fine at 25k vars; heap if
  profiling shows it matters).

## 5. Integration (small, localized)

The CNF build and the FixFlipSat plumbing already exist. The change is **backend-only**:
1. Replace `LocalSat.Dpll` with `LocalSat.Cdcl` (same `Solve(...)` signature + `Status`). Nothing else in
   `LocalSat` changes (the clause generation stays).
2. In `FlipSat.RunSat`: **remove the `MaxGroupEdges` cap** (or raise it sharply) and rely on the CDCL
   budget for graceful failure. Keep per-group skip only as a last-resort safety.
3. Run FixFlipSat on the **coarse** graph as already wired (`EdgeGraphHierarchy.RunSat` /
   `IntegerGridMesh.FixFlipSat`).
4. **Wire it into the production pipeline** (currently NOT wired): in `QuadRemesher.AlignLatticeForExtraction`,
   after the existing FixFlip→Sub→FixFlip→Sub, add `if (grid.NegativeAreaFaceCount() > 0) grid.FixFlipSat();`
   then a final `SubdivideEdgeDiff(1)`. Gate so sphere/torus (negArea 0) are byte-identical.

## 6. Milestones (incremental, each independently testable)

- **M0 — De-risk the payoff (DO THIS FIRST, cheap).** Before building CDCL, confirm that a *fully solved*
  FixFlipSat actually closes the holes. On the cached fandisk grid (largest group ~402 edges), run the
  existing DPLL with the size cap removed and a large budget (slow, minutes OK). Measure: does negArea→~0
  and does extract become watertight (boundary 0)? Also check happy (may be too slow for DPLL — that's the
  point, but try one threshold). **If holes don't close even when fully solved, STOP — CDCL won't help**
  and the real lever is field quality. If they close, CDCL is justified (it just makes it fast enough).
- **M1 — CDCL core, unit-tested in isolation.** Implement `Cdcl` (watched literals + 1-UIP + VSIDS +
  restarts + phase saving). Extend `LocalSatTests` with: random 3-SAT SAT/UNSAT instances cross-checked
  against the existing DPLL (differential testing); pigeonhole UNSAT (small, forces learning); the
  existing semantic tests. Add a stress instance (~10k vars) to confirm it's fast.
- **M2 — Swap backend, keep cap.** Point `LocalSat.Solve` at `Cdcl`. Re-run `FlipSat`/`FixFlipSat`
  experiments on cached grids; confirm same-or-better foldover clearing, much faster, no regressions to
  the existing (capped) behavior.
- **M3 — Remove the cap, measure.** Uncapped FixFlipSat on cached grids (fandisk/happy/dragon/armadillo):
  report negArea before/after and the extract boundary. Target: negArea→~0, boundary→0.
- **M4 — Wire into production + validate.** Gate on negArea>0. Full test suite green; **sphere/torus/box
  byte-identical** (negArea 0 → no-op); `QUADREMESH_BENCH=1` shows watertight (χ matches input) on the
  dense corpus while keeping irregular counts. Re-measure vs goldens.
- **M5 — Perf + cleanup.** Profile (the big fine-level groups). Only if needed: clause-DB reduction,
  heap-based VSIDS, microSAT-style arena. Delete dead code (old single-level `Balance` paths if unused),
  trim diagnostics.

## 7. Risks & open questions

- **Payoff is a hypothesis** (M0 settles it). Capped-DPLL FixFlipSat reduced *some* foldovers but boundary
  was unchanged because the hole-causers were in skipped large groups. M0 verifies that solving those
  large groups closes the holes.
- **Minimal change vs feasibility.** Pure phase-saving biases toward current values but a satisfying
  assignment can still flip a region. If that creates problems (e.g. shifts singularities), consider
  partial-MaxSAT (hard eq + soft "keep current"/area) — bigger effort; defer unless observed.
- **Pure SAT leaves genuinely-UNSAT groups unfixed** (foldover stays → small residual hole). That's the
  reference behavior; FixHoles still patches small even loops. MaxSAT could reduce these.
- **Performance at the fine level.** FixFlipSat runs on the coarse graph (small), so SAT instances stay
  modest; but if we ever run it finer, watch clause counts. Budget + restarts bound worst case.
- **Determinism / parallelism.** Keep the solver single-threaded and index-deterministic; FixFlipSat
  groups could be solved in parallel later (independent), but start sequential.
- **Must-not-regress**: sphere(8)/torus(≈24)/box(watertight, ≤16 irregular) goldens — the negArea>0 gate
  makes FixFlipSat a no-op there, so they should be untouched; assert it.

## 8. Effort estimate

- CDCL core (M1): ~400–700 lines C# + ~150 lines tests. The well-understood part.
- Integration (M2–M4): small (backend swap + one gate + validation runs, each dense mesh is a slow solve).
- M0 is a few hours of runs (no new code beyond toggling the cap).

Net: a focused multi-session effort, but **bounded and de-riskable at M0**. If M0 fails, we abandon
cheaply and pivot to field quality.
