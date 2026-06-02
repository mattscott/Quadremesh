# 03 — Tracked follow-ups / side-quests

Items that are real but deliberately *not* on the v1.0 critical path.

## F1 — Audit the Nanomesh library *(requested)*

> **Status: decoupled, no longer gating.** Quadremesh is now **clean-room** — no code is lifted from Nanomesh
> ([06 §6.5](../Docs/06-csharp-portability-analysis.md#65-relationship-to-nanomesh-clean-room-no-code-reuse)).
> So this audit is **no longer on the reuse path** and gates nothing. It survives as an optional
> side-quest: improve Nanomesh for its own sake, and sanity-check our clean-room `HalfEdgeMesh`
> connectivity design against a working reference. Do it anytime, or not at all.

**Why:** `Nanomesh` is read as a *reference* for connectivity patterns (the `ConnectedMesh`
position/sibling/relative node model, `MinHeap`, decimation matching). Auditing it for bugs and
architectural issues both de-risks that reference reading and lets Nanomesh itself be improved.

**Scope of the audit**
- Correctness of `ConnectedMesh` edge-collapse / sibling-relative bookkeeping (the
  `CollapseEdge`/`ReconnectSiblings` logic is intricate and assertion-heavy — verify invariants).
- `MergePositions`/`Compact` remapping correctness and O(n) behavior.
- Numerical robustness of the quadric decimation (`SymmetricMatrix`, `EdgeCollapse`).
- API/encapsulation issues (public mutable arrays, `// Todo: make private`), triangle-only
  assumptions (`throw new NotImplementedException()` on non-tris).
- Test coverage and whether there is a regression corpus.

**Deliverable:** a findings doc (`Docs/nanomesh-audit.md`) + optionally a PR of fixes back to
Nanomesh.

**When:** anytime — no longer gating any Quadremesh phase (we copy nothing). Most useful if/when our
clean-room connectivity work wants a reference to validate intricate edge-collapse invariants.

> Suggested execution: a focused multi-agent review pass over the Nanomesh sources
> (correctness + architecture lenses), adversarially verified, then synthesized into the findings
> doc. Can be kicked off independently of Quadremesh phases.

## F2 — Hard-surface / CAD mode (QuadWild-style patch layout)

Feature-line-driven coarse patch layout → pure-quad per-patch tessellation, for crisp creases on
mechanical parts. Built *on top of* the field solver; post-1.0. (Study QuadWild for ideas; do not
copy GPL code.)

## F3 — Robust scan reconstruction (VDB-style pre-clean)

Voxel-shell / marching-cubes reconstruction to repair self-intersecting or shelled scan input before
remeshing (autoremesher does this with OpenVDB). Post-1.0 robustness nicety; implement a managed
voxelizer rather than depending on OpenVDB.

## F4 — DCC / engine integration

Blender add-on, Unity/Unreal importers, or a thin native shim — enabled by the clean v1.0 API.
Post-1.0.

## F5 — UV / material / attribute transfer

Carry UVs, vertex colors, and material assignments from input to remeshed output (CurioMesh does
this around QuadriFlow). Post-1.0.
