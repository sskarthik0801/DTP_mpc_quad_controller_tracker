---
paths:
  - "docs/paper/**"
  - "docs/m5-m6-plan.md"
  - "docs/m6*.md"
---

# The M5/M6 plan and the paper draft

Scope: the milestone plan and the rules for editing the conference paper. Split out of the root `CLAUDE.md` on 2026-09-17. The text is as it was there, apart from cross-references. Add new findings for this area here, dated, not in `CLAUDE.md`.

- **▶ THE M5/M6 PLAN IS WRITTEN: [`docs/m5-m6-plan.md`](docs/m5-m6-plan.md).** The user asked for
  it, and for numbers in it. It re-cuts the milestones:
  - **Gate 0** — safe to fly autonomously at all;
  - **M5** — the existing DTP follower flies (absorbs the open M4 checklist and the old M5 list);
  - **M6** — the paper's contribution exists and is evaluated.

  It carries a dated schedule starting 2026-09-14, every pass threshold, sample sizes, a compute
  budget and logistics. Its §1 maps each paper claim to what is doable. **Four claim changes are
  proposed there but NOT applied to `main.tex`** (drop live Eva-Tracker, scripted-event + video
  ground truth instead of RTK, fly CTRV only, "search" = hold-and-climb). They are the authors'
  call.

- **A conference paper draft now exists: [`docs/paper/`](docs/paper/)** (ICRA/RA-L, LaTeX,
  uncompiled — no TeX on this box). Its thesis is map-conditioned classification of *why* the
  target was lost. **Two macros are load-bearing: `\tbd{}` = not measured yet, `\bench{}` = a real
  measurement but bench/handheld, NEVER in flight.** Never drop a `\bench{}` wrapper except by
  replacing the number with its in-flight equivalent. `docs/paper/README.md` traces every
  `\bench{}` figure back to the file in this repo it came from, and lists what blocks submission.

See also: `m6-loss-oracle.md` (what has been built and measured).
