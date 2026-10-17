---
paths:
  - "drone-dtp/**"
  - "docs/dtp-follower-stack.md"
  - "scripts/dtp_follower.sh"
---

# `drone-dtp/`: what it is, where it lives, how it is built

Scope: the human-following stack's place in this repo and in `~/super_ws`, and the build and branch traps that have deleted or shadowed it. Split out of the root `CLAUDE.md` on 2026-09-17. The text is as it was there, apart from cross-references. Add new findings for this area here, dated, not in `CLAUDE.md`.

- **M4/M5 (vision goal adapter, tuning/safety) — superseded 2026-09-12 by
  [`docs/m5-m6-plan.md`](docs/m5-m6-plan.md)** (Gate 0 → M5 "the follower flies" → M6 "the
  paper's contribution"). **M4 ALREADY EXISTS.** A complete human-following perception + goal stack is installed and running on this
  Jetson from `~/super_ws/src/drone-dtp` (`dtp_msgs`, `dtp_perception`, `dtp_bringup`,
  `command_mux`, `follow_planner`): YOLO/BoT-SORT on the Gemini 336L → one locked target in
  `lidar_init` → a standoff viewpoint → a moving goal into SUPER. **Since 2026-09-12 that path is a
  symlink to this repo's `drone-dtp/` (branch `dtp-follower-stack`)**, and it needs a SUPER patch
  (`YAW_TO_TARGET` + follow-goal ingestion + `GoalStatus`) that `scripts/setup.sh` knows nothing
  about — so a clean install of this repo yields a SUPER it cannot drive. Read
  `docs/dtp-follower-stack.md` before doing any M4 work here; don't rebuild what is already
  written. See `docs/ImplementationDoc.md` for the original plan.

- `drone-dtp/` — **the human-following stack (M4), only on branch `dtp-follower-stack`.**
  `~/super_ws/src/drone-dtp` is a symlink to it, built `--symlink-install`, so edits here are what
  the drone runs (unlike `super_bringup/`, which is copied). Since 2026-09-13 (`b78fbc3`) `setup.sh`
  symlinks it into `~/super_ws/src` and applies its SUPER patch when the folder is present. How it is
  wired, built and run: `drone-dtp/README.md`; what the nodes do: `docs/dtp-follower-stack.md`.
  Its `patches/0001-…` is the SUPER follow-goal patch, **regenerated 2026-09-13 (G0.4)** —
  current (`requested_p` present), applies to `2ad3419`+isnan, and applied by `setup.sh`.

- **⛔ The human-following stack is [`drone-dtp/`](drone-dtp/), on branch `dtp-follower-stack`
  (`28d366b`), and the drone builds and runs it straight from this repo.** `dtp_msgs`,
  `dtp_perception`, `dtp_bringup`, `command_mux`, `follow_planner`, ~12.6k lines of Python +
  1.3k of C++ — in substance **M4**. Read [`docs/dtp-follower-stack.md`](docs/dtp-follower-stack.md)
  before doing any M4 work — do not rebuild what is already written.
  **Since 2026-09-12 `~/super_ws/src/drone-dtp` is a SYMLINK to this repo's `drone-dtp/`**, and the
  five packages are built `--symlink-install`, so the repo folder *is* what the drone runs — the
  user's decision, over a separate overlay workspace. **The full write-up is
  [`drone-dtp/README.md`](drone-dtp/README.md)**: the link chain per file type, the edit → relaunch
  / rebuild table, build and test commands, a re-runnable wiring check, undo, and a fresh-install
  checklist. The traps that bite mid-task, all hit on 2026-09-12:
  (1) **checking out any other branch deletes `drone-dtp/` from disk** and no DTP node starts — fly
  from `dtp-follower-stack`. (2) **Never two `colcon build`s at once** — two started 1 s apart
  deleted each other's objects (`ld: cannot find …type_support.c.o`). (3) Changing a package between
  plain and `--symlink-install` needs its `build/` **and** `install/` removed first; for
  `ament_python` the old copied modules otherwise *silently* shadow the edited ones.
  **`super_planner` is a plain build** — don't pass it `--symlink-install`. (4) Source
  `dtp_env_jetson.sh` via `~/super_ws/src/drone-dtp/…`; via the repo path it finds no workspace.
  (5) `rm -rf ~/super_ws/src/drone-dtp/` **with a trailing slash deletes the repo folder's
  contents**. (6) Backups of the tree go outside `~/super_ws` — colcon crawls all of it
  (`base_paths=['.']`); the pre-symlink tree is at `~/drone-dtp.pre-symlink-2026-09-12`.
  **The branch is pushed** (`origin/dtp-follower-stack` at `f4df2aa`, 2026-09-13) — but anything left
  uncommitted or unpushed in the tree is still lost to a reflash. The tree
  was a tar-extracted subset of an upstream repo (`dtp-imp` branch, per `scripts/deploy_calib.sh`)
  that is not on this box; edits here diverge from it.

See also: `dtp-follow-behaviour.md` (what the nodes do and their settings), `m6-loss-oracle.md`, `dtp-calibration.md`.
