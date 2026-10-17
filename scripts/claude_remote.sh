#!/usr/bin/env bash
#
# Claude Code Remote Control on this Jetson -- the process the web GCS's
# "Claude Remote Control" row starts.
#
#   bash scripts/claude_remote.sh
#
# Then open the Code tab in the Claude mobile app, or claude.ai/code, and pick
# the session named "gypsy".  It works in this repo, with its CLAUDE.md.
#
# ALWAYS THE PERSONAL PROFILE.  CLAUDE_CONFIG_DIR is set here, not inherited,
# so whichever shell or service started the GCS, this runs on
# ~/.claude-personal (the `claude-personal` alias in ~/.bashrc) and never on
# the default ~/.claude login or the work one.
#
# NEEDS INTERNET.  At a field with no uplink it starts and cannot connect.
#
# THE "Enable Remote Control? (y/n)" PROMPT is answered here, on every start.
# `claude remote-control` asks it even with /rc on by default (that setting is
# the interactive `claude`'s), and a "y" typed in a terminal was not remembered
# (2026-09-23: remoteEnabled stayed false).  From the dashboard stdin is
# /dev/null, so it sat at the prompt and never appeared in the app.  So stdin
# is a pipe that says "y" once and then stays open: at EOF it would stop
# reading its keys (space = QR code, w = spawn mode) or quit.
#
# Deliberately NOT exec: the GCS finds this row by the basename
# claude_remote.sh, never `claude`, which would match every Claude session on
# the box.  Stopping it signals this script's process group, which takes
# `claude remote-control` and the sessions it spawned with it.
#
set -eo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

export CLAUDE_CONFIG_DIR="$HOME/.claude-personal"
# Started from a shell that is itself inside Claude Code, these would make the
# child think it is nested.
unset CLAUDECODE CLAUDE_CODE_ENTRYPOINT

CLAUDE_BIN="${CLAUDE_BIN:-$(command -v claude || echo "$HOME/.local/bin/claude")}"
if [[ ! -x "$CLAUDE_BIN" ]]; then
    echo "claude not found (looked for $CLAUDE_BIN)" >&2
    exit 1
fi
if [[ ! -f "$CLAUDE_CONFIG_DIR/.credentials.json" ]]; then
    echo "the personal profile is not logged in: run claude-personal once in a terminal" >&2
    exit 1
fi

echo "[claude_remote] profile $CLAUDE_CONFIG_DIR, dir $REPO_DIR"
{ echo y; exec sleep infinity; } |
    "$CLAUDE_BIN" remote-control --name "${CLAUDE_RC_NAME:-$(hostname)}"
