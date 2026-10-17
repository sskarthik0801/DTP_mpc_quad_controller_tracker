#!/usr/bin/env bash
# Kept for docs/geminitest.md and muscle memory: the camera launcher is now
# scripts/gemini_camera.sh, because the web GCS starts the same process and
# "test" was the wrong name for something the dashboard runs. One
# implementation, two names -- never two copies to drift apart.
exec bash "$(dirname "${BASH_SOURCE[0]}")/gemini_camera.sh" "$@"
