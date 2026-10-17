"""Tests for dtp_bringup.preflight.running_pids / duplicate_refusal.

*** WHY THIS FILE EXISTS ***

On 2026-09-12 dtp_system.launch.py was started on top of an already-running M1 stack and brought
up a second Livox driver. It took the MID360's stream, FAST-LIO got IMU and no scans for 225 s,
and the aircraft was armed on a pose 3.74 km wrong (docs/fact-fastlio-divergence-2026-09-12.md).
The launch files now refuse to start a second copy. These tests pin the two properties that
refusal depends on: a real process IS found, and a shell that merely MENTIONS the name is NOT --
the `pkill -f` trap, which would otherwise make every launch refuse because of its own terminal.
"""
import os
import subprocess
import time

from dtp_bringup import preflight


def _spawn(argv):
    proc = subprocess.Popen(argv, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.2)
    return proc


def test_finds_a_process_by_argv_basename(tmp_path):
    probe = tmp_path / 'dtp_dup_probe_real'
    os.symlink('/bin/sleep', probe)
    proc = _spawn([str(probe), '30'])
    try:
        assert proc.pid in preflight.running_pids('dtp_dup_probe_real')
        msg = preflight.duplicate_refusal('dtp_dup_probe_real', 'the probe')
        assert msg is not None and str(proc.pid) in msg and 'NOT starting the probe' in msg
    finally:
        proc.kill()
        proc.wait()


def test_a_shell_that_only_mentions_the_name_does_not_count():
    proc = _spawn(['bash', '-c', 'sleep 30; echo dtp_dup_probe_mentioned'])
    try:
        assert preflight.running_pids('dtp_dup_probe_mentioned') == []
        assert preflight.duplicate_refusal('dtp_dup_probe_mentioned', 'x') is None
    finally:
        proc.kill()
        proc.wait()


def test_a_launcher_carrying_the_name_as_an_argument_does_not_count(tmp_path):
    # The `ros2 run <pkg> <node>` shape: argv[0] is an interpreter, the node name an argument.
    proc = _spawn(['bash', '-c', 'sleep 30', 'dtp_dup_probe_as_arg'])
    try:
        assert preflight.running_pids('dtp_dup_probe_as_arg') == []
    finally:
        proc.kill()
        proc.wait()


def test_nothing_running_means_no_refusal():
    assert preflight.duplicate_refusal('dtp_dup_probe_absent_0f3a', 'x') is None
