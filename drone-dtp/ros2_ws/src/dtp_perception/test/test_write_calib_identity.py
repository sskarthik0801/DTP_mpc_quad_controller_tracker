"""Tests for the identity keys write_calib_yaml puts into the runtime calibration.

*** WHY THIS FILE EXISTS ***

write_calib_yaml accepted --camera_serial and --lidar_ip, documented both at length, printed a
WARNING when camera_serial was omitted -- and never wrote either into the YAML. calib_io
gates every node that loads that file on exactly those two keys (a wrong camera BODY and a
foreign LiDAR each
refuse to start), so regenerating the calibration silently downgraded both gates to log warnings.
The deployed file only had them because a human pasted them in by hand, which no regeneration would
ever reproduce.

The tests that matter are the ROUND-TRIPS: they write a file with write_calib_yaml and read it back
with calib_io.load_calib. The old bug was precisely that the writer and the reader disagreed about
whether the identity existed, and nothing short of a round-trip notices that.

NOTE ON THE no_autodetect FIXTURE: write_calib_yaml can autodetect a single attached Orbbec body.
That makes "nothing is known" environment-dependent -- these tests would pass on the
workstation and fail on the NX, where a camera IS plugged in. Every test that means
"nothing is known" says so
explicitly rather than depending on what happens to be on USB.
"""
import io

import numpy as np
import pytest

from dtp_perception.calib_io import load_calib
from dtp_perception.calibration import write_calib_yaml

T = np.array([[-0.015423638, -0.999685311, 0.019783567, 0.032656188],
              [-0.023166368, -0.019423327, -0.999542922, -0.039839680],
              [0.999612640, -0.015874902, -0.022859500, -0.076500461],
              [0.0, 0.0, 0.0, 1.0]])


@pytest.fixture
def no_autodetect(monkeypatch):
    """No camera attached, regardless of the machine running the tests."""
    monkeypatch.setattr(write_calib_yaml, '_autodetect_camera_serial', lambda: None)


@pytest.fixture
def rig(tmp_path):
    """A minimal but REAL capture run: the files write_calib_yaml actually reads."""
    run = tmp_path / 'run'
    run.mkdir()
    (run / 'camera_info.yaml').write_text(
        'resolution: [1280, 720]\n'
        'fx: 611.159790\nfy: 611.342468\ncx: 637.017029\ncy: 363.181091\n'
        'distortion_model: plumb_bob\n'
        'd: [-0.03124923, 0.03501169, 0.00003306]\n')
    for i in range(8):
        np.savez(str(run / ('pair_%03d.npz' % i)), pts=np.zeros((3, 3)))
    with io.open(str(run / 'T_camera_lidar.txt'), 'w') as fh:
        for row in T:
            fh.write(' '.join('%.9f' % v for v in row) + '\n')

    def invoke(extra, identity=None):
        if identity is not None:
            with io.open(str(run / 'identity.yaml'), 'w') as fh:
                for key, value in identity.items():
                    fh.write('%s: %s\n' % (key, value))
        out = tmp_path / 'out.yaml'
        rc = write_calib_yaml.main([
            '--run_dir', str(run), '--calib_file', str(run / 'T_camera_lidar.txt'),
            '--out', str(out), '--note', 'unit test'] + extra)
        return rc, out

    return invoke


def test_refuses_when_identity_unknown(rig, no_autodetect):
    """*** THE ORIGINAL BUG. *** It used to warn and exit 0, handing back a gate-less file."""
    rc, out = rig([])
    assert rc != 0
    assert not out.exists(), 'refusing must leave no half-written calibration behind'


def test_explicit_flags_reach_calib_io(rig):
    """The round-trip. Flags in, real YAML keys out, read back by the runtime gate's own loader."""
    rc, out = rig(['--camera_serial', 'CPC64630007M', '--lidar_ip', '192.168.1.154'])
    assert rc == 0
    T_read, intr, _note = load_calib(str(out))
    assert intr['camera_serial'] == 'CPC64630007M'
    assert intr['lidar_ip'] == '192.168.1.154'
    assert np.allclose(T_read, T, atol=1e-9), 'identity keys must not disturb the extrinsic'


def test_capture_identity_is_used_without_flags(rig, no_autodetect):
    """Identity recorded at capture time beats anything a human retypes at 1am."""
    rc, out = rig([], identity={'camera_serial': 'CAPTURED123', 'lidar_ip': '192.168.1.99'})
    assert rc == 0
    _, intr, _note = load_calib(str(out))
    assert intr['camera_serial'] == 'CAPTURED123'
    assert intr['lidar_ip'] == '192.168.1.99'
    assert 'recorded at capture' in io.open(str(out), encoding='utf-8').read()


def test_flag_overrides_capture_but_conflict_is_reported(rig, capsys):
    """A contradiction means the flag is wrong OR these pairs are from another rig. Say so."""
    rc, out = rig(['--camera_serial', 'FLAGWINS'],
                  identity={'camera_serial': 'CAPTURED123', 'lidar_ip': '192.168.1.99'})
    assert rc == 0
    _, intr, _note = load_calib(str(out))
    assert intr['camera_serial'] == 'FLAGWINS'
    assert 'CONFLICT' in capsys.readouterr().out


def test_waiver_is_recorded_in_the_file_itself(rig, no_autodetect):
    """Anyone opening this file months later must see a gate was waived, without shell history."""
    rc, out = rig(['--allow_unidentified', '--lidar_ip', '192.168.1.154'])
    assert rc == 0
    text = io.open(str(out), encoding='utf-8').read()
    assert 'IDENTITY GATES WAIVED' in text
    assert 'camera_serial -- a different camera BODY' in text
    # No bogus placeholder: an empty camera_serial key would read as a serial to load_calib.
    assert 'camera_serial:' not in text
    _, intr, _note = load_calib(str(out))
    assert not intr.get('camera_serial')
    assert intr['lidar_ip'] == '192.168.1.154', 'waiving one key must not drop the other'


def test_identity_file_placeholders_are_not_treated_as_values(rig, no_autodetect):
    """'unknown' in a capture identity file is an absence, not a serial."""
    rc, _out = rig([], identity={'camera_serial': 'unknown', 'lidar_ip': ''})
    assert rc != 0, 'placeholder values must not satisfy the gate'


def test_autodetect_fills_a_single_attached_body(rig, monkeypatch):
    """Run on the capture machine, the serial should not need typing."""
    monkeypatch.setattr(write_calib_yaml, 'attached_camera_serials', lambda: ['ONLYONE'])
    rc, out = rig(['--lidar_ip', '192.168.1.154'])
    assert rc == 0
    _, intr, _note = load_calib(str(out))
    assert intr['camera_serial'] == 'ONLYONE'
    assert 'autodetected' in io.open(str(out), encoding='utf-8').read()


def test_autodetect_refuses_to_guess_between_two_bodies(rig, monkeypatch):
    """The 2026-09-07 incident: two Orbbec bodies, the driver binding whichever enumerated first.

    There is no local source of truth for which one was bound -- calib_io documents that the
    driver's own serial_number parameter reports back whatever was requested -- so a coin flip
    written into a provenance field is worse than an honest refusal.
    """
    monkeypatch.setattr(write_calib_yaml, 'attached_camera_serials', lambda: ['ONE', 'TWO'])
    rc, _out = rig(['--lidar_ip', '192.168.1.154'])
    assert rc != 0


def test_placeholder_flags_are_refused_not_written(rig, no_autodetect):
    """*** A typo must not become a node-start failure on the aircraft. ***

    The placeholder filter applied only to identity.yaml, so a typed --camera_serial unknown was
    written verbatim. No attached body reports the serial 'unknown', so calib_io then REFUSES TO
    START, reporting a camera mismatch -- which reads like a hardware fault rather than a typo.
    """
    for bogus in ('unknown', 'none', 'N/A', 'TBD', '?', ''):
        rc, out = rig(['--camera_serial', bogus, '--lidar_ip', '192.168.1.154'])
        assert rc != 0, '--camera_serial %r must be treated as absent, not written' % bogus
        assert not out.exists()


def test_placeholder_rejection_is_reported(rig, no_autodetect, capsys):
    """Silently treating it as absent would leave the operator unsure which branch they hit."""
    rig(['--camera_serial', 'unknown', '--lidar_ip', '192.168.1.154'])
    out = capsys.readouterr().out
    assert 'placeholder' in out.lower()


def test_lidar_ip_must_be_a_dotted_quad(rig, no_autodetect):
    """A malformed IP fails later as a LIDAR MISMATCH, which reads like a wiring problem."""
    for bogus in ('192.168.1', '192.168.1.999', 'livox.local', '192.168.1.1.1'):
        rc, _out = rig(['--camera_serial', 'CPC64630007M', '--lidar_ip', bogus])
        assert rc != 0, '--lidar_ip %r must be rejected' % bogus


def test_valid_edge_ips_are_accepted(rig, no_autodetect):
    """The shape check must not reject legitimate addresses."""
    for good in ('192.168.1.154', '10.0.0.1', '255.255.255.255', '0.0.0.0'):
        rc, out = rig(['--camera_serial', 'CPC64630007M', '--lidar_ip', good])
        assert rc == 0, '--lidar_ip %r must be accepted' % good
        _T, intr, _n = load_calib(str(out))
        assert intr['lidar_ip'] == good


def test_livox_driver_node_matches_what_our_launches_create():
    """Pins the constant that silently disabled the foreign-LiDAR gate for a day.

    calib_io queries the driver BY NAME because FastDDS reports _NODE_NAME_UNKNOWN_ for
    cross-process endpoints, so this fallback is the normal path, not a rare one. When both
    launches were renamed to dtp_livox_driver on 2026-09-11 this constant was left naming the
    upstream default -- which is now exactly what the FOREIGN driver is called.
    """
    from dtp_perception import calib_io
    assert calib_io.LIVOX_DRIVER_NODE == '/dtp_livox_driver'
