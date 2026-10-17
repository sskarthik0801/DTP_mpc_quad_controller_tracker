#!/usr/bin/env python3
"""replay_perception_probe.py -- perception_node exactly as `ros2 run` starts it, plus a per-track record
of the range gate. REPLAY ONLY (replay_perception.sh PROBE=1); never on the aircraft.

    PROBE_OUT=/path/probe.jsonl python3 replay_perception_probe.py --ros-args ...

The range gate's verdicts (accept_range) are what Step 3 scores ("no rise in rejected range jumps
at 6-10 m"), and the node logs them only on a line THROTTLED to one per 5 s -- and not at all with
the hybrid off. The package is not edited: this wraps three methods on the imported classes and
then calls perception_node.main(), the same entry point `ros2 run dtp_perception perception_node`
calls. Each wrapper calls the original with the original arguments and returns its result
unchanged; the only added work is one JSON line per built target.

One line per _build_target call (one track, one processed frame):
  {"s": stamp, "id": track, "ar": [accepted, innovation, gate, r_meas, lidar_sigma|null] | null,
   "reset": r_meas | null,       # a camera<->LiDAR handover restarted the gate (reset_range)
   "held": bool,                 # the track's hold counter moved: unranged / bearing hold
   "src": camera|lidar|hold|none|null (null = hybrid off), "pub": bool (a target was built)}
"""
import json
import os
import sys

from dtp_perception import perception_node as pn

_OUT = open(os.environ.get('PROBE_OUT', 'probe.jsonl'), 'w', buffering=1)
_CUR = {}

_orig_accept = pn._TrackState.accept_range
_orig_reset = pn._TrackState.reset_range
_orig_build = pn.PerceptionNode._build_target


def _accept(self, r_meas, t_s, sigma_m=None):
    res = _orig_accept(self, r_meas, t_s, sigma_m)
    _CUR['ar'] = [bool(res[0]), float(res[1]), float(res[2]) if res[2] != float('inf') else None,
                  float(r_meas), None if sigma_m is None else float(sigma_m)]
    return res


def _reset(self, r_meas, t_s, sigma_m=None):
    res = _orig_reset(self, r_meas, t_s, sigma_m)
    _CUR['reset'] = float(r_meas)
    return res


def _build(self, det, pend, T, T_pipe, conf_by_id):
    _CUR.clear()
    tid = int(det['track_id'])
    st = self._tracks.get(tid)
    h0 = st.holds if st is not None else 0
    tgt = _orig_build(self, det, pend, T, T_pipe, conf_by_id)
    try:
        st = self._tracks.get(tid)
        ch = self._hyb_choice.get(tid) if getattr(self, '_lidar_hybrid', False) else None
        _OUT.write(json.dumps({'s': round(float(pend.stamp_s), 4), 'id': tid, 'ar': _CUR.get('ar'),
                               'reset': _CUR.get('reset'),
                               'held': bool(st is not None and st.holds > h0),
                               'src': None if ch is None else ch.source,
                               'pub': tgt is not None}) + '\n')
    except Exception as exc:                                            # noqa: BLE001
        sys.stderr.write('probe: %s\n' % exc)
    return tgt


pn._TrackState.accept_range = _accept
pn._TrackState.reset_range = _reset
pn.PerceptionNode._build_target = _build

# ---- SCRATCH ONLY (2026-09-30 bench-1455 analysis): the PROPOSED ground-height guard, patched in
# at runtime. A strip-ground bin higher than GUARD_DROP_M below the camera is not ground (a ceiling,
# a table top): it is dropped, so that bin rejects nothing. Not in the package.
import numpy as _np
from dtp_perception import lidar_box_range as _lbr
GUARD_DROP_M = float(os.environ.get('GUARD_DROP_M', '0.30'))
_orig_strip = _lbr.strip_ground
_orig_rfb = _lbr.LidarBoxRange.range_for_box
_GZ = {}


def _strip(*a, **k):
    p = _orig_strip(*a, **k)
    cz = _GZ.get('cz')
    if cz is None:
        return p
    g = p.g.copy()
    g[g > cz - GUARD_DROP_M] = _np.nan
    return _lbr.GroundProfile(p.s0, p.bin_m, g)


def _rfb(self, box_px, stamp_s, forecast_range_m=float('nan'), track_vel_world=None):
    f = self._frame(float(stamp_s))
    _GZ['cz'] = None if f.c_w is None else float(f.c_w[2])
    return _orig_rfb(self, box_px, stamp_s, forecast_range_m, track_vel_world)


_lbr.strip_ground = _strip
_lbr.LidarBoxRange.range_for_box = _rfb


if __name__ == '__main__':
    pn.main()
