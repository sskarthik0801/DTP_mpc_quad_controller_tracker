from __future__ import annotations

import time
from collections import defaultdict, deque
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
from ultralytics import YOLO

try:
    import torch
except ImportError:  
    torch = None

from detection_pipeline.eigen import EigenTrajectoryAdapter


class HumanXZPredictor:
    EIGEN_TRAJECTORY_FUSION_ALPHA = 0.2
    def __init__(
        self,
        model_path="models/yolo26n.engine",
        conf=0.25,
        imgsz=384,
        use_tracking=True,
        detect_every_n=2,
        draw=True,
        profile=True,
        smooth_alpha=0.25,
        history_size=60,
        min_history_for_prediction=3,
        prediction_steps=20,
        prediction_secs=1.5,
        target_class=0,
        select_largest_only=False,
        max_depth_m=6.0,
        max_missed_frames=5,
        max_missed_secs=None,
        min_confirm_frames=3,
        predict_every_n=2,
        prediction_depth_gate_m=6.0,
        max_step_m=0.35,
        half_patch_scale_w=0.08,
        half_patch_scale_h=0.08,
        front_stop_distance_m=1.8,
        front_slow_distance_m=3.8,
        side_stop_distance_m=0.8,
        side_slow_distance_m=2.0,
        front_lateral_half_width_m=0.7,
        immediate_stop_distance_m=0.8,
        agv_hold_max_frames=15,
        agv_stop_hold_max_frames=None,
        agv_hold_secs=None,
        agv_stop_hold_secs=None,
        eigen_enabled=False,
        eigen_hyper_params=None,
        eigen_checkpoint_path=None,
        eigen_device="cpu",
        eigen_obs_len=8,
        eigen_pred_len=12,
        eigen_infer_rate_hz=10.0,
        eigen_fusion_alpha=None,
    ):
        self.model = YOLO(model_path)
        self.conf = conf
        self.imgsz = imgsz
        self.use_tracking = use_tracking
        self.detect_every_n = max(1, int(detect_every_n))
        self.draw = draw
        self.profile = profile

        self.smooth_alpha = smooth_alpha
        self.one_minus_alpha = 1.0 - smooth_alpha

        self.history_size = history_size
        self.min_history_for_prediction = min_history_for_prediction
        self.prediction_steps = prediction_steps
        self.prediction_secs = prediction_secs
        self.target_class = target_class
        self.select_largest_only = select_largest_only
        self.max_depth_m = max_depth_m

        self.max_missed_frames = max_missed_frames
        # Time-based ghost-track window (rate-independent). None -> frame-count, the byte-identical
        # default: a track goes stale after max_missed_frames MISSED frames, whose wall-clock
        # duration scales with frame rate (~0.17 s @ 30 Hz AMR, ~10 s @ 0.5 Hz Spot -- the CTRV
        # model keeps predicting a vanished person into the corridor for that long). Set
        # max_missed_secs to drop a track a fixed time after it was last SEEN, regardless of rate.
        self._time_missed = max_missed_secs is not None
        self.max_missed_secs = None if max_missed_secs is None else float(max_missed_secs)
        self.min_confirm_frames = min_confirm_frames
        self.predict_every_n = max(1, int(predict_every_n))
        self.prediction_depth_gate_m = prediction_depth_gate_m
        self.max_step_m = max_step_m
        self.max_step_m_sq = max_step_m * max_step_m

        self.half_patch_scale_w = half_patch_scale_w
        self.half_patch_scale_h = half_patch_scale_h

        self.front_stop_distance_m = float(front_stop_distance_m)
        self.front_slow_distance_m = float(front_slow_distance_m)
        self.side_stop_distance_m = float(side_stop_distance_m)
        self.side_slow_distance_m = float(side_slow_distance_m)
        self.front_lateral_half_width_m = float(front_lateral_half_width_m)
        self.immediate_stop_distance_m = float(immediate_stop_distance_m)


        self.eigen_adapter = EigenTrajectoryAdapter()
        self.eigen_enabled = bool(eigen_enabled)
        self.eigen_fusion_alpha = (
            self.EIGEN_TRAJECTORY_FUSION_ALPHA
            if eigen_fusion_alpha is None
            else float(np.clip(eigen_fusion_alpha, 0.0, 1.0))
        )
        self.eigen_obs_len = int(eigen_obs_len)
        self.eigen_pred_len = int(eigen_pred_len)

        if self.eigen_enabled:
            self.setup_eigen_trajectory(
                hyper_params=eigen_hyper_params,
                checkpoint_path=eigen_checkpoint_path,
                device=eigen_device,
                obs_len=eigen_obs_len,
                pred_len=eigen_pred_len,
                infer_rate_hz=eigen_infer_rate_hz,
            )

        self.tracks_xz = defaultdict(lambda: deque(maxlen=self.history_size))
        self.tracks_2d = defaultdict(lambda: deque(maxlen=self.history_size))

        self.track_missed_counts = defaultdict(int)
        self.track_last_seen_ts = {}
        self.track_confirmed = defaultdict(bool)
        self.track_seen_counts = defaultdict(int)

        self.frame_idx = 0
        self.last_infer_results = None
        self.last_parsed_detections = []

        self.last_predictions = {}

        self.last_agv_cmd = "normal_operation"
        self.last_agv_state_str = "no_human_or_clear"
        self.agv_hold_counter = 0
        self.agv_hold_max_frames = int(agv_hold_max_frames)
        self.agv_stop_hold_max_frames = int(
            agv_hold_max_frames if agv_stop_hold_max_frames is None else agv_stop_hold_max_frames)

        self._time_holds = agv_hold_secs is not None
        self.agv_hold_secs = None if agv_hold_secs is None else float(agv_hold_secs)
        self.agv_stop_hold_secs = float(
            agv_hold_secs if agv_stop_hold_secs is None else agv_stop_hold_secs) \
            if agv_hold_secs is not None else None
        self._hold_release_ts = float('-inf')   # time mode: hold current cmd until this timestamp

    @staticmethod
    def _get_center(xyxy):
        x1, y1, x2, y2 = xyxy
        return int((x1 + x2) * 0.5), int((y1 + y2) * 0.5)

    @staticmethod
    def _robust_trimmed_mean(values, trim_ratio=0.15):
        values = np.asarray(values, dtype=np.float64)
        values = values[np.isfinite(values)]
        if values.size == 0:
            return None
        values.sort()
        k = int(values.size * trim_ratio)
        if k > 0 and values.size > 2 * k:
            values = values[k:-k]
        return float(np.mean(values))

    @staticmethod
    def _ema_1d(values, alpha=0.35):
        values = np.asarray(values, dtype=np.float64)
        out = np.empty_like(values)
        out[0] = values[0]
        for i in range(1, len(values)):
            out[i] = alpha * values[i] + (1.0 - alpha) * out[i - 1]
        return out

    def setup_eigen_trajectory(
        self,
        hyper_params,
        checkpoint_path=None,
        device="cpu",
        obs_len=8,
        pred_len=12,
        infer_rate_hz=10.0,
    ):
        self.eigen_adapter.setup(
            hyper_params=hyper_params,
            checkpoint_path=checkpoint_path,
            device=device,
            obs_len=obs_len,
            pred_len=pred_len,
            infer_rate_hz=infer_rate_hz,
        )
        self.eigen_enabled = True
        self.eigen_obs_len = int(obs_len)
        self.eigen_pred_len = int(pred_len)

    @staticmethod
    def _timestamp_to_ns(timestamp):
        if timestamp is None:
            return time.time_ns()
        ts = float(timestamp)
        if ts > 1e12:
            return int(ts)
        return int(ts * 1e9)

    def _eigen_prediction_to_tuples(self, pred_xz, timestamp):
        if pred_xz is None:
            return []

        arr = np.asarray(pred_xz, dtype=np.float64)
        if arr.ndim != 2 or arr.shape[1] < 2:
            return []

        arr = arr[:, :2]
        finite_mask = np.isfinite(arr).all(axis=1)
        arr = arr[finite_mask]
        if arr.size == 0:
            return []

        n = min(len(arr), self.prediction_steps)
        step_dt = self.prediction_secs / max(n, 1)
        t0 = float(timestamp)
        return [
            (float(arr[i, 0]), float(arr[i, 1]), float(t0 + (i + 1) * step_dt))
            for i in range(n)
        ]

    def _predict_future_xz_eigen(self, track_id, timestamp):
        if not self.eigen_enabled or not self.eigen_adapter.enabled:
            return []

        preds_by_id = self.eigen_adapter.maybe_predict(self._timestamp_to_ns(timestamp))
        pred_xz = preds_by_id.get(int(track_id))
        return self._eigen_prediction_to_tuples(pred_xz, timestamp)

    def _fuse_future_predictions(self, classic_predictions, eigen_predictions):
        if not classic_predictions:
            return eigen_predictions or []
        if not eigen_predictions:
            return classic_predictions or []

        n = min(len(classic_predictions), len(eigen_predictions), self.prediction_steps)
        if n <= 0:
            return classic_predictions or eigen_predictions or []

        alpha = self.eigen_fusion_alpha
        beta = 1.0 - alpha
        fused = []

        for i in range(n):
            cx, cz, ct = classic_predictions[i]
            ex, ez, _ = eigen_predictions[i]
            if not np.isfinite([cx, cz, ex, ez]).all():
                continue
            fused.append((
                float(beta * cx + alpha * ex),
                float(beta * cz + alpha * ez),
                float(ct),
            ))

        return fused if fused else classic_predictions

    def _predict_future_xz(self, history):
        if len(history) < self.min_history_for_prediction:
            return []

        data = np.asarray(history, dtype=np.float64)
        finite_mask = np.isfinite(data).all(axis=1)
        data = data[finite_mask]
        if len(data) < self.min_history_for_prediction:
            return []

        data = data[-25:]
        x = data[:, 0]
        z = data[:, 1]
        t = data[:, 2]

        keep = np.ones(len(t), dtype=bool)
        keep[1:] = np.diff(t) > 1e-4
        x = x[keep]
        z = z[keep]
        t = t[keep]

        if len(t) < self.min_history_for_prediction:
            return []

        x_s = self._ema_1d(x, alpha=0.30)
        z_s = self._ema_1d(z, alpha=0.30)

        dx = np.diff(x_s)
        dz = np.diff(z_s)
        dt = np.diff(t)

        valid = dt > 1e-4
        dx = dx[valid]
        dz = dz[valid]
        dt = dt[valid]

        if len(dt) < 3:
            return []

        speed_inst = np.sqrt(dx * dx + dz * dz) / dt
        v_est = self._robust_trimmed_mean(speed_inst[-min(6, len(speed_inst)):], trim_ratio=0.10)
        if v_est is None or not np.isfinite(v_est):
            return []

        v_est = float(np.clip(v_est, 0.0, 3.0))

        baseline = min(6, len(x_s) - 1)
        if baseline < 1:
            return []

        headings = []
        heading_times = []

        for i in range(baseline, len(x_s)):
            dx_b = x_s[i] - x_s[i - baseline]
            dz_b = z_s[i] - z_s[i - baseline]
            dt_b = t[i] - t[i - baseline]
            if dt_b <= 1e-4:
                continue

            dist_b = np.hypot(dx_b, dz_b)
            if dist_b < 0.08:
                continue

            headings.append(np.arctan2(dz_b, dx_b))
            heading_times.append(t[i])

        if len(headings) < 2:
            return []

        headings = np.unwrap(np.asarray(headings, dtype=np.float64))
        heading_times = np.asarray(heading_times, dtype=np.float64)

        yaw_est = float(np.mean(headings[-min(4, len(headings)):]))

        if len(headings) >= 3:
            dtheta = np.diff(headings)
            dt_h = np.diff(heading_times)
            valid_h = dt_h > 1e-4
            yaw_rates = dtheta[valid_h] / dt_h[valid_h] if np.any(valid_h) else np.empty(0, dtype=np.float64)
        else:
            yaw_rates = np.empty(0, dtype=np.float64)

        if yaw_rates.size > 2:
            recent_yr = yaw_rates[-min(6, len(yaw_rates)):]
            mean_yr = np.mean(recent_yr)
            std_yr = np.std(recent_yr)

            if std_yr > abs(mean_yr) * 1.8 or abs(mean_yr) < 0.15:
                yaw_rate_est = 0.0
            else:
                yaw_rate_est = float(mean_yr)
        else:
            yaw_rate_est = 0.0

        yaw_rate_est = float(np.clip(yaw_rate_est, -1.2, 1.2))

        if v_est < 0.25:
            yaw_rate_est = 0.0

        x_pred = float(x_s[-1])
        z_pred = float(z_s[-1])
        t0 = float(t[-1])

        step_dt = self.prediction_secs / self.prediction_steps
        preds = []

        yaw_pred = yaw_est
        yaw_rate_pred = yaw_rate_est

        for i in range(self.prediction_steps):
            yaw_rate_pred *= 0.90

            if abs(yaw_rate_pred) < 1e-3:
                x_pred += v_est * np.cos(yaw_pred) * step_dt
                z_pred += v_est * np.sin(yaw_pred) * step_dt
            else:
                yaw_next = yaw_pred + yaw_rate_pred * step_dt
                x_pred += (v_est / yaw_rate_pred) * (np.sin(yaw_next) - np.sin(yaw_pred))
                z_pred += (v_est / yaw_rate_pred) * (-np.cos(yaw_next) + np.cos(yaw_pred))
                yaw_pred = yaw_next

            preds.append((float(x_pred), float(z_pred), float(t0 + (i + 1) * step_dt)))

        return preds

    def _limit_motion_step(self, track_id, x_new, z_new):
        track = self.tracks_xz[track_id]
        if not track:
            return x_new, z_new

        prev_x, prev_z, _ = track[-1]
        dx = x_new - prev_x
        dz = z_new - prev_z
        dist_sq = dx * dx + dz * dz

        if dist_sq <= self.max_step_m_sq or dist_sq < 1e-12:
            return x_new, z_new

        dist = np.sqrt(dist_sq)
        scale = self.max_step_m / dist
        return prev_x + dx * scale, prev_z + dz * scale

    def _reject_far_points(self, x_new, z_new):
        if not np.isfinite(x_new) or not np.isfinite(z_new):
            return None
        if z_new <= 0.0:
            return None
        if self.max_depth_m is not None and z_new > self.max_depth_m:
            return None
        return x_new, z_new

    def _smooth_xz(self, track_id, x_raw, z_raw):
        track = self.tracks_xz[track_id]
        if not track:
            return x_raw, z_raw

        prev_x, prev_z, _ = track[-1]
        a = self.smooth_alpha
        b = self.one_minus_alpha
        return a * x_raw + b * prev_x, a * z_raw + b * prev_z

    def _extract_xz_from_pointcloud(self, point_cloud_xyz, cx, cy, box=None):
        if point_cloud_xyz is None:
            return None

        h, w = point_cloud_xyz.shape[:2]

        if box is not None:
            x1, y1, x2, y2 = map(int, box)
            bw = max(1, x2 - x1)
            bh = max(1, y2 - y1)

            cx = int(x1 + 0.50 * bw)
            cy = int(y1 + 0.65 * bh)

            half_w = max(1, int(self.half_patch_scale_w * bw))
            half_h = max(1, int(self.half_patch_scale_h * bh))
        else:
            half_w = 2
            half_h = 2

        cx = max(0, min(cx, w - 1))
        cy = max(0, min(cy, h - 1))

        px1 = max(0, cx - half_w)
        px2 = min(w, cx + half_w + 1)
        py1 = max(0, cy - half_h)
        py2 = min(h, cy + half_h + 1)

        patch = point_cloud_xyz[py1:py2, px1:px2, :3]
        if patch.size == 0:
            return None

        patch = patch.reshape(-1, 3)
        x_vals = patch[:, 0]
        z_vals = patch[:, 2]

        valid_mask = np.isfinite(x_vals) & np.isfinite(z_vals) & (z_vals > 0.0)
        if self.max_depth_m is not None:
            valid_mask &= (z_vals <= self.max_depth_m)

        if not np.any(valid_mask):
            return None

        x_vals = x_vals[valid_mask]
        z_vals = z_vals[valid_mask]

        return float(np.median(x_vals)), float(np.median(z_vals))

    def _camera_xz_to_odom_xz(self, x_cam, z_cam, T_odom_camera):
        if T_odom_camera is None:
            return x_cam, z_cam

        T = np.asarray(T_odom_camera, dtype=np.float64)
        if T.shape != (4, 4) or not np.isfinite(T).all():
            return None

        x_odom = T[0, 0] * x_cam + T[0, 2] * z_cam + T[0, 3]
        z_odom = T[2, 0] * x_cam + T[2, 2] * z_cam + T[2, 3]

        if not np.isfinite(x_odom) or not np.isfinite(z_odom):
            return None

        return float(x_odom), float(z_odom)

    def _classify_agv_region(self, x_obj, z_obj, agv_x, agv_z, agv_yaw=0.0):
        """Region of (x_obj, z_obj) relative to the AGV at (agv_x, agv_z) heading `agv_yaw`.

        `agv_yaw` is the AGV heading in the SAME frame the positions are given in, measured so
        that its forward axis is (sin yaw, cos yaw) in (x, z) -- yaw=0 means "facing +Z", which
        is what the camera frame itself always is.

        The rotation matters: `_camera_xz_to_odom_xz` moves the person into odom using the yaw
        block of T, so once positions are in odom a plain difference is an ODOM-frame offset, not
        a robot-frame one. Without rotating it back the "front" corridor stays pinned to odom +Z
        and stops following the robot the moment it turns -- a person genuinely ahead is then
        classified "side" and silently gets the much smaller side thresholds. yaw=0 reproduces
        the robot-relative case (T=None) bit-for-bit, so the camera-frame path is untouched.
        """
        dx = float(x_obj - agv_x)
        dz = float(z_obj - agv_z)

        if agv_yaw:
            c, s = np.cos(agv_yaw), np.sin(agv_yaw)
            rel_x = float(dx * c - dz * s)
            rel_z = float(dx * s + dz * c)
        else:
            rel_x, rel_z = dx, dz

        if rel_z >= 0.0 and abs(rel_x) <= self.front_lateral_half_width_m:
            return "front", rel_x, rel_z

        return "side", rel_x, rel_z

    def _thresholds_for_region(self, region):
        if region == "front":
            return self.front_stop_distance_m, self.front_slow_distance_m
        return self.side_stop_distance_m, self.side_slow_distance_m

    @staticmethod
    def _cmd_priority(cmd):
        if cmd == "stop":
            return 2
        if cmd == "slow_down":
            return 1
        return 0

    def _command_from_distance(self, dist_m, stop_dist_m, slow_dist_m):
        if dist_m < stop_dist_m:
            return "stop"
        if dist_m <= slow_dist_m:
            return "slow_down"
        return "normal_operation"

    def _apply_command_hold(self, raw_cmd, raw_state_str, timestamp=None):
        if self._time_holds and timestamp is not None:
            if raw_cmd == "stop":
                self.last_agv_cmd = "stop"
                self.last_agv_state_str = raw_state_str
                self._hold_release_ts = timestamp + self.agv_stop_hold_secs
            elif raw_cmd == "slow_down":
                if self.last_agv_cmd != "stop" or timestamp >= self._hold_release_ts:
                    self.last_agv_cmd = "slow_down"
                    self.last_agv_state_str = raw_state_str
                    self._hold_release_ts = timestamp + self.agv_hold_secs
                # else: keep holding the stop until its release time
            else:  # normal_operation
                if timestamp >= self._hold_release_ts:
                    self.last_agv_cmd = "normal_operation"
                    self.last_agv_state_str = raw_state_str
                # else: keep holding
            return

        if raw_cmd == "stop":
            self.last_agv_cmd = "stop"
            self.last_agv_state_str = raw_state_str
            self.agv_hold_counter = self.agv_stop_hold_max_frames
        elif raw_cmd == "slow_down":
            if self.last_agv_cmd != "stop" or self.agv_hold_counter == 0:
                self.last_agv_cmd = "slow_down"
                self.last_agv_state_str = raw_state_str
                self.agv_hold_counter = self.agv_hold_max_frames
            else:
                self.agv_hold_counter -= 1
        else:  # normal_operation
            if self.agv_hold_counter > 0:
                self.agv_hold_counter -= 1
            else:
                self.last_agv_cmd = "normal_operation"
                self.last_agv_state_str = raw_state_str

    def _mark_all_tracks_missed(self):
        for track_id in list(self.track_seen_counts.keys()):
            self.track_missed_counts[track_id] += 1

    def _mark_track_seen(self, track_id, timestamp):
        self.track_missed_counts[track_id] = 0
        self.track_last_seen_ts[track_id] = timestamp
        self.track_seen_counts[track_id] += 1
        if self.track_seen_counts[track_id] >= self.min_confirm_frames:
            self.track_confirmed[track_id] = True

    def _reset_track(self, track_id):
        self.tracks_xz.pop(track_id, None)
        self.tracks_2d.pop(track_id, None)
        self.track_missed_counts.pop(track_id, None)
        self.track_last_seen_ts.pop(track_id, None)
        self.track_confirmed.pop(track_id, None)
        self.track_seen_counts.pop(track_id, None)
        self.last_predictions.pop(track_id, None)
        self.eigen_adapter.remove_track(track_id)

    def _clear_stale_tracks(self, timestamp=None):
        if self._time_missed and timestamp is not None:
            # Drop a track a fixed wall-clock time after it was last SEEN (rate-independent).
            cutoff = timestamp - self.max_missed_secs
            stale_ids = [tid for tid in list(self.track_missed_counts)
                         if self.track_last_seen_ts.get(tid, cutoff) < cutoff]
        else:
            stale_ids = [tid for tid, missed in self.track_missed_counts.items() if missed > self.max_missed_frames]
        for track_id in stale_ids:
            self._reset_track(track_id)

    def _should_run_inference(self):
        return (self.frame_idx % self.detect_every_n) == 0 or self.last_infer_results is None

    def _run_model(self, frame):
        if self.use_tracking:
            return self.model.track(
                frame,
                persist=True,
                classes=[self.target_class],
                conf=self.conf,
                imgsz=self.imgsz,
                verbose=False,
            )
        return self.model.predict(
            frame,
            classes=[self.target_class],
            conf=self.conf,
            imgsz=self.imgsz,
            verbose=False,
        )

    def _parse_yolo_results(self, yolo_results):
        if not yolo_results:
            return []

        r0 = yolo_results[0]
        boxes_obj = r0.boxes
        if boxes_obj is None or len(boxes_obj) == 0:
            return []

        boxes = boxes_obj.xyxy
        ids_tensor = boxes_obj.id if self.use_tracking else None

        if ids_tensor is not None:
            ids = ids_tensor.int().cpu().tolist()
        else:
            ids = None

        boxes_np = boxes.cpu().numpy().astype(np.float32, copy=False)

        if self.select_largest_only and len(boxes_np) > 0:
            wh = boxes_np[:, 2:4] - boxes_np[:, 0:2]
            areas = wh[:, 0] * wh[:, 1]
            best_idx = int(np.argmax(areas))
            if ids is not None:
                return [{"track_id": int(ids[best_idx]), "box": boxes_np[best_idx]}]
            return [{"track_id": best_idx, "box": boxes_np[best_idx]}]

        if ids is not None:
            return [{"track_id": int(ids[i]), "box": boxes_np[i]} for i in range(len(boxes_np))]
        return [{"track_id": i, "box": boxes_np[i]} for i in range(len(boxes_np))]

    def process_frame(self, color_frame, point_cloud_xyz, timestamp=None, T_odom_camera=None):
        if color_frame is None or point_cloud_xyz is None:
            return None, {
                "success": False,
                "reason": "color_frame or point_cloud_xyz is None",
                "detections": [],
            }

        if timestamp is None:
            timestamp = time.time()

        self.frame_idx += 1
        t0 = time.perf_counter()

        frame_out = color_frame.copy() if self.draw else color_frame
        self._mark_all_tracks_missed()

        t_infer_start = time.perf_counter()
        if self._should_run_inference():
            self.last_infer_results = self._run_model(color_frame)
            self.last_parsed_detections = self._parse_yolo_results(self.last_infer_results)
            inference_ran = True
        else:
            inference_ran = False
        t_infer_end = time.perf_counter()

        detections = self.last_parsed_detections

        output = {
            "success": True,
            "timestamp": timestamp,
            "inference_ran": inference_ran,
            "detections": [],
            "agv_movement": {
                "command": "normal_operation",
                "closest_human_depth_m": None,
                "range_state": "no_human_or_clear",
            },
        }

        if not detections:
            self._clear_stale_tracks(timestamp)
            self._apply_command_hold("normal_operation", "no_human_or_clear", timestamp)
            output["agv_movement"] = {
                "command": self.last_agv_cmd,
                "closest_human_depth_m": None,
                "range_state": self.last_agv_state_str,
            }
            if self.profile:
                output["timings_ms"] = {
                    "total": round((time.perf_counter() - t0) * 1000.0, 2),
                    "inference": round((t_infer_end - t_infer_start) * 1000.0, 2),
                    "post": 0.0,
                }
            return frame_out, output

        cam_x_odom = 0.0
        cam_z_odom = 0.0
        cam_yaw_odom = 0.0
        if T_odom_camera is not None:
            T = np.asarray(T_odom_camera, dtype=np.float64)
            if T.shape == (4, 4) and np.isfinite(T).all():
                cam_x_odom = float(T[0, 3])
                cam_z_odom = float(T[2, 3])
                # Camera forward (0,0,1) in odom is column 2; its XZ projection gives the heading
                # the front corridor must follow. Needed to undo _camera_xz_to_odom_xz when
                # classifying regions -- see _classify_agv_region.
                cam_yaw_odom = float(np.arctan2(T[0, 2], T[2, 2]))
            else:
                T = None
        else:
            T = None

        closest_human_depth = None

        raw_cmd = "normal_operation"
        raw_state_str = "prediction_invalid_or_clear"
        best_cmd_priority = self._cmd_priority(raw_cmd)
        best_eval_dist = None
        best_region = None

        t_post_start = time.perf_counter()

        tracks_xz = self.tracks_xz
        tracks_2d = self.tracks_2d
        confirmed = self.track_confirmed

        for det in detections:
            box = det["box"]
            track_id = int(det["track_id"])

            self._mark_track_seen(track_id, timestamp)

            cx, cy = self._get_center(box)
            tracks_2d[track_id].append((cx, cy))

            xz = self._extract_xz_from_pointcloud(point_cloud_xyz, cx, cy, box=box)
            if xz is None:
                if self.draw:
                    x1, y1, x2, y2 = map(int, box)
                    cv2.rectangle(frame_out, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        frame_out,
                        f"ID:{track_id} pc:invalid",
                        (x1, max(y1 - 10, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 165, 255),
                        2,
                    )
                continue

            x_raw, z_raw = xz

            xz_filtered = self._reject_far_points(x_raw, z_raw)
            if xz_filtered is None:
                continue

            x_cam, z_cam = xz_filtered
            xz_odom = self._camera_xz_to_odom_xz(x_cam, z_cam, T)
            if xz_odom is None:
                continue

            x_raw, z_raw = xz_odom

            if confirmed[track_id]:
                x_raw, z_raw = self._limit_motion_step(track_id, x_raw, z_raw)

            x_smooth, z_smooth = self._smooth_xz(track_id, x_raw, z_raw)
            tmp_depth = float(np.hypot(x_smooth - cam_x_odom, z_smooth - cam_z_odom))

            if closest_human_depth is None or tmp_depth < closest_human_depth:
                closest_human_depth = tmp_depth

            tracks_xz[track_id].append((x_smooth, z_smooth, timestamp))

            if self.eigen_enabled:
                self.eigen_adapter.update_history(
                    track_id, np.asarray([x_smooth, z_smooth], dtype=np.float32)
                )

            current_region, rel_x, rel_z = self._classify_agv_region(
                x_smooth, z_smooth, cam_x_odom, cam_z_odom, cam_yaw_odom
            )
            current_stop_dist, current_slow_dist = self._thresholds_for_region(current_region)

            do_predict = (
                confirmed[track_id]
                and tmp_depth <= self.prediction_depth_gate_m
                and (self.frame_idx % self.predict_every_n == 0)
            )

            if do_predict:
                classic_predictions = self._predict_future_xz(tracks_xz[track_id])
                eigen_predictions = self._predict_future_xz_eigen(track_id, timestamp)
                predictions = self._fuse_future_predictions(classic_predictions, eigen_predictions)
                self.last_predictions[track_id] = predictions
            else:
                predictions = self.last_predictions.get(track_id, [])

            representative_pred_dist = None
            representative_pred_region = None

            if predictions:
                pred_tail = predictions[-min(10, len(predictions)):]
                pred_to_human_dists = [
                    float(np.hypot(px - x_smooth, pz - z_smooth))
                    for px, pz, _ in pred_tail
                ]

                valid_tail = [
                    (px, pz, pt)
                    for (px, pz, pt), d in zip(pred_tail, pred_to_human_dists)
                    if d <= 4.0
                ]

                if valid_tail:
                    rep_px, rep_pz, _ = min(
                        valid_tail,
                        key=lambda p: np.hypot(p[0] - cam_x_odom, p[1] - cam_z_odom)
                    )
                    representative_pred_dist = float(np.hypot(rep_px - cam_x_odom, rep_pz - cam_z_odom))
                    representative_pred_region, _, _ = self._classify_agv_region(
                        rep_px, rep_pz, cam_x_odom, cam_z_odom, cam_yaw_odom
                    )

        
            eval_cmd = "normal_operation"
            eval_state = "clear"
            eval_dist = None
            eval_region = None

            if tmp_depth < self.immediate_stop_distance_m:
                eval_cmd = "stop"
                eval_state = f"critical_range_current_human_{current_region}"
                eval_dist = tmp_depth
                eval_region = current_region
            else:
                current_cmd = self._command_from_distance(
                    tmp_depth, current_stop_dist, current_slow_dist
                )
                if current_cmd != "normal_operation":
                    eval_cmd = current_cmd
                    eval_state = f"{current_region}_range_current_human"
                    eval_dist = tmp_depth
                    eval_region = current_region

                
                if representative_pred_dist is not None:
                    pred_stop_dist, pred_slow_dist = self._thresholds_for_region(representative_pred_region)
                    pred_cmd = self._command_from_distance(
                        representative_pred_dist, pred_stop_dist, pred_slow_dist
                    )
                    pred_state = f"{representative_pred_region}_range_predicted"

                    if self._cmd_priority(pred_cmd) > self._cmd_priority(eval_cmd):
                        eval_cmd = pred_cmd
                        eval_state = pred_state
                        eval_dist = representative_pred_dist
                        eval_region = representative_pred_region
                    elif (
                        self._cmd_priority(pred_cmd) == self._cmd_priority(eval_cmd)
                        and pred_cmd != "normal_operation"
                    ):
                        if eval_dist is None or representative_pred_dist < eval_dist:
                            eval_cmd = pred_cmd
                            eval_state = pred_state
                            eval_dist = representative_pred_dist
                            eval_region = representative_pred_region

            if self._cmd_priority(eval_cmd) > best_cmd_priority:
                raw_cmd = eval_cmd
                raw_state_str = eval_state
                best_cmd_priority = self._cmd_priority(eval_cmd)
                best_eval_dist = eval_dist
                best_region = eval_region
            elif self._cmd_priority(eval_cmd) == best_cmd_priority and eval_cmd != "normal_operation":
                if best_eval_dist is None or (eval_dist is not None and eval_dist < best_eval_dist):
                    raw_cmd = eval_cmd
                    raw_state_str = eval_state
                    best_eval_dist = eval_dist
                    best_region = eval_region

            if self.draw:
                x1, y1, x2, y2 = map(int, box)
                cv2.rectangle(frame_out, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame_out, (cx, cy), 4, (0, 0, 255), -1)

                label_suffix = "" if confirmed[track_id] else " (new)"
                cv2.putText(
                    frame_out,
                    f"ID:{track_id}{label_suffix}",
                    (x1, max(y1 - 44, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    frame_out,
                    f"X:{x_smooth:.2f} Z:{z_smooth:.2f}",
                    (x1, max(y1 - 24, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 255),
                    2,
                )
                cv2.putText(
                    frame_out,
                    f"R:{current_region} dx:{rel_x:.2f} dz:{rel_z:.2f}",
                    (x1, max(y1 - 4, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.50,
                    (255, 255, 0),
                    2,
                )

                if predictions:
                    pred_tail = predictions[-min(5, len(predictions)):]
                    rep_x, rep_z, _ = min(
                        pred_tail,
                        key=lambda p: np.hypot(p[0] - cam_x_odom, p[1] - cam_z_odom)
                    )
                    rep_region, _, _ = self._classify_agv_region(
                        rep_x, rep_z, cam_x_odom, cam_z_odom, cam_yaw_odom)
                    cv2.putText(
                        frame_out,
                        f"P->X:{rep_x:.2f} Z:{rep_z:.2f} R:{rep_region}",
                        (x1, y2 + 18),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 165, 0),
                        2,
                    )

            output["detections"].append(
                {
                    "track_id": track_id,
                    "bbox_xyxy": [float(v) for v in box],
                    "center_px": [cx, cy],
                    "position_xz_m": {"x": float(x_smooth), "z": float(z_smooth)},
                    "history_xz": [
                        {"x": float(x), "z": float(z), "t": float(t)}
                        for x, z, t in tracks_xz[track_id]
                    ],
                    "predictions_xz": [
                        {"x": float(px), "z": float(pz), "t": float(pt)}
                        for px, pz, pt in predictions
                    ],
                    "track_confirmed": bool(confirmed[track_id]),
                    "missed_frames": int(self.track_missed_counts[track_id]),
                    "distance_to_agv_m": float(tmp_depth),
                    "region_relative_to_agv": current_region,
                    "relative_position_to_agv_m": {
                        "x": float(rel_x),
                        "z": float(rel_z),
                    },
                }
            )

        self._clear_stale_tracks(timestamp)

        if best_region is None and raw_cmd == "normal_operation":
            raw_state_str = "prediction_invalid_or_clear"

        self._apply_command_hold(raw_cmd, raw_state_str, timestamp)

        output["agv_movement"] = {
            "command": self.last_agv_cmd,
            "closest_human_depth_m": None if closest_human_depth is None else float(closest_human_depth),
            "range_state": self.last_agv_state_str,
        }

        t_post_end = time.perf_counter()

        if self.profile:
            output["timings_ms"] = {
                "total": round((time.perf_counter() - t0) * 1000.0, 2),
                "inference": round((t_infer_end - t_infer_start) * 1000.0, 2),
                "post": round((t_post_end - t_post_start) * 1000.0, 2),
            }

        return frame_out, output