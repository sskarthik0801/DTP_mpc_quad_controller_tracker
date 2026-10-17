import numpy as np
import cv2
import matplotlib.pyplot as plt


def _label(img, text, org, color):
    """Small text on a dark backing box -- legible over any scene, which raw putText is not."""
    x, y = org
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
    y = max(y, th + 3)                                   # never clip off the top of the frame
    cv2.rectangle(img, (x - 2, y - th - 3), (x + tw + 2, y + 3), (0, 0, 0), -1)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)


def _to_robot_frame(px, pz, cam_pose):
    """World (x, z) -> AGV-frame (x, z). Mirrors HumanXZPredictor._classify_agv_region exactly.

    Predictions are stored in the ODOM frame, but the current position is reported AGV-relative.
    Differencing without rotating would put the two on different axes -- so `pred` would not be
    comparable to `now`, which is the whole point of showing them together.
    """
    cx, cz, yaw = cam_pose
    dx, dz = px - cx, pz - cz
    if yaw:
        c, s = np.cos(yaw), np.sin(yaw)
        return float(dx * c - dz * s), float(dx * s + dz * c)
    return float(dx), float(dz)


def draw_detection_overlay(img, detections, cam_pose=(0.0, 0.0, 0.0)):
    """Per-track box + what the policy is actually thinking about that track.

    Three lines per box, all distances in metres and all coordinates in the AGV frame
    (x = right of the robot, z = ahead of it):

        ID<id> FRONT|SIDE d:<range>       <- which zone's thresholds this track is being judged by
        now (x,z)=(<rel_x>, <rel_z>)      <- where the person is
        pred(x,z)=(<px>, <pz>) d:<pd>     <- where the track is heading, at closest approach

    The predicted point is the one nearest the AGV over the predicted path -- the same
    representative the policy scores against, so a `stop` with no human currently in range is
    explained by that line (`why=front_range_predicted`).

    `cam_pose` is (x, z, yaw) of the AGV in the odom frame; (0, 0, 0) is correct when running
    robot-relative, since the predictor puts positions in the camera frame in that mode.

    Box colour is region (orange = front corridor, green = side); an unconfirmed track is drawn
    thin with a `?` -- the policy treats those differently, so the picture should too.
    """
    for det in detections:
        try:
            x1, y1, x2, y2 = (int(v) for v in det['bbox_xyxy'])
        except (KeyError, TypeError, ValueError):
            continue

        region = det.get('region_relative_to_agv')
        confirmed = bool(det.get('track_confirmed'))
        color = (0, 140, 255) if region == 'front' else (0, 200, 0)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2 if confirmed else 1)

        tag = f"ID{det.get('track_id')} {'FRONT' if region == 'front' else 'SIDE'}"
        if not confirmed:
            tag += "?"
        dist = det.get('distance_to_agv_m')
        _label(img, f"{tag} d:{dist:.2f}" if dist is not None else tag, (x1, y1 - 6), color)

        rel = det.get('relative_position_to_agv_m') or {}
        _label(img, f"now (x,z)=({rel.get('x', 0.0):+.2f}, {rel.get('z', 0.0):+.2f})",
               (x1, y2 + 16), color)

        preds = det.get('predictions_xz') or []
        if preds:
            cx, cz = cam_pose[0], cam_pose[1]
            best = min(preds, key=lambda p: (p['x'] - cx) ** 2 + (p['z'] - cz) ** 2)
            px, pz = _to_robot_frame(best['x'], best['z'], cam_pose)
            pd = ((best['x'] - cx) ** 2 + (best['z'] - cz) ** 2) ** 0.5
            line3 = f"pred(x,z)=({px:+.2f}, {pz:+.2f}) d:{pd:.2f}"
        else:
            line3 = "pred(x,z)= --"
        _label(img, line3, (x1, y2 + 32), color)
    return img


def draw_policy_hud(img, command, closest_m=None, vx=None, nav=None, fps=None):
    """Compact top-left HUD on a camera frame: cmd | closest depth | speed | fps, then nav.

    Shorthand so it doesn't crowd the frame. vx / nav / fps are omitted when None (e.g. the
    offline replay has no robot state or live rate). Draws in place and returns the image.
    """
    short = {'stop': 'STOP', 'slow_down': 'SLOW',
             'normal_operation': 'RUN'}.get(command, str(command)[:4].upper())
    line1 = f"{short}  d:{closest_m:.2f}" if closest_m is not None else f"{short}  d:--"
    if vx is not None:
        line1 += f"  vx:{vx:.2f}"
    if fps is not None:
        line1 += f"  fps:{fps:.1f}"
    cv2.putText(img, line1, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    if nav is not None:
        cv2.putText(img, f"nav:{nav}", (12, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    return img


class WorldXZOpenCVVisualizer:
    """
    OpenCV version of the top-down XZ visualizer.
    """

    def __init__(
        self,
        canvas_size=(800, 800),
        xlim=(-5, 5),
        zlim=(0, 10),
        max_camera_trail=500,
        max_human_trail=100,
        pred_dist_limit=2.0,
        auto_expand=False,
        view_margin=1.0,
        min_view_size=(10.0, 10.0),
        bg_color=(20, 20, 20),
    ):
        self.canvas_w = int(canvas_size[0])
        self.canvas_h = int(canvas_size[1])

        self.xlim = xlim
        self.zlim = zlim
        self.max_camera_trail = max_camera_trail
        self.max_human_trail = max_human_trail
        self.pred_dist_limit = pred_dist_limit

        self.auto_expand = auto_expand
        self.view_margin = view_margin
        self.min_view_size = min_view_size

        self.bg_color = bg_color

        self.camera_trail = []
        self.human_trails = {}

    @staticmethod
    def _pose_to_xz_and_forward(pose):
        if pose is None or "translation" not in pose:
            return None, None

        t = pose["translation"]
        q = pose["orientation_quat"]

        tx, tz = float(t["x"]), float(t["z"])
        x, y, z, w = float(q["x"]), float(q["y"]), float(q["z"]), float(q["w"])

        R = np.array([
            [1 - 2 * (y**2 + z**2),     2 * (x*y - z*w),     2 * (x*z + y*w)],
            [    2 * (x*y + z*w), 1 - 2 * (x**2 + z**2),     2 * (y*z - x*w)],
            [    2 * (x*z - y*w),     2 * (y*z + x*w), 1 - 2 * (x**2 + y**2)]
        ], dtype=np.float64)

        forward = R[:, 2]
        forward_xz = np.array([forward[0], forward[2]], dtype=np.float64)

        norm = np.linalg.norm(forward_xz)
        if norm > 1e-8:
            forward_xz /= norm
        else:
            forward_xz = np.array([0.0, 1.0], dtype=np.float64)

        return (tx, tz), forward_xz

    def _update_camera_trail(self, cam_xz):
        if cam_xz is None:
            return
        self.camera_trail.append(cam_xz)
        if len(self.camera_trail) > self.max_camera_trail:
            self.camera_trail = self.camera_trail[-self.max_camera_trail:]

    def _update_human_trails(self, detections):
        active_ids = set()

        for det in detections:
            track_id = det["track_id"]
            active_ids.add(track_id)

            history = det.get("history_xz", [])
            if not history:
                continue

            pts = [(float(p["x"]), float(p["z"])) for p in history]
            if len(pts) > self.max_human_trail:
                pts = pts[-self.max_human_trail:]

            self.human_trails[track_id] = pts

        stale_ids = [tid for tid in self.human_trails if tid not in active_ids]
        for tid in stale_ids:
            del self.human_trails[tid]

    def _compute_dynamic_limits(self, cam_xz, detections):
        all_x = []
        all_z = []

        if cam_xz is not None:
            all_x.append(cam_xz[0])
            all_z.append(cam_xz[1])

        if self.camera_trail:
            cam_arr = np.asarray(self.camera_trail, dtype=np.float64)
            all_x.extend(cam_arr[:, 0].tolist())
            all_z.extend(cam_arr[:, 1].tolist())

        for det in detections:
            track_id = det["track_id"]
            history = self.human_trails.get(track_id, [])
            preds = det.get("predictions_xz", [])

            for x, z in history:
                all_x.append(float(x))
                all_z.append(float(z))

            if history and preds:
                last_x, last_z = history[-1]
                for p in preds:
                    px, pz = float(p["x"]), float(p["z"])
                    dist = np.hypot(px - last_x, pz - last_z)
                    if dist <= self.pred_dist_limit:
                        all_x.append(px)
                        all_z.append(pz)
                        last_x, last_z = px, pz
                    else:
                        break

        if not all_x or not all_z:
            return self.xlim, self.zlim

        xmin, xmax = min(all_x), max(all_x)
        zmin, zmax = min(all_z), max(all_z)


        xmin -= self.view_margin
        xmax += self.view_margin
        zmin -= self.view_margin
        zmax += self.view_margin

        min_width, min_height = self.min_view_size

        if (xmax - xmin) < min_width:
            cx = 0.5 * (xmin + xmax)
            xmin = cx - min_width / 2.0
            xmax = cx + min_width / 2.0

        if (zmax - zmin) < min_height:
            cz = 0.5 * (zmin + zmax)
            zmin = cz - min_height / 2.0
            zmax = cz + min_height / 2.0

        return (xmin, xmax), (zmin, zmax)



    def _world_to_px(self, x, z, xlim, zlim, pad=50):
        xmin, xmax = xlim
        zmin, zmax = zlim

        draw_w = self.canvas_w - 2 * pad
        draw_h = self.canvas_h - 2 * pad

        if xmax - xmin < 1e-9:
            xmax = xmin + 1.0
        if zmax - zmin < 1e-9:
            zmax = zmin + 1.0

        u = pad + (x - xmin) / (xmax - xmin) * draw_w
        v = self.canvas_h - pad - (z - zmin) / (zmax - zmin) * draw_h
        return int(round(u)), int(round(v))




    def _draw_axes_and_grid(self, canvas, xlim, zlim, pad=50):
        xmin, xmax = xlim
        zmin, zmax = zlim

        cv2.rectangle(
            canvas,
            (pad, pad),
            (self.canvas_w - pad, self.canvas_h - pad),
            (100, 100, 100),
            1,
        )

        grid_color = (50, 50, 50)
        text_color = (180, 180, 180)

        x_start = int(np.floor(xmin))
        x_end = int(np.ceil(xmax))
        z_start = int(np.floor(zmin))
        z_end = int(np.ceil(zmax))

        for gx in range(x_start, x_end + 1):
            p1 = self._world_to_px(gx, zmin, xlim, zlim, pad)
            p2 = self._world_to_px(gx, zmax, xlim, zlim, pad)
            cv2.line(canvas, p1, p2, grid_color, 1)

        for gz in range(z_start, z_end + 1):
            p1 = self._world_to_px(xmin, gz, xlim, zlim, pad)
            p2 = self._world_to_px(xmax, gz, xlim, zlim, pad)
            cv2.line(canvas, p1, p2, grid_color, 1)

        if xmin <= 0 <= xmax:
            p1 = self._world_to_px(0, zmin, xlim, zlim, pad)
            p2 = self._world_to_px(0, zmax, xlim, zlim, pad)
            cv2.line(canvas, p1, p2, (90, 90, 180), 2)

        if zmin <= 0 <= zmax:
            p1 = self._world_to_px(xmin, 0, xlim, zlim, pad)
            p2 = self._world_to_px(xmax, 0, xlim, zlim, pad)
            cv2.line(canvas, p1, p2, (90, 180, 90), 2)

        cv2.putText(canvas, "World XZ View", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2)
        cv2.putText(canvas, f"X:[{xlim[0]:.1f},{xlim[1]:.1f}]  Z:[{zlim[0]:.1f},{zlim[1]:.1f}]",
                    (20, self.canvas_h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1)

    def render(self, pose, results):
        detections = results.get("detections", []) if results is not None else []

        cam_xz, cam_forward_xz = self._pose_to_xz_and_forward(pose)
        self._update_camera_trail(cam_xz)
        self._update_human_trails(detections)

        if self.auto_expand:
            xlim, zlim = self._compute_dynamic_limits(cam_xz, detections)
        else:
            xlim, zlim = self.xlim, self.zlim

        canvas = np.full((self.canvas_h, self.canvas_w, 3), self.bg_color, dtype=np.uint8)

        self._draw_axes_and_grid(canvas, xlim, zlim, pad=50)

        if len(self.camera_trail) >= 2:
            cam_pts = [self._world_to_px(x, z, xlim, zlim, pad=50) for x, z in self.camera_trail]
            for i in range(1, len(cam_pts)):
                cv2.line(canvas, cam_pts[i - 1], cam_pts[i], (0, 220, 220), 2)

        if cam_xz is not None:
            cam_px = self._world_to_px(cam_xz[0], cam_xz[1], xlim, zlim, pad=50)
            cv2.circle(canvas, cam_px, 7, (0, 0, 255), -1)

            if cam_forward_xz is not None:
                arrow_len_m = 0.5
                fx = cam_xz[0] + cam_forward_xz[0] * arrow_len_m
                fz = cam_xz[1] + cam_forward_xz[1] * arrow_len_m
                arrow_tip = self._world_to_px(fx, fz, xlim, zlim, pad=50)
                cv2.arrowedLine(canvas, cam_px, arrow_tip, (0, 0, 255), 2, tipLength=0.25)

            cv2.putText(canvas, "Camera", (cam_px[0] + 10, cam_px[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        for det in detections:
            track_id = det["track_id"]
            history = self.human_trails.get(track_id, [])
            preds = det.get("predictions_xz", [])

            if history:
                hist_px = [self._world_to_px(x, z, xlim, zlim, pad=50) for x, z in history]

                for i in range(1, len(hist_px)):
                    cv2.line(canvas, hist_px[i - 1], hist_px[i], (255, 0, 0), 2)

                for p in hist_px:
                    cv2.circle(canvas, p, 2, (255, 120, 0), -1)

                cv2.circle(canvas, hist_px[-1], 5, (255, 0, 0), -1)
                cv2.putText(canvas, f"ID {track_id}",
                            (hist_px[-1][0] + 8, hist_px[-1][1] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 1)

                if preds:
                    pred_pts = [(float(p["x"]), float(p["z"])) for p in preds]

                    filtered = []
                    last_x, last_z = history[-1]
                    for px, pz in pred_pts:
                        dist = np.hypot(px - last_x, pz - last_z)
                        if dist <= self.pred_dist_limit:
                            filtered.append((px, pz))
                            last_x, last_z = px, pz
                        else:
                            break

                    if filtered:
                        pred_px = [self._world_to_px(x, z, xlim, zlim, pad=50) for x, z in filtered]

                        cv2.line(canvas, hist_px[-1], pred_px[0], (0, 255, 0), 1)
                        for i in range(1, len(pred_px)):
                            cv2.line(canvas, pred_px[i - 1], pred_px[i], (0, 255, 0), 1)

                        for p in pred_px:
                            cv2.drawMarker(
                                canvas,
                                p,
                                (0, 255, 0),
                                markerType=cv2.MARKER_TILTED_CROSS,
                                markerSize=8,
                                thickness=1,
                            )

                        cv2.putText(canvas, f"Pred {track_id}",
                                    (pred_px[-1][0] + 8, pred_px[-1][1] - 8),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        legend_y = 55
        cv2.line(canvas, (20, legend_y), (50, legend_y), (0, 220, 220), 2)
        cv2.putText(canvas, "Camera Trail", (60, legend_y + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1)

        legend_y += 25
        cv2.circle(canvas, (35, legend_y), 5, (0, 0, 255), -1)
        cv2.putText(canvas, "Camera", (60, legend_y + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1)

        legend_y += 25
        cv2.line(canvas, (20, legend_y), (50, legend_y), (255, 0, 0), 2)
        cv2.putText(canvas, "Human Track", (60, legend_y + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1)

        legend_y += 25
        cv2.line(canvas, (20, legend_y), (50, legend_y), (0, 255, 0), 1)
        cv2.putText(canvas, "Prediction", (60, legend_y + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1)

        return canvas





class WorldXZVisualizer:
    def __init__(
        self,
        ax=None,
        xlim=(-10, 10),
        zlim=(0, 20),
        max_camera_trail=500,
        max_human_trail=100,
        pred_dist_limit=2.0,
        auto_expand=False,
        view_margin=1.0,
        min_view_size=(10.0, 10.0),
    ):
        self.ax = ax if ax is not None else plt.subplots(figsize=(6, 6))[1]
        self.xlim = xlim
        self.zlim = zlim
        self.max_camera_trail = max_camera_trail
        self.max_human_trail = max_human_trail
        self.pred_dist_limit = pred_dist_limit

        self.auto_expand = auto_expand
        self.view_margin = view_margin
        self.min_view_size = min_view_size  # (min_width, min_height)

        self.camera_trail = []
        self.human_trails = {}  # track_id -> [(x,z), ...]

    @staticmethod
    def _pose_to_xz_and_forward(pose):
        if pose is None:
            return None, None

        t = pose["translation"]
        q = pose["orientation_quat"]

        tx, tz = float(t["x"]), float(t["z"])
        x, y, z, w = float(q["x"]), float(q["y"]), float(q["z"]), float(q["w"])

        # Quaternion -> rotation matrix
        R = np.array([
            [1 - 2 * (y**2 + z**2),     2 * (x*y - z*w),     2 * (x*z + y*w)],
            [    2 * (x*y + z*w), 1 - 2 * (x**2 + z**2),     2 * (y*z - x*w)],
            [    2 * (x*z - y*w),     2 * (y*z + x*w), 1 - 2 * (x**2 + y**2)]
        ], dtype=np.float64)

        # Camera forward axis in world frame
        forward = R[:, 2]
        forward_xz = np.array([forward[0], forward[2]], dtype=np.float64)

        norm = np.linalg.norm(forward_xz)
        if norm > 1e-8:
            forward_xz /= norm
        else:
            forward_xz = np.array([0.0, 1.0], dtype=np.float64)

        return (tx, tz), forward_xz

    def _update_camera_trail(self, cam_xz):
        if cam_xz is None:
            return
        self.camera_trail.append(cam_xz)
        if len(self.camera_trail) > self.max_camera_trail:
            self.camera_trail = self.camera_trail[-self.max_camera_trail:]

    def _update_human_trails(self, detections):
        active_ids = set()

        for det in detections:
            track_id = det["track_id"]
            active_ids.add(track_id)

            history = det.get("history_xz", [])
            if not history:
                continue

            pts = [(float(p["x"]), float(p["z"])) for p in history]
            if len(pts) > self.max_human_trail:
                pts = pts[-self.max_human_trail:]

            self.human_trails[track_id] = pts

        stale_ids = [tid for tid in self.human_trails if tid not in active_ids]
        for tid in stale_ids:
            del self.human_trails[tid]

    def _compute_dynamic_limits(self, cam_xz, detections):
        all_x = []
        all_z = []

        # Camera current position
        if cam_xz is not None:
            all_x.append(cam_xz[0])
            all_z.append(cam_xz[1])

        # Camera trail
        if self.camera_trail:
            cam_arr = np.asarray(self.camera_trail, dtype=np.float64)
            all_x.extend(cam_arr[:, 0].tolist())
            all_z.extend(cam_arr[:, 1].tolist())

        # Human history + filtered predictions
        for det in detections:
            track_id = det["track_id"]
            history = self.human_trails.get(track_id, [])
            preds = det.get("predictions_xz", [])

            for x, z in history:
                all_x.append(float(x))
                all_z.append(float(z))

            if history and preds:
                last_x, last_z = history[-1]
                for p in preds:
                    px, pz = float(p["x"]), float(p["z"])
                    dist = np.hypot(px - last_x, pz - last_z)
                    if dist <= self.pred_dist_limit:
                        all_x.append(px)
                        all_z.append(pz)
                        last_x, last_z = px, pz
                    else:
                        break

        # Fallback to default limits if no points exist
        if not all_x or not all_z:
            return self.xlim, self.zlim

        xmin, xmax = min(all_x), max(all_x)
        zmin, zmax = min(all_z), max(all_z)

        # Add margin
        xmin -= self.view_margin
        xmax += self.view_margin
        zmin -= self.view_margin
        zmax += self.view_margin
        


        # Enforce minimum view size
        min_width, min_height = self.min_view_size

        if (xmax - xmin) < min_width:
            cx = 0.5 * (xmin + xmax)
            xmin = cx - min_width / 2.0
            xmax = cx + min_width / 2.0

        if (zmax - zmin) < min_height:
            cz = 0.5 * (zmin + zmax)
            zmin = cz - min_height / 2.0
            zmax = cz + min_height / 2.0

        return (xmin, xmax), (zmin, zmax)

    def update(self, pose, results):
        detections = results.get("detections", []) if results is not None else []

        cam_xz, cam_forward_xz = self._pose_to_xz_and_forward(pose)
        self._update_camera_trail(cam_xz)
        self._update_human_trails(detections)

        ax = self.ax
        ax.clear()

        ax.set_title("World XZ View")
        ax.set_xlabel("X (left/right, meters)")
        ax.set_ylabel("Z (forward, meters)")

        if self.auto_expand:
            xlim, zlim = self._compute_dynamic_limits(cam_xz, detections)
            ax.set_xlim(xlim)
            ax.set_ylim(zlim)
        else:
            ax.set_xlim(self.xlim)
            ax.set_ylim(self.zlim)

        ax.grid(True, linestyle="--", alpha=0.6)
        ax.set_aspect("equal")

        # Camera trail
        if self.camera_trail:
            cam_arr = np.asarray(self.camera_trail, dtype=np.float64)
            ax.plot(
                cam_arr[:, 0],
                cam_arr[:, 1],
                linestyle="-",
                linewidth=2,
                alpha=0.8,
                label="Camera Trail",
            )

        # Camera current position
        if cam_xz is not None:
            ax.scatter(
                cam_xz[0],
                cam_xz[1],
                s=80,
                c="red",
                label="Camera",
                zorder=5,
            )

            if cam_forward_xz is not None:
                arrow_len = 0.4
                ax.arrow(
                    cam_xz[0],
                    cam_xz[1],
                    cam_forward_xz[0] * arrow_len,
                    cam_forward_xz[1] * arrow_len,
                    head_width=0.08,
                    head_length=0.12,
                    length_includes_head=True,
                    color="red",
                    zorder=6,
                )

        # Human trails + current positions + predictions
        drew_human_label = False
        drew_pred_label = False

        for det in detections:
            track_id = det["track_id"]
            history = self.human_trails.get(track_id, [])
            preds = det.get("predictions_xz", [])

            if history:
                hist = np.asarray(history, dtype=np.float64)

                ax.plot(
                    hist[:, 0],
                    hist[:, 1],
                    linestyle="-",
                    marker="o",
                    markersize=3,
                    alpha=0.9,
                    c="blue",
                    label="Human Track" if not drew_human_label else None,
                )
                drew_human_label = True

                ax.scatter(
                    hist[-1, 0],
                    hist[-1, 1],
                    s=50,
                    c="blue",
                    zorder=5,
                )
                ax.text(
                    hist[-1, 0],
                    hist[-1, 1],
                    f" ID {track_id}",
                    fontsize=9,
                )

                if preds:
                    pred_pts = [(float(p["x"]), float(p["z"])) for p in preds]

                    # Stop if predictions jump too far
                    filtered = []
                    last_x, last_z = hist[-1, 0], hist[-1, 1]
                    for px, pz in pred_pts:
                        dist = np.hypot(px - last_x, pz - last_z)
                        if dist <= self.pred_dist_limit:
                            filtered.append((px, pz))
                            last_x, last_z = px, pz
                        else:
                            break

                    if filtered:
                        pred = np.asarray(filtered, dtype=np.float64)
                        ax.plot(
                            pred[:, 0],
                            pred[:, 1],
                            linestyle="--",
                            marker="x",
                            alpha=0.9,
                            c="green",
                            label="Prediction" if not drew_pred_label else None,
                        )
                        drew_pred_label = True

                        ax.scatter(
                            pred[-1, 0],
                            pred[-1, 1],
                            s=60,
                            marker="x",
                            c="green",
                            zorder=5,
                        )
                        ax.text(
                            pred[-1, 0],
                            pred[-1, 1],
                            f" Pred {track_id}",
                            fontsize=9,
                        )

        if cam_xz is not None or detections:
            ax.legend(loc="upper right")

        plt.draw()
        plt.pause(0.001)



