"""cuda_cloud -- the organized cloud on the GPU, numerically the same thing as the numpy path.

WHY THIS EXISTS (measured 2026-09-13 on the drone NX, 20260910_flightlegal_bench)
---------------------------------------------------------------------------------
perception_node was the stack's biggest CPU consumer, and the notes blamed JPEG decode. It was
not. Per accepted frame, 200 real frames, scripts/bench_cloud_backend.py --yolo:

    stage                                                    wall        CPU     % core @ 8 Hz
    JPEG decode 1280x720                                      5.8 ms      5.8 ms    4.7
    depth -> colour register -> range image -> cloud (numpy) 38 ms      112 ms     90
    YOLO track (BoT-SORT, GMC sparseOptFlow)                 60 ms       41 ms     33
    the depth chain on the GPU (this module)                 16-31 ms  10-14 ms     8-11

And in the real node, replaying the same bag twice per backend (perception_node, dtp_params.yaml,
use_sim_time, 59 s): numpy 110.3 / 114.2 CPU-seconds, cuda 40.6 / 40.8 -- 1.9 cores down to 0.7.
The node saves more than the isolated bench predicts because OpenBLAS's worker threads spin
alongside everything else in the process. WALL time per frame did NOT improve in the node
(cloud 35 -> 28-33 ms, but YOLO 60-62 -> 62-75 ms on the shared GPU): this buys CPU headroom for
FAST-LIO and the rest of the stack, not latency. The GPU is already this process's single point
of failure -- YOLO runs there -- and torch's CUDA context already exists, so this adds no failure
domain and no context, and Jetson's unified memory means no PCIe copy.

The wall range above is the GPU devfreq governor (nvhost_podgov) clocking down to 306 MHz between
frames: 16 ms back-to-back, 31 ms paced at 8 Hz on an otherwise idle GPU.

WHAT "THE SAME THING" MEANS, PRECISELY -- READ BEFORE EDITING
-------------------------------------------------------------
This module reproduces, step for step and dtype for dtype under NumPy 2 promotion rules:

    CameraDepthSource._points_in_colour_frame -> DepthSource.range_image -> organized_cloud.build

Every elementwise step matches numpy BIT FOR BIT, because it is the same IEEE operation in the same
order: the float64 deprojection `(u - cx) * z / fx` (a float32 array minus a float64 scalar
promotes to float64 in NumPy 2), the float32 `sqrt(x*x + y*y + z*z)` range, the float64 colour
projection and its truncation to a pixel, the nearest-return z-buffer (`scatter_reduce` amin is
exactly argsort-then-last-write-wins), and the whole of `build()`, whose ray tables are computed by
numpy and uploaded rather than recomputed.

ONE STEP IS NOT GUARANTEED BIT-IDENTICAL: the depth->colour rigid transform, `pts_d @ R.T + t`.
numpy may run it in OpenBLAS, the GPU in its own kernels, and nothing promises the two fuse or
order their multiply-adds the same way in every library version. Measured on 2026-09-13 it WAS
identical -- 200 of 200 real frames with the camera's factory extrinsic, and 12 of 12 synthetic
scenes with a rotated one, all np.array_equal including NaN placement. Should a future torch,
CUDA or numpy change that, the damage is a depth sample landing one pixel over or moving by one
float32 ulp (~1 micron at 10 m), which no torso median could see. So the camera path is JUDGED
against a tolerance (CAMERA_MASK_TOL_FRAC, CAMERA_VALUE_TOL_M) rather than exact equality -- by
the unit tests, by scripts/bench_cloud_backend.py, and by perception_node on its first frames of
every run -- and a real regression still fails it by orders of magnitude. The LiDAR path
(`from_points`) starts from numpy's own points and its test asserts exact equality.

WHAT IT REFUSES TO DO
---------------------
It never returns a reused buffer: every call allocates the (H, W, 3) array fresh, the same contract
organized_cloud.build() documents, because perception_node pins queued clouds while it waits for
TF. It never swallows a geometry problem: intrinsics_problem() is the same gate build() applies.
And it never decides policy on failure -- a CUDA error propagates as CudaCloudError, and the
caller (perception_node) chooses the numpy fallback, where the choice is visible in its log.
"""
import numpy as np

from dtp_perception import organized_cloud

# How far the camera path may drift from numpy before it is refused (see the module docstring for
# why this is a tolerance and not equality; measured agreement to date is exact). 1e-4 of 921,600
# pixels is 92 pixels spread over a whole frame -- ~0.2 of a pixel inside a ~2,000 px torso patch
# -- and 1e-5 m is ten float32 ulps at 10 m. Both are far below what a torso median can notice and
# far above what a mere rounding difference could produce, so tripping either means a real bug.
CAMERA_MASK_TOL_FRAC = 1.0e-4
CAMERA_VALUE_TOL_M = 1.0e-5


class CudaCloudError(RuntimeError):
    """The GPU could not produce the cloud: no CUDA, a driver error, an allocation failure."""


def compare(reference, candidate):
    """(mask_mismatch_fraction, max_abs_diff_m) of `candidate` against a numpy `reference` cloud.

    Shared by the unit tests, the bag benchmark and perception_node's in-flight self-check so the
    three cannot disagree about what "the same" means.
    """
    ref = np.asarray(reference)
    cand = np.asarray(candidate)
    if ref.shape != cand.shape:
        return 1.0, float('inf')
    ref_ok = np.isfinite(ref[:, :, 2])
    cand_ok = np.isfinite(cand[:, :, 2])
    mask_frac = float(np.count_nonzero(ref_ok != cand_ok)) / float(ref_ok.size)
    both = ref_ok & cand_ok
    if not both.any():
        return mask_frac, 0.0
    return mask_frac, float(np.max(np.abs(ref[both] - cand[both])))


def within_camera_tolerance(mask_frac, max_diff_m):
    return mask_frac <= CAMERA_MASK_TOL_FRAC and max_diff_m <= CAMERA_VALUE_TOL_M


class CudaCloud:
    """Builds organized clouds on one CUDA device. One instance per process; not thread-safe.

    Construction imports torch and touches the device, so it raises CudaCloudError on a machine
    without CUDA rather than failing later inside a 8 Hz callback.
    """

    def __init__(self, device='cuda'):
        try:
            import torch
        except Exception as exc:  # ImportError, or a missing CUDA library behind it (libcudss)
            raise CudaCloudError('torch is not importable: %s' % exc)
        if not torch.cuda.is_available():
            raise CudaCloudError('torch reports no CUDA device')
        self._torch = torch
        try:
            self._dev = torch.device(device)
            torch.zeros(1, device=self._dev)
        except Exception as exc:
            raise CudaCloudError('cannot use CUDA device %r: %s' % (device, exc))
        self.device_name = torch.cuda.get_device_name(self._dev)
        self._rays_key = None       # (K bytes, width, height) the ray tables below belong to
        self._xn = self._yn = self._n = None
        self._grid_key = None       # (height, width, stride) the pixel grids below belong to
        self._uu = self._vv = None

    # -- cached tables -------------------------------------------------------------------------

    def _rays(self, K, width, height):
        """organized_cloud.build's xn / yn / n, computed BY NUMPY exactly as build() does, uploaded.

        Computing them in numpy and uploading, rather than recomputing with torch, is what makes
        the build step bit-identical instead of merely close.
        """
        key = (np.asarray(K, dtype=np.float64).tobytes(), int(width), int(height))
        if key != self._rays_key:
            Kf = np.asarray(K, dtype=np.float64)
            xn = ((np.arange(width, dtype=np.float64) - Kf[0, 2]) / Kf[0, 0]).astype(np.float32)
            yn = ((np.arange(height, dtype=np.float64) - Kf[1, 2]) / Kf[1, 1]).astype(np.float32)
            n = np.sqrt((xn * xn)[None, :] + (yn * yn)[:, None] + np.float32(1.0),
                        dtype=np.float32)
            t = self._torch
            self._xn = t.from_numpy(xn).to(self._dev)
            self._yn = t.from_numpy(yn).to(self._dev)
            self._n = t.from_numpy(np.ascontiguousarray(n)).to(self._dev)
            self._rays_key = key
        return self._xn, self._yn, self._n

    def _grid(self, height, width, stride):
        """np.mgrid[0:h:s, 0:w:s] as float32 on the device -- the `.astype(np.float32)` of the
        numpy path, applied to the whole grid once instead of to the valid subset every frame."""
        key = (int(height), int(width), int(stride))
        if key != self._grid_key:
            t = self._torch
            vv = t.arange(0, height, stride, device=self._dev, dtype=t.float32)
            uu = t.arange(0, width, stride, device=self._dev, dtype=t.float32)
            self._vv, self._uu = t.meshgrid(vv, uu, indexing='ij')
            self._grid_key = key
        return self._uu, self._vv

    # -- the two entry points ------------------------------------------------------------------

    def from_depth(self, depth_m, K_depth, R_colour_depth, t_colour_depth, stride,
                   K_colour, size, min_range_m, max_range_m):
        """Organized (H, W, 3) float32 cloud in the colour optical frame, from a METRIC depth image.

        `depth_m` is the (h, w) float32 image CameraDepthSource._depth_metres produces (decode and
        the encoding check stay on the CPU, in depth_source, so the two paths cannot disagree about
        units). Returns a freshly allocated numpy array.
        """
        width, height = int(size[0]), int(size[1])
        self._check_intrinsics(K_colour, height, width)
        try:
            return self._from_depth(np.asarray(depth_m, dtype=np.float32), K_depth,
                                    R_colour_depth, t_colour_depth, max(1, int(stride)),
                                    K_colour, width, height, float(min_range_m),
                                    float(max_range_m))
        except CudaCloudError:
            raise
        except Exception as exc:
            raise CudaCloudError('GPU cloud (camera depth) failed: %s: %s'
                                 % (type(exc).__name__, exc))

    def from_points(self, pts_colour, K_colour, size, min_range_m, max_range_m):
        """Organized cloud from (N, 3) float32 points ALREADY in the colour optical frame.

        The LiDAR source's path: its accumulation and extrinsic stay in numpy, and everything from
        the range computation on runs here -- bit-identical to range_image() + build().
        """
        width, height = int(size[0]), int(size[1])
        self._check_intrinsics(K_colour, height, width)
        try:
            t = self._torch
            pts = np.ascontiguousarray(np.asarray(pts_colour, dtype=np.float32).reshape(-1, 3))
            p = t.from_numpy(pts).to(self._dev)
            valid = t.ones(p.shape[0], dtype=t.bool, device=self._dev)
            return self._raster_and_build(p, valid, K_colour, width, height, float(min_range_m),
                                          float(max_range_m))
        except CudaCloudError:
            raise
        except Exception as exc:
            raise CudaCloudError('GPU cloud (points) failed: %s: %s' % (type(exc).__name__, exc))

    def warm_up(self, depth_size=(424, 240), colour_size=(1280, 720)):
        """Run the camera path once on a synthetic frame so the first REAL frame does not pay for
        CUDA kernel selection and allocator growth inside a 0.45 s TF-wait window."""
        dw, dh = int(depth_size[0]), int(depth_size[1])
        cw, ch = int(colour_size[0]), int(colour_size[1])
        depth = np.full((dh, dw), 2.0, dtype=np.float32)
        Kd = np.array([[dw, 0.0, dw / 2.0], [0.0, dw, dh / 2.0], [0.0, 0.0, 1.0]])
        Kc = np.array([[cw, 0.0, cw / 2.0], [0.0, cw, ch / 2.0], [0.0, 0.0, 1.0]])
        self.from_depth(depth, Kd, np.eye(3), np.zeros(3), 1, Kc, (cw, ch), 0.3, 15.0)
        self._rays_key = None       # the synthetic tables must not survive into real frames
        self._grid_key = None

    # -- internals -----------------------------------------------------------------------------

    @staticmethod
    def _check_intrinsics(K_colour, height, width):
        problem = organized_cloud.intrinsics_problem(K_colour, (height, width))
        if problem:
            raise ValueError(problem)

    def _from_depth(self, depth_m, Kd, R, tvec, s, Kc, width, height, min_r, max_r):
        t = self._torch
        dh, dw = depth_m.shape
        zz = t.from_numpy(np.ascontiguousarray(depth_m[0:dh:s, 0:dw:s])).to(self._dev)
        uu, vv = self._grid(dh, dw, s)
        # numpy COMPACTS to the valid pixels here (`uu[ok]`). This keeps every pixel and carries
        # `ok` as a mask instead: a boolean index is a device->host sync on every call, there were
        # eight of them in this chain, and they cost more wall time than the arithmetic. Masked-out
        # rows are computed and then ignored; no valid row's value depends on them.
        ok = (t.isfinite(zz) & (zz > 0)).reshape(-1)
        z = zz.reshape(-1).to(t.float64)
        Kd = np.asarray(Kd, dtype=np.float64)
        # (u(f32) - cx(f64)) * z(f32) / fx(f64): NumPy 2 promotes the whole expression to float64.
        x = (uu.reshape(-1).to(t.float64) - float(Kd[0, 2])) * z / float(Kd[0, 0])
        y = (vv.reshape(-1).to(t.float64) - float(Kd[1, 2])) * z / float(Kd[1, 1])
        pts_d = t.stack([x, y, z], dim=1)
        R64 = t.from_numpy(np.ascontiguousarray(np.asarray(R, dtype=np.float64))).to(self._dev)
        t64 = t.from_numpy(np.asarray(tvec, dtype=np.float64).reshape(3)).to(self._dev)
        # THE ONE STEP NOT GUARANTEED BIT-IDENTICAL -- see the module docstring (numpy: OpenBLAS).
        pts_c = (pts_d @ R64.T + t64).to(t.float32)
        return self._raster_and_build(pts_c, ok, Kc, width, height, min_r, max_r)

    def _raster_and_build(self, p, valid, Kc, width, height, min_r, max_r):
        """DepthSource.range_image() then organized_cloud.build(), for float32 points on device.

        `valid` marks the rows that numpy would have kept after its own compaction. Everything is
        fixed-shape -- no boolean indexing, so no host sync until the final copy out.
        """
        t = self._torch
        Kc = np.asarray(Kc, dtype=np.float64)
        xn, yn, n = self._rays(Kc, width, height)
        nan = float('nan')
        if p.shape[0] == 0:
            rng_img = t.full((height, width), nan, dtype=t.float32, device=self._dev)
        else:
            px, py, pz = p[:, 0], p[:, 1], p[:, 2]
            # np.linalg.norm(pts, axis=1) on float32: sqrt of the sequential sum of squares.
            rng = t.sqrt(px * px + py * py + pz * pz)
            # Python floats against float32 compare in float32 on both sides (NEP 50 weak scalars).
            keep = valid & (pz > 0.0) & (rng >= min_r) & (rng <= max_r)
            p64 = p.to(t.float64)
            # fx(f64 scalar) * x(f32) / z(f32) + cx: float64 throughout, then truncated to a pixel.
            uf = float(Kc[0, 0]) * p64[:, 0] / p64[:, 2] + float(Kc[0, 2])
            vf = float(Kc[1, 1]) * p64[:, 1] / p64[:, 2] + float(Kc[1, 2])
            inb = keep & (uf >= 0) & (uf < width) & (vf >= 0) & (vf < height)
            # Rows that are out become index H*W, a dummy slot sliced off below. They are zeroed
            # BEFORE the integer cast because a masked row can hold inf/NaN (z = 0 or a NaN depth).
            zero = t.zeros_like(uf)
            u_i = t.where(inb, uf, zero).to(t.int64)
            v_i = t.where(inb, vf, zero).to(t.int64)
            idx = t.where(inb, v_i * width + u_i, t.full_like(u_i, height * width))
            inf = float('inf')
            vals = t.where(inb, rng, t.full_like(rng, inf))
            # Nearest return per pixel. numpy sorts by descending range and lets the last write
            # win, which leaves the minimum; amin over the same values leaves the same float.
            flat = t.full((height * width + 1,), inf, dtype=t.float32, device=self._dev)
            flat.scatter_reduce_(0, idx, vals, reduce='amin')
            rng_img = flat[:height * width].view(height, width)
            rng_img = t.where(t.isinf(rng_img), t.full_like(rng_img, nan), rng_img)
        # organized_cloud.build(), operation for operation.
        scale = rng_img / n
        valid = t.isfinite(rng_img) & (rng_img > 0.0)
        scale = t.where(valid, scale, t.full_like(scale, nan))
        cloud = t.stack([scale * xn[None, :], scale * yn[:, None], scale], dim=2)
        # .cpu() allocates a new host tensor every call -> the freshly-allocated contract.
        return cloud.cpu().numpy()
