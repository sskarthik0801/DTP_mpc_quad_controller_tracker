#!/usr/bin/env python3
"""M6.6: the offline predictor study behind the paper's Table II (m5-m6-plan.md, row 24).

THE QUESTION
------------
The follower forecasts the person with a CTRV model (detection.py HumanXZPredictor
._predict_future_xz). A learned branch (EigenTrajectory, fused with CTRV at a constant
eigen_fusion_alpha) exists in the code but is disabled in flight and its assets never reached
any box we have. The paper wants to know: with complete histories and with 10 / 30 / 50 % of the
observation steps missing -- a detector at 8 Hz drops frames, the range gate holds them --
which predictor is better, and does the fusion earn its place?

THE PROTOCOL (TrajImpute-style, on ETH/UCY)
-------------------------------------------
Scenes eth, hotel, univ, zara1, zara2, TEST split, loaded with the public repo's own
TrajectoryDataset (obs 8 x 0.4 s, pred 12 x 0.4 s, skip 1, scenes with >= 2 pedestrians -- the
same trajectories the repo's `trainval.py --test` scores). Leave-one-out: the checkpoint for a
scene was trained on the other four (the public release's convention).

For each trajectory and each missing rate r in {0, 0.1, 0.3, 0.5}: k = round(8 r) of the seven
NON-FINAL observation steps are dropped uniformly at random (k = 0, 1, 2, 4; the last observed
step, t = 2.8 s, is never dropped -- the predictor always knows where the person is now).
Fixed seed per (scene, rate); the same mask feeds every predictor.

Predictors (all score against the same 12-step ground truth, best-of-1 unless said):
  HOLD       the last observed position repeated. The floor every arm must beat.
  CV         constant velocity from the last TWO observed steps and their real stamps.
  CTRV-repo  the flown code: HumanXZPredictor._predict_future_xz called unbound on the observed
             steps with their real timestamps (i * 0.4 s), prediction_steps 12 /
             prediction_secs 4.8 so its steps land on the ground-truth stamps. Two things about
             it are protocol facts, not bugs: (a) its heading needs two 6-sample baselines, so
             it produces NOTHING from fewer than 8 points -- at every non-zero missing rate it
             is 100 % fallback (scored as HOLD; in flight, where the history is 8 Hz x 60, it
             never starves); (b) it starts the forecast from an EMA(0.30) of the positions,
             which at 0.4 s spacing lags ~0.9 s. The fallback share is reported.
  CTRV-fit   the same motion model made gap-native: over the observed points, segment speeds
             s_j = |dp|/dt (v = mean, clipped to 3 m/s as the repo does); headings of segments
             longer than 0.08 m (the repo's threshold), unwrapped; with >= 3 such segments the
             yaw rate is the length-weighted least-squares slope of heading vs segment
             mid-time, clipped to +-1.2 rad/s (repo clip), zeroed below 0.25 m/s (repo rule)
             and zeroed by the repo's consistency guard (segment yaw rates fewer than 3, their
             std > 1.8 x |mean|, or |mean| < 0.15 rad/s), and the heading is that fit evaluated
             at the last stamp; otherwise yaw rate 0 and heading = direction of the summed
             displacement. Propagated with the repo's exact arc formulas from the true last
             position, no per-step decay. Measured 2026-09-14 on full eth/hotel/zara1 at 0 %:
             without the guard the turn term costs 0.1-0.25 m ADE; with it CTRV-fit sits at
             CV level -- on a 4.8 s horizon the yaw-rate state earns nothing on ETH/UCY.
  EIGEN      the public ET-STGCNN, fed an 8-step window with the dropped steps IMPUTED:
               ffill  forward-fill (hold the last observed step; a leading gap is back-filled
                      from the first observed step). This is what the flown code does with a
                      frame it cannot range: perception_node republishes the LAST position
                      under the new stamp ("HELD") -- the repo's own imputation, primary arm.
               interp per-coordinate linear interpolation between observed steps (np.interp;
                      a leading gap holds the first observed value).
             The model emits 20 samples with no likelihood. Point rules scored:
               raw0    the model's sample 0 -- what the flown adapter (preds[0]) would take
                       from an unmodified public forward;
               medoid  the sample whose endpoint is nearest the mean endpoint;
               mean    the mean of the 20 samples;
               nearcv  the sample nearest the CV continuation of the (imputed) window --
                       the rule detection_pipeline/EigenTrajectory/eigen.py puts at index 0,
                       so this is what the adapter flies. Used for the fusion.
             bo20 = min over the 20 samples, the public paper's metric (an oracle).
  FUSED      per step (1 - a) * CTRV + a * EIGEN(nearcv), exactly _fuse_future_predictions:
             when CTRV returned nothing the fused output is EIGEN alone. Both CTRVs are fused.
               const  a = 0.2 = HumanXZPredictor.EIGEN_TRAJECTORY_FUSION_ALPHA
               sched  a = 0.2 * (n_observed / 8)  -- alpha scaled by the fraction of steps
                      seen: 0.2, 0.175, 0.15, 0.10 at k = 0, 1, 2, 4
             plus a constant-alpha sweep {0, 0.2, 0.5, 0.8, 1.0} so the authors can pick.

Metrics: ADE (mean over the 12 steps of the L2 error, m) and FDE (step 12, m), averaged over
the scene's trajectories; AVG = mean of the five scene means (the ETH/UCY convention).

    source ~/super_ws/src/drone-dtp/ros2_ws/dtp_env_jetson.sh      # libcudss for torch
    python3 drone-dtp/scripts/m66_predictor_study.py                  # ~12 min on the Orin NX
    python3 drone-dtp/scripts/m66_predictor_study.py --scenes eth --max-seqs 20   # smoke

Writes results.csv, table.md, per_traj.npz (per-trajectory ADE/FDE for paired tests),
meta.json and run.log to --out (default ~/dtp_datasets/m66_predictor_study/) and prints
the tables.

Sanity anchor: at r = 0, EIGEN bo20 must reproduce the repo's own `trainval.py --test`
(eth 0.365 / 0.582, hotel 0.147 / 0.220 measured here 2026-09-14) -- it does, exactly.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from types import SimpleNamespace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DTP_ROOT = os.path.dirname(HERE)
ET_DIR = os.path.join(DTP_ROOT, 'detection_pipeline', 'EigenTrajectory')
for p in (DTP_ROOT, ET_DIR):
    if p not in sys.path:
        sys.path.insert(0, p)

SCENES = ['eth', 'hotel', 'univ', 'zara1', 'zara2']
RATES = [0.0, 0.1, 0.3, 0.5]
OBS_LEN, PRED_LEN, DT = 8, 12, 0.4
ALPHA_SWEEP = [0.0, 0.2, 0.5, 0.8, 1.0]
IMPUTATIONS = ['ffill', 'interp']
POINT_RULES = ['raw0', 'medoid', 'mean', 'nearcv']
FLOWN_RULE = 'nearcv'
STEPS = np.arange(1, PRED_LEN + 1, dtype=np.float64)[:, None] * DT     # 12 1, seconds ahead

# the repo's CTRV clips, reused by CTRV-fit
V_MAX, YAW_RATE_MAX, V_STATIC, HEADING_MIN_DIST = 3.0, 1.2, 0.25, 0.08


# ----------------------------------------------------------------------------------------------
# Physics predictors
# ----------------------------------------------------------------------------------------------
def make_ctrv_repo():
    """f(history[(x, y, t), ...]) -> [12, 2] or None: HumanXZPredictor._predict_future_xz
    verbatim (self is a stand-in carrying only the attributes it reads)."""
    from detection_pipeline.detection import HumanXZPredictor
    shim = SimpleNamespace(
        min_history_for_prediction=3,
        prediction_steps=PRED_LEN,
        prediction_secs=PRED_LEN * DT,
        _ema_1d=HumanXZPredictor._ema_1d,
        _robust_trimmed_mean=HumanXZPredictor._robust_trimmed_mean,
    )
    fn = HumanXZPredictor._predict_future_xz

    def predict(history):
        out = fn(shim, history)
        if not out:
            return None
        arr = np.asarray(out, dtype=np.float64)[:, :2]
        if arr.shape[0] != PRED_LEN or not np.isfinite(arr).all():
            return None
        return arr

    return predict, float(HumanXZPredictor.EIGEN_TRAJECTORY_FUSION_ALPHA)


def propagate_ctrv(p0, v, yaw, yaw_rate):
    """The repo's arc formulas, 12 steps of DT from p0, constant v and yaw_rate."""
    out = np.zeros((PRED_LEN, 2))
    x, y = float(p0[0]), float(p0[1])
    for i in range(PRED_LEN):
        if abs(yaw_rate) < 1e-3:
            x += v * np.cos(yaw) * DT
            y += v * np.sin(yaw) * DT
        else:
            yaw_next = yaw + yaw_rate * DT
            x += (v / yaw_rate) * (np.sin(yaw_next) - np.sin(yaw))
            y += (v / yaw_rate) * (-np.cos(yaw_next) + np.cos(yaw))
            yaw = yaw_next
        out[i] = (x, y)
    return out


def ctrv_fit(pts, ts):
    """Gap-native CTRV on observed points pts [m, 2] at stamps ts [m] -> [12, 2] (m >= 1)."""
    if len(pts) < 2:
        return np.repeat(pts[-1:], PRED_LEN, axis=0)
    d = np.diff(pts, axis=0)
    dt = np.diff(ts)
    seg_len = np.linalg.norm(d, axis=1)
    v = float(np.clip(np.mean(seg_len / dt), 0.0, V_MAX))
    good = seg_len >= HEADING_MIN_DIST
    if v < V_STATIC or good.sum() < 3:
        yaw_rate = 0.0
        s = d.sum(axis=0)
        yaw = float(np.arctan2(s[1], s[0])) if np.linalg.norm(s) > 1e-9 else 0.0
    else:
        h = np.unwrap(np.arctan2(d[good, 1], d[good, 0]))
        tau = (ts[:-1] + ts[1:])[good] / 2.0
        w = seg_len[good]
        A = np.stack([np.ones_like(tau), tau], axis=1) * np.sqrt(w)[:, None]
        coef, *_ = np.linalg.lstsq(A, h * np.sqrt(w), rcond=None)
        yaw_rate = float(np.clip(coef[1], -YAW_RATE_MAX, YAW_RATE_MAX))
        yaw = float(coef[0] + coef[1] * ts[-1])
        # the repo's consistency guard (_predict_future_xz): a turn is only believed when the
        # segment yaw rates agree and are not tiny; otherwise straight ahead.
        rates = np.diff(h) / np.diff(tau)
        if rates.size < 3 or np.std(rates) > abs(np.mean(rates)) * 1.8 or abs(np.mean(rates)) < 0.15:
            yaw_rate = 0.0
    return propagate_ctrv(pts[-1], v, yaw, yaw_rate)


def cv_last2(pts, ts):
    if len(pts) < 2:
        return np.repeat(pts[-1:], PRED_LEN, axis=0)
    v = (pts[-1] - pts[-2]) / (ts[-1] - ts[-2])
    return pts[-1][None, :] + v[None, :] * STEPS


# ----------------------------------------------------------------------------------------------
# Data, masks, imputation
# ----------------------------------------------------------------------------------------------
def load_scene(scene):
    from utils.dataloader import TrajectoryDataset
    ds = TrajectoryDataset(os.path.join(ET_DIR, 'datasets', scene, 'test') + '/',
                           obs_len=OBS_LEN, pred_len=PRED_LEN)
    obs = ds.obs_traj.numpy().astype(np.float64)      # N 8 2
    gt = ds.pred_traj.numpy().astype(np.float64)      # N 12 2
    return obs, gt, list(ds.seq_start_end)


def make_masks(n, rate, rng):
    """[n, 8] bool, True = observed. k = round(8 * rate) of indices 0..6 dropped per trajectory."""
    k = int(round(OBS_LEN * rate))
    mask = np.ones((n, OBS_LEN), dtype=bool)
    if k:
        for i in range(n):
            mask[i, rng.choice(OBS_LEN - 1, size=k, replace=False)] = False
    return mask, k


def impute(obs, mask, how):
    """obs [n, 8, 2], mask [n, 8] -> [n, 8, 2] with the dropped steps filled."""
    out = obs.copy()
    idx = np.arange(OBS_LEN)
    for i in range(obs.shape[0]):
        m = mask[i]
        if m.all():
            continue
        seen = idx[m]
        if how == 'interp':
            for c in range(2):
                out[i, :, c] = np.interp(idx, seen, obs[i, seen, c])
        elif how == 'ffill':
            last = np.maximum.accumulate(np.where(m, idx, -1))
            last[last < 0] = seen[0]                      # leading gap: back-fill
            out[i] = obs[i, last]
        else:
            raise ValueError(how)
    return out


# ----------------------------------------------------------------------------------------------
# Scoring
# ----------------------------------------------------------------------------------------------
def ade_fde(pred, gt):
    """pred [S, n, 12, 2] or [n, 12, 2]; gt [n, 12, 2] -> (ade[n], fde[n]) best-of-S."""
    if pred.ndim == 3:
        pred = pred[None]
    err = np.linalg.norm(pred - gt[None], axis=-1)       # S n 12
    return err.mean(axis=2).min(axis=0), err[:, :, -1].min(axis=0)


def point_estimates(samples, window):
    """samples [S, n, 12, 2] in the model's raw order, window [n, 8, 2] (imputed) -> dict."""
    S, n = samples.shape[:2]
    ar = np.arange(n)
    final = samples[:, :, -1, :]
    d_end = np.linalg.norm(final - final.mean(axis=0, keepdims=True), axis=-1)
    v = (window[:, -1] - window[:, -3]) / 2.0
    cv = window[:, -1:, :] + v[:, None, :] * np.arange(1, PRED_LEN + 1)[None, :, None]
    d_cv = np.linalg.norm(samples - cv[None], axis=-1).mean(axis=-1)
    return {'raw0': samples[0],
            'medoid': samples[d_end.argmin(axis=0), ar],
            'mean': samples.mean(axis=0),
            'nearcv': samples[d_cv.argmin(axis=0), ar]}


class Rows:
    def __init__(self):
        self.rows, self.per_traj = [], {}

    def add(self, base, a, f, **kw):
        r = dict(base, ctrv='-', imputation='-', point='-', alpha='-', samples=1,
                 fallback_frac=0.0, ade=float(a.mean()), fde=float(f.mean()))
        r.update(kw)
        self.rows.append(r)
        key = '/'.join(str(r[k]) for k in ('scene', 'missing_rate', 'predictor', 'ctrv',
                                            'imputation', 'point', 'alpha', 'samples'))
        self.per_traj[key] = np.stack([a, f]).astype(np.float32)
        return r


def run_scene(scene, args, ctrv_repo, alpha0, device, log, R):
    import torch
    import detection_pipeline.EigenTrajectory.eigen as shim

    obs_all, gt_all, seqs = load_scene(scene)
    if args.max_seqs and len(seqs) > args.max_seqs:
        rng = np.random.default_rng([args.seed, SCENES.index(scene), 999])
        keep = sorted(rng.choice(len(seqs), size=args.max_seqs, replace=False).tolist())
        seqs = [seqs[i] for i in keep]
        sel = np.concatenate([np.arange(s, e) for s, e in seqs])
        obs_all, gt_all = obs_all[sel], gt_all[sel]
        lens = [e - s for s, e in seqs]
        cum = np.concatenate([[0], np.cumsum(lens)])
        seqs = [(int(cum[i]), int(cum[i + 1])) for i in range(len(lens))]
    n = obs_all.shape[0]
    log(f'[{scene}] {n} trajectories in {len(seqs)} sequences')

    model, _hp = shim.load_predictor(
        os.path.join(ET_DIR, 'config', f'eigentrajectory-stgcnn-{scene}.json'),
        os.path.join(ET_DIR, 'checkpoints', 'EigenTrajectory-STGCNN', scene, 'model_best.pth'),
        device=device)
    S = model.num_samples

    t_obs = np.arange(OBS_LEN) * DT
    hold = np.repeat(obs_all[:, -1:, :], PRED_LEN, axis=1)
    for rate in RATES:
        rng = np.random.default_rng([args.seed, SCENES.index(scene), int(round(rate * 100))])
        mask, k = make_masks(n, rate, rng)
        n_seen = mask.sum(axis=1)
        base = dict(scene=scene, missing_rate=rate, k_dropped=k, n_traj=n)

        # --- physics arms on the observed steps with real stamps
        repo_pred = np.full((n, PRED_LEN, 2), np.nan)
        repo_ok = np.zeros(n, dtype=bool)
        fit_pred = np.zeros((n, PRED_LEN, 2))
        cv_pred = np.zeros((n, PRED_LEN, 2))
        for i in range(n):
            m = mask[i]
            pts, ts = obs_all[i, m], t_obs[m]
            p = ctrv_repo([(x, y, t) for (x, y), t in zip(pts, ts)])
            if p is not None:
                repo_pred[i], repo_ok[i] = p, True
            fit_pred[i] = ctrv_fit(pts, ts)
            cv_pred[i] = cv_last2(pts, ts)
        repo_scored = np.where(repo_ok[:, None, None], repo_pred, hold)
        fb = float(1.0 - repo_ok.mean())

        R.add(base, *ade_fde(hold, gt_all), predictor='HOLD')
        R.add(base, *ade_fde(cv_pred, gt_all), predictor='CV')
        r_repo = R.add(base, *ade_fde(repo_scored, gt_all), predictor='CTRV-repo', fallback_frac=fb)
        r_fit = R.add(base, *ade_fde(fit_pred, gt_all), predictor='CTRV-fit')
        ctrvs = {'repo': (repo_pred, repo_ok), 'fit': (fit_pred, np.ones(n, dtype=bool))}

        # --- EigenTrajectory per imputation, whole sequences at once (it is a social model)
        for how in IMPUTATIONS:
            x = impute(obs_all, mask, how)
            eig = np.zeros((S, n, PRED_LEN, 2))
            with torch.no_grad():
                for s, e in seqs:
                    xt = torch.tensor(x[s:e], dtype=torch.float32, device=device)
                    eig[:, s:e] = model(xt, reorder=False).cpu().numpy()
            bad = ~np.isfinite(eig).all(axis=(0, 2, 3))
            if bad.any():
                log(f'  [{scene} r={rate} {how}] {bad.sum()} non-finite Eigen outputs -> hold-last')
                eig[:, bad] = hold[bad][None]
            pts_est = point_estimates(eig, x)
            for rule in POINT_RULES:
                R.add(base, *ade_fde(pts_est[rule], gt_all), predictor='EIGEN', imputation=how,
                      point=rule)
            R.add(base, *ade_fde(eig, gt_all), predictor='EIGEN', imputation=how, point='bo',
                  samples=S)

            # --- fusion with each CTRV: CTRV empty -> Eigen alone (as _fuse_future_predictions)
            e0 = pts_est[FLOWN_RULE]
            for cname, (cpred, cok) in ctrvs.items():
                def fuse(alpha_vec):
                    al = np.asarray(alpha_vec, dtype=np.float64).reshape(n, 1, 1)
                    out = (1.0 - al) * cpred + al * e0
                    return np.where(cok[:, None, None], out, e0)

                for tag, avec in (('const', np.full(n, alpha0)), ('sched', alpha0 * n_seen / OBS_LEN)):
                    R.add(base, *ade_fde(fuse(avec), gt_all), predictor='FUSED', ctrv=cname,
                          imputation=how, point=FLOWN_RULE, alpha=tag,
                          fallback_frac=fb if cname == 'repo' else 0.0)
                for al in ALPHA_SWEEP:
                    R.add(base, *ade_fde(fuse(np.full(n, al)), gt_all), predictor='FUSED', ctrv=cname,
                          imputation=how, point=FLOWN_RULE, alpha=f'{al:.1f}',
                          fallback_frac=fb if cname == 'repo' else 0.0)
        log(f'  r={rate:.1f} k={k}: CTRV-repo {r_repo["ade"]:.3f}/{r_repo["fde"]:.3f} '
            f'(fallback {fb * 100:.1f} %)  CTRV-fit {r_fit["ade"]:.3f}/{r_fit["fde"]:.3f}')


# ----------------------------------------------------------------------------------------------
# Tables
# ----------------------------------------------------------------------------------------------
KEY = ('missing_rate', 'predictor', 'ctrv', 'imputation', 'point', 'alpha', 'samples')


def add_avg_rows(rows, scenes):
    groups = {}
    for r in rows:
        groups.setdefault(tuple(r[k] for k in KEY), []).append(r)
    out = []
    for key, rs in groups.items():
        if len(rs) != len(scenes):
            continue
        avg = dict(rs[0])
        avg.update(scene='AVG', n_traj=sum(r['n_traj'] for r in rs),
                   fallback_frac=float(np.mean([r['fallback_frac'] for r in rs])),
                   ade=float(np.mean([r['ade'] for r in rs])),
                   fde=float(np.mean([r['fde'] for r in rs])))
        out.append(avg)
    return rows + out


def lookup(rows, scene, rate, predictor, ctrv='-', imputation='-', point='-', alpha='-', samples=1):
    want = (scene, rate, predictor, ctrv, imputation, point, str(alpha), samples)
    for r in rows:
        if (r['scene'], r['missing_rate'], r['predictor'], r['ctrv'], r['imputation'], r['point'],
                str(r['alpha']), r['samples']) == want:
            return r
    return None


def fmt(r):
    return '--' if r is None else f'{r["ade"]:.3f} / {r["fde"]:.3f}'


def render_markdown(rows, meta):
    scenes = list(meta['scenes']) + (['AVG'] if len(meta['scenes']) == len(SCENES) else [])
    S, how = meta['num_samples'], meta['primary_imputation']
    L = []
    L.append(f'ADE / FDE in metres, best-of-1 unless marked bo{S}. Eigen imputation **{how}** '
             f'(the repo\'s HELD-frame behaviour), Eigen point rule **{FLOWN_RULE}** (what the adapter flies). '
             f'{meta["n_traj_total"]} trajectories, seed {meta["seed"]}, {meta["elapsed_s"] / 60:.1f} min on '
             f'{meta["device"]}' + (' -- **SUBSAMPLED, --max-seqs %d**' % meta['max_seqs'] if meta['subsampled'] else '') + '.')
    L.append('')
    L.append('## Table II candidate: per scene and missing rate')
    for rate in RATES:
        k = int(round(OBS_LEN * rate))
        L.append('')
        L.append(f'### Missing {rate * 100:.0f} % (k = {k} of 7 non-final steps dropped)')
        L.append('')
        L.append(f'| Scene | HOLD | CV | CTRV-repo (fallback %) | CTRV-fit | Eigen bo1 | Eigen bo{S} | Fused-fit a=0.2 | Fused-fit a sched |')
        L.append('|---|---|---|---|---|---|---|---|---|')
        for sc in scenes:
            c = lookup(rows, sc, rate, 'CTRV-repo')
            fb = '' if c is None else f' ({c["fallback_frac"] * 100:.0f})'
            L.append('| {} | {} | {} | {}{} | {} | {} | {} | {} | {} |'.format(
                sc, fmt(lookup(rows, sc, rate, 'HOLD')), fmt(lookup(rows, sc, rate, 'CV')), fmt(c), fb,
                fmt(lookup(rows, sc, rate, 'CTRV-fit')),
                fmt(lookup(rows, sc, rate, 'EIGEN', '-', how, FLOWN_RULE)),
                fmt(lookup(rows, sc, rate, 'EIGEN', '-', how, 'bo', '-', S)),
                fmt(lookup(rows, sc, rate, 'FUSED', 'fit', how, FLOWN_RULE, 'const')),
                fmt(lookup(rows, sc, rate, 'FUSED', 'fit', how, FLOWN_RULE, 'sched'))))
    if 'AVG' not in scenes:
        return '\n'.join(L)
    L.append('')
    L.append('## Eigen point rule and imputation (AVG over scenes)')
    L.append('')
    L.append(f'| Missing | Imputation | raw0 (public preds[0]) | medoid | mean | nearcv (flown) | bo{S} (oracle) |')
    L.append('|---|---|---|---|---|---|---|')
    for rate in RATES:
        for h in IMPUTATIONS:
            L.append('| {:.0f} % | {} | {} | {} | {} | {} | {} |'.format(
                rate * 100, h, *[fmt(lookup(rows, 'AVG', rate, 'EIGEN', '-', h, p)) for p in POINT_RULES],
                fmt(lookup(rows, 'AVG', rate, 'EIGEN', '-', h, 'bo', '-', S))))
    L.append('')
    L.append('## Fusion (AVG over scenes, Eigen nearcv)')
    L.append('')
    L.append('| Missing | CTRV-repo | CTRV-fit | Fused-repo ffill a=0.2 | Fused-repo ffill sched | Fused-fit ffill a=0.2 | Fused-fit ffill sched | Fused-fit interp a=0.2 | Fused-fit interp sched |')
    L.append('|---|---|---|---|---|---|---|---|---|')
    for rate in RATES:
        L.append('| {:.0f} % | {} | {} | {} | {} | {} | {} | {} | {} |'.format(
            rate * 100, fmt(lookup(rows, 'AVG', rate, 'CTRV-repo')), fmt(lookup(rows, 'AVG', rate, 'CTRV-fit')),
            fmt(lookup(rows, 'AVG', rate, 'FUSED', 'repo', 'ffill', FLOWN_RULE, 'const')),
            fmt(lookup(rows, 'AVG', rate, 'FUSED', 'repo', 'ffill', FLOWN_RULE, 'sched')),
            fmt(lookup(rows, 'AVG', rate, 'FUSED', 'fit', 'ffill', FLOWN_RULE, 'const')),
            fmt(lookup(rows, 'AVG', rate, 'FUSED', 'fit', 'ffill', FLOWN_RULE, 'sched')),
            fmt(lookup(rows, 'AVG', rate, 'FUSED', 'fit', 'interp', FLOWN_RULE, 'const')),
            fmt(lookup(rows, 'AVG', rate, 'FUSED', 'fit', 'interp', FLOWN_RULE, 'sched'))))
    L.append('')
    L.append(f'## Constant-alpha sweep (AVG over scenes, CTRV-fit, {how}, Eigen nearcv; a=0 is CTRV, a=1 is Eigen)')
    L.append('')
    L.append('| Missing | ' + ' | '.join(f'a={al:.1f}' for al in ALPHA_SWEEP) + ' |')
    L.append('|---|' + '---|' * len(ALPHA_SWEEP))
    for rate in RATES:
        L.append(f'| {rate * 100:.0f} % | ' + ' | '.join(
            fmt(lookup(rows, 'AVG', rate, 'FUSED', 'fit', how, FLOWN_RULE, f'{al:.1f}')) for al in ALPHA_SWEEP) + ' |')
    L.append('')
    return '\n'.join(L)


# ----------------------------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n', 1)[0])
    ap.add_argument('--out', default=os.path.expanduser('~/dtp_datasets/m66_predictor_study'))
    ap.add_argument('--scenes', nargs='+', default=SCENES, choices=SCENES)
    ap.add_argument('--max-seqs', type=int, default=0,
                    help='subsample this many sequences per scene (0 = all)')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--device', default=None, help='cuda (default if available) or cpu')
    args = ap.parse_args()

    import torch
    device = args.device or ('cuda' if torch.cuda.is_available() else 'cpu')
    os.makedirs(args.out, exist_ok=True)
    log_f = open(os.path.join(args.out, 'run.log'), 'w')

    def log(msg):
        print(msg, flush=True)
        log_f.write(msg + '\n')
        log_f.flush()

    t0 = time.time()
    log(f'M6.6 predictor study: scenes {args.scenes}, rates {RATES}, seed {args.seed}, device {device}, '
        f'max_seqs {args.max_seqs or "all"}, torch {torch.__version__}')
    ctrv_repo, alpha0 = make_ctrv_repo()
    log(f'CTRV-repo = detection.HumanXZPredictor._predict_future_xz; eigen_fusion_alpha = {alpha0}')

    R = Rows()
    for scene in args.scenes:
        run_scene(scene, args, ctrv_repo, alpha0, device, log, R)
    rows = add_avg_rows(R.rows, args.scenes)
    elapsed = time.time() - t0
    n_total = sum(r['n_traj'] for r in R.rows if r['predictor'] == 'HOLD' and r['missing_rate'] == 0.0)
    S = max(r['samples'] for r in rows)

    meta = dict(scenes=args.scenes, rates=RATES, seed=args.seed, device=device,
                torch=torch.__version__, elapsed_s=elapsed, n_traj_total=n_total,
                num_samples=S, primary_imputation='ffill', flown_point_rule=FLOWN_RULE,
                alpha0=alpha0, alpha_schedule='alpha0 * n_observed / 8',
                obs_len=OBS_LEN, pred_len=PRED_LEN, dt=DT, max_seqs=args.max_seqs,
                subsampled=bool(args.max_seqs),
                checkpoints='detection_pipeline/EigenTrajectory/checkpoints/EigenTrajectory-STGCNN/<scene>/model_best.pth (public release v1.0)',
                date=time.strftime('%Y-%m-%d %H:%M:%S'))

    fields = ['scene', 'missing_rate', 'k_dropped', 'predictor', 'ctrv', 'imputation', 'point',
              'alpha', 'samples', 'n_traj', 'ade', 'fde', 'fallback_frac']
    with open(os.path.join(args.out, 'results.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow({k: (f'{r[k]:.6f}' if isinstance(r[k], float) else r[k]) for k in fields})
    np.savez_compressed(os.path.join(args.out, 'per_traj.npz'), **R.per_traj)
    with open(os.path.join(args.out, 'meta.json'), 'w') as f:
        json.dump(meta, f, indent=2)
    table = render_markdown(rows, meta)
    with open(os.path.join(args.out, 'table.md'), 'w') as f:
        f.write(f'# M6.6 predictor study -- {meta["date"]}\n\n' + table)
    log('')
    log(table)
    log(f'done in {elapsed / 60:.1f} min -> {args.out}' +
        ('  (SUBSAMPLED: --max-seqs %d per scene)' % args.max_seqs if args.max_seqs else ''))


if __name__ == '__main__':
    main()
