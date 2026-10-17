"""M6.6 addendum: the AGX deploy's EigenTrajectoryInference vs this repo's shim.

Same public ET-STGCNN weights (the AGX copies are byte-identical, see
docs/m6.6-predictor-study.md SS8), same ETH/UCY test data with complete histories, same scoring as
m66_predictor_study.py. Trajectories where the AGX wrapper returns NaN are counted, then dropped
from BOTH arms so the table compares like with like.

    tar -xJf drone-dtp/weights/EigenTrajectory.tar.xz -C /some/dir
    source ~/super_ws/src/drone-dtp/ros2_ws/dtp_env_jetson.sh
    python3 drone-dtp/scripts/m66_agx_wrapper_check.py --agx-dir /some/dir/EigenTrajectory
"""
import argparse
import importlib.util
import os
import sys
import time

import numpy as np
import torch

DTP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ET = DTP + '/detection_pipeline/EigenTrajectory'
ap = argparse.ArgumentParser()
ap.add_argument('--agx-dir', required=True, help='the extracted AGX EigenTrajectory/ folder')
AGX = os.path.abspath(ap.parse_args().agx_dir)
for p in (ET, DTP, DTP + '/scripts'):
    sys.path.insert(0, p)

import m66_predictor_study as st                                 # noqa: E402
import detection_pipeline.EigenTrajectory.eigen as shim          # noqa: E402
from utils.utils import get_exp_config                           # noqa: E402

spec = importlib.util.spec_from_file_location('agx_eigen', AGX + '/eigen.py')
agx = importlib.util.module_from_spec(spec)
spec.loader.exec_module(agx)

dev = 'cuda' if torch.cuda.is_available() else 'cpu'
print('device', dev, flush=True)
T_OBS = np.arange(st.OBS_LEN) * st.DT
out = {}
for scene in st.SCENES:
    t0 = time.time()
    obs, gt, seqs = st.load_scene(scene)
    n = obs.shape[0]
    cfg = f'{ET}/config/eigentrajectory-stgcnn-{scene}.json'
    ckpt = f'{ET}/checkpoints/EigenTrajectory-STGCNN/{scene}/model_best.pth'
    m_shim, hp = shim.load_predictor(cfg, ckpt, device=dev)
    m_agx = agx.EigenTrajectoryInference(get_exp_config(cfg)).to(dev)
    sd = torch.load(ckpt, map_location=dev, weights_only=False)
    m_agx.load_state_dict(sd.get('model_state_dict', sd))          # strict
    m_agx.eval()

    S = m_shim.num_samples
    e_shim = np.zeros((S, n, st.PRED_LEN, 2))
    e_agx = np.zeros((S, n, st.PRED_LEN, 2))
    with torch.no_grad():
        for s, e in seqs:
            xt = torch.tensor(obs[s:e], dtype=torch.float32, device=dev)
            e_shim[:, s:e] = m_shim(xt, reorder=False).cpu().numpy()
            e_agx[:, s:e] = m_agx(xt).cpu().numpy()
    cv = np.stack([st.cv_last2(obs[i], T_OBS) for i in range(n)])
    moving = np.linalg.norm((obs[:, -1] - obs[:, -3]) / 2.0, axis=-1) > float(hp.static_dist)

    fin = np.isfinite(e_agx).all(axis=(0, 2, 3))
    disp = np.linalg.norm(obs[:, -1] - obs[:, 0], axis=-1)
    seq_nan = sum(1 for s, e in seqs if not fin[s:e].all())
    print(f'  [{scene}] AGX non-finite: {(~fin).sum()}/{n} traj in {seq_nan}/{len(seqs)} sequences; '
          f'obs displacement <1 mm: {(disp < 1e-3).sum()}; seqs w/ NaN that contain a <1 mm traj: '
          f'{sum(1 for s, e in seqs if not fin[s:e].all() and (disp[s:e] < 1e-3).any())}', flush=True)
    obs, gt, cv, e_shim, e_agx, moving = obs[fin], gt[fin], cv[fin], e_shim[:, fin], e_agx[:, fin], moving[fin]
    res = {'CV': st.ade_fde(cv, gt)}
    for name, eig in (('shim', e_shim), ('agx', e_agx)):
        pe = st.point_estimates(eig, obs)
        res[f'{name} raw0'] = st.ade_fde(pe['raw0'], gt)
        res[f'{name} nearcv'] = st.ade_fde(pe['nearcv'], gt)
        res[f'{name} bo20'] = st.ade_fde(eig, gt)
    out[scene] = (res, moving)
    print(f'[{scene}] n={n} moving={moving.mean():.2f} {time.time() - t0:.1f}s', flush=True)

arms = list(out['eth'][0].keys())
print('\n| arm | ' + ' | '.join(st.SCENES) + ' | AVG | AVG moving-only | AVG static-only |')
print('|---|' + '---|' * (len(st.SCENES) + 3))
for arm in arms:
    cells, avg, mov, sta = [], [], [], []
    for sc in st.SCENES:
        (a, f), moving = out[sc][0][arm], out[sc][1]
        cells.append(f'{a.mean():.3f} / {f.mean():.3f}')
        avg.append((a.mean(), f.mean()))
        if moving.any():
            mov.append((a[moving].mean(), f[moving].mean()))
        if (~moving).any():
            sta.append((a[~moving].mean(), f[~moving].mean()))
    fmt = lambda v: f'{np.mean([x[0] for x in v]):.3f} / {np.mean([x[1] for x in v]):.3f}'
    print(f'| {arm} | ' + ' | '.join(cells) + f' | {fmt(avg)} | {fmt(mov)} | {fmt(sta)} |')
