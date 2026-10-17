"""EigenTrajectoryInference -- the class detection_pipeline/eigen.py's adapter expects.

The flown learned branch was an out-of-band package with this exact entry point; the package,
its config and its checkpoint never reached this box (CLAUDE.md, "The learned predictor is
disabled and its weights are outside git"). This file rebuilds that entry point on top of the
PUBLIC EigenTrajectory release (InhwanBae/EigenTrajectory, ICCV 2023), cloned into this
directory, with its pretrained ET-STGCNN ETH/UCY checkpoints. It is what
scripts/m66_predictor_study.py evaluates. Nothing in flight loads it: eigen_enabled defaults
False and dtp_params.yaml does not turn it on.

Interface (from EigenTrajectoryAdapter.setup / maybe_predict):

    predictor = EigenTrajectoryInference(hyper_params).to(device)
    predictor.load_state_dict(torch.load(ckpt)["model_state_dict" or the dict itself])
    predictor.eval()
    preds = predictor(obs)            # obs: float32 [N, obs_len, 2]
    best  = preds[0]                  # [N, pred_len, 2]  <- the adapter flies sample 0

so forward() must return a tensor [S, N, pred_len, 2] whose index 0 is the single-shot
prediction. The public model emits S = num_samples (20) anchor-refined samples with NO
ordering and no likelihood -- their final points are spread 2-3 m apart on ETH/UCY (measured
2026-09-14), so which one sits at index 0 decides everything a single-goal follower sees.
We put first the sample NEAREST THE CONSTANT-VELOCITY CONTINUATION of the observation
window (velocity = (obs[-1] - obs[-3]) / 2 per step, the same displacement the model's own
normaliser keys on), and keep the other S-1 in their original order. Measured on the full
test scenes (scripts/m66_predictor_study.py, table "point rule"): raw sample 0, the
endpoint medoid and the sample mean are all far worse than plain CV; nearest-to-CV is at CV
level. Index 0 is a real model sample, deterministic, needs nothing but the window, and
"best-of-1" in the study means exactly what the adapter would fly; best-of-S is
order-independent. forward(..., reorder=False) returns the model's raw order.

hyper_params may be the repo's DotDict, a SimpleNamespace (the adapter's fallback when
utils cannot be imported) or a plain dict. Only the stgcnn baseline is wired; the other nine
would need their trainer's constructor arguments copied from utils/trainer.py.
"""
from __future__ import annotations

import os
import sys

import torch

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from EigenTrajectory.model import EigenTrajectory          # noqa: E402  (the public package)
from utils.utils import DotDict                           # noqa: E402


def _as_dotdict(hp):
    if isinstance(hp, DotDict):
        return hp
    if isinstance(hp, dict):
        return DotDict(hp)
    return DotDict(vars(hp))


def _stgcnn_parts(hp):
    """The Social-STGCNN baseline exactly as utils/trainer.py's ETSTGCNNTrainer builds it."""
    from baseline.stgcnn import (TrajectoryPredictor, model_forward_pre_hook, model_forward,
                                 model_forward_post_hook)
    predictor = TrajectoryPredictor(n_stgcnn=1, n_txpcnn=5, input_feat=1, output_feat=hp.num_samples,
                                    kernel_size=3, seq_len=hp.k + 2, pred_seq_len=hp.k)
    hooks = DotDict({"model_forward_pre_hook": model_forward_pre_hook,
                     "model_forward": model_forward,
                     "model_forward_post_hook": model_forward_post_hook})
    return predictor, hooks


_BUILDERS = {"stgcnn": _stgcnn_parts}


class EigenTrajectoryInference(EigenTrajectory):
    """EigenTrajectory (public) + its baseline, with a tensor-returning forward.

    Subclassing keeps the state_dict key layout (baseline_model.*, ET_m_descriptor.*, ...)
    identical to the released model_best.pth files, so load_state_dict is strict and direct.
    """

    def __init__(self, hyper_params):
        hp = _as_dotdict(hyper_params)
        name = str(hp.baseline or "stgcnn").lower()
        if name not in _BUILDERS:
            raise ValueError(f"baseline {name!r} not wired in {__file__}; have {sorted(_BUILDERS)}")
        predictor, hooks = _BUILDERS[name](hp)
        super().__init__(baseline_model=predictor, hook_func=hooks, hyper_params=hp)
        self.obs_len = int(hp.obs_len)
        self.pred_len = int(hp.pred_len)
        self.num_samples = int(hp.num_samples)

    @staticmethod
    def cv_nearest_first(samples, obs_traj):
        """Reorder [S, N, T, 2] so index 0 is, per pedestrian, the sample nearest (mean L2 over
        the T steps) to the constant-velocity continuation of obs_traj [N, obs, 2]; the rest keep
        their order. With S == 1 nothing changes."""
        S, N, T, _ = samples.shape
        if S == 1:
            return samples
        v = (obs_traj[:, -1] - obs_traj[:, -3]) / 2.0                          # N 2 per step
        steps = torch.arange(1, T + 1, device=samples.device, dtype=samples.dtype)
        cv = obs_traj[:, -1:, :] + v[:, None, :] * steps[None, :, None]     # N T 2
        d = (samples - cv[None]).norm(dim=-1).mean(dim=-1)                  # S N
        idx = d.argmin(dim=0)                                               # N
        order = torch.arange(S, device=samples.device)[None, :].repeat(N, 1)
        for n in range(N):
            i = int(idx[n])
            order[n] = torch.cat([order[n, i:i + 1], order[n, :i], order[n, i + 1:]])
        out = samples.permute(1, 0, 2, 3)                                   # N S T 2
        out = torch.gather(out, 1, order[:, :, None, None].expand_as(out))
        return out.permute(1, 0, 2, 3).contiguous()

    def forward(self, obs_traj, pred_traj=None, addl_info=None, reorder=True):
        obs_traj = torch.as_tensor(obs_traj, dtype=torch.float32)
        if obs_traj.dim() == 2:
            obs_traj = obs_traj[None]
        if obs_traj.size(1) != self.obs_len:
            raise ValueError(f"expected obs_len {self.obs_len}, got {obs_traj.size(1)}")
        out = super().forward(obs_traj, pred_traj=None, addl_info=addl_info)
        samples = out["recon_traj"]                                    # S N pred_len 2
        return self.cv_nearest_first(samples, obs_traj) if reorder else samples


def load_predictor(config_path, checkpoint_path, device="cpu"):
    """Convenience for offline use: config JSON + model_best.pth -> eval-mode module."""
    from utils.utils import get_exp_config
    hp = get_exp_config(config_path)
    model = EigenTrajectoryInference(hp).to(device)
    sd = torch.load(checkpoint_path, map_location=device, weights_only=False)
    model.load_state_dict(sd.get("model_state_dict", sd))
    model.eval()
    return model, hp
