from __future__ import annotations

from collections import defaultdict, deque
from typing import Dict, List, Optional, Tuple
import os
import sys
import time
import numpy as np

try:
    import torch
except ImportError:
    torch = None

_EIGEN_DIR = os.path.join(os.path.dirname(__file__), 'EigenTrajectory')
if os.path.isdir(_EIGEN_DIR) and _EIGEN_DIR not in sys.path:
    sys.path.insert(0, _EIGEN_DIR)

try:
    from detection_pipeline.EigenTrajectory.eigen import EigenTrajectoryInference
except (ImportError, SystemExit):   # the package prints a banner + exits on failure
    EigenTrajectoryInference = None


# Default on-disk locations of the (out-of-band) EigenTrajectory assets.
DEFAULT_EIGEN_CONFIG = 'detection_pipeline/EigenTrajectory/config/eigentrajectory-stgcnn-eth.json'
DEFAULT_EIGEN_CHECKPOINT = 'detection_pipeline/EigenTrajectory/checkpoints/model_best.pth'


def eigen_predictor_kwargs(enable, config_path=DEFAULT_EIGEN_CONFIG,
                           checkpoint_path=DEFAULT_EIGEN_CHECKPOINT, device='cuda'):
    """HumanXZPredictor(**kwargs) to enable EigenTrajectory, or {} if it can't.

    The model package/config/checkpoint live outside git (fetch from the AGX deploy). If
    `enable` is False or any piece is missing, return {} (with a warning) so the caller
    silently keeps CTRV -- requesting Eigen is always safe.
    """
    if not enable:
        return {}
    have_files = os.path.isfile(config_path) and os.path.isfile(checkpoint_path)
    if not (have_files and EigenTrajectoryInference is not None):
        print('[EIGEN] assets missing (need the EigenTrajectory package + '
              f'{config_path} + {checkpoint_path}); falling back to CTRV.')
        return {}
    try:
        from utils.utils import get_exp_config
        hyper_params = get_exp_config(config_path)
    except ImportError:
        import json
        from types import SimpleNamespace
        with open(config_path) as f:
            hyper_params = SimpleNamespace(**json.load(f))
    print(f'[EIGEN] enabled on {device}')
    return dict(eigen_enabled=True, eigen_hyper_params=hyper_params,
                eigen_checkpoint_path=checkpoint_path, eigen_device=device,
                eigen_infer_rate_hz=10.0)


class EigenTrajectoryAdapter:
    def __init__(self):
        self._enabled = False
        self._device = "cpu"
        self._predictor = None

        self._obs_len = 8
        self._pred_len = 12
        self._num_samples = 1

        self._track_history: Dict[int, deque] = defaultdict(lambda: deque(maxlen=self._obs_len))
        self._last_predictions: Dict[int, np.ndarray] = {}
        self._last_infer_time_ns: int = 0
        self._infer_period_ns: int = int(1e9 / 10.0)  # 10 Hz

    @property
    def enabled(self) -> bool:
        return self._enabled

    def setup(
        self,
        hyper_params,
        checkpoint_path: Optional[str] = None,
        device: str = "cpu",
        obs_len: int = 8,
        pred_len: int = 12,
        infer_rate_hz: float = 10.0,
    ) -> None:
        if torch is None:
            raise ImportError("PyTorch is required for EigenTrajectoryAdapter.")

        if EigenTrajectoryInference is None:
            raise ImportError(
                "import error"
                )

        self._obs_len = obs_len
        self._pred_len = pred_len
        self._infer_period_ns = int(1e9 / infer_rate_hz)
        self._device = device

        self._predictor = EigenTrajectoryInference(hyper_params).to(device)

        if checkpoint_path is not None:
            checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)
            state_dict = checkpoint.get("model_state_dict", checkpoint)
            self._predictor.load_state_dict(state_dict)

        self._predictor.eval()
        self._enabled = True

    def update_history(
        self,
        track_id: int,
        position_xz: np.ndarray,
    ) -> None:
        self._track_history[track_id].append(position_xz.astype(np.float32))

    def maybe_predict(
        self,
        timestamp_ns: Optional[int] = None,
    ) -> Dict[int, np.ndarray]:
        if not self._enabled or self._predictor is None:
            return {}

        if timestamp_ns is None:
            timestamp_ns = time.time_ns()

        if (timestamp_ns - self._last_infer_time_ns) < self._infer_period_ns:
            return self._last_predictions

        eligible_ids: List[int] = []
        batch_obs: List[np.ndarray] = []

        for track_id, hist in self._track_history.items():
            if len(hist) == self._obs_len:
                eligible_ids.append(track_id)
                batch_obs.append(np.stack(hist, axis=0))  # [8,2]

        if len(batch_obs) == 0:
            return self._last_predictions

        obs_arr = np.stack(batch_obs, axis=0)  

        with torch.no_grad():
            obs_tensor = torch.tensor(obs_arr, dtype=torch.float32, device=self._device)
            preds = self._predictor(obs_tensor)  
            best_pred = preds[0].detach().cpu().numpy()  

        current_preds: Dict[int, np.ndarray] = {}
        for i, track_id in enumerate(eligible_ids):
            current_preds[track_id] = best_pred[i]

        self._last_predictions = current_preds
        self._last_infer_time_ns = timestamp_ns
        return self._last_predictions

    def get_prediction_for_track(self, track_id: int) -> Optional[np.ndarray]:
        return self._last_predictions.get(track_id, None)

    def reset(self) -> None:
        self._track_history.clear()
        self._last_predictions.clear()
        self._last_infer_time_ns = 0
    
    def remove_track(self, track_id: int) -> None:
        """Drop one track's state when it goes stale (mirrors detection._reset_track)."""
        self._track_history.pop(track_id, None)
        self._last_predictions.pop(track_id, None)