"""Project the LiDAR's walker points into the colour image through the bag's own TF chain and compare
with the detector's box: the vertical offset is the camera<->LiDAR pitch error, measured on this bag."""
import pickle, math
import numpy as np
from scipy.spatial.transform import Rotation as R
D = pickle.load(open('bag1455.pkl', 'rb')); t0 = D['t0']
rows = pickle.load(open('rows1455.pkl', 'rb'))
L = np.load('lidar1455.npz'); P, T, dyn = L['P'], L['T'], L['dyn']; Pd, Td = P[dyn], T[dyn]
tf = {(p, c): (np.array(t), np.array(q)) for p, c, t, q in D['tfs']}
def M(tq):
    X = np.eye(4); X[:3, :3] = R.from_quat(tq[1]).as_matrix(); X[:3, 3] = tq[0]; return X
_, tl, ql = D['tf_dyn'][len(D['tf_dyn']) // 2]
W = M((np.array(tl), np.array(ql)))
for pc in [('lidar_imu', 'camera_link'), ('camera_link', 'camera_depth_frame'), ('camera_depth_frame', 'camera_color_frame'), ('camera_color_frame', 'camera_color_optical_frame')]:
    W = W @ M(tf[pc])
Wi = np.linalg.inv(W)
K = np.array(D['ci']['K']).reshape(3, 3); fy = K[1, 1]; print('colour K fx %.1f fy %.1f cx %.1f cy %.1f' % (K[0, 0], K[1, 1], K[0, 2], K[1, 2]), D['ci']['model'], np.round(D['ci']['D'][:5], 3))
CAM = W[:3, 3]
out = []
for w in rows:
    tr = w['tru']
    if not tr or tr['r'] < 3.5: continue
    ts = w['t'] + t0
    b = math.atan2(w['y'] - CAM[1], w['x'] - CAM[0])
    s = np.abs(Td - ts) < 0.15
    Q = Pd[s]
    az = np.arctan2(Q[:, 1] - CAM[1], Q[:, 0] - CAM[0]); rr = np.hypot(Q[:, 0] - CAM[0], Q[:, 1] - CAM[1])
    m = (np.abs(np.angle(np.exp(1j * (az - b)))) < math.radians(5)) & (np.abs(rr - tr['r']) < 0.5)
    if m.sum() < 5: continue
    Qh = np.c_[Q[m], np.ones(m.sum())] @ Wi.T
    uv = (K @ (Qh[:, :3] / Qh[:, 2:3]).T).T
    v_top = np.percentile(uv[:, 1], 2)          # the highest LiDAR return on the walker, in pixels (undistorted approx)
    u_med = np.median(uv[:, 0])
    x1, y1, x2, y2 = w['box']
    dv = y1 - v_top                              # >0: LiDAR head projects ABOVE the box top
    out.append((w['t'], tr['r'], v_top, y1, dv, math.degrees(math.atan(dv / fy)), u_med - 0.5 * (x1 + x2)))
    print('t %6.2f r %5.2f  LiDAR top v %6.1f  box top %6.1f  -> LiDAR %+5.1f px above (%.2f deg)  du %+5.1f px' % out[-1])
a = np.array(out)
print('median: %.1f px above = %.2f deg (n=%d)' % (np.median(a[:, 4]), np.median(a[:, 5]), len(a)))
