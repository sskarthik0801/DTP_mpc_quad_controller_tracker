import sqlite3, sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
for BAG in sys.argv[1:]:
    db = sqlite3.connect('file:%s?mode=ro' % BAG, uri=True)
    topics = {r[0]: (r[1], r[2]) for r in db.execute('select id,name,type from topics')}
    name2id = {v[0]: k for k, v in topics.items()}
    print(BAG.split('/')[-2])
    for name in ['/camera/color/image_raw/compressed', '/camera/depth/image_raw', '/tf', '/lidar_slam/odom', '/cloud_registered_body']:
        if name not in name2id: continue
        tid = name2id[name]; cls = get_message(topics[tid][1]); lag = []
        for i, (ts, data) in enumerate(db.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid,))):
            if i > 3000: break
            m = deserialize_message(data, cls)
            if name == '/tf':
                tr = [x for x in m.transforms if x.child_frame_id == 'lidar_imu']
                if not tr: continue
                h = tr[0].header
            else: h = m.header
            lag.append(ts / 1e9 - (h.stamp.sec + h.stamp.nanosec * 1e-9))
        lag = np.array(lag)
        print('  %-36s n %5d  rx-stamp p5 %.3f p25 %.3f p50 %.3f p90 %.3f' % (name, len(lag), *np.percentile(lag, [5, 25, 50, 90])))
