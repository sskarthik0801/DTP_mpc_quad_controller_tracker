import sqlite3, json, glob, collections
import numpy as np
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
BAG = glob.glob('/home/ideas-ad/dtp_datasets/20260930_1422_stageA_range/*.db3')[0]
db = sqlite3.connect('file:%s?mode=ro' % BAG, uri=True)
tid = db.execute("select id from topics where name='/dtp/range_source'").fetchone()[0]
t0 = db.execute('select min(timestamp) from messages').fetchone()[0] / 1e9
lid = []; sw = []
for ts, d in db.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid,)):
    j = json.loads(deserialize_message(d, String).data)
    for tr in j['tracks']:
        if tr['src'] == 'lidar':
            lid.append((j['stamp'][0] + j['stamp'][1] * 1e-9 - t0, tr['id'], tr['r'], tr.get('cam_r'), tr.get('switched'), tr.get('why')))
print('LiDAR-sourced frames', len(lid))
dis = [x for x in lid if x[3] is not None and x[2] - x[3] > 1.5]
print('LiDAR range > camera + 1.5 m:', len(dis), ' LiDAR < camera - 1.5 m:', sum(1 for x in lid if x[3] is not None and x[3] - x[2] > 1.5))
for x in dis[:25]: print('  t %6.1f id %3d lidar %.2f cam %.2f sw %s  %s' % (x[0], x[1], x[2], x[3], x[4], (x[5] or '')[:70]))
