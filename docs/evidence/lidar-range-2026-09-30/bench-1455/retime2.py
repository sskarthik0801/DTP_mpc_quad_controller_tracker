"""Copy the replay topics of a bag, each delivered at its own header stamp + the latency that topic
had in a HEALTHY run (the 14:22 stage_b bag, p25), and /tf lidar_init->lidar_imu rebuilt from
/lidar_slam/odom (the recorder kept only 77 of those transforms, 0.3-10 s late)."""
import sqlite3, sys
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
SRC, DST = sys.argv[1], sys.argv[2]
LAT = {'/camera/color/image_raw/compressed': 0.002, '/camera/color/camera_info': 0.002,
       '/camera/depth/image_raw': 0.023, '/camera/depth/camera_info': 0.023,
       '/lidar_slam/odom': 0.047, '/cloud_registered_body': 0.046, '/tf_static': None}
TF_LAT = 0.047
db = sqlite3.connect('file:%s?mode=ro' % SRC, uri=True)
topics = {r[0]: r[1:] for r in db.execute('select id,name,type,serialization_format,offered_qos_profiles from topics')}
w = rosbag2_py.SequentialWriter()
w.open(rosbag2_py.StorageOptions(uri=DST, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
rows = []; tf_qos = None
for tid, (name, typ, fmt, qos) in topics.items():
    if name == '/tf': tf_qos = qos
    if name not in LAT: continue
    w.create_topic(rosbag2_py.TopicMetadata(name=name, type=typ, serialization_format=fmt, offered_qos_profiles=qos))
    cls = get_message(typ)
    for ts, data in db.execute('select timestamp,data from messages where topic_id=?', (tid,)):
        if LAT[name] is None:
            rows.append((None, name, data)); continue
        m = deserialize_message(data, cls); h = m.header
        t = h.stamp.sec * 10**9 + h.stamp.nanosec
        rows.append((t + int(LAT[name] * 1e9), name, data))
        if name == '/lidar_slam/odom':
            x = TransformStamped(); x.header = m.header; x.child_frame_id = m.child_frame_id
            p, q = m.pose.pose.position, m.pose.pose.orientation
            x.transform.translation.x, x.transform.translation.y, x.transform.translation.z = p.x, p.y, p.z
            x.transform.rotation = q
            rows.append((t + int(TF_LAT * 1e9), '/tf', serialize_message(TFMessage(transforms=[x]))))
w.create_topic(rosbag2_py.TopicMetadata(name='/tf', type='tf2_msgs/msg/TFMessage', serialization_format='cdr', offered_qos_profiles=tf_qos))
t_first = min(t for t, n, _ in rows if t is not None)
rows = [((t_first - 1000) if t is None else t, n, d) for t, n, d in rows]
rows.sort(key=lambda r: r[0])
for t, n, d in rows: w.write(n, d, t)
del w
print('wrote', len(rows), 'messages')
