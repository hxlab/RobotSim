import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import numpy as np
import os

# ─── CUSTOMIZE THIS BEFORE EACH RUN ───────────────────────────────────────────
RUN_LABEL  = "1000StiffnessCritDamping10Mass"  # <── change this each time
BAG_PATH   = "my_bag"
OUTPUT_DIR = "data"
# ──────────────────────────────────────────────────────────────────────────────

os.makedirs(OUTPUT_DIR, exist_ok=True)

def make_reader(bag_path):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)
    return reader

# ── First pass: find t0 from first /set_force message ─────────────────────────
reader = make_reader(BAG_PATH)
t0 = None
while reader.has_next():
    topic, raw, timestamp = reader.read_next()
    if topic == '/set_force':
        t0 = timestamp * 1e-9
        print(f"Goal pose found at t = {t0:.3f}s, using as t=0")
        break

if t0 is None:
    raise RuntimeError("No /set_force message found in bag — was it recorded?")

# ── Second pass: extract wrench and pos_error relative to t0 ──────────────────
reader = make_reader(BAG_PATH)
topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

data = {
    '/ee_wrench':    {'t': [], 'x': [], 'y': [], 'z': []},
    '/ee_pos_error': {'t': [], 'x': [], 'y': [], 'z': []},
}

while reader.has_next():
    topic, raw, timestamp = reader.read_next()
    if topic in data:
        msg_type = get_message(topic_types[topic])
        msg = deserialize_message(raw, msg_type)
        data[topic]['t'].append(timestamp * 1e-9 - t0)
        data[topic]['x'].append(msg.x)
        data[topic]['y'].append(msg.y)
        data[topic]['z'].append(msg.z)

# ── Save ───────────────────────────────────────────────────────────────────────
for topic, d in data.items():
    topic_key = topic.strip('/')
    out_path = os.path.join(OUTPUT_DIR, f"{RUN_LABEL}_{topic_key}.npz")
    np.savez(out_path,
             t = np.array(d['t']),
             x = np.array(d['x']),
             y = np.array(d['y']),
             z = np.array(d['z']))
    print(f"Saved: {out_path}")