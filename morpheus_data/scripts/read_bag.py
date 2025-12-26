#!/usr/bin/env python3
import rosbag
import csv
import os
import argparse

# --- your topics (from readBag) ---
TOPICS = [
    '/vx300s/wrist_pos', '/vx300s/gripper_pos',
    '/vx300s/upper_arm_pos', '/vx300s/upper_forearm_pos',
    '/vx300s/shoulder_pos', '/vx300s/ee_arm_pos',
    '/vx300s/lower_forearm_pos',
    '/vx300s/gripper_vel', '/vx300s/wrist_vel',
    '/vx300s/upper_arm_vel', '/vx300s/upper_forearm_vel',
    '/vx300s/shoulder_vel', '/vx300s/ee_arm_vel',
    '/vx300s/lower_forearm_vel',
    '/vx300s/joint_states',
    '/vx300s/commands/joy_raw', '/vx300s/commands/joy_processed'
]

def flatten_msg(msg, prefix=""):
    """Recursively flatten a ROS message into {field_path: value}."""
    out = {}
    if hasattr(msg, "__slots__"):
        for slot in msg.__slots__:
            val = getattr(msg, slot)
            new_prefix = f"{prefix}{slot}"
            if hasattr(val, "__slots__"):
                out.update(flatten_msg(val, new_prefix + "."))
            elif isinstance(val, list):
                for i, v in enumerate(val):
                    if hasattr(v, "__slots__"):
                        out.update(flatten_msg(v, f"{new_prefix}[{i}]."))
                    else:
                        out[f"{new_prefix}[{i}]"] = v
            else:
                out[new_prefix] = val
    else:
        out[prefix.rstrip(".")] = msg
    return out

def topic_alias(topic: str) -> str:
    return topic.strip("/").replace("/", "_") or "root"

def export_selected_topics_single_csv(bag_path: str, out_csv: str, topics=TOPICS):
    # --- PASS 1: discover columns from selected topics only ---
    all_cols = set()
    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=topics):
            flat = flatten_msg(msg)
            ta = topic_alias(topic)
            for k in flat.keys():
                all_cols.add(f"{ta}.{k}")
    all_cols = sorted(all_cols)

    # --- PASS 2: write one combined CSV (one row per message) ---
    os.makedirs(os.path.dirname(out_csv), exist_ok=True)
    with rosbag.Bag(bag_path, 'r') as bag, open(out_csv, "w", newline="") as f:
        writer = csv.writer(f)
        header = ["time", "topic"] + all_cols
        writer.writerow(header)

        for topic, msg, t in bag.read_messages(topics=topics):
            flat = flatten_msg(msg)
            ta = topic_alias(topic)
            prefixed = {f"{ta}.{k}": v for k, v in flat.items()}
            row = [t.to_sec(), topic] + [prefixed.get(col, "") for col in all_cols]
            writer.writerow(row)

    print(f"✅ Exported single CSV with selected topics:\n  {out_csv}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Read selected ROS bag topics to a single CSV.")
    parser.add_argument("--bag", "-b",
                        default=os.path.expanduser("~/catkin_ws/src/morpheus_data/data/Task_1.bag"))
    parser.add_argument("--out", "-o",
                        default=os.path.expanduser("~/catkin_ws/src/morpheus_data/data/Task_1_selected_all.csv"))
    args = parser.parse_args()
    export_selected_topics_single_csv(args.bag, args.out)
