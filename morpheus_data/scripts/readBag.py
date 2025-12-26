
import sys
# function to convert bag files to csv files
# called by crunch_Trossen_data
def readBag(file_name):
# file_name: bag file name
    import bagpy
    from bagpy import bagreader
    b = bagreader(file_name)
    wrist_pos = b.message_by_topic('/vx300s/wrist_pos')
    gripper_pos = b.message_by_topic('/vx300s/gripper_pos')
    upper_arm_pos = b.message_by_topic('/vx300s/upper_arm_pos')
    upper_forearm_pos = b.message_by_topic('/vx300s/upper_forearm_pos')
    shoulder_pos = b.message_by_topic('/vx300s/shoulder_pos')
    ee_arm_pos = b.message_by_topic('/vx300s/ee_arm_pos')
    lower_forearm_pos = b.message_by_topic('/vx300s/lower_forearm_pos')
    gripper_vel = b.message_by_topic('/vx300s/gripper_vel')
    wrist_vel = b.message_by_topic('/vx300s/wrist_vel')
    upper_arm_vel = b.message_by_topic('/vx300s/upper_arm_vel')
    upper_forearm_vel = b.message_by_topic('/vx300s/upper_forearm_vel')
    shoulder_vel = b.message_by_topic('/vx300s/shoulder_vel')
    ee_arm_vel = b.message_by_topic('/vx300s/ee_arm_vel')
    lower_forearm_vel = b.message_by_topic('/vx300s/lower_forearm_vel')
    joint_state = b.message_by_topic('/vx300s/joint_states')
    joy_raw = b.message_by_topic('/vx300s/joy')
    joy_pro = b.message_by_topic('/vx300s/commands/joy_processed')
    return [
        gripper_pos, wrist_pos, upper_arm_pos, upper_forearm_pos,
        shoulder_pos, ee_arm_pos, lower_forearm_pos,
        gripper_vel, wrist_vel, upper_arm_vel, upper_forearm_vel,
        shoulder_vel, ee_arm_vel, lower_forearm_vel,
        joy_raw, joy_pro                     # <-- add these
    ]
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 readBag.py /path/to/file.bag")
        sys.exit(1)
    
    file_name = sys.argv[1]
    print(f"Reading {file_name}...")
    outputs = readBag(file_name)
    print("Conversion complete!")