#!/usr/bin/env python3
# ---------------------------------------------------------------------------
# Morpheus2: ROS2 control and feedback for manual teleoperation research
# Copyright (C) 2026 Brian Huo
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
# ---------------------------------------------------------------------------

import rospy
from sensor_msgs.msg import Joy
from pylsl import StreamInfo, StreamOutlet

# how many you actually want to send
N_AXES = 8
N_BUTTONS = 12
CHANNEL_COUNT = N_AXES + N_BUTTONS   # 17

# IMPORTANT: channel_count must be > 0
info = StreamInfo(
    name='PS3_Controller',
    type='controller',
    channel_count=CHANNEL_COUNT,
    nominal_srate=0,          # 0 = irregular, better for joystick
    channel_format='float32',
    source_id='ps3_ros_01'
)
outlet = StreamOutlet(info)

def joy_cb(msg: Joy):
    # pad so we always send exactly CHANNEL_COUNT values
    axes = list(msg.axes)[:N_AXES] + [0.0] * max(0, N_AXES - len(msg.axes))
    buttons = [float(b) for b in msg.buttons][:N_BUTTONS] + [0.0] * max(0, N_BUTTONS - len(msg.buttons))
    sample = axes + buttons
    outlet.push_sample(sample)

if __name__ == "__main__":
    rospy.init_node("ps3_lsl_sender")
    rospy.Subscriber("/vx300s/joy", Joy, joy_cb)
    rospy.loginfo("Streaming /joy to LSL as PS3_Controller (17 channels)")
    rospy.spin()
