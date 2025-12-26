#!/usr/bin/env python

# ROS Packages
import rospy
import rosbag
from rosbag import Compression

# Documentation for rosbag py on ros wiki is poor, instead see source code here:
# https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosbag/src/rosbag/bag.py

### Wrapper for rosbag, allowing convenient recording start/stop ###
### Since rosbag begins reading/writing upon initialization, this is useful for timing the start of a recording session ###
class Data():
    def __init__(self, f, compression=Compression.NONE, chunk_threshold=768 * 1024, allow_unindexed=False, options=None, skip_index=False):
        self.f = f # filename of bag to open or a stream to read from (str or file)
        self.compression = compression
        self.chunk_threshold = chunk_threshold
        self.allow_unindexed = allow_unindexed
        self.options = options
        self.skip_index = skip_index

        self._bag = None
        self._is_open = False
    
    def read(self):
        self.open(self.f, 'r', self.allow_unindexed)
    
    def write(self):
        self.open(self.f, 'w', self.allow_unindexed)
    
    def append(self):
        self.open(self.f, 'a', self.allow_unindexed)

    def open(self, f, mode, allow_unindexed):
        # Arguments here are based on Bag._open() in rosbag's Bag.py
        if self._is_open:
            rospy.loginfo("Rosbag already open, closing then reopening in mode %s." % (mode))
            self.close()
        self._bag = rosbag.Bag(f, mode, self.compression, self.chunk_threshold, allow_unindexed, self.options, self.skip_index)
        self._is_open = True
    
    def close(self):
        self._bag.close()
        self._is_open = False


def main():
    rospy.init_node('data')
    data = Data()
    data.write()

    rate = rospy.Rate(60)  # hz
    while not rospy.is_shutdown():
        rate.sleep()
    data.close()

if __name__=='__main__':
    main()