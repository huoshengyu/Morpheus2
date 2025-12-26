#!/usr/bin/env python

# General Packages
import os
import logging
from collections import deque
# ROS Packages
import rospy
# Keyboard input imports
from readchar import readkey, readchar, key
# Local imports
from data import Data
from utils import *
from ui_strings import *

# Documentation for rosbag py on ros wiki is poor, instead see source code here:
# https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosbag/src/rosbag/bag.py

### Session object, organizing multiple data recordings (trials) ###
class Session():
    def __init__(self, id="", rate=60):
        id = sanitize_filename(id)
        self._id = id # subject ID (for tracking latest trial)
        self._task = "" # trial name (for tracking latest trial)
        self._trial_number = 0 # trial number (for tracking latest trial)
        self._current_data = None # Last touched Data file
        self._rate = rate # loop rate for key press reading
        self._print_count = 0 # Number of lines last printed in the console
        self._queue = deque() # Queue for handling terminal printing
        self._rate = rospy.Rate(60) # Hz, for keyboard reading loop rate

        #logging.basicConfig(level=logging.INFO)
        #self.logger = logging.getLogger(__name__)
    
    def prompt(self, state="", id="", task="", trial=""):
        # Initialize input and output variables
        if state == "":
            state = "intro"
        input = ""

        while not rospy.is_shutdown():
            # Add static strings to queue
            self._queue = self.enqueue_header(self._queue, id, task, trial)
            self._queue = self.enqueue_string(self._queue, state + "_description")
            # Handle state-related events
            if state == "recording":
                self._current_data = self.get_trial(id, task, trial)
                if self._current_data != None:
                    self._current_data.write()
            # Add user input to queue
            self._queue.append(input)
            # Print from the queue
            self._queue = self.dequeue(self._queue)
            
            # Get keyboard input events
            # This happens after print so that blocking to read keys does not delay header printing
            event = None
            input, event = self.handle_key_events(input, event)

            # Respond to keyboard input events
            if event == key.ENTER:
                # ENTER and ESC both stop recording
                if state == "recording":
                    event = key.ESC
                # Flush input to child prompts
                if state == "id":
                    id = input
                if state == "task":
                    task = input
                if state == "trial":
                    trial = input
                    if trial == "":
                        trial = self.get_next_trial_number(id, task)
                for child_state in session_state_dict[state]:
                    self.prompt(child_state, id, task, trial)
                input = ""
            if event == key.ESC:
                if state == "recording" and self._current_data is not None:
                    self._current_data.close()
                break
            # Limit loop rate
            self._rate.sleep()

    def handle_key_events(self, input="", event=None):
        k = get_key()
        if k.isalnum() or k in "._ ":
            input += k
        elif k == key.BACKSPACE or k == key.DELETE:
            if len(input) > 0:
                input = input[:-1]
        elif k == key.ENTER:
            event = key.ENTER
        elif k == key.ESC:
            event = key.ESC
        return input, event
    
    def enqueue_string(self, queue, key, suffix=""):
        try:
            queue.append(session_string_dict[key] + str(suffix)) # OR statement to handle case when suffix==None
        except KeyError: # If Key is missing, assume this is intended and add a blank
            queue.append("")
        return queue
    
    def enqueue_header(self, queue, id, task, trial):
        queue = self.enqueue_string(queue, "header_title")
        queue = self.enqueue_string(queue, "header_id", id)
        queue = self.enqueue_string(queue, "header_task", task)
        queue = self.enqueue_string(queue, "header_trial", trial)
        return queue
    
    def dequeue(self, queue):
        # Remove last printed messages
        for i in range(self._print_count):
            print("\x1b[1A\x1b[2K", end="")
        # Print new messages
        self._print_count = len(queue)
        for i in range(len(queue)):
            print(queue.popleft())
        return queue

    def get_trial(self, id, task, trial_number):
        # Retrieve data from file
        data = None
        filename = self.get_filename(id, task, trial_number)
        filepath = self.get_filepath(filename)
        filepath_exists = os.path.isfile(filepath)
        if filepath_exists:
            self._queue.pop()
            self.enqueue_string(self._queue, "recording_taken")
        else:
            data = self.create_trial(filepath)
        return data
    
    def get_next_trial_number(self, id, task):
        # Get next available trial number (0 indexed)
        trial_number = -1 # Start negative so that first increment goes to 0
        filepath_exists = True
        while filepath_exists:
            trial_number = trial_number + 1 # Try next trial number
            filepath = self.get_filepath(self.get_filename(id, task, trial_number))
            filepath_exists = os.path.isfile(filepath) # Check if filepath is available
        return trial_number

    def get_filename(self, id, task, trial_number):
        # Get filename based on  subject id, task type, trial number (0 indexed)
        filename = "%s%s%s.bag" % (str(id), str(task), str(trial_number))
        return filename
    
    def get_filepath(self, filename):
        # Get absolute filepath (joins filename to the data directory path)
        dirname = os.path.dirname(__file__)
        datadir = os.path.join(os.path.dirname(dirname), 'data')
        filepath = os.path.join(datadir, filename)
        return filepath

    def create_trial(self, filepath):
        # Create new trial with given filepath
        trial = Data(filepath) # Prepare to read/write
        return trial

    def start_trial(self, id, task, trial_number):
        # Start recording trial
        data = self._data_dict[id][task][trial_number] # Get from dict
        data.write() # Start writing

    def stop_trial(self, id, task, trial_number):
        # Stop recording trial
        data = self._data_dict[id][task][trial_number] # Get from dict
        data.close() # Stop writing

def main():
    rospy.init_node('session')
    session = Session()

    # while session.running:
    #     session.handle_events()

    session.prompt()

if __name__=='__main__':
    main()