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

session_state_dict = {
    "intro":["id"],
    "id":["task"],
    "task":["trial"],
    "trial":["ready"],
    "ready":["recording"],
    "recording":[],
}

session_string_dict = {
    "header_title":"Morpheus Recording Session",
    "header_id":"Subject ID: ",
    "header_task":"Task Name: ",
    "header_trial":"Trial Number: ",
    "intro_description":"Press ENTER to continue, or press ESC to cancel.",
    "id_description":"Please enter subject ID: ",
    "id_enter":"Subject ID is ",
    "id_esc":"Ending recording session.",
    "task_description":"Please enter task name: ",
    "task_enter":"Task is ",
    "task_esc":"Cancelling task selection.",
    "trial_description":"Please enter trial number (or leave blank to use next available index): ",
    "trial_enter":"Trial number is ",
    "trial_esc":"Cancelling task selection.",
    "ready_description":"Ready to record. Press ENTER to start or ESC to go back.",
    "ready_enter":"Starting recording ",
    "ready_esc":"Exiting recording ",
    "recording_description":"Now recording. Press ENTER or ESC to stop.",
    "recording_enter":"Stopping recording ",
    "recording_esc":"Stopping recording ",
    "recording_taken":"Filename already exists! Press ESC to re-enter trial number and try again.",
}