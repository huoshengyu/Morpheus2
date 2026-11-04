// ---------------------------------------------------------------------------
// Morpheus2: ROS2 control and feedback for manual teleoperation research
// Copyright (C) 2026 Brian Huo
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
// ---------------------------------------------------------------------------

#include <map>

// PS3 Controller button mappings
static const std::map<std::string, int> ps3 = {{"GRIPPER_PWM_DEC", 0}, // buttons start here
                                               {"GRIPPER_OPEN", 1},
                                               {"GRIPPER_PWM_INC", 2},
                                               {"GRIPPER_CLOSE", 3},
                                               {"EE_Y_INC", 4},
                                               {"EE_Y_DEC", 5},
                                               {"WAIST_CCW", 6},
                                               {"WAIST_CW", 7},
                                               {"SLEEP_POSE", 8},
                                               {"HOME_POSE", 9},
                                               {"TORQUE_ENABLE", 10},
                                               {"FLIP_EE_X", 11},
                                               {"FLIP_EE_ROLL", 12},
                                               {"SPEED_INC", 13},
                                               {"SPEED_DEC", 14},
                                               {"SPEED_COARSE", 15},
                                               {"SPEED_FINE", 16},
                                               {"EE_X", 0},            // axes start here
                                               {"EE_Z", 1},
                                               {"EE_ROLL", 3},
                                               {"EE_PITCH", 4}};

// PS4 Controller button mappings
static const std::map<std::string, int> ps4 = {{"GRIPPER_PWM_DEC", 0}, // buttons start here
                                               {"GRIPPER_OPEN", 1},
                                               {"GRIPPER_PWM_INC", 2},
                                               {"GRIPPER_CLOSE", 3},
                                               {"EE_Y_INC", 4},
                                               {"EE_Y_DEC", 5},
                                               {"WAIST_CCW", 6},
                                               {"WAIST_CW", 7},
                                               {"SLEEP_POSE", 8},
                                               {"HOME_POSE", 9},
                                               {"TORQUE_ENABLE", 10},
                                               {"FLIP_EE_X", 11},
                                               {"FLIP_EE_ROLL", 12},
                                               {"EE_X", 0},            // axes start here
                                               {"EE_Z", 1},
                                               {"EE_ROLL", 3},
                                               {"EE_PITCH", 4},
                                               {"SPEED_TYPE", 6},
                                               {"SPEED", 7}};

// Xbox 360 Controller button mappings
static const std::map<std::string, int> xbox360 = {{"GRIPPER_PWM_DEC", 0}, // buttons start here
                                                   {"GRIPPER_OPEN", 1},
                                                   {"GRIPPER_CLOSE", 2},
                                                   {"GRIPPER_PWM_INC", 3},
                                                   {"WAIST_CCW", 4},
                                                   {"WAIST_CW", 5},
                                                   {"SLEEP_POSE", 6},
                                                   {"HOME_POSE", 7},
                                                   {"TORQUE_ENABLE", 8},
                                                   {"FLIP_EE_X", 9},
                                                   {"FLIP_EE_ROLL", 10},
                                                   {"EE_X", 0},            // axes start here
                                                   {"EE_Z", 1},
                                                   {"EE_Y_INC", 2},
                                                   {"EE_ROLL", 3},
                                                   {"EE_PITCH", 4},
                                                   {"EE_Y_DEC", 5},
                                                   {"SPEED_TYPE", 6},
                                                   {"SPEED", 7}};

// Map of maps to allow accessing the above by name
static const std::map<std::string, std::map<std::string, int>> button_mappings = {{"ps3", ps3},
                                                                                {"ps4", ps4},
                                                                                {"xbox360", xbox360}};