#!/usr/bin/env python
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


# Keyboard input imports
from readchar import readkey, readchar, key

def get_key():
    k = readchar()
    return k

def is_enter(k):
    return k == key.ENTER or k == key.ENTER_2

def is_esc(k):
    return k == key.ESC or k == key.ESC_2

def is_del(k):
    return k == key.DELETE

def is_backspace(k):
    return k == key.BACKSPACE

def is_keyboard_interrupt(k):
    return k == key.CTRL_C

def sanitize_filename(input):
    valid_punctuation = "._ "
    input = input.strip()
    input = input.replace(' ', '_')
    filename = "".join(c for c in input if c.isalnum() or c in valid_punctuation)
    return filename