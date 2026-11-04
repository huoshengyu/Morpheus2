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


from __future__ import print_function

import sys
import rospy

from morpheus_spawner.srv import *

def attacher_index_service_client(index = 0, operation = 0):
    # Make sure the spawner service is running first
    rospy.wait_for_service("attacher_index")
    # Attempt to call the spawner service and print whether it succeeded
    try:
        attacher = rospy.ServiceProxy("attacher_index", AttacherIndexService)
        req = AttacherIndexServiceRequest(index = index, operation = operation)
        res = attacher(req)
        return True
    # Notify and exit if the call fails
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    return

def usage():
    return "%s [index, operation]"%sys.argv[0]

if __name__ == "__main__":
    # Check command line argument count, retrieve name of collision_object.yaml entry
    if len(sys.argv) == 3:
        index = int(sys.argv[1])
        operation = int(sys.argv[2])
    # If command line args incorrect, print a reminder and exit
    else:
        print(usage())
        sys.exit(1)
    # Print input
    print("Requesting index %s, operation %s"%(index, operation))
    # Call
    attacher_index_service_client(index, operation)