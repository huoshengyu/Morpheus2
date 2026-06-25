#!/usr/bin/env python

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