#!/usr/bin/env python

"""
    moveit_cartesian_path.py - Version 0.1 2016-07-28

    Based on the R. Patrick Goebel's moveit_cartesian_demo.py demo code.

    Plan and execute a Cartesian path for the end-effector.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys
import moveit_commander
from geometry_msgs.msg import Pose
from copy import deepcopy
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveItCartesianPath:
    def __init__(self):
        rospy.init_node("moveit_cartesian_path", anonymous=False)

        rospy.loginfo("Starting node moveit_cartesian_path")

        rospy.on_shutdown(self.cleanup)

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the move group for the ur5_arm
        self.arm = moveit_commander.MoveGroupCommander('manipulator')

        # Get the name of the end-effector link
        end_effector_link = self.arm.get_end_effector_link()

        # Set the reference frame for pose targets
        reference_frame = "/base_link"

        # Set the ur5_arm reference frame accordingly
        self.arm.set_pose_reference_frame(reference_frame)

        # Allow replanning to increase the odds of a solution
        self.arm.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)
	self.arm.set_max_velocity_scaling_factor(0.1)
        # Get the current pose so we can add it as a waypoint
        start_pose = self.arm.get_current_pose(end_effector_link).pose

        #print "start pose", start_pose
        orientation_list = [start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w] 
        #print orientation_list
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        print roll, pitch, yaw
        #roll += 0.03
        quat = quaternion_from_euler (roll, pitch, yaw)
        #print quat
        # Initialize the waypoints list
        waypoints = []

        # Set the first waypoint to be the starting pose
        # Append the pose to the waypoints list
        waypoints.append(start_pose)

        wpose = deepcopy(start_pose)
        wpose.position.x = 0.087
        wpose.position.y = -0.492
        wpose.position.z = 0.787 
        #wpose.orientation.x = quat[0]
        #wpose.orientation.y = quat[1]
        #wpose.orientation.z = quat[2]
        #wpose.orientation.w = quat[3]
        waypoints.append(deepcopy(wpose))

        
        fraction = 0.0
        maxtries = 100
        attempts = 0
        #print waypoints
        # Set the internal state to the current state
        self.arm.set_start_state_to_current_state()

        # Plan the Cartesian path connecting the waypoints
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.arm.compute_cartesian_path (waypoints, 0.01, 0.0, True)

            # Increment the number of attempts
            attempts += 1

            # Print out a progress message
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

        # If we have a complete plan, execute the trajectory
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            self.arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")

    def cleanup(self):
        rospy.loginfo("Stopping the robot")

        # Stop any current arm movement
        self.arm.stop()

        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItCartesianPath()
    except KeyboardInterrupt:
        print "Shutting down MoveItCartesianPath node."
