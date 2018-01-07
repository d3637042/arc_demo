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
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf.transformations as tfm
class MoveItCartesianPath:
    def __init__(self):
        rospy.init_node("grasp_object", anonymous=False)

        rospy.loginfo("Starting node grasp_object")

        rospy.on_shutdown(self.cleanup)

        listener = tf.TransformListener()



        
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

        # Get the current pose so we can add it as a waypoint
        start_pose = self.arm.get_current_pose(end_effector_link).pose

        print "start pose", start_pose
        orientation_list = [start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w] 
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        #print roll, pitch, yaw
        #pitch = 0
        quat = quaternion_from_euler (roll, pitch, yaw)
        #print quat
        

        #listener.waitForTransform("/base_link", "/tag_0", rospy.Time(), rospy.Duration(4.0))
        #listener.waitForTransform("/base_link", "/tool0_controller", rospy.Time(), rospy.Duration(4.0))
        listener.waitForTransform("/base_link", "/dove_tag", rospy.Time(), rospy.Duration(4.0))
        running = True
        while running:
            try: 
                now = rospy.Time.now()
                #listener.waitForTransform("/base_link", "/tag_0", now, rospy.Duration(4.0))
                listener.waitForTransform("/base_link", "/dove_tag", now, rospy.Duration(4.0))
                #listener.waitForTransform("/base_link", "/tool0_controller", now, rospy.Duration(4.0))
                #(translation, quaternion) = listener.lookupTransform("/base_link", "/tag_0", now)
                (translation, quaternion) = listener.lookupTransform("/base_link", "/dove_tag", now)
                #(translation, quaternion) = listener.lookupTransform("/base_link", "/tool0_controller", now)
                #transform = tfm.concatenate_matrices(tfm.translation_matrix(translation), tfm.quaternion_matrix(quaternion))
                #inversed_transform = tfm.inverse_matrix(transform)
                #translation = tfm.translation_from_matrix(inversed_transform)
                #quaternion = tfm.quaternion_from_matrix(inversed_transform)
                #print translation, quaternion
                (translation_d1, quaternion_d1) = listener.lookupTransform("/base_link", "/gripper", now)
                (translation_d2, quaternion_d2) = listener.lookupTransform("/base_link", "/tool0_controller", now)
                (translation_middle, quaternion_middle) = listener.lookupTransform("/base_link", "/dove_tag_middle", now)
                running = False
            except(tf.Exception), e:
                print "transform not available, aborting..."
                print "Error message: ", e
        translation_d = deepcopy(translation_d1)
        for index in range(3):
            translation_d[index] = translation_d1[index]-translation_d2[index]
        #print translation[0], translation[1], translation[2]
        #print translation_d[0], translation_d[1], translation_d[2]
        print "trans: ", translation[0]-translation_d[0], translation[1]-translation_d[1], translation[2]-translation_d[2] 
        #print quaternion
        (roll, pitch, yaw) = euler_from_quaternion (quaternion)
        roll = roll + 3.14
        yaw = yaw + 1.57
        #print roll, pitch, yaw
        quaternion = quaternion_from_euler (roll, pitch, yaw)
        print "quat: ", quaternion

        orientation_list = [start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w] 
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        #print roll, pitch, yaw
        

        # Initialize the waypoints list
        waypoints = []

        
        # Set the first waypoint to be the starting pose
        # Append the pose to the waypoints list
        waypoints.append(start_pose)

        wpose = deepcopy(start_pose)
        wpose.position.x = translation_middle[0]
        wpose.position.y = translation_middle[1]
        wpose.position.z = translation_middle[2]
        wpose.orientation.x = quaternion[0]
        wpose.orientation.y = quaternion[1]
        wpose.orientation.z = quaternion[2]
        wpose.orientation.w = quaternion[3]
        waypoints.append(deepcopy(wpose))


        '''
        wpose = deepcopy(start_pose)
        wpose.position.x = translation[0]-translation_d[0]
        wpose.position.y = translation[1]-translation_d[1]
        wpose.position.z = translation[2]-translation_d[2]
        wpose.orientation.x = quaternion[0]
        wpose.orientation.y = quaternion[1]
        wpose.orientation.z = quaternion[2]
        wpose.orientation.w = quaternion[3]
        waypoints.append(deepcopy(wpose))
        '''


        
        

        print waypoints[0]
        print waypoints[1]

        while True:
            x = raw_input('waitkey(s to start): ')
            if x == 's':
                break
        
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
