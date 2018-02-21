#!/usr/bin/env python
import rospy
import tf
import numpy as np
import tf.transformations as tfm
from apriltags_ros.msg import AprilTagDetectionArray
import sys

rospy.init_node('calibration', anonymous=True)
lr = tf.TransformListener()
br = tf.TransformBroadcaster()
camera_pose3d = 0
count = 0
def main():
	file = open(sys.argv[1], 'r+')
	data = file.readlines()
	file.close()


	camera_pose = [float(x) for x in data[0].split(' ') if x.strip()]
	while(1):
		br.sendTransform(camera_pose[0:3], camera_pose[3:7], rospy.Time.now(), '/center_camera', '/virtual_tag')
		rospy.sleep(0.01)
	rospy.on_shutdown(myhook)

def myhook():
    print "shutdown time!"			

if __name__=='__main__':
    main()
