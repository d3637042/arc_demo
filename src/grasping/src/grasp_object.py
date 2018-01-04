#!/usr/bin/env python
import rospy
import tf
import numpy as np
import tf.transformations as tfm

rospy.init_node('grasp_object', anonymous=True)