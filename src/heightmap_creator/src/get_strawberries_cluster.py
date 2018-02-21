#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/thres",Image)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color",Image,self.callback)
    self.depth_image_sub = rospy.Subscriber("/camera/depth/image_rect",Image,self.depth_callback)
    self.thres_mask = None
    self.center = None
    self.avg_depth = None
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    cv2.circle(cv_image, self.center, 5, (0, 0, 255), -1)
    cv2.circle(cv_image, (320, 240), 5, (0, 50, 255), -1)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)
    self.thres_mask = cv2.inRange(cv_image, (0, 0, 86), (82, 70, 255))
    
    thres_image = cv2.bitwise_and(cv_image, cv_image, mask = self.thres_mask)

    #cv2.imshow("Image window", thres_image)
    #cv2.waitKey(3)

    cnts = cv2.findContours(self.thres_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(cnts) > 0:
      c = max(cnts, key=cv2.contourArea)
      ((x, y), radius) = cv2.minEnclosingCircle(c)
      M = cv2.moments(c)
      self.center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

      if radius > 10:
        cv2.circle(thres_image, (int(x), int(y)), int(radius), (0, 255, 255), 1)
        cv2.circle(thres_image, self.center, 5, (0, 0, 255), -1)
        cv2.circle(thres_image, (320, 240), 5, (0, 50, 255), -1)
        print "RESULTS: ", self.center, self.avg_depth
        x_prime = (self.center[0]-308.935)/621.314
        y_prime = (self.center[1]-222.293)/621.314
        print x_prime, y_prime

    #cv2.imshow("Image window", thres_image)
    #cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(thres_image, "bgr8"))
    except CvBridgeError as e:
      print(e)
  def depth_callback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data)
    except CvBridgeError as e:
      print(e)
    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(3)

    thres_depth_image = cv2.bitwise_and(cv_image, cv_image, mask = self.thres_mask)
    count = 0
    acc = 0
    #cv2.imshow("Image window", thres_depth_image)
    #cv2.waitKey(3)
    for i in range(thres_depth_image.shape[0]):
      for j in range(thres_depth_image.shape[1]):
          if thres_depth_image[i][j]!=0 and ~(thres_depth_image[i][j] != thres_depth_image[i][j]):
            #print thres_depth_image[i][j]
            count = count + 1
            acc = acc + thres_depth_image[i][j]

    self.avg_depth = acc/count
    #print acc, count



def main(args):
  
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)