#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
  def __init__(self):
    self.twist = Twist()

  def image_callback(self, image):
    cx = None
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([ 29, 250,  250])
    upper_yellow = numpy.array([ 30, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # cv2.imshow("segue_amarelo", mask)
    # cv2.waitKey(1)

    
    h, w, d = image.shape
    search_top = 67*h/100
    search_bot = search_top + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0

    M = cv2.moments(mask)
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
  
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
    
    return cx


# acertar as faixas de amarelo