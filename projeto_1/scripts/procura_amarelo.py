#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower2:
  def __init__(self):
    self.twist = Twist()

  def image_callback(self, image):
    bx = None
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([ 29,  50,  50])
    upper_yellow = numpy.array([ 30, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    cv2.imshow("procura_amarelo", mask)
    cv2.waitKey(1)

    
    #h, w, d = image.shape
    #search_top = h
    #search_bot = search_top + 20
    #mask[0:search_top, 0:w] = 0
    #mask[search_bot:h, 0:w] = 0

    M = cv2.moments(mask)
    if M['m00'] > 0:
      bx = int(M['m10']/M['m00'])
      by = int(M['m01']/M['m00'])
  
      cv2.circle(image, (bx, by), 20, (0,0,255), -1)
    
    return bx


# acertar as faixas de amarelo