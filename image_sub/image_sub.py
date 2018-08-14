#!/usr/bin/env python

""" cv_bridge_demo.py - Version 0.1 2011-05-29

    A ROS-to-OpenCV node that uses cv_bridge to map a ROS image topic and optionally a ROS
    depth image topic to the equivalent OpenCV image stream(s).
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2011 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import roslib
import rospy
import sys
import cv2
import message_filters
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
def callback(image_sub, point_sub):
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(image_sub, "bgr8")
    cv2.imwrite('frame.jpg', frame)
    '''
    for p in pc2.read_points(point_sub, field_names = ("x", "y", "z"), skip_nans=True):
        print " x : %f  y: %f  z: %f" %(p[0],p[1],p[2])
    '''
# What we do during shutdown

# Create the OpenCV display window for the RGB image



rospy.init_node("image_point_node")
# Create the cv_bridge object


# Subscribe to the camera image and depth topics and set
# the appropriate callbacks
image_sub = message_filters.Subscriber("/flea3/image_raw", Image)
point_sub = message_filters.Subscriber("/velodyne_points", PointCloud2)
ts = message_filters.ApproximateTimeSynchronizer([image_sub, point_sub], 10,1)
ts.registerCallback(callback)
rospy.spin()
# Use cv_bridge() to convert the ROS image to OpenCV format
'''
try:
    frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
except CvBridgeError, e:
    print e

# Convert the image to a Numpy array since most cv2 functions
# require Numpy arrays.
frame = np.array(frame, dtype=np.uint8)

# Process the frame using the process_image() function
#display_image = self.process_image(frame)
               
# Display the image.
cv2.imshow(self.node_name, frame)

# Process any keyboard commands
self.keystroke = cv2.waitKey(5)
if 32 <= self.keystroke and self.keystroke < 128:
    cc = chr(self.keystroke).lower()
    if cc == 'q':
        # The user has press the q key, so exit
        rospy.signal_shutdown("User hit q key to quit.")
'''

          
    


    
