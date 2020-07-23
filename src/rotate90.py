#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

pub = None
bridge = CvBridge()

def callback(image_msg):
    print "IMAGE",image_msg.header.seq
    try:
        cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    cv_image = np.rot90(cv_image).copy() 
    new_image_msg = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
    new_image_msg.header = image_msg.header

    pub.publish(new_image_msg)

if __name__ == '__main__':

    rospy.init_node('rotate90', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # get a parameter from our private namespace
    input_topic = rospy.get_param('~input_topic')
    print input_topic
    output_topic = input_topic+'_rotated'

    pub = rospy.Publisher(output_topic, Image, queue_size=10)

    rospy.Subscriber(input_topic, Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

