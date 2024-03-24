#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import cv2
import numpy as np

class DepthImageProcessor:
    def __init__(self):
        rospy.loginfo("Depth_Processor")
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        self.binarized_pub = rospy.Publisher('/camera/depth/binarized', Image, queue_size=10)
    
    def depth_callback(self, data):
        try:
            
            # Convert ROS image message to CV2 format with original depth values
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            cv_image = cv2.flip(cv_image,1)
            # print(cv_image.shape)
            # Apply thresholding based on original depth values
            threshold_value = 160  # Adjust this value based on your depth sensor's range and your requirements
            thresholded_image = cv2.threshold(cv_image, threshold_value, 255, cv2.THRESH_BINARY)[1]

            # Optionally, normalize the thresholded image for visualization
            # This step makes it easier to visualize the binary mask resulting from thresholding
            thresholded_image_normalized = cv2.normalize(thresholded_image, None, 0, 255, cv2.NORM_MINMAX)
            thresholded_image_normalized = thresholded_image_normalized.astype(np.uint8)


            try:
                self.binarized_pub.publish(self.bridge.cv2_to_imgmsg(thresholded_image_normalized, encoding="mono8"))
            except CvBridgeError as e:
                rospy.logerr(e)

            resized_image = cv2.resize(thresholded_image_normalized, (1280, 720))
            # cv2.imshow("Thresholded and Normalized Image", resized_image)
            # cv2.waitKey(1)

        except Exception as e:
            rospy.logerr(e)
            return

if __name__ == '__main__':
        rospy.init_node('depth_image_processor', anonymous=True)
        depth_processor = DepthImageProcessor()
        rospy.spin()