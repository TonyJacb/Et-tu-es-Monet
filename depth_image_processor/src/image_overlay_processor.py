#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import cv2
import time
import numpy as np

class ImageOverlayProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.background_image = cv2.imread(r'/home/soslab/Downloads/480p_photos/test_image2_downsized_B_W.png')
        self.overlay_image = cv2.imread(r'/home/soslab/Downloads/480p_photos/HACKERCORE_TEST_NOISE.png')

        self.image_list = []
        self.overlay_image_index = 0
        self.change = False
        self.prev_index = None

        image_1 = cv2.imread(r'/home/soslab/Downloads/480p_photos/HACKERCORE_TEST_NOISE.png')
        image_2 = cv2.imread(r'/home/soslab/Downloads/480p_photos/HACKERCORE_TEST.png')
        image_3 = cv2.imread(r'/home/soslab/Downloads/480p_photos/3.png')
        image_4 = cv2.imread(r'/home/soslab/Downloads/480p_photos/4.png')
        image_5 = cv2.imread(r'/home/soslab/Downloads/480p_photos/5.png')

        self.noise = cv2.imread(r'/home/soslab/Downloads/480p_photos/noise.png')

        self.image_list.append(image_1)
        self.image_list.append(image_2)
        self.image_list.append(image_3)
        self.image_list.append(image_4)
        self.image_list.append(image_5)
        
        self.start_time = time.time()
        self.binarized_sub = rospy.Subscriber('/camera/depth/binarized', Image, self.binarized_callback)
        self.overlay_pub = rospy.Publisher('/overlay/image', Image, queue_size=1)

    def binarized_callback(self, data):
        self.mask_image = self.bridge.imgmsg_to_cv2(data)
        # print(self.mask_image.shape)
        self.process_images_fast()

    def select_overlay(self):
        elapsed_time = time.time() - self.start_time
        if elapsed_time > 3:
            self.start_time = time.time()
            self.prev_index = self.overlay_image_index
            while self.prev_index == self.overlay_image_index:
                self.overlay_image_index = np.random.randint(0,len(self.image_list))
        
        # print(self.change, self.prev_index, self.overlay_image_index)
        return self.image_list[self.overlay_image_index]

    def process_images_fast(self):
        self.overlay_image = self.select_overlay()
        # Ensure the overlay image and the mask have the same dimensions
        if self.overlay_image.shape[:2] != self.mask_image.shape[:2]:
            rospy.logerr("Overlay image and mask image have different dimensions.")
            return

        # Reset background image for each callback to avoid accumulation of overlay
        self.background_image = cv2.imread('/home/soslab/Downloads/480p_photos/test_image2_downsized_B_W.png')

        # Create a boolean mask from the binarized image
        mask = self.mask_image > 0

        # Apply the mask to overlay the second image onto the first
        self.background_image[mask] = self.overlay_image[mask]

        resized_image = cv2.resize(self.background_image, (1280, 720))
        try:
            self.overlay_pub.publish(self.bridge.cv2_to_imgmsg(resized_image, encoding="bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)


if __name__ == '__main__':
    rospy.init_node('image_overlay_processor', anonymous=True)
    image_processor = ImageOverlayProcessor()
    rospy.spin()