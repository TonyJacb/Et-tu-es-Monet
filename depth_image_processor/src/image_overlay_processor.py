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
        self.background_image = cv2.imread(r'/home/soslab/Downloads/visathon_second_set/background.png')
        self.overlay_image = cv2.imread(r'/home/soslab/Downloads/visathon_second_set/1.png')
        self.previous_overlay_image = self.overlay_image.copy()
        
        self.image_list = []
        self.overlay_image_index = 0
        self.prev_index = None
        self.blend_factor = 1 # Initialize blend factor to 1 (fully new image)
        self.mask_image = None  # Initialize mask_image
        self.last_binarized_image = None  # To store the last binarized image
        # Timer setup
        self.update_interval = 0.05  # Interval in seconds to update the overlay image
        rospy.Timer(rospy.Duration(self.update_interval), self.timer_callback)

        self.display_duration = 3  # Display each image for 5 seconds before starting to blend
        self.transition_duration = 6  # Transition to the next image over 3 seconds
        self.last_change_time = time.time()
        self.in_transition = False  # Track whether currently in a transition

        # image_1 = cv2.imread(r'/home/soslab/Downloads/480p_photos/HACKERCORE_TEST_NOISE.png')
        # image_2 = cv2.imread(r'/home/soslab/Downloads/480p_photos/HACKERCORE_TEST.png')
        # image_3 = cv2.imread(r'/home/soslab/Downloads/480p_photos/3.png')
        # image_4 = cv2.imread(r'/home/soslab/Downloads/480p_photos/4.png')
        # image_5 = cv2.imread(r'/home/soslab/Downloads/480p_photos/5.png')

        # self.noise = cv2.imread(r'/home/soslab/Downloads/480p_photos/noise.png')

        # self.image_list.append(image_1)
        # self.image_list.append(image_2)
        # self.image_list.append(image_3)
        # self.image_list.append(image_4)
        # self.image_list.append(image_5)

        # Directory where the images are stored
        self.image_directory = r'/home/soslab/Downloads/visathon_second_set/'

        # Initialize an empty list to store the images
        # image_list = []

        # Loop through the numbers, construct each filename, read the image, and append it to the list
        for i in range(1, 7):  # This will iterate through numbers 1 to 5
            filename = f"{i}.png"  # Construct the filename dynamically
            print(filename)
            image_path = self.image_directory + filename  # Construct the full image path
            image = cv2.imread(image_path)  # Read the image
            resized_image = cv2.resize(image, (848, 480)) 
            self.image_list.append(resized_image)  # Append the image to the list
        
        self.start_time = time.time()
        self.binarized_sub = rospy.Subscriber('/camera/depth/binarized', Image, self.binarized_callback)
        self.overlay_pub = rospy.Publisher('/overlay/image', Image, queue_size=1)


    def timer_callback(self, event):
        """
        Timer callback to periodically update the overlay image.
        """
        self.select_overlay()
        # Call process_images_fast with the last received binarized image
        self.process_images_fast()

    def binarized_callback(self, data):
        # self.mask_image = self.bridge.imgmsg_to_cv2(data)
        # print(self.mask_image.shape)
        #self.process_images_fast()

        #New
        try:
            self.mask_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            rospy.logerr(e)

        self.last_binarized_image = self.bridge.imgmsg_to_cv2(data)

    def select_overlay(self):
        # elapsed_time = time.time() - self.start_time
        # if elapsed_time > 3:
        #     self.start_time = time.time()
        #     self.prev_index = self.overlay_image_index
        #     while self.prev_index == self.overlay_image_index:
        #         self.overlay_image_index = np.random.randint(0,len(self.image_list))
        
        # # print(self.change, self.prev_index, self.overlay_image_index)
        # return self.image_list[self.overlay_image_index]

        current_time = time.time()
        elapsed_since_last_change = current_time - self.last_change_time

        # Start a new transition if needed
        if not self.in_transition and elapsed_since_last_change > self.display_duration:
            self.in_transition = True
            self.last_change_time = current_time
            self.prev_index = self.overlay_image_index
            while self.prev_index == self.overlay_image_index:
                self.overlay_image_index = np.random.randint(0, len(self.image_list))
            self.previous_overlay_image = self.overlay_image.copy()  # Update previous image for blending
            self.blend_factor = 0  # Reset blend factor for new transition

        # Handle the blending during the transition
        if self.in_transition:
            elapsed_since_transition_start = current_time - self.last_change_time
            self.blend_factor = min(1, elapsed_since_transition_start / self.transition_duration)  # Ensure blend_factor does not exceed 1

            if self.blend_factor < 1:
                # Perform the blending
                self.overlay_image = cv2.addWeighted(
                    self.previous_overlay_image, 1 - self.blend_factor,
                    self.image_list[self.overlay_image_index], self.blend_factor, 0
                )
            else:
                # Transition complete, update to the new image directly
                self.overlay_image = self.image_list[self.overlay_image_index]
                self.in_transition = False
                self.last_change_time = current_time

        # Debugging output to check dimensions and types
        # print(f"Previous image dimensions: {self.previous_overlay_image.shape}, type: {self.previous_overlay_image.dtype}")
        # print(f"Current image dimensions: {self.overlay_image.shape}, type: {self.overlay_image.dtype}")

        # Perform blending with error handling
        try:
            blended_image = cv2.addWeighted(self.previous_overlay_image, 1 - self.blend_factor,
                                            self.overlay_image, self.blend_factor, 0)
            if blended_image is not None:
                return blended_image
            else:
                print("Blending failed, using previous overlay image.")
                return self.previous_overlay_image
        except Exception as e:
            print(f"Error during blending: {e}")
            # Fallback to a default behavior
            return self.previous_overlay_image
    
    # def process_images_fast(self):
    #     #self.overlay_image = self.select_overlay()
    #     # self.select_overlay() - This is now called within the timer_callback

    #     # Check if a binarized image has been received; if not, log a warning and return
    #     if self.mask_image is None:
    #         rospy.logwarn("No binarized image received yet.")
    #         return 
        
    #     # Ensure the overlay image and the mask have the same dimensions
    #     if self.overlay_image.shape[:2] != self.mask_image.shape[:2]:
    #         rospy.logerr("Overlay image and mask image have different dimensions.")
    #         return
    #     # Reset background image for each callback to avoid accumulation of overlay
    #     self.background_image = cv2.imread('/home/soslab/Downloads/visathon_second_set/background_2.png')

    #     # Create a boolean mask from the binarized image
    #     mask = self.mask_image > 0

    #     # Apply the mask to overlay the second image onto the first
    #     self.background_image[mask] = self.overlay_image[mask]

    #     resized_image = cv2.resize(self.background_image, (848, 480))
    #     try:
    #         self.overlay_pub.publish(self.bridge.cv2_to_imgmsg(resized_image, encoding="bgr8"))
    #     except CvBridgeError as e:
    #         rospy.logerr(e)

    def process_images_fast(self):
        """
        Processes images to apply the overlay, now independent of direct binarized image updates.
        """
        # First, check if a binarized image has been received; if not, log a warning and skip processing.
        if self.last_binarized_image is None:
            rospy.logwarn("No binarized image received yet. Skipping processing.")
            return

        # Reset background image for each processing to avoid accumulation of overlay effects.
        # Ensure the correct path is set for your background image.
        self.background_image = cv2.imread('/home/soslab/Downloads/visathon_second_set/background.png')

        # Convert the last received binarized image to a boolean mask for overlay application.
        mask = self.last_binarized_image > 0

        # Check if the overlay image and the mask have the same dimensions.
        if self.overlay_image.shape[:2] != self.last_binarized_image.shape[:2]:
            rospy.logerr("Overlay image and mask image have different dimensions.")
            return

        # Apply the mask to overlay the image onto the background.
        self.background_image[mask] = self.overlay_image[mask]

        # Resize the final image if necessary, to match your output specifications.
        resized_image = cv2.resize(self.background_image, (848, 480))

        # Publish the processed image.
        try:
            self.overlay_pub.publish(self.bridge.cv2_to_imgmsg(resized_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(e)


if __name__ == '__main__':
    rospy.init_node('image_overlay_processor', anonymous=True)
    image_processor = ImageOverlayProcessor()
    rospy.spin()