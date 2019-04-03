#!/usr/bin/env python


import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision.vision_constants import IMAGE_SIZE, CAMERA_INDEX, CV_WAITKEY

def resize_image_to_square_size(image):
    w, h, _ = image.shape
    if h != w:
        h_to_cut = (h - w)//2
        sqr_image = image[0:w, h_to_cut:h - h_to_cut]
        return sqr_image
    else:
        return image


rospy.init_node('camera_node', anonymous=True)


cv_bridge = CvBridge()
image_publisher = rospy.Publisher("square_image", Image, queue_size=1)

RUN = True

stream = cv2.VideoCapture(CAMERA_INDEX)
stream.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
stream.set(cv2.CAP_PROP_FPS, 30)


while RUN and not rospy.is_shutdown():
    ret, img = stream.read()
    if ret:
        square_img = resize_image_to_square_size(img)
        resized_sqaure_img = square_img
        resized_sqaure_img = cv2.resize(square_img, (IMAGE_SIZE, IMAGE_SIZE))
        image_message = cv_bridge.cv2_to_imgmsg(resized_sqaure_img, "bgr8")
        image_publisher.publish(image_message)
        if cv2.waitKey(CV_WAITKEY) & 0xFF == 27:
            RUN = not RUN
            stream.release()
            cv2.destroyAllWindows()
    else:
        print("Incorrect camera index << {} >>!".format(CAMERA_INDEX))
        RUN = not RUN


