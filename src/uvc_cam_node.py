#!/usr/bin/env python
"""Publishes UVC Cam images as ROS messages."""

import uvc
import sys
import numpy as np
import cv2

import rospy
from camera_info_manager import CameraInfoManager
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

DEFAULT_CAMERA_NAME = 'uvc_cam'
DEFAULT_IMAGE_TOPIC = 'image_raw'
DEFAULT_CAMERA_TOPIC = 'camera_info'


class UVCCamNode:

    def __init__(self):

        rospy.init_node('{}_node'.format(DEFAULT_CAMERA_NAME), argv=sys.argv)

	# possible values to set can be found by running "rosrun uvc_camera uvc_camera_node _device:=/dev/video1", see http://github.com/ros-drivers/camera_umd.git
	dev_list = uvc.device_list()
	self.cap = uvc.Capture(dev_list[0]["uid"])
	#self.cap.frame_mode = (960, 540, 120) 
	#self.cap.frame_mode = (640, 480, 120) 
	self.cap.frame_mode = (640, 480, 90)
	frame = self.cap.get_frame_robust()
	controls_dict = dict([(c.display_name, c) for c in self.cap.controls])
	controls_dict['Brightness'].value = 10 #[-64,64], not 0 (no effect)!!
	controls_dict['Contrast'].value = 0 #[0,95]
	controls_dict['Hue'].value = 0 #[-2000,2000]
	controls_dict['Saturation'].value = 0 #[0,100]
	controls_dict['Sharpness'].value = 1 #[1,100]
	controls_dict['Gamma'].value = 100 #[80,300]
	controls_dict['Power Line frequency'].value = 1 #1:50Hz, 2:60Hz
	controls_dict['Backlight Compensation'].value = False #True or False
	#controls_dict['Absolute Exposure Time'].value = 10000 #[78,10000] set Auto Exposure Mode to 1
	controls_dict['Auto Exposure Mode'].value = 8 #1:manual, 8:apperturePriority
	controls_dict['White Balance temperature,Auto'].value = True
	#controls_dict['White Balance temperature'].value = 4600 #[2800,6500]
	rospy.loginfo("These camera settings will be applied:")
	for c in self.cap.controls:
		rospy.loginfo('%s: %i'%(c.display_name, c.value))

        self.image_topic  = rospy.get_param('~image', DEFAULT_IMAGE_TOPIC)
        self.camera_topic = rospy.get_param('~camera', DEFAULT_CAMERA_TOPIC)
        self.calibration  = rospy.get_param('~calibration', '')
	self.encoding     = rospy.get_param('~encoding','mono8')

        self.manager = CameraInfoManager(cname=DEFAULT_CAMERA_NAME,
                                         url='file://' + self.calibration,
                                         namespace=DEFAULT_CAMERA_NAME)
        self.manager.loadCameraInfo()  # Needs to be called before getter!
        self.camera_info = self.manager.getCameraInfo()
        self.bridge = CvBridge()
        self.image_publisher = rospy.Publisher(self.image_topic, Image,
                                               queue_size=1)
        self.camera_publisher = rospy.Publisher(self.camera_topic, CameraInfo,
                                                queue_size=1)
        self.seq = 0
	#self.rate = rospy.Rate(60)

    def read_and_publish_image(self):

        # Read image from camera
        frame = self.cap.get_frame_robust()
	capture_time = rospy.Time.now()

        # Convert numpy image to ROS image message
        image_msg = self.bridge.cv2_to_imgmsg(frame.gray, encoding=self.encoding)

        # Add timestamp and sequence number (empty by default)
        image_msg.header.stamp = capture_time
        image_msg.header.seq = self.seq

        self.image_publisher.publish(image_msg)
        camera_msg = self.camera_info
        camera_msg.header = image_msg.header  # Copy header from image message
        self.camera_publisher.publish(camera_msg)

        if self.seq == 0:
            rospy.loginfo("Publishing images from UVC Cam at '/{}/{}' "
                          .format(DEFAULT_CAMERA_NAME,self.image_topic))
        self.seq += 1


def main():

    uvc_cam_node = UVCCamNode()

    while not rospy.is_shutdown():
        uvc_cam_node.read_and_publish_image()
	#uvc_cam_node.rate.sleep()

    uvc_cam_node.cap.close()


if __name__ == '__main__':
    main()
