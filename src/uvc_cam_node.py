#!/usr/bin/env python
"""Publishes UVC Cam images as ROS messages."""

import uvc
import sys
import numpy as np
import cv2
import rospy
from platform import python_version
from camera_info_manager import CameraInfoManager
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

# dynamic reconfigure 
from dynamic_reconfigure.server import Server
from uvc_cam.cfg import uvc_camConfig

DEFAULT_CAMERA_NAME = 'uvc_cam'
DEFAULT_IMAGE_TOPIC = 'image_raw'
DEFAULT_CAMERA_TOPIC = 'camera_info'
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_FPS = 90


class UVCCamNode:

    def __init__(self):

        rospy.init_node('{}_node'.format(DEFAULT_CAMERA_NAME), argv=sys.argv)

        self.image_topic    = rospy.get_param('~image', DEFAULT_IMAGE_TOPIC)
        self.camera_topic   = rospy.get_param('~camera', DEFAULT_CAMERA_TOPIC)
        self.calibration    = rospy.get_param('~calibration', '')
        self.encoding       = rospy.get_param('~encoding','mono8')
        self.width          = rospy.get_param('~width', DEFAULT_WIDTH)
        self.height         = rospy.get_param('~height', DEFAULT_HEIGHT)
        self.fps            = rospy.get_param('~fps', DEFAULT_FPS)
        self.cam_prod_id    = rospy.get_param('~cam_prod_id', 9760)

	self.contrast       = rospy.get_param('~contrast', 0)
	self.sharpness      = rospy.get_param('~sharpness', 1)
        self.gamma          = rospy.get_param('~gamma',80)
        self.exposure_abs   = rospy.get_param('~exposure',50) #50 - 
        self.brightness     = rospy.get_param('~brightness',1)
        self.hue            = rospy.get_param('~hue',0)
        self.saturation     = rospy.get_param('~saturation',0)
        self.pwr_line_freq  = rospy.get_param('~pwr_line_freq',1)
        self.backlight_comp = rospy.get_param('~backlight_comp',0)
        self.auto_exposure  = rospy.get_param('~auto_exposure',8)
        self.auto_exp_prio  = rospy.get_param('~auto_exp_prio',0)
        self.white_bal_auto = rospy.get_param('~white_bal_auto',True)
        self.white_bal      = rospy.get_param('~white_bal',4600)

	# possible values to set can be found by running "rosrun uvc_camera uvc_camera_node _device:=/dev/video0", see http://github.com/ros-drivers/camera_umd.git
	dev_list = uvc.device_list()
	for i in range(0,len(dev_list)):
	  print dev_list[i]
          rospy.loginfo('available device %i: idProd: %i   -   comparing to idProd: %i'%(i,int(dev_list[i]["idProduct"]),self.cam_prod_id))
	  if i == self.cam_prod_id: #int(dev_list[i]["idProduct"]) == self.cam_prod_id:
            rospy.loginfo("connecting to camera idProd: %i, device %i"%(self.cam_prod_id,i))
            self.cap = uvc.Capture(dev_list[i]["uid"])
            #self.cap.set(CV_CAP_PROP_CONVERT_RGB, false)
            rospy.loginfo("successfully connected to camera %i"%i)
        rospy.loginfo('starting cam at %ifps with %ix%i resolution'%(self.fps,self.width,self.height))
        self.cap.frame_mode = (self.width, self.height, self.fps)
	frame = self.cap.get_frame_robust()
	self.controls_dict = dict([(c.display_name, c) for c in self.cap.controls])
	self.controls_dict['Brightness'].value = self.brightness #10 #[-64,64], not 0 (no effect)!!
	self.controls_dict['Contrast'].value = self.contrast #0 #[0,95]
	self.controls_dict['Hue'].value = self.hue #[-2000,2000]
	self.controls_dict['Saturation'].value = self.saturation #[0,100]
	self.controls_dict['Sharpness'].value = self.sharpness #1 #[1,100]
	self.controls_dict['Gamma'].value = self.gamma #[80,300]
	self.controls_dict['Power Line frequency'].value = self.pwr_line_freq #1:50Hz, 2:60Hz
	self.controls_dict['Backlight Compensation'].value = self.backlight_comp #True or False
	self.controls_dict['Absolute Exposure Time'].value = self.exposure_abs #[78,10000] set Auto Exposure Mode to 1
	self.controls_dict['Auto Exposure Mode'].value = self.auto_exposure #1:manual, 8:apperturePriority
	self.controls_dict['Auto Exposure Priority'].value = self.auto_exp_prio #1:manual, 8:apperturePriority
	self.controls_dict['White Balance temperature,Auto'].value = self.white_bal_auto
	self.controls_dict['White Balance temperature'].value = self.white_bal #[2800,6500]
	rospy.loginfo("These camera settings will be applied:")
	for c in self.cap.controls:
		rospy.loginfo('%s: %i'%(c.display_name, c.value))

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
	self.counter = 0
	#self.rate = rospy.Rate(60)

    def callback(self, config, level):
        rospy.loginfo("""Reconfigure Request: \
          {exposure}, \
          {auto_exposure}, \
          {brightness}, \
          {backlight_comp}, \
          {gamma}, \
          {contrast}, \
          {sharpness}, \
          {hue} \
          {saturation} \
          {pwr_line_freq} \
          {auto_exp_prio} \
          {white_bal_auto} \
          {white_bal} \
          """.format(**config))
        self.controls_dict['Absolute Exposure Time'].value = config.exposure;
        self.controls_dict['Auto Exposure Mode'].value = config.auto_exposure;
        self.controls_dict['Brightness'].value = config.brightness;
        self.controls_dict['Backlight Compensation'].value = config.backlight_comp;
        self.controls_dict['Gamma'].value = config.gamma;
        self.controls_dict['Contrast'].value = config.contrast;
        self.controls_dict['Sharpness'].value = config.sharpness;
        self.controls_dict['Hue'].value = config.hue;
        self.controls_dict['Saturation'].value = config.saturation;
        self.controls_dict['Power Line frequency'].value = config.pwr_line_freq;
        self.controls_dict['Auto Exposure Priority'].value = config.auto_exp_prio;
        self.controls_dict['White Balance temperature,Auto'].value = config.white_bal_auto;
        self.controls_dict['White Balance temperature'].value = config.white_bal;

	for c in self.cap.controls:
		rospy.loginfo('%s: %i'%(c.display_name, c.value))
        return config

    def read_and_publish_image(self):

        # Read image from camera
        self.frame = self.cap.get_frame_robust()
        if (self.counter % 2) == 0:
            capture_time = rospy.Time.now()

            # Convert numpy image to ROS image message
            self.image_msg = self.bridge.cv2_to_imgmsg(self.frame.gray, encoding=self.encoding)

            # Add timestamp and sequence number (empty by default)
            self.image_msg.header.stamp = capture_time
            self.image_msg.header.seq = self.seq

            self.image_publisher.publish(self.image_msg)
            camera_msg = self.camera_info
            camera_msg.header = self.image_msg.header  # Copy header from image message
            self.camera_publisher.publish(camera_msg)

            if self.seq == 0:
                rospy.loginfo("Publishing images from UVC Cam at '/{}/{}' "
                            .format(DEFAULT_CAMERA_NAME,self.image_topic))
            self.seq += 1

	self.counter += 1


def main():

    uvc_cam_node = UVCCamNode()
    srv = Server(uvc_camConfig, uvc_cam_node.callback)

    while not rospy.is_shutdown():
        uvc_cam_node.read_and_publish_image()
	#uvc_cam_node.rate.sleep()

    uvc_cam_node.cap.close()


if __name__ == '__main__':
    main()
