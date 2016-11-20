#!/usr/bin/env python
'''
Pupil_ROS_Bridge Plugin
- Start ROS node to publish gaze_positions, pupil_positions, and world image
- GUI-enabled control over ROS topics
- Using customized ROS msg: gaze_positions, gaze, pupil and pupil_positions
- Eye image is not supported in the plugin

Author: Long Qian
Email: lqian8@jhu.edu

'''

import sys
import roslib
roslib.load_manifest('pupil_ros_plugin')
import rospy

from cv_bridge import CvBridge
import cv2
import numpy as np

from pupil_ros_plugin.msg import gaze_positions, gaze, pupil, pupil_positions
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image

from plugin import Plugin
from pyglui.cygl.utils import draw_points_norm,RGBA
from pyglui import ui
from glfw import *


def tupleToPoint(tup):
    '''
    Convert the first three elements of a tuple to ROS Geometry Point Message
    :param tup: A tuple, of arbitrary length, or None
    :return: Point message
    '''
    p = Point()
    if tup:
        l = len(tup)
        if l == 0:
            return p
        elif l == 1:
            p.x = tup[0]
            return p
        elif l == 2:
            p.x, p.y = tup[0], tup[1]
        else:
            p.x, p.y, p.z = tup[0], tup[1], tup[2]
    return p



class Pupil_ROS_Bridge(Plugin):
    '''
    Pupil_ROS_Bridge that interacts with ROS environment
    '''

    def __init__(self, g_pool):
        '''
        Init necessary parameters of ROS connection, and GUI elements
        ROS node is not started here

        :param g_pool: Global Container
        '''
        super(Pupil_ROS_Bridge, self).__init__(g_pool)
        self.g_pool = g_pool
        # The update function of this Plugin is executed after other plugins
        self.order = .9
        self.menu = None
        self.ros_gaze_publisher = None
        self.ros_pupil_publisher = None
        self.ros_world_img_publisher = None
        self.ros_started = False
        self.gaze_publish_flag = False
        self.pupil_publish_flag = False
        self.world_img_publish_flag = False
        self.cv_bridge = CvBridge()
        self.ros_button_position = None
        self.ros_start_button = None
        self.ros_stop_button = None
        self.seq = 0
        self.bgr_frame_publisher_started = False


    def init_gui(self):
        '''
        GUI elements to control ROS communication
            - Start / Stop ROS node button
            - Gaze information publishing switch
            - Pupil information publishing switch
            - World image publishing switch
            - Remove the plugin itself
        :return: Nothing
        '''
        self.menu = ui.Growing_Menu('Pupil ROS Bridge')
        self.g_pool.sidebar.append(self.menu)
        self.menu.append(ui.Info_Text('A plugin to publish pupil data to ROS environment'))
        self.ros_start_button = ui.Button('Start ROS node', lambda: self.init_ros())
        self.ros_stop_button = ui.Button('Stop ROS node', lambda: self.stop_ros())
        self.ros_button_position = len(self.menu)
        self.menu.append(self.ros_start_button)
        self.menu.append(ui.Switch('gaze_publish_flag',self, label='Publish gaze data'))
        self.menu.append(ui.Switch('pupil_publish_flag',self, label='Publish pupil data'))
        self.menu.append(ui.Switch('world_img_publish_flag',self, label='Publish world image'))
        self.menu.append(ui.Button('Remove plugin', lambda:self.remove_plugin()))


    def init_ros(self):
        '''
        Init ROS node, three publishers on three topics: gaze, pupil, and world
        :return:
        '''
        try:
            rospy.init_node('Pupil_ROS_Bridge', anonymous=True)
            self.ros_gaze_publisher = rospy.Publisher('/pupil_capture/gaze', gaze_positions, queue_size=10)
            self.ros_pupil_publisher = rospy.Publisher('/pupil_capture/pupil', pupil_positions, queue_size=10)
            self.ros_world_img_publisher = rospy.Publisher('/pupil_capture/world', Image, queue_size=2)
            self.ros_started = True
            self.seq = 0
            self.menu[self.ros_button_position] = self.ros_stop_button
        except rospy.ROSInterruptException as e:
            self.gaze_publish_flag = False
            self.pupil_publish_flag = False
            self.world_img_publish_flag = False
            self.ros_started = False
            self.seq = 0
            print "Unable to start ros node:", e

    def stop_ros(self):
        '''
        Stop ROS node, reset parameters related to ROS and pupil GUI
        :return:
        '''
        if self.ros_started:
            rospy.signal_shutdown('Stop button pressed')
            self.ros_gaze_publisher = None
            self.ros_pupil_publisher = None
            self.ros_world_img_publisher = None
            self.ros_started = False
            self.seq = 0
            self.gaze_publish_flag = False
            self.pupil_publish_flag = False
            self.world_img_publish_flag = False
            self.menu[self.ros_button_position] = self.ros_start_button
            print 'ROS node shutdown'


    def update(self,frame=None,events={}):
        '''
        Update function being called by main event loop of world.py
            - Publish Gaze information if ROS started, and gaze_publisher enabled
            - Publish Pupil information if ROS started, and pupil_publisher enabled
            - Publish World image if ROS started, Frame_Publisher plugin enabled,
                and world_img_publisher enabled
        :param frame: image of frame
        :param events:  'dt': delta time between frames
                        'pupil_positions': pupil detection algorithm output
                        'gaze_positions': gaze mapping results combining pupil_positions
                                    and calibration
                        'frame.world': world image
        :return: Nothing
        '''
        if self.ros_started:
            if not rospy.is_shutdown():
                header = Header()
                header.seq = self.seq
                header.stamp = rospy.get_rostime()
                header.frame_id = str(self.g_pool.app) + ' ' + str(self.g_pool.version)
                if self.gaze_publish_flag:
                    # gaze_positions message
                    gaze_msg = gaze_positions()
                    gaze_msg.dt = events['dt']
                    gaze_msg.mapper = self.g_pool.active_gaze_mapping_plugin.__class__.__name__
                    gaze_info_list = []
                    for e in events['gaze_positions']:
                        gaze_info = gaze()
                        gaze_info.confidence = e['confidence']
                        gaze_info.norm_pos = tupleToPoint(e.get('norm_pos'))
                        gaze_info.gaze_point_3d = tupleToPoint(e.get('gaze_point_3d'))
                        gaze_info.gaze_normal_3d = tupleToPoint(e.get('gaze_normal_3d'))
                        gaze_info.eye_center_3d = tupleToPoint(e.get('eye_center_3d'))
                        gaze_info.pupil_timestamp = e['timestamp']
                        gaze_info_list.append(gaze_info)
                    gaze_msg.gazes = gaze_info_list
                    gaze_msg.header = header
                    self.ros_gaze_publisher.publish(gaze_msg)
                if self.pupil_publish_flag:
                    # pupil_positions message
                    pupil_msg = pupil_positions()
                    pupil_msg.dt = events['dt']
                    pupil_msg.header = header
                    pupil_info_list = []
                    for e in events['pupil_positions']:
                        pupil_info = pupil()
                        pupil_info.diameter = e['diameter']
                        pupil_info.confidence = e['confidence']
                        pupil_info.projected_sphere_axes = tupleToPoint(e['projected_sphere'].get('axes'))
                        pupil_info.projected_sphere_angle = e['projected_sphere'].get('angle')
                        pupil_info.projected_sphere_center = tupleToPoint(e['projected_sphere'].get('center'))
                        pupil_info.model_id = e['model_id']
                        pupil_info.model_confidence = e['model_confidence']
                        pupil_info.pupil_timestamp = e['timestamp']
                        pupil_info.model_birth_timestamp = e['model_birth_timestamp']
                        pupil_info.topic = e['topic']
                        pupil_info.sphere_radius = e['sphere'].get('radius')
                        pupil_info.sphere_center = tupleToPoint(e['sphere'].get('center'))
                        pupil_info.diameter_3d = e['diameter_3d']
                        pupil_info.ellipse_axes = tupleToPoint(e['ellipse'].get('axes'))
                        pupil_info.ellipse_angle = e['ellipse'].get('angle')
                        pupil_info.ellipse_center= tupleToPoint(e['ellipse'].get('center'))
                        pupil_info.norm_pos = tupleToPoint(e['norm_pos'])
                        pupil_info.phi = e['phi']
                        pupil_info.theta = e['theta']
                        pupil_info.circle_3d_radius = e['circle_3d'].get('radius')
                        pupil_info.circle_3d_center = tupleToPoint(e['circle_3d'].get('center'))
                        pupil_info.circle_3d_normal = tupleToPoint(e['circle_3d'].get('normal'))
                        pupil_info.id = e['id']
                        pupil_info_list.append(pupil_info)
                    pupil_msg.pupils = pupil_info_list
                    self.ros_pupil_publisher.publish(pupil_msg)
                if self.bgr_frame_publisher_started:
                    if 'frame.world' in events and self.world_img_publish_flag:
                        world_image_msg = self.cv_bridge.cv2_to_imgmsg(events['frame.world'][0]['__raw_data__'][0], encoding="bgr8")
                        world_image_msg.header = header
                        self.ros_world_img_publisher.publish(world_image_msg)
                else:
                    self.world_img_publish_flag = False

            else:
                self.ros_started = False
        self.seq += 1


    def gl_display(self):
        pass

    def on_click(self,pos,button,action):
        pass


    def on_window_resize(self,window,w,h):
        pass

    def on_notify(self, notification):
        '''
        Tune the parameter self.bgr_frame_publisher_started, to enable world
        image publish behaviour
        :param notification:
        :return:
        '''
        if notification['subject'].startswith('frame_publishing.started'):
            if notification['format'] == 'bgr':
                self.bgr_frame_publisher_started = True
            else:
                self.bgr_frame_publisher_started = False
        if notification['subject'].startswith('frame_publishing.stopped'):
            self.bgr_frame_publisher_started = False



    def get_init_dict(self):
        return {}


    def cleanup(self):
        self.deinit_gui()


    def deinit_gui(self):
        '''
        Shutdown ROS node and remove GUI elements, called when the plugin is not alive
        :return:
        '''
        self.stop_ros()
        if self.menu:
            self.g_pool.sidebar.remove(self.menu)
            self.menu = None


    def remove_plugin(self):
        '''
        Set the plugin to be not alive, will be GCed by main event loop
        :return:
        '''
        self.stop_ros()
        self.alive = False
