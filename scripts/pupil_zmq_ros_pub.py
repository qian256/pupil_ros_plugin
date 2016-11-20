#!/usr/bin/env python
'''
Pupil_ZMQ_ROS Publisher
- ZMQ subscriber to receive gaze and pupil data
- Start ROS node to publish gaze_positions, pupil_positions, world image
    and eye images
- Using customized ROS msg: gaze_positions, gaze, pupil and pupil_positions

Author: Long Qian
Email: lqian8@jhu.edu

'''
import zmq
import sys
from msgpack import loads
import roslib
roslib.load_manifest('pupil_ros_plugin')
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from pupil_ros_plugin.msg import gaze_positions, gaze, pupil, pupil_positions
from std_msgs.msg import Header
from geometry_msgs.msg import Point


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


class Pupil_ZMQ_ROS(object):
    '''
    Pupil_ZMQ_ROS: a standalone class to interface Pupil ZMQ messages
    and ROS environment.
    '''

    def __init__(self, addr='localhost', req_port='50020'):
        '''
        Initialize Pupil_ZMQ_ROS object, init ZMQ and ROS
        :param addr: 'localhost' if pupil_capture or pupil_service is
                    running localy
        :param req_port: port number of Pupil_Remote plugin
        '''
        self.zmq_req = None
        self.zmq_sub = None
        self.ros_gaze_publisher = None
        self.ros_pupil_publisher = None
        self.ros_world_img_publisher = None
        self.ros_eye0_img_publisher = None
        self.ros_eye1_img_publisher = None
        self.cv_bridge = CvBridge()
        self.ros_started = True
        self.seq = 0
        self.init_zmq(addr, req_port)
        self.init_ros()



    def init_zmq(self, addr, req_port):
        '''
        Initialize ZMQ subscriber
        :param addr: parsed from class constructor
        :param req_port: parsed from class constructor
        :return:
        '''
        context = zmq.Context()
        self.zmq_req = context.socket(zmq.REQ)
        self.zmq_req.connect("tcp://%s:%s" %(addr,req_port))
        # ask for the sub port
        self.zmq_req.send('SUB_PORT')
        sub_port = self.zmq_req.recv()
        # open a sub port to listen to pupil
        self.zmq_sub = context.socket(zmq.SUB)
        self.zmq_sub.connect("tcp://%s:%s" %(addr,sub_port))
        # set subscriptions to topics
        # recv just pupil/gaze/frame/notifications
        self.zmq_sub.setsockopt(zmq.SUBSCRIBE, 'pupil.')
        self.zmq_sub.setsockopt(zmq.SUBSCRIBE, 'gaze')
        self.zmq_sub.setsockopt(zmq.SUBSCRIBE, 'frame.')
        # self.zmq_sub.setsockopt(zmq.SUBSCRIBE, 'notify.')
        # self.zmq_sub.setsockopt(zmq.SUBSCRIBE, 'logging.')
        # or everything
        # sub.setsockopt(zmq.SUBSCRIBE, '')
        print 'Pupil_ZMQ_ROS: zmq environment initialized'


    def init_ros(self):
        '''
        Initialize ROS node, two publishers on two topics: gaze and pupil
        :return:
        '''
        try:
            rospy.init_node('Pupil_ZMQ_ROS', anonymous=True)
            self.ros_gaze_publisher = rospy.Publisher('/pupil_capture/gaze', gaze_positions, queue_size=10)
            self.ros_pupil_publisher = rospy.Publisher('/pupil_capture/pupil', pupil_positions, queue_size=10)
            self.ros_world_img_publisher = rospy.Publisher('/pupil_capture/world', Image, queue_size=2)
            self.ros_eye0_img_publisher = rospy.Publisher('/pupil_capture/eye/0', Image, queue_size=2)
            self.ros_eye1_img_publisher = rospy.Publisher('/pupil_capture/eye/1', Image, queue_size=2)
            self.ros_started = True
            self.seq = 0
            print 'Pupil_ZMQ_ROS: ros environment initialized'
        except rospy.ROSInterruptException as e:
            self.ros_started = False
            self.seq = 0
            print 'Pupil_ZMQ_ROS: unable to start ros node:', e


    def spin(self):
        '''
        Spinning loop: receive ZMQ data and publish to ROS topics
        :return:
        '''
        if not self.ros_started:
            print 'Pupil_ZMQ_ROS: ros not started'
            return
        while True:
            # rospy.is_shutdown check inside while loop to enable Ctrl-C termination
            if rospy.is_shutdown():
                break
            # receive message from ZMQ subscriber
            zmq_multipart =  self.zmq_sub.recv_multipart()
            zmq_topic, zmq_raw_msg = zmq_multipart[0], zmq_multipart[1]
            # ROS header message
            header = Header()
            header.seq = self.seq
            header.stamp = rospy.get_rostime()
            header.frame_id = "Pupil_ZMQ_ROS"
            zmq_msg = loads(zmq_raw_msg)
            if 'pupil' in zmq_topic:
                # pupil data parser
                pupil_msg = pupil_positions()
                pupil_msg.header = header
                # the pupil_info_list contains only one pupil data to keep
                # the ROS message format same as Pupil_ROS_Bridge plugin
                pupil_info_list = []
                pupil_info = pupil()
                pupil_info.diameter = zmq_msg['diameter']
                pupil_info.confidence = zmq_msg['confidence']
                pupil_info.projected_sphere_axes = tupleToPoint(zmq_msg['projected_sphere'].get('axes'))
                pupil_info.projected_sphere_angle = zmq_msg['projected_sphere'].get('angle')
                pupil_info.projected_sphere_center = tupleToPoint(zmq_msg['projected_sphere'].get('center'))
                pupil_info.model_id = zmq_msg['model_id']
                pupil_info.model_confidence = zmq_msg['model_confidence']
                pupil_info.pupil_timestamp = zmq_msg['timestamp']
                pupil_info.model_birth_timestamp = zmq_msg['model_birth_timestamp']
                pupil_info.topic = zmq_msg['topic']
                pupil_info.sphere_radius = zmq_msg['sphere'].get('radius')
                pupil_info.sphere_center = tupleToPoint(zmq_msg['sphere'].get('center'))
                pupil_info.diameter_3d = zmq_msg['diameter_3d']
                pupil_info.ellipse_axes = tupleToPoint(zmq_msg['ellipse'].get('axes'))
                pupil_info.ellipse_angle = zmq_msg['ellipse'].get('angle')
                pupil_info.ellipse_center = tupleToPoint(zmq_msg['ellipse'].get('center'))
                pupil_info.norm_pos = tupleToPoint(zmq_msg['norm_pos'])
                pupil_info.phi = zmq_msg['phi']
                pupil_info.theta = zmq_msg['theta']
                pupil_info.circle_3d_radius = zmq_msg['circle_3d'].get('radius')
                pupil_info.circle_3d_center = tupleToPoint(zmq_msg['circle_3d'].get('center'))
                pupil_info.circle_3d_normal = tupleToPoint(zmq_msg['circle_3d'].get('normal'))
                pupil_info.id = zmq_msg['id']
                pupil_info_list.append(pupil_info)
                pupil_msg.pupils = pupil_info_list
                self.ros_pupil_publisher.publish(pupil_msg)
            if zmq_topic == 'gaze':
                # gaze data after combining pupil data and gaze mapping plugin
                gaze_msg = gaze_positions()
                # the gaze_info_list contains only one gaze data to keep
                # the ROS message format same as Pupil_ROS_Bridge plugin
                gaze_info_list = []
                gaze_info = gaze()
                gaze_info.confidence = zmq_msg['confidence']
                gaze_info.norm_pos = tupleToPoint(zmq_msg.get('norm_pos'))
                gaze_info.gaze_point_3d = tupleToPoint(zmq_msg.get('gaze_point_3d'))
                gaze_info.gaze_normal_3d = tupleToPoint(zmq_msg.get('gaze_normal_3d'))
                gaze_info.eye_center_3d = tupleToPoint(zmq_msg.get('eye_center_3d'))
                gaze_info.pupil_timestamp = zmq_msg['timestamp']
                gaze_info_list.append(gaze_info)
                gaze_msg.gazes = gaze_info_list
                gaze_msg.header = header
                self.ros_gaze_publisher.publish(gaze_msg)
            if 'frame.world' in zmq_topic:
                if zmq_msg['format'] == 'bgr':
                    # pupil eye.py and frame_publisher.py should be updated according to
                    # Issue: https://github.com/pupil-labs/pupil/issues/525
                    cv_img = np.frombuffer(zmq_multipart[2], dtype=np.uint8).reshape( zmq_msg['height'], zmq_msg['width'], 3)
                    world_image_msg = self.cv_bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
                    world_image_msg.header = header
                    self.ros_world_img_publisher.publish(world_image_msg)
            if 'frame.eye.0' in zmq_topic:
                if zmq_msg['format'] == 'bgr':
                    cv_img = np.frombuffer(zmq_multipart[2], dtype=np.uint8).reshape( zmq_msg['height'], zmq_msg['width'], 3)
                    eye0_image_msg = self.cv_bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
                    eye0_image_msg.header = header
                    self.ros_eye0_img_publisher.publish(eye0_image_msg)
            if 'frame.eye.1' in zmq_topic:
                if zmq_msg['format'] == 'bgr':
                    cv_img = np.frombuffer(zmq_multipart[2], dtype=np.uint8).reshape( zmq_msg['height'], zmq_msg['width'], 3)
                    eye1_image_msg = self.cv_bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
                    eye1_image_msg.header = header
                    self.ros_eye1_img_publisher.publish(eye1_image_msg)
        # Disable ROS interface
        self.ros_started = False



if __name__ == "__main__":
    '''
    >> python pupil_zmq_ros_pub.py #addr #port
    '''
    zmq_ros_pub = None
    if len(sys.argv) >= 3:
        zmq_ros_pub = Pupil_ZMQ_ROS(sys.argv[1], sys.argv[2])
    elif len(sys.argv) == 2:
        zmq_ros_pub = Pupil_ZMQ_ROS(sys.argv[1])
    else:
        zmq_ros_pub = Pupil_ZMQ_ROS()
    # Spinning on ZMQ messages, terminated by Ctrl-C
    zmq_ros_pub.spin()

