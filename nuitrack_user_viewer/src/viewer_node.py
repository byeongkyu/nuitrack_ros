#!/usr/bin/env python
# -*- encoding: utf8 -*-

import rospy
import cv2
import threading
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from nuitrack_msgs.msg import UserDataArray
from nuitrack_msgs.msg import SkeletonDataArray

class UserViewer:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.RLock()

        with self.lock:
            self.user_data = UserDataArray()

        self.img_sub = rospy.Subscriber('/nuitrack/rgb/image_raw', Image, self.handle_raw_image)
        self.user_sub = rospy.Subscriber('/nuitrack/detected_users', UserDataArray, self.handle_user_data)
        self.skeleton_sub = rospy.Subscriber('/nuitrack/skeletons', SkeletonDataArray, self.handle_skeleton_data)
        self.viz_user_pub = rospy.Publisher('/nuitrack/viz_user_markers', MarkerArray, queue_size=10)
        self.viz_skeleton_pub = rospy.Publisher('/nuitrack/viz_skeleton_markers', MarkerArray, queue_size=10)

    def handle_raw_image(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        with self.lock:
            data = self.user_data

        if len(data.users) > 0:
            for user in data.users:
                x1 = user.box.x_offset
                y1 = user.box.y_offset
                x2 = user.box.x_offset + user.box.width
                y2 = user.box.y_offset + user.box.height

                hsv = np.uint8([[[user.id * 10, 255, 255]]])
                color = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0][0]
                render_color = (int(color[0]), int(color[1]), int(color[2]))
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), render_color , 1)
                cv2.rectangle(cv_image, (x1, y1), (x1+80, y1+20), render_color, -1)
                cv2.putText(cv_image, 'User_%d'%user.id, (x1+6, y1+16), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(1)

    def handle_user_data(self, msg):
        with self.lock:
            self.user_data = msg
            data = self.user_data

        markers = MarkerArray()

        for user in data.users:
            marker = Marker()

            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = 'nuitrack_link'
            marker.ns = 'User_%d'%user.id
            marker.id = user.id
            marker.type = marker.CUBE
            marker.action = marker.ADD

            marker.pose.position.x = user.real.z / 1000.0
            marker.pose.position.y = -1 * user.real.x / 1000.0
            marker.pose.position.z = user.real.y / 1000.0

            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 1.5

            hsv = np.uint8([[[user.id * 10, 255, 255]]])
            color = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)[0][0]

            marker.color.a = 0.5
            marker.color.r = color[0] / 255.0
            marker.color.g = color[1] / 255.0
            marker.color.b = color[2] / 255.0

            markers.markers.append(marker)

        self.viz_user_pub.publish(markers)

    def handle_skeleton_data(self, msg):

        for skeleton in msg.skeletons:
            markers = MarkerArray()

            for i in range(len(skeleton.joints)):
                marker = Marker()

                marker.header.stamp = rospy.Time.now()
                marker.header.frame_id = 'nuitrack_link'
                marker.ns = 'User_%d'%skeleton.id + '_%s'%skeleton.joints[i]
                marker.id = skeleton.id
                marker.type = marker.SPHERE
                marker.action = marker.ADD

                marker.pose.position.x = skeleton.joint_pos[i].z / 1000.0
                marker.pose.position.y = -1 * skeleton.joint_pos[i].x / 1000.0
                marker.pose.position.z = skeleton.joint_pos[i].y / 1000.0

                marker.pose.orientation.w = 1.0

                if skeleton.joints[i] == 'joint_head':
                    marker.scale.x = marker.scale.y = marker.scale.z = 0.16
                else:
                    marker.scale.x = marker.scale.y = marker.scale.z = 0.04

                hsv = np.uint8([[[skeleton.id * 10, 255, 255]]])
                color = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)[0][0]

                marker.color.a = 0.5
                marker.color.r = color[0] / 255.0
                marker.color.g = color[1] / 255.0
                marker.color.b = color[2] / 255.0

                markers.markers.append(marker)

            draw_joint_lines = [
                [skeleton.joints.index('joint_head'), skeleton.joints.index('joint_neck'), skeleton.joints.index('joint_torso'), skeleton.joints.index('joint_waist')],
                [skeleton.joints.index('joint_left_collar'), skeleton.joints.index('joint_left_shoulder'), skeleton.joints.index('joint_left_elbow'), skeleton.joints.index('joint_left_wrist'), skeleton.joints.index('joint_left_hand')],
                [skeleton.joints.index('joint_right_collar'), skeleton.joints.index('joint_right_shoulder'), skeleton.joints.index('joint_right_elbow'), skeleton.joints.index('joint_right_wrist'), skeleton.joints.index('joint_right_hand')],
                [skeleton.joints.index('joint_left_ankle'), skeleton.joints.index('joint_left_knee'), skeleton.joints.index('joint_left_hip'), skeleton.joints.index('joint_waist'), skeleton.joints.index('joint_right_hip'), skeleton.joints.index('joint_right_knee'), skeleton.joints.index('joint_right_ankle')],
            ]

            for i in range(len(draw_joint_lines)):
                line = Marker()

                line.header.stamp = rospy.Time.now()
                line.header.frame_id = 'nuitrack_link'
                line.ns = 'User_%d'%skeleton.id + '_line%d'%i
                line.id = skeleton.id
                line.type = marker.LINE_STRIP
                line.action = marker.ADD

                line.scale.x = 0.008
                line.color.g = 0.6
                line.color.a = 1.0

                for j in draw_joint_lines[i]:
                    p = Point()
                    p.x = skeleton.joint_pos[j].z / 1000.0
                    p.y = -1 * skeleton.joint_pos[j].x / 1000.0
                    p.z = skeleton.joint_pos[j].y / 1000.0

                    line.points.append(p)
                markers.markers.append(line)

            self.viz_skeleton_pub.publish(markers)


if __name__ == '__main__':
    rospy.init_node('nuitrack_user_viwer', anonymous=False)
    m = UserViewer()
    rospy.spin()
