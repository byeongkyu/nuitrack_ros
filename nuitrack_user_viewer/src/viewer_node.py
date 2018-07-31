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
        self.viz_skeleton_pub1 = rospy.Publisher('/nuitrack/viz_skeleton_marker', Marker, queue_size=10)

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

            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
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
                    marker.scale.x = 0.16
                    marker.scale.y = 0.16
                    marker.scale.z = 0.16
                else:
                    marker.scale.x = 0.04
                    marker.scale.y = 0.04
                    marker.scale.z = 0.04


                hsv = np.uint8([[[skeleton.id * 10, 255, 255]]])
                color = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)[0][0]

                marker.color.a = 0.5
                marker.color.r = color[0] / 255.0
                marker.color.g = color[1] / 255.0
                marker.color.b = color[2] / 255.0

                markers.markers.append(marker)

            line1 = Marker()

            line1.header.stamp = rospy.Time.now()
            line1.header.frame_id = 'nuitrack_link'
            line1.ns = 'User_%d'%skeleton.id + '_line1'
            line1.id = skeleton.id + skeleton.id * 20
            line1.type = marker.LINE_STRIP
            line1.action = marker.ADD

            line1.pose.orientation.w = 1.0

            line1.scale.x = 0.02
            line1.color.g = 1.0
            line1.color.a = 1.0

            for j in [skeleton.joints.index('joint_head'), skeleton.joints.index('joint_neck'), skeleton.joints.index('joint_torso'), skeleton.joints.index('joint_waist')]:
                p = Point()
                p.x = skeleton.joint_pos[j].z / 1000.0
                p.y = -1 * skeleton.joint_pos[j].x / 1000.0
                p.z = skeleton.joint_pos[j].y / 1000.0

                line1.points.append(p)

            markers.markers.append(line1)

            line2 = Marker()

            line2.header.stamp = rospy.Time.now()
            line2.header.frame_id = 'nuitrack_link'
            line2.ns = 'User_%d'%skeleton.id + '_line2'
            line2.id = skeleton.id + skeleton.id * 30
            line2.type = marker.LINE_STRIP
            line2.action = marker.ADD

            line2.scale.x = 0.02
            line2.color.g = 1.0
            line2.color.a = 1.0

            for j in [skeleton.joints.index('joint_left_collar'), skeleton.joints.index('joint_left_shoulder'), skeleton.joints.index('joint_left_elbow'), skeleton.joints.index('joint_left_wrist'), skeleton.joints.index('joint_left_hand')]:
                p = Point()
                p.x = skeleton.joint_pos[j].z / 1000.0
                p.y = -1 * skeleton.joint_pos[j].x / 1000.0
                p.z = skeleton.joint_pos[j].y / 1000.0

                line2.points.append(p)

            markers.markers.append(line2)

            line3 = Marker()

            line3.header.stamp = rospy.Time.now()
            line3.header.frame_id = 'nuitrack_link'
            line3.ns = 'User_%d'%skeleton.id + '_line3'
            line3.id = skeleton.id + skeleton.id * 40
            line3.type = marker.LINE_STRIP
            line3.action = marker.ADD

            line3.scale.x = 0.02
            line3.color.g = 1.0
            line3.color.a = 1.0

            for j in [skeleton.joints.index('joint_right_collar'), skeleton.joints.index('joint_right_shoulder'), skeleton.joints.index('joint_right_elbow'), skeleton.joints.index('joint_right_wrist'), skeleton.joints.index('joint_right_hand')]:
                p = Point()
                p.x = skeleton.joint_pos[j].z / 1000.0
                p.y = -1 * skeleton.joint_pos[j].x / 1000.0
                p.z = skeleton.joint_pos[j].y / 1000.0

                line3.points.append(p)

            markers.markers.append(line3)

            line4 = Marker()

            line4.header.stamp = rospy.Time.now()
            line4.header.frame_id = 'nuitrack_link'
            line4.ns = 'User_%d'%skeleton.id + '_line4'
            line4.id = skeleton.id + skeleton.id * 50
            line4.type = marker.LINE_STRIP
            line4.action = marker.ADD

            line4.scale.x = 0.02
            line4.color.g = 1.0
            line4.color.a = 1.0

            for j in [skeleton.joints.index('joint_left_ankle'), skeleton.joints.index('joint_left_knee'), skeleton.joints.index('joint_left_hip'), skeleton.joints.index('joint_waist'), skeleton.joints.index('joint_right_hip'), skeleton.joints.index('joint_right_knee'), skeleton.joints.index('joint_right_ankle')]:
                p = Point()
                p.x = skeleton.joint_pos[j].z / 1000.0
                p.y = -1 * skeleton.joint_pos[j].x / 1000.0
                p.z = skeleton.joint_pos[j].y / 1000.0

                line4.points.append(p)

            markers.markers.append(line4)
            self.viz_skeleton_pub.publish(markers)


if __name__ == '__main__':
    rospy.init_node('nuitrack_user_viwer', anonymous=False)
    m = UserViewer()
    rospy.spin()
