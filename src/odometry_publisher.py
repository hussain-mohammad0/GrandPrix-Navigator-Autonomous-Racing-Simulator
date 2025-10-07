#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class OdometryPublisher:
    def __init__(self):
        rospy.init_node('odometry_publisher', anonymous=True)
        
        self.odom_pub = rospy.Publisher('/car_1/amcl/odom', Odometry, queue_size=10)
        
        rospy.Subscriber('/car_1/amcl/pose', PoseWithCovarianceStamped, self.pose_callback)
        
    def pose_callback(self, pose_msg):
        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.header.frame_id = 'world'
        odom_msg.child_frame_id = 'car_1/amcl/base_link'
        
        odom_msg.pose.pose = pose_msg.pose.pose
        
        self.odom_pub.publish(odom_msg)
        
if __name__ == '__main__':
    odometry_publisher = OdometryPublisher()
    rospy.spin()