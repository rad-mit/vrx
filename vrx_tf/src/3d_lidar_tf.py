#!/usr/bin/env python  
import rospy
import math
import tf
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import LaserScan

laser_point = Point()

def laser_callback(msg):
    
    global laser_point  
    laser_point.x = msg.ranges[370]


if __name__ == '__main__':
    rospy.init_node('3d_lidar_tf')

    listener = tf.TransformListener()
    listener.waitForTransform("/robot1_tf/laser_sensor_link", "/robot2_tf/odom", rospy.Time(0),rospy.Duration(5.0))

    laser_sub = rospy.Subscriber('/kobuki/laser/scan', LaserScan, laser_callback)
    point_pub = rospy.Publisher('/new_point', PointStamped,queue_size=1)

    rate = rospy.Rate(10.0)
    global laser_point
    
    while not rospy.is_shutdown():
            
        laser_point_msg=PointStamped()
        laser_point_msg.header.frame_id = "/robot1_tf/laser_sensor_link"
        laser_point_msg.header.stamp =rospy.Time(0)
        laser_point_msg.point = laser_point
        
        try:
            p=listener.transformPoint("/robot2_tf/odom",laser_point_msg)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
            
        point_pub.publish(p)
        rate.sleep()
            