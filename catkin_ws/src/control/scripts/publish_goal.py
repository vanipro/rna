#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import tf

def publish_goal():
    rospy.init_node('goal_publisher', anonymous=True)
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    
    # Wait for move_base to be ready
    rospy.sleep(5.0)
    
    for i in range(5):
        goal = PoseStamped()
        goal.header.frame_id = "world"
        goal.header.stamp = rospy.Time.now()
        
        # Set position
        goal.pose.position.x = 0.2
        goal.pose.position.y = 2.7
        goal.pose.position.z = 0.0
        
        # Set orientation (facing forward)
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        
        # Alternative: Set orientation using yaw angle
        # quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)  # 0 radians yaw
        # goal.pose.orientation.x = quaternion[0]
        # goal.pose.orientation.y = quaternion[1]
        # goal.pose.orientation.z = quaternion[2]
        # goal.pose.orientation.w = quaternion[3]
        
        pub.publish(goal)
        rospy.loginfo("Published goal: x=%.2f, y=%.2f", goal.pose.position.x, goal.pose.position.y)
        
        # Keep the node alive briefly to ensure message is sent
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        publish_goal()
    except rospy.ROSInterruptException:
        pass