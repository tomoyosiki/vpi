#!/usr/bin/env python
# --------------- forward and park -----------------
# import rospy
# from geometry_msgs.msg import PoseStamped
# from geometry_msgs.msg import Twist
# import sensor_msgs.point_cloud2 as pc2
# from sensor_msgs.msg import PointCloud2
# import numpy as np
# import math
# import statistics

# # Global variables
# pub = None
# start_time = None
# current_state = 'moving_forward'  # Initial state
# global pub, start_time, current_state

# def control(msg):
#     # Initialize the twist message
#     twist = Twist()

#     x = msg.pose.position.x
#     y = msg.pose.position.y

#     if current_state == 'moving_forward':
#         if nearest_distance > 3:  
#             twist.linear.x = 0.4 
#         elif nearest_distance < 3 and nearest_distance > 2.9 :
#             twist.linear.x = 0  
#             current_state = 'turning' 
#             start_time = rospy.get_time()  
#         else:
#             twist.linear.x = -0.1 

#     elif current_state == 'turning':
#         if rospy.get_time() - start_time < 9.1:
#             twist.angular.z = -0.22 
#             twist.linear.x = 0.3
#         else:
#             # After turning, stop the vehicle and reset variables
#             current_state = 'finding'  # Update state to stop
#             twist.angular.z = 0
#             twist.linear.x = 0
#             start_time = None # Reset start time

#     elif current_state == 'finding':
#         if nearest_distance > 1.5:  
#             twist.linear.x = 0.4 
#         elif nearest_distance < 1.5 and nearest_distance > 1.4 :
#             twist.linear.x = 0  
#             current_state = 'parking' 
#             start_time = rospy.get_time()  
#         else:
#             twist.linear.x = -0.1 
    
#     elif current_state == 'parking':
#         current_time = rospy.get_time()
#         if current_time - start_time <= 5:
#             twist.angular.z = 0.22 
#             twist.linear.x = 0.2
#         elif current_time - start_time > 5 and current_time - start_time < 9.8:
#             twist.angular.z = -0.22 
#             twist.linear.x = 0.2
#         else:
#             current_state = 'stop'  # Update state to stop
#             twist.angular.z = 0
#             twist.linear.x = 0
#             start_time = None # Reset start time
                
#         else:
#             twist.angular.z = 0
#             twist.linear.x = 0
#     # Publish the twist message
#     pub.publish(twist)
#     rospy.loginfo(f'Current State: {current_state}, Command speed: {twist.linear.x}, Command angular: {twist.angular.z}')

# if __name__ == '__main__':
#     try:
#         rospy.init_node('vehicle_control_node', anonymous=True)
#         rospy.init_node('pose_listener', anonymous=True)
#         rospy.Subscriber("ndt_pose", PoseStamped, pose_callback)
#         pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass



# --------------- AutoPark -----------------
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

# Global variables
pub = None
start_time = None
end_time = None
current_time = None
current_state = 'moving_forward'  # Initial state

def control(msg):
    global pub, current_state, start_time, end_time, current_time
    # Initialize the twist message
    twist = Twist()

    x = msg.pose.position.x
    y = msg.pose.position.y

    if current_state == 'moving_forward':
        if x < -0.44:
            # if y < 5.91:
            #     twist.angular.z = 0.2 
            #     twist.linear.x = 0.1
            # elif y > 5.92 :
            #     twist.angular.z = -0.2 
            #     twist.linear.x = 0.1
            # else:
            twist.linear.x = 0.4 # forward
        elif x > -0.43:
            twist.linear.x = -0.1 
        else:
            twist.linear.x = 0
            current_state = 'turning'
            start_time = rospy.get_time()

    elif current_state == 'turning':
x
        if x > -1.81:
            twist.angular.z = -0.22 
            twist.linear.x = -0.2
        else:
            # After turning, stop the vehicle and reset variables
            current_state = 'parking'  
            twist.angular.z = 0
            twist.linear.x = 0
            end_time = rospy.get_time() 

    
    elif current_state == 'parking':
        # if x > -2.9 and y < 6.76:
        #     twist.angular.z = 0.22 
        #     twist.linear.x = -0.2
        # else:
        #     current_state = 'stop'  
        #     twist.angular.z = 0
        #     twist.linear.x = 0
        current_time = rospy.get_time()
        if current_time - end_time < end_time - start_time:
            twist.angular.z = 0.22 
            twist.linear.x = -0.2
        else:
            current_state = 'stop'  
            twist.angular.z = 0
            twist.linear.x = 0

                
    if current_state == 'stop':
            twist.angular.z = 0
            twist.linear.x = 0
    # Publish the twist message
    pub.publish(twist)
    rospy.loginfo('Current State: {}, Command speed: {}, Command angular: {}'.format(current_state, twist.linear.x, twist.angular.z))

if __name__ == '__main__':
    try:
        rospy.init_node('vehicle_control_node', anonymous=True)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("ndt_pose", PoseStamped, control)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



