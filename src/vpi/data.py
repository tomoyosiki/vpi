# data_subscriber.py
import rospy
from std_msgs.msg import String
import threading
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from math import acos, sin, sqrt, atan,atan2, asin, cos, radians
import numpy as np

vehicle_id = 0
vehicles = {}

class PIDController:
    def __init__(self, p=0.1, i=0, d=0.001):
        self.kp = p
        self.ki = i
        self.kd = d
        self.integral = 0
        self.last_error = 0

    def compute(self, error, delta_time):
        self.integral += error * delta_time
        derivative = (error - self.last_error) / delta_time
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        # print([self.kp * error, self.ki * self.integral, self.kd * derivative])
        self.last_error = error
        return output


def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def calculate_angle_to_target(x, y, x_t, y_t):
    print([x, y, x_t, y_t])
    return atan2(y_t - y, x_t - x)

def is_facing_target(x, y, qx, qy, qz, qw, x_t, y_t, tolerance_deg=30):
    _, _, yaw = euler_from_quaternion(qx, qy, qz, qw)
    angle_to_target = calculate_angle_to_target(x, y, x_t, y_t)
    yaw_to_target = atan2(sin(angle_to_target - yaw), cos(angle_to_target - yaw))
    
    # Convert tolerance to radians
    tolerance = radians(tolerance_deg)

    print(yaw)
    print(angle_to_target)
    
    # Check if within tolerance
    return abs(yaw_to_target) <= tolerance



class Vehicle:
    def __init__(self):
        global vehicle_id

        rospy.init_node('vpi_data_listener', anonymous=True)
        self.ndt_pose = None

        self.id = vehicle_id
        vehicle_id += 1

        self.ndt_pose_lock = threading.Lock()

        # self.ndt_pose_sub = rospy.Subscriber("ndt_pose", PoseStamped, self.ndt_pose_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.sensor_apis = {}
        self.sensor_apis["ndt_pose"] = self.getNDTPose

        # rate = rospy.Rate(10)

        # Sleep for the duration
        # rate.sleep()
        # now2 = rospy.get_rostime()
        # rospy.loginfo("Finish subscribe %i %i", now2.secs, now2.nsecs)

    # def ndt_pose_callback(self, poseStamped_msg):
    #     with self.ndt_pose_lock:
    #         self.ndt_pose = poseStamped_msg

    def getLatestNDTPose(self):
        return self.ndt_pose


    def getNDTPose(self, *args):
        # output =None
        # with self.ndt_pose_lock:
        #     output = self.ndt_pose
        # return output
        self.ndt_pose = rospy.wait_for_message("ndt_pose", PoseStamped, timeout=None)
        return self.ndt_pose

    def get_sensor_api(self, sensorType):
        return self.sensor_apis[sensorType]

    def get_id(self):
        return self.id

    def send_cmd_vel(self, twist):
        self.cmd_vel_pub.publish(twist)


# Global instance to maintain state and initialization
vehicle = Vehicle()
vehicles[vehicle.get_id()] = vehicle

def sleep_ms(milliseconds):
    """
    Sleeps for the specified number of milliseconds.

    :param milliseconds: The number of milliseconds to sleep.
    """
    # rospy.Rate expects a frequency in hertz, so convert milliseconds to seconds and create a rate object
    seconds = milliseconds / 1000.0
    rate = rospy.Rate(1 / seconds)

    # Sleep for the duration
    rate.sleep()

def getSystime():
    return rospy.get_rostime()

def getSensorData(vehicleId, sensorType, sensorId, params):
    global vehicles

    vehicle = vehicles[vehicleId]
    api = vehicle.get_sensor_api(sensorType)

    return api(sensorId, params)

def controlVehicle(vehicleId, twist):
    global vehicles
    vehicle = vehicles[vehicleId]

    vehicle.send_cmd_vel(twist)

def MoveVehicleToPose(vehicleId, target_pose, pid_for_distance, pid_for_yaw):

    global vehicles
    vehicle = vehicles[vehicleId]

    pid_distance = PIDController(p=pid_for_distance[0], i=pid_for_distance[1], d=pid_for_distance[2])
    # pid_distance = PIDController(p=0.1, i=0, d=0.01)
    pid_yaw = PIDController(p=pid_for_yaw[0], i=pid_for_yaw[1], d=pid_for_yaw[2])
    #pid_yaw = PIDController(p=0.1, i=0, d=0.02)

    job_not_done = True
    twist = Twist()

    last_time = rospy.Time.now()

    # Extract target x, y, and orientation (yaw) from the PoseStamped
    target_x = target_pose[0]
    target_y = target_pose[1]
    qx = target_pose[3]
    qy = target_pose[4]
    qz = target_pose[5]
    qw = target_pose[6]

    _, _, target_yaw = euler_from_quaternion(qx, qy, qz, qw)
    print(f"target_yaw {target_yaw}")

    while job_not_done:
        ndt_pose = vehicle.getNDTPose()
        if ndt_pose is None:
            continue

        current_time = rospy.Time.now()
        delta_time = (current_time - last_time).to_sec()
        last_time = current_time

        current_x = ndt_pose.pose.position.x
        current_y = ndt_pose.pose.position.y
        current_orientation = ndt_pose.pose.orientation
        _, _, current_yaw = euler_from_quaternion(current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w)

        # Calculate distance and angle to target
        distance = sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        angle_to_target = atan2(target_y - current_y, target_x - current_x)
        angle_error1 = target_yaw - current_yaw
        angle_error2 = angle_to_target - current_yaw
        #angle_error = angle_error1 * 0.7 + angle_error2 * 0.3
        angle_error = angle_error1 * 1 + angle_error2 * 1
        # Normalize the angle error
        angle_error = atan2(sin(angle_error), cos(angle_error))

        # PID control for distance and yaw
        v = pid_distance.compute(distance, delta_time)
        omega = pid_yaw.compute(angle_error, delta_time)
        print([distance, angle_error, v, omega])
        # Velocity controls
        v = max(0.05, min(v, 0.4))  # Limit speed
        # omega = max(-0.4, min(omega, 0.4))  # Limit angular velocity

        if omega > 0:
            omega = max(0.025, min(omega, 0.4)) 
        else:
            omega = max(0.025, min(omega, 0.4)) * -1

        if (v < 0.05 and abs(omega) < 0.025) or (current_x < target_x):
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            job_not_done = False
        else:
            # Set velocities in the twist message
            twist.linear.x = v
            twist.angular.z = omega

        # Check if both position and orientation thresholds are met
        if distance < 0.1 and abs(angle_error1) < radians(2):  # Small thresholds to stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            job_not_done = False
            rospy.loginfo(f"Current position ({current_x}, {current_y}) and orientation {current_yaw} and distance {distance}, angle error {angle_error}")
            rospy.loginfo(f"Reached target position ({target_x}, {target_y}) and orientation {target_yaw}")

        vehicle.send_cmd_vel(twist)


def MoveVehicleStraight(vehicleId, target):
    global vehicles
    vehicle = vehicles[vehicleId]
    
    pid = PIDController(p=0.4, i=0.001, d=0.001)
    job_not_done = True
    twist = Twist()

    last_time = rospy.Time.now()

    ndt_pose = vehicle.getNDTPose()
    x = ndt_pose.pose.position.x
    y = ndt_pose.pose.position.y
    qx = ndt_pose.pose.orientation.x
    qy = ndt_pose.pose.orientation.y
    qz = ndt_pose.pose.orientation.z
    qw = ndt_pose.pose.orientation.w
    
    if is_facing_target(x, y, qx, qy, qz, qw, target, y):
        direction_modifier = 1
    else:
        direction_modifier = -1
    
    print(direction_modifier)
    while job_not_done:
        ndt_pose = vehicle.getNDTPose()
        if ndt_pose is None:
            continue
        
        current_time = rospy.Time.now()
        delta_time = (current_time - last_time).to_sec()
        last_time = current_time

        x = ndt_pose.pose.position.x
        error = abs(target - x)

        v = pid.compute(error, delta_time)
        #print(v)
        v = max(-0.4, min(v, 0.4))
        #print([error, v])
        if abs(error) < 0.01:  # Small threshold to stop
            v = 0.0
            job_not_done = False
            rospy.loginfo(f"Reached target {target} at position {x}")

        twist.linear.x = v * direction_modifier
        vehicle.send_cmd_vel(twist)

def calculateSShapeAngle(x,y):
    T = x /y
    a = T**2 + 1
    b = -2 * T**2
    c = T**2 -1
    delta = b**2 - 4 * a * c
    out2 = (-1 * b - sqrt(delta)) / (2*a)
    return acos(out2)

def calculateDesiredX(vehicleId, target):
    # x = (1.0, 1.732)
    # target x,y (1.50, -7.12))
    # current pose
    # w = 0.15, v = 0.22
    global vehicles
    vehicle = vehicles[vehicleId]

    twist = Twist()
    angular_spd = 0.15
    linear_spd = 0.22

    radius = linear_spd / angular_spd
    diff_x = radius * sqrt(3)
    diff_y = radius

    print(f"desired x {target[0] - diff_x}, y {diff_y + target[1] - 0.3}")

def MoveVehicleInBackwardSShape(vehicleId, target):
    # x = (1.0, 1.732)
    # target x,y (1.50, -7.12)
    # current pose
    global vehicles
    vehicle = vehicles[vehicleId]

    twist = Twist()
    angular_spd = 0.15
    linear_spd = 0.22

    # radius = linear_spd / angular_spd
    # diff_x = radius * sqrt(3)
    # diff_y = radius

    start_time = rospy.get_time()
    job_not_done = True

    ndt_pose = vehicle.getNDTPose()
    diff_x = abs(ndt_pose.pose.position.x - target[0])
    diff_y = abs(ndt_pose.pose.position.y - target[1])

    mid_x = target[0] - diff_x / 2.0
    mid_y = target[1] + diff_y - 0.06
    # rospy.loginfo(f"Current x {ndt_pose.pose.position.x}, y {ndt_pose.pose.position.y}")
    # alpha = calculateSShapeAngle(diff_x, diff_y)
    # # alpha = abs(atan(diff_x / diff_y))
    # # radius = diff_x * 0.5 / sin(alpha)
    # radius = diff_y
    # linear_spd = abs(angular_spd * radius)

    # rospy.loginfo(f"linear_spd {linear_spd},  angular_spd {angular_spd}, mid_x {mid_x}, radius {radius}")
    # return
    while job_not_done:
        ndt_pose = vehicle.getNDTPose()
        if ndt_pose is None:
            continue

        x = ndt_pose.pose.position.x
        y = ndt_pose.pose.position.y
        # if x < mid_x:
        if y > mid_y or x < mid_x:
            v = -1 * linear_spd
            w = -1 * angular_spd
        else:
            v = 0.0
            w = 0.0
            rospy.loginfo(f"Reached mid_x {mid_x} at position {x}")
            end_time = rospy.get_time()
            job_not_done = False
        twist.linear.x = v
        twist.angular.z = w
        vehicle.send_cmd_vel(twist)
    
    T = end_time - start_time
    job_not_done = True

    while job_not_done:
        ndt_pose = vehicle.getNDTPose()
        if ndt_pose is None:
            continue
        current_time = rospy.get_time()
        x = ndt_pose.pose.position.x

        if current_time - end_time < T:
            v = -1 * linear_spd
            w = angular_spd
        else:
            rospy.loginfo(f"Reached target at position {x}")
            v = 0.0
            w = 0.0
            job_not_done = False
        twist.linear.x = v
        twist.angular.z = w
        vehicle.send_cmd_vel(twist)
    
    # job_not_done = True

    # while job_not_done:
    #     ndt_pose = vehicle.getNDTPose()
    #     if ndt_pose is None:
    #         continue
    #     x = ndt_pose.pose.position.x

    #     if x - target[0] > 0.005:
    #         v = 0.1
    #     else:
    #         rospy.loginfo(f"Reached target at position {x}")
    #         v = 0.0
    #         w = 0.0
    #         job_not_done = False
    #     twist.linear.x = v
    #     twist.angular.z = w
    #     vehicle.send_cmd_vel(twist)


        
# def MoveVehicleForward(vehicleId, target):

#     global vehicles
#     vehicle = vehicles[vehicleId]
    
#     v = 0.0
#     w = 0.0
#     job_not_done = True
#     twist = Twist()

#     while job_not_done:
#         ndt_pose = vehicle.getNDTPose()
#         if ndt_pose ==None:
#             continue
#         x = ndt_pose.pose.position.x
#         # rospy.loginfo(f"{getSystime()} x: {x}")
#         if x < target:
#             v = 0.1
#         elif x > target + 0.05:
#             v = -0.1
#         else:
#             v = 0.0
#             job_not_done = False
#             rospy.loginfo(f"stop time {getSystime()} x: {x}")
#         twist.linear.x = v
#         vehicle.send_cmd_vel(twist)
#         # sleep_ms(1)

# def MoveVehicleBackward(vehicleId, target):

#     global vehicles
#     vehicle = vehicles[vehicleId]
    
#     v = 0.0
#     w = 0.0
#     job_not_done = True
#     twist = Twist()

#     while job_not_done:
#         ndt_pose = vehicle.getNDTPose()
#         if ndt_pose ==None:
#             continue
#         x = ndt_pose.pose.position.x
#         # rospy.loginfo(f"{getSystime()} x: {x}")
#         if x > target:
#             v = -0.1
#         elif x < target - 0.05:
#             v = 0.1
#         else:
#             v = 0.0
#             job_not_done = False
#             rospy.loginfo(f"stop time {getSystime()} x: {x}")
#         twist.linear.x = v
#         vehicle.send_cmd_vel(twist)
#         # sleep_ms(1)


