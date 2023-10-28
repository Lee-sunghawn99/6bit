# 카메라 중앙에 과일이 위치하면 로봇을 정지함
# Stop the robot when the fruit is in the center of the camera
# 바운딩 박스의 중심 좌표를 받아옴
# Receive the center coordinates of the bounding box
# 중심좌표가 이미지의 중심에 위치하면 로봇을 정지함
# Stop the robot when the center coordinates are located at the center of the image

# 로봇이 정지하면 제자리에서 90도 회전함

# 바운딩 박스의 면적을 계산하여 과일의 크기를 추정함
# 과일의 크기에 따라 로봇이 전진하는 거리를 조절함
# 로봇이 과일을 수확할 수 있는 위치까지 전진함

# 카메라를 이용하여 과일의 위치를 파악함
# 바운딩 박스의 중심좌표를 받아옴
# 중심좌표에 맞게 로봇 팔을 제어함
# 로봇 팔이 과일을 수확함

# 수확한 과일을 바구니에 담음

# 로봇을 반대 방향으로 90도 회전함

# 다음 목표지점으로 진행함


import rospy
import math
import time
import numpy as np
from detection_msgs.msg import BoundingBoxes
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Twist


class HarvestTest:
    def __init__(self):
        self.bb = BoundingBoxes()
        self.twist = Twist()
        self.goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)
        self.goal = MoveBaseActionGoal()
        self.goal.goal.target_pose.header.frame_id = 'map'
        self.goal.goal.target_pose.pose.orientation.w = 1.0
        self.goal.goal.target_pose.pose.position.x = 0.0
        self.goal.goal.target_pose.pose.position.y = 0.0
        self.goal.goal.target_pose.pose.position.z = 0.0
        self.goal.goal.target_pose.header.stamp = rospy.Time.now()
        self.rate = rospy.Rate(5.35)  # define rate here

    def odom_callback(msg):
        global pose_position
        global pose_orientation
        pose_position = msg.pose.pose.position
        pose_orientation = msg.pose.pose.orientation


    def left_callback(data):
        global left_fruit
        global cnt_n_left
        global cnt_d_left
        global img_left
        img_left = data
        cnt_n_left = 0
        cnt_d_left = 0


    def right_callback(data):
        global right_fruit
        global cnt_n_right
        global cnt_d_right
        global img_right
        img_right = data
        cnt_n_right = 0
        cnt_d_right = 0

    def harvest(self):
        x_lenght = self.bb.xmax - self.bb.xmin
        y_lenght = self.bb.ymax - self.bb.ymin
        bb_area = x_lenght * y_lenght
        fruit_size = bb_area / 100
        threshold = 100

        x_center = (self.bb.xmax + self.bb.xmin) / 2
        y_center = (self.bb.ymax + self.bb.ymin) / 2

        if x_center >= 320:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0

            # Set the desired angle to turn (in radians)
            desired_angle = math.pi / 2  # 90 degrees

            # Get the robot's current orientation
            current_orientation = pose_orientation.z

            # Calculate the target orientation
            target_orientation = current_orientation + desired_angle

            # Ensure target orientation is within [-pi, pi] range
            target_orientation = math.atan2(math.sin(target_orientation), math.cos(target_orientation))

            # Set the angular velocity to turn the robot
            angular_velocity = 0.5  # adjust as needed

            # Stop the robot after turning
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            target_orientation = math.atan2(math.sin(target_orientation), math.cos(target_orientation))

            # Set the angular velocity to turn the robot
            angular_velocity = 0.5  # adjust as needed

            # Loop until the robot has turned the desired angle
            while abs(pose_orientation.z - target_orientation) > 0.05:  # adjust tolerance as needed
                self.twist.linear.x = 0.0
                self.twist.angular.z = angular_velocity
                self.rate.sleep()

            # Stop the robot after turning
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0

        while fruit_size <=threshold:
            self.twist.linear.x = 0.3
            self.twist.angular.z = 0.0
            if fruit_size >= threshold:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                break
        
        
        if y_center <120:
            control_robot_arm("up")
            time.sleep(1)
            control_robot_arm("grab")
            time.sleep(2)
            control_robot_arm("center_grab")
            time.sleep(1)
        elif 120 < y_center < 160:
            control_robot_arm("mid_up")
            time.sleep(1)
            control_robot_arm("grab")
            time.sleep(2)
            control_robot_arm("center_grab")
            time.sleep(1)
        elif 170 < y_center < 230:
            control_robot_arm("mid_down")
            time.sleep(1)
            control_robot_arm("grab")
            time.sleep(2)
            control_robot_arm("center_grab")
            time.sleep(1)
        elif y_center > 230:
            control_robot_arm("down")
            time.sleep(1)
            control_robot_arm("grab")
            time.sleep(2)
            control_robot_arm("center_grab")
            time.sleep(1)
        else:
            control_robot_arm("center")      
                    
          
                    
from Transbot_Lib import Transbot

def arm_speed(speed):
    time.sleep(0.2)

try:
    bot=Transbot()
except AttributeError as e:
    print(f"Erorr init Transbot:{e}")

# Initialize the robot arm
def control_robot_arm(position):
    servo_id1 = 7
    servo_id2 = 8
    servo_id3 = 9

    if position == "up":
        bot.set_uart_servo_angle(servo_id2, 170)
        arm_speed()
        bot.set_uart_servo_angle(servo_id1, 160)
        arm_speed()
        bot.set_uart_servo_angle(servo_id3, 30)
    
    elif position == "mid_up":
        bot.set_uart_servo_angle(servo_id2, 170)
        arm_speed()
        bot.set_uart_servo_angle(servo_id1, 140)
        arm_speed()
        bot.set_uart_servo_angle(servo_id3, 30)

    elif position == "mid_down":
        bot.set_uart_servo_angle(servo_id2, 170)
        arm_speed()
        bot.set_uart_servo_angle(servo_id1, 125)
        arm_speed()
        bot.set_uart_servo_angle(servo_id3, 30)
    
    elif position == "down":
        bot.set_uart_servo_angle(servo_id2, 190)
        arm_speed()
        bot.set_uart_servo_angle(servo_id1, 110)
        arm_speed()
        bot.set_uart_servo_angle(servo_id3, 30)

    elif position == "center":
        bot.set_uart_servo_angle(servo_id2, 30)
        arm_speed()
        bot.set_uart_servo_angle(servo_id1, 135)
        arm_speed()
        bot.set_uart_servo_angle(servo_id3, 31)
    
    elif position == "center_grab":
        bot.set_uart_servo_angle(servo_id2, 33)
        arm_speed()
        bot.set_uart_servo_angle(servo_id1, 225)
        arm_speed()
        bot.set_uart_servo_angle(servo_id3, 180)

    elif position == "grab":
        bot.set_uart_servo_angle(servo_id3, 180)
            

        



rospy.init_node("current_position", anonymous=True)

sub_right = rospy.Subscriber("/yolov5/detections_right", BoundingBoxes, HarvestTest.right_callback)
sub_left = rospy.Subscriber("/yolov5/detections_left", BoundingBoxes, HarvestTest.left_callback)
img_right = None
img_left = None
pose_position = None
pose_orientation = None
cnt_n_left = None
cnt_d_left = None
cnt_n_right = None
cnt_d_right = None

rate = rospy.Rate(5.35)
sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, odom_callback)


# calcualte the area of the bounding box
# estimate the size of the fruit
# adjust the distance the robot moves according to the size of the fruit
# move the robot forward to the position where it can harvest the fruit
    

