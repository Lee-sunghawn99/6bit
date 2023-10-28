import rospy
import math
from detection_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped

def left_callback(data):
    global left_fruit
    global img_left
    img_left = data



def right_callback(data):
    global right_fruit
    global img_right
    img_right = data

def odom_callback(msg):
    global pose_position
    global pose_orientation
    pose_position = msg.pose.pose.position
    pose_orientation = msg.pose.pose.orientation

twist = Twist()

rospy.init_node("current_position", anonymous=True)

sub_right = rospy.Subscriber("/yolov5/detections_right", BoundingBoxes, right_callback)
sub_left = rospy.Subscriber("/yolov5/detections_left", BoundingBoxes, left_callback)
sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, odom_callback)

img_right = None
img_left = None

rate = rospy.Rate(5.35)

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

while not rospy.is_shutdown():

    if img_left is not None:
        for each in img_left.bounding_boxes:
            if each.Class == 'apple-R':
                x_lenght = each.xmax - each.xmin
                y_lenght = each.ymax - each.ymin
                bb_area = x_lenght * y_lenght
                fruit_size = bb_area / 100
                threshold = 100

                x_center = (each.xmax + each.xmin) / 2
                y_center = (each.ymax + each.ymin) / 2
            
                if x_center >= 320:
                    twist.linear.x = 1.0
                    twist.angular.z = 0.0
                    pub.publish(twist)

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
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    pub.publish(twist)
                    target_orientation = math.atan2(math.sin(target_orientation), math.cos(target_orientation))

                    # Set the angular velocity to turn the robot
                    angular_velocity = 0.5  # adjust as needed

                    # Loop until the robot has turned the desired angle
                    while abs(pose_orientation.z - target_orientation) > 0.05:  # adjust tolerance as needed
                        twist.linear.x = 0.0
                        twist.angular.z = angular_velocity
                        pub.publish(twist)
                        rate.sleep()

                    # Stop the robot after turning
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    pub.publish(twist)

                    while fruit_size <=threshold:
                        twist.linear.x = 0.3
                        twist.angular.z = 0.0
                        pub.publish(twist)
                        if fruit_size >= threshold:
                            twist.linear.x = 0.0
                            twist.angular.z = 0.0
                            pub.publish(twist)
                            break

                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    pub.publish(twist)        
    rate.sleep()