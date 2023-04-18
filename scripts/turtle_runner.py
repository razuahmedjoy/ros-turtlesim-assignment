#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
from math import pow, atan2, sqrt,degrees
from turtlesim_assignment.msg import custom_msg

global x 
global y 
global theta


def rotate(userX,userY):
    
    global x,y,theta

    velocity_msg = Twist()

    rospy.loginfo(("hello from rotate"))

    velocity_pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)

    while not rospy.is_shutdown():

        angle = atan2(userY - y, userX- x)

        velocity_msg.angular.z = 4.0 * (angle - theta)

        dif = int(degrees(angle) - degrees(theta))
        # # print the angle to be rotated
        # rospy.loginfo("Angle: %f", angle)
        # # print self theta
        # rospy.loginfo("Difference  : %d", dif)

        if dif == 0:
            break

        if velocity_msg.angular.z > 2.0:
            velocity_msg.angular.z = 2.0
        elif velocity_msg.angular.z < -2.0:
            velocity_msg.angular.z = -2.0
        
        # Publish the velocity message to move the turtle
        velocity_pub.publish(velocity_msg)    

    velocity_msg.angular.z = 0
    velocity_pub.publish(velocity_msg)


def move_forward(speed,userX,userY):
    vel_msg = Twist()
    global x,y

    velocity_publihser = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)

    custom_msg_publisher = rospy.Publisher("turtle_publisher", custom_msg, queue_size=10)
    msg = custom_msg()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # rospy.loginfo("Turtlesim moves forward")

        k_linear = 0.5
        distance = abs(sqrt(((x-userX) ** 2) + ((y-userY) ** 2)))


        # rospy.loginfo(msg="distance: " + str(distance))
        linear_speed = distance * k_linear

        vel_msg.linear.x = abs(linear_speed)

        msg.robot_name = "Robot name here"
        msg.velocity = linear_speed
        msg.distance = distance
        custom_msg_publisher.publish(msg)

        velocity_publihser.publish(vel_msg)
        custom_msg_subscriber = rospy.Subscriber("turtle_publisher", custom_msg, callback=custom_msg_callback)

        rate.sleep()

       
        if distance < 0.2:
            rospy.loginfo("reached")
            break

    vel_msg.linear.x = 0
    velocity_publihser.publish(vel_msg)
    msg.velocity=0
    custom_msg_publisher.publish(msg)
    print("currentX: " + str(x))
    print("currentY: " + str(y))



def poseCallback(pose_message:Pose):
    global x,y,theta
    x = pose_message.x
    y = pose_message.y
    theta = pose_message.theta

    # rospy.loginfo(msg="x: " + str(x) + " y: " + str(y) + " theta: " + str(theta))



def custom_msg_callback(msg):
    rospy.loginfo(msg.robot_name)
    rospy.loginfo(msg.velocity)
    rospy.loginfo(msg.distance)

if __name__== "__main__":
    try:
        # initialize a node
     

        rospy.init_node("turtle_runner",anonymous=True)
        rospy.loginfo("Node has been started")
        position_topic = "/turtle1/pose"

        position_subscriber = rospy.Subscriber(position_topic, Pose,callback=poseCallback)



        while not rospy.is_shutdown():
            userX = float(input("Enter the x-coordinate of the goal point: "))
            userY = float(input("Enter the y-coordinate of the goal point: "))

            rotate(userX, userY)
            move_forward(5,userX,userY)
        

            


        
       
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)