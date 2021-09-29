#!/usr/bin/env python

from numpy.core.einsumfunc import einsum
from numpy.lib.function_base import angle
import rospy
import matplotlib.pyplot as plt
from Ellipse import Ellipse
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from simple_pid import PID
from math import sqrt
import time

x = 0.0
y = 0.0
theta = 0.0
es = Ellipse([[0,0],[0,3],[0,1]], 0, 0, 360)

pid_angle = PID(0.3, 0.05, 0, setpoint=0)
pid_angle.output_limits = (-0.2, 0.2)
pid_angle.set_auto_mode(enabled=True)


pid_distance = PID(0.1, 0.01, 0, setpoint=0)
pid_distance.output_limits = (-0.2, 0.2)
pid_distance.set_auto_mode(enabled=True)

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("pi_controller")

sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(4)

goalx, goaly = es.ellipse(0.1)
# plt.plot(goalx, goaly)
# plt.xlabel('x')
# plt.ylabel('y')
# plt.title('Path')
# plt.grid(True)
# plt.show()
x_error=[]
y_error=[]
t=[]
j=0


for i in range(0,62):
    inc_x = goalx[i] -x
    inc_y = goaly[i] -y
    angle_to_goal = atan2(inc_y, inc_x)
    distance_to_goal = sqrt(inc_y*inc_y+inc_x*inc_x)

    while not rospy.is_shutdown() and distance_to_goal>0.08:
        inc_x = goalx[i] -x
        inc_y = goaly[i] -y
        angle_to_goal = atan2(inc_y, inc_x)
        distance_to_goal = sqrt(inc_y*inc_y+inc_x*inc_x)
        if abs(angle_to_goal-theta) > 0.1:
            speed.linear.x = 0
            speed.angular.z = pid_angle(theta-angle_to_goal)

        else:
            if distance_to_goal > 0.08:
                speed.linear.x = pid_distance(0.1-distance_to_goal)
                speed.angular.z = 0
            else:
                speed.linear.x = 0
                speed.angular.z = 0
        pub.publish(speed)
        print([angle_to_goal,angle_to_goal-theta,distance_to_goal,i])
        x_error.append(inc_x)
        y_error.append(inc_y)
        t.append(time.clock_gettime(3)) 

plt.plot(t,x_error)
plt.xlabel('time')
plt.ylabel('x error')
plt.title('X Error')
plt.grid(True)
plt.show()
plt.plot(t,y_error)
plt.xlabel('time')
plt.ylabel('y error')
plt.title('Y Error')
plt.grid(True)
plt.show()

    

r.sleep()
