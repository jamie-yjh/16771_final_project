#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point

from scipy.spatial.transform import Rotation as R
import numpy as np
from control import lqr


def get_rotation (msg):
    orientation_q = msg.orientation
    r = R.from_quat([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    euler_angle = r.as_euler('xyz', degrees=False)
    roll = euler_angle[0]
    pitch = euler_angle[1]
    yaw = euler_angle[2]
    print(pitch)
    return (roll, pitch, yaw)
    

def lqr_control():
    #ignore the platform mass and interia
    m = 10               #blue bass mass
    mw = 0.4             #wheel mass
    h = 0.8              #center of mass height(roughly)
    g = 9.8              #gravity coefficient
    r = 0.1              #wheel radius 
    Iw = 0.15            #moment of inertia of wheel about y
    Iz = 0.5 * mw * r**2 #moment of inertia of robotbase about z 
    # reference chapter 4 lqr model 
    t1 = 
    t2 = 
    t3 = 
    t4 =
    t5 =
    A = np.array([[0,1,0,0,0,0],
        [t1,0,0,0,0,0],
        [0,0,0,1,0,0],
        [t2,0,0,0,0,0],
        [0,0,0,0,0,1],
        [0,0,0,0,0,0]])
    B = np.array([[0,0],
        [t3,0],
        [0,0],
        [t4,0],
        [0,0],
        [0,t5]])
    Q = np.eye(6)
    R = np.eye(2)
    K,S,E = lqr(A,B,Q,R)
    print(K)

    return K

def pub_torque(model_states):

    robot_pose = model_states.pose[1]
    base_x = robot_pose.position.x 
    base_y = robot_pose.position.y
    base_z = robot_pose.position.z
    base_roll, base_pitch, base_yaw = get_rotation(robot_pose)

    robot_twist = model_states.twist[1]
    base_xdot = robot_twist.linear.x 
    base_ydot = robot_twist.linear.y
    base_zdot = robot_twist.linear.z
    base_rolldot = robot_twist.linear.x 
    base_pitchdot = robot_twist.linear.y
    base_yawdot = robot_twist.linear.z

    rate = rospy.Rate(1) # 1hz
    pitch = Float64(base_pitch)
    msg_lw = Float64()
    msg_rw = Float64()
    
    kp = 12
    kd = 0.1*kp
    ltau = kp * base_pitch + kd * base_pitchdot
    rtau = ltau
    msg_lw.data = ltau
    msg_rw.data = rtau
    rospy.loginfo(pitch)
    lwpub.publish(msg_lw)
    rwpub.publish(msg_rw)
    #rate.sleep()

if __name__ == '__main__':
    try:
        lwpub = rospy.Publisher('/lw_left_wheel_controller/command', Float64 , queue_size=10)
        rwpub = rospy.Publisher('/lw_right_wheel_controller/command', Float64 , queue_size=10)
        rospy.init_node('pub_wheel_tau', anonymous=False)
        rospy.Subscriber('/gazebo/model_states', ModelStates, pub_torque, queue_size =10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass