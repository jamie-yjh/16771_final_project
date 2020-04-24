#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point

from scipy.spatial.transform import Rotation as Rot
import numpy as np
from control import lqr


def create_traj(N, R):

    theta = np.linspace(0, 2*np.pi, num = N)
    traj = np.zeros((N,2))
    for i in range(N):
        traj[i,0] = R * np.cos(theta[i])
        traj[i,1] = R + R*np.sin(theta[i])
    return traj

def wrap2pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

def get_rotation (msg):
    orientation_q = msg.orientation
    r = Rot.from_quat([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    euler_angle = r.as_euler('xyz', degrees=False)
    roll = euler_angle[0]
    pitch = euler_angle[1]
    yaw = euler_angle[2]
    print(pitch)
    return (roll, pitch, yaw)

def closest_node(X, Y, traj):
    point = np.array([X, Y])
    traj = np.asarray(traj)
    dist = point - traj
    dist_2 = np.sum(dist ** 2, axis=1)
    minIndex = np.argmin(dist_2)
    return np.sqrt(dist_2[minIndex]), minIndex
# number:
def A_matrix_num(m, M, mw, I, g, l):
    A = np.zeros((6,6))
    A[0,1] = 1
    A[1,0] = (m+2*mw+M)*m*l*g/((m+2*mw+M)*I+(m+2*mw+M)*l**2)
    A[2,3] = 1
    A[3,0] = -m*m*g*l**2/((M+2*mw)*I+(M+2*mw)*m*l**2)
    A[4,5] = 1

    return A

def B_matrix_num(m, M, mw, Iw, Ip, l, d, r):#u1 is t1+t2 u2 is t1 - t2
    B = np.zeros((6,2))
    t3 = -m*l/((m+2*mw+M)*I+(M+2*mw)*m*l**2)
    t4 = 1/r * (I+m*l**2)/((m+2*mw+M)*I+(M+2*mw)*m*l**2)
    t5 = 2*d/(r*(2*Iw+Ip))

    B[1,0] = t3
    B[3,0] = t4
    B[5,1] = t5

    return B    

def lqr_control():
    #ignore the platform mass and interia
    m = 10               #blue bass mass
    mw = 0.4             #wheel mass
    l = 0.7              #center of mass height(roughly)
    g = 9.8              #gravity coefficient
    r = 0.1              #wheel radius 
    Ip = 0.15            #moment of inertia of wheel about y
    Iw = 0.5 * mw * r**2 #moment of inertia of robotbase about z 
    d = 0.35             #distance between wheels
    Ip = 0.15
    IM = 0
    M = 0
    # reference chapter 4 lqr model 

    A = A_matrix_num(m, M, mw, IM, g, l)
    B = B_matrix_num(m, M, mw, IM, Iw, Ip, l, d, r)

    '''
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
    '''
    Q = np.diag([100, 10, 1, 1, 1, 1])
    R = np.eye(2) * 0.1
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
    base_rolldot = robot_twist.angular.x 
    base_pitchdot = robot_twist.angular.y
    base_yawdot = robot_twist.angular.z

    
    
    
    distance, minIndex = closest_node(base_x, base_y, traj)
    if minIndex < N - lookAhead:
        goalIndex = minIndex + lookAhead
    else:
        goalIndex = minIndex + lookAhead - N
    
    # build error state vector
    v_des = 0.2 # 1m/s
    alpha = base_pitch
    alpha_dot = base_pitchdot
    
    x = np.sqrt(
            (traj[goalIndex, 1] - base_y) ** 2 + (traj[goalIndex, 0] - base_x) ** 2)
    base_v = np.sqrt(base_xdot**2 + base_ydot**2)
    v = base_v - v_des
    theta_des = np.arctan2(traj[goalIndex, 1] - base_y, traj[goalIndex, 0] - base_x)
    theta = wrap2pi(base_yaw - theta_des)
    thetadot_des = v_des*v_des/R
    thetadot = wrap2pi(base_yawdot - thetadot_des)


    error_state = np.array([alpha, alpha_dot, x, v, theta, thetadot]).reshape(6,1)
    u = np.matmul(K, error_state)/20
    
 
    rtau = -(u[0,0] + u[1,0])/2
    ltau = -(u[0,0] - u[1,0])/2
    print([minIndex, goalIndex])
    #print([rtau,ltau])
    #print(error_state)
    print([theta_des,base_yaw])
    rate = rospy.Rate(50) # 1hz
    msg_lw = Float64()
    msg_rw = Float64()
    msg_lw.data = ltau
    msg_rw.data = rtau

    # compute knee angle
    h = 0.8
    g = 9.8
    gamma = np.arccos(1 - d*base_v**2/(h*g*R))
    if theta >= 0: # bend left knee and ankle, right leg is straight
        left_ankle_angle = gamma
        left_knee_angle = 2*gamma
    else:         # bend right knee and ankle, left leg is straight
        right_ankle_angle = gamma
        right_knee_angle = 2*gamma


    lwpub.publish(msg_lw)
    rwpub.publish(msg_rw)
    #rate.sleep()
   
    


if __name__ == '__main__':
    try:

        m = 10               #blue bass mass
        mw = 0.4             #wheel mass
        h = 0.7              #center of mass height(roughly)
        g = 9.8              #gravity coefficient
        r = 0.1              #wheel radius 
        Iw = 0.5 * mw * r**2 #moment of inertia of wheel about y
                             #moment of inertia of robotbase about z 

        M = 0
        I = 0
        l = h
        Ip = 0.15
        d = 0.35


        def A_matrix_num(m, M, mw, I, g, l):
            A = np.zeros((6,6))
            A[0,1] = 1
            A[1,0] = (m+2*mw+M)*m*l*g/((m+2*mw+M)*I+(m+2*mw+M)*l**2)
            A[2,3] = 1
            A[3,0] = -m*m*g*l**2/((M+2*mw)*I+(M+2*mw)*m*l**2)
            A[4,5] = 1

            return A

        def B_matrix_num(m, M, mw, Iw, Ip, l, d, r):#u1 is t1+t2 u2 is t1 - t2
            B = np.zeros((6,2))
            t3 = -m*l/((m+2*mw+M)*I+(M+2*mw)*m*l**2)
            t4 = 1/r * (I+m*l**2)/((m+2*mw+M)*I+(M+2*mw)*m*l**2)
            t5 = 2*d/(r*(2*Iw+Ip))

            B[1,0] = t3
            B[3,0] = t4
            B[5,1] = t5

            return B    

        A = A_matrix_num(m, M, mw, I, g, l)
        B = B_matrix_num(m, M, mw, Iw, Ip, l, d, r)
        Q = np.eye(6)
        Q[0][0] = 100
        Q[1][1] = 10
        Q[2][2] = 1
        Q[3][3] = 1
        R = np.eye(2)
        R[0][0] = 1
        R[1][1] = 1
        global K
        K,S,E = lqr(A, B, Q, R)
        global PID
        PID = False
        global traj
        N = 200 # points
        R = 1.5 # radius
        global lookAhead
        lookAhead = 10
        traj = create_traj(N, R)
        lwpub = rospy.Publisher('/lw_left_wheel_controller/command', Float64 , queue_size=2)
        rwpub = rospy.Publisher('/lw_right_wheel_controller/command', Float64 , queue_size=2)
        rospy.init_node('pub_wheel_tau', anonymous=False)
        rospy.Subscriber('/gazebo/model_states', ModelStates, pub_torque, queue_size =2)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass