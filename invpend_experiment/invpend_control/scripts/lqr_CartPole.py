#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import numpy as np
import control
from gazebo_msgs.msg import LinkState
from geometry_msgs.msg import Point
import math

mPole = 2
mCart = 20
g = 9.81
lPole = 0.5

A = np.matrix([[0, 1, 0, 0],
               [0, 0, (-12 * mPole * g) / ((12 * mCart) + mPole), 0],
               [0, 0, 0, 1],
               [0, 0, (12 * g * (mCart + mPole)) / (lPole * ((13 * mCart) + mPole)), 0]
               ])

B = np.matrix([0, 13 / ((13 * mCart) + mPole), 0, -12 / (lPole * ((13 * mCart) + mPole))]).T

Q = np.diag([1, 10, 1, 1])
R = np.diag([0.1])

K, S, E = control.lqr(A, B, Q, R)


class CartPole:
    def __init__(self):
        self.pos_cart = 0
        self.vel_cart = 0
        self.pos_pole = 0
        self.vel_pole = 0
        self.current_state = np.array([0., 0., 0., 0.])
        self.desired_state = np.array([1., 0., 0., 0.])
        self.currentStateSub = rospy.Subscriber('/invpend/joint_states', JointState, self.state_Callback)
        self.pub_velocity = rospy.Publisher('/invpend/joint1_velocity_controller/command', Float64, queue_size=10)

    def state_Callback(self, data):
        self.current_state[0] = data.position[1]
        self.current_state[1] = data.velocity[1]
        self.current_state[2] = data.position[0]
        self.current_state[3] = data.velocity[0]

    def LQR(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            U = -1 * np.matmul(K, (self.current_state - self.desired_state))
            cmd_vel = float(U)
            self.pub_velocity.publish(cmd_vel)
            if np.mean(np.square(self.desired_state - self.current_state)) < 0.001:
                print("Stabilized")
                continue
            else:
                print("current state", self.current_state)
                print("desired state", self.desired_state)
                print("control command", cmd_vel)
            rate.sleep()



if __name__ == '__main__':
    rospy.init_node('CartController', anonymous=True)
    cartpole = CartPole()
    cartpole.LQR()
    rospy.spin()
