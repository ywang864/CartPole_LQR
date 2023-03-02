# CartPole_LQR
Design a LQR controller for Cart Pole system in ROS/Gazebo

based on https://github.com/linZHank/invpend_experiment

To execute, go to catkin_ws, run source and catkin build

in one terminal, run roslaunch invpend_control load_invpend.launch

in a new one terminal, run rosrun rosrun invpend_control lqr_CartPole.py

# Lagrangian Dynamics

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

# LQR parameters

Q = np.diag([1, 10, 1, 1])
R = np.diag([0.1])

K, S, E = control.lqr(A, B, Q, R)

# Plot

 ![lqr](https://user-images.githubusercontent.com/111333965/222573968-145bbe65-d26b-4875-a30b-484e3baae8ce.png)

# PID vedio

https://user-images.githubusercontent.com/111333965/222574096-34e7e6f3-9e78-439f-abb2-e4c8d8a2aed3.mp4

# LQR vedio

https://user-images.githubusercontent.com/111333965/222574128-0cff5a16-be6a-4f7c-8d5a-3d14719971a0.mp4


