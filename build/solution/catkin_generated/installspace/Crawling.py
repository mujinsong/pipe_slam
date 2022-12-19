#!/usr/bin python3
import numpy as np
from roboticstoolbox import *
import rospy
from std_msgs.msg import String
from nav_msgs import Odometry
import dynamixel_control_X
pi = np.pi
def talker():
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
if __name__ == '__main__':
    # talker()
    
    
    lenof_link = 10
    a_arm = 0.081
    R = 6
    t = np.arange(0, 1, 0.01)
    t1 = np.arange(0, 1.01, 0.01)
    theta_end = [25.5031, -10.5507, -18.6721, 35.7715, -34.3595, 18.9978, -8.0826, -17.1588, 33.2843, -13.4935]
    print("run")
    DHs = [RevoluteDH(a=a_arm, alpha=pi / 2, qlim=[-70 * pi / 180, 70 * pi / 180])]
    for i in range(1, lenof_link):
        DHs.append(RevoluteDH(a=a_arm, alpha=(pi / 2) * (-1) ** i, qlim=[-75 * pi / 180, 75 * pi / 180]))
    snake = DHRobot(DHs, name="arm")
    theta_end = snake.toradians(theta_end)

    T0 = snake.fkine(snake.q)
    T_goal = snake.fkine_all(theta_end)
    T_r = T_goal[R].copy()
    sol = snake.ikine_min(T_r, qlim=True)
    solq = np.array([sol.q])

    for i in range(1, ((lenof_link - R) // 2) + 1):
        solj = jtraj(solq[-1][-(i * 2):], theta_end[-(lenof_link - R): R + i * 2], t1)  # 末端角度值校准
        snake2 = DHRobot(DHs[:-(i * 2)])
        snake2.q = solq[-1][:-(i * 2)]
        T0 = snake2.fkine(snake2.q)
        Ts = ctraj(T0, T_r, t)
        solq1 = [snake2.q]
        for T in Ts:
            sol = snake2.ikine_min(T, solq[-1][: -(i * 2)], qlim=True, method='L-BFGS-B')
            # sol = snake2.ikine_min(T, solq1[-1], qlim=True, ilimit=2000, method='L-BFGS-B')
            solq1.append(sol.q)
        solq1 = np.array(solq1)
        solq = np.r_[solq, np.c_[solq1, solj.q]]

        del snake2
    # snake.plot(solq, limits=[0, 0.7, -0.5, 0.5, -0.5, 0.5]).hold()
    d = dynamixel_control_X.dxlControl_X('/dev/ttyUSB0')
    d.open_init_port(3000000)
    d.enable_torque([0,1,2,3,4,5,6,7,8,9])
    d.move2goal(solq)
    d.disable_torque()


