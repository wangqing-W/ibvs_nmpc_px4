#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from acados_settings import acados_settings
import time
import os
import numpy as np
import matplotlib.pyplot as plt
from plotFnc import *
from utils import *
from trajectory import *
import rospy
import message_filters
from std_msgs.msg import Header
from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from ibvs_pkg.msg import xyzyawVel, point_xyz
from quadrotor_msgs.msg import ControlCommand
from scipy.spatial.transform import Rotation as R
import threading

class QuadrotorController(object):
    def __init__(self, Th, N):
        # mpc and simulation parameters
        self.Ts = Th / N   # sampling time[s]

        # constants
        self.g = 9.81     # m/s^2
        self.N = N

        # load model and acados_solver
        self.model, self.acados_solver, self.acados_integrator = acados_settings(self.Ts, Th, N)

        # dimensions
        self.nx = self.model.x.size()[0]
        self.nu = self.model.u.size()[0]
        self.ny = self.nx + self.nu
        # self.ny_e = self.nx - 12

        # initialize data structs
        self.tot_comp_sum = 0
        self.tcomp_max = 0
        self.iter = 0

        # set initial condition for acados integrator
        self.xcurrent = self.model.x0.reshape((self.nx,))

        # initial info
        self.quad = np.array([0.0, 0.0, 0.0, 1.0]) # x,y,z,w
        self.R = R.from_quat(self.quad)
        self.R_m = self.R.as_matrix()

        # self.odom = Odometry()
        self.vel_B = np.zeros([3,1])
        self.vel_W = np.zeros([3,1])
        self.vel_W_history = []  # 存储历史速度数据
        self.ref_vel_history = []  # 存储历史参考速度数据

        self.time = rospy.Time.now()
        self.control_command = ControlCommand()
        self.control_command.control_mode = self.control_command.BODY_RATES

        self.target_reached = False

        # get sensor topics
        imu_topic = rospy.get_param('~imu_topic', "/hummingbird/imu")
        odom_topic = rospy.get_param('~odom_topic', "/hummingbird/ground_truth/odometry")

        # get ibvs topics
        ref_vel_topic = rospy.get_param('~ref_vel_topic', "/hummingbird/reference_vel")
        marker_topic = rospy.get_param('~marker_topic', "/hummingbird/markerpoint")

        # get control topics
        control_command_topic = rospy.get_param('~control_command_topic', "/hummingbird/autopilot/control_command_input")
        # arm_topic = rospy.get_param('~arm_topic', "bridge/arm")

        self.imu_sub = rospy.Subscriber(imu_topic, Imu, self.imuCallback)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odomCallback)
        self.target_reached_sub = rospy.Subscriber('target_reached', Bool, self.target_reached_callback)
        self.control_command_pub = rospy.Publisher(control_command_topic, ControlCommand, queue_size=1)

        self.ref_vel_sub = message_filters.Subscriber(ref_vel_topic, xyzyawVel)
        self.marker_sub = message_filters.Subscriber(marker_topic, point_xyz)
        sync_listener = message_filters.ApproximateTimeSynchronizer([self.ref_vel_sub,self.marker_sub], 1, 0.05)
        sync_listener.registerCallback(self.refVelMarkerPointCallback)

        # # create a new thread for plotting
        # self.plot_thread = threading.Thread(target=self.plotVelocities)
        # self.plot_thread.daemon = True
        # self.plot_thread.start()

    def imuCallback(self, imu_msg):
        self.quad[3] = imu_msg.orientation.w
        self.quad[0] = imu_msg.orientation.x
        self.quad[1] = imu_msg.orientation.y
        self.quad[2] = imu_msg.orientation.z
        self.R = R.from_quat(self.quad)
        self.R_m = self.R.as_matrix()
        # print("[INFO] estimated quant w,x,y,z is:", self.quad)
    
    def odomCallback(self, odom_msg):
        self.vel_B[0] = odom_msg.twist.twist.linear.x
        self.vel_B[1] = odom_msg.twist.twist.linear.y
        self.vel_B[2] = odom_msg.twist.twist.linear.z
        self.vel_W = self.R_m.dot(self.vel_B)
        # print("[INFO] estimated vel_W is:", self.vel_W)
    
    def refVelMarkerPointCallback(self, ref_vel_msg: xyzyawVel, marker_msg: point_xyz):
        print("[INFO] estimated quant w,x,y,z is:", self.quad)
        print("[INFO] estimated vel_W is:", self.vel_W)
        print("[INFO] reference_vel is:", ref_vel_msg)
        x_current = np.array([0.0, 0.0, 1, self.quad[3], self.quad[0], self.quad[1], self.quad[2], self.vel_W[0][0], self.vel_W[1][0], self.vel_W[2][0], 
                              marker_msg.p1_x, marker_msg.p1_y, marker_msg.p1_z, marker_msg.p2_x, marker_msg.p2_y, marker_msg.p2_z,
                              marker_msg.p3_x, marker_msg.p3_y, marker_msg.p3_z, marker_msg.p4_x, marker_msg.p4_y, marker_msg.p4_z])
        # solve ocp for a fixed reference
        self.acados_solver.set(0, "lbx", x_current)
        self.acados_solver.set(0, "ubx", x_current)
        # 设置acados_solver的reference
        # 将所有阶段的数据存储在一个numpy数组中
        yref_all = np.zeros((self.N, 26))
        yref_all[:, 7] = ref_vel_msg.Vx
        yref_all[:, 8] = ref_vel_msg.Vy
        yref_all[:, 9] = ref_vel_msg.Vz
        # yref_all[:, 10] = 0.0
        # yref_all[:, 11] = 0.0
        yref_all[:, 12] = 1.0
        # yref_all[:, 13] = 0.0
        # yref_all[:, 14] = 0.0
        yref_all[:, 15] = 1.0
        # yref_all[:, 16] = 0.0
        # yref_all[:, 17] = 0.0
        yref_all[:, 18] = 1.0
        # yref_all[:, 19] = 0.0
        # yref_all[:, 20] = 0.0
        yref_all[:, 21] = 1.0
        yref_all[:, 22] = 9.81
        yref_all[:, 23] = 0.0
        yref_all[:, 24] = 0.0
        yref_all[:, 25] = ref_vel_msg.Vyaw
        

        # 使用numpy的切片操作来设置每个阶段的数据
        for j in range(self.N):
            self.acados_solver.set(j, "yref", yref_all[j])
        x_e  = 0.0
        y_e  = 0.0
        z_e  = 0.0
        qw_e = 1.0
        qx_e = 0.0
        qy_e = 0.0
        qz_e = 0.0
        vx_e = ref_vel_msg.Vx
        vy_e = ref_vel_msg.Vy
        vz_e = ref_vel_msg.Vz

        yref_N = np.array([x_e, y_e, z_e, qw_e, qx_e,
                           qy_e, qz_e, vx_e, vy_e, vz_e,
                           0.0, 0.0, 1.0, 0.0, 0.0, 1.0,
                           0.0, 0.0, 1.0, 0.0, 0.0, 1.0])
        self.acados_solver.set(self.N, "yref", yref_N)

        comp_time = time.time()
        status = self.acados_solver.solve()
        if status != 0:
            print("acados returned status {} in closed loop iteration {}.".format(status, iter))

        # manage timings
        elapsed = time.time() - comp_time
        self.tot_comp_sum += elapsed
        if elapsed > self.tcomp_max:
            self.tcomp_max = elapsed

        # get solution from acados_solver
        u0 = self.acados_solver.get(0, "u")
        self.time = rospy.Time.now()
        self.publishCommand(u0)
        # simulate the system
        self.acados_integrator.set("x", x_current)
        self.acados_integrator.set("u", u0)
        status = self.acados_integrator.solve()
        if status != 0:
            raise Exception(
                'acados integrator returned status {}. Exiting.'.format(status))

        # get state
        print("[PREDICTION] predict state:", self.acados_integrator.get("x")[10:])
        self.iter += 1

        # plot the ref and cur velocity
        self.vel_W_history.append(self.vel_W)  # 存储历史速度数据
        self.ref_vel_history.append([ref_vel_msg.Vx, ref_vel_msg.Vy, ref_vel_msg.Vz])  # 存储历史参考速度数据
        
        # print the computation times
        print("Total computation time: {}".format(self.tot_comp_sum))
        print("Average computation time: {}".format(self.tot_comp_sum / self.iter))
        print("Maximum computation time: {}".format(self.tcomp_max))

    def publishCommand(self, u0):
        self.control_command.header = Header(stamp=self.time)
        self.control_command.armed = True
        self.control_command.expected_execution_time = self.time
        self.control_command.bodyrates = bodyratesToGeometry(u0)
        self.control_command.collective_thrust = u0[0]
        print("[INFO] OUTPUT is:\nthrust: ", self.control_command.collective_thrust, "\nbodyrates: ", self.control_command.bodyrates)
        # TO BE TEST
        self.control_command_pub.publish(self.control_command)
    
    # 在QuadrotorController类外面
    def plotVelocities(self):
        fig, axs = plt.subplots(3, 1, figsize=(8, 6))
        axs[0].plot([v[0] for v in self.vel_W_history], label='Actual Vx')
        axs[0].plot([v[0] for v in self.ref_vel_history], label='Reference Vx')
        axs[0].set_ylabel('Vx [m/s]')
        axs[0].legend()
        axs[1].plot([v[1] for v in self.vel_W_history], label='Actual Vy')
        axs[1].plot([v[1] for v in self.ref_vel_history], label='Reference Vy')
        axs[1].set_ylabel('Vy [m/s]')
        axs[1].legend()
        axs[2].plot([v[2] for v in self.vel_W_history], label='Actual Vz')
        axs[2].plot([v[2] for v in self.ref_vel_history], label='Reference Vz')
        axs[2].set_ylabel('Vz [m/s]')
        axs[2].legend()
        plt.show()

    def target_reached_callback(self, msg):
        self.target_reached = msg.data

    def get_reached(self):
        return self.target_reached

if __name__ == "__main__":
    rospy.init_node('acados_mpc_control', anonymous=True)
    quadrotorController = QuadrotorController(3, 10)
    rate = rospy.Rate(50)  # 设置循环频率为50Hz
    while not rospy.is_shutdown():
        # 在这里执行其他操作，例如检查程序是否应该退出
        if quadrotorController.get_reached():
            break
        # 调用spinOnce()函数处理所有的回调函数
        try:
            rospy.rostime.wallsleep(0.01)
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass
        # 控制循环的频率
        rate.sleep()
    quadrotorController.plotVelocities()