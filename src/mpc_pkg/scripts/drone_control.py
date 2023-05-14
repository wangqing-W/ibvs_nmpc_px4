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
from custom_msgs.msg import ControlCommand
from scipy.spatial.transform import Rotation as R
import threading

class QuadrotorController(object):
    def __init__(self, Th, N):
        # mpc and simulation parameters
        self.Ts = Th / N   # sampling time[s]

        # stop msg
        self.stop = False

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
        self.alpha = 0.6
        self.beta = 0.75

        # set initial condition for acados integrator
        self.xcurrent = self.model.x0.reshape((self.nx,))

        # initial info
        self.quad = np.array([0.0, 0.0, 0.0, 1.0]) # x,y,z,w
        self.R = R.from_quat(self.quad)
        self.R_m = self.R.as_matrix()
        self.cnt = 0
        self.num = 0

        # self.odom = Odometry()
        self.const_vel_B = np.array([[1], [0.0], [0.075]])
        self.vel_B = np.zeros([3,1])
        self.vel_W = np.zeros([4,1])
        self.ref_vel = np.zeros([4,1])
        self.ref_vel_filter = np.zeros([4,1])
        self.const_vel_W = np.zeros([4,1])
        self.vel_W_history = []  # 存储历史速度数据
        self.ref_vel_history = []  # 存储历史参考速度数据
        self.ref_const_vel_history = []  # 存储历史参考速度数据

        self.time = rospy.Time.now()
        self.control_command = ControlCommand()
        self.control_command.control_mode = self.control_command.BODY_RATES

        self.aruco_reached = False
        self.ring_reached = False

        # get sensor topics
        imu_topic = rospy.get_param('~imu_topic', "/mavros/imu/data")
        odom_topic = rospy.get_param('~odom_topic', "/mavros/local_position/odom")

        # get ibvs topics
        ref_vel_topic = rospy.get_param('~ref_vel_topic', "/hummingbird/reference_vel")
        marker_topic = rospy.get_param('~marker_topic', "/hummingbird/markerpoint")

        # get control topics
        control_command_topic = rospy.get_param('~control_command_topic', "/mavros/feedthrough/control_command_input")
        # arm_topic = rospy.get_param('~arm_topic', "bridge/arm")

        self.imu_sub = rospy.Subscriber(imu_topic, Imu, self.imuCallback)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odomCallback)
        self.aruco_reached_sub = rospy.Subscriber('aruco/target_reached', Bool, self.aruco_reached_callback)
        self.ring_reached_sub = rospy.Subscriber('ring/target_reached', Bool, self.ring_reached_callback)
        self.control_command_pub = rospy.Publisher(control_command_topic, ControlCommand, queue_size=1)

        self.ref_vel_sub = message_filters.Subscriber(ref_vel_topic, xyzyawVel)
        self.marker_sub = message_filters.Subscriber(marker_topic, point_xyz)
        sync_listener = message_filters.ApproximateTimeSynchronizer([self.ref_vel_sub,self.marker_sub], 1, 0.05)
        sync_listener.registerCallback(self.refVelMarkerPointCallback)

        # # create a new thread for plotting
        # self.plot_thread = threading.Thread(target=self.plotVelocities)
        # self.plot_thread.daemon = True
        # self.plot_thread.start()
        self.pose_history = np.zeros((1, 7))
        self.pose_num = 0

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
        self.vel_W[0:3,:] = self.R_m.dot(self.vel_B)
        self.vel_W[3] = odom_msg.twist.twist.angular.z
        # print("[INFO] estimated vel_W is:", self.vel_W)
        # np.save("filename.npy", a)
        # b = np.load("filename.npy")
        if(self.pose_num == 0):
            self.pose_history[0,0] = odom_msg.pose.pose.position.x
            self.pose_history[0,1] = odom_msg.pose.pose.position.y
            self.pose_history[0,2] = odom_msg.pose.pose.position.z
            self.pose_history[0,3] = odom_msg.pose.pose.orientation.w
            self.pose_history[0,4] = odom_msg.pose.pose.orientation.x
            self.pose_history[0,5] = odom_msg.pose.pose.orientation.y
            self.pose_history[0,6] = odom_msg.pose.pose.orientation.z
            self.pose_num += 1
        else:
            self.pose_num += 1
            self.pose_history.resize((self.pose_num, 7), refcheck = False)
            self.pose_history[-1,0] = odom_msg.pose.pose.position.x
            self.pose_history[-1,1] = odom_msg.pose.pose.position.y
            self.pose_history[-1,2] = odom_msg.pose.pose.position.z
            self.pose_history[-1,3] = odom_msg.pose.pose.orientation.w
            self.pose_history[-1,4] = odom_msg.pose.pose.orientation.x
            self.pose_history[-1,5] = odom_msg.pose.pose.orientation.y
            self.pose_history[-1,6] = odom_msg.pose.pose.orientation.z
    
    def refVelMarkerPointCallback(self, ref_vel_msg: xyzyawVel, marker_msg: point_xyz):
        if(self.stop): return
        print("[INFO] estimated quant w,x,y,z is:", self.quad)
        print("[INFO] estimated vel_W is:", self.vel_W)
        print("[INFO] reference_vel is:", ref_vel_msg)
        if(ref_vel_msg.Vx == float("nan") or ref_vel_msg.Vy == float("nan") or ref_vel_msg.Vz == float("nan")): return
        self.x_current = np.array([0.0, 0.0, 1, self.quad[3], self.quad[0], self.quad[1], self.quad[2], self.vel_W[0][0], self.vel_W[1][0], self.vel_W[2][0], 
                              marker_msg.p1_x, marker_msg.p1_y, marker_msg.p1_z, marker_msg.p2_x, marker_msg.p2_y, marker_msg.p2_z,
                              marker_msg.p3_x, marker_msg.p3_y, marker_msg.p3_z, marker_msg.p4_x, marker_msg.p4_y, marker_msg.p4_z])
        # solve ocp for a fixed reference
        self.acados_solver.set(0, "lbx", self.x_current)
        self.acados_solver.set(0, "ubx", self.x_current)
        # 设置acados_solver的reference
        self.ref_vel = np.array([[ref_vel_msg.Vx], [ref_vel_msg.Vy], [ref_vel_msg.Vz], [ref_vel_msg.Vyaw]])
        self.ref_vel_filter[0:3,:] = self.vel_W[0:3,:] + self.alpha * (self.ref_vel[0:3,:] - self.vel_W[0:3,:])
        self.ref_vel_filter[3] = self.vel_W[3] + self.beta * (self.ref_vel[3] - self.vel_W[3])
        # self.ref_vel_filter[0:3,:] = self.vel_W[0:3,:] + self.alpha * (self.ref_vel[0:3,:] - self.vel_W[0:3,:])
        # 将所有阶段的数据存储在一个numpy数组中
        yref_all = np.zeros((self.N, 26))
        # for circle
        yref_all[:, 7] = self.ref_vel_filter[0, 0]
        yref_all[:, 8] = self.ref_vel_filter[1, 0]
        yref_all[:, 9] = self.ref_vel_filter[2, 0]
        yref_all[:, 25] = self.ref_vel_filter[3, 0]
        # for aruco
        # yref_all[:, 7] = self.ref_vel[0, 0]
        # yref_all[:, 8] = self.ref_vel[1, 0]
        # yref_all[:, 9] = self.ref_vel[2, 0]
        # yref_all[:, 25] = self.ref_vel[3, 0]
        # yref_all[:, 10] = 0.0
        # yref_all[:, 11] = 0.0
        yref_all[:, 12] = 3.0
        # yref_all[:, 13] = 0.0
        # yref_all[:, 14] = 0.0
        yref_all[:, 15] = 3.0
        # yref_all[:, 16] = 0.0
        # yref_all[:, 17] = 0.0
        yref_all[:, 18] = 3.0
        # yref_all[:, 19] = 0.0
        # yref_all[:, 20] = 0.0
        yref_all[:, 21] = 3.0
        yref_all[:, 22] = 9.81
        yref_all[:, 23] = 0.0
        yref_all[:, 24] = 0.0
        

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
        vx_e = self.ref_vel_filter[0, 0]
        vy_e = self.ref_vel_filter[1, 0]
        vz_e = self.ref_vel_filter[2, 0]

        yref_N = np.array([x_e, y_e, z_e, qw_e, qx_e,
                           qy_e, qz_e, vx_e, vy_e, vz_e,
                           0.0, 0.0, 3.0, 0.0, 0.0, 3.0,
                           0.0, 0.0, 3.0, 0.0, 0.0, 3.0])
        self.acados_solver.set(self.N, "yref", yref_N)

        comp_time = time.time()
        status = self.acados_solver.solve()
        if status != 0:
            print("acados returned status {} in closed loop iteration {}.".format(status, iter))
            self.publishCommand()
            return

        # manage timings
        elapsed = time.time() - comp_time
        self.tot_comp_sum += elapsed
        if elapsed > self.tcomp_max:
            self.tcomp_max = elapsed

        # get solution from acados_solver
        self.u0 = self.acados_solver.get(0, "u")
        self.time = rospy.Time.now()
        self.publishCommand()
        # simulate the system
        # self.acados_integrator.set("x", self.x_current)
        # self.acados_integrator.set("u", u0)
        # status = self.acados_integrator.solve()
        # if status != 0:
        #     raise Exception(
        #         'acados integrator returned status {}. Exiting.'.format(status))

        # # get state
        # print("[PREDICTION] predict state:", self.acados_integrator.get("x")[10:])
        self.iter += 1

        # plot the ref and cur velocity
        print("[INFO] self.vel_W:", self.vel_W)
        est_vel = np.zeros((4,1))
        est_vel[:,:] = self.vel_W
        ref_vel = np.zeros((4,1))
        ref_vel[:,:] = self.ref_vel_filter
        self.vel_W_history.append(est_vel)  # 存储历史速度数据
        self.ref_vel_history.append(ref_vel)  # 存储历史参考速度数据
        
        # print the computation times
        print("Total computation time: {}".format(self.tot_comp_sum))
        print("Average computation time: {}".format(self.tot_comp_sum / self.iter))
        print("Maximum computation time: {}".format(self.tcomp_max))

    def publishCommand(self):
        self.control_command.header = Header(stamp=self.time)
        self.control_command.armed = True
        self.control_command.expected_execution_time = self.time
        self.control_command.bodyrates = bodyratesToGeometry(self.u0)
        self.control_command.collective_thrust = self.u0[0]
        print("[INFO] OUTPUT is:\nthrust: ", self.control_command.collective_thrust, "\nbodyrates: ", self.control_command.bodyrates)
        self.control_command_pub.publish(self.control_command)
    
    def publishConstantCommand(self):
        self.x_constant = np.array([0.0, 0.0, 1.8, self.quad[3], self.quad[0], self.quad[1], self.quad[2], self.vel_W[0][0], self.vel_W[1][0], self.vel_W[2][0], 
                              0, 0, 5, 0, 0, 5,
                              0, 0, 5, 0, 0, 5])
        # solve ocp for a fixed reference
        self.acados_solver.set(0, "lbx", self.x_constant)
        self.acados_solver.set(0, "ubx", self.x_constant)

        # 设置acados_solver的reference
        const_vel_W = self.R_m.dot(self.const_vel_B)
        self.ref_vel[0:3, :] = const_vel_W
        self.ref_vel[3, 0] = 0.0
        self.ref_vel_filter[0:3,:] = self.vel_W[0:3,:] + self.alpha * (self.ref_vel[0:3,:] - self.vel_W[0:3,:])
        self.ref_vel_filter[3] = self.vel_W[3] + self.beta * (self.ref_vel[3] - self.vel_W[3])
        # 将所有阶段的数据存储在一个numpy数组中
        yref_all = np.zeros((self.N, 26))
        yref_all[:, 7] = self.ref_vel[0, 0]
        yref_all[:, 8] = self.ref_vel[1, 0]
        yref_all[:, 9] = self.ref_vel[2, 0]
        # yref_all[:, 10] = 0.0
        # yref_all[:, 11] = 0.0
        yref_all[:, 12] = 3.0
        # yref_all[:, 13] = 0.0
        # yref_all[:, 14] = 0.0
        yref_all[:, 15] = 3.0
        # yref_all[:, 16] = 0.0
        # yref_all[:, 17] = 0.0
        yref_all[:, 18] = 3.0
        # yref_all[:, 19] = 0.0
        # yref_all[:, 20] = 0.0
        yref_all[:, 21] = 3.0
        yref_all[:, 22] = 10.02
        yref_all[:, 23] = 0.0
        yref_all[:, 24] = 0.0
        yref_all[:, 25] = self.ref_vel_filter[3, 0]
        
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
        vx_e = self.ref_vel[0, 0]
        vy_e = self.ref_vel[1, 0]
        vz_e = self.ref_vel[2, 0]

        yref_N = np.array([x_e, y_e, z_e, qw_e, qx_e,
                           qy_e, qz_e, vx_e, vy_e, vz_e,
                           0.0, 0.0, 3.0, 0.0, 0.0, 3.0,
                           0.0, 0.0, 3.0, 0.0, 0.0, 3.0])
        self.acados_solver.set(self.N, "yref", yref_N)

        comp_time = time.time()
        status = self.acados_solver.solve()
        if status != 0:
            print("acados returned status {} in closed loop iteration {}.".format(status, iter))
            self.publishCommand()
            return

        # manage timings
        elapsed = time.time() - comp_time
        self.tot_comp_sum += elapsed
        if elapsed > self.tcomp_max:
            self.tcomp_max = elapsed

        # get solution from acados_solver
        self.u0 = self.acados_solver.get(0, "u")
        self.time = rospy.Time.now()
        self.publishCommand()
    
    def plotVelocities(self):
        np.save("/home/zyh/ibvs_rpg_ws/src/mpc_pkg/scripts/outputs/traj_2.npy", self.pose_history)
        fig, axs = plt.subplots(4, 1, figsize=(12, 9))
        # print("[OUTPUT]self.vel_W_history:", self.vel_W_history)
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
        axs[3].plot([v[3] for v in self.vel_W_history], label='Actual Vyaw')
        axs[3].plot([v[3] for v in self.ref_vel_history], label='Reference Vyaw')
        axs[3].set_ylabel('Vyaw [m/s]')
        axs[3].legend()
        plt.show()

    def aruco_reached_callback(self, msg):
        self.aruco_reached = msg.data

    def ring_reached_callback(self, msg):
        self.ring_reached = msg.data

    def get_ring_reached(self):
        return self.ring_reached

    def get_aruco_reached(self):
        return self.aruco_reached
    
    def stop2publishThrust(self):
        if(self.stop == False):
            self.cnt += 1
            # self.const_vel_B[2, 0] = self.ref_vel_filter[2, 0]
            # self.ref_const_vel_horizon = [0.0 for _ in range(self.horizon)]
        self.stop = True
        self.publishConstantCommand()
    
    def setContinue(self):
        if(self.stop == True):
            print("[INFO] cnt!!!!!!!!!!!!!!!!! is:", self.cnt)
            self.num = 0
        #     self.ref_vel_horizon = [0.0 for _ in range(self.horizon)]
        self.stop = False
        # self.publishConstantCommand()

    def getCnt(self):
        return self.cnt


if __name__ == "__main__":
    rospy.init_node('acados_mpc_control', anonymous=True)
    # for circle
    quadrotorController = QuadrotorController(2, 5)
    # for aruco
    # quadrotorController = QuadrotorController(2, 4)
    rate = rospy.Rate(50)  # 设置循环频率为50Hz
    while not rospy.is_shutdown():
        # 在这里执行其他操作，例如检查程序是否应该退出
        if quadrotorController.get_aruco_reached():
            break
        if quadrotorController.get_ring_reached():
            quadrotorController.stop2publishThrust()
        else:
            quadrotorController.setContinue()
        
        if quadrotorController.getCnt()==4:
            continue
            # TODO:plot results and stop the quadrotor
            # break
        # 控制循环的频率
        rate.sleep()
    quadrotorController.plotVelocities()