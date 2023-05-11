from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from drone_model import drone_model
from acados_integrator import export_drone_integrator

import scipy.linalg
import numpy as np

def acados_settings(Ts, Tf, N):

    # create OCP object to formulate the optimization
    ocp = AcadosOcp()

    # export model
    model = drone_model()

    # constants
    g = 9.82 # m/s^2

    # define acados ODE 
    model_ac = AcadosModel()
    model_ac.f_impl_expr = model.f_impl_expr
    model_ac.f_expl_expr = model.f_expl_expr
    model_ac.x = model.x
    model_ac.xdot = model.xdot
    model_ac.u = model.u
    model_ac.z = model.z
    model_ac.p = model.p
    model_ac.name = model.name
    ocp.model = model_ac

    # dimensions 
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    # nx_e = nx - 12
    # ny_e = nx_e
    # discretization 
    ocp.dims.N = N
    
    # set cost 
    Q = np.eye(nx)
    Q[0][0] = 0  # weight of px
    Q[1][1] = 0  # weight of py
    Q[2][2] = 0  # weight of pz
    Q[3][3] = 0  # weight of qw
    Q[4][4] = 0  # weight of qx
    Q[5][5] = 0  # weight of qy
    Q[6][6] = 0  # weight of qz
    # for ring
    Q[7][7] = 2e2  # weight of vx
    Q[8][8] = 2e2  # weight of vy
    Q[9][9] = 6e3  # weight of vz
    # for aruco
    # Q[7][7] = 150  # weight of vx
    # Q[8][8] = 150  # weight of vy
    # Q[9][9] = 3e3  # weight of vz
    Q[10][10] = 0  # weight of p1_u
    Q[11][11] = 0  # weight of p1_v
    Q[12][12] = 0  # weight of p1_z
    Q[13][13] = 0  # weight of p2_u
    Q[14][14] = 0  # weight of p2_v
    Q[15][15] = 0  # weight of p2_z
    Q[16][16] = 0  # weight of p3_u
    Q[17][17] = 0  # weight of p3_v
    Q[18][18] = 0  # weight of p3_z
    Q[19][19] = 0  # weight of p4_u
    Q[20][20] = 0  # weight of p4_v
    Q[21][21] = 0  # weight of p4_z

    R = np.eye(nu)
    # for ring
    R[0][0] = 1e1  # weight of Thrust
    R[1][1] = 1e1  # weight of wx
    R[2][2] = 1e1  # weight of wy
    R[3][3] = 6e3  # weight of wz
    # for aruco
    # R[0][0] = 1e0  # weight of Thrust
    # R[1][1] = 1e1  # weight of wx
    # R[2][2] = 1e1  # weight of wy
    # R[3][3] = 1e2  # weight of wz

    # Qe = np.eye(ny_e)
    Qe = np.eye(nx)
    Qe[0][0] = Q[0][0]  # terminal weight of px
    Qe[1][1] = Q[1][1]  # terminal weight of py
    Qe[2][2] = Q[2][2]  # terminal weight of pz
    Qe[3][3] = Q[3][3]  # terminal weight of qw
    Qe[4][4] = Q[4][4]  # terminal weight of qx
    Qe[5][5] = Q[5][5]  # terminal weight of qy
    Qe[6][6] = Q[6][6]  # terminal weight of qz
    Qe[7][7] = Q[7][7]  # terminal weight of vx
    Qe[8][8] = Q[8][8]  # terminal weight of vy
    Qe[9][9] = Q[9][9]  # terminal weight of vz
    Qe[10][10] = 0  # terminal weight of p1_u
    Qe[11][11] = 0  # terminal weight of p1_v
    Qe[12][12] = 0  # terminal weight of p1_z
    Qe[13][13] = 0  # terminal weight of p2_u
    Qe[14][14] = 0  # terminal weight of p2_v
    Qe[15][15] = 0  # terminal weight of p2_z
    Qe[16][16] = 0  # terminal weight of p3_u
    Qe[17][17] = 0  # terminal weight of p3_v
    Qe[18][18] = 0  # terminal weight of p3_z
    Qe[19][19] = 0  # terminal weight of p4_u
    Qe[20][20] = 0  # terminal weight of p4_v
    Qe[21][21] = 0  # terminal weight of p4_z

    ocp.cost.cost_type   = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"
    
    ocp.cost.W   = scipy.linalg.block_diag(Q,R)
    ocp.cost.W_e = Qe

    Vx = np.zeros((ny,nx))
    Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx = Vx

    Vu = np.zeros((ny, nu))
    Vu[-4:,-4:] = np.eye(nu)
    ocp.cost.Vu = Vu

    # Vx_e = np.zeros((ny_e, nx_e))
    # Vx_e[:nx_e, :nx_e] = np.eye(nx_e)
    # ocp.cost.Vx_e = Vx_e
    Vx_e = np.zeros((nx, nx))
    Vx_e[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx_e = Vx_e
    
    # Initial reference trajectory (will be overwritten during the simulation)
    x_ref = np.array([1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0])
    # x_e_ref = np.array([1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    ocp.cost.yref   = np.concatenate((x_ref, np.array([g, 0.0, 0.0, 0.0])))
    # ocp.cost.yref   = np.concatenate((x_ref, np.array([model.params.m * g, 0.0, 0.0, 0.0])))

    ocp.cost.yref_e = x_ref

    # set constraints on thrust and angular velocities
    # for ring
    # ocp.constraints.lbu   = np.array([model.thrust_min, -0.4*np.pi, -0.4*np.pi, -0.4*np.pi])
    # ocp.constraints.ubu   = np.array([model.thrust_max,  0.4*np.pi,  0.4*np.pi,  0.4*np.pi])
    # ocp.constraints.idxbu = np.array([0,1,2,3])
    # for aruco
    ocp.constraints.lbu   = np.array([model.thrust_min, -np.pi, -np.pi, -np.pi])
    ocp.constraints.ubu   = np.array([model.thrust_max,  np.pi,  np.pi,  np.pi])
    ocp.constraints.idxbu = np.array([0,1,2,3])

    # TODO:Path constraints
    # tol = 0.1
    # def path_constraint1(x, p, t):
    #     if t < Tf/3 - tol or t > Tf/3 + tol:
    #         return np.zeros((1,))
    #     else:
    #         return np.array([x[2] - 1])

    # def path_constraint2(x, p, t):
    #     if t < 2*Tf/3 - tol or t > 2*Tf/3 + tol:
    #         return np.zeros((1,))
    #     else:
    #         return np.array([x[2] - 2])
    # model_ac.con_phi_expr = lambda x, t, p: np.concatenate((path_constraint1(x, p, t), path_constraint2(x, p, t)))

    ocp.constraints.lbx     = np.array([model.u_min, model.v_min, model.z_min, model.u_min, model.v_min, model.z_min, model.u_min, model.v_min, model.z_min, model.u_min, model.v_min, model.z_min])
    ocp.constraints.ubx     = np.array([model.u_max, model.v_max, model.z_max, model.u_max, model.v_max, model.z_max, model.u_max, model.v_max, model.z_max, model.u_max, model.v_max, model.z_max])
    ocp.constraints.idxbx   = np.array([10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21])

    '''
    ocp.constraints.lbx = np.array([-15.0, -15.0, -15.0]) # lower bounds on the velocity states
    ocp.constraints.ubx = np.array([ 15.0,  15.0,  15.0]) # upper bounds on the velocity states
    ocp.constraints.idxbx = np.array([3, 4, 5])
    '''

    # set initial condition
    ocp.constraints.x0 = model.x0

    # set QP solver and integration
    ocp.solver_options.tf = Tf
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 3
    ocp.solver_options.nlp_solver_max_iter = 200
    ocp.solver_options.tol = 1e-4

    # create ocp solver 
    acados_solver = AcadosOcpSolver(ocp, json_file=(model_ac.name + "_" + "acados_ocp.json"))

    acados_integrator = export_drone_integrator(Ts, model_ac)


    return model, acados_solver, acados_integrator