from casadi import *

def drone_model():
    # define structs
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "Translational_drone"

    # Quadrotor intrinsic parameters

    # constants
    g = 9.81 # m/s^2
    epsilon = 0.1
    # pixel range, u_rang = c_u / fa = 320 / 462.138 = 0.69, v_rang = c_v / fa = 240 / 462.138 = 0.46
    u_range = 0.69; v_range = 0.46
    ## CasAdi Model
    # set up states and controls
    px = MX.sym("px")
    py = MX.sym("py")
    pz = MX.sym("pz")
    qw = MX.sym("qw")
    qx = MX.sym("qx")
    qy = MX.sym("qy")
    qz = MX.sym("qz")
    vx = MX.sym("vx")
    vy = MX.sym("vy")
    vz = MX.sym("vz")
    p1_u = MX.sym("p1_u")
    p1_v = MX.sym("p1_v")
    p1_z = MX.sym("p1_z")
    p2_u = MX.sym("p2_u")
    p2_v = MX.sym("p2_v")
    p2_z = MX.sym("p2_z")
    p3_u = MX.sym("p3_u")
    p3_v = MX.sym("p3_v")
    p3_z = MX.sym("p3_z")
    p4_u = MX.sym("p4_u")
    p4_v = MX.sym("p4_v")
    p4_z = MX.sym("p4_z")
    x  = vertcat(px, py, pz, qw, qx, qy, qz, vx, vy, vz,
                 p1_u, p1_v, p1_z, p2_u, p2_v, p2_z, p3_u, p3_v, p3_z, p4_u, p4_v, p4_z)

    # controls
    T  = MX.sym("T")
    wx = MX.sym("wx")
    wy = MX.sym("wy")
    wz = MX.sym("wz")
    u = vertcat(T, wx, wy, wz)

    # xdot
    pxdot = MX.sym("pxdot")
    pydot = MX.sym("pydot")
    pzdot = MX.sym("pzdot")
    qwdot = MX.sym("qwdot")
    qxdot = MX.sym("qxdot")
    qydot = MX.sym("qydot")
    qzdot = MX.sym("qzdot")
    vxdot = MX.sym("vxdot")
    vydot = MX.sym("vydot")
    vzdot = MX.sym("vzdot")
    p1_udot = MX.sym("p1_udot")
    p1_vdot = MX.sym("p1_vdot")
    p1_zdot = MX.sym("p1_zdot")
    p2_udot = MX.sym("p2_udot")
    p2_vdot = MX.sym("p2_vdot")
    p2_zdot = MX.sym("p2_zdot")
    p3_udot = MX.sym("p3_udot")
    p3_vdot = MX.sym("p3_vdot")
    p3_zdot = MX.sym("p3_zdot")
    p4_udot = MX.sym("p4_udot")
    p4_vdot = MX.sym("p4_vdot")
    p4_zdot = MX.sym("p4_zdot")
    xdot  = vertcat(pxdot, pydot, pzdot, qwdot, qxdot, qydot, qzdot, vxdot, vydot, vzdot,
                    p1_udot, p1_vdot, p1_zdot, p2_udot, p2_vdot, p2_zdot, p3_udot, p3_vdot, p3_zdot, p4_udot, p4_vdot, p4_zdot)
    
    # intermediate variables
    vb_x = vx * (1 - 2*qy*qy - 2*qz*qz) + vy * 2*(qx*qy - qw*qz) + vz * 2*(qx*qz + qw*qy)
    vb_y = vx * 2*(qx*qy + qw*qz) + vy * (1 - 2*qx*qx - 2*qz*qz) + vz * 2*(qy*qz - qw*qx)
    vb_z = vx * 2*(qx*qz - qw*qy) + vy * 2*(qy*qz + qw*qx) + vz * (1 - 2*qx*qx - 2*qy*qy)

    # algebraic variables 
    z = vertcat([])

    # parameters
    p = vertcat([])

    # dynamics
    f_expl = vertcat(
        vx,
        vy,
        vz,
        0.5 * ( - wx * qx - wy * qy - wz * qz),
        0.5 * (   wx * qw + wz * qy - wy * qz),
        0.5 * (   wy * qw - wz * qx + wx * qz),
        0.5 * (   wz * qw + wy * qx - wx * qy),
        2 * ( qw * qy + qx * qz ) * T,
        2 * ( qy * qz - qw * qx ) * T,
        ( ( 1 - 2 * qx * qx - 2 * qy * qy ) * T ) - g,
        (- vb_x  + vb_z * p1_u + p1_u * p1_v * wx * p1_z - (1 + p1_u * p1_u) * wy * p1_z + p1_v * wz * p1_z) / (p1_z + epsilon),
        (- vb_y  + vb_z * p1_v + (1 + p1_v * p1_v) * wx * p1_z - p1_u * p1_v * wy * p1_z - p1_u * wz * p1_z) / (p1_z + epsilon),
        - vb_x,
        (- vb_x  + vb_z * p2_u + p2_u * p2_v * wx * p2_z - (1 + p2_u * p2_u) * wy * p2_z + p2_v * wz * p2_z) / (p2_z + epsilon),
        (- vb_y  + vb_z * p2_v + (1 + p2_v * p2_v) * wx * p2_z - p2_u * p2_v * wy * p2_z - p2_u * wz * p2_z) / (p2_z + epsilon),
        - vb_x,
        (- vb_x  + vb_z * p3_u + p3_u * p3_v * wx * p3_z - (1 + p3_u * p3_u) * wy * p3_z + p3_v * wz * p3_z) / (p3_z + epsilon),
        (- vb_y  + vb_z * p3_v + (1 + p3_v * p3_v) * wx * p3_z - p3_u * p3_v * wy * p3_z - p3_u * wz * p3_z) / (p3_z + epsilon),
        - vb_x,
        (- vb_x  + vb_z * p4_u + p4_u * p4_v * wx * p4_z - (1 + p4_u * p4_u) * wy * p4_z + p4_v * wz * p4_z) / (p4_z + epsilon),
        (- vb_y  + vb_z * p4_v + (1 + p4_v * p4_v) * wx * p4_z - p4_u * p4_v * wy * p4_z - p4_u * wz * p4_z) / (p4_z + epsilon),
        - vb_x 
    )
    
    # model.phi_min = -80 * np.pi / 180
    # model.phi_max =  80 * np.pi / 180

    # model.theta_min = -80 * np.pi / 180
    # model.theta_max =  80 * np.pi / 180

    # input bounds
    
    # model.thrust_max = 0.9 * ((46.3e-3 * g)) # 90 % of max_thrust (max_thrust = 57g in research papers) ----- ( max_thrsut = 46g when tested) 
    model.thrust_max = 1.5 * g
    model.thrust_min = 0.1 * model.thrust_max
    model.u_max = u_range
    model.u_min = -u_range
    model.v_max = v_range
    model.v_min = -v_range

    # model.torque_max = 1 / 2 * model.thrust_max * length # divided by 2 since we only have 2 propellers in a planar quadrotor
    # model.torque_max = 0.1 * model.torque_max # keeping 10% margin for steering torque. This is done because the torque_max 
    #                                           # is the maximum torque that can be given around any one axis. But, we are going to
    #                                           # limit the torque greatly.
    # model.torque_min = - model.torque_max

    # define initial condition
    model.x0 = np.array([0.0, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                         0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0])

    # define model struct
    params = types.SimpleNamespace()
    # params.m = m
    # params.J = J
    # params.length = length
    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.z = z
    model.p = p
    model.name = model_name
    model.params = params

    return model