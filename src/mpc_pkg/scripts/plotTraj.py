from plotFnc import *

def plot3dDroneTraj(traj, save=False, type=1):
    plt.style.use('seaborn')
    x = traj[:,0]
    y = traj[:,1]
    z = traj[:,2]

    fig, ax = plt.subplots()
    plt.title('Performed trajectory')
    ax = plt.axes(projection = "3d")
    name = 'traj_' + str(type)
    ax.plot3D(x, y, z, label=name)
    ax.set_xlabel("x[m]")
    ax.set_ylabel("y[m]")
    ax.set_zlabel("z[m]")
    ax.legend()


    NUM_STEPS = traj.shape[0]
    MEAS_EVERY_STEPS = 200 # 100 for aruco

    X0 = [traj[0,0], traj[0,1], traj[0,2]]
    q0 = [traj[0,3], traj[0,4], traj[0,5], traj[0,6]]
    plotDrone3D(ax,X0,q0)
    X0 = [traj[-1,0], traj[-1,1], traj[-1,2]]
    q0 = [traj[-1,3], traj[-1,4], traj[-1,5], traj[-1,6]]
    plotDrone3D(ax,X0,q0)

    for step in range(NUM_STEPS):
        if step !=0 and step % MEAS_EVERY_STEPS ==0:
            X = [traj[step,0], traj[step,1], traj[step,2]]
            q = [traj[step,3], traj[step,4], traj[step,5], traj[step,6]]
            plotDrone3D(ax,X,q)

    axisEqual3D(ax)
    # ax.set_xlim3d(-5, 5)
    # ax.set_ylim3d(-5, 5)
    # ax.set_zlim3d(0, 5)

    if save == True:
        fig_path = '/home/zyh/ibvs_rpg_ws/src/mpc_pkg/scripts/outputs/'+ name +'_3D.png'
        fig.savefig(fig_path, dpi=300)

def plot3dTraj(traj, save=False, type=1):
    plt.style.use('seaborn')
    x = traj[:,0]
    y = traj[:,1]
    z = traj[:,2]

    fig, ax = plt.subplots()
    plt.title('Performed trajectory')
    ax = plt.axes(projection = "3d")
    name = 'traj_' + str(type)
    ax.plot3D(x, y, z, label=name)
    ax.set_xlabel("x[m]")
    ax.set_ylabel("y[m]")
    ax.set_zlabel("z[m]")
    ax.legend()


    NUM_STEPS = traj.shape[0]
    MEAS_EVERY_STEPS = 100

    # X0 = [traj[0,0], traj[0,1], traj[0,2]]
    # q0 = [traj[0,3], traj[0,4], traj[0,5], traj[0,6]]
    # plotDrone3D(ax,X0,q0)
    # X0 = [traj[-1,0], traj[-1,1], traj[-1,2]]
    # q0 = [traj[-1,3], traj[-1,4], traj[-1,5], traj[-1,6]]
    # plotDrone3D(ax,X0,q0)

    # for step in range(NUM_STEPS):
    #     if step !=0 and step % MEAS_EVERY_STEPS ==0:
    #         X = [traj[step,0], traj[step,1], traj[step,2]]
    #         q = [traj[step,3], traj[step,4], traj[step,5], traj[step,6]]
    #         plotDrone3D(ax,X,q)

    axisEqual3D(ax)
    # ax.set_xlim3d(-5, 5)
    # ax.set_ylim3d(-5, 5)
    # ax.set_zlim3d(0, 5)

    if save == True:
        fig_path = '/home/zyh/ibvs_rpg_ws/src/mpc_pkg/scripts/outputs/'+ name +'_3D.png'
        fig.savefig(fig_path, dpi=300)

def plot3dTraj12(traj_1, traj_2, save=False):
    plt.style.use('seaborn')

    x1 = traj_1[:,0]
    y1 = traj_1[:,1]
    z1 = traj_1[:,2]

    x2 = traj_2[:,0]
    y2 = traj_2[:,1]
    z2 = traj_2[:,2]

    fig, ax = plt.subplots()
    plt.title('Performed trajectory')
    ax = plt.axes(projection = "3d")
    ax.plot3D(x1, y1, z1, label='traj_1')
    ax.plot3D(x2, y2, z2, label='traj_2')
    ax.set_xlabel("x[m]")
    ax.set_ylabel("y[m]")
    ax.set_zlabel("z[m]")
    ax.legend()


    # NUM1_STEPS = traj_1.shape[0]
    # NUM2_STEPS = traj_2.shape[0]
    # MEAS_EVERY_STEPS = 100

    # X0 = [traj_1[0,0], traj_1[0,1], traj_1[0,2]]
    # q0 = [traj_1[0,3], traj_1[0,4], traj_1[0,5], traj_1[0,6]]
    # plotDrone3D(ax,X0,q0)

    # for step in range(NUM1_STEPS):
    #     if step !=0 and step % MEAS_EVERY_STEPS ==0:
    #         X = [traj_1[step,0], traj_1[step,1], traj_1[step,2]]
    #         q = [traj_1[step,3], traj_1[step,4], traj_1[step,5], traj_1[step,6]]
    #         plotDrone3D(ax,X,q)
    
    # X0 = [traj_2[0,0], traj_2[0,1], traj_2[0,2]]
    # q0 = [traj_2[0,3], traj_2[0,4], traj_2[0,5], traj_2[0,6]]
    # plotDrone3D(ax,X0,q0)

    # for step in range(NUM2_STEPS):
    #     if step !=0 and step % MEAS_EVERY_STEPS ==0:
    #         X = [traj_2[step,0], traj_2[step,1], traj_2[step,2]]
    #         q = [traj_2[step,3], traj_2[step,4], traj_2[step,5], traj_2[step,6]]
    #         plotDrone3D(ax,X,q)

    axisEqual3D(ax)
    # ax.set_xlim3d(-5, 5)
    # ax.set_ylim3d(-5, 5)
    # ax.set_zlim3d(0, 5)

    if save == True:
        fig.savefig('/home/zyh/ibvs_rpg_ws/src/mpc_pkg/scripts/outputs/trajs3D.png', dpi=300)

if __name__ == "__main__":
    traj1 = np.load("/home/zyh/ibvs_rpg_ws/src/mpc_pkg/scripts/outputs/traj_1.npy")
    traj2 = np.load("/home/zyh/ibvs_rpg_ws/src/mpc_pkg/scripts/outputs/traj_2.npy")
    plot3dDroneTraj(traj1, True, 1)
    plot3dDroneTraj(traj2, True, 2)
    plot3dTraj12(traj1, traj2, True)
    plot3dTraj(traj2, True, 2)
    plt.show()
