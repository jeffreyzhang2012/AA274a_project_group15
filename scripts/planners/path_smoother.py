import numpy as np
import scipy.interpolate
from numpy.core.function_base import linspace
from scipy.interpolate.fitpack import splev, splrep

def compute_smoothed_traj(path, V_des, alpha, dt):
    """
    Fit cubic spline to a path and generate a resulting trajectory for our
    wheeled robot.

    Inputs:
        path (np.array [N,2]): Initial path
        V_des (float): Desired nominal velocity, used as a heuristic to assign nominal
            times to points in the initial path
        alpha (float): Smoothing parameter (see documentation for
            scipy.interpolate.splrep)
        dt (float): Timestep used in final smooth trajectory
    Outputs:
        traj_smoothed (np.array [N,7]): Smoothed trajectory
        t_smoothed (np.array [N]): Associated trajectory times
    Hint: Use splrep and splev from scipy.interpolate
    """
    ########## Code starts here ##########
    timeNom = np.array([np.linalg.norm(np.array(path[i])-np.array(path[i-1]))/V_des if i > 0 else 0 for i in range(len(path))])
    for i in range(1, len(timeNom)):
        timeNom[i] += timeNom[i-1]
    splX = splrep(timeNom, np.array([path[i][0] for i in range(len(path))]))
    splY = splrep(timeNom, np.array([path[i][1] for i in range(len(path))]))
    totTime = timeNom[-1]
    t_smoothed = linspace(0.0, totTime, round(totTime/dt))
    x_d = splev(t_smoothed, splX, der=0)
    y_d = splev(t_smoothed, splY, der=0)
    xd_d = splev(t_smoothed, splX, der=1)
    yd_d = splev(t_smoothed, splY, der=1)
    xdd_d = splev(t_smoothed, splX, der=2)
    ydd_d = splev(t_smoothed, splY, der=2)
    theta_d = [np.arctan2(yd_d[i], xd_d[i]) for i in range(len(t_smoothed))]
    ########## Code ends here ##########
    traj_smoothed = np.stack([x_d, y_d, theta_d, xd_d, yd_d, xdd_d, ydd_d]).transpose()

    return traj_smoothed, t_smoothed
