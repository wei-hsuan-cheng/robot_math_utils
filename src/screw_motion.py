import numpy as np
from scipy.spatial.transform import Rotation 
import modern_robotics as mr
from tmr_utils import TMRUtils

degree = True
r2d = 180 / np.pi
d2r = 1 / r2d
m2mm = 1000
mm2m = 1 / 1000

tmr = TMRUtils()

def send_script(script):
    print(f"\n{script}")

def ToStr(array):
    return ", ".join(str(array_num) for array_num in array)

def MoveCartesian(Cpose):
    send_script("PTP(\"CPP\"," + ToStr(Cpose) + ",6,500,100,false)") # TCP Speed = 55 [mm/s] / 5 %, 110 [mm/s] / 10 %
    
def Cmotions(cmd_Cpose):
    for i in range(len(cmd_Cpose)):
        MoveCartesian(cmd_Cpose[i])

def screw_motion():
    '''
    End-effector pose w.r.t. base
    '''
    pose_b_e = np.array([100., 100.0, 500.0, 0.0, 0.0, 0.0]) # [mm, deg]


    '''
    Relative motion (from current to desired end-effector pose)
    '''
    pose_e_e_d = np.array([100., 0.0, 0.0, 0.0, 45, 0.0]) # [mm, deg]
    R_e_e_d, p_e_e_d = tmr.R6Pose2Rp(pose_e_e_d)
    print(f"\n\n ----- End-effector relative motion -----")
    print(f"\npose_e_e_d [mm, deg]= {pose_e_e_d}")


    '''
    Represent in screw motion
    1) Translation: p_e_e_d
    2) Rotation: R_e_e_d = exp(so3) = exp(uhat * theta)
    '''
    so3_e_e_d = mr.so3ToVec( mr.MatrixLog3(R_e_e_d) ) # [rad]
    uhat_e_e_d, theta_e_e_d = mr.AxisAng3(so3_e_e_d) # [unit vector, rad]
    print(f"\n\n ----- The screw motion -----")
    print(f"\nso3_e_e_d [deg] = {so3_e_e_d * r2d}")
    print(f"\nuhat_e_e_d [unit vector] = {uhat_e_e_d}")
    print(f"\ntheta_e_e_d [deg] = {theta_e_e_d * r2d}")


    '''
    Divide the screw motion into "n_screw" parts, 
    totally n + 1 waypoints: 
    initial (wp_0), 
    wp_1, ..., 
    wp_n, 
    desired (wp_n + 1)
    '''
    print(f"\n\n ----- Generate waypoints -----")
    n_screw = 10
    pso3_e_e_d_wps = []
    pose_e_e_d_wps = []
    pose_b_e_d_wps = []

    for i in range(n_screw):
        p_e_e_d_i = p_e_e_d  * ( (i + 1) / n_screw )
        so3_e_e_d_i = so3_e_e_d * ( (i + 1) / n_screw )
        pso3_e_e_d_i = np.concatenate((p_e_e_d_i, so3_e_e_d_i), axis=None)
        pso3_e_e_d_wps.append(pso3_e_e_d_i)

        R_e_e_d_i = mr.MatrixExp3( mr.VecToso3(so3_e_e_d_i) )
        pose_e_e_d_i = tmr.Rp2R6Pose(R_e_e_d_i, p_e_e_d_i)
        pose_e_e_d_wps.append(pose_e_e_d_i)

        pose_b_e_d_i = tmr.TransformR6Pose(pose_b_e, pose_e_e_d_i)
        pose_b_e_d_wps.append(pose_b_e_d_i)

        # print(f"\npso3_e_e_d_{i} [mm, rad] = {pso3_e_e_d_i.tolist()}")
        # print(f"\npose_e_e_d_{i} [mm, deg] = {pose_e_e_d_i.tolist()}")
        print(f"\npose_b_e_d_{i} [mm, deg] = {pose_b_e_d_i.tolist()}")
    
    pso3_e_e_d_wps = np.asarray(pso3_e_e_d_wps)
    pose_e_e_d_wps = np.asarray(pose_e_e_d_wps)
    pose_b_e_d_wps = np.asarray(pose_b_e_d_wps)

    '''
    Send robot pose commands
    '''
    print(f"\n\n ----- Send pose commands -----")
    cmd_Cpose = pose_b_e_d_wps
    Cmotions(cmd_Cpose)

def main():
    screw_motion()

if __name__ == '__main__':
    main()