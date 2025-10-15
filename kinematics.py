import numpy as np
import math

def inverse_kinematics(self, xyz_coords):
    # xyz_coords é um array NumPy: [x, y, z]
    y_linha = np.linalg.norm([xyz_coords[0], xyz_coords[1]]) - self.L1
    L = np.linalg.norm([xyz_coords[2], y_linha])

    # math.acos, math.atan2, etc.
    alpha = math.acos(np.clip((self.L2**2 + self.L3**2 - L**2) / (2 * self.L2 * self.L3), -1.0, 1.0))
    beta = math.acos(np.clip((L**2 + self.L2**2 - self.L3**2) / (2 * self.L2 * L), -1.0, 1.0))

    angles_rad_tibia = -math.pi + alpha
    angles_rad_femur = math.atan2(xyz_coords[2], y_linha) + beta
    angles_rad_ombro = math.atan2(xyz_coords[1], xyz_coords[0])
    return np.array([angles_rad_ombro, angles_rad_femur, angles_rad_tibia])

def forward_kinematics(self, angles_rad, L1, L2, L3):

    ''' Calculate the XYZ position of the foot tip given the joint angles in radians. 
        returns a NumPy array [x, y, z].
    '''
    theta1, theta2, theta3 = angles_rad # ombro, femur, tibia

    x = -np.sin(theta1) * (L1 + L3 * np.cos(theta2 + theta3) + L2 * np.cos(theta2))
    y = np.cos(theta1) * (L1 + L3 * np.cos(theta2 + theta3) + L2 * np.cos(theta2))
    z = L3 * np.sin(theta2 + theta3) + L2 * np.sin(theta2)

    return np.array([x, y, z])

def trajectory_linear(self, xyz_ini, k, offset, angle_rad, P0, P1, P2, P3):
    xyz = np.zeros(3)
    kn = (k + offset) % TOTAL_PONTOS
    if kn < METADE_PONTOS:
        t = float(kn) / (METADE_PONTOS - 1)
        u = 1 - t
        bezier_x = u**3 * P0[0] + 3 * u**2 * t * P1[0] + 3 * u * t**2 * P2[0] + t**3 * P3[0]
        bezier_y = u**3 * P0[1] + 3 * u**2 * t * P1[1] + 3 * u * t**2 * P2[1] + t**3 * P3[1]
        xyz[0] = xyz_ini[0] + np.cos(angle_rad) * (-xyz_ini[0] + bezier_x)
        xyz[1] = xyz_ini[1] + np.sin(angle_rad) * (-xyz_ini[1] + bezier_x)
        xyz[2] = bezier_y
        return xyz
    
    xyz[0] = xyz_ini[0] + np.cos(angle_rad) * (-xyz_ini[0] + P3[0] + (P0[0] - P3[0]) * (float(kn - METADE_PONTOS) / (METADE_PONTOS - 1)))
    xyz[1] = xyz_ini[1] + np.sin(angle_rad) * (-xyz_ini[1] + P3[0] + (P0[0] - P3[0]) * (float(kn - METADE_PONTOS) / (METADE_PONTOS - 1)))
    xyz[2] = xyz_ini[2]
    
    return xyz