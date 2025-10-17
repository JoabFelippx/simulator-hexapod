# hexapod/kinematics.py

import numpy as np
import math
# from config import L1_COXA, L2_FEMUR, L3_TIBIA


L1_COXA = 5.28
L2_FEMUR = 10.4
L3_TIBIA = 14.62


def forward_kinematics(angles_rad):
    coxa_angle, femur_angle, tibia_angle = angles_rad

    # xyz.x = -sin(angles_rad.ombro)*(this->L1 + this->L3*cos(angles_rad.femur + angles_rad.tibia) + this->L2*cos(angles_rad.femur));
																																																																										 
    # xyz.y = cos(angles_rad.ombro)*(this->L1 + this->L3*cos(angles_rad.femur + angles_rad.tibia) +this->L2*cos(angles_rad.femur));
    # xyz.z = this->L3*sin(angles_rad.femur + angles_rad.tibia) + this->L2*sin(angles_rad.femur);
    
    x = -math.sin(coxa_angle) * (L1_COXA + L3_TIBIA * math.cos(femur_angle + tibia_angle) + L2_FEMUR * math.cos(femur_angle))
    y = math.cos(coxa_angle) * (L1_COXA + L3_TIBIA * math.cos(femur_angle + tibia_angle) + L2_FEMUR * math.cos(femur_angle))
    z = L3_TIBIA * math.sin(femur_angle + tibia_angle) + L2_FEMUR * math.sin(femur_angle)
    
    return np.array([x, y, z])

def inverse_kinematics(target_pos):
    x, y, z = target_pos
    
    y_linha = math.sqrt(x**2 + y**2) - L1_COXA
    L = math.sqrt(z**2 + y_linha**2)
    
    alpha = math.acos(np.clip((L2_FEMUR**2 + L3_TIBIA**2 - L**2) / (2 * L2_FEMUR * L3_TIBIA), -1.0, 1.0))
    beta = math.acos(np.clip((L**2 + L2_FEMUR**2 - L3_TIBIA**2) / (2 * L * L2_FEMUR), -1.0, 1.0))
    
    tibia_angle_rad = -math.pi + alpha
    coxa_angle_rad = -math.atan2(x, y)
    femur_angle_rad = beta + math.atan2(z, y_linha)

    return np.array([coxa_angle_rad, femur_angle_rad, tibia_angle_rad])

def get_rotation_matrix(roll, pitch, yaw):
    roll_rad, pitch_rad, yaw_rad = math.radians(roll), math.radians(pitch), math.radians(yaw)

    Rz = np.array([[math.cos(yaw_rad),-math.sin(yaw_rad), 0], [math.sin(yaw_rad), math.cos(yaw_rad), 0], [0, 0, 1]])
    Ry = np.array([[math.cos(pitch_rad), 0, math.sin(pitch_rad)], [0, 1, 0], [-math.sin(pitch_rad), 0, math.cos(pitch_rad)]])
    Rx = np.array([[1, 0, 0], [0, math.cos(roll_rad), -math.sin(roll_rad)], [0, math.sin(roll_rad), math.cos(roll_rad)]])

    return Rz @ Ry @ Rx


# example usage:
angles = np.radians([-30, -45, -90])  # coxia, femur, tibia in degrees
foot_pos = forward_kinematics(angles)
print("Foot Position:", foot_pos)
target_pos = np.array([-10.0, 15.0, -5.0])  # x, y, z
angles_rad = inverse_kinematics(target_pos)
print("Joint Angles (radians):", angles_rad)
print("Joint Angles (degrees):", np.degrees(angles_rad))

