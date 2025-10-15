import numpy as np
import math

TOTAL_PONTOS = 100  # Example total points
METADE_PONTOS = TOTAL_PONTOS // 2

class Leg:
    def __init__(self, L1, L2, L3, initial_angles_deg):

        self.L1 = L1  # Comprimento do Coxa (ombro)
        self.L2 = L2  # Comprimento do Femur
        self.L3 = L3  # Comprimento do Tibia

        # Ângulos atuais (ombro, femur, tibia) em radianos
        self.angles = np.radians(initial_angles_deg)
        
        # Posição inicial da ponta da pata (foot tip)
        self.xyz_ini = self.forward_kinematics(self.angles)

        self.P0 = np.array([0, 0])  # Define control points as needed
        self.P1 = np.array([0, 0])
        self.P2 = np.array([0, 0])
        self.P3 = np.array([0, 0])



    def set_angles(self, angles_deg):
        ''' Define the joint angles in degrees.
            angles_deg: list or array of [shoulder, femur, tibia] in degrees.
        '''
        self.angles = np.radians(angles_deg)

    def forward_kinematics(self, angles_rad):

        ''' Calculate the XYZ position of the foot tip given the joint angles in radians. 
            returns a NumPy array [x, y, z].
        '''

        theta1, theta2, theta3 = angles_rad # ombro, femur, tibia

        x = -np.sin(theta1) * (self.L1 + self.L3 * np.cos(theta2 + theta3) + self.L2 * np.cos(theta2))
        y = np.cos(theta1) * (self.L1 + self.L3 * np.cos(theta2 + theta3) + self.L2 * np.cos(theta2))
        z = self.L3 * np.sin(theta2 + theta3) + self.L2 * np.sin(theta2)

        return np.array([x, y, z])

    def inverse_kinematics(self, xyz_coords):
        ''' Calculate the joint angles in radians given the XYZ position of the foot tip.
            xyz_coords: NumPy array [x, y, z].
            returns a NumPy array of angles [shoulder, femur, tibia] in radians.
        '''

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

    # floatxyz trajetoriaLinear(floatxyz xyz_ini, int k, int offset, float angle_rad){
    #     floatxyz xyz;
    #     int kn = (k + offset) % TOTAL_PONTOS;
    #     if (kn < METADE_PONTOS){
    #     float t = float(kn)/(METADE_PONTOS-1);
    #     float u = 1 - t;
    #     xyz.x = xyz_ini.x + cos(angle_rad)*(-xyz_ini.x + u * u * u * this->P0[0] + 3 * u * u * t * this->P1[0] + 3 * u * t * t * this->P2[0] + t * t * t * this->P3[0]);
    #     xyz.y = xyz_ini.y + sin(angle_rad)*(-xyz_ini.x + u * u * u * this->P0[0] + 3 * u * u * t * this->P1[0] + 3 * u * t * t * this->P2[0] + t * t * t * this->P3[0]);
    #     xyz.z = u * u * u * this->P0[1] + 3 * u * u * t * this->P1[1] + 3 * u * t * t * this->P2[1] + t * t * t * this->P3[1];
    #     }
    #     else{
    #     xyz.x = xyz_ini.x + cos(angle_rad)*(-xyz_ini.x + this->P3[0] + (this->P0[0] - this->P3[0])*(float(kn - METADE_PONTOS) / (METADE_PONTOS - 1)));
    #     xyz.y = xyz_ini.y + sin(angle_rad)*(-xyz_ini.x + this->P3[0] + (this->P0[0] - this->P3[0])*(float(kn - METADE_PONTOS) / (METADE_PONTOS - 1)));
    #     xyz.z = xyz_ini.z;
    #     }
    #     return xyz;
    # }

    def trajectory_linear(self, xyz_ini, k, offset, angle_rad):
        ''' Calculate a linear trajectory point for the leg.
            xyz_ini: NumPy array [x, y, z] initial position.
            k: current step index.
            offset: step offset.
            angle_rad: rotation angle in radians.
            returns a NumPy array [x, y, z] for the trajectory point.
        '''

        kn = (k + offset) % TOTAL_PONTOS

        xyz = np.zeros(3)

        if kn < METADE_PONTOS:
            t = float(kn) / (METADE_PONTOS - 1)
            u = 1 - t
            bezier_x = u**3 * self.P0[0] + 3 * u**2 * t * self.P1[0] + 3 * u * t**2 * self.P2[0] + t**3 * self.P3[0]
            bezier_y = u**3 * self.P0[1] + 3 * u**2 * t * self.P1[1] + 3 * u * t**2 * self.P2[1] + t**3 * self.P3[1]
            xyz[0] = xyz_ini[0] + np.cos(angle_rad) * (-xyz_ini[0] + bezier_x)
            xyz[1] = xyz_ini[1] + np.sin(angle_rad) * (-xyz_ini[1] + bezier_x)
            xyz[2] = bezier_y
            return xyz
            
        xyz[0] = xyz_ini[0] + np.cos(angle_rad) * (-xyz_ini[0] + self.P3[0] + (self.P0[0] - self.P3[0]) * (float(kn - METADE_PONTOS) / (METADE_PONTOS - 1)))
        xyz[1] = xyz_ini[1] + np.sin(angle_rad) * (-xyz_ini[1] + self.P3[0] + (self.P0[0] - self.P3[0]) * (float(kn - METADE_PONTOS) / (METADE_PONTOS - 1)))
        xyz[2] = xyz_ini[2]
        return xyz