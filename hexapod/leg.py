# hexapod/leg.py

import numpy as np
import math
from hexapod import kinematics
import config

class Leg:
    def __init__(self, name, shoulder_position, initial_angles_deg):
        self.name = name
        self.shoulder_position = shoulder_position
        self.current_angles_rad = np.radians(initial_angles_deg)
        # A cinemática inversa da versão antiga calcula a posição da ponta do pé em relação ao ombro
        self.home_foot_tip_pos_relative = kinematics.forward_kinematics(self.current_angles_rad)
        self.current_foot_tip_pos_relative = self.home_foot_tip_pos_relative.copy()

    def set_foot_tip_position(self, target_pos_relative_to_shoulder): # Atualiza a posição da ponta do pé e os ângulos das juntas
        self.current_foot_tip_pos_relative = target_pos_relative_to_shoulder
        self.current_angles_rad = kinematics.inverse_kinematics(self.current_foot_tip_pos_relative)

    def get_all_joint_positions(self):
        """
        Calcula as coordenadas globais de todas as juntas da perna.
        A matemática foi ajustada para ser consistente com forward_kinematics.
        """
        coxia_rad, femur_rad, tibia_rad = self.current_angles_rad

        # Ponto 0: Ombro (referência global)
        p0 = self.shoulder_position

        # Vetor do ombro até a junta coxa-femur, no sistema de coordenadas da perna
        # Este cálculo agora espelha a lógica de forward_kinematics
        v1 = np.array([
            -config.L1_COXA * math.sin(coxia_rad),
            config.L1_COXA * math.cos(coxia_rad),
            0
        ])
        p1 = p0 + v1

        # Vetor da junta coxa-femur até a junta femur-tibia
        # Precisa considerar a rotação da coxia e do femur
        L_horizontal = config.L2_FEMUR * math.cos(femur_rad)
        v2 = np.array([
            -L_horizontal * math.sin(coxia_rad),
            L_horizontal * math.cos(coxia_rad),
            config.L2_FEMUR * math.sin(femur_rad)
        ])
        p2 = p1 + v2
        
        # Ponto 3: Ponta do Pé (calculado pela cinemática)
        p3 = self.shoulder_position + self.current_foot_tip_pos_relative

        return [p0, p1, p2, p3]
