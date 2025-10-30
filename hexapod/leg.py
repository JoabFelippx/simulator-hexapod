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
        
        

        
        """

            Pontos de controle da curva de Bezier para o movimento da pata
            p0: posição inicial (início do passo)
            p1: ponto de controle 1 (influencia a curvatura)
            p2: ponto de controle 2 (influencia a curvatura)
            p3: posição final (fim do passo)
 
        """
        self.p0 = np.array([0.0, 0.0]) # [x, z]
        self.p1 = np.array([0.0, 0.0]) # [x, z]
        self.p2 = np.array([0.0, 0.0]) # [x, z]
        self.p3 = np.array([0.0, 0.0]) # [x, z]
        self.update_bezier_points(config.WALK_STEP_LENGTH, config.WALK_STEP_HEIGHT)
        
    def update_bezier_points(self, step_length, step_height):
        
        
        home_x, home_z = self.home_foot_tip_pos_relative[0], self.home_foot_tip_pos_relative[2] # Posição "em pé" da pata (no eixo x e z)
        
        
        half_length = step_length / 2.0
        
        # P0: Ponto inicial do passo (totalmente para trás)
        self.p0 = np.array([home_x - half_length, home_z])
        # P3: Ponto final do passo (totalmente para frente)
        self.p3 = np.array([home_x + half_length, home_z]) 
        
        
        # P1 e P2: Pontos de controle para definir a curvatura do passo
        # Usando um deslocamento no eixo x e a altura do passo no eixo z
        control_x_offset = half_length * 0.5
        self.p1 = np.array([home_x - control_x_offset, home_z + step_height])
        self.p2 = np.array([home_x + control_x_offset, home_z + step_height])

    def set_foot_tip_position(self, target_pos_relative_to_shoulder): # Atualiza a posição da ponta do pé e os ângulos das juntas
        self.current_foot_tip_pos_relative = target_pos_relative_to_shoulder
        self.current_angles_rad = kinematics.inverse_kinematics(self.current_foot_tip_pos_relative)

    def set_current_angles(self, angles):
        self.current_angles_rad = np.radians(angles)
        self.set_foot_tip_position(self.current_angles_rad)

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
