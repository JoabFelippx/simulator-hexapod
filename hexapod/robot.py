# hexapod/robot.py

import numpy as np
import math
from hexapod.leg import Leg
from hexapod import kinematics
import config
from get_angles import reading

def _cubic_bezier(p0, p1, p2, p3, t):
    """
    Calcula um ponto em uma curva de Bézier cúbica.
    t é o tempo/fase, variando de 0.0 a 1.0.
    """
    u = 1 - t
    return (u**3 * p0) + (3 * u**2 * t * p1) + (3 * u * t**2 * p2) + (t**3 * p3)


class Hexapod:
    def __init__(self):
        self.legs = {}
        for name, shoulder_pos in config.SHOULDER_POSITIONS.items():
            # Usa o nome completo da perna (ex: "EsqF") para pegar os ângulos corretos
            initial_angles = config.INITIAL_ANGLES_DEG[name]
            self.legs[name] = Leg(name, shoulder_pos, initial_angles)
        # print(self.legs)
            
        self.body_translation = np.array([0.0, 0.0, 0.0])
        self.body_rotation = np.array([0.0, 0.0, 0.0]) # Roll, Pitch, Yaw
        self.body_rotation_matrix = np.identity(3)

    def walk(self, step_phase, direction_angle_rad):
        for leg in self.legs.values():
            
            phase_offset = (step_phase + config.GAIT_OFFSETS[leg.name]) % 1.0      
            
            home_pos = leg.home_foot_tip_pos_relative.copy() # Posição "em pé" da pata
            
            if 0 <= phase_offset < 0.5: # Fase de apoio (pé no chão)
                # Move a pata para trás ao longo do eixo x local
                t = phase_offset * 2.0
                
                xz_point = _cubic_bezier(leg.p0, leg.p1, leg.p2, leg.p3, t)
                
                final_x_local = xz_point[0]
                final_z_local = xz_point[1]
            else: # Fase de pé no ar
                t = (phase_offset - 0.5) * 2.0
                
                xz_point = leg.p3 * (1 - t) + leg.p0 * t
                
                final_x_local = xz_point[0]
                final_z_local = xz_point[1]
                
            # Agora, aplicamos a direção do movimento (omnidirecional)
            # O deslocamento horizontal é a diferença entre a posição X atual e a de repouso.
            horizontal_displacement = final_x_local - home_pos[0]
            
            # Cria a matriz de rotação 2D
            rot_matrix = np.array([
                [math.cos(direction_angle_rad), -math.sin(direction_angle_rad)],
                [math.sin(direction_angle_rad), math.cos(direction_angle_rad)]
            ])
            
            # O vetor base do deslocamento é para "frente" no frame da perna (eixo X)
            base_vector = np.array([horizontal_displacement, 0])
            # Rotaciona o vetor para a direção desejada
            rotated_delta = rot_matrix @ base_vector

            # Monta a nova posição final da pata
            new_foot_pos = np.array([
                home_pos[0] + rotated_delta[0], # Posição X rotacionada
                home_pos[1] + rotated_delta[1], # Posição Y rotacionada
                final_z_local                   # Posição Z da trajetória (altura)
            ])
            
            leg.set_foot_tip_position(new_foot_pos)

    def rotate_body(self, roll, pitch, yaw):
        
        self.body_rotation = np.array([roll, pitch, yaw])
        self.body_rotation_matrix = kinematics.get_rotation_matrix(roll, pitch, yaw)
  
        for leg in self.legs.values():
        
            foot_tip_target_in_world_frame = leg.shoulder_position + leg.home_foot_tip_pos_relative
            
            rotated_shoulder_pos = self.body_rotation_matrix @ leg.shoulder_position
        
            new_foot_pos_relative_to_rotated_shoulder = foot_tip_target_in_world_frame - rotated_shoulder_pos
        
            inv_rotation_matrix = self.body_rotation_matrix.T
            new_foot_pos_in_leg_frame = inv_rotation_matrix @ new_foot_pos_relative_to_rotated_shoulder
        
            leg.set_foot_tip_position(new_foot_pos_in_leg_frame)


    def stop(self):
        for leg in self.legs.values():
            leg.set_foot_tip_position(leg.home_foot_tip_pos_relative)
            
    def ohyes(self):
        angles = reading()

        if angles is None:
            return None
      
        angles_rad = np.radians(angles)
        for k, leg  in enumerate(self.legs.values()):
            
            

            xyz = kinematics.forward_kinematics(angles_rad[k])

            # print(leg.name, angles[k])

            leg.set_foot_tip_position(xyz)

    def reset_body_orientation(self):
        if np.any(self.body_rotation):
            self.rotate_body(0, 0, 0)
