# hexapod/robot.py

import numpy as np
import math
from hexapod.leg import Leg
from hexapod import kinematics
import config

class Hexapod:
    def __init__(self):
        self.legs = {}
        for name, shoulder_pos in config.SHOULDER_POSITIONS.items():
            # Usa o nome completo da perna (ex: "EsqF") para pegar os ângulos corretos
            initial_angles = config.INITIAL_ANGLES_DEG[name]
            self.legs[name] = Leg(name, shoulder_pos, initial_angles)
            
            
        self.body_translation = np.array([0.0, 0.0, 0.0])
        self.body_rotation = np.array([0.0, 0.0, 0.0]) # Roll, Pitch, Yaw

    def walk(self, step_phase, direction_angle_rad):
        for leg in self.legs.values():
            phase = (step_phase + config.GAIT_OFFSETS[leg.name]) % 1.0
            new_foot_pos = leg.home_foot_tip_pos_relative.copy()
            
            if 0 <= phase < 0.5:
                phase_normalized = phase / 0.5
                new_foot_pos[2] -= config.WALK_STEP_HEIGHT * math.sin(phase_normalized * math.pi)
                delta_x = (1 - phase_normalized * 2) * config.WALK_STEP_LENGTH / 2
            else:
                phase_normalized = (phase - 0.5) / 0.5
                delta_x = (phase_normalized * 2 - 1) * config.WALK_STEP_LENGTH / 2

            rot_z = np.array([
                [math.cos(direction_angle_rad), -math.sin(direction_angle_rad)],
                [math.sin(direction_angle_rad), math.cos(direction_angle_rad)]
            ])
            rotated_delta = rot_z @ np.array([delta_x, 0])
            
            new_foot_pos[0] += rotated_delta[0]
            new_foot_pos[1] += rotated_delta[1]
            
            leg.set_foot_tip_position(new_foot_pos)

    def rotate_body(self, roll, pitch, yaw):
        rotation_matrix = kinematics.get_rotation_matrix(roll, pitch, yaw)
        for leg in self.legs.values():
            foot_pos_body_frame = leg.shoulder_position + leg.home_foot_tip_pos
            rotated_foot_pos = rotation_matrix @ foot_pos_body_frame
            new_foot_pos_relative_to_shoulder = rotated_foot_pos - leg.shoulder_position
            leg.set_foot_tip_position(new_foot_pos_relative_to_shoulder)

    def stop(self):
        for leg in self.legs.values():
            leg.set_foot_tip_position(leg.home_foot_tip_pos_relative)
