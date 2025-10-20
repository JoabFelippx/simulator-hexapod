# simulation/input_handler.py

import pygame
import numpy as np
import math
import config

def handle_input(keys): # Retorna um dicionário de comandos baseado nas teclas pressionadas
    commands = { 'walk_direction': None, 'body_rotation': np.array([0.0, 0.0, 0.0]) }

    move_vector = np.array([0.0, 0.0])
    if keys[pygame.K_w]: move_vector[1] += 1.0
    if keys[pygame.K_s]: move_vector[1] -= 1.0
    if keys[pygame.K_a]: move_vector[0] -= 1.0
    if keys[pygame.K_d]: move_vector[0] += 1.0
    

    if np.linalg.norm(move_vector) > 0: # 
        commands['walk_direction'] = math.atan2(move_vector[0], move_vector[1])

    max_angle = config.BODY_ROTATION_MAX_ANGLE_DEG
    if keys[pygame.K_UP]: commands['body_rotation'][1] = max_angle    
    if keys[pygame.K_DOWN]: commands['body_rotation'][1] = -max_angle 
    if keys[pygame.K_LEFT]: commands['body_rotation'][0] = max_angle    
    if keys[pygame.K_RIGHT]: commands['body_rotation'][0] = -max_angle
    if keys[pygame.K_q]: commands['body_rotation'][2] = max_angle
    if keys[pygame.K_e]: commands['body_rotation'][2] = -max_angle
        
    return commands
