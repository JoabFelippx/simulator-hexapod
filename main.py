# main.py

import pygame
from copy import deepcopy
import numpy as np
import math
import config
from settings import *
from hexapod.models import VirtualHexapod
from hexapod.templates import HEXAPOD_POSE
from simulation.renderer import Renderer
from simulation.input_handler import handle_input

from simulation.camera import Camera # 👈 Importa a nova classe
from hexapod.models import Hexagon

# Offsets para a marcha tripé: pernas 0, 2, 4 se movem juntas, e 1, 3, 5 juntas.
GAIT_OFFSETS = [0.0, 0.5, 0.0, 0.5, 0.0, 0.5]
STEP_LENGTH = 40
STEP_HEIGHT = 30

def calculate_walking_pose(step_phase, direction_rad):
    """
    Calcula os ângulos de cada perna para um determinado passo da caminhada.
    Esta versão usa projeção de vetores para uma caminhada omnidirecional correta.
    """
    new_pose = deepcopy(HEXAPOD_POSE)
    
    # Vetor da direção de movimento desejada (no plano XY)
    direction_vector = np.array([math.sin(direction_rad), math.cos(direction_rad)])

    for leg_id in range(6):
        leg_phase = (step_phase + GAIT_OFFSETS[leg_id]) % 1.0
        
        # O eixo neutro de movimento de cada perna
        coxia_axis_rad = math.radians(Hexagon.COXIA_AXES[leg_id])
        leg_axis_vector = np.array([math.sin(coxia_axis_rad), math.cos(coxia_axis_rad)])
        
        # Projeção: calcula o quanto o movimento da perna está alinhado com a direção desejada
        projection = np.dot(direction_vector, leg_axis_vector)

        # Magnitude do passo para frente e para trás
        step_magnitude = 0

        # Perna no ar (swing)
        if 0 <= leg_phase < 0.5:
            phase_ratio = leg_phase / 0.5
            femur_angle = 45 + STEP_HEIGHT * math.sin(phase_ratio * math.pi)
            tibia_angle = -90 + STEP_HEIGHT * math.sin(phase_ratio * math.pi)
            step_magnitude = (1 - phase_ratio * 2) * STEP_LENGTH / 2
        
        # Perna no chão (stance)
        else:
            phase_ratio = (leg_phase - 0.5) / 0.5
            femur_angle = 45
            tibia_angle = -90
            step_magnitude = (phase_ratio * 2 - 1) * STEP_LENGTH / 2

        # O ângulo da coxia é a magnitude do passo modulada pela projeção na direção
        coxia_angle = step_magnitude * projection

        new_pose[leg_id]["coxia"] = coxia_angle
        new_pose[leg_id]["femur"] = femur_angle
        new_pose[leg_id]["tibia"] = tibia_angle

    return new_pose

def main():
    pygame.init()
    pygame.font.init()
    screen = pygame.display.set_mode((config.SCREEN_WIDTH, config.SCREEN_HEIGHT))
    pygame.display.set_caption(config.WINDOW_TITLE)
    clock = pygame.time.Clock()

    # --- Cria as instâncias principais ---
    hexapod = VirtualHexapod(config.BASE_DIMENSIONS)
    hexapod.update(HEXAPOD_POSE)
    
    camera = Camera(config.SCREEN_WIDTH, config.SCREEN_HEIGHT) # 👈 Cria a câmera
    renderer = Renderer(screen, camera) # 👈 Passa a câmera para o renderer
    
    neutral_pose = deepcopy(HEXAPOD_POSE)
    step_phase = 0.0
    total_rx, total_ry, total_rz, total_tx, total_ty, total_tz = 0,0,0,0,0,0

    running = True
    while running:
        # --- Gerenciamento de Eventos ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            # 👈 Passa os eventos do mouse para a câmera
            camera.handle_event(event)
            
        keys_pressed = pygame.key.get_pressed()
        commands = handle_input(keys_pressed)
        
        current_mode = "Parado"

        # --- Lógica de atualização ---
        if commands['reset']:
            total_rx, total_ry, total_rz, total_tx, total_ty, total_tz = 0,0,0,0,0,0
            hexapod = VirtualHexapod(config.BASE_DIMENSIONS)
            hexapod.update(neutral_pose)
            step_phase = 0.0
        
        # MODO 1: CAMINHADA (prioridade)
        elif commands['walk_direction'] is not None:
            current_mode = "Andando"
            step_phase = (step_phase + 0.02) % 1.0 # Avança o ciclo da caminhada
            walking_pose = calculate_walking_pose(step_phase, commands['walk_direction'])
            hexapod.update(walking_pose)
        
        # MODO 2: CONTROLE DE CORPO
        elif any([commands['rx'], commands['ry'], commands['rz'], commands['tz']]):
            current_mode = "Controle de Corpo"
            total_rx += commands['rx']; total_ry += commands['ry']; total_rz += commands['rz']
            total_tx += commands['tx']; total_ty += commands['ty']; total_tz += commands['tz']

            hexapod = VirtualHexapod(config.BASE_DIMENSIONS)
            hexapod.update(neutral_pose)
            hexapod.detach_body_rotate_and_translate(total_rx, total_ry, total_rz, total_tx, total_ty, total_tz)
        
        else: # Nenhum comando, fica parado
            hexapod.update(neutral_pose)
            step_phase = 0.0

        # --- Renderização ---
        screen.fill(config.COLOR_BACKGROUND)
        renderer.draw_grid()
        renderer.draw_hexapod(hexapod)
        
        status_data = {
            "Modo": current_mode,
            "FPS": f"{clock.get_fps():.1f}",
            "Rot (X,Y,Z)": f"{int(total_rx)}, {int(total_ry)}, {int(total_rz)}",
            "Trans (X,Y,Z)": f"{int(total_tx)}, {int(total_ty)}, {int(total_tz)}",
            "Fase Passo": f"{step_phase:.2f}" if current_mode == "Andando" else "N/A",
        }
        renderer.draw_hud(status_data)

        pygame.display.flip()
        clock.tick(config.FPS)

    pygame.quit()

if __name__ == '__main__':
    main()
