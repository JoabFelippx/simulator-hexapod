# main.py

import pygame
import numpy as np
import math
import config
from hexapod.robot import Hexapod
from simulation.renderer import Renderer
from simulation.input_handler import handle_input
from simulation.camera import Camera

def main():
    pygame.init(); pygame.font.init()
    screen = pygame.display.set_mode((config.SCREEN_WIDTH, config.SCREEN_HEIGHT))
    pygame.display.set_caption(config.WINDOW_TITLE); clock = pygame.time.Clock()

    scarlet = Hexapod()
    camera = Camera(config.SCREEN_WIDTH, config.SCREEN_HEIGHT)
    renderer = Renderer(screen, camera)
    
    step_phase = 0.0

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            camera.handle_event(event)

        keys_pressed = pygame.key.get_pressed()
        commands = handle_input(keys_pressed)

        is_moving = False
        current_mode = "Parado"
        if commands['walk_direction'] is not None:
            scarlet.reset_body_orientation()
            scarlet.walk(step_phase, commands['walk_direction'])
            step_phase = (step_phase + (config.MOVEMENT_SPEED / config.FPS)) % 1.0
            is_moving = True
            current_mode = "Andando"
        
        if np.any(commands['body_rotation']):
            roll, pitch, yaw = commands['body_rotation']
            scarlet.rotate_body(roll, pitch, yaw)
            is_moving = True
            current_mode = "Rotacionando Corpo"
        
        if not is_moving:
            scarlet.stop()
            scarlet.reset_body_orientation()
            step_phase = 0.0
            
        screen.fill(config.COLOR_BACKGROUND)
        renderer.draw_grid()
        renderer.draw_axes()
        renderer.draw_support_plane(scarlet)
        renderer.draw_hexapod(scarlet)
        
        status_data = {"Modo": current_mode, "FPS": f"{clock.get_fps():.1f}"}
        renderer.draw_hud(status_data)

        pygame.display.flip()
        clock.tick(config.FPS)

    pygame.quit()

if __name__ == '__main__':
    main()
