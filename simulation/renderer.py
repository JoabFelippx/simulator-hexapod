

# simulation/renderer.py

import pygame
import config
import numpy as np

class Renderer:
    def __init__(self, screen, camera):
        self.screen = screen
        self.camera = camera
        try:
            self.font_large = pygame.font.SysFont('Consolas', 24)
            self.font_medium = pygame.font.SysFont('Consolas', 18)
        except:
            self.font_large = pygame.font.Font(None, 30)
            self.font_medium = pygame.font.Font(None, 24)

    def draw_grid(self):
        plane_corners_3d = [
            np.array([-150, -150, 0]), np.array([150, -150, 0]),
            np.array([150, 150, 0]), np.array([-150, 150, 0])
        ]
        plane_corners_2d = [self.camera.project(p) for p in plane_corners_3d]
        pygame.draw.polygon(self.screen, config.COLOR_GRID, plane_corners_2d)

    def draw_axes(self, origin=None, size=30):
        """
        Desenha eixos coordenados no display.
        origin: ponto 3D (numpy array) onde os eixos começam.
        size: comprimento dos eixos em unidades do mundo.
        """
        if origin is None:
            origin = np.array([0.0, 0.0, 0.0])

        # Pontos finais dos eixos
        x_end = origin + np.array([size, 0.0, 0.0])
        y_end = origin + np.array([0.0, size, 0.0])
        z_end = origin + np.array([0.0, 0.0, size])

        # Projeta para 2D
        p_origin = self.camera.project(origin)
        p_x = self.camera.project(x_end)
        p_y = self.camera.project(y_end)
        p_z = self.camera.project(z_end)

        # Cores padrão dos eixos
        COLOR_X = (230, 50, 50)
        COLOR_Y = (50, 230, 50)
        COLOR_Z = (50, 50, 230)

        # Desenha linhas e pontos finais
        pygame.draw.line(self.screen, COLOR_X, p_origin, p_x, 3)
        pygame.draw.line(self.screen, COLOR_Y, p_origin, p_y, 3)
        pygame.draw.line(self.screen, COLOR_Z, p_origin, p_z, 3)

        pygame.draw.circle(self.screen, COLOR_X, p_x, 6)
        pygame.draw.circle(self.screen, COLOR_Y, p_y, 6)
        pygame.draw.circle(self.screen, COLOR_Z, p_z, 6)

        # Rótulos dos eixos
        try:
            label_x = self.font_medium.render("X", True, COLOR_X)
            label_y = self.font_medium.render("Y", True, COLOR_Y)
            label_z = self.font_medium.render("Z", True, COLOR_Z)
            self.screen.blit(label_x, (p_x[0] + 6, p_x[1] - 6))
            self.screen.blit(label_y, (p_y[0] + 6, p_y[1] - 6))
            self.screen.blit(label_z, (p_z[0] + 6, p_z[1] - 6))
        except Exception:
            pass

    def draw_hexapod(self, hexapod):
        """
        LÓGICA DE DESENHO ATUALIZADA - Copiada e adaptada da versão robusta.
        """
        # --- Desenha o corpo ---
        # Pega as posições dos ombros e as ordena para desenhar o polígono corretamente
        ordered_leg_names = ["EsqF", "DirF", "DirM", "DirT", "EsqT", "EsqM"]
        shoulder_points = [hexapod.legs[name].shoulder_position for name in ordered_leg_names]
        points_2d = [self.camera.project(p) for p in shoulder_points]

        # Desenha o preenchimento semi-transparente
        body_surface = pygame.Surface((config.SCREEN_WIDTH, config.SCREEN_HEIGHT), pygame.SRCALPHA)
        pygame.draw.polygon(body_surface, config.COLOR_ROBOT_BODY_FILL, points_2d)
        self.screen.blit(body_surface, (0, 0))
        
        # Desenha o contorno
        pygame.draw.polygon(self.screen, config.COLOR_ROBOT_BODY_OUTLINE, points_2d, 5)

        # --- Desenha o centro do corpo (simulado) ---
        center_point = np.mean(shoulder_points, axis=0)
        cog_2d = self.camera.project(center_point)
        pygame.draw.circle(self.screen, config.COLOR_CENTER_SPHERE, cog_2d, 10)

        # --- Desenha as Pernas ---
        for leg in hexapod.legs.values():
            # Pega a lista completa de 4 pontos da junta que acabamos de implementar!
            joint_positions = leg.get_all_joint_positions()
            leg_points_2d = [self.camera.project(p) for p in joint_positions]
            
            # Desenha os segmentos da perna
            pygame.draw.lines(self.screen, config.COLOR_LEGS, False, leg_points_2d, 6)
            
            # Desenha as juntas
            for point in leg_points_2d:
                pygame.draw.circle(self.screen, config.COLOR_LEGS, point, 8)
                pygame.draw.circle(self.screen, config.COLOR_BACKGROUND, point, 4)

    def draw_hud(self, status_data):
        x_offset, y_offset = 30, 30
        title_surf = self.font_large.render("STATUS DO SISTEMA", True, config.COLOR_HUD_TEXT)
        self.screen.blit(title_surf, (x_offset, y_offset))
        pygame.draw.line(self.screen, config.COLOR_HUD_ACCENT, (x_offset, y_offset + 40), (x_offset + 250, y_offset + 40), 2)
        y_pos = y_offset + 60
        for key, value in status_data.items():
            key_surf = self.font_medium.render(f"{key}:", True, config.COLOR_HUD_TEXT)
            self.screen.blit(key_surf, (x_offset, y_pos))
            value_surf = self.font_medium.render(str(value), True, config.COLOR_HUD_ACCENT)
            self.screen.blit(value_surf, (x_offset + 150, y_pos))
            y_pos += 30
