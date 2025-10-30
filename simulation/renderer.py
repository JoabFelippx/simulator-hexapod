# simulation/renderer.py

import pygame
import config
import numpy as np
import math
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
        
        """
        Desenha uma grade no plano XY para referência visual em z = -9.49.
        """
     
        grid_surface = pygame.Surface((config.SCREEN_WIDTH, config.SCREEN_HEIGHT), pygame.SRCALPHA)
        grid_color = (200, 200, 200, 40) 
        spacing = 10
        grid_size = 50

        for x in range(-grid_size, grid_size + 1, spacing):
            start_point = self.camera.project(np.array([x, -grid_size, -9.49458607]))
            end_point = self.camera.project(np.array([x, grid_size, -9.49458607]))
            pygame.draw.line(grid_surface, grid_color, start_point, end_point, 1)

        for y in range(-grid_size, grid_size + 1, spacing):
            start_point = self.camera.project(np.array([-grid_size, y, -9.49458607]))
            end_point = self.camera.project(np.array([grid_size, y, -9.49458607]))
            pygame.draw.line(grid_surface, grid_color, start_point, end_point, 1)

        self.screen.blit(grid_surface, (0, 0))
           
            
    def draw_support_plane(self, hexapod):
        """
        Desenha um polígono conectando as pontas das patas que estão no chão.
        """
        foot_tip_points_3d = []
        ground_z_level = -9.65
    
        ordered_leg_names = ["EsqF", "DirF", "DirM", "DirT", "EsqT", "EsqM"]
        
        rotation_matrix = hexapod.body_rotation_matrix

        for name in ordered_leg_names:
            leg = hexapod.legs[name]
            foot_tip_local = leg.get_all_joint_positions()[-1]
            
            foot_tip_world = rotation_matrix @ foot_tip_local
        
            # print(foot_tip_world)

            if math.isclose(foot_tip_world[2], ground_z_level, abs_tol=0.01): # Verifica se a pata está no chão usando isclose
                foot_tip_points_3d.append(foot_tip_world)
               
        if len(foot_tip_points_3d) >= 3: # Precisa de pelo menos 3 pontos para formar um polígono

            points_2d = [self.camera.project(p) for p in foot_tip_points_3d]
            

            support_surface = pygame.Surface((config.SCREEN_WIDTH, config.SCREEN_HEIGHT), pygame.SRCALPHA)
            
            pygame.draw.polygon(support_surface, config.COLOR_SUPPORT_PLANE, points_2d)
            pygame.draw.polygon(support_surface, config.COLOR_SUPPORT_PLANE_OUTLINE, points_2d, 2)
            
            self.screen.blit(support_surface, (0, 0))

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

    # def draw_hexapod(self, hexapod):
    #     """
    #     LÓGICA DE DESENHO ATUALIZADA - Copiada e adaptada da versão robusta.
    #     """
   
    #     ordered_leg_names = ["EsqF", "DirF", "DirM", "DirT", "EsqT", "EsqM"]
        
    #     rotation_matrix = hexapod.body_rotation_matrix
        
    #     # shoulder_points = [hexapod.legs[name].shoulder_position for name in ordered_leg_names]
    #     # points_2d = [self.camera.project(p) for p in shoulder_points]


    #     # Calcula os pontos dos ombros rotacionados
    #     original_shoulder_points = [hexapod.legs[name].shoulder_position for name in ordered_leg_names]
    #     rotated_shoulder_points = [rotation_matrix @ p for p in original_shoulder_points]


    #     # Pontos 2D projetados
    #     points_2d = [self.camera.project(p) for p in rotated_shoulder_points]


    #     # Desenha o preenchimento semi-transparente
    #     body_surface = pygame.Surface((config.SCREEN_WIDTH, config.SCREEN_HEIGHT), pygame.SRCALPHA)
    #     pygame.draw.polygon(body_surface, config.COLOR_ROBOT_BODY_FILL, points_2d)
    #     self.screen.blit(body_surface, (0, 0))
        
    #     pygame.draw.polygon(self.screen, config.COLOR_ROBOT_BODY_OUTLINE, points_2d, 5)

    #     # Desenha o centro do corpo
    #     center_point = np.mean(rotated_shoulder_points, axis=0)
    #     cog_2d = self.camera.project(center_point)
    #     pygame.draw.circle(self.screen, config.COLOR_CENTER_SPHERE, cog_2d, 10)

    #     # --- Desenha as Pernas ---
    #     for leg in hexapod.legs.values():
    #         # Pega a lista completa de 4 pontos da junta que acabamos de implementar!
    #         joint_positions = leg.get_all_joint_positions()
            
    #         # Aplica a rotação do corpo a cada ponto da junta
    #         rotated_joint_positions = [rotation_matrix @ pos for pos in joint_positions]
            
    #         # Projeta os pontos 3D para 2D
    #         leg_points_2d = [self.camera.project(pos) for pos in rotated_joint_positions]        
    #         # Desenha os segmentos da perna
    #         pygame.draw.lines(self.screen, config.COLOR_LEGS, False, leg_points_2d, 6)
            
    #         # Desenha as juntas
    #         for point in leg_points_2d:
    #             pygame.draw.circle(self.screen, config.COLOR_LEGS, point, 8)
    #             pygame.draw.circle(self.screen, config.COLOR_BACKGROUND, point, 4)

    def draw_hexapod(self, hexapod):
        """
        LÓGICA DE DESENHO ATUALIZADA
        - Desenha o corpo rotacionado.
        - Desenha as pernas segmento por segmento.
        - Desenha polígonos para os SHIELDS da Fêmur e da Tíbia.
        """
        # --- Desenha o corpo (lógica existente) ---
        ordered_leg_names = ["EsqF", "DirF", "DirM", "DirT", "EsqT", "EsqM"]
        rotation_matrix = hexapod.body_rotation_matrix
        original_shoulder_points = [hexapod.legs[name].shoulder_position for name in ordered_leg_names]
        rotated_shoulder_points = [rotation_matrix @ p for p in original_shoulder_points]
        points_2d = [self.camera.project(p) for p in rotated_shoulder_points]

        body_surface = pygame.Surface((config.SCREEN_WIDTH, config.SCREEN_HEIGHT), pygame.SRCALPHA)
        pygame.draw.polygon(body_surface, config.COLOR_ROBOT_BODY_FILL, points_2d)
        self.screen.blit(body_surface, (0, 0))
        pygame.draw.polygon(self.screen, config.COLOR_ROBOT_BODY_OUTLINE, points_2d, 10)

        center_point = np.mean(rotated_shoulder_points, axis=0)
        cog_2d = self.camera.project(center_point)
        pygame.draw.circle(self.screen, config.COLOR_CENTER_SPHERE, cog_2d, 10)

        # --- Lógica de Desenho das Pernas e Shields ---
        for leg in hexapod.legs.values():
            # Pega as 4 posições das juntas da perna no frame local do robô
            joint_positions_local = leg.get_all_joint_positions()
            # Aplica a rotação do corpo para obter as posições no frame do mundo
            joint_positions_world = [rotation_matrix @ p for p in joint_positions_local]

            # Pega os 4 pontos principais no espaço 3D
            p0_shoulder, p1_hip, p2_knee, p3_foot = joint_positions_world

            # --- Lógica para desenhar os Shields ---
            world_up_vector = np.array([0.0, 0.0, 1.0]) # Usamos o eixo Z como referência "para cima"

            # ## Segmento da Fêmur (p1 ao p2) - com shield ##
            femur_vec = p2_knee - p1_hip
            # Produto vetorial para encontrar a direção da 'largura' do shield
            width_vec_femur = np.cross(femur_vec, world_up_vector)
            # if np.linalg.norm(width_vec_femur) > 1e-6: # Evita divisão por zero
            #     width_vec_femur = width_vec_femur / np.linalg.norm(width_vec_femur) * (config.FEMUR_SHIELD_WIDTH / 2)
                
            #     # Calcula os 4 cantos do polígono no espaço 3D
            #     femur_c1 = p1_hip + width_vec_femur
            #     femur_c2 = p1_hip - width_vec_femur
            #     femur_c3 = p2_knee - width_vec_femur
            #     femur_c4 = p2_knee + width_vec_femur
                
            #     # Projeta os cantos para 2D e desenha o polígono
            #     femur_points_2d = [self.camera.project(p) for p in [femur_c1, femur_c4, femur_c3, femur_c2]]
            #     pygame.draw.polygon(self.screen, config.SHIELD_COLOR, femur_points_2d)

            # ## Segmento da Tíbia (p2 ao p3) - com shield ##
            tibia_vec = p3_foot - p2_knee
            width_vec_tibia = np.cross(tibia_vec, world_up_vector)
            if np.linalg.norm(width_vec_tibia) > 1e-6:
                width_vec_tibia = width_vec_tibia / np.linalg.norm(width_vec_tibia) * (config.TIBIA_SHIELD_WIDTH / 2)

                tibia_c1 = p2_knee + width_vec_tibia
                tibia_c2 = p2_knee - width_vec_tibia
                tibia_c3 = p3_foot - width_vec_tibia
                tibia_c4 = p3_foot + width_vec_tibia

                tibia_points_2d = [self.camera.project(p) for p in [tibia_c1, tibia_c4, tibia_c3, tibia_c2]]
                pygame.draw.polygon(self.screen, config.SHIELD_COLOR, tibia_points_2d)


            # --- Desenha os segmentos estruturais e juntas por cima dos shields ---
            leg_points_2d = [self.camera.project(p) for p in joint_positions_world]

            # Segmento da Coxa (sem shield, desenhado como uma linha grossa)
            pygame.draw.line(self.screen, config.COLOR_LEGS, leg_points_2d[0], leg_points_2d[1], 15)
            # Linha fina central para Fêmur e Tíbia (estrutura interna)
            pygame.draw.line(self.screen, config.COLOR_LEGS, leg_points_2d[1], leg_points_2d[2], 15)
            pygame.draw.line(self.screen, config.COLOR_LEGS, leg_points_2d[2], leg_points_2d[3], 15)

            # Desenha as juntas por último para ficarem na frente
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
