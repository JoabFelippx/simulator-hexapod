# simulation/camera.py

import pygame
import numpy as np
import math

class Camera:
   
    def __init__(self, width, height, initial_distance=30): # Zoom inicial ajustado
        self.width = width
        self.height = height
        self.distance = initial_distance
        self.yaw = -math.pi / 4
        self.pitch = math.pi / 6
        self.center = np.array([0.0, 0.0, -5.0]) # Centro ajustado
        self.view_matrix = np.identity(4)
        self.projection_matrix = self._create_projection_matrix()
        self.update_view_matrix()
    
    def _create_projection_matrix(self, fov=45, aspect_ratio=None, near=0.1, far=1000.0):
        if aspect_ratio is None: aspect_ratio = self.width / self.height
        f = 1.0 / math.tan(math.radians(fov) / 2)
        return np.array([[f/aspect_ratio,0,0,0],[0,f,0,0],[0,0,(far+near)/(near-far),(2*far*near)/(near-far)],[0,0,-1,0]])

    def update_view_matrix(self):
        eye_x = self.center[0] + self.distance * math.cos(self.pitch) * math.sin(self.yaw)
        eye_y = self.center[1] + self.distance * math.cos(self.pitch) * math.cos(self.yaw)
        eye_z = self.center[2] + self.distance * math.sin(self.pitch)
        eye = np.array([eye_x, eye_y, eye_z])
        forward = self.center - eye; forward /= np.linalg.norm(forward)
        world_up = np.array([0.0, 0.0, 1.0]); right = np.cross(forward, world_up); right /= np.linalg.norm(right)
        up = np.cross(right, forward)
        self.view_matrix = np.array([[right[0],up[0],-forward[0],0],[right[1],up[1],-forward[1],0],[right[2],up[2],-forward[2],0],[-np.dot(right,eye),-np.dot(up,eye),np.dot(forward,eye),1]]).T

    def handle_event(self, event):
        if event.type == pygame.MOUSEWHEEL: self.distance = max(10, self.distance - event.y * 2)
        if event.type == pygame.MOUSEMOTION:
            if event.buttons[0]: self.yaw -= event.rel[0]*0.01; self.pitch = np.clip(self.pitch - event.rel[1]*0.01, -math.pi/2.1, math.pi/2.1)
            if event.buttons[2]: right = self.view_matrix.T[0,:3]; up = self.view_matrix.T[1,:3]; self.center -= right * (event.rel[0]*0.1); self.center += up * (event.rel[1]*0.1)
        self.update_view_matrix()


    def project(self, point_3d):
        
        p_homo = np.array([point_3d[0], point_3d[1], point_3d[2], 1.0])
        p_clip = self.projection_matrix @ self.view_matrix @ p_homo
        if p_clip[3] != 0: p_ndc = p_clip / p_clip[3]
        else: p_ndc = p_clip
        screen_x = (p_ndc[0] + 1) * 0.5 * self.width
        screen_y = (1 - (p_ndc[1] + 1) * 0.5) * self.height
        return int(screen_x), int(screen_y)
