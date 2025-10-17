# config.py

import numpy as np

# --- Configurações da Simulação ---
SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 800
WINDOW_TITLE = "Simulador de Robô Hexapod"
FPS = 60

# --- Cores no estilo "Sci-Fi" ---
COLOR_BACKGROUND = (34, 47, 62)
COLOR_GRID = (10, 61, 98, 150)
COLOR_ROBOT_BODY_OUTLINE = (252, 66, 123)
COLOR_ROBOT_BODY_FILL = (255, 99, 72, 77)
COLOR_LEGS = (238, 90, 36)
COLOR_CENTER_SPHERE = (50, 255, 126)
COLOR_HEAD = (252, 66, 123)
COLOR_HUD_TEXT = (46, 204, 113)
COLOR_HUD_ACCENT = (252, 66, 123)

# --- Dimensões do Robô (em cm) ---
L1_COXA = 5.28
L2_FEMUR = 10.4
L3_TIBIA = 14.62

# --- Posições dos Ombros no Corpo ---
# As posições são relativas ao centro do corpo do robô
SHOULDER_POSITIONS = {
    "EsqF": np.array([8.30, 5.55, 0.0]),
    "EsqM": np.array([0.0, 7.50, 0.0]),
    "EsqT": np.array([-8.29, 5.50, 0.0]),
    "DirF": np.array([8.30, -5.55, 0.0]),
    "DirM": np.array([0.0, -7.50, 0.0]),
    "DirT": np.array([-8.29, -5.50, 0.0])
}


    # int3 anglesF = {45,26,-100};
    # int3 anglesM = {0,26,-100};
    # int3 anglesT = {-45,26,-100};
    # int3 anglesStartF = {anglesF.ombro,90,-145};
    # int3 anglesStartM = {anglesM.ombro,90,-145};
    # int3 anglesStartT = {anglesT.ombro,90,-145};
# --- Ângulos Iniciais (posição "em pé") ---
INITIAL_ANGLES_DEG = {
    # "F": np.array([0,0,0]), # Perna da frente (coxia 0)
    # "M": np.array([0,0,0]), # Perna do meio
    # "T": np.array([0,0,0])  # Perna de trás
    # [ombro, femur, tibia]
    # "F": np.array([45,26,-100]), # Perna da frente (coxia 45)
    # "M": np.array([0,26,-100]),  # Perna do meio (coxia 0)
    # "T": np.array([-45,26,-100]) # Perna de trás (coxia -45)
    
    # Pernas da DIREITA
    "DirF": np.array([225, 26, -100]),
    "DirM": np.array([180, 26, -100]),
    "DirT": np.array([135, 26, -100]),
    
    # Pernas da ESQUERDA (com o ângulo do ombro invertido para simetria)
    "EsqF": np.array([-45, 26, -100]),
    "EsqM": np.array([0, 26, -100]),
    "EsqT": np.array([45, 26, -100]),
    
}

# --- Parâmetros de Movimento ---
WALK_STEP_LENGTH = 8.0
WALK_STEP_HEIGHT = 6.0
MOVEMENT_SPEED = 1
BODY_ROTATION_MAX_ANGLE_DEG = 15

# --- Offsets da Marcha (Tripod Gait) ---
GAIT_OFFSETS = {
    "EsqF": 0.0, "DirM": 0.0, "EsqT": 0.0,
    "DirF": 0.5, "EsqM": 0.5, "DirT": 0.5
}
