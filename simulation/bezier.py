import numpy as np
import matplotlib.pyplot as plt
from math import comb

# --- A função de cálculo de Bézier (sua versão otimizada) ---
def calcular_ponto_bezier(pontos, t):
    """
    Calcula um ponto na curva de Bézier usando a fórmula cúbica otimizada.
    """
    u = 1 - t
    L = len(pontos)-1
    mol = 0
    for k, p in enumerate(pontos):
        mol += (u**(L-k)) * (t**k) * comb(L,k) * p
    return mol

def calcular_ponto_bezier1(pontos, t):
    """
    Calcula um ponto na curva de Bézier usando a fórmula cúbica otimizada.
    """
    u = 1 - t
    p0, p1, p2, p3 = pontos
    return (u**3 * p0) + (3 * u**2 * t * p1) + (3 * u * t**2 * p2) + (t**3 * p3)

# --- NOVA FUNÇÃO para reparametrizar a curva ---
def gerar_pontos_uniformes(pontos_controle, num_pontos_finais =25):
    """
    Gera pontos com espaçamento uniforme ao longo da curva de Bézier
    usando a técnica de Reparametrização por Comprimento de Arco.
    """
    # 1. Criar uma Tabela de Consulta (LUT) de alta resolução da curva

    # 2. Criar um espaço linear de 0 a PI. Este será o nosso "tempo" ou "progresso" linear.
    espaco_linear_pi = np.linspace(0, np.pi, num_pontos_finais)

    # 3. Aplicar a fórmula de suavização baseada em cosseno.
    # A função (1 - cos(x)) / 2 mapeia o intervalo [0, PI] para [0, 1] de forma não-linear.
    t_lut = (1 - np.cos(espaco_linear_pi)) / 2



    # num_pontos_lut = 1000
    # t_lut = np.linspace(0, 1, num_pontos_lut)
    curva_lut = np.array([calcular_ponto_bezier(pontos_controle, t) for t in t_lut])

        
    return curva_lut

# --- Preparação para o Gráfico ---

pontos_controle2 = np.array([
    [0, 0],  # P0
    [0, 8],  # P1
    [8, 8],  # P2
    [8, 0]   # P3
])
pontos_controle = np.array([
    [0, 0],  # P0
    [2, 6],  # P1
    [4, 8],  # P2
    [6, 6],  # P2
    [8, 0]   # P3
])

num_pontos_plot = 25

# Curva original com espaçamento NÃO-UNIFORME
t_nao_uniforme = np.linspace(0, 1, num_pontos_plot)
pontos_nao_uniformes = np.array([calcular_ponto_bezier(pontos_controle, t) for t in t_nao_uniforme])

# NOVA curva com espaçamento UNIFORME
pontos_uniformes = gerar_pontos_uniformes(pontos_controle, num_pontos_plot)

# --- Plotagem do Gráfico ---

plt.figure(figsize=(12, 9))

# Plotar a curva de fundo (linha fina)
t_fundo = np.linspace(0, 1, 200)
curva_fundo = np.array([calcular_ponto_bezier(pontos_controle, t) for t in t_fundo])
plt.plot(curva_fundo[:, 0], curva_fundo[:, 1], 'g-', lw=1, alpha=0.5, label='Trajetória da Curva')

# Plotar os pontos originais (NÃO-UNIFORMES)
plt.plot(pontos_nao_uniformes[:, 0], pontos_nao_uniformes[:, 1], 'rx', markersize=12, label='Espaçamento Não-Uniforme (Original)')

# Plotar os novos pontos (UNIFORMES)
plt.plot(pontos_uniformes[:, 0], pontos_uniformes[:, 1], 'bo', markersize=8, mfc='lightblue', label='Espaçamento Uniforme (Corrigido)')

# Plotar os pontos de controle
pontos_controle_x = pontos_controle[:, 0]
pontos_controle_y = pontos_controle[:, 1]
plt.plot(pontos_controle_x, pontos_controle_y, 'ko--', label='Pontos de Controle', mfc='orange')

# Configurações do gráfico
plt.title('Comparação: Espaçamento Uniforme vs. Não-Uniforme', fontsize=16)
plt.xlabel('Eixo X')
plt.ylabel('Eixo Y')
plt.legend(fontsize=12)
plt.grid(True)
plt.axis('equal')
plt.savefig('comparacao_espacamento_corrigido.png')

print("Gráfico 'comparacao_espacamento_corrigido.png' gerado com sucesso!")
