class Hexapod:
    def __init__(self):
        # Define as dimensões e posições iniciais
        # ...
        
        # Cria as 6 pernas
        self.legs = {
            "EsqF": Leg(...),
            "EsqM": Leg(...),
            # ... outras 4 pernas
        }
        
    def walk(self, k, angle_rad):
        # Lógica do seu método andar()
        # Calcula o xyz para cada perna usando a trajetória
        xyz_esq_f = self.legs["EsqF"].trajetoriaLinear(...)
        
        # Calcula os novos ângulos usando cinemática inversa
        angles_esq_f = self.legs["EsqF"].inverse_kinematics(xyz_esq_f)
        
        # ATUALIZA os ângulos da perna no simulador
        self.legs["EsqF"].set_angles(angles_esq_f)
        
        # ... faz o mesmo para as outras 5 pernas ...

    def body_inverse_kinematics(self, body_angles_deg):
        # ... sua lógica de cinematicaInversaCorpo aqui ...
        pass
