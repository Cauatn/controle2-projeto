"""
Parâmetros do projeto — altere AQUI e rode de novo `projeto_controlador.py`.

Planta: modelo de velocidade de um motor DC (2ª ordem, ganho DC unitário)

            20
  G(s) = -------------
         (s + 2)(s + 10)

  - polo em -2  -> dinâmica mecânica (lenta)
  - polo em -10 -> dinâmica elétrica (rápida)
"""

# Planta G(s) = NUM / DEN  (coeficientes em potências decrescentes de s)
NUM_PLANTA = [20.0]
DEN_PLANTA = [1.0, 12.0, 20.0]  # (s+2)(s+10) = s^2 + 12 s + 20

# Especificações de desempenho (malha fechada, degrau na referência)
SOBRESSINAL_MAX = 0.10   # 10 %
TS_MAX = 1.0             # tempo de acomodação (critério 2 %) em segundos
# + erro nulo em regime ao degrau -> exige integrador (controlador PI)

# Polos desejados (escolhidos com folga em relação às especificações)
ZETA_PROJ = 0.65         # zeta >= 0.591 (10 %) + folga para o atraso da amostragem
SIGMA_PROJ = 5.0         # ts ~ 4/sigma = 0.8 s < 1 s

# Amostragem (Tustin)
T = 0.01                 # 10 ms -> 100 Hz (>> 20x a banda de malha fechada)

# Saturação do sinal de controle (tensão no motor, em V)
U_MIN = -12.0
U_MAX = 12.0
