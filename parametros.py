# =============================================================================
# PARÂMETROS ELETROMECÂNICOS ATUALIZADOS (NÃO MUDAR - IDENTIFICADOS EM BANCADA)
# =============================================================================
CONSTANTE_VELOCIDADE_GERADOR_V_RAD_S = 2.675717 
OFFSET_FEM = 9.5104 
TORQUE_MAXIMO_MOTOR_NM = 79.63 

# Atualizado com o resultado do ensaio elétrico de carga estacionária
RESISTENCIA_INTERNA_OHMS = 25.0 # chute
TAU_ARMADURA = 0.03 # chute
GANHO_ARMADURA = 1/RESISTENCIA_INTERNA_OHMS # chute
CAPACITANCIA_BARRAMENTO_FARADAY = 0.007

# Atualizado com o resultado do ensaio de coast-down (Tau = 3.5707 s)
COEFICIENTE_ATRITO_GERADOR = 0.001885
INERCIA_GERADOR_KG_M2 = 2 * COEFICIENTE_ATRITO_GERADOR

import numpy as np
pontos_v_inversor_hibrido = np.array([90, 110, 130, 150, 170, 190, 210, 230, 250, 270, 290, 310])
pontos_i_inversor_hibrido = np.array([0.001, 1.5, 3.0, 4.5, 6.0, 7.5, 9.0, 10.5, 12.0, 13.5, 15.0, 16.5])/2

"""pontos_v_inversor_hibrido = np.array([
    50,  70,  90,  110, 130, 150, 170, 190, 210, 230, 
    250, 270, 290, 310
])

pontos_i_inversor_hibrido = np.array([
    0.3,  2.2,  3.6,  4.4,  4.4,  4.5,  4.8,  5.7,  6.8,  8.1, 
    9.6,  11.2, 12.8, 14.4
])"""

pontos_r_inversor_hibrido = pontos_v_inversor_hibrido/pontos_i_inversor_hibrido
# =============================================================================

# apenas para isolar o comportamento do motor/gerador de bancada,
COEFICIENTE_ATRITO_TURBINA = 0.0
INERCIA_TURBINA_KG_M2 = 0.0

DENSIDADE_AR_KG_M3 = 1.2754
RAIO_TURBINA_METROS = 2.5 # escolha
RELACAO_TRANSMISSAO_CAIXA_ENGRENAGENS = 4 # escolha
TSR_IDEAL = 7.5 # chute

TAXA_VARIACAO_RPM_MAX = 100 # limite fisico do motor
VELOCIDADE_MAXIMA_PAS_GRAUS_S = 5.0 # escolha
TAXA_VARIACAO_TENSAO_BARRAMENTO_V_S = 10.0 # chute/escolha
TENSAO_MAXIMA_INVERSOR_V = pontos_v_inversor_hibrido[-1] # tabela inversor hibrido
TENSAO_MINIMA_INVERSOR_V = pontos_v_inversor_hibrido[0]  # tabela inversor hibrido
TEMPO_ESPERA_LIGAR = 5.0  # Tempo necessário (em segundos) com tensão alta para ligar

POTENCIA_MECANICA_MAXIMA_W = 3000 # escolha
PASSO_SIMULACAO_SEGUNDOS = 0.004 
PASSO_HARDWARE_SEGUNDOS = 0.01 # deve ser maior que a latencia da comunicacao
TEMPO_TOTAL_EMULACAO_SEGUNDOS = 100 # escolha
TAXA_ATUALIZACAO_GRAFICOS_SEGUNDOS = 1.0 #escolha

CONSTANTE_TEMPO_FILTRO_SENSOR_POTENCIA = 0.1
CONSTANTE_TEMPO_FILTRO_U_INVERSOR = 0.4
CONSTANTE_TEMPO_FILTRO_VELOCIDADE_ALVO = 0.5
CONSTANTE_TEMPO_FILTRO_FORCAELETROMOTRIZ = 0.0

#PERFIL_VENTO = 'CONSTANTE'
#PERFIL_VENTO = 'ESCADA' 
PERFIL_VENTO = 'NATURAL'
#PERFIL_VENTO = 'RAMPA'
from numpy import load
VETOR_VENTO_M_S = load('vetor_vento.npy')
vento_constante = 5

