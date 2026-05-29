import time
import numpy as np
import matplotlib.pyplot as plt

from parametros import *
from modelo_aerogerador import TurbinaVirtual
class RegistroDeSimulacao:
    def __init__(self, duracao_s, passo_simulacao):
        self.n_sim = int(duracao_s / passo_simulacao) + 1
        self.idx_sim = 0

        # Tempos e Entradas
        self.tempos_simulacao_digital = np.zeros(self.n_sim)
        self.ventos_simulacao_entrada = np.zeros(self.n_sim)
        self.angulos_pas_radianos = np.zeros(self.n_sim)
        
        # Cinemática
        self.velocidades_angulares_gerador_digital = np.zeros(self.n_sim)
        self.velocidades_angulares_gerador_ideais = np.zeros(self.n_sim)
        
        # Elétrica
        self.tensoes_barramento_volts = np.zeros(self.n_sim)
        self.forca_eletromotriz_filtrada = np.zeros(self.n_sim)
        self.correntes_armadura = np.zeros(self.n_sim)
        
        # Torques
        self.torques_aerodinamicos_digitais = np.zeros(self.n_sim)
        self.torques_eletromagneticos_digitais = np.zeros(self.n_sim)
        self.torques_friccao_digitais = np.zeros(self.n_sim)
        self.torques_atrito_pas = np.zeros(self.n_sim)
        self.torques_aceleracao = np.zeros(self.n_sim)
        
        # Potências e Perdas
        self.potencias_absorvida_vento = np.zeros(self.n_sim)
        self.potencias_eixo_alta_velocidade_digitais = np.zeros(self.n_sim)
        self.potencias_gerada = np.zeros(self.n_sim)
        self.potencia_eletrica_entregue_w = np.zeros(self.n_sim)
        self.potencias_inercial = np.zeros(self.n_sim)
        self.potencias_perdas_efeito_joule = np.zeros(self.n_sim)
        self.potencias_perdas_atrito_pas = np.zeros(self.n_sim)
        self.potencias_perdas_atrito_gerador = np.zeros(self.n_sim)
        
        # Eficiência e Erros
        self.coeficientes_potencia_digitais = np.zeros(self.n_sim)
        self.erros_balanco_energetico_digital = np.zeros(self.n_sim)

def calcular_velocidade_vento_degraus(tempo_decorrido_segundos):
    if tempo_decorrido_segundos < 10.0: return 0.0
    if tempo_decorrido_segundos < 20.0: return 1.0
    if tempo_decorrido_segundos < 30.0: return 2.0
    if tempo_decorrido_segundos < 40.0: return 3.0
    if tempo_decorrido_segundos < 50.0: return 4.0
    if tempo_decorrido_segundos < 60.0: return 5.0
    if tempo_decorrido_segundos < 70.0: return 6.0
    if tempo_decorrido_segundos < 80.0: return 7.0
    if tempo_decorrido_segundos < 90.0: return 0.0
    return 0.0

def calcular_velocidade_vento_rampa(tempo):
    # Rampa de subida (de 0 a 50 segundos)
    # Sobe de 0.0 até 4.0
    if 0.0 <= tempo < 50.0:
        return (10.0 / 50.0) * tempo
    
    # Rampa de descida (de 50 a 100 segundos)
    # Desce de 4.0 até 0.0
    elif 50.0 <= tempo < 100.0:
        return 10.0 - ((10.0 / 50.0) * (tempo - 50.0))
    
    # Fora do intervalo
    return 0.0


def plotar_graficos_analise_final(registros, nome_ensaio=f"simu_gemeo_uxi_{PERFIL_VENTO}"):
    s = registros.idx_sim
    t = registros.tempos_simulacao_digital[:s]

    # Estilo geral
    plt.style.use('default')
    
    # -------------------------------------------------------------
    # 1. FIGURA: DADOS DE ENTRADA E CINEMÁTICA
    # -------------------------------------------------------------
    fig1, axs1 = plt.subplots(2, 1, sharex=True)
    fig1.canvas.manager.set_window_title('Gêmeo Digital - Condições e Cinemática')
    fig1.suptitle('Condições de Entrada e Cinemática do Sistema', fontsize=14, fontweight='bold')

    axs1[0].plot(t, registros.ventos_simulacao_entrada[:s], color='dodgerblue', lw=2, label='Vento Input')
    axs1[0].set_ylabel('Vento [m/s]')
    axs1[0].legend(loc='upper right'); axs1[0].grid(True, ls='--')

    # Restaurado: Ângulo de Pitch
    axs1[1].plot(t, np.rad2deg(registros.angulos_pas_radianos[:s]), color='darkcyan', lw=2, label='Ângulo de Pitch')
    axs1[1].set_ylabel('Pitch [Graus]')
    
    # Movido para um terceiro sub-plot ou mantido junto se desejar comparar
    # Nota: Se quiser manter os dois no mesmo eixo, remova o comment abaixo e ajuste axs1[1]
    axs1[1].plot(t, registros.velocidades_angulares_gerador_digital[:s], color='purple', lw=2, label='Velocidade Angular (Modelo)')
    axs1[1].plot(t, registros.velocidades_angulares_gerador_ideais[:s], color='lightcoral', lw=2, alpha=0.7, ls='--', label=f'Velocidade Ideal (TSR={TSR_IDEAL})')
    axs1[1].set_ylabel('Rotação [RPM]')
    axs1[1].set_xlabel('Tempo [s]')
    axs1[1].legend(loc='upper right'); axs1[1].grid(True, ls='--')
    plt.tight_layout()
    fig1.savefig(f"{nome_ensaio}_condicoes_cinematica.pdf", format='pdf', bbox_inches='tight')

    # -------------------------------------------------------------
    # 2. FIGURA: GRANDEZAS ELÉTRICAS
    # -------------------------------------------------------------
    fig2, axs2 = plt.subplots(2, 1, sharex=True)
    axs2[0].plot(t, registros.tensoes_barramento_volts[:s], color='blue', lw=2, label='Tensão Barramento (V_DC)')
    axs2[0].plot(t, registros.forca_eletromotriz_filtrada[:s], color='orange', lw=2, alpha=0.8, ls='--', label='Força Eletromotriz (FEM Filtrada)')
    axs2[0].set_ylabel('Tensão [V]')
    axs2[0].legend(loc='upper right'); axs2[0].grid(True, ls='--')

    axs2[1].plot(t, registros.correntes_armadura[:s], color='red', lw=2, label='Corrente da Armadura (I_a)')
    axs2[1].set_ylabel('Corrente [A]')
    axs2[1].set_xlabel('Tempo [s]')
    axs2[1].legend(loc='upper right'); axs2[1].grid(True, ls='--')

    plt.tight_layout()
    fig2.savefig(f"{nome_ensaio}_grandezas_eletricas.pdf", format='pdf', bbox_inches='tight')

    fig2_1, axs2_1 = plt.subplots(figsize=(10, 5)) 
    axs2_1.plot(registros.tensoes_barramento_volts[:s], registros.correntes_armadura[:s], 
                color='red', lw=2, label='I_a(V_bus) Real')
    axs2_1.set_xlabel('Tensão [V]')
    axs2_1.set_ylabel('Corrente [A]')
    axs2_1.legend(loc='upper left')
    axs2_1.grid(True, ls='--')

    plt.tight_layout()
    fig2_1.savefig(f"{nome_ensaio}_curva_UxI.pdf", format='pdf', bbox_inches='tight')

    # -------------------------------------------------------------
    # 3. FIGURA: DINÂMICA DE TORQUES
    # -------------------------------------------------------------
    fig3, ax3 = plt.subplots()
    ax3.plot(t, registros.torques_aerodinamicos_digitais[:s], color='orange', lw=2, label='T_Aerodinâmico (no gerador)')
    ax3.plot(t, registros.torques_eletromagneticos_digitais[:s], color='firebrick', lw=2, label='T_Eletromagnético')
    ax3.plot(t, registros.torques_friccao_digitais[:s], color='orange', lw=1.5, label='T_Fricção (Gerador)')
    ax3.plot(t, registros.torques_atrito_pas[:s], color='gold', lw=1.5, label='T_Fricção (Pás)')
    ax3.plot(t, registros.torques_aceleracao[:s], color='black', lw=1.5, ls='-.', label='T_Aceleração (Inercial)')
    
    ax3.set_title('Dinâmica de Torques', fontsize=14, fontweight='bold')
    ax3.set_ylabel('Torque [N.m]'); ax3.set_xlabel('Tempo [s]')
    ax3.legend(loc='upper right'); ax3.grid(True, ls='--')
    plt.tight_layout()
    fig3.savefig(f"{nome_ensaio}_dinamica_torques.pdf", format='pdf', bbox_inches='tight')

    # -------------------------------------------------------------
    # 4. FIGURA: FLUXO DE POTÊNCIA
    # -------------------------------------------------------------
    fig4, ax4 = plt.subplots()
    ax4.plot(t, registros.potencias_absorvida_vento[:s], color='orange', lw=2, label='Potência Absorvida do Vento')
    ax4.plot(t, registros.potencias_eixo_alta_velocidade_digitais[:s], color='green', lw=2, label='Potência Eixo Alta Vel. (Mecânica)')
    ax4.plot(t, registros.potencias_gerada[:s], color='cyan', lw=2, label='Potência Gerada (Interna)')
    ax4.plot(t, registros.potencia_eletrica_entregue_w[:s], color='red', lw=2, label='Potência Entregue (Saída Útil)')
    ax4.plot(t, registros.potencias_inercial[:s], color='black', lw=1.5, ls=':', label='Potência Inercial (Aceleração)')
    
    ax4.set_title('Fluxo de Potência', fontsize=14, fontweight='bold')
    ax4.set_ylabel('Potência [W]'); ax4.set_xlabel('Tempo [s]')
    ax4.legend(loc='upper right'); ax4.grid(True, ls='--')
    plt.tight_layout()
    fig4.savefig(f"{nome_ensaio}_fluxo_potencias.pdf", format='pdf', bbox_inches='tight')

    # -------------------------------------------------------------
    # 5. FIGURA: PERDAS, BALANÇO ENERGÉTICO E EFICIÊNCIA
    # -------------------------------------------------------------
    fig5, axs5 = plt.subplots(3, 1, sharex=True, figsize=(8, 10))
    fig5.canvas.manager.set_window_title('Gêmeo Digital - Balanço e Eficiência')
    fig5.suptitle('Análise Energética do Sistema', fontsize=14, fontweight='bold')

    # Subplot 1: Perdas
    axs5[0].plot(t, registros.potencias_perdas_efeito_joule[:s], color='red', lw=1.5, label='Perdas Joule (I²R)')
    axs5[0].plot(t, registros.potencias_perdas_atrito_pas[:s], color='olive', lw=1.5, label='Perdas Atrito (Pás)')
    axs5[0].plot(t, registros.potencias_perdas_atrito_gerador[:s], color='darkkhaki', lw=1.5, label='Perdas Atrito (Gerador)')
    axs5[0].set_ylabel('Perdas [W]')
    axs5[0].legend(loc='upper right'); axs5[0].grid(True, ls='--')

    # Subplot 2: Erro de Balanço Energético (Adicionado)
    axs5[1].plot(t, registros.erros_balanco_energetico_digital[:s], color='black', lw=2, label='Erro de Balanço Energético')
    axs5[1].set_ylabel('Erro [W]')
    axs5[1].legend(loc='upper right'); axs5[1].grid(True, ls='--')

    # Subplot 3: Cp
    axs5[2].plot(t, registros.coeficientes_potencia_digitais[:s], color='purple', lw=2, label='Coeficiente de Potência (Cp)')
    axs5[2].axhline(y=0.593, color='r', linestyle='--', zorder=4, label='Limite de Betz')
    axs5[2].set_ylim(0, 0.7)
    axs5[2].set_ylabel('Cp [-]')
    axs5[2].set_xlabel('Tempo [s]')
    axs5[2].legend(loc='upper right'); axs5[2].grid(True, ls='--')
    
    plt.tight_layout()
    fig5.savefig(f"{nome_ensaio}_eficiencia_balanco.pdf", format='pdf', bbox_inches='tight')

    plt.show()


turbina_gemeo_digital = TurbinaVirtual(
    constante_velocidade=CONSTANTE_VELOCIDADE_GERADOR_V_RAD_S, 
    offset_fem=OFFSET_FEM,
    ganho_armadura=GANHO_ARMADURA, 
    tau_armadura=TAU_ARMADURA,
    resistencia_interna=RESISTENCIA_INTERNA_OHMS,
    capacitancia_barramento=CAPACITANCIA_BARRAMENTO_FARADAY,
    raio_turbina=RAIO_TURBINA_METROS, 
    inercia_turbina=INERCIA_TURBINA_KG_M2, 
    inercia_gerador=INERCIA_GERADOR_KG_M2, 
    coeficiente_atrito_turbina=COEFICIENTE_ATRITO_TURBINA,
    coeficiente_atrito_gerador = COEFICIENTE_ATRITO_GERADOR,
    taxa_variacao_tensao_barramento=TAXA_VARIACAO_TENSAO_BARRAMENTO_V_S, 
    taxa_variacao_rpm_maxima=TAXA_VARIACAO_RPM_MAX,
    velocidade_angular_maxima_pas=VELOCIDADE_MAXIMA_PAS_GRAUS_S, 
    torque_maximo_motor=TORQUE_MAXIMO_MOTOR_NM,
    tsr_ideal=TSR_IDEAL,
    relacao_transmissao_caixa_engrenagens=RELACAO_TRANSMISSAO_CAIXA_ENGRENAGENS, 
    modelo_coeficiente_potencia=0,
    densidade_ar_kg_m3=DENSIDADE_AR_KG_M3,
    sinal_controle_mppt='corrente de carga'
)


from pid_module import PIDController
pid_mppt = PIDController(kp=2.0, ki=1.0, kd=0.0, out_min=0, out_max=16.5)
corrente_carga_inversor = 0.0
corrente_carga_inversor_filtrada = corrente_carga_inversor
alpha = 0.1

registros = RegistroDeSimulacao(TEMPO_TOTAL_EMULACAO_SEGUNDOS, PASSO_SIMULACAO_SEGUNDOS)

velocidade_vento_atual_m_s = 0.0
index_vetor_vento = 0

tempo_decorrido = 0.0

print(f"Simulação do Gêmeo Digital Iniciada por {TEMPO_TOTAL_EMULACAO_SEGUNDOS}s...")

estado_anterior_coef_torque = True
estado_anterior_flag_vazio = True


# --- Parâmetros de Inicialização do Inversor ---
tempo_acumulado_acima_minimo = 0.0  # Cronômetro
inversor_conectado = False  # Estado lógico do inversor

while (tempo_decorrido) < TEMPO_TOTAL_EMULACAO_SEGUNDOS:

    if (tempo_decorrido) >= PASSO_SIMULACAO_SEGUNDOS: 
        
        if PERFIL_VENTO == 'CONSTANTE':
                if tempo_decorrido < 10.0:
                    velocidade_vento_atual_m_s = vento_constante * (tempo_decorrido / 10.0)
                else:
                    velocidade_vento_atual_m_s = vento_constante   
        if PERFIL_VENTO == 'ESCADA' : velocidade_vento_atual_m_s = calcular_velocidade_vento_degraus(tempo_decorrido)
        if PERFIL_VENTO == 'RAMPA'  : velocidade_vento_atual_m_s = calcular_velocidade_vento_rampa(tempo_decorrido)
        if PERFIL_VENTO == 'NATURAL': 
            try: 
                velocidade_vento_atual_m_s = VETOR_VENTO_M_S[index_vetor_vento]
                index_vetor_vento += 1
            except: 
                velocidade_vento_atual_m_s = VETOR_VENTO_M_S[index_vetor_vento - 1]
                index_vetor_vento = 100
                
        registros.ventos_simulacao_entrada[registros.idx_sim] = velocidade_vento_atual_m_s
        
        # -------------------------------------------------------------------------
        # CONTROLE MPPT - PID PARA AJUSTE
        # -------------------------------------------------------------------------
        omega_alvo = (velocidade_vento_atual_m_s * TSR_IDEAL / RAIO_TURBINA_METROS)
            
        tensao_atual_barramento = turbina_gemeo_digital._tensao_capacitor_volts
        fem = turbina_gemeo_digital._forca_eletromotriz

        u = pid_mppt.compute(turbina_gemeo_digital.get_velocidade_angular_turbina_rad_s(), omega_alvo,PASSO_SIMULACAO_SEGUNDOS, True)
        corrente_carga_inversor = u
        if corrente_carga_inversor == 0.0: 
            turbina_gemeo_digital.set_flag_vazio(True)
        else: 
            turbina_gemeo_digital.set_flag_vazio(False)

        ANGULO_PAS_REFERENCIA_RADIANOS = 0  

        corrente_carga_inversor_filtrada = (1-alpha)*corrente_carga_inversor_filtrada+alpha*corrente_carga_inversor

        turbina_gemeo_digital.executar_passo_simulacao(
            velocidade_vento_atual_m_s,
            ANGULO_PAS_REFERENCIA_RADIANOS, 
            PASSO_SIMULACAO_SEGUNDOS,
            corrente_carga = corrente_carga_inversor_filtrada 
        ) 
        
        velocidade_angular_alvo_gerador_rad_s = (velocidade_vento_atual_m_s * TSR_IDEAL / RAIO_TURBINA_METROS) * RELACAO_TRANSMISSAO_CAIXA_ENGRENAGENS

        if registros.idx_sim < registros.n_sim:
            registros.tempos_simulacao_digital[registros.idx_sim] = tempo_decorrido
            registros.ventos_simulacao_entrada[registros.idx_sim] = velocidade_vento_atual_m_s
            registros.angulos_pas_radianos[registros.idx_sim] = turbina_gemeo_digital.get_angulo_pas_radianos()
            
            registros.velocidades_angulares_gerador_digital[registros.idx_sim] = turbina_gemeo_digital.get_velocidade_angular_gerador_rad_s() * 60.0 / (2.0 * np.pi)
            registros.velocidades_angulares_gerador_ideais[registros.idx_sim] = velocidade_angular_alvo_gerador_rad_s * 60.0 / (2.0 * np.pi)
            
            # Grandezas Elétricas
            registros.tensoes_barramento_volts[registros.idx_sim] = turbina_gemeo_digital.get_tensao_capacitor_volts()
            registros.forca_eletromotriz_filtrada[registros.idx_sim] = turbina_gemeo_digital.get_forca_eletromotriz()
            registros.correntes_armadura[registros.idx_sim] = corrente_carga_inversor#turbina_gemeo_digital.get_corrente_armadura_amperes()
            
            # Dinâmica de Torques
            registros.torques_aerodinamicos_digitais[registros.idx_sim] = turbina_gemeo_digital._torque_aerodinamico_no_gerador
            registros.torques_eletromagneticos_digitais[registros.idx_sim] = turbina_gemeo_digital._torque_eletromagnetico_gerador_nm
            registros.torques_friccao_digitais[registros.idx_sim] = turbina_gemeo_digital._torque_atrito_gerador_nm
            registros.torques_atrito_pas[registros.idx_sim] = turbina_gemeo_digital._torque_atrito_pas_nm
            registros.torques_aceleracao[registros.idx_sim] = turbina_gemeo_digital._torque_aceleracao_nm
            
            # Dinâmica de Potências
            registros.potencias_absorvida_vento[registros.idx_sim] = turbina_gemeo_digital._potencia_absorvida_vento_w
            registros.potencias_eixo_alta_velocidade_digitais[registros.idx_sim] = turbina_gemeo_digital.get_potencia_eixo_alta_velocidade_w()
            registros.potencias_gerada[registros.idx_sim] = turbina_gemeo_digital._potencia_gerada_w
            registros.potencia_eletrica_entregue_w[registros.idx_sim] = turbina_gemeo_digital.get_potencia_eletrica_entregue_w()
            registros.potencias_inercial[registros.idx_sim] = turbina_gemeo_digital._potencia_inercial_w
            
            # Perdas e Erros
            registros.potencias_perdas_efeito_joule[registros.idx_sim] = turbina_gemeo_digital._potencia_perdas_efeito_joule_w
            registros.potencias_perdas_atrito_pas[registros.idx_sim] = turbina_gemeo_digital._potencia_perdas_atrito_pas_w
            registros.potencias_perdas_atrito_gerador[registros.idx_sim] = turbina_gemeo_digital._potencia_perdas_atrito_gerador_w
            registros.erros_balanco_energetico_digital[registros.idx_sim] = turbina_gemeo_digital._erro_balanco_energetico_w
            
            # Eficiência
            potencia_disponivel_vento = 0.5 * DENSIDADE_AR_KG_M3 * (np.pi*RAIO_TURBINA_METROS**2) * (velocidade_vento_atual_m_s**3) if velocidade_vento_atual_m_s > 0 else 0.01
            registros.coeficientes_potencia_digitais[registros.idx_sim] = (turbina_gemeo_digital._potencia_gerada_w / potencia_disponivel_vento)
            
            registros.idx_sim += 1

        estado_atual_coef_torque = turbina_gemeo_digital.get_flag_torque_estatico()
        if estado_anterior_coef_torque != estado_atual_coef_torque:
            print(f"Transição detectada! Sistema operando com coef. torque estático?: {estado_atual_coef_torque}. Tempo: {tempo_decorrido:.3f} s")
        estado_anterior_coef_torque = estado_atual_coef_torque

        estado_atual_flag_vazio = turbina_gemeo_digital.get_flag_vazio()
        if estado_anterior_flag_vazio != estado_atual_flag_vazio:
            print(f"Transição detectada! Sistema operando a vazio?: {estado_atual_flag_vazio}. Tempo: {tempo_decorrido:.3f} s")
        estado_anterior_flag_vazio = estado_atual_flag_vazio

    tempo_decorrido += PASSO_SIMULACAO_SEGUNDOS

plt.ioff()
print("Simulação Finalizada. Gerando gráficos de análise...")
potencias = registros.potencia_eletrica_entregue_w[:registros.idx_sim]
energia_joules = np.sum(potencias) * PASSO_SIMULACAO_SEGUNDOS
energia_wh = energia_joules / 3600.0

print(f"\n>> [LOG ENERGIA] Total entregue para a curva UxI ajustada: {energia_wh:.4f} Wh ({energia_joules:.2f} J)")
plotar_graficos_analise_final(registros)