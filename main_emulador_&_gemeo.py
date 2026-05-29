import time
import numpy as np
import matplotlib.pyplot as plt

from parametros import *
from modelo_aerogerador import TurbinaVirtual
from driver_torquimetro import Torquimeter
from driver_inversor_cfw11 import Inverter
from pid_module import PIDController 
from init_serial_devices import selecionar_porta

class RegistroDeEmulacao:
    def __init__(self, duracao_s, passo_hardware, passo_simulacao):
        self.n_hw = int(duracao_s / passo_hardware) + 1
        self.n_sim = int(duracao_s / passo_simulacao) + 1
        
        self.idx_hw = 0
        self.idx_sim = 0

        # --- REGISTROS DO GÊMEO DIGITAL (SIMULAÇÃO) ---
        self.tempos_simulacao_digital = np.zeros(self.n_sim)
        self.ventos_simulacao_entrada = np.zeros(self.n_sim)
        self.angulos_pas_radianos = np.zeros(self.n_sim)
        
        self.velocidades_angulares_gerador_digital = np.zeros(self.n_sim)
        self.velocidades_angulares_gerador_ideais = np.zeros(self.n_sim)
        
        self.tensoes_barramento_volts = np.zeros(self.n_sim)
        self.forca_eletromotriz_filtrada = np.zeros(self.n_sim)
        self.correntes_armadura_digitais = np.zeros(self.n_sim)
        
        self.torques_no_eixo_digital = np.zeros(self.n_sim)
        self.torques_eletromagneticos_digitais = np.zeros(self.n_sim)
        self.torques_friccao_digitais = np.zeros(self.n_sim)
        self.torques_atrito_pas = np.zeros(self.n_sim)
        self.torques_aceleracao = np.zeros(self.n_sim)
        
        self.potencias_absorvida_vento = np.zeros(self.n_sim)
        self.potencias_eixo_alta_velocidade_digitais = np.zeros(self.n_sim)
        self.potencias_gerada = np.zeros(self.n_sim)
        self.potencias_eletricas_digitais = np.zeros(self.n_sim) # Entregue util
        self.potencias_inercial = np.zeros(self.n_sim)
        
        self.potencias_perdas_efeito_joule = np.zeros(self.n_sim)
        self.potencias_perdas_atrito_pas = np.zeros(self.n_sim)
        self.potencias_perdas_atrito_gerador = np.zeros(self.n_sim)
        
        self.erros_balanco_energetico_digital = np.zeros(self.n_sim)
        self.coeficientes_potencia_digitais = np.zeros(self.n_sim)

        # --- REGISTROS DO HARDWARE (EMULADOR FÍSICO) ---
        self.tempos_hardware_fisico = np.zeros(self.n_hw)
        self.velocidades_vento_m_s_hw = np.zeros(self.n_hw)
        self.comandos_esforco_inversor = np.zeros(self.n_hw)
        
        self.velocidades_angulares_reais_rpm = np.zeros(self.n_hw)
        self.velocidades_angulares_ideais_rpm = np.zeros(self.n_hw)
        self.velocidades_angulares_referencia_rad_s = np.zeros(self.n_hw)
        self.velocidades_angulares_reais_rad_s = np.zeros(self.n_hw)
        
        self.torques_medidos_sensor_nm = np.zeros(self.n_hw) 
        self.torques_hss_referencia = np.zeros(self.n_hw) 
        
        self.potencias_medidas_sensor = np.zeros(self.n_hw)
        self.latencias_comunicacao_ms = np.zeros(self.n_hw)
        self.coeficientes_potencia_reais = np.zeros(self.n_hw)

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
    if 0.0 <= tempo < 50.0:
        return (4.0 / 50.0) * tempo
    elif 50.0 <= tempo < 100.0:
        return 4.0 - ((4.0 / 50.0) * (tempo - 50.0))
    return 0.0

def aplicar_filtro_passa_baixa(valor_atual, valor_anterior_filtrado, passo_tempo, constante_tempo):
    fator_suavizacao = passo_tempo / (constante_tempo + passo_tempo)
    return fator_suavizacao * valor_atual + (1.0 - fator_suavizacao) * valor_anterior_filtrado

def configurar_interface_tempo_real():
    plt.ion() 
    figura, (eixo_velocidade, eixo_potencia, eixo_corrente) = plt.subplots(3, 1, figsize=(10, 10))
    figura.canvas.manager.set_window_title('Monitoramento Multi-Malha: Hardware & Gêmeo Digital')

    linha_vel_referencia, = eixo_velocidade.plot([], [], 'r--', label='Velocidade Twin (Ref)')
    linha_vel_real, = eixo_velocidade.plot([], [], 'lightgray', alpha=0.7, label='Velocidade Hardware')
    eixo_velocidade.set_ylabel("Velocidade [rad/s]")
    eixo_velocidade.legend(loc='upper right')
    eixo_velocidade.grid(True)

    linha_potencia_virtual, = eixo_potencia.plot([], [], 'purple', label='P. Mecânica Twin')
    linha_potencia_real, = eixo_potencia.plot([], [], 'lightgray', alpha=0.7, label='P. Mecânica HW')
    linha_potencia_eletrica, = eixo_potencia.plot([], [], 'blue', linewidth=1.2, label='P. Elétrica Twin')
    eixo_potencia.set_ylabel("Potência [W]")
    eixo_potencia.legend(loc='upper right')
    eixo_potencia.grid(True)

    linha_corrente, = eixo_corrente.plot([], [], 'orange', label='Corrente Twin')
    eixo_corrente.set_xlabel("Tempo [s]")
    eixo_corrente.set_ylabel("Corrente [A]")
    eixo_corrente.legend(loc='upper right')
    eixo_corrente.grid(True)

    visor_digital = eixo_potencia.text(0.5, 0.85, '', transform=eixo_potencia.transAxes, 
                                       fontsize=14, fontweight='bold', color='darkgreen',
                                       ha='center', bbox=dict(facecolor='white', alpha=0.8))
    
    plt.tight_layout()
    return (figura, eixo_velocidade, eixo_potencia, eixo_corrente, 
            linha_vel_referencia, linha_vel_real, linha_potencia_virtual, 
            linha_potencia_real, linha_potencia_eletrica, linha_corrente, visor_digital)

def plotar_graficos_analise_final(registros, nome_ensaio=f"ensaio_integrado_{PERFIL_VENTO}"):
    from scipy.signal import butter, filtfilt

    h = registros.idx_hw
    s = registros.idx_sim
    t_hw = registros.tempos_hardware_fisico[:h]
    t_sim = registros.tempos_simulacao_digital[:s]

    if h == 0 or s == 0:
        print("Dados insuficientes para gerar gráficos de análise.")
        return

    # --- FUNÇÃO AUXILIAR PARA ALINHAMENTO TEMPORAL ---
    def interpolar_twin_para_hw(sinal_twin):
        return np.interp(t_hw, t_sim, sinal_twin)

    # --- PROCESSAMENTO DOS DADOS HW (FILTROS) ---
    fs = 1.0 / PASSO_HARDWARE_SEGUNDOS
    nyq = 0.5 * fs
    fc = 1.0 / (2 * np.pi * 0.1)
    wn = min(fc / nyq, 0.99) 
    b, a = butter(2, wn, btype='low') 
    
    torques_filtrados_plot = filtfilt(b, a, registros.torques_medidos_sensor_nm[:h])
    potencias_filtradas_plot = filtfilt(b, a, registros.potencias_medidas_sensor[:h])
    cp_filtrado_plot = filtfilt(b, a, registros.coeficientes_potencia_reais[:h])

    # Dados do Twin alinhados para o erro
    rpm_twin_interp = interpolar_twin_para_hw(registros.velocidades_angulares_gerador_digital[:s])
    torque_twin_interp = interpolar_twin_para_hw(registros.torques_no_eixo_digital[:s])
    pot_twin_interp = interpolar_twin_para_hw(registros.potencias_gerada[:s])

    plt.style.use('default')

    # 1. Cinemática (Adicionado Erro de RPM)
    fig1, axs1 = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig1.canvas.manager.set_window_title('Integração - Cinemática')
    axs1[0].plot(t_sim, registros.ventos_simulacao_entrada[:s], color='dodgerblue', lw=2, label='Vento de Entrada [m/s]')
    axs1[0].set_ylabel('Vento [m/s]'); axs1[0].legend(); axs1[0].grid(True)

    axs1[1].plot(t_hw, registros.velocidades_angulares_reais_rpm[:h], color='lightgray', alpha=0.8, lw=2, label='RPM Real (Sensor)')
    axs1[1].plot(t_sim, registros.velocidades_angulares_gerador_digital[:s], color='purple', lw=2, label='RPM Modelo (Gemeo)')
    axs1[1].plot(t_hw, registros.velocidades_angulares_ideais_rpm[:h], color='lightcoral', lw=1.5, ls='--', label='RPM Ideal (TSR ideal)')
    axs1[1].set_ylabel('Rotação [RPM]'); axs1[1].legend(); axs1[1].grid(True)
    
    axs1[2].plot(t_hw, registros.velocidades_angulares_reais_rpm[:h] - rpm_twin_interp, color='red', lw=1.5, label='Erro RPM (Real - Modelo)')
    axs1[2].set_ylabel('Erro RPM [RPM]'); axs1[2].set_xlabel('Tempo [s]'); axs1[2].legend(); axs1[2].grid(True)
    plt.tight_layout()
    fig1.savefig(f"{nome_ensaio}_cinematica.svg", format='svg', bbox_inches='tight')

    # 2. Dinâmica de Torques (Adicionado Erro de Torque)
    fig2, axs2 = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig2.canvas.manager.set_window_title('Integração - Dinâmica de Torques')
    axs2[0].plot(t_sim, registros.torques_no_eixo_digital[:s], color='forestgreen', alpha=0.6, lw=3, label='Torque Mecanico Eixo (Gemeo)')
    axs2[0].plot(t_hw, registros.torques_medidos_sensor_nm[:h], color='lightgray', alpha=0.7, lw=2, label='Torque Real Bruto (Sensor)')
    axs2[0].plot(t_hw, torques_filtrados_plot, color='black', lw=1.5, ls='--', label='Torque Mecanico Real Filtrado')
    axs2[0].set_ylabel('Torque [N.m]'); axs2[0].legend(); axs2[0].grid(True)
    
    axs2[1].plot(t_hw, torques_filtrados_plot - torque_twin_interp, color='red', lw=1.5, label='Erro Torque (Real - Modelo)')
    axs2[1].set_ylabel('Erro Torque [N.m]'); axs2[1].set_xlabel('Tempo [s]'); axs2[1].legend(); axs2[1].grid(True)
    plt.tight_layout()
    fig2.savefig(f"{nome_ensaio}_torque.svg", format='svg', bbox_inches='tight')

    # 3. Potência e Eficiência (Adicionado Erro de Potência)
    fig3, axs3 = plt.subplots(3, 1, figsize=(12, 12), sharex=True)
    fig3.canvas.manager.set_window_title('Integração - Potência e Eficiência')
    axs3[0].plot(t_hw, registros.potencias_medidas_sensor[:h], color='lightgray', alpha=0.7, lw=2, label='Potência Mecânica Eixo (Sensor)')
    axs3[0].plot(t_hw, potencias_filtradas_plot, color='black', lw=1.5, ls='--', label='Potência Mecânica Real Filtrada')
    axs3[0].plot(t_sim, registros.potencias_gerada[:s], color='green', lw=2, label='Potência Mecânica Eixo (Gemeo)')
    axs3[0].set_ylabel('Potência [W]'); axs3[0].legend(); axs3[0].grid(True)

    axs3[1].plot(t_hw, cp_filtrado_plot, color='black', ls='--', lw=2, label='Cp Real (Filtrado)')
    axs3[1].plot(t_sim, registros.coeficientes_potencia_digitais[:s], color='purple', lw=2, label='Cp Twin (Modelo)')
    axs3[1].axhline(y=0.593, color='r', linestyle='--', label='Limite de Betz')
    axs3[1].set_ylim(0, 0.7); axs3[1].set_ylabel('Cp [-]'); axs3[1].legend(); axs3[1].grid(True)
    
    axs3[2].plot(t_hw, potencias_filtradas_plot - pot_twin_interp, color='red', lw=1.5, label='Erro Potência (Real - Modelo)')
    axs3[2].set_ylabel('Erro Potência [W]'); axs3[2].set_xlabel('Tempo [s]'); axs3[2].legend(); axs3[2].grid(True)
    plt.tight_layout()
    fig3.savefig(f"{nome_ensaio}_potencia.svg", format='svg', bbox_inches='tight')

    # 4. Detalhamento Gêmeo Digital: Elétrica e Fluxo (Mantido Original)
    fig4, axs4 = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig4.canvas.manager.set_window_title('Integração - Elétrica')
    axs4[0].plot(t_sim, registros.tensoes_barramento_volts[:s], color='blue', lw=2, label='Tensão No Capacitor (Gemeo)')
    axs4[0].plot(t_sim, registros.forca_eletromotriz_filtrada[:s], color='orange', ls='--', label='Forca Eletromotriz (Gemeo)')
    axs4[0].set_ylabel('Tensão [V]'); axs4[0].legend(); axs4[0].grid(True)
    axs4[1].plot(t_sim, registros.correntes_armadura_digitais[:s], color='red', lw=2, label='Corrente Armadura (Gemeo)')
    axs4[1].set_ylabel('Corrente [A]'); axs4[1].set_xlabel('Tempo [s]'); axs4[1].legend(); axs4[1].grid(True)
    plt.tight_layout()
    fig4.savefig(f"{nome_ensaio}_twin_eletrica.svg", format='svg', bbox_inches='tight')

    # 5. Performance Hardware (Mantido Original)
    fig5, ax5 = plt.subplots(figsize=(12, 4))
    fig5.canvas.manager.set_window_title('Performance - Latência')
    ax5.plot(t_hw, registros.latencias_comunicacao_ms[:h], color='teal', alpha=0.7, lw=1.5, label='Latência Ciclo Serial')
    ax5.axhline(y=PASSO_HARDWARE_SEGUNDOS*1000, color='r', linestyle='--', zorder=4, label=f"Limite RT ({PASSO_HARDWARE_SEGUNDOS*1000} ms)")
    ax5.set_ylabel('Latência [ms]'); ax5.set_xlabel('Tempo [s]'); ax5.legend(); ax5.grid(True)
    plt.tight_layout()
    fig5.savefig(f"{nome_ensaio}_latencia.svg", format='svg', bbox_inches='tight')

    # 6. Performance do Controle de Torque (Seguimento e Esforço)
    fig6, axs6 = plt.subplots(3, 1, figsize=(12, 12), sharex=True)
    fig6.canvas.manager.set_window_title('Hardware - Controle de Torque')
    
    # Gráfico 1: Seguimento da Referência
    axs6[0].plot(t_hw, registros.torques_hss_referencia[:h], color='blue', lw=2, label='Torque Aero. Referência')
    axs6[0].plot(t_hw, registros.torques_medidos_sensor_nm[:h], color='lightgray', alpha=0.7, lw=1.5, label='Torque Real (Bruto)')
    axs6[0].plot(t_hw, torques_filtrados_plot, color='black', ls='--', lw=2, label='Torque Real (Filtrado)')
    axs6[0].set_ylabel('Torque [N.m]')
    axs6[0].set_title('Seguimento de Referência de Torque Aero. Eixo')
    axs6[0].legend(loc='upper right')
    axs6[0].grid(True)

    # Gráfico 2: Erro de Seguimento
    erro_seguimento = registros.torques_hss_referencia[:h] - torques_filtrados_plot
    axs6[1].plot(t_hw, erro_seguimento, color='red', lw=1.5, label='Erro de Seguimento (Ref - Filtrado)')
    axs6[1].axhline(y=0, color='black', lw=1, ls='--')
    axs6[1].set_ylabel('Erro [N.m]')
    axs6[1].legend(loc='upper right')
    axs6[1].grid(True)

    # Gráfico 3: Sinal de Controle (Esforço do Inversor)
    axs6[2].plot(t_hw, registros.comandos_esforco_inversor[:h], color='purple', lw=2, label='Esforço de Controle (Sinal Inversor)')
    axs6[2].set_ylabel('Comando Velocidade')
    axs6[2].set_xlabel('Tempo [s]')
    axs6[2].legend(loc='upper right')
    axs6[2].grid(True)

    plt.tight_layout()
    fig6.savefig(f"{nome_ensaio}_controle_torque.svg", format='svg', bbox_inches='tight')

    plt.show()

# -------------------------------------------------------------------------
# INICIALIZAÇÃO DOS MODELOS E COMUNICAÇÃO
# ------------------------------------------------------------------------- 
# O Gêmeo Digital que simula todo o comportamento (Cinemático e Elétrico MPPT)
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
    sinal_controle_mppt='resistencia de carga'
)

# A turbina de referência para o Emulador (usada para calcular apenas a curva aerodinâmica T(w))
turbina_do_emulador = TurbinaVirtual(
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
    sinal_controle_mppt='resistencia de carga'
)

torquimetro_fisico = Torquimeter(Port=selecionar_porta("Torquimetro"), Baudrate=230400, Timeout=0.001) 
inversor_motor = Inverter(Port=selecionar_porta("Inversor"), ADR=1, Baudrate=57600, Timeout=0.001)

inversor_motor.ActivateMotor()
time.sleep(0.1)
inversor_motor.SendReferenceAngularVelocity(0)
time.sleep(1)

# Controladores
controlador_seguimento_torque = PIDController(kp=450.0, ki=300.0, kd=0.0, out_min=0.0, out_max=1000.0) 

registros = RegistroDeEmulacao(TEMPO_TOTAL_EMULACAO_SEGUNDOS, PASSO_HARDWARE_SEGUNDOS, PASSO_SIMULACAO_SEGUNDOS)
figura, eixo_velocidade, eixo_potencia, eixo_corrente, linha_vel_referencia, linha_vel_real, linha_potencia_virtual, linha_potencia_real, linha_potencia_eletrica, linha_corrente, visor_digital = configurar_interface_tempo_real()

# Variáveis de Estado de inicialização
velocidade_vento_atual_m_s = 0.0
comando_velocidade_inversor_filtrado = 0.0
index_vetor_vento = 0

estado_anterior_coef_torque_twin = True
estado_anterior_coef_torque_hw = True
estado_anterior_flag_vazio = True

tempo_inicio_absoluto = time.perf_counter()
tempo_ultima_simulacao_digital = tempo_inicio_absoluto
tempo_ultimo_controle_hardware = tempo_inicio_absoluto
tempo_ultima_atualizacao_graficos = tempo_inicio_absoluto

# --- Parâmetros de Inicialização do Inversor ---
tempo_acumulado_acima_minimo = 0.0  # Cronômetro
inversor_conectado = False  # Estado lógico do inversor

historico_eventos = []

print(f"Emulação Integrada (Hardware & Twin) Iniciada por {TEMPO_TOTAL_EMULACAO_SEGUNDOS}s...")

try:
    while (time.perf_counter() - tempo_inicio_absoluto) < TEMPO_TOTAL_EMULACAO_SEGUNDOS:
        tempo_atual = time.perf_counter()
        tempo_decorrido = tempo_atual - tempo_inicio_absoluto
    
        # --- GERAÇÃO UNIFICADA DO PERFIL DE VENTO ---
        # Atualiza o vento na taxa de simulação digital mas atende o hardware também
        if (tempo_atual - tempo_ultima_simulacao_digital) >= PASSO_SIMULACAO_SEGUNDOS: 
            if PERFIL_VENTO == 'CONSTANTE':
                velocidade_vento_atual_m_s = vento_constante * (tempo_decorrido / 10.0) if tempo_decorrido < 10.0 else vento_constante
            elif PERFIL_VENTO == 'ESCADA': 
                velocidade_vento_atual_m_s = calcular_velocidade_vento_degraus(tempo_decorrido)
            elif PERFIL_VENTO == 'RAMPA':  
                velocidade_vento_atual_m_s = calcular_velocidade_vento_rampa(tempo_decorrido)
            elif PERFIL_VENTO == 'NATURAL': 
                try: 
                    velocidade_vento_atual_m_s = VETOR_VENTO_M_S[index_vetor_vento]
                    index_vetor_vento += 1
                except: 
                    velocidade_vento_atual_m_s = VETOR_VENTO_M_S[index_vetor_vento - 1]
                    index_vetor_vento = 100
            
            # -------------------------------------------------------------------------
            # CONTROLE MPPT - MODELO DO INVERSOR DEYE
            # -------------------------------------------------------------------------
            tensao_atual_barramento = turbina_gemeo_digital._tensao_capacitor_volts
            fem = turbina_gemeo_digital._forca_eletromotriz
            
            if fem >= TENSAO_MINIMA_INVERSOR_V:
                tempo_acumulado_acima_minimo += PASSO_SIMULACAO_SEGUNDOS
                
                if tempo_acumulado_acima_minimo >= TEMPO_ESPERA_LIGAR:
                    if not inversor_conectado:
                        historico_eventos.append(f"[{tempo_decorrido:.3f}s] [MPPT] Inversor ligou (Tensão > {TENSAO_MINIMA_INVERSOR_V}V por {TEMPO_ESPERA_LIGAR}s)")
                    inversor_conectado = True
            else:
                if inversor_conectado:
                    historico_eventos.append(f"[{tempo_decorrido:.3f}s] [MPPT] Inversor desligou! (Tensão caiu abaixo de {TENSAO_MINIMA_INVERSOR_V}V)")
                tempo_acumulado_acima_minimo = 0.0
                inversor_conectado = False
    
            if inversor_conectado:
                resistencia_carga_inversor = np.interp(
                    tensao_atual_barramento,
                    pontos_v_inversor_hibrido,  
                    pontos_r_inversor_hibrido,
                    left=1e50
                )
            else:
                resistencia_carga_inversor = 1e50
    
            if inversor_conectado: turbina_gemeo_digital.set_flag_vazio(False)
            if not inversor_conectado: turbina_gemeo_digital.set_flag_vazio(True)
    
            ANGULO_PAS_REFERENCIA_RADIANOS = 0  
    
            turbina_gemeo_digital.executar_passo_simulacao(
                velocidade_vento_atual_m_s, 
                ANGULO_PAS_REFERENCIA_RADIANOS, 
                PASSO_SIMULACAO_SEGUNDOS,
                resistencia_carga_inversor
            ) 
            
                    
            velocidade_angular_alvo_gerador_rad_s = (velocidade_vento_atual_m_s * TSR_IDEAL / RAIO_TURBINA_METROS) * RELACAO_TRANSMISSAO_CAIXA_ENGRENAGENS
    
            # Grava Registros do Twin
            if registros.idx_sim < registros.n_sim:
                s_idx = registros.idx_sim
                registros.tempos_simulacao_digital[s_idx] = tempo_decorrido
                registros.ventos_simulacao_entrada[s_idx] = velocidade_vento_atual_m_s
                registros.angulos_pas_radianos[s_idx] = turbina_gemeo_digital.get_angulo_pas_radianos()
                
                rpm_modelo = turbina_gemeo_digital.get_velocidade_angular_gerador_rad_s() * 60.0 / (2.0 * np.pi)
                registros.velocidades_angulares_gerador_digital[s_idx] = rpm_modelo
                omega_alvo_twin = (velocidade_vento_atual_m_s * TSR_IDEAL / RAIO_TURBINA_METROS)
                registros.velocidades_angulares_gerador_ideais[s_idx] = velocidade_angular_alvo_gerador_rad_s * 60.0 / (2.0 * np.pi)
                
                registros.tensoes_barramento_volts[s_idx] = turbina_gemeo_digital.get_tensao_capacitor_volts()
                registros.forca_eletromotriz_filtrada[s_idx] = turbina_gemeo_digital.get_forca_eletromotriz()
                registros.correntes_armadura_digitais[s_idx] = turbina_gemeo_digital.get_corrente_armadura_amperes()
                
                registros.torques_no_eixo_digital[s_idx] = turbina_gemeo_digital._torque_eletromagnetico_gerador_nm + turbina_gemeo_digital._torque_atrito_gerador_nm
                registros.torques_eletromagneticos_digitais[s_idx] = turbina_gemeo_digital._torque_eletromagnetico_gerador_nm
                registros.torques_friccao_digitais[s_idx] = turbina_gemeo_digital._torque_atrito_gerador_nm
                registros.torques_atrito_pas[s_idx] = turbina_gemeo_digital._torque_atrito_pas_nm
                registros.torques_aceleracao[s_idx] = turbina_gemeo_digital._torque_aceleracao_nm
                
                registros.potencias_absorvida_vento[s_idx] = turbina_gemeo_digital._potencia_absorvida_vento_w
                registros.potencias_gerada[s_idx] = turbina_gemeo_digital._potencia_gerada_w
                registros.potencias_eletricas_digitais[s_idx] = turbina_gemeo_digital.get_potencia_eletrica_entregue_w()
                registros.potencias_inercial[s_idx] = turbina_gemeo_digital._potencia_inercial_w
                
                registros.potencias_perdas_efeito_joule[s_idx] = turbina_gemeo_digital._potencia_perdas_efeito_joule_w
                registros.potencias_perdas_atrito_pas[s_idx] = turbina_gemeo_digital._potencia_perdas_atrito_pas_w
                registros.potencias_perdas_atrito_gerador[s_idx] = turbina_gemeo_digital._potencia_perdas_atrito_gerador_w
                registros.erros_balanco_energetico_digital[s_idx] = turbina_gemeo_digital._erro_balanco_energetico_w
                
                pot_disp_vento = 0.5 * DENSIDADE_AR_KG_M3 * (np.pi*RAIO_TURBINA_METROS**2) * (velocidade_vento_atual_m_s**3) if velocidade_vento_atual_m_s > 0 else 1.0
                registros.coeficientes_potencia_digitais[s_idx] = (turbina_gemeo_digital._potencia_gerada_w / pot_disp_vento)
                
                registros.idx_sim += 1
    
            estado_atual_coef_torque_twin = turbina_gemeo_digital.get_flag_torque_estatico()
            if estado_anterior_coef_torque_twin != estado_atual_coef_torque_twin:
                historico_eventos.append(f"[{tempo_decorrido:.3f}s] [TWIN] Coef. torque estático alterado para: {estado_atual_coef_torque_twin}")
            estado_anterior_coef_torque_twin = estado_atual_coef_torque_twin
    
            estado_atual_flag_vazio = turbina_gemeo_digital.get_flag_vazio()
            if estado_anterior_flag_vazio != estado_atual_flag_vazio:
                historico_eventos.append(f"[{tempo_decorrido:.3f}s] [TWIN] Flag vazio alterado para: {estado_atual_flag_vazio}")
            estado_anterior_flag_vazio = estado_atual_flag_vazio
    
            tempo_ultima_simulacao_digital += PASSO_SIMULACAO_SEGUNDOS
    
        # =========================================================================
        # MALHA DE HARDWARE: EMULADOR FÍSICO
        # =========================================================================
        if (tempo_atual - tempo_ultimo_controle_hardware) >= PASSO_HARDWARE_SEGUNDOS:
            tempo_inicio_comunicacao = time.perf_counter()
            torquimetro_fisico.ReadRaw() 
    
            torque_real_lido_nm           = torquimetro_fisico.Torque_calibrated 
            velocidade_angular_real_rad_s = torquimetro_fisico.RPM_calibrated * (2.0 * np.pi) / 60.0
            potencia_real_lida_w          = torquimetro_fisico.Potencia_calculated
    
            if potencia_real_lida_w >= POTENCIA_MECANICA_MAXIMA_W:
                historico_eventos.append(f"[{tempo_decorrido:.3f}s] [ALERTA FATAL] Limite de potência atingido na bancada!")
                break
    
            ANGULO_PAS_REFERENCIA_RADIANOS = 0.0
            torque_hss_referencia = turbina_do_emulador.calcular_torque_aerodinamico_no_gerador(
                            velocidade_vento_atual_m_s, 
                            velocidade_angular_real_rad_s / RELACAO_TRANSMISSAO_CAIXA_ENGRENAGENS,
                            ANGULO_PAS_REFERENCIA_RADIANOS)
            
            if torque_hss_referencia > 0.0:
                comando_velocidade_inversor = controlador_seguimento_torque.compute(torque_hss_referencia, torque_real_lido_nm, PASSO_HARDWARE_SEGUNDOS)
            else: 
                comando_velocidade_inversor = 0.0
            
            comando_velocidade_inversor_filtrado = aplicar_filtro_passa_baixa(comando_velocidade_inversor, comando_velocidade_inversor_filtrado, PASSO_HARDWARE_SEGUNDOS, CONSTANTE_TEMPO_FILTRO_U_INVERSOR)
            inversor_motor.SendReferenceAngularVelocity(comando_velocidade_inversor_filtrado)
    
            # Gravação de Registros HW
            if registros.idx_hw < registros.n_hw:
                h_idx = registros.idx_hw
                registros.tempos_hardware_fisico[h_idx] = tempo_decorrido
                registros.velocidades_vento_m_s_hw[h_idx] = velocidade_vento_atual_m_s
                
                registros.comandos_esforco_inversor[h_idx] = comando_velocidade_inversor_filtrado
                registros.velocidades_angulares_reais_rpm[h_idx] = torquimetro_fisico.RPM_calibrated
                
                rpm_ideal = ((velocidade_vento_atual_m_s * TSR_IDEAL / RAIO_TURBINA_METROS) * RELACAO_TRANSMISSAO_CAIXA_ENGRENAGENS) * 60.0 / (2.0 * np.pi)
                registros.velocidades_angulares_ideais_rpm[h_idx] = rpm_ideal
                registros.velocidades_angulares_referencia_rad_s[h_idx] = rpm_ideal * (2.0 * np.pi) / 60.0 
                registros.velocidades_angulares_reais_rad_s[h_idx] = velocidade_angular_real_rad_s
                
                registros.torques_medidos_sensor_nm[h_idx] = torque_real_lido_nm
                registros.torques_hss_referencia[h_idx] = torque_hss_referencia
                
                registros.potencias_medidas_sensor[h_idx] = potencia_real_lida_w
                registros.latencias_comunicacao_ms[h_idx] = (time.perf_counter() - tempo_inicio_comunicacao) * 1000.0
                
                pot_disp_vento_hw = 0.5 * DENSIDADE_AR_KG_M3 * (np.pi*RAIO_TURBINA_METROS**2) * (velocidade_vento_atual_m_s**3) if velocidade_vento_atual_m_s > 0 else 1.0
                registros.coeficientes_potencia_reais[h_idx] = (potencia_real_lida_w / pot_disp_vento_hw)
                
                registros.idx_hw += 1
    
            estado_atual_coef_torque_hw = turbina_do_emulador.get_flag_torque_estatico()
            if estado_anterior_coef_torque_hw and not estado_atual_coef_torque_hw:
                historico_eventos.append(f"[{tempo_decorrido:.3f}s] [HW] Coef. torque estático alterado para: {estado_atual_coef_torque_hw}")
            estado_anterior_coef_torque_hw = estado_atual_coef_torque_hw
    
            tempo_ultimo_controle_hardware = tempo_atual
            time.sleep(0.0001)

finally:
    inversor_motor.SendReferenceAngularVelocity(0)
    time.sleep(0.1)
    inversor_motor.StopMotor()
    plt.ioff()
    
    print("\n" + "="*55)
    print("=== HISTÓRICO DE EVENTOS DO SISTEMA (LOG TEMPORAL) ===")
    print("="*55)
    if not historico_eventos:
        print("Nenhum evento crítico registrado durante a emulação.")
    else:
        for evento in historico_eventos:
            print(evento)
    print("="*55 + "\n")
    
    print("Emulação Finalizada. Gerando gráficos comparativos de análise...")
    plotar_graficos_analise_final(registros)