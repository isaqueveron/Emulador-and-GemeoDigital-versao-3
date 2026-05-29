import time
import numpy as np
import matplotlib.pyplot as plt

from parametros import *
from modelo_aerogerador import TurbinaVirtual
from driver_torquimetro import Torquimeter
from driver_inversor_cfw11 import Inverter
from pid_module import PIDController 
from init_serial_devices import selecionar_porta

class RegistroDeHardware:
    def __init__(self, duracao_s, passo_hardware):
        self.n_hw = int(duracao_s / passo_hardware) + 1
        self.idx_hw = 0

        self.tempos_hardware_fisico = np.zeros(self.n_hw)
        self.velocidades_vento_m_s = np.zeros(self.n_hw) # Corrigido: Vetor de vento adicionado ao registro
        self.comandos_esforco_inversor = np.zeros(self.n_hw)
        self.velocidades_angulares_reais_rpm = np.zeros(self.n_hw)
        
        self.velocidades_angulares_ideais_rpm = np.zeros(self.n_hw)
        self.velocidades_angulares_reais_rpm = np.zeros(self.n_hw)
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
    if tempo_decorrido_segundos < 60.0: return 3.0
    if tempo_decorrido_segundos < 70.0: return 2.0
    if tempo_decorrido_segundos < 80.0: return 1.0
    if tempo_decorrido_segundos < 90.0: return 0.0
    return 0.0

def calcular_velocidade_vento_rampa(tempo):
    # Rampa de subida (de 0 a 50 segundos)
    # Sobe de 0.0 até 4.0
    if 0.0 <= tempo < 50.0:
        return (4.0 / 50.0) * tempo
    
    # Rampa de descida (de 50 a 100 segundos)
    # Desce de 4.0 até 0.0
    elif 50.0 <= tempo < 100.0:
        return 4.0 - ((4.0 / 50.0) * (tempo - 50.0))
    
    # Fora do intervalo
    return 0.0

def aplicar_filtro_passa_baixa(valor_atual, valor_anterior_filtrado, passo_tempo, constante_tempo):
    fator_suavizacao = passo_tempo / (constante_tempo + passo_tempo)
    return fator_suavizacao * valor_atual + (1.0 - fator_suavizacao) * valor_anterior_filtrado

def configurar_interface_tempo_real():
    plt.ion() 
    figura, (eixo_velocidade, eixo_potencia) = plt.subplots(2, 1, figsize=(10, 8))
    figura.canvas.manager.set_window_title('Monitoramento em Tempo Real - Bancada de Emulação')

    linha_vel_referencia, = eixo_velocidade.plot([], [], 'r--', label=f'Velocidade Ideal (TSR={TSR_IDEAL})')
    linha_vel_real, = eixo_velocidade.plot([], [], 'lightgray',alpha=0.7,linewidth=1.2, label='Velocidade Real (Eixo)')
    eixo_velocidade.set_ylabel("Velocidade Angular [RPM]")
    eixo_velocidade.legend(loc='upper right')
    eixo_velocidade.grid(True)

    linha_potencia_real, = eixo_potencia.plot([], [], 'lightgray',alpha=0.7,linewidth=1.2, label='Potência Eixo Real')
    eixo_potencia.set_xlabel("Tempo [s]")
    eixo_potencia.set_ylabel("Potência [W]")
    eixo_potencia.legend(loc='upper right')
    eixo_potencia.grid(True)

    visor_digital = eixo_potencia.text(0.5, 0.85, '', transform=eixo_potencia.transAxes, 
                                       fontsize=16, fontweight='bold', color='darkgreen',
                                       ha='center', bbox=dict(facecolor='white', alpha=0.8, edgecolor='green'))
    plt.tight_layout()
    return figura, eixo_velocidade, eixo_potencia, linha_vel_referencia, linha_vel_real, linha_potencia_real, visor_digital

def plotar_graficos_analise_final(registros, nome_ensaio=f"ensaio_{PERFIL_VENTO}"):
    from scipy.signal import butter, filtfilt

    h = registros.idx_hw
    t = registros.tempos_hardware_fisico[:h]

    if h > 0:
        fs = 1.0 / PASSO_HARDWARE_SEGUNDOS
        nyq = 0.5 * fs
        
        fc = 1.0 / (2 * np.pi * 0.1)
        wn = min(fc / nyq, 0.99) 
        b, a = butter(2, wn, btype='low') 
        
        torques_filtrados_plot = filtfilt(b, a, registros.torques_medidos_sensor_nm[:h])
        potencias_filtradas_plot = filtfilt(b, a, registros.potencias_medidas_sensor[:h])
        cp_filtrado_plot = filtfilt(b, a, registros.coeficientes_potencia_reais[:h])
    else:
        torques_filtrados_plot = np.zeros(0)
        potencias_filtradas_plot = np.zeros(0)
        cp_filtrado_plot = np.zeros(0)

    plt.style.use('default')

    # -------------------------------------------------------------
    # 1. FIGURA: DADOS DE ENTRADA E CINEMÁTICA
    # -------------------------------------------------------------
    fig1, axs1 = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig1.canvas.manager.set_window_title('Emulador em Hardware - Condições e Cinemática')
    fig1.suptitle('Condições de Entrada e Cinemática do Sistema Físico', fontsize=14, fontweight='bold')

    axs1[0].plot(t, registros.velocidades_vento_m_s[:h], color='dodgerblue', lw=2, label='Vento Input')
    axs1[0].set_ylabel('Vento [m/s]')
    axs1[0].legend(loc='upper right'); axs1[0].grid(True, ls='--')

    axs1[1].plot(t, registros.velocidades_angulares_reais_rpm[:h], color='purple', lw=2, label='Velocidade Angular Real (Sensor)')
    axs1[1].plot(t, registros.velocidades_angulares_ideais_rpm[:h], color='lightcoral', lw=2, alpha=0.7, ls='--', label=f'Velocidade Ideal (TSR={TSR_IDEAL})')
    axs1[1].set_ylabel('Rotação [RPM]')
    axs1[1].set_xlabel('Tempo [s]')
    axs1[1].legend(loc='upper right'); axs1[1].grid(True, ls='--')
    plt.tight_layout()
    
    # Salva o Gráfico 1
    fig1.savefig(f"{nome_ensaio}_condicoes_cinematica.svg", format='svg', bbox_inches='tight')

    # -------------------------------------------------------------
    # 2. FIGURA: DINÂMICA DE TORQUES E ESFORÇO DE CONTROLE
    # -------------------------------------------------------------
    fig2, axs2 = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig2.canvas.manager.set_window_title('Emulador em Hardware - Dinâmica de Torques')
    fig2.suptitle('Seguimento de Referência Aerodinâmica (Controle de Torque)', fontsize=14, fontweight='bold')

    axs2[0].plot(t, registros.torques_medidos_sensor_nm[:h], color='lightgray', alpha=0.7, lw=2, label='Torque Medido (Sensor Bruto)')
    axs2[0].plot(t, registros.torques_hss_referencia[:h], color='forestgreen', lw=2, label='Torque Aerodinâmico Ref. (Calculado do Modelo)')
    axs2[0].plot(t, torques_filtrados_plot, color='black', lw=1.5, ls='--', label='Torque Medido (Sensor Filtrado)')
    axs2[0].set_ylabel('Torque [N.m]')
    axs2[0].legend(loc='upper right'); axs2[0].grid(True, ls='--')

    axs2[1].plot(t, registros.comandos_esforco_inversor[:h], color='blue', lw=2, label='Ref. Velocidade para o controle malha aberta V/f do Inversor')
    axs2[1].set_ylabel('Ref Inversor [RPM]')
    axs2[1].set_xlabel('Tempo [s]')
    axs2[1].legend(loc='upper right'); axs2[1].grid(True, ls='--')
    plt.tight_layout()
    
    # Salva o Gráfico 2
    fig2.savefig(f"{nome_ensaio}_dinamica_torques.svg", format='svg', bbox_inches='tight')

    # -------------------------------------------------------------
    # 3. FIGURA: POTÊNCIA E EFICIÊNCIA
    # -------------------------------------------------------------
    fig3, axs3 = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig3.canvas.manager.set_window_title('Emulador em Hardware - Potência e Eficiência')
    fig3.suptitle('Geração de Potência e Coeficiente Aerodinâmico Real', fontsize=14, fontweight='bold')

    axs3[0].plot(t, registros.potencias_medidas_sensor[:h], color='lightgray', alpha=0.7, lw=2, label='Potência Eixo (Bruta Sensor)')
    axs3[0].plot(t, potencias_filtradas_plot, color='green', lw=2, label='Potência Eixo (Filtrada Sensor)')
    axs3[0].set_ylabel('Potência [W]')
    axs3[0].legend(loc='upper right'); axs3[0].grid(True, ls='--')

    axs3[1].plot(t, registros.coeficientes_potencia_reais[:h], color='lightgray', alpha=0.7, lw=2, label='Cp Bruto')
    axs3[1].plot(t, cp_filtrado_plot, color='purple', lw=2, label='Cp Filtrado')
    axs3[1].axhline(y=0.593, color='r', linestyle='--', zorder=4, label='Limite de Betz (0.593)')
    axs3[1].set_ylim(0, 0.7)
    axs3[1].set_ylabel('Cp [-]')
    axs3[1].set_xlabel('Tempo [s]')
    axs3[1].legend(loc='upper right'); axs3[1].grid(True, ls='--')
    plt.tight_layout()
    
    # Salva o Gráfico 3
    fig3.savefig(f"{nome_ensaio}_potencia_eficiencia.svg", format='svg', bbox_inches='tight')

    # -------------------------------------------------------------
    # 4. FIGURA: PERFORMANCE DE HARDWARE E COMUNICAÇÃO
    # -------------------------------------------------------------
    fig4, ax4 = plt.subplots(figsize=(12, 4))
    fig4.canvas.manager.set_window_title('Emulador em Hardware - Latência')
    
    ax4.plot(t, registros.latencias_comunicacao_ms[:h], color='teal', alpha=0.7, lw=1.5, label='Latência Ciclo Serial')
    ax4.axhline(y=PASSO_HARDWARE_SEGUNDOS*1000, color='r', linestyle='--', zorder=4, label=f"Limite Hard Real-Time ({PASSO_HARDWARE_SEGUNDOS*1000} ms)")
    
    ax4.set_title('Performance da Comunicação Serial', fontsize=14, fontweight='bold')
    ax4.set_ylabel('Latência [ms]')
    ax4.set_xlabel('Tempo [s]')
    ax4.legend(loc='upper right'); ax4.grid(True, ls='--')
    plt.tight_layout()
    
    # Salva o Gráfico 4
    fig4.savefig(f"{nome_ensaio}_performance_latencia.svg", format='svg', bbox_inches='tight')

    plt.show()

# Usado APENAS para calcular a referência de torque a ser enviada pelo vento
turbina_do_emulador = TurbinaVirtual(
    constante_velocidade=CONSTANTE_VELOCIDADE_GERADOR_V_RAD_S, 
    offset_fem=OFFSET_FEM,
    ganho_armadura=GANHO_ARMADURA, 
    tau_armadura=TAU_ARMADURA,
    resistencia_interna=RESISTENCIA_INTERNA_OHMS,
    raio_turbina=RAIO_TURBINA_METROS, 
    inercia_turbina=0, 
    inercia_gerador=INERCIA_GERADOR_KG_M2, 
    coeficiente_atrito_turbina=0,
    coeficiente_atrito_gerador = COEFICIENTE_ATRITO_GERADOR,
    taxa_variacao_tensao_barramento=TAXA_VARIACAO_TENSAO_BARRAMENTO_V_S, 
    taxa_variacao_rpm_maxima=TAXA_VARIACAO_RPM_MAX,
    velocidade_angular_maxima_pas=VELOCIDADE_MAXIMA_PAS_GRAUS_S, 
    torque_maximo_motor=TORQUE_MAXIMO_MOTOR_NM,
    tsr_ideal=TSR_IDEAL,
    relacao_transmissao_caixa_engrenagens=RELACAO_TRANSMISSAO_CAIXA_ENGRENAGENS, 
    modelo_coeficiente_potencia=0,
    constante_tempo_filtro_forcaeletromotriz=CONSTANTE_TEMPO_FILTRO_FORCAELETROMOTRIZ,
    densidade_ar_kg_m3=DENSIDADE_AR_KG_M3
)

torquimetro_fisico = Torquimeter(Port=selecionar_porta("Torquimetro"), Baudrate=230400, Timeout=0.003) 
inversor_motor = Inverter(Port=selecionar_porta("Inversor"), ADR=1, Baudrate=57600)

inversor_motor.ActivateMotor()
time.sleep(0.1)
inversor_motor.SendReferenceAngularVelocity(0)
time.sleep(1)

controlador_seguimento_torque = PIDController(kp=500.0, ki=300.0, kd=0.0, out_min=0.0, out_max=1000.0) 

registros = RegistroDeHardware(TEMPO_TOTAL_EMULACAO_SEGUNDOS, PASSO_HARDWARE_SEGUNDOS)
#figura, eixo_velocidade, eixo_potencia, linha_vel_referencia, linha_vel_real, linha_potencia_real, visor_digital = configurar_interface_tempo_real()

velocidade_vento_atual_m_s = 0.0
comando_velocidade_inversor_filtrado = 0.0
index_vetor_vento = 0
estado_anterior_coef_torque = True


tempo_inicio_absoluto = time.perf_counter()
tempo_ultimo_controle_hardware = tempo_inicio_absoluto
tempo_ultima_simulacao_digital = tempo_inicio_absoluto
tempo_ultima_atualizacao_graficos = tempo_inicio_absoluto


print(f"Emulação de Hardware Iniciada por {TEMPO_TOTAL_EMULACAO_SEGUNDOS}s...")

try:
    while (time.perf_counter() - tempo_inicio_absoluto) < TEMPO_TOTAL_EMULACAO_SEGUNDOS:
        tempo_atual = time.perf_counter()
        tempo_decorrido = tempo_atual - tempo_inicio_absoluto
           

        if PERFIL_VENTO == 'NATURAL': 
            if (tempo_atual - tempo_ultima_simulacao_digital) >= PASSO_SIMULACAO_SEGUNDOS:
                try: 
                    velocidade_vento_atual_m_s = VETOR_VENTO_M_S[index_vetor_vento]
                    index_vetor_vento += 1
                except: 
                    velocidade_vento_atual_m_s = VETOR_VENTO_M_S[index_vetor_vento - 1]
                    index_vetor_vento = 100
                tempo_ultima_simulacao_digital += PASSO_SIMULACAO_SEGUNDOS

        if (tempo_atual - tempo_ultimo_controle_hardware) >= PASSO_HARDWARE_SEGUNDOS:
            tempo_inicio_comunicacao = time.perf_counter()

            if PERFIL_VENTO == 'CONSTANTE':
                if tempo_decorrido < 10.0:
                    velocidade_vento_atual_m_s = vento_constante * (tempo_decorrido / 10.0)
                else:
                    velocidade_vento_atual_m_s = vento_constante            
            if PERFIL_VENTO == 'ESCADA' : velocidade_vento_atual_m_s = calcular_velocidade_vento_degraus(tempo_decorrido)
            if PERFIL_VENTO == 'RAMPA'  : velocidade_vento_atual_m_s = calcular_velocidade_vento_rampa(tempo_decorrido)
                
            torquimetro_fisico.ReadRaw() 

            torque_real_lido_nm             = torquimetro_fisico.Torque_calibrated 
            velocidade_angular_real_rad_s   = torquimetro_fisico.RPM_calibrated * (2.0 * np.pi) / 60.0
            potencia_real_lida_w            = torquimetro_fisico.Potencia_calculated

            if potencia_real_lida_w >= POTENCIA_MECANICA_MAXIMA_W:
                inversor_motor.SendReferenceAngularVelocity(0)
                time.sleep(0.1)
                inversor_motor.StopMotor()
                plt.ioff()
                print("Limite de potência atingido na bancada!")
                break

            # -------------------------------------------------------------------------
            # CONTROLE FORÇA DO VENTO - EMULAÇÃO DO TORQUE AERODINÂMICO NO ROTOR
            # ------------------------------------------------------------------------- 
            ANGULO_PAS_REFERENCIA_RADIANOS = 0.0

            torque_hss_referencia = turbina_do_emulador.calcular_torque_aerodinamico_no_gerador(
                            velocidade_vento_atual_m_s,velocidade_angular_real_rad_s/RELACAO_TRANSMISSAO_CAIXA_ENGRENAGENS,
                            ANGULO_PAS_REFERENCIA_RADIANOS)
            
            if torque_hss_referencia > 0.0:
                comando_velocidade_inversor = controlador_seguimento_torque.compute(torque_hss_referencia, 
                                                                                torque_real_lido_nm, 
                                                                                PASSO_HARDWARE_SEGUNDOS)
            else: comando_velocidade_inversor = 0.0
            
            comando_velocidade_inversor_filtrado = aplicar_filtro_passa_baixa(comando_velocidade_inversor, comando_velocidade_inversor_filtrado, PASSO_HARDWARE_SEGUNDOS, CONSTANTE_TEMPO_FILTRO_U_INVERSOR)
            
            inversor_motor.SendReferenceAngularVelocity(comando_velocidade_inversor_filtrado)
            
            # Velocidade ideal calculada por TSR apenas para o gráfico de acompanhamento
            velocidade_angular_ideal_rad_s = (velocidade_vento_atual_m_s * TSR_IDEAL / RAIO_TURBINA_METROS) * RELACAO_TRANSMISSAO_CAIXA_ENGRENAGENS

            if registros.idx_hw < registros.n_hw:
                registros.tempos_hardware_fisico[registros.idx_hw] = tempo_decorrido
                registros.velocidades_vento_m_s[registros.idx_hw] = velocidade_vento_atual_m_s # Corrigido: Gravando o log de vento
                
                registros.comandos_esforco_inversor[registros.idx_hw] = comando_velocidade_inversor
                registros.velocidades_angulares_reais_rpm[registros.idx_hw] = torquimetro_fisico.RPM_calibrated
                
                registros.velocidades_angulares_ideais_rpm[registros.idx_hw] = velocidade_angular_ideal_rad_s / ((2.0 * np.pi) / 60.0)
                registros.velocidades_angulares_reais_rpm[registros.idx_hw] = velocidade_angular_real_rad_s / ((2.0 * np.pi) / 60.0)
                registros.torques_medidos_sensor_nm[registros.idx_hw] = torque_real_lido_nm
                registros.torques_hss_referencia[registros.idx_hw] = torque_hss_referencia
                
                registros.potencias_medidas_sensor[registros.idx_hw] = potencia_real_lida_w
                registros.latencias_comunicacao_ms[registros.idx_hw] = (time.perf_counter() - tempo_inicio_comunicacao) * 1000.0
                
                potencia_disponivel_vento_hardware = 0.5 * DENSIDADE_AR_KG_M3 * (np.pi*RAIO_TURBINA_METROS**2) * (velocidade_vento_atual_m_s**3) if velocidade_vento_atual_m_s > 0 else 1.0
                registros.coeficientes_potencia_reais[registros.idx_hw] = (potencia_real_lida_w / potencia_disponivel_vento_hardware)
                
                registros.idx_hw += 1

            tempo_ultimo_controle_hardware = tempo_atual
        
        estado_atual_coef_torque = turbina_do_emulador.get_flag_torque_estatico()
        if estado_anterior_coef_torque and not estado_atual_coef_torque:
            print(f"Transição detectada! Sistema operando com coef. torque estatico?: {estado_atual_coef_torque}. Tempo: {tempo_decorrido:.3f} s")
        estado_anterior_coef_torque = estado_atual_coef_torque

        """if (tempo_atual - tempo_ultima_atualizacao_graficos) >= TAXA_ATUALIZACAO_GRAFICOS_SEGUNDOS:
            linha_vel_referencia.set_data(registros.tempos_hardware_fisico[:registros.idx_hw], 
                                          registros.velocidades_angulares_ideais_rpm[:registros.idx_hw])
            
            linha_vel_real.set_data(registros.tempos_hardware_fisico[:registros.idx_hw], 
                                    registros.velocidades_angulares_reais_rpm[:registros.idx_hw])
            
            linha_potencia_real.set_data(registros.tempos_hardware_fisico[:registros.idx_hw], 
                                         registros.potencias_medidas_sensor[:registros.idx_hw])
            
            if registros.idx_hw > 0:
                valor_potencia_atual = registros.potencias_medidas_sensor[registros.idx_hw - 1]
                visor_digital.set_text(f'POTÊNCIA NO EIXO: {valor_potencia_atual:.2f} W')
            
            for eixo in [eixo_velocidade, eixo_potencia]:
                eixo.relim()
                eixo.autoscale_view()
            
            plt.pause(0.001)
            tempo_ultima_atualizacao_graficos = tempo_atual"""

finally:
    inversor_motor.SendReferenceAngularVelocity(0)
    time.sleep(0.1)
    inversor_motor.StopMotor()
    plt.ioff()
    print("Emulação Finalizada. Gerando gráficos de análise...")
    plotar_graficos_analise_final(registros)