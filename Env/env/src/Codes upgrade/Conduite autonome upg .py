# Importation des bibliothèques nécessaires
from rplidar import RPLidar
import numpy as np
import time
from rpi_hardware_pwm import HardwarePWM  # type: ignore
import threading
import matplotlib.pyplot as plt
from enum import Enum, auto

# --- Paramètres globaux ---
PORT_NAME = "/dev/ttyUSB0"  # Port série du LiDAR
BAUDRATE = 256000           # Vitesse en bauds
Run_Lidar = True            # Drapeau pour arrêter proprement les threads

# Paramètres PWM pour vitesse
direction_prop = -1
pwm_stop_prop = 8.17
point_mort_prop = 0.13
delta_pwm_max_prop = 1.5

vitesse_max_m_s_hard = 8.0  # Vitesse maximale physique
vitesse_max_m_s_soft = 2.0  # Vitesse maximale souhaitée

# Paramètres PWM pour direction
direction = 1
angle_pwm_min = 6.0
angle_pwm_max = 9.0
angle_pwm_centre = 7.5

angle_degre_max = +18

# Plage d'angles pour détecter les obstacles devant (par exemple, -30° à +30°)
ANGLE_MIN_FRONT = 330  # Correspond à -30° (dans un tableau de 0 à 359)
ANGLE_MAX_FRONT = 30   # Correspond à +30°

# Initialisation des tableaux LiDAR
tableau_lidar_mm = np.zeros(360, dtype=np.float32)
acqui_tableau_lidar_mm = np.zeros(360, dtype=np.float32)
drapeau_nouveau_scan = False

# Initialisation des contrôleurs PWM pour vitesse et direction
pwm_prop = HardwarePWM(pwm_channel=0, hz=50)
pwm_dir = HardwarePWM(pwm_channel=1, hz=50)

pwm_prop.start(pwm_stop_prop)
pwm_dir.start(angle_pwm_centre)

# --- Classe PIDController ---
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0.0
        self.integral = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output

pid_direction = PIDController(0.02, 0.001, 0.005)

# --- États de conduite ---
class EtatConduite(Enum):
    NORMAL = auto()
    EVITEMENT = auto()
    URGENCE = auto()

etat_courant = EtatConduite.NORMAL

# --- Fonctions PWM ---
def set_vitesse_m_s(vitesse_m_s):
    if vitesse_m_s > vitesse_max_m_s_soft:
        vitesse_m_s = vitesse_max_m_s_soft
    elif vitesse_m_s < -vitesse_max_m_s_hard:
        vitesse_m_s = -vitesse_max_m_s_hard
    
    if vitesse_m_s == 0:
        pwm_prop.change_duty_cycle(pwm_stop_prop)
    elif vitesse_m_s > 0:
        vitesse_pwm = vitesse_m_s * delta_pwm_max_prop / vitesse_max_m_s_hard
        pwm_prop.change_duty_cycle(pwm_stop_prop + direction_prop * (point_mort_prop + vitesse_pwm))
    elif vitesse_m_s < 0:
        vitesse_pwm = abs(vitesse_m_s) * delta_pwm_max_prop / vitesse_max_m_s_hard
        pwm_prop.change_duty_cycle(pwm_stop_prop - direction_prop * (point_mort_prop + vitesse_pwm))

def set_direction_degre(angle_degre):
    angle_pwm = angle_pwm_centre + direction * (angle_pwm_max - angle_pwm_min) * angle_degre / (2 * angle_degre_max)
    angle_pwm = max(min(angle_pwm, angle_pwm_max), angle_pwm_min)
    pwm_dir.change_duty_cycle(angle_pwm)

# --- Fonction d'acquisition des données LiDAR ---
def lidar_scan():
    global drapeau_nouveau_scan, acqui_tableau_lidar_mm, Run_Lidar
    
    try:
        lidar = RPLidar(PORT_NAME, baudrate=BAUDRATE)
        lidar.connect()
        lidar.start_motor()
        
        while Run_Lidar:
            for scan in lidar.iter_scans(scan_type="express"):
                if not Run_Lidar:
                    break
                
                for i in range(len(scan)):
                    angle = min(359, max(0, int(scan[i][1])))
                    acqui_tableau_lidar_mm[angle] = scan[i][2]
                
                drapeau_nouveau_scan = True

    except Exception as e:
        print(f"Erreur LiDAR : {e}")
    
    finally:
        lidar.stop_motor()
        lidar.stop()
        lidar.disconnect()

# --- Fonction de conduite autonome ---
def conduite_autonome():
    global drapeau_nouveau_scan, tableau_lidar_mm, Run_Lidar
    
    while Run_Lidar:
        if not drapeau_nouveau_scan:
            time.sleep(0.01)
            continue
        
        np.copyto(tableau_lidar_mm, acqui_tableau_lidar_mm)
        drapeau_nouveau_scan = False
        
        # Limiter l'analyse aux angles correspondant à l'avant du véhicule (-30° à +30°)
        distances_frontales = np.concatenate((tableau_lidar_mm[ANGLE_MIN_FRONT:], tableau_lidar_mm[:ANGLE_MAX_FRONT+1]))
        
        # Détection d'obstacles uniquement devant le véhicule
        distance_min_avant = np.min(distances_frontales[distances_frontales > 0])
        
        if distance_min_avant < 500:  # Si obstacle proche devant (<50 cm)
            print("Obstacle détecté devant ! Arrêt.")
            set_vitesse_m_s(0)
            continue
        
        # Contrôle PID pour ajuster la direction en fonction des données frontales (-30° à +30°)
        erreur_direction_gauche_droite = tableau_lidar_mm[60] - tableau_lidar_mm[-60]
        angle_degre_cible = pid_direction.update(erreur_direction_gauche_droite, 0.01)
        
        set_direction_degre(angle_degre_cible)
        
        # Ajustement dynamique de la vitesse en fonction de la distance minimale détectée devant
        vitesse_cible = min(vitesse_max_m_s_soft, distance_min_avant / 1000)  # Limiter à max soft
        
        set_vitesse_m_s(vitesse_cible)

# --- Fonction principale ---
def main():
    global Run_Lidar

    try:
        thread_scan_lidar = threading.Thread(target=lidar_scan)
        thread_conduite_autonome = threading.Thread(target=conduite_autonome)
        
        thread_scan_lidar.start()
        thread_conduite_autonome.start()

    except KeyboardInterrupt:
        print("Arrêt demandé par l'utilisateur.")
    
    finally:
        Run_Lidar = False
        thread_scan_lidar.join()
        thread_conduite_autonome.join()

if __name__ == "__main__":
    main()
