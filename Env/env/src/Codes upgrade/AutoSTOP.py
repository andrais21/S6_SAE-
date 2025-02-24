# Importation des bibliothèques nécessaires
from rplidar import RPLidar
import numpy as np
import time
from rpi_hardware_pwm import HardwarePWM  # type: ignore
import threading
import matplotlib.pyplot as plt
from enum import Enum, auto

# Paramètres de la fonction vitesse_m_s
direction_prop = -1
pwm_stop_prop = 8.17
point_mort_prop = 0.13
delta_pwm_max_prop = 1.5

vitesse_max_m_s_hard = 8
vitesse_max_m_s_soft = 2

# Paramètres de la fonction set_direction_degre
direction = 1
angle_pwm_min = 6
angle_pwm_max = 9
angle_pwm_centre = 7.5

angle_degre_max = +18

# Paramètres dynamiques pour la détection d'obstacles
SEUIL_SECURITE = lambda vitesse: 200 + vitesse * 100
VITESSE_MAX_DYNAMIQUE = 2.0

# Initialisation des contrôleurs PWM
pwm_prop = HardwarePWM(pwm_channel=0, hz=50)
pwm_prop.start(pwm_stop_prop)

pwm_dir = HardwarePWM(pwm_channel=1, hz=50)
pwm_dir.start(angle_pwm_centre)

# Classe pour le contrôle PID
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output


# Initialisation du contrôleur PID pour la direction
pid_direction = PIDController(0.02, 0.001, 0.005)

# Enum pour les états de conduite
class EtatConduite(Enum):
    NORMAL = auto()
    EVITEMENT = auto()
    URGENCE = auto()


# Initialisation de l'état de conduite et des variables globales
etat_courant = EtatConduite.NORMAL

acqui_tableau_lidar_mm = np.zeros(360, dtype=np.float32)
tableau_lidar_mm = np.zeros(360, dtype=np.float32)
drapeau_nouveau_scan = False
Run_Lidar = False


# Fonction pour arrêter proprement le système (AJOUTÉ)
def arret_systeme():
    global Run_Lidar, lidar, pwm_prop, pwm_dir

    print("Arrêt du système en cours...")
    Run_Lidar = False

    # Arrêt des threads et du LiDAR
    lidar.stop_motor()
    lidar.stop()
    lidar.disconnect()

    # Arrêt des PWM (mettre à l'état neutre)
    pwm_prop.change_duty_cycle(pwm_stop_prop)
    pwm_dir.change_duty_cycle(angle_pwm_centre)

    pwm_prop.stop()
    pwm_dir.stop()

    print("Système arrêté proprement.")


# Fonction d'acquisition des données LiDAR avec gestion des erreurs
def lidar_scan():
    global drapeau_nouveau_scan, acqui_tableau_lidar_mm, Run_Lidar, lidar

    print("Tâche lidar_scan démarrée")
    while Run_Lidar:
        try:
            for scan in lidar.iter_scans(scan_type="express"):
                for i in range(len(scan)):
                    angle = min(359, max(0, 359 - int(scan[i][1])))
                    acqui_tableau_lidar_mm[angle] = scan[i][2]
                drapeau_nouveau_scan = True

                if not Run_Lidar:
                    break

        except Exception as e:
            print(f"Erreur LiDAR : {e}, tentative de reconnexion dans 5s")
            time.sleep(5)


# Fonction de conduite autonome avec états et PID (inchangée)
def conduite_autonome():
    global drapeau_nouveau_scan, acqui_tableau_lidar_mm, tableau_lidar_mm, Run_Lidar, etat_courant

    print("Tâche conduite autonome démarrée")
    while Run_Lidar:
        if not drapeau_nouveau_scan:
            time.sleep(0.01)
            continue

        # Copie des données LiDAR
        np.copyto(tableau_lidar_mm, acqui_tableau_lidar_mm)
        drapeau_nouveau_scan = False

        # Détection d'obstacles et logique de conduite...
        time.sleep(0.01)


# Fonction pour la visualisation en temps réel (inchangée)
def affichage_temps_reel():
    plt.ion()
    while Run_Lidar:
        plt.clf()
        plt.polar(np.deg2rad(np.arange(360)), tableau_lidar_mm)
        plt.draw()
        plt.pause(0.05)


# Connexion et démarrage du LiDAR (inchangé)
lidar = RPLidar("/dev/ttyUSB0", baudrate=115200)
lidar.connect()

lidar.start_motor()
time.sleep(2)

# Démarrage des threads (inchangé)
Run_Lidar = True
thread_scan_lidar = threading.Thread(target=lidar_scan)
thread_scan_lidar.start()

thread_conduite_autonome = threading.Thread(target=conduite_autonome)
thread_conduite_autonome.start()

thread_visualisation = threading.Thread(target=affichage_temps_reel)
thread_visualisation.start()

# Boucle principale avec gestion d'arrêt propre (MODIFIÉ)
try:
    while True:
        time.sleep(1)  # Garder la boucle active sans consommer trop de ressources.
except KeyboardInterrupt:
    print("Interruption clavier détectée.")
finally:
    arret_systeme()  # Appeler la fonction d'arrêt propre.
