from rplidar import RPLidar
import time
from rpi_hardware_pwm import HardwarePWM
import socket
import threading

# Paramètres de la fonction vitesse_m_s
direction_prop = 1  # -1 pour les variateurs inversés ou un petit rapport correspond à une marche avant
pwm_stop_prop = 7.3
point_mort_prop = 0.4  
delta_pwm_max_prop = 1.5     # pwm à laquelle on atteint la vitesse maximale
vitesse_max_m_s_hard = 8  # vitesse que peut atteindre la voiture
vitesse_max_m_s_soft = 1   # vitesse maximale que l'on souhaite atteindre

# Paramètres de la fonction set_direction_degre
direction = 1  # 1 pour angle_pwm_min à gauche, -1 pour angle_pwm_min à droite
angle_pwm_min = 2   # min
angle_pwm_max = 10   # max
angle_pwm_centre = 6

angle_degre_max = +25  # vers la gauche
angle_degre = 0

pwm_prop = HardwarePWM(pwm_channel=0, hz=50)
pwm_prop.start(pwm_stop_prop)

def set_vitesse_m_s(vitesse_m_s):
    if vitesse_m_s > vitesse_max_m_s_soft:
        vitesse_m_s = vitesse_max_m_s_soft
    elif vitesse_m_s < -vitesse_max_m_s_hard:
        vitesse_m_s = -vitesse_max_m_s_hard
    if vitesse_m_s == 0:
        pwm_prop.change_duty_cycle(pwm_stop_prop)
    elif vitesse_m_s > 0:
        vitesse = vitesse_m_s * (delta_pwm_max_prop) / vitesse_max_m_s_hard
        pwm_prop.change_duty_cycle(pwm_stop_prop + direction_prop * (point_mort_prop + vitesse))
    elif vitesse_m_s < 0:
        vitesse = vitesse_m_s * (delta_pwm_max_prop) / vitesse_max_m_s_hard
        pwm_prop.change_duty_cycle(pwm_stop_prop - direction_prop * (point_mort_prop - vitesse))

def recule():
    set_vitesse_m_s(-vitesse_max_m_s_hard)
    time.sleep(0.2)
    set_vitesse_m_s(0)
    time.sleep(0.2)
    set_vitesse_m_s(-1)

pwm_dir = HardwarePWM(pwm_channel=1, hz=50)
pwm_dir.start(angle_pwm_centre)

def set_direction_degre(angle_degre):
    global angle_pwm_min
    global angle_pwm_max
    global angle_pwm_centre
    angle_pwm = angle_pwm_centre + direction * (angle_pwm_max - angle_pwm_min) * angle_degre / (2 * angle_degre_max)
    if angle_pwm > angle_pwm_max:
        angle_pwm = angle_pwm_max
    if angle_pwm < angle_pwm_min:
        angle_pwm = angle_pwm_min
    pwm_dir.change_duty_cycle(angle_pwm)

# Connexion et démarrage du LIDAR
lidar = RPLidar("/dev/ttyUSB0", baudrate=256000)
lidar.connect()
print(lidar.get_info())
lidar.start_motor()
time.sleep(1)

tableau_lidar_mm = [0] * 360  # Création d'un tableau de 360 zéros

# Fonction pour gérer les commandes à distance
def handle_commands(conn):
    global direction, vitesse_max_m_s_soft
    while True:
        try:
            command = conn.recv(1024).decode('utf-8').strip()
            if command:
                if command == 'w':  # Avancer
                    set_vitesse_m_s(vitesse_max_m_s_soft)
                elif command == 's':  # Reculer
                    recule()
                elif command == 'a':  # Tourner à gauche
                    direction = 1
                    set_direction_degre(angle_degre_max)
                elif command == 'd':  # Tourner à droite
                    direction = -1
                    set_direction_degre(angle_degre_max)
                elif command == 'x':  # Arrêter
                    set_vitesse_m_s(0)
                elif command == 'o' : # centrer direction
                    set_direction_degre(0)
                elif command == 'q':  # Quitter
                    break
        except Exception as e:
            print(f"[ERREUR] {e}")
            break

# Création de la connexion socket
try:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(('192.168.2.148', 12345))  # Écoute sur toutes les interfaces
        s.listen()
        print("[INFO] En attente de connexion...")
        conn, addr = s.accept()
        print(f"[INFO] Connexion établie avec {addr}")

        # Démarrer le thread pour gérer les commandes
        command_thread = threading.Thread(target=handle_commands, args=(conn,), daemon=True)
        command_thread.start()

        # Boucle principale pour le LIDAR
        for scan in lidar.iter_scans():
            for i in range(len(scan)):
                angle = min(359, max(0, 359 - int(scan[i][1])))  # scan[i][1] : angle
                tableau_lidar_mm[angle] = scan[i][2]  # scan[i][2] : distance

except KeyboardInterrupt:  # Récupération du CTRL+C
    print("Fin des acquisitions")

# Arrêt et déconnexion du LIDAR et des moteurs
lidar.stop_motor()
lidar.stop()
time.sleep(1)
lidar.disconnect()
pwm_dir.stop()
pwm_prop.change_duty_cycle(pwm_stop_prop)
