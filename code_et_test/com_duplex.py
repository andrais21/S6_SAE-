#!/usr/bin/env python3
"""
Script sur Raspberry Pi :
  - Récupère les données du LIDAR.
  - Envoie les données via une connexion réseau en format JSON.
  - Reçoit et exécute des commandes de contrôle.
"""

import time
import json
import socket
import threading
from rplidar import RPLidar
import numpy as np
from rpi_hardware_pwm import HardwarePWM
import board
import busio
import adafruit_bno055

# --- Configuration du LIDAR ---
LIDAR_PORT = '/dev/ttyUSB0'  # Adapté à votre branchement
LIDAR_BAUDRATE = 256000  # Utilisation de 256000 pour le LIDAR

# --- Configuration de la connexion réseau ---
HOST = '192.168.2.148'  # Adresse IP de la machine distante (ordinateur)
PORT = 65432  # Port à utiliser pour la connexion
TIMEOUT = 5  # Délai d'attente en secondes

# adresse I2C du BNO055
ADDRESS = 0x28

# --- Configuration des moteurs ---
direction_prop = 1  # -1 pour les variateurs inversés
pwm_stop_prop = 7.3
point_mort_prop = 0.4
delta_pwm_max_prop = 1.5  # pwm à laquelle on atteint la vitesse maximale
vitesse_max_m_s_hard = 8  # vitesse que peut atteindre la voiture
vitesse_max_m_s_soft = 6  # vitesse maximale que l'on souhaite atteindre
vitesse = 0
vitesse_cible = 0

direction = 1  # 1 pour angle_pwm_min à gauche, -1 pour angle_pwm_min à droite
angle_pwm_min = 2.5  # min
angle_pwm_max = 9  # max
angle_pwm_centre = 5.75

angle_degre_max = 20  # vers la gauche
angle_degre = 0
angle_plus = 0
# last_action = 0

# --- Hysteresis Configuration ---
hysteresis_angle = -6  # Angle en degrés pour l'hystérésis
last_angle_degre = 0  # Stocke le dernier angle commandé

# --- Initialisation des moteurs ---
pwm_prop = HardwarePWM(pwm_channel=0, hz=50)
pwm_prop.start(pwm_stop_prop)

# --- Initialisation du bus I2C ---
i2c = busio.I2C(board.SCL, board.SDA)

# --- Initialisation du BNO055 ---
try:
    sensor = adafruit_bno055.BNO055_I2C(i2c, ADDRESS)
    print("BNO055 sensor initialized successfully")
except Exception as e:
    print(f"[ERREUR] Erreur lors de l'initialisation du BNO055 : {e}")
    sensor = None  # Définir le capteur sur None en cas d'erreur

# --- Initialisation du LIDAR ---
try:
    lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)
    lidar.connect()
    info = lidar.get_info()
    print(info)
    lidar.start_motor()
    time.sleep(1)
except Exception as e:
    print(f"[ERREUR] Erreur lors de l'initialisation du LIDAR : {e}")
    exit()

# --- Fonctions de contrôle des moteurs ---
def set_vitesse_m_s(vitesse_m_s):
    if vitesse_m_s > vitesse_max_m_s_soft:
        vitesse_m_s = vitesse_max_m_s_soft
    elif vitesse_m_s < -vitesse_max_m_s_hard:
        vitesse_m_s = -vitesse_max_m_s_hard
    if vitesse_m_s == 0:
        pwm_prop.change_duty_cycle(pwm_stop_prop)
    elif vitesse_m_s > 0:
        vitesse = vitesse_m_s * (delta_pwm_max_prop) / vitesse_max_m_s_hard
        pwm_prop.change_duty_cycle(
            pwm_stop_prop + direction_prop * (point_mort_prop + vitesse)
        )
    elif vitesse_m_s < 0:
        vitesse = vitesse_m_s * (delta_pwm_max_prop) / vitesse_max_m_s_hard
        pwm_prop.change_duty_cycle(
            pwm_stop_prop - direction_prop * (point_mort_prop - vitesse)
        )
    return vitesse_m_s

def recule():
    """
    Fait reculer le robot brièvement.
    """
    set_vitesse_m_s(-vitesse_max_m_s_hard)
    time.sleep(0.2)
    set_vitesse_m_s(0)
    time.sleep(0.2)
    set_vitesse_m_s(-1)


pwm_dir = HardwarePWM(pwm_channel=1, hz=50)
pwm_dir.start(angle_pwm_centre)


def set_direction_degre(angle_degre):
    """
    Contrôle la direction du moteur de direction.
    """
    global angle_pwm_min
    global angle_pwm_max
    global angle_pwm_centre
    global last_angle_degre

    # Appliquer l'hystérésis
    if angle_degre > last_angle_degre + hysteresis_angle:
        angle_degre_adjusted = angle_degre - hysteresis_angle
    elif angle_degre < last_angle_degre - hysteresis_angle:
        angle_degre_adjusted = angle_degre + hysteresis_angle
    else:
        angle_degre_adjusted = last_angle_degre

    # Ajustement pour permettre les angles négatifs
    angle_pwm = angle_pwm_centre + (
        (angle_pwm_max - angle_pwm_min) * angle_degre_adjusted / (2 * angle_degre_max)
    )
    if angle_pwm > angle_pwm_max:
        angle_pwm = angle_pwm_max
    if angle_pwm < angle_pwm_min:
        angle_pwm = angle_pwm_min
    pwm_dir.change_duty_cycle(angle_pwm)

    last_angle_degre = angle_degre  # Mettre à jour le dernier angle


# --- Fonction pour gérer les commandes à distance ---
def handle_commands(conn):
    """
    Gère les commandes reçues via la connexion socket.
    """
    global direction, vitesse_max_m_s_soft, angle_degre_max, vitesse_cible
    try:
        while True:
            try:
                command = conn.recv(1024).decode('utf-8').strip()
                if command:
                    if command == 'w':  # Avancer
                        # Augmente la vitesse par un pas de 0.25 m/s, sans dépasser la vitesse maximale
                        set_vitesse_m_s(vitesse_cible + (vitesse_cible < vitesse_max_m_s_soft))

                    elif command == 'r':  # Reculer
                        # Diminue la vitesse par un pas de 0.25 m/s, sans devenir négative
                        set_vitesse_m_s(vitesse_cible- (vitesse_cible>1))
                    elif command == 's':  # Reculer
                        recule()
                    elif command == 'a':  # Tourner à gauche
                        # if last_action == 1 : angle_plus = 0
                        set_direction_degre(angle_plus+10)
                        # last_action = 2
                    elif command == 'd':  # Tourner à droite
                        # if last_action == 2 : angle_plus = 0
                        set_direction_degre(angle_plus-10)
                        # last_action = 1
                    elif command == 'x':  # Arrêter
                        vitesse_cible = 0
                    elif command == 'o':  # centrer direction
                        set_direction_degre(0)
                    elif command == 'q':  # Quitter
                        break
                    elif command.startswith('angle'):
                        try:
                            angle = int(command.split(' ')[1])
                            set_direction_degre(angle)
                        except (IndexError, ValueError):
                            print("Format d'angle invalide. Utilisez 'angle [-25 à 25]'.") 
            except ConnectionResetError:
                print("[ERREUR] Connexion interrompue par l'ordinateur.")
                break
            except Exception as e:
                print(f"[ERREUR] Erreur lors de la réception des commandes : {e}")
                break
    except Exception as e:
        print(f"[ERREUR] Erreur dans handle_commands : {e}") 

# --- Fonction principale ---
def main():
    """
    Fonction principale pour gérer le LIDAR, les commandes et la connexion réseau.
    """
    # Initialiser le bus I2C
    i2c = busio.I2C(board.SCL, board.SDA)

    # Initialiser le capteur BNO055
    sensor = None
    try:
        sensor = adafruit_bno055.BNO055_I2C(i2c, ADDRESS)
        print("BNO055 sensor initialized successfully")
    except Exception as e:
        print(f"[ERREUR] Erreur lors de l'initialisation du BNO055 : {e}")
    set_direction_degre(0)
    try:
        # Créer un socket et se lier à l'adresse et au port
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            # Configurer l'adresse et le port pour le serveur
            s.bind((HOST, PORT))  # Ici on utilise bind et non connect
            s.listen(1)  # Le serveur écoute les connexions entrantes

            print("[INFO] En attente de connexion...")  # Ajout du message de recherche
            conn, addr = s.accept()  # Accepter la connexion du client
            with conn:
                print(f"[INFO] Connexion établie avec {addr}")

                # Démarrer le thread pour gérer les commandes
                command_thread = threading.Thread(
                    target=handle_commands, args=(conn,), daemon=True
                )
                command_thread.start()

                # Démarrer le thread pour contrôler la vitesse

                # Boucle principale pour le LIDAR
                try:
                    set_vitesse_m_s(1)
                    while True:
                        try:
                            for scan in lidar.iter_scans():
                                # Traitement des scans du LIDAR
                                tableau_lidar_mm = [0] * 360
                                for i, (quality, angle, distance) in enumerate(scan):
                                    if not all(isinstance(x, (int, float)) for x in [quality, angle, distance]):
                                        print(f"[AVERTISSEMENT] Type incorrect dans le scan {i}")
                                        continue
                                    if 90 <= angle <= 270:  # Ignorer ces points
                                        continue
                                    if distance < 1000:
                                        # Remap et stockage des données LIDAR
                                        if 270 <= angle < 360 or 0 <= angle <= 90:
                                            angle_index = int(angle - 360) if angle >= 270 else int(angle)
                                            if -90 <= angle_index <= 90:
                                                tableau_lidar_mm[angle_index + 90] = distance

                                # Préparer les données à envoyer
                                acceleration = sum(abs(x) for x in sensor.linear_acceleration)
                                data = {
                                    "measurements": tableau_lidar_mm,
                                    "accel": acceleration
                                }
                                data_json = json.dumps(data)

                                try:
                                    conn.sendall((data_json + "\n").encode('utf-8'))
                                except BrokenPipeError:
                                    print("[ERREUR] Connexion interrompue par le client.")
                                    break
                                except socket.timeout:
                                    print("[ERREUR] Délai d'attente d'envoi dépassé.")
                                    break
                        except Exception as e:
                            print(f"[ERREUR] Erreur LIDAR : {e}")
                            break
                except Exception as e:
                    print(f"[ERREUR] Erreur dans la boucle principale du LIDAR : {e}")
    except KeyboardInterrupt:
        print("Fin des acquisitions")
    except Exception as e:
        print(f"[ERREUR] Erreur dans la connexion serveur : {e}")
    finally:
        # Arrêt des moteurs et du LIDAR
        try:
            lidar.stop_motor()
            lidar.stop()
            time.sleep(1)
            lidar.disconnect()
        except Exception as e:
            print(f"[ERREUR] Erreur lors de l'arrêt du LIDAR : {e}")
        pwm_dir.stop()
        pwm_prop.change_duty_cycle(pwm_stop_prop)


if __name__ == '__main__':
    main()
