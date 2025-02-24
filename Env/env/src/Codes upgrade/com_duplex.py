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

# --- Configuration du LIDAR ---
LIDAR_PORT = '/dev/ttyUSB0'  # Adapté à votre branchement
LIDAR_BAUDRATE = 256000  # Utilisation de 256000 pour le LIDAR

# --- Configuration de la connexion réseau ---
HOST = '192.168.2.147'  # Adresse IP de la machine distante (ordinateur)
PORT = 12345  # Port à utiliser pour la connexion
TIMEOUT = 5  # Délai d'attente en secondes

# --- Configuration des moteurs ---
direction_prop = 1  # -1 pour les variateurs inversés
pwm_stop_prop = 7.3
point_mort_prop = 0.4
delta_pwm_max_prop = 1.5  # pwm à laquelle on atteint la vitesse maximale
vitesse_max_m_s_hard = 8  # vitesse que peut atteindre la voiture
vitesse_max_m_s_soft = 1  # vitesse maximale que l'on souhaite atteindre

direction = 1  # 1 pour angle_pwm_min à gauche, -1 pour angle_pwm_min à droite
angle_pwm_min = 4.25  # min
angle_pwm_max = 7.75  # max
angle_pwm_centre = 6

angle_degre_max = 25  # vers la gauche
angle_degre = 0

# --- Initialisation des moteurs ---
pwm_prop = HardwarePWM(pwm_channel=0, hz=50)
pwm_prop.start(pwm_stop_prop)


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

# --- Variables globales ---
tableau_lidar_mm = [0] * 360  # Création d'un tableau de 360 zéros

# --- Fonctions de contrôle des moteurs ---
def set_vitesse_m_s(vitesse_m_s):
    """
    Contrôle la vitesse du moteur de propulsion.
    """
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
    # Ajustement pour permettre les angles négatifs
    angle_pwm = angle_pwm_centre + (
        (angle_pwm_max - angle_pwm_min) * angle_degre / (2 * angle_degre_max)
    )
    if angle_pwm > angle_pwm_max:
        angle_pwm = angle_pwm_max
    if angle_pwm < angle_pwm_min:
        angle_pwm = angle_pwm_min
    pwm_dir.change_duty_cycle(angle_pwm)


# --- Fonction pour gérer les commandes à distance ---
def handle_commands(conn):
    """
    Gère les commandes reçues via la connexion socket.
    """
    global direction, vitesse_max_m_s_soft, angle_degre_max
    try:
        while True:
            try:
                command = conn.recv(1024).decode('utf-8').strip()
                if command:
                    if command == 'w':  # Avancer
                        set_vitesse_m_s(vitesse_max_m_s_soft)
                    elif command == 's':  # Reculer
                        recule()
                    elif command == 'a':  # Tourner à gauche
                        set_direction_degre(angle_degre_max)
                    elif command == 'd':  # Tourner à droite
                        set_direction_degre(-angle_degre_max)
                    elif command == 'x':  # Arrêter
                        set_vitesse_m_s(0)
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
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            # Définir le délai d'attente d'envoi
            # s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDTIMEO, TIMEOUT)

            s.connect((HOST, PORT))  # Se connecter à l'ordinateur
            print(f"[INFO] Connexion établie avec l'ordinateur sur {HOST}:{PORT}")

            # Démarrer le thread pour gérer les commandes
            command_thread = threading.Thread(
                target=handle_commands, args=(s,), daemon=True
            )
            command_thread.start()

            # Boucle principale pour le LIDAR
            try:
                while True:
                    try:
                        for scan in lidar.iter_scans():
                            # print(f"[DEBUG] Scan data: {scan}")
                            # Adaptez cette partie en fonction de la structure réelle des données
                            for i, (quality, angle, distance) in enumerate(scan):
                                # Vérification des types
                                if not all(
                                    isinstance(x, (int, float))
                                    for x in [quality, angle, distance]
                                ):
                                    print(
                                        f"[AVERTISSEMENT] Type de données incorrect dans le scan {i}: quality={type(quality)}, angle={type(angle)}, distance={type(distance)}"
                                    )
                                    continue

                                # Réinitialiser le tableau des mesures avant chaque scan
                                tableau_lidar_mm = [0] * 360

                                # Pour chaque scan, mettre à jour la mesure pour chaque angle.

                                # Ignorer les points entre 135° et 225°
                                if 135 <= angle <= 225:
                                    continue  # Ignorer ce point

                                # N'envoyer que les points dont la distance est inférieure à 4000 mm
                                if distance < 2000:
                                    # Convertir l'angle en un index entre 0 et 359.
                                    angle_index = int(min(359, max(0, 359 - angle)))

                                    # Vérification de l'index
                                    if 0 <= angle_index < len(tableau_lidar_mm):
                                        tableau_lidar_mm[angle_index] = distance
                                    else:
                                        print(
                                            f"[AVERTISSEMENT] Index d'angle invalide : {angle_index}"
                                        )

                        # Préparer les données à envoyer
                        data = {
                            "timestamp": time.time(),
                            "measurements": tableau_lidar_mm,
                        }
                        data_json = json.dumps(data)
                        # Envoyer les données avec un saut de ligne pour délimiter les messages
                        try:
                            s.sendall((data_json + "\n").encode('utf-8'))
                        except BrokenPipeError:
                            print("[ERREUR] Connexion interrompue par l'ordinateur.")
                            break  # Sortir de la boucle LIDAR
                        except socket.timeout:
                            print("[ERREUR] Délai d'attente d'envoi dépassé.")
                            break  # Sortir de la boucle LIDAR

                    except Exception as e:
                        print(
                            f"[ERREUR] Erreur lors de la lecture des scans du LIDAR : {e}"
                        )
                        break  # Sortir de la boucle LIDAR
            except Exception as e:
                print(f"[ERREUR] Erreur dans la boucle principale du LIDAR : {e}")

    except KeyboardInterrupt:  # Récupération du CTRL+C
        print("Fin des acquisitions")

    except Exception as e:
        print(f"[ERREUR] Erreur principale : {e}")

    finally:
        # Arrêt et déconnexion du LIDAR et des moteurs
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
