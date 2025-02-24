#!/usr/bin/env python3
"""
Script sur Raspberry Pi :
- Récupère les données du LIDAR.
- Envoie les données via une connexion réseau en format JSON.
"""

import time
import json
import socket
from rplidar import RPLidar
import numpy as np

# --- Configuration du LIDAR ---
LIDAR_PORT = '/dev/ttyUSB0'  # Adapté à votre branchement
LIDAR_BAUDRATE = 256000  # Utilisation de 256000 pour le LIDAR

# --- Configuration de la connexion réseau ---
HOST = '192.168.2.147'  # Remplacez par l'adresse IP correcte de votre machine distante
PORT = 12345  # Port à utiliser pour la connexion

# Initialisation du LIDAR
lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUDRATE)
lidar.connect()
print(lidar.get_info())
lidar.start_motor()
time.sleep(1)

# Création d'un tableau pour les mesures à 360 degrés (en mm)
tableau_lidar_mm = [0] * 360

# Pré-calcul des angles (en radians) pour chaque degré (0 à 359)
teta = [i * np.pi / 180 for i in range(360)]

# Création d'une socket pour la connexion réseau
try:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        print("[INFO] Connexion établie avec la machine distante.")

        # Boucle principale : mise à jour des données avec chaque scan
        try:
            for scan in lidar.iter_scans():
                # Réinitialiser le tableau des mesures avant chaque scan
                tableau_lidar_mm = [0] * 360

                # Pour chaque scan, mettre à jour la mesure pour chaque angle.
                for quality, angle, distance in scan:
                    # Ignorer les points entre 135° et 225°
                    if 135 <= angle <= 225:
                        continue  # Ignorer ce point

                    # N'envoyer que les points dont la distance est inférieure à 4000 mm
                    if distance < 2000:
                        # Convertir l'angle en un index entre 0 et 359.
                        angle_index = int(min(359, max(0, 359 - angle)))
                        tableau_lidar_mm[angle_index] = distance

                # Préparer les données à envoyer
                data = {
                    "timestamp": time.time(),
                    "measurements": tableau_lidar_mm
                }
                data_json = json.dumps(data)
                # Envoyer les données avec un saut de ligne pour délimiter les messages
                print(f"[DEBUG] Envoi des données : {data_json}")  # Impression pour débogage
                s.sendall((data_json + "\n").encode('utf-8'))

                # Optionnel : imprimer le nombre de points traités dans ce scan
                print("Nombre de points dans le scan : {}".format(len(scan)))

                # Réduire le délai pour augmenter la réactivité
                time.sleep(0.05)  # 50 ms pour envoyer plus fréquemment

        except KeyboardInterrupt:
            print("Acquisition arrêtée par l'utilisateur.")

except Exception as e:
    print(f"[ERREUR] Une erreur est survenue lors de la connexion : {e}")

finally:
    # Arrêter le LIDAR et fermer la connexion
    lidar.stop_motor()
    lidar.stop()
    time.sleep(1)
    lidar.disconnect()
    print("[INFO] LIDAR déconnecté.")
