#!/usr/bin/env python3
"""
Script sur la machine distante :
  - Se connecte à la socket pour recevoir les données du Raspberry Pi.
  - Lit et affiche les données reçues en format JSON.
  - Affiche une carte polaire des points du LIDAR en direct.
"""

import socket
import json
import numpy as np
import matplotlib.pyplot as plt

# --- Configuration de la connexion réseau ---
HOST = '0.0.0.0'  # Écoute sur toutes les interfaces
PORT = 12345  # Port à utiliser pour la connexion

# Création d'une socket pour écouter les connexions entrantes
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    print(f"[INFO] En attente de connexion sur {HOST}:{PORT}...")
    conn, addr = s.accept()
    with conn:
        print(f"[INFO] Connexion établie avec {addr}")

        # Préparation de la figure pour le graphique polaire
        plt.ion()  # Mode interactif
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='polar')
        ax.set_ylim(0, 1000)  # Ajustez la portée selon vos besoins
        ax.set_title("Carte polaire des points LIDAR en direct")

        buffer = ""
        # Boucle principale pour recevoir et afficher les données
        while True:
            try:
                line = conn.recv(1024).decode('utf-8')
                if not line:
                    print("[INFO] La connexion a été fermée par l'autre partie.")
                    break  # Fin de la connexion

                buffer += line
                # Vérifiez si le buffer contient un message complet
                while '\n' in buffer:
                    message, buffer = buffer.split('\n', 1)  # Séparez le message complet
                    try:
                        data = json.loads(message.strip())
                        measurements = data['measurements']

                        # Convertir les mesures en coordonnées polaires
                        angles = np.linspace(0, 2 * np.pi, num=360)  # Angles de 0 à 360 degrés
                        distances = np.array(measurements)  # Convertir en tableau numpy

                        # Effacer l'ancienne carte polaire
                        ax.clear()
                        ax.set_ylim(0, 1000)  # Ajustez la portée selon vos besoins
                        ax.set_title("Carte polaire des points LIDAR en direct")

                        # Tracer les points
                        sc = ax.scatter(angles, distances, s=5, c=distances, cmap='viridis', alpha=0.75)

                        # Mettre à jour le graphique
                        plt.draw()
                        plt.pause(0.001)  # Pause pour permettre le rafraîchissement

                    except json.JSONDecodeError as e:
                        print(f"[ERREUR] Problème de décodage JSON : {e}")

            except Exception as e:
                print(f"[ERREUR] Une erreur est survenue lors de la réception des données : {e}")
                break  # Sortir de la boucle en cas d'erreur

# Fermer la figure à la fin
plt.ioff()
plt.show()
