#!/usr/bin/env python3
"""
Script côté ordinateur pour :
  - Recevoir les données du LIDAR du Raspberry Pi.
  - Envoyer des commandes de contrôle au Raspberry Pi.
"""

import socket
import json
import threading

# --- Configuration de la connexion réseau ---
HOST = '0.0.0.0'  # Écoute sur toutes les interfaces
PORT = 12345  # Port à utiliser pour la connexion

# --- Variables globales ---
lidar_data = None  # Pour stocker les données du LIDAR
data_lock = threading.Lock()  # Pour synchroniser l'accès aux données


# --- Fonctions ---
def receive_lidar_data(conn):
    """
    Reçoit les données du LIDAR et les stocke.
    """
    global lidar_data
    buffer = ""
    try:
        while True:
            line = conn.recv(1024).decode('utf-8')
            if not line:
                print("[INFO] La connexion LIDAR a été fermée par l'autre partie.")
                break

            buffer += line
            while '\n' in buffer:
                message, buffer = buffer.split('\n', 1)
                try:
                    data = json.loads(message.strip())
                    measurements = data['measurements']

                    with data_lock:
                        lidar_data = measurements  # Stocke les données

                    # print(f"[INFO] Données LIDAR reçues: {measurements}")

                except json.JSONDecodeError as e:
                    print(f"[ERREUR] Problème de décodage JSON : {e}")

    except Exception as e:
        print(f"[ERREUR] Erreur lors de la réception des données LIDAR : {e}")


def send_commands(conn):
    """
    Envoie des commandes de contrôle au Raspberry Pi.
    """
    try:
        while True:
            command = input(
                "Entrez une commande (w: avancer, s: reculer, "
                "a: gauche, d: droite, angle (-25 à 25): angle, x: arrêter, q: quitter): "
            )
            if command == 'q':
                print("[INFO] Fermeture de la connexion de commande...")
                conn.sendall(command.encode('utf-8'))
                break
            elif command.startswith('angle'):
                try:
                    angle = int(command.split(' ')[1])
                    if -25 <= angle <= 25:
                        conn.sendall(f'angle {angle}'.encode('utf-8'))
                    else:
                        print("Angle invalide. Veuillez entrer un angle entre -25 et 25.")
                except (IndexError, ValueError):
                    print("Format d'angle invalide. Utilisez 'angle [-25 à 25]'.")
            elif command in ['w', 's', 'a', 'd', 'x', 'o']:
                conn.sendall(command.encode('utf-8'))
            else:
                print("Commande invalide. Veuillez réessayer.")
    except KeyboardInterrupt:
        print("\n[INFO] Fermeture de la connexion de commande...")
    except Exception as e:
        print(f"[ERREUR] Erreur lors de l'envoi de commandes : {e}")


def main():
    """
    Fonction principale pour gérer les connexions et les threads.
    """
    global lidar_data

    try:
        # --- Création d'une socket pour écouter les connexions entrantes ---
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((HOST, PORT))
            s.listen()
            print(f"[INFO] En attente de connexion sur {HOST}:{PORT}...")
            conn, addr = s.accept()
            with conn:
                print(f"[INFO] Connexion établie avec {addr}")

                # --- Démarrage du thread pour recevoir les données LIDAR ---
                lidar_thread = threading.Thread(
                    target=receive_lidar_data, args=(conn,), daemon=True
                )
                lidar_thread.start()

                # --- Envoi des commandes dans le thread principal ---
                send_commands(conn)

                # --- Attendre la fin du thread LIDAR ---
                lidar_thread.join()

    except Exception as e:
        print(f"[ERREUR] Erreur principale : {e}")


if __name__ == '__main__':
    main()
