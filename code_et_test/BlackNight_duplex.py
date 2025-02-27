#!/usr/bin/env python3
"""
Script côté ordinateur pour :
  - Recevoir les données du LIDAR du Raspberry Pi.
  - Envoyer des commandes de contrôle au Raspberry Pi.
"""

import socket
import json
import threading
import neat
import pickle
import os
import math

# --- Configuration de la connexion réseau ---
HOST = '0.0.0.0'  # Écoute sur toutes les interfaces
PORT = 12345  # Port à utiliser pour la connexion

# --- Variables globales ---
lidar_data = None  # Pour stocker les données du LIDAR
data_lock = threading.Lock()  # Pour synchroniser l'accès aux données
genome = None  # Pour stocker le génome de l'IA
net = None  # Pour stocker le réseau neuronal
config = None  # Pour stocker la configuration NEAT
BEST_GENOME_FILE = "best_genome.pkl"  # Nom du fichier pour le meilleur génome


# --- Fonctions ---
def load_genome_and_config(config_file):
    """
    Charge le génome et la configuration NEAT à partir des fichiers.
    """
    global genome, net, config
    config = neat.config.Config(
        neat.DefaultGenome,
        neat.DefaultReproduction,
        neat.DefaultSpeciesSet,
        neat.DefaultStagnation,
        config_file,
    )

    if os.path.exists(BEST_GENOME_FILE):
        with open(BEST_GENOME_FILE, 'rb') as f:
            genome = pickle.load(f)
        net = neat.nn.FeedForwardNetwork.create(genome, config)
        print("[INFO] Génome chargé depuis le fichier.")
    else:
        print(
            "[WARNING] Aucun fichier de génome trouvé. "
            "Assurez-vous d'avoir entraîné l'IA et sauvegardé le génome."
        )


def get_action_from_genome(lidar_data):
    """
    Utilise le génome chargé pour déterminer l'action à effectuer en fonction
    des données du LIDAR.
    """
    global net
    if net is None or lidar_data is None:
        print(
            "[WARNING] Le réseau neuronal n'est pas initialisé ou les données "
            "LIDAR sont manquantes."
        )
        return None

    # Normaliser les données du LIDAR (important pour la cohérence avec
    # l'entraînement)
    normalized_lidar_data = [x / 30 for x in lidar_data]

    # Activer le réseau neuronal avec les données normalisées
    output = net.activate(normalized_lidar_data)

    # Interpréter la sortie du réseau neuronal comme une action
    action = output.index(max(output))  # Choisir l'action avec la plus haute valeur

    return action


def receive_lidar_data(conn):
    """
    Reçoit les données du LIDAR, les stocke et utilise l'IA pour déterminer
    l'action.
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

                    # Utiliser l'IA pour obtenir une action basée sur les données
                    action = get_action_from_genome(lidar_data)
                    if action is not None:
                        # Envoyer la commande correspondante au Raspberry Pi
                        command = interpret_action(action)
                        if command:
                            conn.sendall(command.encode('utf-8'))
                            print(f"[INFO] Envoi de la commande: {command}")
                        else:
                            print("[INFO] Aucune commande à envoyer.")

                except json.JSONDecodeError as e:
                    print(f"[ERREUR] Problème de décodage JSON : {e}")

    except Exception as e:
        print(f"[ERREUR] Erreur lors de la réception des données LIDAR : {e}")


def interpret_action(action):
    """
    Interprète l'action de l'IA et retourne la commande correspondante.
    """
    if action == 0:
        return 'a'  # Gauche
    elif action == 1:
        return 'd'  # Droite
    elif action == 2:
        return 's'  # Ralentir (reculer)
    elif action == 3:
        return 'w'  # Accélérer (avancer)
    else:
        return None  # Aucune action


def send_commands(conn):
    """
    Envoie des commandes de contrôle au Raspberry Pi (désactivé si l'IA est
    utilisée).
    """
    try:
        while True:
            command = input(
                "Entrez une commande (w: avancer, s: reculer, "
                "a: gauche, d: droite, angle (-20 à 20): angle, x: arrêter, q: quitter): "
            )
            if command == 'q':
                print("[INFO] Fermeture de la connexion de commande...")
                conn.sendall(command.encode('utf-8'))
                break
            elif command.startswith('angle'):
                try:
                    angle = int(command.split(' ')[1])
                    if -20 <= angle <= 20:
                        conn.sendall(f'angle {angle}'.encode('utf-8'))
                    else:
                        print("Angle invalide. Veuillez entrer un angle entre -20 et 20.")
                except (IndexError, ValueError):
                    print("Format d'angle invalide. Utilisez 'angle [-20 à 20]'.")
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

    # Charger le génome et la configuration NEAT
    config_file = "./code_et_test/IA/radar_cfg.txt"  # Chemin vers le fichier de config NEAT
    load_genome_and_config(config_file)

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
                # send_commands(conn) # Désactiver l'envoi manuel de commandes

                # --- Attendre la fin du thread LIDAR ---
                lidar_thread.join()

    except Exception as e:
        print(f"[ERREUR] Erreur principale : {e}")


if __name__ == '__main__':
    main()
