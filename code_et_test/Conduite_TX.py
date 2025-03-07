import socket
import time
import threading

# Configuration de la connexion
HOST = '192.168.2.148'  # Remplacez par l'adresse IP de votre Raspberry Pi
PORT = 65432  # Port à utiliser pour la connexion (doit correspondre au serveur)

# Variable globale pour stocker les données reçues
received_data = None


def receive_data(s):
    """Reçoit les données du serveur."""
    global received_data
    try:
        while True:
            data = s.recv(1024).decode('utf-8')
            if not data:
                print("[INFO] Connexion fermée par le serveur.")
                break
            received_data = data
            # print(f"[INFO] Données reçues du serveur: {data}")
    except Exception as e:
        print(f"[ERREUR] Erreur lors de la réception des données: {e}")


def send_commands(s):
    """Envoie des commandes au serveur."""
    try:
        while True:
            command = input(
                "Entrez une commande (w: avancer, s: reculer, "
                "a: gauche, d: droite, x: arrêter, q: quitter): "
            )
            if command in ['w', 's', 'a', 'd', 'x', 'q', 'o']:
                s.sendall(command.encode('utf-8'))
                if command == 'q':
                    print("[INFO] Fermeture de la connexion...")
                    break
            else:
                print("Commande invalide. Veuillez réessayer.")
    except KeyboardInterrupt:
        print("\n[INFO] Fermeture de la connexion...")
    except Exception as e:
        print(f"[ERREUR] Erreur lors de l'envoi de commandes: {e}")


def main():
    try:
        # Création de la connexion socket
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            print("Tentative de connexion...")
            s.connect((HOST, PORT))  # Utilisez connect au lieu de bind/accept
            print(f"[INFO] Connexion établie avec {HOST}:{PORT}")

            # Démarrer un thread pour recevoir les données
            receive_thread = threading.Thread(target=receive_data, args=(s,), daemon=True)
            receive_thread.start()

            # Envoyer les commandes dans le thread principal
            send_commands(s)

            # Attendre la fin du thread de réception
            receive_thread.join()

    except ConnectionRefusedError:
        print(
            f"[ERREUR] Impossible de se connecter à {HOST}:{PORT}. "
            "Vérifiez si le serveur est en cours d'exécution."
        )
    except Exception as e:
        print(f"[ERREUR] {e}")


if __name__ == '__main__':
    main()
    