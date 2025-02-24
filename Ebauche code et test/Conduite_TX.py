import socket

# Configuration de la connexion
HOST = '192.168.2.148'  # Remplacez par l'adresse IP de votre Raspberry Pi
PORT = 12345             # Port à utiliser pour la connexion

def main():
    # Création de la connexion socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        print(f"[INFO] Connexion établie avec {HOST}:{PORT}")

        try:
            while True:
                command = input("Entrez une commande (w: avancer, s: reculer, "
                                "a: gauche, d: droite, x: arrêter, q: quitter): ")
                if command in ['w', 's', 'a', 'd', 'x', 'q','o']:
                    s.sendall(command.encode('utf-8'))
                    if command == 'q':
                        print("[INFO] Fermeture de la connexion...")
                        break
                else:
                    print("Commande invalide. Veuillez réessayer.")
        except KeyboardInterrupt:
            print("\n[INFO] Fermeture de la connexion...")
        except Exception as e:
            print(f"[ERREUR] {e}")

if __name__ == '__main__':
    main()
