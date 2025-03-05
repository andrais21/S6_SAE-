import socket
import json
import time

def handle_client(conn):
    # Réception des données avec un buffer plus petit
    data = conn.recv(1024)  # Taille réduite à 1024 octets
    buffer = b''  # Buffer pour stocker les données reçues

    while data:
        buffer += data  # Ajoute les données reçues dans le buffer
        if len(data) < 1024:
            # Si la taille du paquet est inférieure à 1024, on suppose que l'envoi est terminé
            break
        data = conn.recv(1024)  # Continue à recevoir tant qu'il y a plus de données

    # Décoder et traiter les données reçues
    try:
        json_data = json.loads(buffer.decode())
        print("Tableau JSON reçu du client:", json_data)
    except json.JSONDecodeError as e:
        print("Erreur de décodage JSON du client:", e)

    # Envoi de la réponse au client
    response = {"status": "OK", "message": "Données reçues"}
    conn.sendall(json.dumps(response).encode())  # Envoi de la réponse
    conn.close()

def server():
    # Crée un socket serveur
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('192.168.2.147', 65432))  # Adapte l'adresse IP du serveur et le port
    server_socket.listen(5)  # Augmente le nombre de connexions en attente

    print("En attente de connexion...")

    try:
        while True:  # Boucle infinie pour accepter plusieurs connexions
            conn, addr = server_socket.accept()
            print(f"Connexion établie avec {addr}")
            handle_client(conn)

    except KeyboardInterrupt:
        print("Serveur interrompu par l'utilisateur.")
    finally:
        server_socket.close()

if __name__ == "__main__":
    server()
