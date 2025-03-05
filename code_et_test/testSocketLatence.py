import socket
import json
import time  # Importation du module time

x = 0

def client():
    # Crée un socket client
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Connecte-toi au serveur (remplace l'IP du serveur et le port)
    client_socket.connect(('192.168.2.147', 65432))  # Remplace par l'IP du serveur

    # Crée un tableau JSON 32x16
    tableau = [[(i + j) % 256 for j in range(16)] for i in range(32)]  # Exemple de tableau 32x16

    # Convertir en JSON
    json_data = json.dumps(tableau)

    # Mesure du temps de réponse
    start_time = time.time()  # Enregistrement du temps avant l'envoi

    # Envoi des données en plusieurs paquets de taille réduite
    for i in range(0, len(json_data), 1024):  # Envoi par morceaux de 1024 octets
        client_socket.sendall(json_data[i:i+1024].encode())

    # Réception de la réponse du serveur
    data = client_socket.recv(1024)
    response = json.loads(data.decode())

    # Fin de la mesure du temps
    end_time = time.time()

    # Affichage du temps de réponse
    client_socket.close()
    print("Réponse du serveur:", response)
    delay = round(end_time - start_time, 4)
    print(f"Temps de réponse: {delay:.4f} secondes")
    return delay

if __name__ == "__main__":
    delays = []
    for i in range(100):
        delay = client()
        delays.append(delay)
        time.sleep(0.1)
    average_delay = sum(delays) / len(delays)
    print(f"Temps de réponse moyen: {average_delay:.4f} secondes")
