import socket
import cv2
import numpy as np

# Configuration
HOST = '192.168.2.148'  # Remplacez par l'adresse IP du Raspberry Pi
PORT = 8000
ENCODING = 'utf-8'

def receive_image():
    """Reçoit et affiche les images du serveur."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            client_socket.connect((HOST, PORT))
            print(f"[INFO] Connecté au serveur sur {HOST}:{PORT}")

            while True:
                try:
                    size_bytes = client_socket.recv(16)
                    if not size_bytes:
                        print("[INFO] Connexion fermée par le serveur.")
                        break

                    image_size = int(size_bytes.decode(ENCODING).strip())
                    image_data = b""
                    while len(image_data) < image_size:
                        chunk = client_socket.recv(min(image_size - len(image_data), 4096))
                        if not chunk:
                            break
                        image_data += chunk

                    if len(image_data) != image_size:
                        print("[ERREUR] Image incomplète reçue.")
                        continue

                    image = cv2.imdecode(np.frombuffer(image_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if image is None:
                        print("[ERREUR] Impossible de décoder l'image.")
                        continue

                    cv2.imshow("Flux de la caméra", image)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                except Exception as e:
                    print(f"[ERREUR] Erreur lors de la réception de l'image: {e}")
                    break

    except ConnectionRefusedError:
        print("[ERREUR] Connexion refusée. Assurez-vous que le serveur est en cours d'exécution.")
    except Exception as e:
        print(f"[ERREUR] Erreur lors de la connexion au serveur: {e}")
    finally:
        cv2.destroyAllWindows()
        print("[INFO] Connexion fermée.")

if __name__ == "__main__":
    receive_image()
