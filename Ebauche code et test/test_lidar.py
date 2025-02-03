from rplidar import RPLidar
import numpy as np
import time
import matplotlib.pyplot as plt
import serial  # Importation de la librairie serial

# Configuration du port et du débit
port = "/dev/ttyUSB0"  # Remplacez par le bon port si nécessaire
baudrate = 256000

# Vérification du port série
try:
    ser = serial.Serial(port, baudrate)
    print(ser)  # Affiche les informations du port série
    ser.close()
except serial.SerialException as e:
    print(f"Erreur lors de l'ouverture du port série {port}: {e}")
    exit()  # Quitte le programme en cas d'erreur

# Connexion et démarrage du lidar
try:
    lidar = RPLidar(port, baudrate=baudrate)
    lidar.connect()
    print(lidar.get_info())
    lidar.start_motor()
    time.sleep(1)
except Exception as e:
    print(f"Erreur lors de la connexion au lidar: {e}")
    exit()

try:
    # Création de la figure et des axes polaires
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    ax.set_rmax(8000)  # Définir la distance maximale du graphique
    ax.grid(True)

    for scan in lidar.iter_scans(scan_type='express'):
        # Effacer le graphique précédent
        ax.clear()

        # Affichage du nombre de points récupérés lors du tour, pour les tests
        print("nb pts : " + str(len(scan)))

        # Rangement des données dans des listes pour les angles et les distances
        angles = []
        distances = []
        for i in range(len(scan)):
            angle = 359 - int(scan[i][1])  # scan[i][1] : angle
            distance = scan[i][2]  # scan[i][2] : distance
            angles.append(np.radians(angle))  # Conversion en radians pour matplotlib
            distances.append(distance)

        # Affichage des points avec des traits pour la distance
        ax.plot(angles, distances, marker='o', markersize=2, linestyle='-', linewidth=0.5)

        # Mettre à jour le graphique
        fig.canvas.draw()
        fig.canvas.flush_events()

except KeyboardInterrupt:  # Récupération du CTRL+C
    print("fin des acquisitions")

# Arrêt et déconnexion du lidar
try:
    lidar.stop_motor()
    lidar.stop()
    time.sleep(1)
    lidar.disconnect()
except Exception as e:
    print(f"Erreur lors de l'arrêt du lidar: {e}")

plt.close()  # Fermer la fenêtre graphique