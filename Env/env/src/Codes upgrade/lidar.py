from rplidar import RPLidar
import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading

class LidarScanner:
    def __init__(self, port="/dev/ttyUSB0", baudrate=256000):
        """Initialise le Lidar et le tableau des distances."""
        self.lidar = RPLidar(port, baudrate=baudrate)
        self.data = [0] * 360  # Tableau de distances
        self.running = False

    def connect(self):
        """Connecte et démarre le Lidar."""
        try:
            self.lidar.connect()
            print(self.lidar.get_info())
            self.lidar.start_motor()
            time.sleep(1)
            self.running = True
        except Exception as e:
            print(f"Erreur lors de la connexion au Lidar : {e}")
            self.running = False

    def update_scan(self):
        """Met à jour le tableau des distances en temps réel."""
        try:
            for scan in self.lidar.iter_scans(scan_type='express'):
                print(f"Nombre de points : {len(scan)}")
                for _, angle, distance in scan:
                    index = min(359, max(0, 359 - int(angle)))
                    self.data[index] = distance
                time.sleep(0.05)  # Laisse le temps au thread de fonctionner
        except KeyboardInterrupt:
            print("Arrêt du scan.")
        except Exception as e:
            print(f"Erreur dans l'acquisition du Lidar : {e}")
        finally:
            self.stop()

    def stop(self):
        """Arrête et déconnecte le Lidar."""
        if self.running:
            try:
                self.lidar.stop_motor()
                self.lidar.stop()
                time.sleep(1)
                self.lidar.disconnect()
            except Exception as e:
                print(f"Erreur lors de l'arrêt du Lidar : {e}")
            self.running = False
        print("Lidar arrêté.")

    def get_data(self):
        """Retourne les données du scan."""
        return self.data


class LidarVisualizer:
    def __init__(self, scanner):
        """Initialise la visualisation avec un scanner Lidar."""
        self.scanner = scanner
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.theta = np.linspace(0, 2 * np.pi, 360)  # Angles en radians
        self.points = self.ax.scatter(self.theta, self.scanner.get_data(), s=5, c=self.scanner.get_data(), cmap='viridis')
        self.ax.set_rmax(8000)  # Distance max affichée
        self.ax.grid(True)
        plt.colorbar(self.points, label="Distance (mm)")
        
        # Correction : Stocker l'animation en tant qu'attribut de classe
        self.ani = None

    def update_plot(self, frame):
        """Met à jour l'affichage du graphique avec les nouvelles données."""
        data = self.scanner.get_data()
        self.points.set_offsets(np.c_[self.theta, data])
        self.points.set_array(np.array(data))  # Met à jour la couleur
        return self.points,

    def animate(self):
        """Démarre l'animation en temps réel."""
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=100, blit=False, cache_frame_data=False
        )
        plt.show()


if __name__ == "__main__":
    lidar = LidarScanner()
    lidar.connect()

    if lidar.running:
        # Démarrer l'acquisition dans un autre thread
        scan_thread = threading.Thread(target=lidar.update_scan, daemon=True)
        scan_thread.start()

        # Lancer la visualisation
        visualizer = LidarVisualizer(lidar)
        visualizer.animate()

        # Attendre que l'utilisateur ferme la fenêtre
        scan_thread.join()

        # Arrêter proprement le Lidar quand l'animation se termine
        lidar.stop()
