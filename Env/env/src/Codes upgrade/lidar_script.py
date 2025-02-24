from rplidar import RPLidar
import numpy as np
import time
import matplotlib.pyplot as plt

class LidarScanner:
    def __init__(self, port="/dev/ttyUSB0", baudrate=256000):
        self.lidar = RPLidar(port, baudrate=baudrate)
        self.data = [0] * 360  # Tableau des distances en mm
        self.running = False

    def connect(self):
        """Initialise et démarre le Lidar."""
        self.lidar.connect()
        print(self.lidar.get_info())
        self.lidar.start_motor()
        time.sleep(1)
        self.running = True

    def scan(self):
        """Effectue des acquisitions et stocke les valeurs dans le tableau."""
        try:
            for scan in self.lidar.iter_scans(scan_type='express'):
                print(f"Nombre de points : {len(scan)}")
                for _, angle, distance in scan:
                    index = min(359, max(0, 359 - int(angle)))
                    self.data[index] = distance
        except KeyboardInterrupt:
            print("Fin des acquisitions.")
        finally:
            self.stop()

    def stop(self):
        """Arrête et déconnecte le Lidar."""
        if self.running:
            self.lidar.stop_motor()
            self.lidar.stop()
            time.sleep(1)
            self.lidar.disconnect()
            self.running = False
        print("Lidar arrêté.")

    def get_data(self):
        """Retourne les données de scan."""
        return self.data


class LidarVisualizer:
    def __init__(self, data):
        self.data = data

    def plot(self):
        """Affiche les données en coordonnées polaires."""
        theta = np.linspace(0, 2 * np.pi, 360)
        
        fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
        scatter = ax.scatter(theta, self.data, s=5, c=self.data, cmap='viridis')
        ax.set_rmax(8000)
        ax.grid(True)
        plt.colorbar(scatter, label="Distance (mm)")
        plt.show()


if __name__ == "__main__":
    lidar = LidarScanner()
    lidar.connect()
    lidar.scan()
    
    visualizer = LidarVisualizer(lidar.get_data())
    visualizer.plot()
