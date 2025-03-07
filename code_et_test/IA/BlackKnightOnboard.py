import neat
import pickle
import socket
import json
import threading
import time

current_generation = 0
BEST_GENOME_FILE = "best_knight.pkl"
HOST = '192.168.2.148'
PORT = 65432
lidar_data = None
acceleration_data = None
data_lock = threading.Lock()

class Car:
    def __init__(self):
        self.angle = 0
        self.speed = 0
        self.speed_set = False
        self.alive = True
        self.distance = 0
        self.time = 0
        self.last_angle = self.angle
        self.last_action = None
        self.oscillation_count = 0
        
        # Création de la connexion unique au serveur
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        connected = False
        while not connected:
            try:
                self.socket.connect((HOST, PORT))
                connected = True
                print(f"Connexion réussie à {HOST}:{PORT}")
            except ConnectionRefusedError:
                print(f"[ERREUR] Impossible de se connecter à {HOST}:{PORT}. Tentative de reconnexion...")
                time.sleep(1)
        
        # Démarrage d'un thread pour recevoir les données via la même connexion
        self.receive_thread = threading.Thread(target=self.receive_data, daemon=True)
        self.receive_thread.start()

    def receive_data(self):
        global lidar_data, acceleration_data
        try:
            while True:
                buffer = b""
                # Attendre la fin d'un message délimité par "\n"
                while b"\n" not in buffer:
                    data = self.socket.recv(1024)
                    if not data:
                        print("[INFO] Connexion fermée par le serveur")
                        return
                    buffer += data
                
                # Découper le buffer selon le délimiteur "\n"
                messages = buffer.split(b"\n")
                # Traiter tous les messages complets reçus
                for message in messages[:-1]:
                    try:
                        data_json = json.loads(message.decode())
                        with data_lock:
                            if 'measurements' in data_json:
                                lidar_data = data_json['measurements']
                            if 'accel' in data_json:
                                acceleration_data = data_json['accel']
                                print(f"[INFO] Données d'accélération reçues : {acceleration_data}")
                    except json.JSONDecodeError as e:
                        print(f"[ERREUR] Problème de décodage JSON : {e}")
                # Le dernier morceau peut être incomplet, on le garde pour la prochaine lecture
                buffer = messages[-1]
        except Exception as e:
            print(f"[ERREUR] Erreur lors de la réception des données : {e}")

    def send_action(self, action):
        try:
            message = json.dumps({'action': action})
            self.socket.sendall(message.encode('utf-8') + b'\n')
            print(f"[INFO] Action envoyée : {action}")
        except Exception as e:
            print(f"[ERREUR] Échec de l'envoi de l'action : {e}")

    def check_collision(self, accel_data):
        if accel_data < 0.5:
            self.alive = False

    def update(self):
        if not self.speed_set:
            self.speed = 1
            self.speed_set = True
        with data_lock:
            if acceleration_data:
                self.check_collision(acceleration_data)

    def is_alive(self):
        return self.alive

    def get_reward(self):
        steering_penalty = abs(self.angle - self.last_angle) * 0.1
        self.last_angle = self.angle
        reward = self.distance - steering_penalty - self.oscillation_count
        reward += self.speed * 0.5
        print(f"[INFO] Récompense calculée : {reward}")
        return reward

    def get_data(self):
        with data_lock:
            if lidar_data is not None:
                # On suppose que lidar_data est une liste de distances en millimètres
                radar_data = [int(distance / 1000) for distance in lidar_data]
            else:
                radar_data = [0] * 360  # Par exemple, 360 valeurs
        print(f"[INFO] Données des radars : {radar_data}")
        return radar_data

def get_action_from_genome(car, net):
    data = car.get_data()
    output = net.activate(data)
    print(f"[INFO] Sortie du réseau de neurones : {output}")
    return output.index(max(output))

def run_simulation(genomes, config):
    global current_generation
    nets = []
    cars = []
    
    for i, g in genomes:
        net = neat.nn.FeedForwardNetwork.create(g, config)
        nets.append(net)
        g.fitness = 0
        cars.append(Car())
    
    current_generation += 1
    counter = 0
    still_alive = len(cars)
    
    while still_alive > 0:
        print(f"[INFO] Génération: {current_generation}, Voitures vivantes: {still_alive}")
        for i, car in enumerate(cars):
            if not car.is_alive():
                continue
            action = get_action_from_genome(car, nets[i])
            print(f"[INFO] Action de la voiture {i}: {action}")
            
            # Logique d'action
            if action == 0:
                car.angle += 10
                car.send_action('a')
            elif action == 1:
                car.angle -= 10
                car.send_action('d')
            elif action == 2 and car.speed >= 2:
                car.speed -= 1
                car.send_action('r')
            elif action == 3 and car.speed <= 6:
                car.speed += 1
                car.send_action('w')
            
            car.update()
            
            if not car.is_alive():
                genomes[i][1].fitness -= 10
                car.speed = 1
                car.send_action('s')
                continue
            
            genomes[i][1].fitness += car.get_reward()
        
        still_alive = sum(car.is_alive() for car in cars)
        counter += 1
        print(f"[INFO] Compteur: {counter}, Voitures vivantes: {still_alive}")
        
        if counter == 30 * 40:
            print("[INFO] Fin de la simulation après 40 secondes.")
            break
    
    best_genome = max(genomes, key=lambda g: g[1].fitness)
    with open(BEST_GENOME_FILE, 'wb') as f:
        pickle.dump(best_genome[1], f)

if __name__ == "__main__":
    config_path = "./code_et_test/IA/radar_cfg.txt"
    config = neat.config.Config(neat.DefaultGenome, neat.DefaultReproduction,
                                neat.DefaultSpeciesSet, neat.DefaultStagnation, config_path)
    
    population = neat.Population(config)
    population.add_reporter(neat.StdOutReporter(True))
    stats = neat.StatisticsReporter()
    population.add_reporter(stats)
    
    population.run(run_simulation, 1000)
