import neat
import pickle
import socket
import json
import threading

current_generation = 0
BEST_GENOME_FILE = "best_knight.pkl"
HOST = '196.168.2.147'
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
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((HOST, PORT))  # Connect to the same socket

    def send_action(self, action):
        try:
            message = json.dumps({'action': action})  # Send action as JSON
            self.socket.sendall(message.encode('utf-8') + b'\n')
        except Exception as e:
            print(f"[ERREUR] Échec de l'envoi de l'action : {e}")

    def check_collision(self, speed_data):
        if speed_data < 0.1:
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
        return self.distance - steering_penalty - self.oscillation_count

    def get_data(self):
        with data_lock:
            return [x / 1000 for x in lidar_data]


def get_action_from_genome(car, net):
    lidar_data = car.get_data()
    output = net.activate(lidar_data)
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
        for i, car in enumerate(cars):
            if not car.is_alive():
                continue

            action = get_action_from_genome(car, nets[i])

            if action == 0:
                car.angle += 10
                car.send_action('a')
            elif action == 1:
                car.angle -= 10
                car.send_action('d')
            elif action == 2 and car.speed >= 2:
                car.speed -= 1
                car.send_action('r')
            elif action == 3:
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
        if counter == 30 * 40:
            break

    best_genome = max(genomes, key=lambda g: g[1].fitness)
    with open(BEST_GENOME_FILE, 'wb') as f:
        pickle.dump(best_genome[1], f)


def receive_lidar_data():
    global lidar_data, acceleration_data
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((HOST, PORT))
            s.listen()
            print(f"[INFO] En attente de connexion sur {HOST}:{PORT}...")
            conn, addr = s.accept()
            with conn:
                print(f"[INFO] Connexion établie avec {addr}")
                buffer = b""  # Buffer pour stocker les données reçues
                while True:
                    data = conn.recv(1024)  # Taille réduite à 1024 octets
                    if not data:
                        print("[INFO] Connexion fermée")
                        break
                    buffer += data  # Ajoute les données reçues dans le buffer
                    if len(data) < 1024:
                        # Si la taille du paquet est inférieure à 1024, on suppose que l'envoi est terminé
                        break
                # Décoder et traiter les données reçues
                try:
                    data = json.loads(buffer.decode())
                    with data_lock:
                        if 'measurements' in data:
                            lidar_data = data['measurements']
                        if 'acceleration' in data:
                            acceleration_data = data['acceleration']
                except json.JSONDecodeError as e:
                    print(f"[ERREUR] Problème de décodage JSON : {e}")
    except Exception as e:
        print(f"[ERREUR] Erreur lors de la réception des données : {e}")


if __name__ == "__main__":
    lidar_thread = threading.Thread(target=receive_lidar_data, daemon=True)
    lidar_thread.start()
    config_path = "./code_et_test/IA/radar_cfg.txt"
    config = neat.config.Config(neat.DefaultGenome, neat.DefaultReproduction, neat.DefaultSpeciesSet, neat.DefaultStagnation, config_path)

    population = neat.Population(config)
    population.add_reporter(neat.StdOutReporter(True))
    stats = neat.StatisticsReporter()
    population.add_reporter(stats)



    population.run(run_simulation, 1000)
