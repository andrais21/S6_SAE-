import math
import random
import sys
import os
import neat
import pygame
import pickle
from rpi_hardware_pwm import HardwarePWM
import time

# Constants for PWM and Car Control
DIRECTION_FORWARD = 1
DIRECTION_REVERSE = -1
PWM_STOP_DEFAULT = 7.7
POINT_MORT_DEFAULT = 0.5
DELTA_PWM_MAX_DEFAULT = 1.0
VITESSE_MAX_HARD = 8.0  # max speed in m/s
VITESSE_MAX_SOFT = 2.0  # desired max speed in m/s

# Initialize PWM for back tires
pwm_back_tires = HardwarePWM(pwm_channel=0, hz=50)
pwm_back_tires.start(PWM_STOP_DEFAULT)

# Pygame Constants
WIDTH = 1920
HEIGHT = 1080
CAR_SIZE_X = 60
CAR_SIZE_Y = 60
BORDER_COLOR = (255, 255, 255, 255)
current_generation = 0
BEST_GENOME_FILE = "best_genome.pkl"

class Car:
    def __init__(self):
        self.sprite = pygame.image.load('car.png').convert()
        self.sprite = pygame.transform.scale(self.sprite, (CAR_SIZE_X, CAR_SIZE_Y))
        self.rotated_sprite = self.sprite
        self.position = [830, 920]
        self.angle = 0
        self.speed = 0
        self.speed_set = False
        self.center = [self.position[0] + CAR_SIZE_X / 2, self.position[1] + CAR_SIZE_Y / 2]
        self.radars = []
        self.alive = True
        self.distance = 0
        self.time = 0
        self.last_angle = self.angle
        self.last_action = None
        self.oscillation_count = 0
        self.stuck_counter = 0

    def draw(self, screen):
        screen.blit(self.rotated_sprite, self.position)
        for radar in self.radars:
            pos = radar[0]
            pygame.draw.line(screen, (0, 255, 0), self.center, pos, 1)
            pygame.draw.circle(screen, (0, 255, 0), pos, 5)

    def check_collision(self, game_map):
        self.alive = True
        for point in self.corners:
            if game_map.get_at((int(point[0]), int(point[1]))) == BORDER_COLOR:
                self.alive = False
                break

    def check_radar(self, degree, game_map):
        length = 0
        while length < 300:
            x = int(self.center[0] + math.cos(math.radians(360 - (self.angle + degree))) * length)
            y = int(self.center[1] + math.sin(math.radians(360 - (self.angle + degree))) * length)
            if x < 0 or x >= game_map.get_width() or y < 0 or y >= game_map.get_height():
                break
            if game_map.get_at((x, y)) == BORDER_COLOR:
                break
            length += 1
        dist = int(math.sqrt((x - self.center[0]) ** 2 + (y - self.center[1]) ** 2))
        self.radars.append([(x, y), dist])

    def update(self, game_map):
        if not self.speed_set:
            self.speed = 20
            self.speed_set = True
        self.rotated_sprite = self.rotate_center(self.sprite, self.angle)
        self.position[0] += math.cos(math.radians(360 - self.angle)) * self.speed
        self.position[0] = max(self.position[0], 20)
        self.position[0] = min(self.position[0], WIDTH - 120)
        self.position[1] += math.sin(math.radians(360 - self.angle)) * self.speed
        self.position[1] = max(self.position[1], 20)
        self.position[1] = min(self.position[1], HEIGHT - 120)
        self.distance += self.speed
        self.time += 1
        self.center = [int(self.position[0]) + CAR_SIZE_X / 2, int(self.position[1]) + CAR_SIZE_Y / 2]
        length = 0.5 * CAR_SIZE_X
        left_top = [self.center[0] + math.cos(math.radians(360 - (self.angle + 30))) * length,
                    self.center[1] + math.sin(math.radians(360 - (self.angle + 30))) * length]
        right_top = [self.center[0] + math.cos(math.radians(360 - (self.angle + 150))) * length,
                     self.center[1] + math.sin(math.radians(360 - (self.angle + 150))) * length]
        left_bottom = [self.center[0] + math.cos(math.radians(360 - (self.angle + 210))) * length,
                       self.center[1] + math.sin(math.radians(360 - (self.angle + 210))) * length]
        right_bottom = [self.center[0] + math.cos(math.radians(360 - (self.angle + 330))) * length,
                        self.center[1] + math.sin(math.radians(360 - (self.angle + 330))) * length]
        self.corners = [left_top, right_top, left_bottom, right_bottom]
        self.check_collision(game_map)
        self.radars.clear()
        num_front_sensors = 20
        front_arc = 120
        front_start = -front_arc / 2
        front_step = front_arc / (num_front_sensors - 1)
        for i in range(num_front_sensors):
            sensor_angle = front_start + i * front_step
            self.check_radar(sensor_angle, game_map)
        num_side_sensors = 10
        side_arc = 60
        side_step = side_arc / (num_side_sensors // 2 - 1)
        for i in range(num_side_sensors // 2):
            sensor_angle = -120 + i * side_step
            self.check_radar(sensor_angle, game_map)
        for i in range(num_side_sensors // 2):
            sensor_angle = 60 + i * side_step
            self.check_radar(sensor_angle, game_map)
        front_distances = [radar[1] for radar in self.radars[:num_front_sensors]]
        if front_distances and min(front_distances) < 20:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
        if self.stuck_counter > 10 or (self.time > 200 and self.distance < 50):
            self.alive = False

    def get_data(self):
        return [int(radar[1] / 30) for radar in self.radars]

    def is_alive(self):
        return self.alive

    def get_reward(self):
        steering_penalty = abs(self.angle - self.last_angle) * 0.1
        self.last_angle = self.angle
        return self.distance - steering_penalty

    def rotate_center(self, image, angle):
        rect = image.get_rect()
        rotated_image = pygame.transform.rotate(image, angle)
        rotated_rect = rect.copy()
        rotated_rect.center = rotated_image.get_rect().center
        return rotated_image.subsurface(rotated_rect).copy()

    def set_speed(self, speed_m_s):
        # Set speed using PWM control
        self.speed = speed_m_s
        set_vitesse_m_s(speed_m_s)

def set_vitesse_m_s(vitesse_m_s):
    vitesse_m_s = max(min(vitesse_m_s, VITESSE_MAX_SOFT), -VITESSE_MAX_HARD)
    if vitesse_m_s == 0:
        pwm_back_tires.change_duty_cycle(PWM_STOP_DEFAULT)
    else:
        vitesse = vitesse_m_s * (DELTA_PWM_MAX_DEFAULT) / VITESSE_MAX_HARD
        duty_cycle = PWM_STOP_DEFAULT + DIRECTION_FORWARD * (POINT_MORT_DEFAULT + vitesse if vitesse_m_s > 0 else -vitesse)
        pwm_back_tires.change_duty_cycle(duty_cycle)

def recule():
    set_vitesse_m_s(-VITESSE_MAX_HARD)
    time.sleep(0.2)
    set_vitesse_m_s(0)
    time.sleep(0.2)
    set_vitesse_m_s(-1.0)

def run_simulation(genomes, config):
    global current_generation
    nets = []
    cars = []
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.RESIZABLE)
    for i, g in genomes:
        net = neat.nn.FeedForwardNetwork.create(g, config)
        nets.append(net)
        g.fitness = 0
        cars.append(Car())
    clock = pygame.time.Clock()
    generation_font = pygame.font.SysFont("Arial", 30)
    alive_font = pygame.font.SysFont("Arial", 20)
    game_map = pygame.image.load('map4.png').convert()
    current_generation += 1
    counter = 0
    for car in cars:
        car.update(game_map)
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit(0)
        for i, car in enumerate(cars):
            output = nets[i].activate(car.get_data())
            new_action = output.index(max(output))
            if new_action == 0:  # Turn right
                car.angle += 10
            elif new_action == 1:  # Turn left
                car.angle -= 10
            elif new_action == 2:  # Decrease speed
                car.set_speed(max(car.speed - 2, 0))  # Prevent negative speed
            elif new_action == 3:  # Increase speed
                car.set_speed(min(car.speed + 2, VITESSE_MAX_SOFT))  # Limit max speed
            car.last_action = new_action
        still_alive = 0
        for i, car in enumerate(cars):
            if car.is_alive():
                still_alive += 1
                car.update(game_map)
                genomes[i][1].fitness += car.get_reward()
        if still_alive == 0:
            break
        counter += 1
        if counter == 30 * 40:
            break
        screen.blit(game_map, (0, 0))
        for car in cars:
            if car.is_alive():
                car.draw(screen)
        text = generation_font.render("Generation: " + str(current_generation), True, (0, 0, 0))
        text_rect = text.get_rect(center=(900, 450))
        screen.blit(text, text_rect)
        text = alive_font.render("Still Alive: " + str(still_alive), True, (0, 0, 0))
        text_rect = text.get_rect(center=(900, 490))
        screen.blit(text, text_rect)
        pygame.display.flip()
        clock.tick(60)
    best_genome = max(genomes, key=lambda g: g[1].fitness)
    with open(BEST_GENOME_FILE, 'wb') as f:
        pickle.dump(best_genome[1], f)

if __name__ == "__main__":
    config_path = "./radar_cfg.txt"
    config = neat.config.Config(neat.DefaultGenome, neat.DefaultReproduction, neat.DefaultSpeciesSet, neat.DefaultStagnation, config_path)
    best_genome = None
    if os.path.exists(BEST_GENOME_FILE):
        with open(BEST_GENOME_FILE, 'rb') as f:
            best_genome = pickle.load(f)
    population = neat.Population(config)
    population.add_reporter(neat.StdOutReporter(True))
    stats = neat.StatisticsReporter()
    population.add_reporter(stats)
    if best_genome is not None:
        existing_keys = list(population.population.keys())
        if existing_keys:
            del population.population[existing_keys[0]]
        population.population[best_genome.key] = best_genome
        population.species.speciate(config, population.population, population.generation)
    population.run(run_simulation, 1000)

# Cleanup PWM on exit
pwm_back_tires.change_duty_cycle(PWM_STOP_DEFAULT)
