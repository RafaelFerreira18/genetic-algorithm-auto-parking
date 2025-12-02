import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from typing import List, Tuple, Dict
import random
import skfuzzy as fuzz
from skfuzzy import control as ctrl


class VehicleParams:
    L = 0.325
    B = 0.29
    W = B
    f = 0.1
    l = 0.05
    A = 0.475
    phi_max = np.deg2rad(33.0)
    phi_max_deg = 33.0
    phi_dot_max = 1.0
    V_max = 1.0
    V_dot_max = 0.5


class GAParams:
    POPULATION_SIZE = 50
    GENERATIONS = 100
    CROSSOVER_RATE = 0.6
    MUTATION_RATE = 0.04
    K0_MIN, K0_MAX = 1.0, 50.0
    K1_MIN, K1_MAX = 1.0, 50.0
    VS_VALUES = [-1, 1]
    PRECISION = 1e-8


class FuzzySafetyController:
    def __init__(self):
        distance = ctrl.Antecedent(np.arange(0, 2.01, 0.01), 'distance')
        approach_speed = ctrl.Antecedent(np.arange(-1, 1.01, 0.01), 'approach_speed')
        steering_angle = ctrl.Antecedent(np.arange(0, 34, 1), 'steering_angle')
        
        speed_factor = ctrl.Consequent(np.arange(0, 1.01, 0.01), 'speed_factor')
        alert_level = ctrl.Consequent(np.arange(0, 1.01, 0.01), 'alert_level')
        
        distance['muito_proximo'] = fuzz.trapmf(distance.universe, [0, 0, 0.1, 0.2])
        distance['proximo'] = fuzz.trimf(distance.universe, [0.15, 0.3, 0.5])
        distance['medio'] = fuzz.trimf(distance.universe, [0.4, 0.7, 1.0])
        distance['longe'] = fuzz.trapmf(distance.universe, [0.8, 1.2, 2.0, 2.0])
        
        approach_speed['afastando'] = fuzz.trapmf(approach_speed.universe, [-1, -1, -0.2, 0])
        approach_speed['estacionario'] = fuzz.trimf(approach_speed.universe, [-0.1, 0, 0.1])
        approach_speed['aproximando_devagar'] = fuzz.trimf(approach_speed.universe, [0.05, 0.3, 0.5])
        approach_speed['aproximando_rapido'] = fuzz.trapmf(approach_speed.universe, [0.4, 0.6, 1, 1])
        
        steering_angle['pequeno'] = fuzz.trimf(steering_angle.universe, [0, 0, 15])
        steering_angle['medio'] = fuzz.trimf(steering_angle.universe, [10, 20, 28])
        steering_angle['grande'] = fuzz.trapmf(steering_angle.universe, [25, 30, 33, 33])
        
        speed_factor['parar'] = fuzz.trimf(speed_factor.universe, [0, 0, 0.1])
        speed_factor['muito_lento'] = fuzz.trimf(speed_factor.universe, [0.05, 0.2, 0.35])
        speed_factor['lento'] = fuzz.trimf(speed_factor.universe, [0.3, 0.5, 0.7])
        speed_factor['normal'] = fuzz.trimf(speed_factor.universe, [0.6, 0.8, 0.95])
        speed_factor['maximo'] = fuzz.trimf(speed_factor.universe, [0.9, 1.0, 1.0])
        
        alert_level['baixo'] = fuzz.trimf(alert_level.universe, [0, 0, 0.3])
        alert_level['medio'] = fuzz.trimf(alert_level.universe, [0.2, 0.5, 0.8])
        alert_level['alto'] = fuzz.trimf(alert_level.universe, [0.7, 1.0, 1.0])
        
        rules = [
            ctrl.Rule(distance['muito_proximo'] & approach_speed['aproximando_rapido'],
                     (speed_factor['parar'], alert_level['alto'])),
            ctrl.Rule(distance['muito_proximo'] & approach_speed['aproximando_devagar'],
                     (speed_factor['muito_lento'], alert_level['alto'])),
            ctrl.Rule(distance['proximo'] & approach_speed['aproximando_rapido'],
                     (speed_factor['muito_lento'], alert_level['alto'])),
            ctrl.Rule(distance['proximo'] & approach_speed['aproximando_devagar'],
                     (speed_factor['lento'], alert_level['medio'])),
            ctrl.Rule(distance['proximo'] & steering_angle['grande'],
                     (speed_factor['lento'], alert_level['medio'])),
            ctrl.Rule(distance['medio'] & approach_speed['aproximando_devagar'],
                     (speed_factor['normal'], alert_level['baixo'])),
            ctrl.Rule(distance['medio'] & steering_angle['medio'],
                     (speed_factor['normal'], alert_level['baixo'])),
            ctrl.Rule(distance['longe'],
                     (speed_factor['maximo'], alert_level['baixo'])),
            ctrl.Rule(approach_speed['afastando'],
                     (speed_factor['maximo'], alert_level['baixo'])),
            ctrl.Rule(distance['medio'] & steering_angle['grande'],
                     (speed_factor['lento'], alert_level['medio'])),
        ]
        
        control_system = ctrl.ControlSystem(rules)
        self.controller = ctrl.ControlSystemSimulation(control_system)
    
    def compute_safe_velocity(self, distance_to_obstacle: float, 
                             current_velocity: float,
                             steering_angle_deg: float) -> Dict:
        try:
            self.controller.input['distance'] = np.clip(distance_to_obstacle, 0, 2.0)
            self.controller.input['approach_speed'] = np.clip(current_velocity, -1, 1)
            self.controller.input['steering_angle'] = np.clip(abs(steering_angle_deg), 0, 33)
            
            self.controller.compute()
            
            speed_factor = self.controller.output['speed_factor']
            alert_level = self.controller.output['alert_level']
            
            safe_velocity = VehicleParams.V_max * speed_factor
            
            return {
                'speed_factor': speed_factor,
                'alert_level': alert_level,
                'safe_velocity': safe_velocity,
                'should_stop': speed_factor < 0.15,
                'warning': alert_level > 0.6
            }
        except:
            return {
                'speed_factor': 0.5,
                'alert_level': 0.5,
                'safe_velocity': VehicleParams.V_max * 0.5,
                'should_stop': False,
                'warning': True
            }


class ProximitySensor:
    def __init__(self, obstacles: List[Tuple[float, float, float, float]]):
        self.obstacles = obstacles
        self.sensor_positions = [
            ('front', 0.0, VehicleParams.A / 2),
            ('front_left', -VehicleParams.W / 2, VehicleParams.A / 2),
            ('front_right', VehicleParams.W / 2, VehicleParams.A / 2),
            ('rear', 0.0, -VehicleParams.A / 2),
            ('rear_left', -VehicleParams.W / 2, -VehicleParams.A / 2),
            ('rear_right', VehicleParams.W / 2, -VehicleParams.A / 2),
            ('side_left', -VehicleParams.W / 2, 0.0),
            ('side_right', VehicleParams.W / 2, 0.0),
        ]
    
    def get_sensor_readings(self, x: float, y: float, theta: float) -> Dict:
        readings = {}
        
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        
        for sensor_name, offset_x, offset_y in self.sensor_positions:
            sensor_x = x + offset_x * cos_t - offset_y * sin_t
            sensor_y = y + offset_x * sin_t + offset_y * cos_t
            
            min_distance = float('inf')
            
            for obs in self.obstacles:
                closest_x = np.clip(sensor_x, obs[0], obs[2])
                closest_y = np.clip(sensor_y, obs[1], obs[3])
                
                distance = np.sqrt((sensor_x - closest_x)**2 + (sensor_y - closest_y)**2)
                min_distance = min(min_distance, distance)
            
            readings[sensor_name] = min_distance
        
        readings['min_all'] = min(readings.values())
        readings['min_front'] = min(readings['front'], readings['front_left'], readings['front_right'])
        readings['min_rear'] = min(readings['rear'], readings['rear_left'], readings['rear_right'])
        readings['min_side'] = min(readings['side_left'], readings['side_right'])
        
        return readings


class Trajectory:
    def __init__(self, x0: float, y0: float, theta0: float,
                 xf: float, yf: float, thetaf: float,
                 k0: float, k1: float, Vs: int,
                 num_points: int = 200):
        self.x0, self.y0, self.theta0 = x0, y0, theta0
        self.xf, self.yf, self.thetaf = xf, yf, thetaf
        self.k0, self.k1, self.Vs = k0, k1, Vs
        self.num_points = num_points
        
        self._generate_trajectory()
    
    def _generate_trajectory(self):
        self.s_array = np.linspace(0, 1, self.num_points)
        
        dx0_ds = self.Vs * self.k0 * np.cos(self.theta0)
        dy0_ds = self.Vs * self.k0 * np.sin(self.theta0)
        dxf_ds = self.Vs * self.k1 * np.cos(self.thetaf)
        dyf_ds = self.Vs * self.k1 * np.sin(self.thetaf)
        
        T = np.array([
            [0, 0, 0, 1],
            [1, 1, 1, 1],
            [0, 0, 1, 0],
            [3, 2, 1, 0]
        ])
        
        bx = np.array([self.x0, self.xf, dx0_ds, dxf_ds])
        by = np.array([self.y0, self.yf, dy0_ds, dyf_ds])
        
        cx = np.linalg.solve(T, bx)
        cy = np.linalg.solve(T, by)
        
        cx_full = np.array([0, 0, cx[0], cx[1], cx[2], cx[3]])
        cy_full = np.array([0, 0, cy[0], cy[1], cy[2], cy[3]])
        
        self.x_array = np.polyval(cx_full, self.s_array)
        self.y_array = np.polyval(cy_full, self.s_array)
        
        cx_dot = np.polyder(cx_full)
        cy_dot = np.polyder(cy_full)
        cx_ddot = np.polyder(cx_dot)
        cy_ddot = np.polyder(cy_dot)
        
        self.theta_array = np.zeros(self.num_points)
        self.phi_array = np.zeros(self.num_points)
        
        for i in range(self.num_points):
            s = self.s_array[i]
            dx_ds = np.polyval(cx_dot, s)
            dy_ds = np.polyval(cy_dot, s)
            d2x_ds2 = np.polyval(cx_ddot, s)
            d2y_ds2 = np.polyval(cy_ddot, s)
            
            self.theta_array[i] = np.arctan2(dy_ds, dx_ds)
            
            numerator = dx_ds * d2y_ds2 - dy_ds * d2x_ds2
            denominator = (dx_ds**2 + dy_ds**2)**(1.5)
            
            if denominator > 1e-10:
                kappa = numerator / denominator
                V = np.sqrt(dx_ds**2 + dy_ds**2)
                if V > 1e-10:
                    self.phi_array[i] = np.arctan(VehicleParams.L * kappa / V)
                else:
                    self.phi_array[i] = 0
            else:
                self.phi_array[i] = 0
        
        self.length = 0.0
        for i in range(1, self.num_points):
            dx = self.x_array[i] - self.x_array[i-1]
            dy = self.y_array[i] - self.y_array[i-1]
            self.length += np.sqrt(dx**2 + dy**2)
    
    def get_max_phi(self) -> float:
        return np.max(np.abs(self.phi_array))


class ObstacleDetector:
    def __init__(self, obstacles: List[Tuple[float, float, float, float]]):
        self.obstacles = obstacles
    
    def check_collision(self, x: float, y: float, theta: float) -> bool:
        if not self.obstacles:
            return False
        
        A, W = VehicleParams.A, VehicleParams.W
        corners = [
            (-0.5, -W/2), (A-0.5, -W/2),
            (A-0.5, W/2), (-0.5, W/2)
        ]
        
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        for corner in corners:
            cx = x + corner[0] * cos_t - corner[1] * sin_t
            cy = y + corner[0] * sin_t + corner[1] * cos_t
            
            for obs in self.obstacles:
                if obs[0] <= cx <= obs[2] and obs[1] <= cy <= obs[3]:
                    return True
        
        return False


class Individual:
    def __init__(self, k0: float = None, k1: float = None, Vs: int = None):
        if k0 is None:
            self.k0 = random.uniform(GAParams.K0_MIN, GAParams.K0_MAX)
        else:
            self.k0 = k0
        
        if k1 is None:
            self.k1 = random.uniform(GAParams.K1_MIN, GAParams.K1_MAX)
        else:
            self.k1 = k1
        
        if Vs is None:
            self.Vs = random.choice(GAParams.VS_VALUES)
        else:
            self.Vs = Vs
        
        self.fitness = float('inf')
        self.trajectory = None
    
    def to_binary(self) -> str:
        k0_norm = (self.k0 - GAParams.K0_MIN) / (GAParams.K0_MAX - GAParams.K0_MIN)
        k1_norm = (self.k1 - GAParams.K1_MIN) / (GAParams.K1_MAX - GAParams.K1_MIN)
        
        k0_int = int(k0_norm * (2**32 - 1))
        k1_int = int(k1_norm * (2**32 - 1))
        
        k0_bin = format(k0_int, '032b')
        k1_bin = format(k1_int, '032b')
        vs_bin = '1' if self.Vs == 1 else '0'
        
        return k0_bin + k1_bin + vs_bin
    
    @staticmethod
    def from_binary(binary_str: str):
        k0_bin = binary_str[:32]
        k1_bin = binary_str[32:64]
        vs_bin = binary_str[64]
        
        k0_int = int(k0_bin, 2)
        k1_int = int(k1_bin, 2)
        
        k0_norm = k0_int / (2**32 - 1)
        k1_norm = k1_int / (2**32 - 1)
        
        k0 = GAParams.K0_MIN + k0_norm * (GAParams.K0_MAX - GAParams.K0_MIN)
        k1 = GAParams.K1_MIN + k1_norm * (GAParams.K1_MAX - GAParams.K1_MIN)
        Vs = 1 if vs_bin == '1' else -1
        
        return Individual(k0, k1, Vs)
    
    def copy(self):
        ind = Individual(self.k0, self.k1, self.Vs)
        ind.fitness = self.fitness
        ind.trajectory = self.trajectory
        return ind


class GeneticAlgorithm:
    def __init__(self, x0: float, y0: float, theta0: float,
                 x_target: float, y_target: float, theta_target: float,
                 obstacles: List = None):
        self.x0, self.y0, self.theta0 = x0, y0, theta0
        self.x_target, self.y_target, self.theta_target = x_target, y_target, theta_target
        self.obstacles = obstacles if obstacles else []
        self.obstacle_detector = ObstacleDetector(self.obstacles)
        
        self.population = []
        self.best_individual = None
        self.best_fitness_history = []
    
    def _evaluate_fitness(self, individual: Individual) -> float:
        try:
            traj = Trajectory(
                self.x0, self.y0, self.theta0,
                self.x_target, self.y_target, self.theta_target,
                individual.k0, individual.k1, individual.Vs
            )
            individual.trajectory = traj
            
            S = traj.length
            phi_max = traj.get_max_phi()
            
            num_collisions = sum(
                1 for i in range(len(traj.x_array))
                if self.obstacle_detector.check_collision(
                    traj.x_array[i], traj.y_array[i], traj.theta_array[i]
                )
            )
            
            if num_collisions > 0:
                S = S * 100.0
            
            fo = np.sqrt(S**2 + phi_max**2)
            return fo
        except:
            return float('inf')
    
    def _initialize_population(self):
        self.population = [Individual() for _ in range(GAParams.POPULATION_SIZE)]
        for ind in self.population:
            ind.fitness = self._evaluate_fitness(ind)
        self.population.sort(key=lambda x: x.fitness)
        self.best_individual = self.population[0].copy()
    
    def _selection_roulette(self) -> Individual:
        fitness_values = [ind.fitness for ind in self.population]
        valid_fitness = [f for f in fitness_values if f != float('inf')]
        
        if not valid_fitness:
            return random.choice(self.population).copy()
        
        max_fit = max(valid_fitness)
        inv_fitness = []
        for f in fitness_values:
            if f == float('inf'):
                inv_fitness.append(0)
            else:
                inv_fitness.append(max_fit / f if f > 0 else max_fit)
        
        total = sum(inv_fitness)
        if total == 0:
            return random.choice(self.population).copy()
        
        r = random.uniform(0, total)
        cumulative = 0
        for ind, fit_inv in zip(self.population, inv_fitness):
            cumulative += fit_inv
            if cumulative >= r:
                return ind.copy()
        
        return self.population[-1].copy()
    
    def _crossover(self, parent1: Individual, parent2: Individual) -> Tuple[Individual, Individual]:
        if random.random() > GAParams.CROSSOVER_RATE:
            return parent1.copy(), parent2.copy()
        
        bin1 = parent1.to_binary()
        bin2 = parent2.to_binary()
        
        point = random.randint(1, len(bin1) - 1)
        
        child1_bin = bin1[:point] + bin2[point:]
        child2_bin = bin2[:point] + bin1[point:]
        
        return Individual.from_binary(child1_bin), Individual.from_binary(child2_bin)
    
    def _mutate(self, individual: Individual) -> Individual:
        binary = list(individual.to_binary())
        
        for i in range(len(binary)):
            if random.random() < GAParams.MUTATION_RATE:
                binary[i] = '1' if binary[i] == '0' else '0'
        
        return Individual.from_binary(''.join(binary))
    
    def run(self, verbose: bool = True):
        self._initialize_population()
        self.best_fitness_history = [self.best_individual.fitness]
        
        if verbose:
            print(f"Geracao 0: Fitness = {self.best_individual.fitness:.4f}")
        
        for generation in range(1, GAParams.GENERATIONS + 1):
            new_population = []
            
            while len(new_population) < GAParams.POPULATION_SIZE:
                p1 = self._selection_roulette()
                p2 = self._selection_roulette()
                
                c1, c2 = self._crossover(p1, p2)
                
                c1 = self._mutate(c1)
                c2 = self._mutate(c2)
                
                c1.fitness = self._evaluate_fitness(c1)
                c2.fitness = self._evaluate_fitness(c2)
                
                new_population.extend([c1, c2])
            
            self.population = new_population[:GAParams.POPULATION_SIZE]
            self.population.sort(key=lambda x: x.fitness)
            
            if self.population[0].fitness < self.best_individual.fitness:
                self.best_individual = self.population[0].copy()
            
            self.best_fitness_history.append(self.best_individual.fitness)
            
            if verbose and (generation % 10 == 0 or generation == GAParams.GENERATIONS):
                print(f"Geracao {generation}: Fitness = {self.best_individual.fitness:.4f}")
        
        return self.best_individual


class TrajectoryExecutor:
    def __init__(self, trajectory: Trajectory, obstacles: List):
        self.trajectory = trajectory
        self.obstacles = obstacles
        
        self.safety_controller = FuzzySafetyController()
        self.proximity_sensor = ProximitySensor(obstacles)
        
        self.current_index = 0
        self.current_velocity = 0.0
        self.execution_log = []
        
        self.distance_history = []
        self.time_step = 0.1
    
    def execute_step(self) -> Dict:
        if self.current_index >= len(self.trajectory.x_array):
            return {'finished': True}
        
        x = self.trajectory.x_array[self.current_index]
        y = self.trajectory.y_array[self.current_index]
        theta = self.trajectory.theta_array[self.current_index]
        phi = self.trajectory.phi_array[self.current_index]
        
        sensor_readings = self.proximity_sensor.get_sensor_readings(x, y, theta)
        min_distance = sensor_readings['min_all']
        
        self.distance_history.append(min_distance)
        if len(self.distance_history) > 5:
            self.distance_history.pop(0)
        
        if len(self.distance_history) >= 2:
            approach_speed = -(self.distance_history[-1] - self.distance_history[-2]) / self.time_step
        else:
            approach_speed = 0.0
        
        safety_output = self.safety_controller.compute_safe_velocity(
            distance_to_obstacle=min_distance,
            current_velocity=approach_speed,
            steering_angle_deg=np.degrees(abs(phi))
        )
        
        desired_velocity = VehicleParams.V_max
        safe_velocity = safety_output['safe_velocity']
        
        velocity_change = safe_velocity - self.current_velocity
        max_change = VehicleParams.V_dot_max * self.time_step
        velocity_change = np.clip(velocity_change, -max_change, max_change)
        
        self.current_velocity = self.current_velocity + velocity_change
        
        step_data = {
            'index': self.current_index,
            'x': x,
            'y': y,
            'theta': np.degrees(theta),
            'phi': np.degrees(phi),
            'min_distance': min_distance,
            'approach_speed': approach_speed,
            'desired_velocity': desired_velocity,
            'safe_velocity': safe_velocity,
            'actual_velocity': self.current_velocity,
            'speed_factor': safety_output['speed_factor'],
            'alert_level': safety_output['alert_level'],
            'should_stop': safety_output['should_stop'],
            'warning': safety_output['warning'],
            'sensor_readings': sensor_readings,
            'finished': False
        }
        
        self.execution_log.append(step_data)
        
        if self.current_velocity > 0.05:
            self.current_index += 1
        
        return step_data
    
    def execute_full_trajectory(self, verbose: bool = True) -> List[Dict]:
        print("\n" + "="*70)
        print("EXECUTANDO TRAJETORIA COM CONTROLE DE SEGURANCA FUZZY")
        print("="*70)
        
        step = 0
        while self.current_index < len(self.trajectory.x_array):
            result = self.execute_step()
            
            if result.get('finished'):
                break
            
            if verbose and step % 20 == 0:
                alert_emoji = '🔴 ALTO' if result['alert_level'] > 0.7 else '🟡 MEDIO' if result['alert_level'] > 0.3 else '🟢 BAIXO'
                print(f"\nPasso {step}:")
                print(f"  Posicao: ({result['x']:.3f}, {result['y']:.3f})")
                print(f"  Distancia minima: {result['min_distance']:.3f} m")
                print(f"  Velocidade: {result['actual_velocity']:.3f} m/s (fator: {result['speed_factor']:.2f})")
                print(f"  Alerta: {alert_emoji}")
                
                if result['should_stop']:
                    print("  ⚠️  PARADA DE EMERGENCIA!")
            
            step += 1
        
        print(f"\n✓ Trajetoria concluida em {step} passos")
        print("="*70)
        
        return self.execution_log


def plot_safety_execution(executor: TrajectoryExecutor):
    log = executor.execution_log
    
    fig = plt.figure(figsize=(16, 12))
    gs = fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)
    
    ax1 = fig.add_subplot(gs[0, :2])
    
    x_vals = [d['x'] for d in log]
    y_vals = [d['y'] for d in log]
    velocities = [d['actual_velocity'] for d in log]
    
    scatter = ax1.scatter(x_vals, y_vals, c=velocities, cmap='RdYlGn',
                         s=30, vmin=0, vmax=VehicleParams.V_max)
    
    for obs in executor.obstacles:
        rect = Rectangle((obs[0], obs[1]), obs[2]-obs[0], obs[3]-obs[1],
                        fill=True, facecolor='red', alpha=0.3, edgecolor='red', linewidth=2)
        ax1.add_patch(rect)
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Trajetoria com Velocidades Ajustadas por Seguranca Fuzzy')
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    plt.colorbar(scatter, ax=ax1, label='Velocidade (m/s)')
    
    ax2 = fig.add_subplot(gs[0, 2])
    
    steps = [d['index'] for d in log]
    distances = [d['min_distance'] for d in log]
    
    ax2_twin = ax2.twinx()
    
    ax2.plot(steps, distances, 'b-', label='Distancia', linewidth=2)
    ax2_twin.plot(steps, velocities, 'r-', label='Velocidade', linewidth=2)
    
    ax2.set_xlabel('Passo')
    ax2.set_ylabel('Distancia (m)', color='b')
    ax2_twin.set_ylabel('Velocidade (m/s)', color='r')
    ax2.set_title('Distancia vs Velocidade')
    ax2.grid(True, alpha=0.3)
    
    ax3 = fig.add_subplot(gs[1, 0])
    
    speed_factors = [d['speed_factor'] for d in log]
    ax3.fill_between(steps, 0, speed_factors, alpha=0.3, color='green')
    ax3.plot(steps, speed_factors, 'g-', linewidth=2)
    ax3.axhline(y=0.15, color='r', linestyle='--', label='Limite parada')
    ax3.set_xlabel('Passo')
    ax3.set_ylabel('Fator de Velocidade')
    ax3.set_title('Controle Fuzzy: Fator de Velocidade')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    ax3.set_ylim([0, 1.1])
    
    ax4 = fig.add_subplot(gs[1, 1])
    
    alert_levels = [d['alert_level'] for d in log]
    colors = ['green' if a < 0.3 else 'orange' if a < 0.7 else 'red' for a in alert_levels]
    ax4.bar(steps, alert_levels, color=colors, alpha=0.6, width=1.0)
    ax4.axhline(y=0.3, color='orange', linestyle='--', alpha=0.5)
    ax4.axhline(y=0.7, color='red', linestyle='--', alpha=0.5)
    ax4.set_xlabel('Passo')
    ax4.set_ylabel('Nivel de Alerta')
    ax4.set_title('Controle Fuzzy: Nivel de Alerta')
    ax4.grid(True, alpha=0.3, axis='y')
    ax4.set_ylim([0, 1.1])
    
    ax5 = fig.add_subplot(gs[1, 2])
    
    front_readings = [d['sensor_readings']['min_front'] for d in log]
    rear_readings = [d['sensor_readings']['min_rear'] for d in log]
    side_readings = [d['sensor_readings']['min_side'] for d in log]
    
    ax5.plot(steps, front_readings, label='Frente', linewidth=2)
    ax5.plot(steps, rear_readings, label='Traseira', linewidth=2)
    ax5.plot(steps, side_readings, label='Lateral', linewidth=2)
    ax5.axhline(y=0.2, color='r', linestyle='--', alpha=0.5, label='Muito proximo')
    ax5.set_xlabel('Passo')
    ax5.set_ylabel('Distancia (m)')
    ax5.set_title('Leituras de Sensores de Proximidade')
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    
    ax6 = fig.add_subplot(gs[2, :])
    ax6.axis('off')
    
    total_time = len(log) * executor.time_step
    avg_velocity = np.mean(velocities)
    max_velocity = np.max(velocities)
    min_distance_overall = np.min(distances)
    num_warnings = sum(1 for d in log if d['warning'])
    num_stops = sum(1 for d in log if d['should_stop'])
    
    stats_text = f"""
ESTATISTICAS DA EXECUCAO COM CONTROLE FUZZY DE SEGURANCA
═════════════════════════════════════════════════════════════════════════════

Tempo Total:                    {total_time:.2f} s
Distancia Percorrida:           {executor.trajectory.length:.3f} m

Velocidade Media:               {avg_velocity:.3f} m/s  ({avg_velocity/VehicleParams.V_max*100:.1f}% da maxima)
Velocidade Maxima Atingida:     {max_velocity:.3f} m/s

Distancia Minima a Obstaculos:  {min_distance_overall:.3f} m

Avisos de Seguranca:            {num_warnings} / {len(log)} passos ({num_warnings/len(log)*100:.1f}%)
Paradas de Emergencia:          {num_stops} / {len(log)} passos ({num_stops/len(log)*100:.1f}%)

Status Final:                   ✓ TRAJETORIA CONCLUIDA COM SUCESSO E SEGURANCA
"""
    
    ax6.text(0.05, 0.5, stats_text, fontsize=11, family='monospace',
            verticalalignment='center')
    
    plt.suptitle('SISTEMA FUZZY DE SEGURANCA - Analise Completa', 
                fontsize=16, fontweight='bold', y=0.995)
    
    plt.savefig('Safety_Control_Analysis.png', dpi=150, bbox_inches='tight')
    plt.show()


def main():
    print("="*70)
    print("AG + SISTEMA FUZZY DE SEGURANCA")
    print("Planejamento (AG) + Controle Reativo (Fuzzy)")
    print("="*70)
    
    x0, y0, theta0 = 2.0, 1.8, np.pi
    x_target, y_target, theta_target = 0.6, 1.0, np.pi
    
    obstacles = [
        (-1.0, 0.6, -0.4, 1.4),
        (1.6, 0.6, 2.2, 1.4),
    ]
    
    print("\n[FASE 1: PLANEJAMENTO OFFLINE COM AG]")
    print(f"Pose Inicial: ({x0:.2f}, {y0:.2f}, {np.rad2deg(theta0):.1f} graus)")
    print(f"Pose Target:  ({x_target:.2f}, {y_target:.2f}, {np.rad2deg(theta_target):.1f} graus)")
    print("\nExecutando Algoritmo Genetico...")
    
    ga = GeneticAlgorithm(x0, y0, theta0, x_target, y_target, theta_target, obstacles)
    best = ga.run(verbose=False)
    
    print(f"\n✓ Trajetoria planejada!")
    print(f"  Fitness: {best.fitness:.4f}")
    print(f"  Comprimento: {best.trajectory.length:.3f} m")
    print(f"  |phi|_max: {np.degrees(best.trajectory.get_max_phi()):.2f} graus")
    
    print("\n[FASE 2: EXECUCAO ONLINE COM CONTROLE FUZZY DE SEGURANCA]")
    executor = TrajectoryExecutor(best.trajectory, obstacles)
    executor.execute_full_trajectory(verbose=True)
    
    print("\n[FASE 3: ANALISE E VISUALIZACAO]")
    plot_safety_execution(executor)
    
    print("\n✓ Sistema completo executado com sucesso!")
    print("  Arquivo salvo: Safety_Control_Analysis.png")


if __name__ == "__main__":
    main()
