import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from typing import List, Tuple
import random


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
                    self.phi_array[i] = np.arctan(VehicleParams.L * kappa / V) # Aproximação ds/dt
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
        
        self.DP = 1e9
        A = VehicleParams.A
        B = VehicleParams.B
        
        self.r_side = np.sqrt((self.DP + B)**2 + (A/2)**2)
        self.r_front_back = np.sqrt((self.DP + A)**2 + (B/2)**2)
        
        self.centers_local = [
            (0.0, -(self.DP + B/2)),
            (0.0, +(self.DP + B/2)),
            (-(self.DP + A/2), 0.0),
            (+(self.DP + A/2), 0.0)
        ]
        
        self.radii = [self.r_side, self.r_side, self.r_front_back, self.r_front_back]

    def _dist_point_rect(self, px, py, rx_min, ry_min, rx_max, ry_max):
        """Distância euclidiana mínima de um ponto a um retângulo."""
        closest_x = max(rx_min, min(px, rx_max))
        closest_y = max(ry_min, min(py, ry_max))
        dx = px - closest_x
        dy = py - closest_y
        return np.sqrt(dx*dx + dy*dy)

    def check_collision(self, x_veh: float, y_veh: float, theta_veh: float) -> bool:
    
        if not self.obstacles:
            return False
            
        cos_t = np.cos(theta_veh)
        sin_t = np.sin(theta_veh)
        
        for obs in self.obstacles:
            ox_min, oy_min, ox_max, oy_max = obs
            
            is_collision_candidate = True
            
            for i in range(4):
                lx, ly = self.centers_local[i]
                r = self.radii[i]
                
                px_global = x_veh + (lx * cos_t - ly * sin_t)
                py_global = y_veh + (lx * sin_t + ly * cos_t)
                
                dist = self._dist_point_rect(px_global, py_global, ox_min, oy_min, ox_max, oy_max)
                
                if dist > r:
                    is_collision_candidate = False
                    break 
            
            if is_collision_candidate:
                return True
                
        return False
    
    def count_collisions(self, trajectory: Trajectory) -> int:
        count = 0
        for i in range(len(trajectory.x_array)):
            if self.check_collision(
                trajectory.x_array[i], 
                trajectory.y_array[i], 
                trajectory.theta_array[i]
            ):
                count += 1
        return count


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
            
            has_collision = any(
                self.obstacle_detector.check_collision(
                    traj.x_array[i], traj.y_array[i], traj.theta_array[i]
                )
                for i in range(len(traj.x_array))
            )
            
            if has_collision:
                S = S * 100.0
            
            fo = np.sqrt(S**2 + phi_max**2)
            
            return fo
        
        except Exception as e:
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
            print(f"Geração 0: Fitness = {self.best_individual.fitness:.4f}")
            print(f"  k0={self.best_individual.k0:.3f}, k1={self.best_individual.k1:.3f}, Vs={self.best_individual.Vs}")
        
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
                print(f"Geração {generation}: Fitness = {self.best_individual.fitness:.4f}")
                print(f"  k0={self.best_individual.k0:.3f}, k1={self.best_individual.k1:.3f}, Vs={self.best_individual.Vs}")
        
        return self.best_individual


def plot_trajectory(traj: Trajectory, title: str, 
                            x0: float, y0: float, x_target: float, y_target: float,
                            parking_space=None, obstacles=None):
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    ax = axes[0, 0]
    ax.plot(traj.x_array, traj.y_array, 'b-', linewidth=2, label='Trajetória')
    ax.plot(x0, y0, 'go', markersize=12, label='Início')
    ax.plot(x_target, y_target, 'rs', markersize=12, label='Target')
    ax.plot(traj.x_array[-1], traj.y_array[-1], 'mo', markersize=10, label='Final')
    
    if parking_space:
        rect = Rectangle(parking_space[:2], parking_space[2], parking_space[3],
                        fill=False, edgecolor='green', linewidth=2, linestyle='--')
        ax.add_patch(rect)
    
    if obstacles:
        for obs in obstacles:
            rect = Rectangle((obs[0], obs[1]), obs[2]-obs[0], obs[3]-obs[1],
                           fill=True, facecolor='red', alpha=0.3)
            ax.add_patch(rect)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Trajetória (Detecção via 4 Círculos)')
    ax.legend()
    ax.grid(True)
    ax.axis('equal')
    
    ax = axes[0, 1]
    phi_deg = np.degrees(traj.phi_array)
    ax.plot(traj.s_array, phi_deg, 'b-', linewidth=2)
    ax.axhline(y=VehicleParams.phi_max_deg, color='r', linestyle='--', label='φ_max')
    ax.axhline(y=-VehicleParams.phi_max_deg, color='r', linestyle='--')
    ax.set_xlabel('s')
    ax.set_ylabel('φ(s) (graus)')
    ax.set_title(f'Ângulo de esterçamento: |φ|_max = {np.max(np.abs(phi_deg)):.2f}°')
    ax.legend()
    ax.grid(True)
    
    ax = axes[1, 0]
    theta_deg = np.degrees(traj.theta_array)
    ax.plot(traj.s_array, theta_deg, 'g-', linewidth=2)
    ax.set_xlabel('s')
    ax.set_ylabel('θ(s) (graus)')
    ax.set_title('Orientação do veículo')
    ax.grid(True)
    
    ax = axes[1, 1]
    ax.axis('off')
    
    info = f"""
TRAJETÓRIA (Equações 32-33)

x(s) = Σ Cxi·s^i  (i=0 a 5)
y(s) = Σ Cyi·s^i  (i=0 a 5)

Genes:
k0 = {traj.k0:.3f}
k1 = {traj.k1:.3f}
Vs = {traj.Vs:+d}

Métricas:
Comprimento |S| = {traj.length:.3f} m
|φ|_max = {np.max(np.abs(phi_deg)):.2f}°
"""
    ax.text(0.1, 0.5, info, fontsize=12, family='monospace', verticalalignment='center')
    
    plt.tight_layout()
    plt.savefig(f'{title.replace(" ", "_")}.png', dpi=150)
    plt.show()


def parallel_parking_article():
    print("\n" + "="*70)
    print("ESTACIONAMENTO PARALELO - ALGORITMO EXATO DO ARTIGO")
    print("="*70)
    print("\n[IMPLEMENTACAO METODOLOGICA:]")
    print("  * Otimização: Algoritmo Genético (AG)")
    print("  * Trajetória: Polinômios de 5ª ordem (Quintic) em s(t)")
    print("  * Detecção Colisão: 4 Círculos Gigantes (Eq. 15-18 - Método Analítico)")
    print("  * Genes: k0, k1, Vs")
    print("="*70)
    
    x0, y0, theta0 = 2.0, 1.8, np.pi
    x_target, y_target, theta_target = 0.6, 1.0, np.pi
    
    obstacles = [
        (-1.0, 0.6, -0.4, 1.4),
        (1.6, 0.6, 2.2, 1.4),
    ]
    
    parking_space = (0.0, 0.7, 1.2, 0.6)
    
    print(f"\nPose Inicial: ({x0:.2f}, {y0:.2f}, {np.rad2deg(theta0):.1f}°)")
    print(f"Pose Target:  ({x_target:.2f}, {y_target:.2f}, {np.rad2deg(theta_target):.1f}°)")
    print(f"\nExecutando AG com Detecção por 4 Círculos (DP=1e9)...\n")
    
    ga = GeneticAlgorithm(x0, y0, theta0, x_target, y_target, theta_target, obstacles)
    best = ga.run(verbose=True)
    
    print(f"\n" + "-"*70)
    print("RESULTADOS FINAIS")
    print("-"*70)
    
    traj = best.trajectory
    xf, yf, thetaf = traj.x_array[-1], traj.y_array[-1], traj.theta_array[-1]
    phi_max_deg = np.max(np.abs(np.degrees(traj.phi_array)))
    
    print(f"Fitness (Minimizado): {best.fitness:.4f}")
    print(f"Comprimento da Trajetória |S|: {traj.length:.3f} m")
    print(f"Ângulo de Esterçamento Máx |φ|_max: {phi_max_deg:.2f} graus [limite: {VehicleParams.phi_max_deg} graus]")
    
    print(f"\nValidação da Pose Final:")
    print(f"  Desejado: ({x_target:.3f}, {y_target:.3f}, {np.rad2deg(theta_target):.1f}°)")
    print(f"  Obtido:   ({xf:.3f}, {yf:.3f}, {np.rad2deg(thetaf):.1f}°)")
    
    error_pos = np.sqrt((xf-x_target)**2 + (yf-y_target)**2)
    
    dtheta = thetaf - theta_target
    while dtheta > np.pi: dtheta -= 2*np.pi
    while dtheta < -np.pi: dtheta += 2*np.pi
    error_theta = np.rad2deg(abs(dtheta))
    
    print(f"  Erro Posição: {error_pos:.3e} m")
    print(f"  Erro Orientação: {error_theta:.3e} graus")
    
    print(f"\nMelhores Genes Encontrados (Parâmetros da Curva):")
    print(f"  k0 = {best.k0:.3f}")
    print(f"  k1 = {best.k1:.3f}")
    print(f"  Vs = {best.Vs:+d} (1=Frente, -1=Ré)")
    
    num_col = ga.obstacle_detector.count_collisions(traj)
    print(f"\nVerificação de Segurança:")
    print(f"  Colisões Detectadas: {num_col}/{len(traj.x_array)} pontos ({100*num_col/len(traj.x_array):.1f}%)")
    
    plot_trajectory(traj, "Estacionamento_Paralelo_Final",
                   x0, y0, x_target, y_target, parking_space, obstacles)
    
    plt.figure(figsize=(10, 6))
    plt.plot(ga.best_fitness_history, 'b-', linewidth=2)
    plt.xlabel('Geração')
    plt.ylabel('Melhor Fitness (fo)')
    plt.title('Convergência do AG (Minimização)')
    plt.grid(True)
    plt.yscale('log')
    plt.savefig('Convergencia_AG_Final.png', dpi=150)
    plt.show()

if __name__ == "__main__":
    parallel_parking_article()