from mesa import Model
from mesa.time import SimultaneousActivation
from mesa.space import MultiGrid
from .agent import RobotAgent, Obstacle
from .coop_astar import cooperative_a_star
from .prioritized_planning import prioritized_planning
from .cbs import cbs
from .map_generator import generate_grid
import random
from typing import List

class GridMASModel(Model):
    """
    Динамическая симуляционная модель мультиагентной системы.
    Генерация препятствий и маршрутов агентов происходит при каждом запуске.
    """

    def __init__(self, width=20, height=15, num_agents=3,
                 obstacle_prob=0.15, planner="prioritized",
                 pp_priority="id", seed=None):
        super().__init__()
        self.width = width
        self.height = height
        self.num_agents = num_agents
        self.planner = planner
        self.pp_priority = pp_priority
        self.seed = seed

        # Фиксируем seed для воспроизводимости
        if seed is not None:
            random.seed(seed)

        # Генерация карты препятствий
        self.grid_map = generate_grid(width, height, obstacle_prob, seed)

        # Создаём сетку Mesa
        self.grid = MultiGrid(width, height, torus=False)

        # Планировщик шагов
        self.schedule = SimultaneousActivation(self)

        # Размещение препятствий на сетке
        for y in range(height):
            for x in range(width):
                if self.grid_map[y][x] == 1:
                    obs = Obstacle((x, y), self)
                    self.grid.place_agent(obs, (x, y))

        # Создание агентов
        self.agents_list: List[RobotAgent] = []
        agents_info = []
        for a in range(num_agents):
            start = self._get_random_free_cell()
            goal = self._get_random_free_cell()
            agent = RobotAgent(unique_id=a, model=self, start=start, goal=goal)
            self.agents_list.append(agent)
            self.schedule.add(agent)
            self.grid.place_agent(agent, tuple(start))
            agents_info.append({"id": a, "start": start, "goal": goal})

        # Планирование маршрутов
        if planner == "prioritized":
            plans = prioritized_planning(self.grid_map, agents_info, priority=pp_priority)
        elif planner == "cbs":
            plans = cbs(self.grid_map, agents_info)
        else:
            plans = cooperative_a_star(self.grid_map, agents_info)

        # Присваиваем каждому агенту его маршрут
        for agent in self.agents_list:
            agent.path = plans.get(agent.unique_id, [agent.start])
            agent.path_step = 0
            agent.finished = (agent.start == agent.goal)

        # Флаг продолжающейся симуляции
        self.running = True

    def _get_random_free_cell(self):
        """Находит случайную свободную клетку на карте"""
        while True:
            x = random.randint(0, self.width - 1)
            y = random.randint(0, self.height - 1)
            # свободная клетка без препятствия и без агента
            if self.grid_map[y][x] == 0 and not any(isinstance(a, RobotAgent) for a in self.grid.get_cell_list_contents((x, y))):
                return [x, y]

    def step(self):
        """Выполняет один шаг симуляции"""
        self.schedule.step()
        if all(agent.finished for agent in self.agents_list):
            self.running = False