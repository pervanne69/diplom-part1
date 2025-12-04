from mesa import Model
from mesa.time import SimultaneousActivation
from mesa.space import MultiGrid
from .agent import RobotAgent
from .coop_astar import cooperative_a_star
from .prioritized_planning import prioritized_planning
from .cbs import cbs
import json
from typing import Dict, List, Tuple

class GridMASModel(Model):
    """
        Наименование: GridMASModel
        Назначение:
            Описывает симуляционную модель мультиагентной системы (МАС),
            работающей на двумерной сетке. Модель загружает сценарий,
            создаёт агентов, рассчитывает маршруты с помощью Cooperative A*,
            размещает роботов в начальных позициях и выполняет симуляцию.

        Входные параметры:
            scenario_path (str) — путь к JSON-файлу сценария, содержащему:
                - grid (2D-массив 0/1) — карту препятствий;
                - agents — список агентов: id, start, goal.

        Атрибуты:
            width (int) — ширина сетки.
            height (int) — высота сетки.
            grid_map (List[List[int]]) — карта препятствий.
            grid (MultiGrid) — сетка Mesa для размещения агентов.
            schedule (SimultaneousActivation) — планировщик действий Mesa.
            agents_list (List[RobotAgent]) — список всех созданных агентов.
            running (bool) — флаг продолжения симуляции.

        Поведение:
            - При инициализации загружает сценарий, создаёт агентов,
              рассчитывает пути и размещает их на сетке.
            - В методе step() выполняется один шаг симуляции.
    """

    def __init__(self, scenario_path: str, planner: str = "prioritized", pp_priority: str = "id"):
        super().__init__()

        with open(scenario_path, "r") as f:
            data = json.load(f)

        # Двумерная карта сетки (0 - свободно, 1 - препятствие)
        grid_map: List[List[int]] = data["grid"]

        # Размеры сетки
        self.width: int = len(grid_map[0])
        self.height: int = len(grid_map)

        # сохраняем карту как атрибут модели
        self.grid_map: List[List[int]] = grid_map

        # Создаём сетку Mesa
        # MultiGrid позволяет размещать несколько агентов в одной ячейке
        # (в этом проекте можно разрешить совместное пребывание для простоты)
        self.grid = MultiGrid(self.width, self.height, torus=False)

        # Планировщик шагов — SimultaneousActivation выполняет step() у всех агентов одновременно
        self.schedule = SimultaneousActivation(self)

        # Список агнетов
        self.agents_list: List[RobotAgent] = []

        # Создание агентов по сценарию
        for agent_info in data["agents"]:
            agent = RobotAgent(
                unique_id=agent_info["id"],
                model=self,
                start=agent_info["start"],
                goal=agent_info["goal"]
            )

            # Добавляем в список
            self.agents_list.append(agent)

            # Регистрируем агента у планировщика
            self.schedule.add(agent)

            # Размещаем агента на сетке в начальной точке
            x, y = agent_info["start"]
            self.grid.place_agent(agent, (x, y))

        # Инициализация маршрутов агентов через Cooperative A*
        if planner == "prioritized":
            plans = prioritized_planning(self.grid_map, data["agents"], priority=pp_priority)
        elif planner == "cbs":
            plans = cbs(self.grid_map, data["agents"])
        else:
            plans = cooperative_a_star(self.grid_map, data["agents"])

        # Присваиваем каждому агенту его маршрут
        for agent in self.agents_list:
            agent.path = plans.get(agent.unique_id, [agent.start])
            agent.path_step = 0
            agent.finished = (agent.start == agent.goal)

        # Флаг продолжающейся симуляции
        self.running: bool = True

    # Основной шаг симуляции
    def step(self) -> None:
        """
        Наименование: step
        Назначение:
            Выполняет один шаг симуляции:
            - вызывает step() у всех агентов через планировщик,
            - проверяет, завершили ли движение все агенты.

        Входные параметры:
            отсутствуют.

        Возвращаемое значение:
            None
        """

        # Выполняем шаг для всех агентов
        self.schedule.step()

        # Проверяем, достигли ли все цели
        all_completed = all(agent.finished for agent in self.agents_list)

        # Если все движения завершены - останавливаем симуляцию
        if all_completed:
            self.running = False

