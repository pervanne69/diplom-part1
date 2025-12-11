from mesa import Model
from mesa.time import SimultaneousActivation
from mesa.space import MultiGrid
from mesa.datacollection import DataCollector

import random
from typing import List, Tuple

from .agent import RobotAgent, Obstacle, Task
from .map_generator import generate_grid

from .task_allocation import hungarian_assignment, greedy_assignment, cbba_assignment, assignment_to_tasks
from .coop_astar import cooperative_a_star
from .prioritized_planning import prioritized_planning
from .cbs import cbs


class GridMASModel(Model):
    def __init__(
        self,
        width=20,
        height=15,
        num_agents=3,
        num_tasks=5,
        obstacle_prob=0.15,
        planner="prioritized",
        pp_priority="id",
        seed=None,
        mrta_method="hungarian",
        dynamic_tasks=True,
        task_spawn_prob=0.02
    ):
        super().__init__()
        self.width = width
        self.height = height
        self.num_agents = num_agents
        self.planner = planner
        self.pp_priority = pp_priority
        self.seed = seed
        self.mrta_method = mrta_method
        self.dynamic_tasks = dynamic_tasks
        self.task_spawn_prob = task_spawn_prob

        if seed is not None:
            random.seed(seed)

        self.grid_map = generate_grid(width, height, obstacle_prob, seed)
        self.grid = MultiGrid(width, height, torus=False)
        self.schedule = SimultaneousActivation(self)

        # Препятствия
        for y in range(height):
            for x in range(width):
                if self.grid_map[y][x] == 1:
                    obs = Obstacle((x, y), self)
                    self.grid.place_agent(obs, (x, y))

        # Задачи
        self.tasks: List[Task] = []
        for tid in range(num_tasks):
            pos = self._get_random_free_cell()
            task = Task(tid, pos, self)
            self.tasks.append(task)
            self.grid.place_agent(task, pos)

        # Агенты
        self.agents_list: List[RobotAgent] = []
        for agent_id in range(num_agents):
            start = self._get_random_free_cell()
            ag = RobotAgent(agent_id, self, start)
            self.agents_list.append(ag)
            self.schedule.add(ag)
            self.grid.place_agent(ag, start)

        # MRTA
        self._assign_tasks()

        # MAPF
        self._compute_paths()

        # DataCollector
        self.datacollector = DataCollector(
            model_reporters={
                "SoC": lambda m: sum(len(a.path) for a in m.agents_list),
                "Makespan": lambda m: max((len(a.path) for a in m.agents_list), default=0),
                "RemainingTasks": lambda m: len([t for t in m.tasks if not t.completed])
            },
            agent_reporters={}
        )
        self.running = True

    # ----------------------------------------------------------------------
    def _get_random_free_cell(self) -> Tuple[int, int]:
        """Возвращает случайную свободную клетку, без препятствий, роботов и задач."""
        while True:
            x = random.randint(0, self.width - 1)
            y = random.randint(0, self.height - 1)
            if self.grid_map[y][x] == 0:
                contents = self.grid.get_cell_list_contents((x, y))
                # нет роботов и нет задач
                if not any(isinstance(a, RobotAgent) for a in contents) and \
                        not any(getattr(a, "agent_type", None) == "task" for a in contents):
                    return (x, y)

    def _assign_tasks(self):
        """Назначение задач агентам через MRTA"""
        if not self.tasks:
            return
        if self.mrta_method == "hungarian":
            assignment = hungarian_assignment(self.agents_list, self.tasks)
        elif self.mrta_method == "greedy":
            assignment = greedy_assignment(self.agents_list, self.tasks)
        elif self.mrta_method == "cbba":
            assignment = cbba_assignment(self.agents_list, self.tasks)
        else:
            raise ValueError(f"Unknown MRTA method: {self.mrta_method}")

        # преобразуем assignment agent_id -> Task объект
        assignment_obj = assignment_to_tasks(assignment, self.tasks)
        for ag in self.agents_list:
            if ag.unique_id in assignment_obj:
                ag.assign_tasks([assignment_obj[ag.unique_id]])

    def _compute_paths(self):
        """MAPF для всех агентов"""
        agent_specs = []
        for ag in self.agents_list:
            if ag.goal_task and not ag.finished:
                agent_specs.append({
                    "id": ag.unique_id,
                    "start": list(ag.pos),
                    "goal": list(ag.goal_task.pos)
                })

        if not agent_specs:
            return

        if self.planner == "prioritized":
            plans = prioritized_planning(self.grid_map, agent_specs, priority=self.pp_priority)
        elif self.planner == "cbs":
            plans = cbs(self.grid_map, agent_specs)
        else:
            plans = cooperative_a_star(self.grid_map, agent_specs)

        for ag in self.agents_list:
            if ag.unique_id in plans:
                ag.path = plans[ag.unique_id]
                ag.path_step = 0
                ag.finished = (ag.pos == ag.goal_task.pos)

    # ----------------------------------------------------------------------
    def step(self):
        self.datacollector.collect(self)

        # динамические задачи
        if self.dynamic_tasks and random.random() < self.task_spawn_prob:
            pos = self._get_random_free_cell()
            task = Task(len(self.tasks), pos, self)
            self.tasks.append(task)
            self.grid.place_agent(task, pos)
            self._assign_tasks()
            self._compute_paths()

        self.schedule.step()