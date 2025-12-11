from mesa import Agent
from typing import List, Tuple, Optional


class Obstacle(Agent):
    """Препятствие."""
    def __init__(self, pos: Tuple[int, int], model):
        super().__init__("obs_" + str(pos), model)
        self.pos = pos
        self.agent_type = "obstacle"


class Task(Agent):
    """Агент-задача."""
    def __init__(self, task_id: int, pos: Tuple[int, int], model):
        super().__init__("task_" + str(task_id), model)
        self.pos = tuple(pos)
        self.task_id = task_id
        self.agent_type = "task"
        self.completed = False


class RobotAgent(Agent):
    """Робот с очередью задач."""
    def __init__(self, unique_id: int, model, start: List[int]):
        super().__init__(unique_id, model)

        self.pos: Tuple[int, int] = tuple(start)
        self.start: Tuple[int, int] = tuple(start)

        self.goal_task: Optional[Task] = None
        self.goal: Optional[Tuple[int, int]] = None

        self.path: List[Tuple[int, int]] = []
        self.path_step: int = 0

        self.task_queue: List[Task] = []
        self.finished: bool = False

    def assign_tasks(self, tasks: List[Task]):
        """Назначение задач (обычно 1 после MRTA)."""
        self.task_queue = [t for t in tasks if not t.completed]
        self._set_next_goal()

    def _set_next_goal(self):
        """Берём следующую задачу."""
        if not self.task_queue:
            self.goal = None
            self.goal_task = None
            self.finished = True
            return

        task = self.task_queue.pop(0)
        if task.completed:
            return self._set_next_goal()

        self.goal_task = task
        self.goal = task.pos
        self.path = []
        self.path_step = 0
        self.finished = False

    def step(self):
        """Движение по маршруту и выполнение задачи."""
        if self.finished or self.goal is None:
            return

        # движение по пути
        if self.path_step < len(self.path):
            new_pos = self.path[self.path_step]
            self.model.grid.move_agent(self, new_pos)
            self.pos = new_pos
            self.path_step += 1

        # достигли цели
        if self.pos == self.goal and self.goal_task:
            # сохраняем координаты задачи
            task_pos = self.goal_task.pos
            # помечаем задачу выполненной
            self.goal_task.completed = True
            if self.goal_task in self.model.tasks:
                self.model.tasks.remove(self.goal_task)
            # удаляем задачу с карты
            self.model.grid.remove_agent(self.goal_task)
            self.goal_task = None
            self.goal = None

            # берём следующую задачу
            self._set_next_goal()