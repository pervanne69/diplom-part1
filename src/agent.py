from mesa import Agent
from typing import List, Tuple


class Obstacle(Agent):
    def __init__(self, pos, model):
        super().__init__("obs_" + str(pos), model)
        self.pos = pos
        self.agent_type = "obstacle"


class RobotAgent(Agent):
    """
        Наименование: RobotAgent
        Назначение:
            Класс агента-робота, перемещающегося по дискретной двумерной сетке.
            Агент следует заранее рассчитанному маршруту (списку координат)
            и выполняет движение по одной ячейке за шаг симуляции.

        Входные параметры конструктора:
            unique_id (int) — уникальный идентификатор агента в модели.
            model (mesa.Model) — ссылка на модель Mesa, которой принадлежит агент.
            start (List[int]) — начальная позиция агента [x, y].
            goal (List[int]) — целевая позиция агента [x, y].

        Атрибуты:
            pos (Tuple[int,int]) — текущая позиция агента в сетке.
            start (Tuple[int,int]) — начальная позиция агента.
            goal (Tuple[int,int]) — конечная позиция агента.
            path (List[Tuple[int,int]]) — маршрут агента, полученный планировщиком.
            path_step (int) — индекс текущего шага пути.
            finished (bool) — флаг, показывающий, что агент достиг цели.

        Поведение:
            На каждом шаге симуляции вызывает метод step(), который перемещает
            агента вдоль маршрута, если путь ещё не завершён.
        """

    def __init__(self, unique_id: int, model, start: List[int], goal: List[int]):
        super().__init__(unique_id, model)

        # Текущая позиция агента (кортеж (x,y))
        self.pos: Tuple[int, int] = tuple(start)

        # Целевая позиция агента
        self.goal: Tuple[int, int] = tuple(goal)

        # Путь агента - список координат (x, y), заполняется планировщиком
        self.path: List[Tuple[int, int]] = []

        # Индекс шага пути (начинается с 0)
        self.path_step: int = 0

        # Завершил ли агент движение к цели
        self.finished: bool = False

        self.start: Tuple[int, int] = tuple(start)

    # Основной метод шага агента
    def step(self) -> None:
        """
            Наименование: step
            Назначение:
                Осуществляет один шаг движения агента в симуляции.
                Если у агента есть путь и он ещё не достиг конца маршрута,
                то агент перемещается в следующую клетку пути.
                Если цель достигнута, флаг finished становится True.

            Входные параметры:
                отсутствуют.

            Возвращаемое значение:
                None
        """

        # Если агент уже достиг цели, значит ничего не делаем
        if self.finished:
            return

        # Если путь есть и индекс path_step еще в пределах длины списка
        if self.path and self.path_step < len(self.path):
            # Берем новое положение из пути
            new_pos = self.path[self.path_step]

            # Перемещаем агента в сетке модели (Mesa сама обновит координаты)
            self.model.grid.move_agent(self, new_pos)
            self.pos = new_pos # сохраняем новую позицию в атрибуте

            # Переходим к следующему шагу маршрута
            self.path_step += 1

            # Проверяем, достиг ли агент цели
            if self.pos == self.goal:
                self.finished = True # ставим флаг завершения пути

            else:
                # Если путь пустой или пройден полностью — агент стоит на месте
                # (в будущем можно добавить поведение ожидания или перепланирования)
                pass