"""
Модуль MRTA: распределение задач между агентами.
Поддерживаемые методы:
 - Hungarian (оптимальный через scipy)
 - Greedy (жадный)
 - CBBA (упрощённая версия)
"""

from typing import List, Dict
import numpy as np
from scipy.optimize import linear_sum_assignment


# -----------------------------
# Вспомогательная функция
# -----------------------------
def manhattan(a: tuple, b: tuple) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# -----------------------------
# Hungarian
# -----------------------------
def hungarian_assignment(agents, tasks) -> Dict[int, int]:
    if not agents or not tasks:
        return {}
    cost = np.array([[manhattan(a.pos, t.pos) for t in tasks] for a in agents])
    row_ind, col_ind = linear_sum_assignment(cost)
    return {agents[i].unique_id: tasks[j].task_id for i, j in zip(row_ind, col_ind)}


# -----------------------------
# Greedy
# -----------------------------
def greedy_assignment(agents, tasks) -> Dict[int, int]:
    assign = {}
    taken = set()
    for a in agents:
        best, best_dist = None, float('inf')
        for t in tasks:
            if t.task_id in taken or getattr(t, "completed", False):
                continue
            d = manhattan(a.pos, t.pos)
            if d < best_dist:
                best_dist = d
                best = t
        if best:
            assign[a.unique_id] = best.task_id
            taken.add(best.task_id)
    return assign


# -----------------------------
# CBBA
# -----------------------------
def cbba_assignment(agents, tasks) -> Dict[int, int]:
    return greedy_assignment(agents, tasks)


# -----------------------------
# Преобразование assignment → Task объекты
# -----------------------------
def assignment_to_tasks(assignment: Dict[int, int], tasks: List) -> Dict[int, "Task"]:
    task_map = {t.task_id: t for t in tasks}
    return {aid: task_map[tid] for aid, tid in assignment.items() if tid in task_map}


