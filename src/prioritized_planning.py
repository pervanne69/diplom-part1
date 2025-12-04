# prioritized_planning.py
# Улучшенная реализация Prioritized Planning (PP) для MAPF.
# Поддерживает vertex- и edge-reservation, разные стратегии приоритетов.
# Автор:
# Дата: 2025-11-28

import heapq
import random
from typing import List, Tuple, Dict, Set, Optional

# Допустимые движения (4-связность + wait)
MOVES = [(0, 1), (0, -1), (1, 0), (-1, 0), (0, 0)]

# -----------------------------------------------------------------------------
# Вспомогательные типы:
# Grid = List[List[int]]  (0 - свободно, 1 - препятствие)
# AgentSpec = {"id":int, "start":[x,y], "goal":[x,y]}
# -----------------------------------------------------------------------------

def heuristic(a: Tuple[int,int], b: Tuple[int,int]) -> int:
    """Манхэттеновская эвристика."""
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def in_bounds(pos: Tuple[int,int], width: int, height: int) -> bool:
    x,y = pos
    return 0 <= x < width and 0 <= y < height

def is_free(grid: List[List[int]], pos: Tuple[int,int]) -> bool:
    x,y = pos
    return grid[y][x] == 0

# ---------------- A* в пространстве-времени с поддержкой vertex/edge constraints ----------
def a_star_with_reservations(grid: List[List[int]],
                             start: Tuple[int,int],
                             goal: Tuple[int,int],
                             reserved_vertices: Dict[int, Set[Tuple[int,int]]],
                             reserved_edges: Dict[int, Set[Tuple[Tuple[int,int], Tuple[int,int]]]],
                             constraints: Dict[int, Set[Tuple[int,int]]] = None,
                             max_t: int = 500) -> Optional[List[Tuple[int,int]]]:
    """
    A* в пространстве-времени с проверкой:
      - vertex reservations: на момент t клетка не должна быть занята
      - edge reservations: переход (u->v) запрещён, если в том же шаге другой агент идёт v->u
      - constraints: словарь времени -> набор запрещённых вершин (для CBS)
    Возвращает путь как список (x,y) по времени или None.
    """
    width = len(grid[0]); height = len(grid)
    start_node = (start[0], start[1], 0)
    openpq = []
    heapq.heappush(openpq, (heuristic(start, goal), 0, start_node))
    came_from = {}
    gscore = {start_node: 0}

    while openpq:
        f, g, node = heapq.heappop(openpq)
        x,y,t = node
        # цель достигнута (позиция), возвращаем путь
        if (x,y) == goal:
            # reconstruct
            path = []
            cur = node
            while cur in came_from:
                path.append((cur[0], cur[1]))
                cur = came_from[cur]
            path.append((start[0], start[1]))
            return list(reversed(path))

        if t >= max_t:
            continue

        for dx,dy in MOVES:
            nx, ny = x+dx, y+dy
            nt = t+1

            # границы и проходимость
            if not in_bounds((nx,ny), width, height):
                continue
            if not is_free(grid, (nx,ny)):
                continue

            # constraints (используется в CBS) — запреты на вершины в определённое время
            if constraints and nt in constraints and (nx,ny) in constraints[nt]:
                continue

            # vertex reservation: кто-то занят в nt
            if nt in reserved_vertices and (nx,ny) in reserved_vertices[nt]:
                continue

            # edge reservation (swap): если в момент nt-1 кто-то зарезервировал переход (nx,ny)->(x,y)
            # мы запрещаем наш переход (x,y)->(nx,ny) если существует противоположный edge в reserved_edges[nt]
            if nt in reserved_edges and ((nx,ny),(x,y)) in reserved_edges[nt]:
                continue

            neigh = (nx, ny, nt)
            tentative_g = gscore[node] + 1
            if tentative_g < gscore.get(neigh, 10**9):
                gscore[neigh] = tentative_g
                came_from[neigh] = node
                fscore = tentative_g + heuristic((nx,ny), goal)
                heapq.heappush(openpq, (fscore, tentative_g, neigh))
    return None

# ---------------- Prioritized Planning (основная функция) ----------------------
def prioritized_planning(grid: List[List[int]],
                         agents: List[Dict],
                         priority: str = "id") -> Dict[int, List[Tuple[int,int]]]:
    """
    Наименование: prioritized_planning
    Назначение:
      Последовательное планирование путей в порядке приоритетов.
    Параметры:
      grid - карта,
      agents - список агенто-спеков {"id","start","goal"},
      priority - стратегия приоритета: "id" | "distance" | "random"
    Возвращаемое значение:
      словарь agent_id -> путь (list of (x,y))
    """
    # Копируем список агентов (чтобы не менять вход)
    ags = list(agents)

    # Определяем порядок планирования
    if priority == "distance":
        # сортируем по расстоянию start->goal (сначала ближние)
        ags.sort(key=lambda a: abs(a["start"][0]-a["goal"][0]) + abs(a["start"][1]-a["goal"][1]))
    elif priority == "random":
        random.shuffle(ags)
    else:
        # по id по возрастанию
        ags.sort(key=lambda a: a["id"])

    # reserved structures: time -> set(positions) и time -> set((u,v)edges)
    reserved_vertices: Dict[int, Set[Tuple[int,int]]] = {}
    reserved_edges: Dict[int, Set[Tuple[Tuple[int,int], Tuple[int,int]]]] = {}

    plans: Dict[int, List[Tuple[int,int]]] = {}
    max_len = 0

    for ag in ags:
        aid = ag["id"]
        start = tuple(ag["start"])
        goal = tuple(ag["goal"])

        # запустим A* с учётом текущих резервов
        path = a_star_with_reservations(grid, start, goal, reserved_vertices, reserved_edges, constraints=None, max_t=1000)

        if path is None:
            # без пути — оставляем стоять на старте (альтернатива: попытка вставить ожидание и повтор)
            path = [start]

        # резервируем вершины и ребра по времени
        for t in range(len(path)):
            pos = path[t]
            reserved_vertices.setdefault(t, set()).add(pos)
            # ребро от path[t-1] -> path[t] резервируем в момент t (т.е. переход, который произойдёт между t-1 и t)
            if t > 0:
                u = path[t-1]
                v = path[t]
                reserved_edges.setdefault(t, set()).add((u, v))

        plans[aid] = path
        if len(path) > max_len:
            max_len = len(path)

    # выравниваем длины путей (wait at goal)
    for aid, p in plans.items():
        if len(p) < max_len:
            goalpos = p[-1]
            plans[aid] = p + [goalpos] * (max_len - len(p))

    return plans

# ---------------- Пример использования (если вызвать файл напрямую) ----------------
if __name__ == "__main__":
    # небольшой пример "на коленке"
    sample_grid = [[0]*10 for _ in range(6)]
    agents = [
        {"id": 0, "start":[1,1], "goal":[8,4]},
        {"id": 1, "start":[1,4], "goal":[8,1]},
    ]
    plans = prioritized_planning(sample_grid, agents, priority="id")
    print("Plans:", plans)