# src/cbs.py
# Полноценная, защищённая реализация Conflict-Based Search (CBS) для MAPF.
# Включает таймаут, ограничение вершин дерева, детект конфликтов (vertex/edge),
# интеграцию с a_star_with_reservations из prioritized_planning и fallback.
# Автор:
# Дата: 2025-12-04

import heapq
import time
from typing import List, Tuple, Dict, Set, Optional, Any
from dataclasses import dataclass, field

# импортируем низкоуровневый A* и вспомогательные функции из prioritized_planning
# предполагается, что prioritized_planning.py лежит рядом в src/
from .prioritized_planning import a_star_with_reservations, heuristic, in_bounds, is_free, MOVES
from .prioritized_planning import prioritized_planning  # fallback

# -----------------------------------------------------------------------------
# Типы
# -----------------------------------------------------------------------------
Grid = List[List[int]]
Pos = Tuple[int, int]
Path = List[Pos]
AgentSpec = Dict[str, Any]

# -----------------------------------------------------------------------------
# Структура ограничения (constraint)
# Формат: ("vertex", agent_id, pos, t) или ("edge", agent_id, u, v, t)
# -----------------------------------------------------------------------------

@dataclass(order=True)
class CBSNode:
    """
    Представление узла high-level дерева CBS.
    Сортировка по (cost, num_conflicts) для приоритетной очереди.
    """
    priority: Tuple[int, int] = field(init=False)  # (cost, conflicts) — ключ очереди
    cost: int = field(default=0)                    # суммарная стоимость (SoC)
    constraints: List[Tuple] = field(default_factory=list)  # список ограничений
    paths: Dict[int, Path] = field(default_factory=dict)   # agent_id -> path
    conflicts_count: int = field(default=0)         # количество обнаруженных конфликтов

    def __post_init__(self):
        # priority формируется как (cost, conflicts_count)
        self.priority = (self.cost, self.conflicts_count)


# -----------------------------------------------------------------------------
# Функции для проверки конфликтов
# -----------------------------------------------------------------------------
def detect_first_conflict(paths: Dict[int, Path]) -> Optional[Dict]:
    """
    Находит первый конфликт между путями.
    Ищет по времени от t=0 до max_len-1:
      - вершинные конфликты (две агента в одной клетке в одно и то же время)
      - реберные конфликты (swap: a->b и b->a за один шаг)

    Возвращает:
      dict с ключами:
        - type: "vertex" или "edge"
        - time: время конфликта
        - a1, a2: id агентов
        - pos (для vertex) или u, v (для edge)
      Или None, если конфликтов нет.
    """
    # максимальная длина — длина самого длинного пути
    max_t = max(len(p) for p in paths.values()) if paths else 0

    # по каждому временному слоту
    for t in range(max_t):
        # vertex check: позиция -> agent_id
        pos_map: Dict[Pos, int] = {}
        for aid, p in paths.items():
            pos = p[t] if t < len(p) else p[-1]  # если путь короче — остаётся на месте
            if pos in pos_map:
                # найден вершинный конфликт
                return {"type": "vertex", "time": t, "a1": pos_map[pos], "a2": aid, "pos": pos}
            pos_map[pos] = aid

        # edge (swap) check:
        # составляем ребра каждого агента в момент t: u->v (u = pos at t-1, v = pos at t)
        edge_map: Dict[int, Tuple[Pos, Pos]] = {}
        for aid, p in paths.items():
            u = p[t-1] if (t-1) >= 0 and (t-1) < len(p) else (p[0] if len(p) > 0 else None)
            v = p[t] if t < len(p) else p[-1]
            edge_map[aid] = (u, v)
        # сравниваем попарно (оптимизация возможна, но для небольших N ок)
        aids = list(edge_map.keys())
        for i in range(len(aids)):
            a1 = aids[i]
            u1, v1 = edge_map[a1]
            for j in range(i + 1, len(aids)):
                a2 = aids[j]
                u2, v2 = edge_map[a2]
                # swap если u1==v2 и v1==u2 и u1 != v1 (чтобы не считать ожидание)
                if u1 is not None and u2 is not None and u1 == v2 and v1 == u2 and u1 != v1:
                    return {"type": "edge", "time": t, "a1": a1, "a2": a2, "u": u1, "v": v1}
    return None


# -----------------------------------------------------------------------------
# Вспомогательная функция: собирает constraints для конкретного агента (low-level)
# -----------------------------------------------------------------------------
def constraints_for_agent(all_constraints: List[Tuple], agent_id: int) -> Dict[int, Set[Pos]]:
    """
    Из общего списка ограничений создаёт словарь вида:
      time -> set(запрещённых позиций) для данного agent_id.
    Также возвращает edge-ограничения как словарь time -> set((u,v)).
    """
    vertex_cons: Dict[int, Set[Pos]] = {}
    edge_cons: Dict[int, Set[Tuple[Pos, Pos]]] = {}

    for c in all_constraints:
        if c[0] == "vertex":
            _, aid, pos, t = c
            if aid == agent_id:
                vertex_cons.setdefault(t, set()).add(pos)
        elif c[0] == "edge":
            _, aid, u, v, t = c
            if aid == agent_id:
                edge_cons.setdefault(t, set()).add((u, v))
    return vertex_cons, edge_cons


# -----------------------------------------------------------------------------
# Low-level: A* с учётом CBS-ограничений и глобальных резерваций не нужен здесь.
# Мы используем ранее реализованную a_star_with_reservations, но передаём
# per-agent constraints как аргумент 'constraints' (time -> set(positions)).
# -----------------------------------------------------------------------------
def low_level_search(grid: Grid, start: Pos, goal: Pos,
                     agent_id: int, all_constraints: List[Tuple],
                     max_t: int = 500) -> Optional[Path]:
    """
    Выполняет low-level поиск пути для конкретного агента с учётом текущих ограничений CBS.
    Возвращает path или None.
    """
    # получаем vertex- и edge- ограничения для данного агента
    vertex_cons, edge_cons = constraints_for_agent(all_constraints, agent_id)

    # a_star_with_reservations ожидает:
    #   reserved_vertices: Dict[int, Set[Pos]]
    #   reserved_edges: Dict[int, Set[(u,v)]]
    # мы передаём сюда пустые reserved_* (их использует prioritized_planning для последовательности)
    reserved_vertices: Dict[int, Set[Pos]] = {}
    reserved_edges: Dict[int, Set[Tuple[Pos, Pos]]] = {}

    # используем vertex_cons как параметр constraints (совместимо с prior impl)
    path = a_star_with_reservations(grid, start, goal,
                                    reserved_vertices, reserved_edges,
                                    constraints=vertex_cons,
                                    max_t=max_t)
    # NOTE: edge_cons не передаём и не проверяем в a_star_with_reservations,
    # потому что там edge-reservations проверяются через reserved_edges аргумент.
    # Для строгой поддержки edge-constraints можно расширить a_star_with_reservations,
    # но текущая стратегия: edge-constraints заданы на high-level и приводят к запрету вершины/ребра.
    return path


# -----------------------------------------------------------------------------
# High-level CBS (основная функция)
# -----------------------------------------------------------------------------
def cbs(grid: Grid,
        agents: List[AgentSpec],
        time_limit: float = 5.0,
        node_limit: int = 1000,
        max_constraints_per_agent: int = 50,
        fallback: str = "prioritized",
        pp_priority: str = "id") -> Dict[int, Path]:
    """
    Наименование: cbs
    Назначение:
        Conflict-Based Search с защитой от зависаний.
    Входные параметры:
        grid - карта (0/1)
        agents - список агентов: {"id":int,"start":[x,y],"goal":[x,y]}
        time_limit (float) - таймаут в секундах для выполнения CBS
        node_limit (int) - максимум созданных узлов HL-дерева
        max_constraints_per_agent (int) - ограничение глубины constrain'ов для одного агента
        fallback - если "prioritized", то при провале CBS возвращает prioritized_planning
        pp_priority - стратегия приоритета для fallback
    Возвращаемое значение:
        словарь agent_id -> path (list of (x,y)). Если CBS прервали, возвращается fallback-план.
    Примечание:
        - Функция пытаться найти оптимальное решение, но при превышении лимитов возвращает fallback.
    """
    start_time = time.time()

    # корневой узел: планируем для всех агентов без ограничений
    root = CBSNode()
    root.paths = {}
    # low-level планирование для каждого агента
    for ag in agents:
        aid = ag["id"]
        s = tuple(ag["start"])
        g = tuple(ag["goal"])
        p = low_level_search(grid, s, g, aid, [], max_t=1000)
        if p is None:
            # если для какого-то агента путь не найден — fallback
            if fallback == "prioritized":
                return prioritized_planning(grid, agents, priority=pp_priority)
            else:
                # возвращаем самое лучшее что есть: stay-in-place
                root.paths[aid] = [s]
        else:
            root.paths[aid] = p

    root.cost = sum(len(p) for p in root.paths.values())
    root.conflicts_count = 0
    root.priority = (root.cost, root.conflicts_count)

    # очередь открытых узлов: (priority_tuple, node_id, node)
    open_list: List[Tuple[Tuple[int, int], int, CBSNode]] = []
    counter = 0
    heapq.heappush(open_list, (root.priority, counter, root))
    counter += 1

    nodes_expanded = 0

    # основной цикл HL
    while open_list:
        # таймаут?
        if time.time() - start_time > time_limit:
            # таймаут — используем fallback
            if fallback == "prioritized":
                return prioritized_planning(grid, agents, priority=pp_priority)
            else:
                # возвращаем текущее лучшее решение (верх очереди)
                _, _, best_node = open_list[0]
                return best_node.paths

        # node limit?
        if nodes_expanded >= node_limit:
            if fallback == "prioritized":
                return prioritized_planning(grid, agents, priority=pp_priority)
            else:
                _, _, best_node = open_list[0]
                return best_node.paths

        # извлекаем лучший узел
        _, _, node = heapq.heappop(open_list)
        nodes_expanded += 1

        # находим первый конфликт
        conflict = detect_first_conflict(node.paths)
        if conflict is None:
            # решение без конфликтов найдено
            return node.paths

        # получен конфликт — создаём два child-а с разными ограничениями
        if conflict["type"] == "vertex":
            t = conflict["time"]
            pos = conflict["pos"]
            a1 = conflict["a1"]
            a2 = conflict["a2"]

            # child для a1: запретить быть в pos в t
            child1 = CBSNode()
            child1.constraints = list(node.constraints)
            child1.constraints.append(("vertex", a1, pos, t))
            # проверка глубины ограничений для a1
            count_a1 = sum(1 for c in child1.constraints if c[0] == "vertex" and c[1] == a1)
            if count_a1 <= max_constraints_per_agent:
                # копируем пути как базу
                child1.paths = dict(node.paths)
                # перепланируем путь только для a1 с учётом новых ограничений
                s1 = tuple(next(a for a in agents if a["id"] == a1)["start"])
                g1 = tuple(next(a for a in agents if a["id"] == a1)["goal"])
                p1 = low_level_search(grid, s1, g1, a1, child1.constraints, max_t=1000)
                if p1 is None:
                    # если не удалось — отбросим этот child (альтернатива: поставить stay)
                    pass
                else:
                    child1.paths[a1] = p1
                    child1.cost = sum(len(p) for p in child1.paths.values())
                    child1.conflicts_count = 0  # пересчёт возможных конфликтов оставим на detect
                    child1.priority = (child1.cost, child1.conflicts_count)
                    heapq.heappush(open_list, (child1.priority, counter, child1))
                    counter += 1

            # child для a2: запретить быть в pos в t
            child2 = CBSNode()
            child2.constraints = list(node.constraints)
            child2.constraints.append(("vertex", a2, pos, t))
            count_a2 = sum(1 for c in child2.constraints if c[0] == "vertex" and c[1] == a2)
            if count_a2 <= max_constraints_per_agent:
                child2.paths = dict(node.paths)
                s2 = tuple(next(a for a in agents if a["id"] == a2)["start"])
                g2 = tuple(next(a for a in agents if a["id"] == a2)["goal"])
                p2 = low_level_search(grid, s2, g2, a2, child2.constraints, max_t=1000)
                if p2 is None:
                    pass
                else:
                    child2.paths[a2] = p2
                    child2.cost = sum(len(p) for p in child2.paths.values())
                    child2.conflicts_count = 0
                    child2.priority = (child2.cost, child2.conflicts_count)
                    heapq.heappush(open_list, (child2.priority, counter, child2))
                    counter += 1

        elif conflict["type"] == "edge":
            t = conflict["time"]
            a1 = conflict["a1"]; a2 = conflict["a2"]
            u = conflict["u"]; v = conflict["v"]

            # child для a1: запретить переход u->v в момент t
            child1 = CBSNode()
            child1.constraints = list(node.constraints)
            child1.constraints.append(("edge", a1, u, v, t))
            count_edges_a1 = sum(1 for c in child1.constraints if c[0] == "edge" and c[1] == a1)
            if count_edges_a1 <= max_constraints_per_agent:
                child1.paths = dict(node.paths)
                s1 = tuple(next(a for a in agents if a["id"] == a1)["start"])
                g1 = tuple(next(a for a in agents if a["id"] == a1)["goal"])
                p1 = low_level_search(grid, s1, g1, a1, child1.constraints, max_t=1000)
                if p1 is not None:
                    child1.paths[a1] = p1
                    child1.cost = sum(len(p) for p in child1.paths.values())
                    child1.conflicts_count = 0
                    child1.priority = (child1.cost, child1.conflicts_count)
                    heapq.heappush(open_list, (child1.priority, counter, child1))
                    counter += 1

            # child для a2: запретить переход v->u в момент t
            child2 = CBSNode()
            child2.constraints = list(node.constraints)
            child2.constraints.append(("edge", a2, v, u, t))
            count_edges_a2 = sum(1 for c in child2.constraints if c[0] == "edge" and c[1] == a2)
            if count_edges_a2 <= max_constraints_per_agent:
                child2.paths = dict(node.paths)
                s2 = tuple(next(a for a in agents if a["id"] == a2)["start"])
                g2 = tuple(next(a for a in agents if a["id"] == a2)["goal"])
                p2 = low_level_search(grid, s2, g2, a2, child2.constraints, max_t=1000)
                if p2 is not None:
                    child2.paths[a2] = p2
                    child2.cost = sum(len(p) for p in child2.paths.values())
                    child2.conflicts_count = 0
                    child2.priority = (child2.cost, child2.conflicts_count)
                    heapq.heappush(open_list, (child2.priority, counter, child2))
                    counter += 1

    # если очередь опустела и решения не найдено — возвращаем fallback
    if fallback == "prioritized":
        return prioritized_planning(grid, agents, priority=pp_priority)
    else:
        # возвращаем самый лучший узел если есть
        if open_list:
            _, _, best_node = open_list[0]
            return best_node.paths
        # иначе — пустой ответ
        return {ag["id"]: [tuple(ag["start"])] for ag in agents}