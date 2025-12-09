# Модуль предназначен для запуска графической визуализации модели
# мультиагентной системы (МАС), реализованной с использованием Mesa.

from mesa.visualization.modules import CanvasGrid
from mesa.visualization.ModularVisualization import ModularServer
from mesa.visualization.modules import TextElement

from src.model import GridMASModel

def agent_portrayal(agent):
    if agent is None:
        return

    # === Препятствия ===
    if hasattr(agent, "agent_type") and agent.agent_type == "obstacle":
        return {
            "Shape": "rect",
            "w": 1.0,
            "h": 1.0,
            "Filled": "true",
            "Layer": 0,
            "Color": "black"
        }

    # === Роботы ===
    color = "green" if agent.finished else "red"

    return {
        "Shape": "rect",
        "w": 0.9,
        "h": 0.9,
        "Filled": "true",
        "Layer": 1,
        "Color": color,
        "text": str(agent.unique_id),
        "text_color": "white"
    }


class MapInfo(TextElement):
    """
        Наименование: MapInfo
        Назначение:
            Элемент текстовой визуализации, выводящий информацию о модели
            (количество агентов и состояние симуляции).

        Входные параметры:
            отсутствуют.

        Возвращаемое значение:
            str — текст, отображаемый на панели визуализации.
    """
    def render(self, model):
        # Строка информации - количество агентов и статус
        completed = sum(a.finished for a in model.agents_list)
        total = len(model.agents_list)

        return f"Агенты: {completed}/{total} завершили маршрут"

def build_server():
    """
    Создание и конфигурация веб-сервера Mesa
    с динамическими препятствиями и маршрутами агентов.
    """
    grid_canvas = CanvasGrid(
        agent_portrayal,
        20,  # ширина сетки
        15,  # высота сетки
        600,
        450
    )

    server = ModularServer(
        GridMASModel,
        [grid_canvas, MapInfo()],
        "Multi-Agent Grid Simulation",
        {
            "width": 20,
            "height": 15,
            "num_agents": 3,
            "obstacle_prob": 0.18,
            "planner": "prioritized",
            "pp_priority": "id",
            "seed": None  # Можно поставить число для фиксированной карты
        }
    )

    server.port = 8521
    return server


if __name__ == "__main__":
    server = build_server()
    server.launch()