# Модуль предназначен для запуска графической визуализации модели
# мультиагентной системы (МАС), реализованной с использованием Mesa.

from mesa.visualization.modules import CanvasGrid
from mesa.visualization.ModularVisualization import ModularServer
from mesa.visualization.modules import TextElement

from src.model import GridMASModel

def agent_portrayal(agent):
    """
        Наименование: agent_portrayal
        Назначение:
            Формирует словарь свойств (portrayal) для визуализации агента
            на холсте CanvasGrid Mesa.

        Входные параметры:
            agent (RobotAgent) — объект агента.

        Возвращаемое значение:
            dict — параметры отрисовки агента.
    """
    if agent is None:
        return

    # Цвета агента в зависимости от завершенности
    color = "green" if agent.finished else "red"

    # Описание отрисовки агента в ячейке сетки
    portrayal = {
        "Shape": "rect",
        "w": 0.9,
        "h": 0.9,
        "Filled": "true",
        "Layer": 1,
        "Color": color,
        "text": str(agent.unique_id),
        "text_color": "white"
        }
    return portrayal


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

def build_server(scenario_path: str):
    """
        Наименование: build_server
        Назначение:
            Создание и конфигурация веб-сервера Mesa,
            отображающего 2D-сетку и состояние агентов.

        Входные параметры:
            scenario_path (str) — путь к JSON-сценарию.

        Возвращаемое значение:
            ModularServer — готовый объект сервера Mesa.
    """

    # Создаём объект CanvasGrid:
    #  - agent_portrayal — функция визуализации агента
    #  - размеры сетки будут автоматически получены из модели

    grid_canvas = CanvasGrid(
        agent_portrayal,
        20,
        20,
        600,
        600
    )
    server = ModularServer(
        GridMASModel,
        [grid_canvas, MapInfo()],
        "Multi-Agent Grid Simulation",
        {"scenario_path": scenario_path}
    )

    server.port = 8521
    return server


if __name__ == "__main__":
    scenario = "scenarios/sample_map.json"

    server = build_server(scenario)

    server.launch()