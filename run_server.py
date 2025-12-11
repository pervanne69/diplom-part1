from mesa.visualization.modules import CanvasGrid, ChartModule, TextElement
from mesa.visualization.ModularVisualization import ModularServer
from src.model import GridMASModel


def agent_portrayal(agent):
    if agent is None:
        return

    if getattr(agent, "agent_type", None) == "obstacle":
        return {
            "Shape": "rect",
            "Color": "black",
            "Layer": 0,
            "w": 1.0,
            "h": 1.0,
            "Filled": True
        }

    if getattr(agent, "agent_type", None) == "task":
        return {
            "Shape": "circle",
            "r": 0.45,
            "Filled": True,
            "Layer": 1,
            "Color": "yellow",
            "text": str(agent.task_id),
            "text_color": "black"
        }

    if hasattr(agent, "goal"):
        color = "green" if getattr(agent, "finished", False) else "red"
        return {
            "Shape": "rect",  # квадратный робот
            "Color": color,
            "Layer": 2,
            "w": 1.0,
            "h": 1.0,
            "Filled": True,
            "text": str(agent.unique_id),
            "text_color": "white"
        }


class MetricsText(TextElement):
    def render(self, model):
        soc = sum(len(a.path) for a in model.agents_list)
        makespan = max((len(a.path) for a in model.agents_list), default=0)
        return (
            f"Agents: {len(model.agents_list)} | "
            f"Tasks: {len(model.tasks)} | "
            f"SoC: {soc} | "
            f"Makespan: {makespan}"
        )


def build_server():
    grid_canvas = CanvasGrid(agent_portrayal, 20, 15, 600, 450)

    chart_module = ChartModule([
        {"Label": "SoC", "Color": "Black"},
        {"Label": "Makespan", "Color": "Blue"}
    ])

    metrics_text = MetricsText()

    server_params = {
        "width": 20,
        "height": 15,
        "num_agents": 3,
        "num_tasks": 10,  # 10 задач
        "obstacle_prob": 0.18,
        "planner": "prioritized",
        "pp_priority": "id",
        "seed": None,
        "mrta_method": "hungarian",
    }

    server = ModularServer(
        GridMASModel,
        [grid_canvas, metrics_text, chart_module],
        "Multi-Agent Grid Simulation",
        server_params
    )

    server.port = 8521
    return server


if __name__ == "__main__":
    server = build_server()
    server.launch()