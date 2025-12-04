import os
from src.model import GridMASModel

if __name__ == "__main__":
    # Папка tests
    TESTS_DIR = os.path.dirname(__file__)

    # Папка diploma (корень проекта)
    PROJECT_DIR = os.path.dirname(TESTS_DIR)

    scenario = os.path.join(PROJECT_DIR, "scenarios", "sample_map.json")

    print("Scenario path:", scenario)
    assert os.path.exists(scenario), "Сценарий не найден!"

    model = GridMASModel(scenario, )

    steps = 0
    while model.running and steps < 1000:
        model.step()
        steps += 1

    soc = sum(len(a.path) for a in model.agents_list)
    makespan = max(len(a.path) for a in model.agents_list)

    print("steps:", steps)
    print("SoC:", soc)
    print("Makespan:", makespan)