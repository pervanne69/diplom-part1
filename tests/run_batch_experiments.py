import os
import time
import csv
from typing import List, Dict

from src.model import GridMASModel
from src.map_generator import generate_grid, save_scenario


# ----------------------------------------------------------
#  Автоматическая генерация тестовых карт для экспериментов
# ----------------------------------------------------------
def generate_experiment_scenarios(output_dir: str,
                                  num_maps: int,
                                  width: int,
                                  height: int,
                                  obstacle_probs: List[float],
                                  num_agents_list: List[int],
                                  seed: int = None):  # добавили seed
    os.makedirs(output_dir, exist_ok=True)
    scenarios = []

    counter = 0
    for p in obstacle_probs:
        for n_agents in num_agents_list:
            for i in range(num_maps):
                map_seed = seed + i if seed is not None else None
                grid = generate_grid(width, height, obstacle_prob=p, seed=map_seed)

                # Генерируем агентов
                agents = []
                for a in range(n_agents):
                    agents.append({
                        "id": a,
                        "start": [1, 1 + a],
                        "goal": [width - 2, height - 2 - a]
                    })

                filename = f"map_p{p}_agents{n_agents}_n{i}_s{seed}.json"
                path = os.path.join(output_dir, filename)
                save_scenario(path, grid, agents)

                scenarios.append(path)
                counter += 1

    print(f"Сгенерировано сценариев: {counter}")
    return scenarios

# ----------------------------------------------------------
#  Функция запуска теста одного сценария одним планировщиком
# ----------------------------------------------------------
def run_single_test(scenario_path: str, planner: str, **planner_args) -> Dict:
    start_time = time.time()
    model = GridMASModel(scenario_path, planner=planner, **planner_args)

    steps = 0
    while model.running and steps < 2000:
        model.step()
        steps += 1

    runtime = time.time() - start_time

    soc = sum(len(a.path) for a in model.agents_list)
    makespan = max(len(a.path) for a in model.agents_list)

    return {
        "scenario": os.path.basename(scenario_path),
        "planner": planner,
        "soc": soc,
        "makespan": makespan,
        "steps": steps,
        "runtime": round(runtime, 4),
        "agents": len(model.agents_list)
    }


# ----------------------------------------------------------
#  Основной батч-раннер с разными seed
# ----------------------------------------------------------
def run_experiments(num_seeds: int = 3):
    BASE_DIR = os.path.dirname(os.path.dirname(__file__))
    SCENARIO_DIR = os.path.join(BASE_DIR, "batch_scenarios")
    CSV_OUTPUT = os.path.join(BASE_DIR, "experiment_results.csv")

    # 1. Генерируем сценарии
    scenarios = generate_experiment_scenarios(
        output_dir=SCENARIO_DIR,
        num_maps=3,
        width=20,
        height=15,
        obstacle_probs=[0.05, 0.15, 0.30],
        num_agents_list=[3, 5]
    )

    # 2. Конфигурации планировщиков
    planners = [
        ("cooperative", {}),
        ("prioritized", {"pp_priority": "id"}),
        ("cbs", {})
    ]

    # 3. CSV заголовок
    with open(CSV_OUTPUT, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "scenario",
            "planner",
            "agents",
            "soc_mean",
            "makespan_mean",
            "steps_mean",
            "runtime_mean"
        ])

        # 4. Прогоняем все сценарии и планировщики
        for scenario in scenarios:
            print(f"\n=== Сценарий: {scenario} ===")

            for planner_name, args in planners:
                soc_list, makespan_list, steps_list, runtime_list = [], [], [], []

                for seed in range(num_seeds):
                    # При каждом seed мы можем заново сгенерировать сценарий,
                    # если хотим разные карты, иначе seed влияет только на агента
                    result = run_single_test(scenario, planner_name, **args)

                    soc_list.append(result["soc"])
                    makespan_list.append(result["makespan"])
                    steps_list.append(result["steps"])
                    runtime_list.append(result["runtime"])

                # усредняем результаты
                soc_mean = round(sum(soc_list) / num_seeds, 2)
                makespan_mean = round(sum(makespan_list) / num_seeds, 2)
                steps_mean = round(sum(steps_list) / num_seeds, 2)
                runtime_mean = round(sum(runtime_list) / num_seeds, 4)

                print(
                    f"  → {planner_name}: SoC={soc_mean}, "
                    f"Makespan={makespan_mean}, t={runtime_mean} сек."
                )

                writer.writerow([
                    os.path.basename(scenario),
                    planner_name,
                    len(soc_list),  # количество агентов
                    soc_mean,
                    makespan_mean,
                    steps_mean,
                    runtime_mean
                ])

    print("\nГотово! Результаты записаны в", CSV_OUTPUT)


# ----------------------------------------------------------
#  Запуск
# ----------------------------------------------------------
if __name__ == "__main__":
    run_experiments(num_seeds=3)