import os
import csv
from collections import defaultdict
from typing import List, Dict
from run_batch_experiments import generate_experiment_scenarios, run_single_test

def run_experiments_avg():
    BASE_DIR = os.path.dirname(os.path.dirname(__file__))
    SCENARIO_DIR = os.path.join(BASE_DIR, "batch_scenarios_avg")
    CSV_OUTPUT = os.path.join(BASE_DIR, "experiment_results_avg.csv")

    # Параметры экспериментов
    obstacle_probs = [0.05, 0.15, 0.30]
    num_agents_list = [3, 5]
    num_maps = 1         # на seed генерируем по одной карте
    seeds = [0, 1, 2]   # разные seed для усреднения
    planners = [
        ("cooperative", {}),
        ("prioritized", {"pp_priority": "id"}),
        ("cbs", {})
    ]

    # CSV заголовок
    with open(CSV_OUTPUT, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "obstacle_prob", "agents", "planner",
            "avg_soc", "avg_makespan", "avg_runtime", "runs"
        ])

        # По каждому сочетанию obstacle_prob и количества агентов
        for p in obstacle_probs:
            for n_agents in num_agents_list:
                avg_data = defaultdict(lambda: {"soc": 0, "makespan": 0, "runtime": 0, "runs": 0})

                print(f"\n=== Obstacle prob: {p}, Agents: {n_agents} ===")

                # Пробегаемся по seed
                for seed in seeds:
                    scenarios = generate_experiment_scenarios(
                        output_dir=SCENARIO_DIR,
                        num_maps=num_maps,
                        width=20,
                        height=15,
                        obstacle_probs=[p],
                        num_agents_list=[n_agents],
                        seed=seed
                    )

                    scenario_file = scenarios[0]
                    print(f"Сценарий: {os.path.basename(scenario_file)}, seed={seed}")

                    # Пробегаемся по планировщикам
                    for planner_name, args in planners:
                        result = run_single_test(scenario_file, planner_name, **args)

                        avg_data[planner_name]["soc"] += result["soc"]
                        avg_data[planner_name]["makespan"] += result["makespan"]
                        avg_data[planner_name]["runtime"] += result["runtime"]
                        avg_data[planner_name]["runs"] += 1

                        print(
                            f"  → {planner_name}: SoC={result['soc']}, "
                            f"Makespan={result['makespan']}, t={result['runtime']} сек."
                        )

                # Усредняем и записываем в CSV
                for planner_name in planners:
                    pname = planner_name[0]
                    data = avg_data[pname]
                    writer.writerow([
                        p,
                        n_agents,
                        pname,
                        round(data["soc"] / data["runs"], 2),
                        round(data["makespan"] / data["runs"], 2),
                        round(data["runtime"] / data["runs"], 4),
                        data["runs"]
                    ])

    print("\nГотово! Усреднённые результаты записаны в", CSV_OUTPUT)


if __name__ == "__main__":
    run_experiments_avg()