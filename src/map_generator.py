import json
import random


def generate_grid(width=29, height=15, obstacle_prob=0.15, seed=None):
    if seed is not None:
        random.seed(seed)
    grid = [[0 for _ in range(width)] for __ in range(height)]
    for y in range(height):
        for x in range(width):
            if random.random() < obstacle_prob:
                grid[y][x] = 1
    return grid


def save_scenario(path, grid, agents):
    """
    agents: list of dicts: {"id": 0, "start": [x, y], "goal": [x, y]}
    :param path:
    :param grid:
    :param agents:
    :return:
    """
    data = {"grid": grid, "agents": agents}
    with open(path, "w") as f:
        json.dump(data, f, indent=2)


if __name__ == "__main__":
    g = generate_grid(20, 15, obstacle_prob=0.18, seed=42)
    #example agents
    agents = [
        {"id": 0, "start": [1, 1], "goal": [18, 13]},
        {"id": 1, "start": [1, 13], "goal": [18, 1]},
        {"id": 2, "start": [10, 1], "goal": [10, 13]}
    ]
    save_scenario("../scenarios/sample_map.json", g, agents)
    print("Saved scenarios")