import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Загружаем CSV с усреднёнными результатами
df = pd.read_csv("experiment_results_avg.csv")

# Уникальные комбинации obstacle_prob × num_agents
combinations = df.groupby(["obstacle_prob", "agents"]).first().reset_index()
x_labels = [f"{row.obstacle_prob}_{row.agents}" for _, row in combinations.iterrows()]

# Список планировщиков и цвета
planners = ["cooperative", "prioritized", "cbs"]
colors = ["#1f77b4", "#ff7f0e", "#2ca02c"]  # синие, оранжевые, зелёные
n_planners = len(planners)
x = np.arange(len(x_labels))  # позиции на оси X
width = 0.25  # ширина столбцов

# --- Создаём фигуру с двумя осями ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))

# --- График SoC ---
for i, planner in enumerate(planners):
    values = df[df["planner"] == planner]["avg_soc"].values
    bars = ax1.bar(x + i*width, values, width=width, label=planner, color=colors[i])
    # подписи над столбцами
    for bar in bars:
        height = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2, height + 1, f'{height:.1f}', ha='center', va='bottom', fontsize=9)

ax1.set_title("Сравнение планировщиков по SoC", fontsize=14)
ax1.set_ylabel("SoC", fontsize=12)
ax1.set_xticks(x + width)
ax1.set_xticklabels(x_labels, rotation=45)
ax1.legend()
ax1.grid(axis='y', linestyle='--', alpha=0.7)

# --- График Makespan ---
for i, planner in enumerate(planners):
    values = df[df["planner"] == planner]["avg_makespan"].values
    bars = ax2.bar(x + i*width, values, width=width, label=planner, color=colors[i])
    # подписи над столбцами
    for bar in bars:
        height = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width()/2, height + 0.3, f'{height:.1f}', ha='center', va='bottom', fontsize=9)

ax2.set_title("Сравнение планировщиков по Makespan", fontsize=14)
ax2.set_xlabel("Комбинация obstacle_prob × num_agents", fontsize=12)
ax2.set_ylabel("Makespan", fontsize=12)
ax2.set_xticks(x + width)
ax2.set_xticklabels(x_labels, rotation=45)
ax2.legend()
ax2.grid(axis='y', linestyle='--', alpha=0.7)

plt.tight_layout()
# --- Сохраняем в файл ---
plt.savefig("planner_comparison_charts_labeled.png", dpi=300)
plt.show()