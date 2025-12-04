import pandas as pd

# Загружаем CSV с усреднёнными результатами
df = pd.read_csv("experiment_results_avg.csv")

# Поворот таблицы, чтобы планировщики были столбцами
soc_table = df.pivot_table(index=["obstacle_prob", "agents"], columns="planner", values="avg_soc")
makespan_table = df.pivot_table(index=["obstacle_prob", "agents"], columns="planner", values="avg_makespan")

print("=== Сравнительная таблица SoC ===")
print(soc_table)
print("\n=== Сравнительная таблица Makespan ===")
print(makespan_table)

# Опционально: сохранить в Excel
with pd.ExcelWriter("planner_comparison.xlsx") as writer:
    soc_table.to_excel(writer, sheet_name="SoC")
    makespan_table.to_excel(writer, sheet_name="Makespan")