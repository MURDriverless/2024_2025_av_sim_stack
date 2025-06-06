import pandas as pd
import matplotlib.pyplot as plt


df = pd.read_csv("../src/csv_to_pointcloud_node/data/straight.csv")  # 또는 네가 자른 CSV

# 히스토그램 그리기
plt.hist(df["z"], bins=200, color='skyblue', edgecolor='black')
plt.title("Z Value Distribution (Height)")
plt.xlabel("Z value (meters)")
plt.ylabel("Number of points")
plt.grid(True)
plt.show()