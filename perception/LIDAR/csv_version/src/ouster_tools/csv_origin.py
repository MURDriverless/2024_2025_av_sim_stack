import csv
import os

input_csv = "/home/jaewonl/sf_dev2new/src/csv_to_pointcloud_node/data/straight.csv"
output_csv = "/home/jaewonl/sf_dev2new/src/csv_to_pointcloud_node/data/straight_filtered1.csv"


# 남길 열 이름들 (Frame Index, Near Infrared 제외)
keep_columns = [
    "Timestamp (ns)", "Range (mm)", "Signal (photons)",
    "Reflectivity (%)", "X1 (mm)", "Y1 (mm)", "Z1 (mm)"
]

with open(input_csv, 'r') as infile:
    reader = list(csv.reader(infile))
    header = reader[0]
    data = reader[1:]

# 남길 열 인덱스
keep_indices = [i for i, col in enumerate(header) if col in keep_columns]
timestamp_index = header.index("Timestamp (ns)")
# range_index = header.index("Range (mm)")  # 필요 없다면 주석 처리

# 앞 절반 제거
halfway = len(data) // 2
trimmed_data = data[halfway:]

# 필터링 후 저장
with open(output_csv, 'w', newline='') as outfile:
    writer = csv.writer(outfile)
    writer.writerow([header[i] for i in keep_indices])
    for row in data:
        try:
            # Timestamp가 0인 행은 건너뜀
            if int(row[timestamp_index]) == 0:
                continue
            writer.writerow([row[i] for i in keep_indices])
        except (IndexError, ValueError):
            continue

print(f"✅ Saved filtered CSV → {output_csv}")
