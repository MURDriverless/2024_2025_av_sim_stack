from ouster.sdk import client, pcap
import csv
import datetime
import os

# === 경로 설정 ===
base_dir = os.path.expanduser("~/ros2_ws/src/csv_to_pointcloud_node/data")
os.makedirs(base_dir, exist_ok=True)

pcap_path = "/mnt/c/Users/Com/OneDrive/Desktop/IMU/20241213_2323_OS-1-128_122212002238.pcap"
json_path = "/mnt/c/Users/Com/OneDrive/Desktop/IMU/20241213_2323_OS-1-128_122212002238.json"
csv_out_path = os.path.expanduser("~/ros2_ws/src/csv_to_pointcloud_node/data/extracted_by_time.csv")

# === 시간 설정 (초 단위)
start_time_sec = 27   
end_time_sec = 39  

with open(json_path, 'r') as f:
    metadata = f.read()
info = client.SensorInfo(metadata)

source = pcap.Pcap(pcap_path, info)
scan_iter = client.Scans(source)

# Saving
with open(csv_out_path, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['x', 'y', 'z'])

    for scan in scan_iter:
        timestamp = scan.timestamp  # in nanoseconds
        timestamp_sec = timestamp / 1e9

        if timestamp_sec < start_time_sec:
            continue
        elif timestamp_sec > end_time_sec:
            break

        xyz = client.XYZLut(info)(scan)
        for pt in xyz.reshape(-1, 3):
            writer.writerow(pt)

print(f" Saved CSV between {start_time_sec}s ~ {end_time_sec}s → {csv_out_path}")
