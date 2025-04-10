import csv
import json

class TrackExporter:
    @staticmethod
    def to_json(track_data, filename):
        with open(filename, 'w') as f:
            json.dump(track_data, f, indent=2)

    def to_csv(track_data, filename):
        cones = track_data['left_cones'] + track_data['right_cones']
        centerline = track_data['centerline']

        with open(filename, 'w', newline='') as csvfile:
            fieldnames = ['x', 'y', 'type', 'color']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

            # Write cones
            for cone in cones:
                writer.writerow({
                    'x': cone['x'],
                    'y': cone['y'],
                    'type': f"{cone['side']}_cone",
                    'color': cone['color']
                })

            # Write centerline points
            for pt in centerline:
                writer.writerow({
                    'x': pt[0],
                    'y': pt[1],
                    'type': 'centerline',
                    'color': 'black'
                })

        print(f"Track (cones + centerline) exported to CSV: {filename}")