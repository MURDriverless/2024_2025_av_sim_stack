from scipy.interpolate import splprep, splev
import argparse
import csv
import json
import matplotlib.pyplot as plt
import numpy as np
import os

# will add more comments later to describe functionality

def generate_formula_student_track(num_waypoints, track_width, cone_spacing, total_length):
    # Generate random waypoints on a perturbed circle
    angles = np.linspace(0, 2*np.pi, num_waypoints, endpoint=False)
    radii = np.random.uniform(20, 30, size=num_waypoints)  # Controls how "round" the track is
    x = radii * np.cos(angles)
    y = radii * np.sin(angles)

    # Close the loop
    # x = np.append(x, x[0])
    # y = np.append(y, y[0])

    # Fit a periodic B-spline to get a smooth centerline
    tck, u = splprep([x, y], s=0.5, per=True)
    num_samples = int(total_length / cone_spacing)
    u_fine = np.linspace(0, 1, num_samples)
    x_smooth, y_smooth = splev(u_fine, tck)
    centerline = np.vstack((x_smooth, y_smooth)).T

    # Find closest point to origin
    closest_index = np. argmin(np.linalg.norm(centerline, axis=1))
    closest_point = centerline[closest_index]

    # Rotate and translate so taht the closest point is at (0,0)
    centerline = np.roll(centerline, -closest_index, axis=0)
    offset = centerline[0]
    centerline -= offset

    # Generate cones along left and right boundaries
    left_cones = []
    right_cones = []

    for i in range(len(centerline)):
        p = centerline[i]
        p_next = centerline[(i + 1) % len(centerline)]
        direction = p_next - p
        direction /= np.linalg.norm(direction)
        normal = np.array([-direction[1], direction[0]])

        left = p + (track_width / 2) * normal
        right = p - (track_width / 2) * normal

        # left_cones.append(left.tolist())
        # right_cones.append(right.tolist())

        # Check if it's the origin / start
        # if np.allclose(p, [0.0, 0.0], atol=1e-3):
        #     left_cones.append({
        #         "x": left[0],
        #         "y": left[1],
        #         "side": "left",
        #         "color": "orange"
        #     })

        #     right_cones.append({
        #         "x": right[0],
        #         "y": right[1],
        #         "side": "left",
        #         "color": "orange"
        #     })

        # else:
        #     # add color to the cones as well as which side of the track these cones are placed
        #     left_cones.append({ 
        #         "x": left[0],
        #         "y": left[1],
        #         "side": "left",
        #         "color": "yellow"
        #     })

        #     right_cones.append({
        #         "x": right[0],
        #         "y": right[1],
        #         "side": "right",
        #         "color": "blue"
        #     })

        if i == 0:
            # Place orange cones at start/finish line
            left_cones.append({"x": left[0], "y": left[1], "side": "left", "color": "orange"})
            right_cones.append({"x": right[0], "y": right[1], "side": "right", "color": "orange"})
        else:
            # add color to the cones as well as which side of the track these cones are placed
            left_cones.append({"x": left[0], "y": left[1], "side": "left", "color": "yellow"})
            right_cones.append({"x": right[0], "y": right[1], "side": "right", "color": "blue"})

    return {
        "centerline": centerline.tolist(),
        "left_cones": left_cones,
        "right_cones": right_cones
    }

def save_track(track_data, filename="fs_track.json"):
    with open(filename, 'w') as f:
        json.dump(track_data, f, indent=2)

def visualize_track(track_data, show_centerline):
    center = np.array(track_data['centerline'])
    left = track_data['left_cones']
    right = track_data['right_cones']

    plt.figure(figsize=(8, 8))
    
    if show_centerline == 1:
        plt.plot(center[:, 0], center[:, 1], 'k--', label='Centerline')

    plt.scatter([c['x'] for c in left if c['color'] == 'orange'], [c['y'] for c in left if c['color'] == 'orange'], color='orange', s=10,)
    plt.scatter([c['x'] for c in left if c['color'] == 'yellow'], [c['y'] for c in left if c['color'] == 'yellow'], color='yellow', s=10, label='Left Cones (Yellow)')
    plt.scatter([c['x'] for c in right if c['color'] == 'orange'], [c['y'] for c in right if c['color'] == 'orange'], color='orange', s=10)
    plt.scatter([c['x'] for c in right if c['color'] == 'blue'], [c['y'] for c in right if c['color'] == 'blue'], color='blue', s=10, label='Right Cones (Blue)')

    # Add solid black start/finish line using orange cones
    left_start = next((c for c in left if c['color'] == 'orange'), None)
    right_start = next((c for c in right if c['color'] == 'orange'), None)

    if left_start and right_start:
        plt.plot(
            [left_start['x'], right_start['x']],
            [left_start['y'], right_start['y']],
            'k-', linewidth=2, label='Start/Finish'
        )

    plt.axis('equal')
    plt.title("Formula Student Style Track")
    plt.legend()
    plt.grid(True)
    plt.show()


## old
# def visualize_track(track_data, show_centerline):
#     if show_centerline == 1:
#         center = np.array(track_data['centerline'])
#     left = np.array(track_data['left_cones'])
#     right = np.array(track_data['right_cones'])

#     plt.figure(figsize=(8, 8))
#     if show_centerline == 1:
#         plt.plot(center[:, 0], center[:, 1], 'k--', label='Centerline')
#     plt.scatter(left[:, 0], left[:, 1], c='blue', s=10, label='Left Cones')
#     plt.scatter(right[:, 0], right[:, 1], c='red', s=10, label='Right Cones')
#     plt.axis('equal')
#     plt.title("Formula Student Style Track")
#     plt.legend()
#     plt.grid(True)
#     plt.show()

# export left, and right cone coordinates
# def export_cones_to_csv(track_data, filename="fs_track.csv"):
#     cones = track_data['left_cones'] + track_data['right_cones']
#     with open(filename, 'w', newline='') as csvfile:
#         fieldnames = ['x', 'y', 'side', 'color']
#         writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
#         writer.writeheader()kk
#         for cone in cones:
#             writer.writerow(cone)
#     print(f"Cones exported to CSV: {filename}")

# includes centerline, left, and right cone coordinates
def export_cones_to_csv(track_data, filename="fs_track.csv"):
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


def main():
    parser = argparse.ArgumentParser(description="Formula Student Track Generator")
    parser.add_argument('--force', action='store_true', help="Force re-generate the track even if it exists")
    parser.add_argument('--file', type=str, default='fs_track.json', help="Filename to save/load the track")
    parser.add_argument('--export-csv', action='store_true', help="Export cones to CSV format")
    parser.add_argument('--csv-file', type=str, default='fs_track.csv', help="CSV filename to export to (if --export-csv used)")
    args = parser.parse_args()

    if os.path.exists(args.file) and not args.force:
        print(f"Track file '{args.file}' already exists. Loading and visualizing...")
        with open(args.file, 'r') as f:
            track_data = json.load(f)
    else:
        print(f"{'--force used:' if args.force else 'Track file not found.'} Generating new track...")
        track_data = generate_formula_student_track(num_waypoints, track_width, cone_spacing, total_length)
        save_track(track_data, args.file)

    if args.export_csv:
        export_cones_to_csv(track_data, args.csv_file)

    visualize_track(track_data, show_centerline)

############################################  END OF FUNCTION DEFINITIONS ##################################

FILENAME = "fs_track.json"

num_waypoints = 18
track_width = 5
cone_spacing = 5
total_length = 500

show_centerline = 1

if __name__ == "__main__":
    main()
