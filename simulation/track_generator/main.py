import argparse
import os
from track import Track
from visualizer import TrackVisualizer
from exporter import TrackExporter

def main(num_waypoints, track_width, cone_spacing, total_length, show_centerline):
    parser = argparse.ArgumentParser(description="Formula Student Track Generator")
    parser.add_argument('--force', action='store_true', help="Force re-generate the track even if it exists")
    parser.add_argument('--file', type=str, default='fs_track.json', help="Filename to save/load the track")
    parser.add_argument('--export-csv', action='store_true', help="Export cones to CSV format")
    parser.add_argument('--csv-file', type=str, default='fs_track.csv', help="CSV filename to export to (if --export-csv used)")
    args = parser.parse_args()

    if os.path.exists(args.file) and not args.force:
        print(f"Track file '{args.file}' already exists. Loading and visualizing...")
        with open(args.file, 'r') as f:
            import json
            track_data = json.load(f)

    else:
        print(f"{'--force used:' if args.force else 'Track file not found.'} Generating new track...")
        track = Track(num_waypoints, track_width, cone_spacing, total_length)
        track.generate_track()
        track_data = track.get_track_data()
        print(track.get_start_finish_line)

        print(f"Saving track to json...")
        TrackExporter.to_json(track_data, args.file)

    if args.export_csv:
        print(f"Saving track to csv...")
        TrackExporter.to_csv(track_data, args.csv_file)

    TrackVisualizer.plot(track_data, show_centerline)

############################################  END OF FUNCTION DEFINITIONS ##################################

FILENAME = "fs_track.json"

num_waypoints = 18
track_width = 5
cone_spacing = 5
total_length = 500

show_centerline = 1

if __name__ == "__main__":
    main(num_waypoints, track_width, cone_spacing, total_length, show_centerline)