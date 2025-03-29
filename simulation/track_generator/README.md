# Track Generation

This is the README for the track generator. What this script does is it creates a randomized FSAE style track demarcated by yellow and blue cones, as well as shows the ground truth center line of the track.

The generated track will automatically be saved as a json file in the current directory with the left cone, right cone, and centerline coordinates. The filename will be saved as fs_track.json and if this file does not exist in the current directory, the track generator will generate a new track with the given parameters. If the json file does exist, then it will load that file and plot it.

## Configurable parameters in the file are:

> num_waypoints = integer # This signifies how "curvy/jagged" the track is. The higher the value, the more curvy/jagged the track is. Ideal values are between 10-22.

> track_width = integer # This determines the spacing of the cones track width / lateral spacing

> cone_spacing = integer # This determines the spacing of cones longitudonally

> total_length = integer # This determines the total length of points on the track

> show_centerline = 0 or 1 # This option determines whether the centerline is shown on the plot. Note that the coordinates of the centerline is still known/saved

## CLI options
File is executed by:
> python track_generator.py

CLI options are described in the help section:
> python track_generator.py --help

CLI options include:
> usage: track_generator.py [-h] [--force] [--file FILE] [--export-csv] [--csv-file CSV_FILE] 
> 
> Formula Student Track Generator options:
>  
> 
> -h, --help show this help message and exit 
> 
> --force Force re-generate the track even if it exists 
> 
> --file FILE Filename to save/load the track 
> 
> --export-csv Export cones to CSV format 
> 
> --csv-file CSV_FILE CSV filename to export to (if --export-csv used)

Examples:
> python track_generator.py --force --export-csv