import matplotlib.pyplot as plt
import numpy as np

class TrackVisualizer:
    @staticmethod
    def plot(track_data, show_centerline):
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