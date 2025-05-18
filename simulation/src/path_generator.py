import math
import numpy as np
from scipy.spatial import Delaunay


class PathGenerator:
    def __init__(self):
        self.data = None

    def find_centerline(self, forward_cones):

        cone_positions = np.array([cone.get_position() for cone in forward_cones])
        cone_colors = np.array([cone.color for cone in forward_cones])

        tri_vis = Delaunay(cone_positions)                                                  # Compute Delaunay Triangulation
        simplices = tri_vis.simplices
        triangle_colors = cone_colors[simplices]

        mask = (
        (np.any(triangle_colors == 'yellow', axis=1) & np.any(triangle_colors == 'blue', axis=1)) |
        (np.sum(triangle_colors == 'orange', axis=1) == 2)
            )       
        
        valid_triangles = simplices[mask]

        data = {
            'path': [],            # You can replace [] with your actual path data
            'path boundary': [],   # Replace [] with boundary data if available
            'valid edges': [],      # Replace [] with valid edges data
            'seen edges': []
        }

        for simplex in valid_triangles:
            for i in range(3):                                                              # Each triangle has 3 edges
                p1, p2 = sorted([simplex[i], simplex[(i + 1) % 3]])                         # Sort to avoid duplicate edges
                edge = (p1, p2)

                if edge in data['seen edges']:
                    continue                                                                # Skip if already processed
                data['seen edges'].append(edge)

                cone1 = forward_cones[p1]
                cone2 = forward_cones[p2]

                color1 = cone1.color
                color2 = cone2.color

                if ('orange' not in [color1,color2]):
                    
                    if (color1 == color2):
                        data['path boundary'].append((cone1, cone2))

                    else:
                        
                        data['valid edges'].append((cone1, cone2))

                        midpoint = (
                            (cone1.x + cone2.x) / 2.0,
                            (cone1.y + cone2.y) / 2.0
                        )
                        data['path'].append(midpoint)
                        
                else:
                    if (color1 == color2):
                        data['valid edges'].append((cone1, cone2))

                        midpoint = (
                            (cone1.x + cone2.x) / 2.0,
                            (cone1.y + cone2.y) / 2.0
                        )
                        data['path'].append(midpoint)
                    
                    else:
                        dx = cone1.get_position()[0] - cone2.get_position()[0]
                        dy = cone1.get_position()[1] - cone2.get_position()[1]

                        if math.hypot(dx, dy) > 2:
                            data['valid edges'].append((cone1, cone2))

                            midpoint = (
                                (cone1.x + cone2.x) / 2.0,
                                (cone1.y + cone2.y) / 2.0
                            )
                            data['path'].append(midpoint)

                        else:
                            data['path boundary'].append((cone1, cone2))

        return data




class SlowLap(PathGenerator):
    def __init__(self):
        self.data = None

    def find_centerline(self, forward_cones):
        self.data = super().find_centerline(forward_cones)
    
    def update(self, forward_cones):
        self.find_centerline(forward_cones)
        return self.data



class FastLap(PathGenerator):
    def __init__(self):
        self.data = None

    def find_centerline(self, forward_cones):
        self.data = super().find_centerline(forward_cones)
    
    def update(self, forward_cones):
        self.find_centerline(forward_cones)
        return self.data