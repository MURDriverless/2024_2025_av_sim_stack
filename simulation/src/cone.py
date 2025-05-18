class Cone:
    def __init__(self, x, y, colour):
        """
        Initializes a Cone object.

        Args:
            x (float): X-coordinate of the cone.
            y (float): Y-coordinate of the cone.
            color (str): Color of the cone ('blue', 'yellow', etc.).
        """
        self.x = x
        self.y = y
        self.color = colour
        self.radius = 0.1

    def get_position(self):
        """Returns the (x, y) position of the cone."""
        return [self.x, self.y]

    def __repr__(self):
        return f"Cone(x={self.x}, y={self.y}, color='{self.color}')"
    


class TrackedCone:
    def __init__(self, x, y, color, id):
        self.x = x
        self.y = y
        self.color = color
        self.id = id
        self.last_seen = 0
        self.missed_count = 0  # How many frames in a row it's been missed
    
    def __repr__(self):
        return f"Cone(x={self.x}, y={self.y}, id='{self.id}')"

