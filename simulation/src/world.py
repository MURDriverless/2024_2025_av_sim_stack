worlds = []
DT = 0.001


class World:
    def __init__(self, name):
        self.name = name
        self.tracks = []
        worlds.append(self)

    def add_track(self, track):
        self.tracks.append(track)

    def add_car_to_track(self, car, track_name):
        """
        Adds a car to the specified track by name.
        """
        for track in self.tracks:
            if track.name == track_name:
                track.add_car(car)
                return
        raise ValueError(f"Track '{track_name}' not found in world.")

    def get_track(self, name):
        for track in self.tracks:
            if track.name == name:
                return track
        raise ValueError(f"Track '{name}' not found.")

    def __repr__(self):
        return f"World(tracks={len(self.tracks)})"