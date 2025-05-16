Simultaneous Localization and Mapping (SLAM)
============================================

What is SLAM? To appreciate the magic of SLAM—*Simultaneous Localization and Mapping*—we first need to unpack its two core components: **mapping** and **localization**.

Let’s begin with mapping. At its core, mapping is the act of reconstructing the environment around an autonomous system. Perception systems allow the vehicle to *see* its surroundings, but vision alone is fleeting. Without mapping, the system is like someone watching a movie with amnesia—everything seen is quickly forgotten. Mapping, then, is the mechanism that takes those raw perceptions and assembles them into a persistent, navigable model of the world.

Of course, maps aren’t always perfect—just as our memories can blur over time, mapping algorithms often construct an approximate representation of reality. After all, these systems don’t have perfect recall either.

Next up is localization. Once we have a map, the next question is: *Where am I on this map?* Localization answers this by estimating the vehicle's position and orientation (collectively called its *pose*) with respect to the map. It’s like dropping a pin on Google Maps, but done continuously and with far more math.

So where does SLAM come in? Traditionally, mapping and localization were done in sequence: first build a map, then localize within it. But real-world autonomous systems—especially in fast-paced domains like autonomous racing—don’t have the luxury of waiting. Enter SLAM: a real-time algorithmic framework that builds the map *and* localizes the vehicle within it *at the same time*. 

Hence the “simultaneous” in *Simultaneous Localization and Mapping*—a computational two-for-one deal that allows autonomous systems to explore and understand the world without needing a pre-made map. It's like learning where you are while simultaneously sketching the map on a moving train.
