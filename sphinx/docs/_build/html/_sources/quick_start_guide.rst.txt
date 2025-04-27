Quick Start Guide
=================

.. note::
    WORK IN PROGRESS

Overview
--------

So what defines an autonomous vehicle? Well, for starters—it drives itself. Shocking, we know.

Designing an FSAE car is already a complex task, but going autonomous means adding a whole new layer of engineering wizardry. On top of the traditional FSAE design process, teams must now integrate a fully-fledged autonomous system. If a team is particularly fortunate—or flush with funding—they might have the luxury of two separate vehicles: one for humans, and one purely for autonomy.

But for the rest of us broke teams, the autonomous system is developed in tandem with the same car that has a driver’s seat. This parallel development approach brings its own set of challenges and compromises.

Beyond the typical components of a driver-operated FSAE car (which we won’t get into here), an autonomous vehicle requires a suite of sensors, processors, and computing hardware to bring the system to life. The biggest friction point between the autonomous team and the conventional EV subteams in MUR usually comes down to one word: packaging. Fitting all that extra tech into an already cramped vehicle demands excellent communication and collaboration across subteams—because nobody wants to play Tetris with lidar mounts at 2 a.m in the MUR garage.

When it comes to the software, the autonomous architecture consists of many different **pipelines** that ensures modularity and isolation of each sub-system:

- :doc:`Perception <perception/index>`
- :doc:`Pathing <pathing/index>`
- SLAM
- :doc:`Controls <controls/index>`

A pipeline is basically a series of steps (or stages) where each step processess data and passes it along to the next step. You can think of it as an assembly line in a factory where raw material goes in (input), each station adds something or modifies it (data processing), and returns a finished product (output of the pipeline). These inputs and outputs may be interdependent with other pipelines as we will see in their integration with one another. We will go over these pipelines in detail in subsequent sections.

System requirements
-------------------

When it comes to computing, MUR has provided an NVIDIA jetson for any autonomous development. However, having your own device for development is preferred. Although not required, having a discrete GPU (dGPU), preferably NVIDIA, allows for efficient training of any machine learning algorithms, rendering of simulations, and any other GPU offloading activity.

Esnure that you are familiar and have the ability to use Linux, whether you run it natively, dual booting it, running in a virtual machine, or using Windows Subsystem for Linux (WSL).