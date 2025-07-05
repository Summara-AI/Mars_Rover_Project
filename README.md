# 🚀 Mars Rover Path Planning and Navigation Simulation

This project demonstrates a simulated Mars rover navigating a grid-based environment with obstacles, using the A* pathfinding algorithm and the PyBullet physics engine. The rover autonomously plans a collision-free path and follows it using simple velocity control applied to its wheels.

The simulation is intended for educational and prototyping purposes, illustrating how a rover can use algorithmic path planning and basic locomotion control in a simulated Martian environment.

---

## 📂 Project Structure

mars_rover_simulation/
├── grid_map.py # Contains the GridMap class and A* path planning logic
├── rover_sim.py # Sets up the PyBullet simulation, renders obstacles and rover, follows path
└── README.md # Project documentation (this file)

yaml
Copy
Edit

---

## 🛠 Dependencies

This project requires:

- **Python 3.x**
- **PyBullet**

### Installation

To install PyBullet, simply run:

```bash
pip install pybullet
⚙ How to Run the Simulation
1️⃣ Make sure grid_map.py and rover_sim.py are in the same directory.
2️⃣ Launch the simulation by running:

bash
Copy
Edit
python rover_sim.py
3️⃣ A PyBullet GUI window will appear. The rover will automatically:

Plan a path from the start (0, 0) to the goal (7, 7)

Display that path in cyan

Move toward the goal while avoiding red-box obstacles

🧠 Project Logic
Grid and A* Path Planning
The grid is a 10x10 2D space.

Obstacles are placed at specific cells (e.g., (3,3), (3,4), (3,5)).

The GridMap class uses the A* search algorithm to compute the shortest path between the start and goal positions, taking obstacles into account.

Rover Simulation
The rover is modeled as a box chassis with four cylindrical wheels (black).

The rover starts at position (0, 0) and follows the path waypoint-by-waypoint.

The rover’s wheels are driven via simple velocity control, blending turning and forward movement to align with the path direction.

Visualization
Obstacles appear as red cubes.

The planned path is drawn as cyan lines.

The rover chassis is gray, with black wheels.

📌 Example Console Output
less
Copy
Edit
Planned path: [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (2, 3), (2, 4), (2, 5), (2, 6), (3, 6), (4, 6), (5, 6), (6, 6), (7, 6), (7, 7)]
🖥 Controls & Interaction
The simulation is fully autonomous; no user input is required during runtime.

Use your mouse in the PyBullet GUI window to zoom, pan, and rotate the view to observe the rover's motion from different angles.

🌟 Potential Future Enhancements
Implementing PID controllers for smoother and more realistic turning

Adding dynamic obstacles that can move during simulation

Integrating sensor simulation (e.g., LIDAR or stereo cameras) for perception-based navigation

Logging and visualizing rover telemetry (position, velocity, wheel speeds)

Simulating rough Martian terrain and inclines

🚀 Notes
The simulation assumes a flat plane with Martian gravity (3.7 m/s² downward).

The path planner assumes a static map (obstacles are fixed during the run).

👨‍💻 Author & Purpose
This project was created to demonstrate fundamental concepts in robotics navigation:

Grid-based path planning with A* algorithm

Simple wheeled rover kinematics

Visualization of planned paths and rover motion

It is meant as a foundational prototype for more advanced autonomous rover simulations in planetary exploration contexts, including Mars.

🔗 References
PyBullet Physics Engine

A* Pathfinding Algorithm - Stanford CS

📌 Disclaimer
This is a simplified educational simulation and does not represent the full complexity of actual planetary rover navigation systems, which incorporate far more sophisticated sensors, controls, and terrain analysis. However, it provides a strong starting point for understanding the core principles.

