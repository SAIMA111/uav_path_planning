# UAV Path Planning Simulation

This repository contains a prototype simulator for **multi-UAV path planning in dynamic environments**. The project was developed as part of my final-year work to explore how drones can **cooperate and adapt** when unexpected threats appear in their flight paths.  

The system is grid-based and deliberately lightweight — the focus is on **decision-making logic** rather than heavy graphics. However, the design makes it easy to extend to richer 3D simulators in the future.

---

##  Features

- **Grid environment**  
  The world is represented as a 2D grid where each cell is either free or blocked. UAVs move one step at a time in the four cardinal directions.  

- **A* path planning**  
  Each UAV uses the A* algorithm with Manhattan distance to compute an optimal path to its goal.  

- **Dynamic threats**  
  Circular no-fly zones appear randomly during the run. Affected cells instantly become obstacles, forcing UAVs to replan in real time. Threats expire after a random lifetime.  

- **Goal swapping**  
  If swapping destinations reduces the combined travel distance by more than 10%, UAVs trade goals mid-flight.  

- **Collision avoidance**  
  The coordinator prevents multiple UAVs from entering the same grid cell in the same step.  

- **Live visualisation**  
  Matplotlib shows the grid, UAVs, their planned paths, goals, and threats.  

---

##  Code Structure

uav_simulation/
│
├── path_planner.py
├── uav.py
├── threat_simulator.py
├── coordinator.py
├── visualizer.py
└── run_simulation.py


---

## Running the Simulation

Make sure you have **Python 3.8+**, **NumPy**, and **Matplotlib** installed.  

Run a basic demo with the UAVs:

```bash
python run_simulation.py --rows 20 --cols 20 --steps 200 --seed 42
What happens:

Two UAVs start from different corners and aim for opposite corners.

Random threats appear, forcing UAVs to replan.

If goal swapping reduces the total cost, UAVs exchange goals mid-flight.

You can stop the simulation anytime by closing the Matplotlib window.

To try the experimental D-Lite* planner, set use_dstar=True inside run_simulation.py.

 **Extending the Project**
Threats → Replace circular threats with moving obstacles or real sensor inputs.

Planners → Add new algorithms (LPA*, Jump-Point Search, etc.).

Communication layer → Simulate delays or limited bandwidth.

3D expansion → Port to simulators like AirSim or Webots.

** Current Limitations**
Threats are static once spawned; moving threats are not implemented.

D*-Lite support is partial and experimental.

Visualisation is basic.

**License**
This project is provided for educational purposes only.
No warranty is implied — use at your own risk.


