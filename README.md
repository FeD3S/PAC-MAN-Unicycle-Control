# The PAC-MAN Challenge: Unicycle AGV Control

This repository contains the MATLAB and Simulink implementation for the "PAC-MAN Challenge." The project focuses on simulating an Automated Guided Vehicle (AGV), modeled as a non-holonomic unicycle, navigating through a maze. 

**Authors:**
* Federico Saporiti
* Matteo Bino
* Leonardo Luigi Pepe
Control Systems Engineering

## Project Overview

We tackled two fundamental control tasks for non-holonomic systems:

1. **Trajectory Tracking:** We developed and compared the performance of two different state-error feedback approaches: an approximate linearization controller and a non-linear controller. These were evaluated across multiple path geometries (linear, circular, elliptical, rectangular, and zigzag).
2. **Posture Regulation:** Since non-holonomic systems cannot be stabilized to a fixed point using smooth, time-invariant linear state feedback, we implemented a Cartesian posture control to execute the final parking maneuver.

## Repository Contents

* **`full_task` (Simulink):** The main Simulink model used for the final maze navigation simulation.
* **Testing & Strategy Files:** All other files in this repository contain the complete schemes to independently test and compare the linear and non-linear controllers. You can use these files to experiment with different path shapes, control strategies, and parameter tuning without running the entire final maze task.

## How to Run the Final Simulation

To execute the final "PAC-MAN" maze simulation, follow these steps strictly in order:

1. Open MATLAB and navigate to the project directory.
2. Run the initialization script by executing `sim_FT` in the command window.
3. Once initialized, run the final execution script by typing `final_sim`.
4. The simulation will utilize the `full_task` Simulink model to navigate the unicycle through the environment.
