# Sphero Collision Simulation and Data Analysis

This repository contains a Blender 3D physics simulation of a Sphero rolling and colliding with a custom energy absorption barrier. It includes physics data logging scripts and sample data for analysis, as well as Python plotting scripts to visualize the simulation results.

---

## Repository Contents

- **Blender Scripts/**  
  Blender Python scripts to set up the scene, run the simulation, and log physics data to CSV files.

- **Data/**  
  Sample CSV datasets generated from the simulation, including `sphero_log_sample.csv`.

- **Analysis/**  
  Python scripts using `pandas` and `matplotlib` for plotting and analyzing the simulation data.

---

## Features

- Fully rigged Blender scene including Sphero, barrier, wall, and ground.
- Physics simulation using Blender’s rigid body and force fields.
- Physics logging of position, velocity, acceleration, force, momentum, energy, and impact events.
- Synthetic realistic dataset generation for offline analysis.
- Python plotting scripts for visualization of motion and collision data.
- Google Colab-ready notebooks for easy data upload and plotting.

---

## Usage

### Running the Blender Simulation

1. Open Blender and load the provided Python scripts in the Scripting tab.
2. Adjust file paths as needed (default saves to `Documents/SpheroRender/`).
3. Run the simulation script to bake physics and output CSV logs.

### Analyzing Data with Python

1. Upload or download the CSV dataset (`sphero_log_sample.csv`).
2. Use the provided plotting scripts or Google Colab notebooks to visualize:
   - Position, velocity, acceleration over time
   - Energy profiles and impact events
   - Force and momentum dynamics

---

## Requirements

- Blender 2.83 or later for simulation scripts.
- Python 3.x environment with:
  - `pandas`
  - `matplotlib`

For Google Colab notebooks, no local setup is required beyond an internet browser.

---

## How to Use Google Colab for Analysis

1. Open the provided Colab notebook or create a new one.
2. Upload your CSV data using the upload cell.
3. Run the plotting cells to generate graphs and insights.

---

## License

This project is licensed under the MIT License — see the LICENSE file for details.

---

## Contact

For questions or contributions, please open an issue or contact [Your Name or Email].

---

Thank you for checking out the Sphero Collision Simulation project! 🚀
