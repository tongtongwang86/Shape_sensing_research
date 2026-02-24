# 6DOF Strain Sensor Simulation (rod_sim)

Simulation suite for a Flex-2P6D capacitive 6DOF strain sensor embedded in a soft robot.
Based on the Geometric Variable-Strain (GVS) model for 3D shape reconstruction.

## Overview

This project simulates data from embedded capacitive strain sensors and reconstructs
the 3D shape of a soft robot manipulator. The approach is based on:

> "Advancing Soft Robot Proprioception Through 6D Strain Sensors Embedding"  
> Feliu-Talegon et al., Soft Robotics, 2025

### 6D Strain Components

The Flex-2P6D sensor measures 6 strain components at each location along the robot:

| Strain | Symbol | Description | Units | Range |
|--------|--------|-------------|-------|-------|
| ε₁ | Twist | Rotation about local X-axis | rad/m | [-15, 15] |
| ε₂ | Bend Y | Rotation about local Y-axis | rad/m | [-12, 12] |
| ε₃ | Bend Z | Rotation about local Z-axis | rad/m | [-12, 12] |
| ε₄ | Stretch | Elongation along X-axis | m/m | [-0.1, 0.1] |
| ε₅ | Shear Y | Shear in Y direction | m/m | [-0.25, 0.25] |
| ε₆ | Shear Z | Shear in Z direction | m/m | [-0.2, 0.2] |

## Project Structure

```
rod_sim/
├── firmware_sim/
│   └── main.c          # Simulated CDC + strain + shape integration
├── protocol/
│   └── packets.h       # Binary packet format definition
├── host/
│   ├── receiver.m      # MATLAB data loader
│   └── render.m        # MATLAB 3D visualization
├── data/
│   └── logs/           # Output CSV files
└── README.md
```

---

## Quick Start Guide

### Step 1: Compile the C Simulation

Open a terminal and navigate to the firmware directory:

```bash
cd rod_sim/firmware_sim
gcc -o sim_sensor main.c -lm
```

### Step 2: Generate Simulated Data

Run the simulation to generate strain and shape data:

```bash
./sim_sensor
```

**Expected output:**
```
6DOF Strain Sensor Simulation
==============================
Robot length: 0.165 m
Number of sensors: 3
Sample rate: 20 Hz
Duration: 30.0 s

Output files:
  Strain data: ../data/logs/strain_data.csv
  Shape data:  ../data/logs/shape_data.csv

Simulating...
  t = 0.0 s
  t = 5.0 s
  ...
Simulation complete!
Generated 600 samples
```

This creates two CSV files in `data/logs/`:
- **strain_data.csv** - 6D strain measurements from 3 sensors over time
- **shape_data.csv** - Integrated 3D shape points (101 points along robot)

### Step 3: Visualize in MATLAB

Open MATLAB and navigate to the `host/` directory:

```matlab
cd('path/to/rod_sim/host')
```

Run the visualization:

```matlab
render
```

**What you'll see:**
- Animated 3D tube showing the soft robot shape
- Color-coded curvature (blue = low, red = high)
- Coordinate frame (RGB arrows) at the robot tip
- Time stamp display

### Optional: Load Data Manually

To work with the data directly:

```matlab
[strain, shape] = receiver('../data/logs');

% strain.timestamps  - Nx1 time vector (ms)
% strain.epsilon     - Nx3x6 strain array (samples x sensors x dims)

% shape.timestamps   - Mx1 time vector (ms)  
% shape.points       - MxPx3 position array (samples x points x xyz)
```

---

## Data File Formats

### strain_data.csv

Header: `timestamp_ms, eps1_s1, eps2_s1, ..., eps6_s1, eps1_s2, ..., eps6_s3`

- 19 columns total: 1 timestamp + 3 sensors × 6 strain components
- Each row is one time sample at 20 Hz

### shape_data.csv

Header: `timestamp_ms, point_idx, x, y, z`

- Each timestamp has 101 rows (point indices 0-100)
- Points trace the robot centerline from base to tip

---

## Mathematical Model

The GVS (Geometric Variable-Strain) model uses the SE(3) Lie group:

1. **Strain interpolation** - FEM-like linear basis between sensor locations
2. **Angular velocity** ω = (ε₁, ε₂, ε₃) defines local rotation rate
3. **Linear velocity** v = (ε₅, ε₆, 1+ε₄) defines local translation rate
4. **Shape integration** using Rodrigues formula:
   ```
   R(X+dX) = R(X) · exp(skew(ω)·dX)
   p(X+dX) = p(X) + R(X) · v · dX
   ```

---

## Configuration

Edit parameters in `firmware_sim/main.c`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ROBOT_LENGTH` | 0.165 m | Total robot length |
| `NUM_SENSORS` | 3 | Number of Flex-2P6D sensors |
| `SIM_DURATION_MS` | 30000 | Simulation duration (ms) |
| `SAMPLE_PERIOD_MS` | 50 | Sample period (20 Hz) |

After changing parameters, recompile and re-run the simulation.

---

## Troubleshooting

**"File not found" error in MATLAB:**
- Make sure you ran the C simulation first
- Check that you're in the `host/` directory
- Verify `../data/logs/` contains the CSV files

**Compilation error on macOS:**
- Ensure Xcode Command Line Tools are installed: `xcode-select --install`

**Robot appears too straight:**
- The simulation creates time-varying deformations
- Watch the full animation or check different time stamps
