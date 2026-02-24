# Operating Manual: 6DOF Strain Sensor Simulation

## Purpose

Simulates capacitive strain sensor data for a soft robot and reconstructs the 3D shape using the Geometric Variable-Strain (GVS) model. The C program generates fake AD7147 CDC readings, converts them to 6D strain, integrates the rod shape, and outputs CSV files that MATLAB reads for visualization.

## Requirements

- GCC compiler (or any C compiler)
- MATLAB R2019b or later

## File Locations

```
rod_sim/
  firmware_sim/main.c    -- C simulation source
  protocol/packets.h     -- packet format header
  host/receiver.m        -- MATLAB data loader
  host/render.m          -- MATLAB 3D visualization
  data/logs/             -- output directory for CSV files
```

## Running the Simulation

### Step 1: Compile

```
cd rod_sim/firmware_sim
gcc -o sim_sensor main.c -lm
```

### Step 2: Run

```
./sim_sensor
```

Creates:
- `../data/logs/strain_data.csv`
- `../data/logs/shape_data.csv`

### Step 3: Visualize in MATLAB

```
cd rod_sim/host
render
```

Or load data manually:

```
[strain, shape] = receiver('../data/logs');
```

## Data Format

strain_data.csv columns:
```
timestamp_ms, eps1_s1, eps2_s1, eps3_s1, eps4_s1, eps5_s1, eps6_s1,
              eps1_s2, eps2_s2, eps3_s2, eps4_s2, eps5_s2, eps6_s2,
              eps1_s3, eps2_s3, eps3_s3, eps4_s3, eps5_s3, eps6_s3
```

shape_data.csv columns:
```
timestamp_ms, point_idx, x, y, z
```

Each timestamp has 101 points (indices 0-100) tracing the robot centerline.

## Strain Convention

- eps1: twist (rotation about rod axis), rad/m
- eps2: bending about local y, rad/m
- eps3: bending about local z, rad/m
- eps4: axial stretch, m/m
- eps5: shear y, m/m
- eps6: shear z, m/m

## Parameters

In main.c:
- ROBOT_LENGTH: 0.165 m
- NUM_SENSORS: 3
- SIM_DURATION_MS: 30000
- SAMPLE_PERIOD_MS: 50

Sensor positions: X = 0, L/3, 2L/3

---

## TODO

### Critical

- [ ] Calibration matrix values are placeholder. Replace with actual least-squares calibration from real sensor hardware.
- [ ] Baseline capacitance values are hardcoded. Should be measured at startup when robot is straight.
- [ ] Strain-to-capacitance model is simplified (diagonal matrix). Real sensor has cross-coupling between channels.

### Firmware Simulation

- [ ] Add noise model to fake capacitance readings (Gaussian noise, drift, quantization).
- [ ] Implement temperature compensation placeholder.
- [ ] Add hysteresis model for silicone dielectric behavior.
- [ ] Support variable number of sensors (currently hardcoded to 3).
- [ ] Add command-line arguments for simulation parameters instead of recompiling.
- [ ] Output binary packet format in addition to CSV for testing real receiver code.

### Shape Integration

- [ ] Current integration uses forward Euler. Consider higher-order method (RK4) for better accuracy.
- [ ] Add option to output rotation matrices at each point, not just positions.
- [ ] Strain interpolation uses piecewise linear. Paper mentions other basis options (quadratic, constant segments).

### MATLAB Visualization

- [ ] Add strain vs time subplot alongside 3D view.
- [ ] Add option to export animation as video file.
- [ ] Improve tube rendering performance for real-time playback.
- [ ] Add tip trajectory trace overlay.
- [ ] Color mapping should optionally show elongation (eps4) instead of curvature.
- [ ] Add pause/resume and frame stepping controls.

### Protocol

- [ ] packets.h defines binary format but simulation only outputs CSV. Implement binary output.
- [ ] Add CRC or checksum field for packet validation.
- [ ] Define error/status packet for real hardware communication.

### Testing

- [ ] Add known-shape test cases (pure bend, pure twist, helix) to validate integration math.
- [ ] Compare integrated shape against analytical solution for constant curvature.
- [ ] Unit tests for Rodrigues formula implementation.

### Documentation

- [ ] Add derivation of calibration matrix from paper supplementary material.
- [ ] Document coordinate frame conventions (which axis is along rod, which is up).
- [ ] Add block diagram of data flow from capacitance to 3D shape.
