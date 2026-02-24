import numpy as np

def generate_sensor_data(x, y, z, rx, ry, rz):
    """
    Takes desired 6DoF movements (strains) and calculates the 6 raw sensor values.
    Returns: [h1, v1, h2, v2, h3, v3]
    """
    # 1. Define the Mixing Matrix (M) based on 120-degree symmetry
    # This is a simplified version of the geometric relationship
    # Rows: X, Y, Z, Rx, Ry, Rz
    # Cols: h1, v1, h2, v2, h3, v3
    M = np.array([
        [ 1.0,  0.0, -0.5,  0.0, -0.5,  0.0], # X
        [ 0.0,  0.0,  0.86, 0.0, -0.86, 0.0], # Y
        [ 0.0,  1.0,  0.0,  1.0,  0.0,  1.0], # Z
        [ 0.0,  1.0,  0.0, -0.5,  0.0, -0.5], # Rx
        [ 0.0,  0.0,  0.0,  0.86, 0.0, -0.86], # Ry
        [ 1.0,  0.0,  1.0,  0.0,  1.0,  0.0]  # Rz
    ])

    # 2. Invert the matrix to go from Movement -> Sensors
    M_inv = np.linalg.inv(M)

    # 3. Create the movement vector
    movement = np.array([x, y, z, rx, ry, rz])

    # 4. Calculate synthetic sensor data
    # raw_sensors = M_inv * movement
    raw_sensors = np.dot(M_inv, movement)
    
    return raw_sensors

# --- Example Usage ---

# Suppose we want to simulate a "Twist" (Rz) and a "Pull Up" (Z)
desired_x  = 0.0
desired_y  = 0.0
desired_z  = 10.0   # Pulling up
desired_rx = 0.0
desired_ry = 0.0
desired_rz = 5.0    # Twisting clockwise

sensors = generate_sensor_data(desired_x, desired_y, desired_z, 
                               desired_rx, desired_ry, desired_rz)

labels = ['Station 1 Horiz (h1)', 'Station 1 Vert (v1)', 
          'Station 2 Horiz (h2)', 'Station 2 Vert (v2)', 
          'Station 3 Horiz (h3)', 'Station 3 Vert (v3)']

print(f"Target Movement: Z={desired_z}, Rz={desired_rz}\n")
print("Generated Synthetic Sensor Values:")
for label, val in zip(labels, sensors):
    print(f"{label}: {val:>8.2f}")

