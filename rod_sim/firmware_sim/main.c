/**
 * main.c - Simulated 6DOF Strain Sensor Firmware
 * 
 * Generates fake capacitance data from AD7147 CDC, converts to 6D strain,
 * and outputs to a CSV log file for MATLAB visualization.
 * 
 * Based on the Flex-2P6D sensor and GVS (Geometric Variable-Strain) model
 * from "Advancing Soft Robot Proprioception Through 6D Strain Sensors Embedding"
 * 
 * The 6D strain components are:
 *   ε₁ = twist (rotation about local x-axis)     [rad/m]
 *   ε₂ = bending about y-axis                    [rad/m]
 *   ε₃ = bending about z-axis                    [rad/m]
 *   ε₄ = stretch/elongation                      [m/m]
 *   ε₅ = shear y                                 [m/m]
 *   ε₆ = shear z                                 [m/m]
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "../protocol/packets.h"

/* Robot parameters */
#define ROBOT_LENGTH        0.165f      /* 165 mm total length */
#define NUM_INTEGRATION_STEPS 100       /* Integration steps for shape */
#define DX (ROBOT_LENGTH / (float)NUM_INTEGRATION_STEPS)

/* Simulation parameters */
#define SIM_DURATION_MS     30000       /* 30 seconds of simulation */
#define SAMPLE_PERIOD_MS    50          /* 20 Hz sample rate */
#define PI 3.14159265358979f

/* Sensor positions along the rod (normalized 0 to 1) */
static const float sensor_positions[NUM_SENSORS] = {0.0f, 0.333f, 0.667f};

/* Calibration matrix (maps delta capacitance to strain) */
/* In practice, this comes from least-squares calibration */
static const float cal_matrix[STRAIN_DIMS][STRAIN_DIMS] = {
    { 0.0020f, 0.0001f, 0.0000f, 0.0000f, 0.0000f, 0.0000f },  /* twist */
    { 0.0001f, 0.0020f, 0.0001f, 0.0000f, 0.0000f, 0.0000f },  /* bend y */
    { 0.0000f, 0.0001f, 0.0020f, 0.0000f, 0.0000f, 0.0000f },  /* bend z */
    { 0.0000f, 0.0000f, 0.0000f, 0.0003f, 0.0000f, 0.0000f },  /* stretch */
    { 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0003f, 0.0000f },  /* shear y */
    { 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0000f, 0.0003f }   /* shear z */
};

/* Baseline capacitance (when robot is straight) */
static uint16_t baseline_cap[NUM_SENSORS][STRAIN_DIMS] = {
    {5000, 5200, 5100, 5300, 5150, 5250},
    {5050, 5180, 5120, 5280, 5170, 5230},
    {5020, 5220, 5090, 5310, 5140, 5260}
};

/*----------------------------------------------------------------------------
 * Linear Algebra Helpers
 *---------------------------------------------------------------------------*/
typedef struct { float x, y, z; } vec3;
typedef struct { float R[9]; vec3 p; } SE3;

static inline vec3 vec3_new(float x, float y, float z) {
    vec3 v = {x, y, z}; return v;
}

static inline vec3 vec3_add(vec3 a, vec3 b) {
    return vec3_new(a.x + b.x, a.y + b.y, a.z + b.z);
}

static inline vec3 vec3_scale(vec3 a, float s) {
    return vec3_new(a.x * s, a.y * s, a.z * s);
}

static vec3 mat3_mul_vec(const float R[9], vec3 v) {
    return vec3_new(
        R[0]*v.x + R[1]*v.y + R[2]*v.z,
        R[3]*v.x + R[4]*v.y + R[5]*v.z,
        R[6]*v.x + R[7]*v.y + R[8]*v.z
    );
}

static void mat3_mul_mat(float out[9], const float A[9], const float B[9]) {
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            out[r*3 + c] = 0.0f;
            for (int k = 0; k < 3; k++) {
                out[r*3 + c] += A[r*3 + k] * B[k*3 + c];
            }
        }
    }
}

static void mat3_identity(float R[9]) {
    memset(R, 0, 9 * sizeof(float));
    R[0] = R[4] = R[8] = 1.0f;
}

/**
 * Rodrigues formula: compute rotation matrix from angular velocity * dx
 * omega = (wx, wy, wz) angular velocity
 * R = exp(skew(omega) * dx)
 */
static void rodrigues(float R[9], vec3 omega, float dx) {
    float wx = omega.x * dx;
    float wy = omega.y * dx;
    float wz = omega.z * dx;
    float theta = sqrtf(wx*wx + wy*wy + wz*wz);
    
    /* Skew-symmetric matrix K */
    float K[9] = {
         0.0f, -wz,   wy,
         wz,   0.0f, -wx,
        -wy,   wx,   0.0f
    };
    
    float I[9];
    mat3_identity(I);
    
    if (theta < 1e-8f) {
        /* Small angle approximation: R ≈ I + K */
        for (int i = 0; i < 9; i++) R[i] = I[i] + K[i];
        return;
    }
    
    float s = sinf(theta) / theta;
    float c = (1.0f - cosf(theta)) / (theta * theta);
    
    /* K² */
    float K2[9];
    mat3_mul_mat(K2, K, K);
    
    /* R = I + sin(θ)/θ * K + (1-cos(θ))/θ² * K² */
    for (int i = 0; i < 9; i++) {
        R[i] = I[i] + s * K[i] + c * K2[i];
    }
}

/*----------------------------------------------------------------------------
 * Fake CDC (AD7147) Simulation
 *---------------------------------------------------------------------------*/

/**
 * Generate fake capacitance readings that simulate various deformations
 * Creates interesting time-varying shapes similar to the paper's examples
 * 
 * Target strain ranges (from paper Table 2):
 *   ε₁ (twist):   [-15, 15] rad/m
 *   ε₂ (bend y):  [-12, 12] rad/m
 *   ε₃ (bend z):  [-12, 12] rad/m
 *   ε₄ (stretch): [-0.1, 0.1] m/m
 *   ε₅ (shear y): [-0.25, 0.25] m/m
 *   ε₆ (shear z): [-0.2, 0.2] m/m
 */
static void fake_read_ad7147(uint16_t raw[NUM_SENSORS][STRAIN_DIMS], float t) {
    /* 
     * Create complex, visually interesting shapes:
     * - Large bending that creates curved/spiral shapes
     * - Variable strain along the length (different at each sensor)
     * - Some twist for 3D effect
     */
    
    /* Base amplitudes (will be scaled by calibration matrix) */
    /* With cal_matrix diagonal ~0.002 for rotation, need ~5000 delta for 10 rad/m */
    float base_bend = 4000.0f;
    float base_twist = 2000.0f;
    
    /* Time-varying pattern: creates sweeping, curling motion */
    float t_slow = 0.3f * t;   /* Slow variation */
    float t_med  = 0.5f * t;   /* Medium variation */
    
    for (int s = 0; s < NUM_SENSORS; s++) {
        float pos = sensor_positions[s];
        /* Phase varies along the rod for variable curvature */
        float phase = pos * PI * 1.5f;
        
        /* Position-dependent amplitude creates variable strain along rod */
        float pos_scale = 0.5f + 0.5f * pos;  /* Increases toward tip */
        
        /* Channel 0,1: Twist (ε₁) - modest twist */
        float twist = base_twist * pos_scale * sinf(t_slow + phase);
        raw[s][0] = baseline_cap[s][0] + (int16_t)(twist * 0.5f);
        raw[s][1] = baseline_cap[s][1] + (int16_t)(twist * 0.5f);
        
        /* Channel 2,3: Bending Y (ε₂) - primary bending direction */
        float bend_y = base_bend * (0.8f + 0.4f * sinf(t_med)) * sinf(t_slow + phase * 0.7f);
        raw[s][2] = baseline_cap[s][2] + (int16_t)(bend_y);
        raw[s][3] = baseline_cap[s][3] + (int16_t)(bend_y * 0.95f);
        
        /* Channel 4,5: Bending Z (ε₃) - secondary bending, phase shifted */
        float bend_z = base_bend * 0.7f * cosf(t_slow * 0.8f + phase);
        raw[s][4] = baseline_cap[s][4] + (int16_t)(bend_z);
        raw[s][5] = baseline_cap[s][5] + (int16_t)(bend_z * 0.95f);
    }
}

/**
 * Convert raw capacitance delta to 6D strain using calibration matrix
 */
static void capacitance_to_strain(const float delta[STRAIN_DIMS], float eps[STRAIN_DIMS]) {
    for (int i = 0; i < STRAIN_DIMS; i++) {
        eps[i] = 0.0f;
        for (int j = 0; j < STRAIN_DIMS; j++) {
            eps[i] += cal_matrix[i][j] * delta[j];
        }
    }
}

/*----------------------------------------------------------------------------
 * GVS Shape Reconstruction
 *---------------------------------------------------------------------------*/

/**
 * Linear interpolation of strain field using FEM-like basis functions
 * Given strains at 3 sensor positions, interpolate strain at position X
 */
static void interpolate_strain(const StrainData sensors[NUM_SENSORS], 
                               float X, float eps_out[STRAIN_DIMS]) {
    float L = ROBOT_LENGTH;
    float X1 = sensor_positions[0] * L;  /* 0 */
    float X2 = sensor_positions[1] * L;  /* L/3 */
    float X3 = sensor_positions[2] * L;  /* 2L/3 */
    
    /* 
     * Use piecewise linear interpolation:
     * - Linear between sensor 1 and 2 (0 to L/3)
     * - Linear between sensor 2 and 3 (L/3 to 2L/3)
     * - Constant extrapolation from sensor 3 (2L/3 to L)
     */
    
    for (int i = 0; i < STRAIN_DIMS; i++) {
        float e1 = sensors[0].epsilon[i];
        float e2 = sensors[1].epsilon[i];
        float e3 = sensors[2].epsilon[i];
        
        if (X <= X2) {
            /* Linear interpolation between sensor 1 and 2 */
            float t = (X - X1) / (X2 - X1 + 1e-8f);
            eps_out[i] = e1 * (1.0f - t) + e2 * t;
        } else if (X <= X3) {
            /* Linear interpolation between sensor 2 and 3 */
            float t = (X - X2) / (X3 - X2 + 1e-8f);
            eps_out[i] = e2 * (1.0f - t) + e3 * t;
        } else {
            /* Constant extrapolation from sensor 3 */
            eps_out[i] = e3;
        }
    }
}

/**
 * Integrate rod shape from strain field using SE(3) formulation
 * Returns array of 3D points along the rod centerline
 */
static void integrate_shape(const StrainData sensors[NUM_SENSORS],
                            Point3D points[], int num_points) {
    SE3 g;
    mat3_identity(g.R);
    g.p = vec3_new(0, 0, 0);
    
    for (int k = 0; k <= num_points; k++) {
        float X = k * DX;
        
        /* Store current position (swap axes for visualization: z down) */
        points[k].x = g.p.y;
        points[k].y = g.p.z;
        points[k].z = -g.p.x;  /* Negative so robot hangs down */
        
        if (k < num_points) {
            /* Get interpolated strain at this position */
            float eps[STRAIN_DIMS];
            interpolate_strain(sensors, X, eps);
            
            /* 
             * Strain convention:
             * omega = (ε₁, ε₂, ε₃) = angular velocity (twist, bend_y, bend_z)
             * v = (ε₅, ε₆, 1 + ε₄) = linear velocity (shear_y, shear_z, 1+stretch)
             */
            vec3 omega = vec3_new(eps[0], eps[1], eps[2]);
            vec3 v_local = vec3_new(eps[4], eps[5], 1.0f + eps[3]);
            
            /* Update rotation: R_new = R * exp(skew(omega) * dx) */
            float dR[9], R_new[9];
            rodrigues(dR, omega, DX);
            mat3_mul_mat(R_new, g.R, dR);
            memcpy(g.R, R_new, 9 * sizeof(float));
            
            /* Update position: p_new = p + R * v * dx */
            vec3 dp = mat3_mul_vec(g.R, vec3_scale(v_local, DX));
            g.p = vec3_add(g.p, dp);
        }
    }
}

/*----------------------------------------------------------------------------
 * Main Simulation Loop
 *---------------------------------------------------------------------------*/

int main(int argc, char *argv[]) {
    const char *output_dir = "../data/logs";
    char strain_path[256], shape_path[256];
    
    /* Create output directory (ignore error if exists) */
    char mkdir_cmd[512];
    snprintf(mkdir_cmd, sizeof(mkdir_cmd), "mkdir -p %s", output_dir);
    system(mkdir_cmd);
    
    snprintf(strain_path, sizeof(strain_path), "%s/strain_data.csv", output_dir);
    snprintf(shape_path, sizeof(shape_path), "%s/shape_data.csv", output_dir);
    
    FILE *strain_file = fopen(strain_path, "w");
    FILE *shape_file = fopen(shape_path, "w");
    
    if (!strain_file || !shape_file) {
        fprintf(stderr, "Error: Could not open output files\n");
        return 1;
    }
    
    /* Write headers */
    fprintf(strain_file, "%s\n", LOG_HEADER);
    fprintf(shape_file, "timestamp_ms,point_idx,x,y,z\n");
    
    printf("6DOF Strain Sensor Simulation\n");
    printf("==============================\n");
    printf("Robot length: %.3f m\n", ROBOT_LENGTH);
    printf("Number of sensors: %d\n", NUM_SENSORS);
    printf("Sample rate: %d Hz\n", 1000 / SAMPLE_PERIOD_MS);
    printf("Duration: %.1f s\n", SIM_DURATION_MS / 1000.0f);
    printf("\nOutput files:\n");
    printf("  Strain data: %s\n", strain_path);
    printf("  Shape data:  %s\n", shape_path);
    printf("\nSimulating...\n");
    
    /* Allocate shape points array */
    Point3D *shape_points = malloc((NUM_INTEGRATION_STEPS + 1) * sizeof(Point3D));
    if (!shape_points) {
        fprintf(stderr, "Error: Memory allocation failed\n");
        return 1;
    }
    
    int num_samples = 0;
    
    /* Main simulation loop */
    for (uint32_t t_ms = 0; t_ms < SIM_DURATION_MS; t_ms += SAMPLE_PERIOD_MS) {
        float t = t_ms / 1000.0f;
        
        /* Read fake capacitance data */
        uint16_t raw_cap[NUM_SENSORS][STRAIN_DIMS];
        fake_read_ad7147(raw_cap, t);
        
        /* Convert to strain for each sensor */
        StrainData sensors[NUM_SENSORS];
        
        for (int s = 0; s < NUM_SENSORS; s++) {
            float delta[STRAIN_DIMS];
            for (int i = 0; i < STRAIN_DIMS; i++) {
                delta[i] = (float)((int)raw_cap[s][i] - (int)baseline_cap[s][i]);
            }
            capacitance_to_strain(delta, sensors[s].epsilon);
        }
        
        /* Write strain data to log */
        fprintf(strain_file, "%u", t_ms);
        for (int s = 0; s < NUM_SENSORS; s++) {
            for (int i = 0; i < STRAIN_DIMS; i++) {
                fprintf(strain_file, ",%.6f", sensors[s].epsilon[i]);
            }
        }
        fprintf(strain_file, "\n");
        
        /* Integrate shape and write to log */
        integrate_shape(sensors, shape_points, NUM_INTEGRATION_STEPS);
        
        for (int k = 0; k <= NUM_INTEGRATION_STEPS; k++) {
            fprintf(shape_file, "%u,%d,%.6f,%.6f,%.6f\n",
                    t_ms, k, shape_points[k].x, shape_points[k].y, shape_points[k].z);
        }
        
        num_samples++;
        
        /* Progress indicator */
        if (t_ms % 5000 == 0) {
            printf("  t = %.1f s\n", t);
        }
    }
    
    printf("\nSimulation complete!\n");
    printf("Generated %d samples\n", num_samples);
    
    /* Cleanup */
    free(shape_points);
    fclose(strain_file);
    fclose(shape_file);
    
    return 0;
}
