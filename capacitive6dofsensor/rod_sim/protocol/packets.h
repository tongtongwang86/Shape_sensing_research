/**
 * packets.h - Binary packet format for 6DOF strain sensor data
 * 
 * The Flex-2P6D sensor measures 6D strain:
 *   - epsilon[0] (ε₁): Twist (rotation about X-axis) [rad/m]
 *   - epsilon[1] (ε₂): Bending Y (rotation about Y-axis) [rad/m]  
 *   - epsilon[2] (ε₃): Bending Z (rotation about Z-axis) [rad/m]
 *   - epsilon[3] (ε₄): Stretch/elongation along X [m/m]
 *   - epsilon[4] (ε₅): Shear Y [m/m]
 *   - epsilon[5] (ε₆): Shear Z [m/m]
 */

#ifndef PACKETS_H
#define PACKETS_H

#include <stdint.h>

#define PACKET_SYNC_BYTE    0xAA
#define PACKET_VERSION      0x01
#define NUM_SENSORS         3       // Number of Flex-2P6D sensors along the rod
#define STRAIN_DIMS         6       // 6D strain per sensor
#define NUM_INTEGRATION_PTS 100     // Number of points for shape integration

#pragma pack(push, 1)

/**
 * Raw capacitance data from AD7147 CDC
 * 6 channels per sensor (CR1, CL1, CR2, CL2, CR3, CL3)
 */
typedef struct {
    uint16_t raw[STRAIN_DIMS];      // Raw ADC codes
} CapacitanceData;

/**
 * Calibrated 6D strain at one sensor location
 */
typedef struct {
    float epsilon[STRAIN_DIMS];     // [rad/m, rad/m, rad/m, m/m, m/m, m/m]
} StrainData;

/**
 * 3D position point
 */
typedef struct {
    float x, y, z;
} Point3D;

/**
 * Full sensor packet - sent at each time step
 */
typedef struct {
    uint8_t sync;                   // PACKET_SYNC_BYTE (0xAA)
    uint8_t version;                // PACKET_VERSION
    uint32_t timestamp_ms;          // Milliseconds since start
    uint8_t num_sensors;            // Number of sensors in this packet
    StrainData sensors[NUM_SENSORS]; // Strain data for each sensor
} SensorPacket;

/**
 * Shape reconstruction data - integrated rod shape
 */
typedef struct {
    uint8_t sync;                   // PACKET_SYNC_BYTE (0xAA)
    uint8_t version;                // PACKET_VERSION
    uint32_t timestamp_ms;          // Milliseconds since start
    uint16_t num_points;            // Number of integration points
    Point3D points[];               // Variable-length array of 3D points
} ShapePacket;

#pragma pack(pop)

/**
 * Text-based log format for MATLAB compatibility
 * Each line: timestamp_ms, eps1_s1, eps2_s1, ..., eps6_s1, eps1_s2, ..., eps6_s3
 * Total: 1 timestamp + 3 sensors * 6 strains = 19 values per line
 */
#define LOG_HEADER "timestamp_ms,eps1_s1,eps2_s1,eps3_s1,eps4_s1,eps5_s1,eps6_s1,eps1_s2,eps2_s2,eps3_s2,eps4_s2,eps5_s2,eps6_s2,eps1_s3,eps2_s3,eps3_s3,eps4_s3,eps5_s3,eps6_s3"

#endif /* PACKETS_H */
