/*
   ================================================
   Dead-Wheel Odometry with BNO055 IMU and Encoders
   ================================================

   Wheel Placement Diagram:

           (Forward)
              ↑
              │
              │
       +--------------+
       |              |
       |      ⊙       |   ⊙  = Robot Center (Reference point)
       |              |
       |   <--->      |
       +--------------+
              │
              │
              │
       [Parallel Wheel]  ← Dead wheel mounted parallel to the forward direction.
       
       [Perpendicular Wheel]
               ⟲
         (mounted perpendicular to the parallel wheel)
         (typically on the side of the robot)

   In this example, we assume:
     - The "Parallel Wheel" measures forward/backward motion.
     - The "Perpendicular Wheel" measures lateral motion.
     - The robot's heading is obtained from the BNO055 IMU.
     
   Adjust the placement in your kinematic model as needed.
*/

#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// =======================
// Define encoder pins
// =======================
// Update these pin numbers to match your wiring
#define ENC_PARALLEL_A 2
#define ENC_PARALLEL_B 3
#define ENC_PERP_A     4
#define ENC_PERP_B     5

// Create Encoder objects for each dead wheel
Encoder encParallel(ENC_PARALLEL_A, ENC_PARALLEL_B); // For forward/backward movement
Encoder encPerp(ENC_PERP_A, ENC_PERP_B);             // For lateral (side-to-side) movement

// =======================
// Setup BNO055 IMU
// =======================
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);  // Modify the address if necessary

// =======================
// Odometry Parameters
// =======================
const float TICKS_PER_REV      = 1440.0;            // Encoder ticks per wheel revolution
const float WHEEL_DIAMETER     = 1.0;               // Dead wheel diameter in inches (example value)
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI; // Circumference of the dead wheel
const float DIST_PER_TICK      = WHEEL_CIRCUMFERENCE / TICKS_PER_REV;  // Distance per tick (inches per tick)

// =======================
// Global variables for robot pose
// =======================
float robotX = 0.0;  // Global X position (inches)
float robotY = 0.0;  // Global Y position (inches)

// Variables to hold the last encoder counts
long lastParallelTicks = 0;
long lastPerpTicks     = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for serial monitor (optional)

  // Initialize the BNO055 sensor
  if (!bno.begin()) {
    Serial.println("BNO055 not detected! Check wiring or I2C address.");
    while (1); // Halt execution if sensor is not found
  }
  delay(1000);  // Allow sensor time to settle
  bno.setExtCrystalUse(true);

  // Optionally, you can set the sensor's operating mode here.
  // For example: bno.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);

  // Reset the encoder counts to zero
  encParallel.write(0);
  encPerp.write(0);
  lastParallelTicks = encParallel.read();
  lastPerpTicks     = encPerp.read();

  Serial.println("Odometry system initialized.");
}

void loop() {
  // -------------------------
  // 1. Read the encoder values
  // -------------------------
  long currentParallelTicks = encParallel.read();
  long currentPerpTicks     = encPerp.read();

  // Compute the change in ticks since the last iteration
  long deltaParallelTicks = currentParallelTicks - lastParallelTicks;
  long deltaPerpTicks     = currentPerpTicks - lastPerpTicks;

  // Convert the tick differences into distances (in inches)
  float deltaParallelDist = deltaParallelTicks * DIST_PER_TICK;
  float deltaPerpDist     = deltaPerpTicks * DIST_PER_TICK;

  // -------------------------
  // 2. Get the current heading from the IMU
  // -------------------------
  // The BNO055 provides Euler angles (in degrees); we use the z-axis (yaw).
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float headingDegrees = euler.z();
  float theta = radians(headingDegrees); // Convert heading to radians

  // -------------------------
  // 3. Transform local displacements to the global coordinate frame
  // -------------------------
  // The displacements (deltaParallelDist and deltaPerpDist) are measured in the robot's
  // local frame. Rotate these values by the current heading (theta) to obtain global changes.
  float deltaX = deltaParallelDist * cos(theta) - deltaPerpDist * sin(theta);
  float deltaY = deltaParallelDist * sin(theta) + deltaPerpDist * cos(theta);

  // Update the global pose
  robotX += deltaX;
  robotY += deltaY;

  // -------------------------
  // 4. Debug: Print values to the Serial Monitor
  // -------------------------
  Serial.print("Encoders (ticks): Parallel = ");
  Serial.print(currentParallelTicks);
  Serial.print(" | Perp = ");
  Serial.print(currentPerpTicks);
  Serial.print(" | Δ (inches): Parallel = ");
  Serial.print(deltaParallelDist, 4);
  Serial.print(" | Perp = ");
  Serial.print(deltaPerpDist, 4);
  Serial.print(" | Heading = ");
  Serial.print(headingDegrees, 2);
  Serial.print("° | Global Pos = (");
  Serial.print(robotX, 4);
  Serial.print(", ");
  Serial.print(robotY, 4);
  Serial.println(")");

  // -------------------------
  // 5. Save current encoder counts for the next iteration
  // -------------------------
  lastParallelTicks = currentParallelTicks;
  lastPerpTicks     = currentPerpTicks;

  // Loop delay (adjust as necessary)
  delay(100);
}
