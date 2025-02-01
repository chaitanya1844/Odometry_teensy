#include <Encoder.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Pin definitions for encoders
#define ENC_X_A 2    // Encoder for X-axis wheel
#define ENC_X_B 3
#define ENC_Y_A 4    // Encoder for Y-axis wheel
#define ENC_Y_B 5

// Dead wheel specifications
#define WHEEL_RADIUS 0.03   // Radius of the wheel in meters (e.g., 3 cm)
#define TICKS_PER_REV 2048  // Encoder ticks per revolution

// IMU initialization (using hardware UART on Serial2)
Adafruit_BNO055 bno = Adafruit_BNO055(55, &Serial2);

// Encoders initialization
Encoder encoderX(ENC_X_A, ENC_X_B);
Encoder encoderY(ENC_Y_A, ENC_Y_B);

// Global position variables
float x_global = 0;        // X-coordinate in global frame
float y_global = 0;        // Y-coordinate in global frame
float theta_global = 0;    // Orientation (angle) in radians

// Function to convert encoder ticks to distance in meters
float ticksToDistance(long ticks) {
  return (2 * M_PI * WHEEL_RADIUS * ticks) / TICKS_PER_REV;
}

// IMU initialization
void initializeIMU() {
  if (!bno.begin() {
    Serial.println("No BNO055 detected over UART. Check wiring!");
    while (1);
  }
  bno.setExtCrystalUse(true); // Use external crystal for better accuracy
}

// Function to update position and orientation
void updatePosition() {
  static long lastTicksX = 0; // Previous tick count for X
  static long lastTicksY = 0; // Previous tick count for Y

  // Read current tick counts
  long ticksX = encoderX.read();
  long ticksY = encoderY.read();

  // Calculate displacement in X and Y
  float deltaX = ticksToDistance(ticksX - lastTicksX);
  float deltaY = ticksToDistance(ticksY - lastTicksY);

  // Update last tick counts
  lastTicksX = ticksX;
  lastTicksY = ticksY;

  // Get orientation from IMU
  sensors_event_t event;
  bno.getEvent(&event);
  theta_global = event.orientation.x * M_PI / 180.0; // Convert degrees to radians

  // Update global position in the fixed frame
  x_global += deltaX * cos(theta_global) - deltaY * sin(theta_global);
  y_global += deltaX * sin(theta_global) + deltaY * cos(theta_global);
}

// Setup function
void setup() {
  Serial.begin(115200); // Serial monitor
  Serial2.begin(115200); // Hardware UART for BNO055

  // Initialize encoders
  encoderX.write(0);
  encoderY.write(0);

  // Initialize IMU
  initializeIMU();

  Serial.println("System initialized!");
}

// Loop function
void loop() {
  // Update position and orientation
  updatePosition();

  // Print global position and orientation
  Serial.print("X: ");
  Serial.print(x_global, 4); // Print X with 4 decimal places
  Serial.print(" m, Y: ");
  Serial.print(y_global, 4); // Print Y with 4 decimal places
  Serial.print(" m, Theta: ");
  Serial.print(theta_global * 180.0 / M_PI, 2); // Print theta in degrees
  Serial.println(" deg");

  delay(100); // Update every 100 ms
}
