#include <Encoder.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Encoder Pins
#define ENC_X_A 2    // X-axis (forward motion) encoder
#define ENC_X_B 3
#define ENC_Y_A 4    // Y-axis (lateral motion) encoder
#define ENC_Y_B 5

// Dead wheel specifications
#define WHEEL_RADIUS 0.03   // Wheel radius in meters
#define TICKS_PER_REV 2048  // Encoder ticks per revolution
#define CM_PER_TICK (2 * M_PI * WHEEL_RADIUS / TICKS_PER_REV)

// IMU initialization (UART on Serial2 for Teensy 4.1)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29,&Wire2);

// Encoders initialization
Encoder encoderX(ENC_X_A, ENC_X_B);  // Forward motion encoder
Encoder encoderY(ENC_Y_A, ENC_Y_B);  // Lateral motion encoder

// Global position variables
float x_global = 0;        // X-coordinate in global frame
float y_global = 0;        // Y-coordinate in global frame
float theta_global = 0;    // Orientation (angle) in radians

// Previous encoder values
long oldX = 0, oldY = 0;

// IMU initialization
void initializeIMU() {
    if (!bno.begin()) {
        Serial.println("No BNO055 detected over UART. Check wiring!");
        while (1);
    }
    bno.setExtCrystalUse(true); // Use external crystal for better accuracy
}

// Function to update position
void updateOdometry() {
    // Read IMU for current heading
    sensors_event_t event;
    bno.getEvent(&event);
    float newTheta = event.orientation.x * M_PI / 180.0; // Convert degrees to radians

    // Read current encoder values
    long currentX = encoderX.read();
    long currentY = encoderY.read();

    // Calculate encoder differences
    long dx_ticks = currentX - oldX;
    long dy_ticks = currentY - oldY;

    // Convert encoder ticks to real-world distances
    float dx = CM_PER_TICK * dx_ticks; 
    float dy = CM_PER_TICK * dy_ticks;

    // Save previous values for next loop
    oldX = currentX;
    oldY = currentY;

    // Apply rotation transformation to global frame
    x_global += dx * cos(theta_global) - dy * sin(theta_global);
    y_global += dx * sin(theta_global) + dy * cos(theta_global);

    // Update global orientation
    theta_global = newTheta;
}

// Setup function
void setup() {
    Serial.begin(115200);   // Serial monitor
    Serial2.begin(115200);  // UART for BNO055

    // Initialize encoders
    encoderX.write(0);
    encoderY.write(0);

    // Initialize IMU
    initializeIMU();

    Serial.println("System initialized!");
}

// Loop function
void loop() {
    // Update odometry
    updateOdometry();

    // Print position and orientation
    Serial.print("X: ");
    Serial.print(x_global, 4);
    Serial.print(" m, Y: ");
    Serial.print(y_global, 4);
    Serial.print(" m, Theta: ");
    Serial.print(theta_global * 180.0 / M_PI, 2); // Convert radians to degrees
    Serial.println(" deg");

    delay(100); // Update every 100 ms
}
