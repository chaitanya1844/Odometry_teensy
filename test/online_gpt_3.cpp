#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Encoder pins
const int encoderXPinA = 2;  // X-axis encoder (parallel to the robot)
const int encoderXPinB = 3;
const int encoderYPinA = 4;  // Y-axis encoder (perpendicular to the robot)
const int encoderYPinB = 5;

// Create Encoder objects
Encoder encoderX(encoderXPinA, encoderXPinB);
Encoder encoderY(encoderYPinA, encoderYPinB);

// IMU object for heading tracking
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Wheel parameters
const double wheelDiameter = 2.0; // inches
const double countsPerRevolution = 360.0; // Encoder resolution
const double wheelCircumference = wheelDiameter * PI;
const double distancePerCount = wheelCircumference / countsPerRevolution; // Distance traveled per encoder count
const double trackWidth = 5.0; // Distance between the robot center and the Y-axis dead wheel

// Robot position tracking
double posX = 0.0; // Global X position
double posY = 0.0; // Global Y position
double heading = 0.0; // Robot heading in radians

// Previous state tracking
long prevEncoderX = 0;
long prevEncoderY = 0;
double prevHeading = 0.0;

void setup() {
    Serial.begin(9600);

    // Initialize the IMU
    if (!bno.begin()) {
        Serial.println("No BNO055 detected. Check wiring!");
        while (1); // Halt execution if IMU fails to initialize
    }
    bno.setExtCrystalUse(true); // Use external crystal for better accuracy
}

void loop() {
    // Read encoder values
    long currentEncoderX = encoderX.read(); // Get current X encoder counts
    long currentEncoderY = encoderY.read(); // Get current Y encoder counts

    // Read IMU orientation
    sensors_event_t event;
    bno.getEvent(&event);
    double newHeading = radians(event.orientation.z); // Convert heading from degrees to radians

    // Compute encoder differences (deltas)
    long deltaXCounts = currentEncoderX - prevEncoderX;
    long deltaYCounts = currentEncoderY - prevEncoderY;
    double deltaHeading = newHeading - prevHeading; // Change in heading

    // Convert encoder counts to real-world distances
    double deltaX = deltaXCounts * distancePerCount;
    double deltaY = deltaYCounts * distancePerCount;

    // If the robot is rotating in place, adjust movement calculation
    if (abs(deltaX) < 0.001 && abs(deltaY) < 0.001) {
        // Estimate heading change using encoders instead of IMU to reduce drift
        deltaHeading = (deltaXCounts - deltaYCounts) * (distancePerCount / trackWidth);
    }

    // Transform local motion (relative to the robot) into global coordinates
    double globalDeltaX = deltaX * cos(heading) - deltaY * sin(heading);
    double globalDeltaY = deltaX * sin(heading) + deltaY * cos(heading);

    // Update the global position
    posX += globalDeltaX;
    posY += globalDeltaY;
    heading = newHeading; // Update heading with IMU value

    // Store previous values for next loop iteration
    prevEncoderX = currentEncoderX;
    prevEncoderY = currentEncoderY;
    prevHeading = newHeading;

    // Output position data for debugging
    Serial.print("X: "); Serial.print(posX);
    Serial.print(" Y: "); Serial.print(posY);
    Serial.print(" Heading: "); Serial.println(degrees(heading));

    delay(50); // Delay for loop timing and stability
}
