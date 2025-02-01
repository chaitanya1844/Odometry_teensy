#include <Encoder.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29,&Wire2); // 0x28 is the default I2C address

Encoder encoder1(3,2);// Pins for encoder 1
Encoder encoder2(1,0);// Pins for encoder 2

volatile int currentTick1 = 0; // Position for encoder 1
volatile int currentTick2 = 0; // Position for encoder 2

volatile int oldTick1 = 0; 
volatile int oldTick2 = 0; 

volatile int deltaTick1 = 0; 
volatile int deltaTick2 = 0; 

volatile float globalX = 0; 
volatile float globalY = 0; 

volatile float speedX=0;
volatile float speedY=0;

volatile float deltaX = 0; 
volatile float deltaY = 0;

volatile float globalTheta=0;
volatile float oldTheta=0;
volatile float deltaTheta=0;
volatile float virtual_global_Theta=0;

const float wheelDiameter = 5.8; 
const int encoderResolution = 2400;
const float C= (PI * wheelDiameter / encoderResolution );
const int L= 51;
const int B= 49;

void setup() {
  Wire2.begin();
  Serial.begin(9600);
  while (!Serial) delay(10); // Wait for Serial Monitor

  // Initialize BNO055
  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check connections!");
    while (1);
  }
  Serial.println("BNO055 detected!");

  // Optional: Configure to NDOF mode (for fused orientation data)
  bno.setExtCrystalUse(true);
}

void loop() {
  odometry();
  delay(10);
  Serial.print(globalX);
  Serial.print(",");
  Serial.println(globalY);
  Serial.print(",");
  Serial.print(globalTheta);
  Serial.print(",");
  Serial.println(globalY);
  Serial.print(",");
  Serial.print(globalX);
  Serial.print(",");
  Serial.println(globalY);


}

void odometry() {
  sensors_event_t event;
  bno.getEvent(&event);

  // Save old tick values
  oldTick1 = currentTick1;
  oldTick2 = currentTick2;

  // Read encoder positions
  currentTick1 = encoder1.read();
  currentTick2 = encoder2.read();

  // Calculate change in encoder ticks
  deltaTick1 = currentTick1 - oldTick1;
  deltaTick2 = currentTick2 - oldTick2;

  // Save the previous virtual global theta
  oldTheta = virtual_global_Theta;

  // Get the new global theta from the sensor
  globalTheta = event.orientation.x; // BNO055 provides the heading angle

  // Calculate the change in theta from the IMU
  deltaTheta = globalTheta - oldTheta;

  // Handle wrapping of deltaTheta within -180° to 180°
  if (deltaTheta > 180) deltaTheta -= 360;
  if (deltaTheta < -180) deltaTheta += 360;

  // Update virtual global theta and normalize it
  virtual_global_Theta += deltaTheta;

  // Convert the virtual global theta to radians
  float ThetaRad = virtual_global_Theta * PI / 180;

  // Compute local displacements
  deltaX = (deltaTick1 * C) - (L * deltaTheta * PI / 180);
  deltaY = (deltaTick2 * C) - (B * deltaTheta * PI / 180);

  // Convert local displacements to global coordinates
  double cos_theta = cos(ThetaRad);
  double sin_theta = sin(ThetaRad);

  double delta_x_global = cos_theta * deltaX - sin_theta * deltaY;
  double delta_y_global = sin_theta * deltaX + cos_theta * deltaY;

  // Update global positions
  globalX += delta_x_global;
  globalY += delta_y_global;

}