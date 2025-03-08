#include <BleCombo.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

#define UPDATE_PERIOD 4000  // Microseconds (4ms)
#define CALIBRATION_BUTTON 18

MPU6050 mpu;

float pitch = 0, roll = 0;
float pitchOffset = 0, rollOffset = 0;

const float alpha = 0.98;
const float dt = UPDATE_PERIOD / 1000000.0;  // Convert to seconds

//-----------------------------//

//       USER PARAMETERS       //

const float angleDeadzone = 10.0;  // Higher deadzone for keyboard controls
const float tiltThreshold = 20.0;  // Angle threshold to trigger key press

//-----------------------------//

unsigned long lastUpdate = 0;
bool keyW = false, keyA = false, keyS = false, keyD = false;

// Function prototypes
void calibrateMPU6050();
void releaseAllKeys();

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // Faster I2C
  pinMode(CALIBRATION_BUTTON, INPUT_PULLUP);

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  Serial.println("MPU6050 connected!");
  
  // Initialize BLE Keyboard and Mouse
  Keyboard.begin();
  Mouse.begin();
  
  Serial.println("BLE Keyboard started!");
  
  calibrateMPU6050();
}

void loop() {
  if (micros() - lastUpdate >= UPDATE_PERIOD) {
    lastUpdate = micros();

    if (digitalRead(CALIBRATION_BUTTON) == LOW) {
      calibrateMPU6050();
      // Release all keys when calibrating
      releaseAllKeys();
    }

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);

    float pitchAcc = atan2(ay, az) * 180.0 / PI - pitchOffset;
    float rollAcc  = atan2(-ax, az) * 180.0 / PI - rollOffset;

    float gyroX = gx / 500.0;
    float gyroY = gy / 500.0;

    pitch = alpha * (pitch + gyroX * dt) + (1 - alpha) * pitchAcc;
    roll  = alpha * (roll  + gyroY * dt) + (1 - alpha) * rollAcc;

    // Handle keyboard inputs if connected
    if(Keyboard.isConnected()) {
      // Forward (W key)
      if (pitch < -tiltThreshold && !keyW) {
        Keyboard.press('w');
        keyW = true;
        Serial.println("W pressed");
      } else if (pitch >= -tiltThreshold && keyW) {
        Keyboard.release('w');
        keyW = false;
        Serial.println("W released");
      }
      
      // Backward (S key)
      if (pitch > tiltThreshold && !keyS) {
        Keyboard.press('s');
        keyS = true;
        Serial.println("S pressed");
      } else if (pitch <= tiltThreshold && keyS) {
        Keyboard.release('s');
        keyS = false;
        Serial.println("S released");
      }
      
      // Left (A key)
      if (roll < -tiltThreshold && !keyA) {
        Keyboard.press('a');
        keyA = true;
        Serial.println("A pressed");
      } else if (roll >= -tiltThreshold && keyA) {
        Keyboard.release('a');
        keyA = false;
        Serial.println("A released");
      }
      
      // Right (D key)
      if (roll > tiltThreshold && !keyD) {
        Keyboard.press('d');
        keyD = true;
        Serial.println("D pressed");
      } else if (roll <= tiltThreshold && keyD) {
        Keyboard.release('d');
        keyD = false;
        Serial.println("D released");
      }
    } else {
      // If disconnected, make sure all keys are released
      releaseAllKeys();
    }
    
    // Debug output
    Serial.print("Pitch: ");
    Serial.print(pitch);
    Serial.print(" Roll: ");
    Serial.println(roll);
  }
}

void calibrateMPU6050() {
  float pitchSum = 0, rollSum = 0;
  int samples = 100;

  Serial.println("Calibrating... Keep device still");
  
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);

    pitchSum = pitchSum * 0.9 + (atan2(ay, az) * 180.0 / PI) * 0.1;
    rollSum = rollSum * 0.9 + (atan2(-ax, az) * 180.0 / PI) * 0.1;

    delayMicroseconds(10000);  // Faster calibration (~1 sec)
  }

  pitchOffset = pitchSum;
  rollOffset = rollSum;

  Serial.println("Calibration complete.");
  Serial.print("Pitch offset: "); Serial.println(pitchOffset);
  Serial.print("Roll offset: "); Serial.println(rollOffset);
}

void releaseAllKeys() {
  if (keyW) {
    Keyboard.release('w');
    keyW = false;
  }
  if (keyA) {
    Keyboard.release('a');
    keyA = false;
  }
  if (keyS) {
    Keyboard.release('s');
    keyS = false;
  }
  if (keyD) {
    Keyboard.release('d');
    keyD = false;
  }
}
