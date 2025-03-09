#include <BleCombo.h>
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>

#define UPDATE_PERIOD 4000  // Microseconds (4ms)
#define CALIBRATION_BUTTON 18

MPU6050 mpu;

float pitch = 0, roll = 0;
float pitchOffset = 0, rollOffset = 0;
float prevAccZ = 0;  // To track previous Z acceleration

const float alpha = 0.98;
const float dt = UPDATE_PERIOD / 1000000.0;  // Convert to seconds

//-----------------------------//
//       USER PARAMETERS       //
const float angleDeadzone = 10.0;  // Higher deadzone for keyboard controls
const float tiltThreshold = 20.0;  // Angle threshold to trigger key press
const float upwardThreshold = 300.0;  // Threshold for detecting upward movement
const float downwardThreshold = 500.0;  // Increased threshold for downward movement
const float upwardDeadzone = 1000.0;   // Deadzone to prevent minor movements from triggering
const float downwardDeadzone = 1500.0;   // Increased deadzone for downward movement
const int spaceCooldown = 600;  // Cooldown in milliseconds to prevent multiple triggers
const int shiftCooldown = 600;  // Cooldown in milliseconds to prevent multiple triggers
const int spaceShiftDelay = 1000;  // 0.6 seconds delay between spacebar and shift
//-----------------------------//

unsigned long lastUpdate = 0;
unsigned long lastSpaceTime = 0;  // To track when the last spacebar was pressed
unsigned long lastShiftTime = 0;  // To track when the last shift was pressed
bool keyW = false, keyA = false, keyS = false, keyD = false, keySpace = false, keyShift = false;
bool shiftLocked = false;  // Track if shift is locked

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
      shiftLocked = false;  // Reset shift lock on calibration
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
      // Detect upward movement (negative z acceleration)
      // When moving up, az becomes more negative
      if (az < -upwardThreshold && abs(az) > (prevAccZ + upwardDeadzone) && (millis() - lastSpaceTime > spaceCooldown)) {
        Keyboard.press(' ');  // Press spacebar
        keySpace = true;
        lastSpaceTime = millis();
        Serial.println("SPACEBAR pressed (upward movement detected)");
        
        // If shift is locked, unlock it and release the shift key
        if (shiftLocked) {
          Keyboard.release(KEY_LEFT_SHIFT);
          keyShift = false;
          shiftLocked = false;
          Serial.println("SHIFT released (due to spacebar)");
        }
        
        // Set a timer to release spacebar after a short delay
        delay(50);  // Brief press duration
        Keyboard.release(' ');
        keySpace = false;
        Serial.println("SPACEBAR released");
      }
      
      // Detect downward movement (positive z acceleration)
      // When moving down, az becomes more positive
      if (az > downwardThreshold && abs(az) > (prevAccZ + downwardDeadzone) && 
          (millis() - lastShiftTime > shiftCooldown) && 
          (millis() - lastSpaceTime > spaceShiftDelay)) {
        // Toggle shift lock state
        if (!shiftLocked) {
          Keyboard.press(KEY_LEFT_SHIFT);  // Press shift key
          keyShift = true;
          shiftLocked = true;
          lastShiftTime = millis();
          Serial.println("SHIFT pressed and locked (downward movement detected)");
        }
      }
      
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
      shiftLocked = false;  // Reset shift lock on disconnect
    }
    
    // Update previous acceleration value
    prevAccZ = az;
    
    // Debug output
    Serial.print("Pitch: ");
    Serial.print(pitch);
    Serial.print(" Roll: ");
    Serial.print(roll);
    Serial.print(" AccZ: ");
    Serial.print(az);
    Serial.print(" Shift locked: ");
    Serial.println(shiftLocked);
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

  // Initialize prevAccZ after calibration
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  prevAccZ = az;

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
  if (keySpace) {
    Keyboard.release(' ');
    keySpace = false;
  }
  if (keyShift) {
    Keyboard.release(KEY_LEFT_SHIFT);
    keyShift = false;
  }
}
