#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <BleMouse.h>
#include <EEPROM.h>

// Define EEPROM size for storing calibration data
#define EEPROM_SIZE 8  // We need 8 bytes for two float values

// Create BNO055 instance
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Create BLE Mouse with custom name
BleMouse bleMouse("ESP32-BNO055-Mouse", "ESP32", 100);

// Define calibration button pin
const int BUTTON_PIN = 18;

// Variables for sensor readings
float pitch, roll;
float pitchOffset = 0, rollOffset = 0;
bool isCalibrated = false;

// Button state variables
int lastButtonState = HIGH;
int currentButtonState;

// Deadzone and sensitivity settings
const float DEADZONE = 5.0;       // Ignore movements smaller than this (in degrees)
const float SENSITIVITY = 0.5;    // Higher values = more sensitive mouse movement

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("ESP32 BLE Mouse with BNO055 - Continuous Reading");
  
  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Initialize button pin with internal pull-up
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Initialize the BNO055 sensor
  if (!bno.begin()) {
    Serial.println("No BNO055 detected. Check wiring!");
    while (1) {
      delay(100);
    }
  }
  
  // Use the external crystal for better accuracy
  bno.setExtCrystalUse(true);
  
  // Load calibration offsets from EEPROM
  loadOffsetsFromEEPROM();
  
  // Initialize BLE Mouse
  bleMouse.begin();
  Serial.println("BLE Mouse initialized. Waiting for connection...");
  Serial.println("Press button on pin 18 to calibrate orientation");
}

void loop() {
  // Read the current button state
  currentButtonState = digitalRead(BUTTON_PIN);
  
  // Check if button is pressed (LOW when using INPUT_PULLUP)
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    // Button was just pressed
    Serial.println("Calibration button pressed!");
    calibrateSensor();
  }
  
  // Save the current button state for next comparison
  lastButtonState = currentButtonState;
  
  // Get orientation data from BNO055
  sensors_event_t event;
  bno.getEvent(&event);
  
  // Extract pitch and roll values and apply offsets if calibrated
  pitch = event.orientation.y - pitchOffset;
  roll = event.orientation.z - rollOffset;
  
  // Process mouse movement if BLE is connected
  if (bleMouse.isConnected()) {
    // Calculate mouse movement based on orientation
    int xMove = 0;
    int yMove = 0;
    
    // Apply deadzone to roll (X movement)
    if (abs(roll) > DEADZONE) {
      // Only use the portion of movement outside the deadzone
      float adjustedRoll = roll > 0 ? roll - DEADZONE : roll + DEADZONE;
      xMove = adjustedRoll * SENSITIVITY;
    }
    
    // Apply deadzone to pitch (Y movement)
    if (abs(pitch) > DEADZONE) {
      // Only use the portion of movement outside the deadzone
      float adjustedPitch = pitch > 0 ? pitch - DEADZONE : pitch + DEADZONE;
      // Invert Y axis so tilting forward moves mouse up
      yMove = -adjustedPitch * SENSITIVITY;
    }
    
    // Move the mouse only if there's actual movement to make
    if (xMove != 0 || yMove != 0) {
      bleMouse.move(xMove, yMove);
      
      // Print values for debugging
      Serial.print("Pitch: ");
      Serial.print(pitch);
      Serial.print("\tRoll: ");
      Serial.print(roll);
      Serial.print("\tMouse: X=");
      Serial.print(xMove);
      Serial.print(" Y=");
      Serial.println(yMove);
    }
  }
  
  // Small delay to prevent flooding
  delay(10);  // 10ms delay for ~100Hz update rate
}

void calibrateSensor() {
  // Get current orientation
  sensors_event_t event;
  bno.getEvent(&event);
  
  // Store current values as offsets
  pitchOffset = event.orientation.y;
  rollOffset = event.orientation.z;
  isCalibrated = true;
  
  Serial.println("Calibration complete!");
  Serial.print("Pitch offset: ");
  Serial.print(pitchOffset);
  Serial.print(", Roll offset: ");
  Serial.println(rollOffset);
  
  // Save offsets to EEPROM
  saveOffsetsToEEPROM();
  
  // Briefly pause to avoid immediate movement after calibration
  delay(500);
}

void saveOffsetsToEEPROM() {
  // Write pitchOffset to addresses 0-3
  EEPROM.put(0, pitchOffset);
  // Write rollOffset to addresses 4-7
  EEPROM.put(4, rollOffset);
  // Commit the changes to flash
  EEPROM.commit();
  Serial.println("Calibration offsets saved to EEPROM");
}

void loadOffsetsFromEEPROM() {
  // Read pitchOffset from addresses 0-3
  EEPROM.get(0, pitchOffset);
  // Read rollOffset from addresses 4-7
  EEPROM.get(4, rollOffset);
  
  // Check if the values are valid (not NaN)
  if (isnan(pitchOffset) || isnan(rollOffset)) {
    pitchOffset = 0;
    rollOffset = 0;
    isCalibrated = false;
    Serial.println("No valid calibration found in EEPROM");
  } else {
    isCalibrated = true;
    Serial.print("Loaded calibration from EEPROM - Pitch offset: ");
    Serial.print(pitchOffset);
    Serial.print(", Roll offset: ");
    Serial.println(rollOffset);
  }
}
