#include <Arduino.h>
#include <HX711_ADC.h>
#include <Wire.h>

#include "DFRobot_RGBLCD1602.h"

#if defined(ESP8266) || defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

// ============================================================
// LCD
// ============================================================
DFRobot_RGBLCD1602 lcd(0x6B, 16, 2);

// ============================================================
// PINS
// ============================================================
const int HX711_dout_1 = 34;
const int HX711_sck_1 = 25;
const int HX711_dout_2 = 35;
const int HX711_sck_2 = 26;

const int BUTTON_PIN = 37;

// I2C pins for lcd
const int I2C_SDA = 22;
const int I2C_SCL = 19;

// ============================================================
// HX711 OBJECTS
// ============================================================
HX711_ADC LoadCell_1(HX711_dout_1, HX711_sck_1);
HX711_ADC LoadCell_2(HX711_dout_2, HX711_sck_2);

// ============================================================
// CALIBRATION
// ============================================================
float calibrationValue_1 = 696.0f;
float calibrationValue_2 = 733.0f;

// ============================================================
// SENSOR VARIABLES
// ============================================================
float rawFront = 0.0f;
float rawBack = 0.0f;
float front = 0.0f;
float back = 0.0f;
float total = 0.0f;

// EMA values
float fastEMA = 0.0f;
float slowEMA = 0.0f;
float impactSignal = 0.0f;

bool filtersInitialized = false;
unsigned long verifyStart = 0;

bool matOccupied = false;
bool systemArmed = false;
unsigned long occupiedStart = 0;

const float ENTRY_THRESHOLD = 20.0f;
const float EXIT_THRESHOLD = 5.0f;
const unsigned long ARM_DELAY_MS = 700;

// ============================================================
// TUNING VALUES
// ============================================================
// Start with these, then tune based on Serial Monitor output
const float FAST_ALPHA = 0.95f;
const float SLOW_ALPHA = 0.05f;

const float IMPACT_THRESHOLD = 80.0f;
const float OCCUPIED_THRESHOLD = 15.0f;
const float MIN_TOTAL_FOR_EVENT = 5.0f;

const unsigned long VERIFY_TIME_MS = 250;

// Update timing
const unsigned long LCD_UPDATE_MS = 120;
const unsigned long SERIAL_PRINT_MS = 50;

// ============================================================
// BUTTON DEBOUNCE
// ============================================================
bool lastButtonReading = HIGH;
bool buttonStableState = HIGH;
unsigned long lastButtonChangeMs = 0;
const unsigned long BUTTON_DEBOUNCE_MS = 25;

// ============================================================
// STATES
// ============================================================
enum State { NORMAL, VERIFYING_FALL, ALERT_TRIGGERED };

State currentState = NORMAL;

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================
bool updateData();
void processFallDetection();
void updateDisplay();
void tareFromSerial();
void handleButton();
void resetAlert();
void printDebug();
void printLibraryStats();
void writeLine(uint8_t row, const char* text);
const char* stateName(State s);

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Wire.begin(I2C_SDA, I2C_SCL);

  lcd.init();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Fall Detection");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");

  Serial.println();
  Serial.println("Starting...");

  LoadCell_1.begin();
  LoadCell_2.begin();

  // Reduce averaging for faster response.
  // Library supports 1, 2, 4, 8, 16, 32, 64, 128 samples.
  LoadCell_1.setSamplesInUse(4);
  LoadCell_2.setSamplesInUse(4);

  unsigned long stabilizingtime = 2000;
  bool doTare = true;

  byte loadcell_1_rdy = 0;
  byte loadcell_2_rdy = 0;

  while ((loadcell_1_rdy + loadcell_2_rdy) < 2) {
    if (!loadcell_1_rdy) {
      loadcell_1_rdy = LoadCell_1.startMultiple(stabilizingtime, doTare);
    }
    if (!loadcell_2_rdy) {
      loadcell_2_rdy = LoadCell_2.startMultiple(stabilizingtime, doTare);
    }
  }

  if (LoadCell_1.getTareTimeoutFlag()) {
    Serial.println("Timeout: check HX711 #1 wiring / pins");
  }
  if (LoadCell_2.getTareTimeoutFlag()) {
    Serial.println("Timeout: check HX711 #2 wiring / pins");
  }

  LoadCell_1.setCalFactor(calibrationValue_1);
  LoadCell_2.setCalFactor(calibrationValue_2);

  Serial.println("Startup complete");
  printLibraryStats();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Startup done");
  lcd.setCursor(0, 1);
  lcd.print("Open Serial");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  handleButton();
  tareFromSerial();

  bool newSample = updateData();

  // Only process detection when new sensor data arrives
  if (newSample) {
    processFallDetection();
    printDebug();
  }

  updateDisplay();
}

// ============================================================
// SENSOR UPDATE
// ============================================================
bool updateData() {
  bool new1 = LoadCell_1.update();
  bool new2 = LoadCell_2.update();

  if (!(new1 || new2)) {
    return false;
  }

  rawFront = LoadCell_1.getData();
  rawBack = LoadCell_2.getData();

  // Clamp small negative values after tare
  front = max(0.0f, rawFront);
  back = max(0.0f, rawBack);
  total = front + back;

  return true;
}

// ============================================================
// FALL DETECTION
// ============================================================
void processFallDetection() {
  // Prevent big fake startup spike
  if (!filtersInitialized) {
    fastEMA = total;
    slowEMA = total;
    impactSignal = 0.0f;
    filtersInitialized = true;
    return;
  }

  //   ------ Prevent trigger of getting on the mat ----
  if (!matOccupied) {
    // Empty mat waiting for contact
    if (total > ENTRY_THRESHOLD) {
      matOccupied = true;
      occupiedStart = millis();
      systemArmed = false;

      fastEMA = total;
      slowEMA = total;
      impactSignal = 0.0f;
    }
    return;
  }

  // Disarm Mat if person leaves
  if (total < EXIT_THRESHOLD) {
    matOccupied = false;
    systemArmed = false;

    fastEMA = total;
    slowEMA = total;
    impactSignal = 0.0f;

    if (currentState != ALERT_TRIGGERED) {
      currentState = NORMAL;
    }
    return;
  }
  // Person is on mat, but not armed yet
  if (!systemArmed) {
    if (millis() - occupiedStart >= ARM_DELAY_MS) {
      systemArmed = true;

      // Re-baseline once armed
      fastEMA = total;
      slowEMA = total;
      impactSignal = 0.0f;
    }
    return;  // still ignore impacts during entry phase
  }

  // Exponential moving averages
  fastEMA = FAST_ALPHA * total + (1.0f - FAST_ALPHA) * fastEMA;
  slowEMA = SLOW_ALPHA * total + (1.0f - SLOW_ALPHA) * slowEMA;

  impactSignal = fastEMA - slowEMA;

  switch (currentState) {
    case NORMAL:
      if (impactSignal > IMPACT_THRESHOLD && total > MIN_TOTAL_FOR_EVENT) {
        currentState = VERIFYING_FALL;
        verifyStart = millis();
      }
      break;

    case VERIFYING_FALL:
      if (millis() - verifyStart >= VERIFY_TIME_MS) {
        if (total > OCCUPIED_THRESHOLD) {
          currentState = ALERT_TRIGGERED;
        } else {
          currentState = NORMAL;
        }
      }
      break;

    case ALERT_TRIGGERED:
      // Stay here until button reset
      break;
  }
}

// ============================================================
// SERIAL TARE
// Send 't' in Serial Monitor
// ============================================================
void tareFromSerial() {
  if (Serial.available() > 0) {
    char inByte = Serial.read();

    if (inByte == 't' || inByte == 'T') {
      LoadCell_1.tareNoDelay();
      LoadCell_2.tareNoDelay();
      Serial.println("Started tare...");
    }
  }

  if (LoadCell_1.getTareStatus()) {
    Serial.println("Tare load cell 1 complete");
    filtersInitialized = false;
  }
  if (LoadCell_2.getTareStatus()) {
    Serial.println("Tare load cell 2 complete");
    filtersInitialized = false;
  }
}

// ============================================================
// BUTTON HANDLING
// Press button to clear alert
// ============================================================
void handleButton() {
  bool reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonReading) {
    lastButtonChangeMs = millis();
    lastButtonReading = reading;
  }

  if ((millis() - lastButtonChangeMs) > BUTTON_DEBOUNCE_MS) {
    if (reading != buttonStableState) {
      buttonStableState = reading;

      if (buttonStableState == LOW) {
        if (currentState == ALERT_TRIGGERED) {
          resetAlert();
          Serial.println("Alert cleared by button");
        }
      }
    }
  }
}

void resetAlert() {
  currentState = NORMAL;
  fastEMA = total;
  slowEMA = total;
  impactSignal = 0.0f;
}

// ============================================================
// LCD DISPLAY
// ============================================================
void updateDisplay() {
  static unsigned long lastLcdMs = 0;

  if (millis() - lastLcdMs < LCD_UPDATE_MS) {
    return;
  }
  lastLcdMs = millis();

  char line1[17];
  char line2[17];

  if (currentState == NORMAL) {
    snprintf(line1, sizeof(line1), "N T:%4d I:%3d", (int)total,
             (int)impactSignal);
    snprintf(line2, sizeof(line2), "F:%3d B:%3d", (int)front, (int)back);
  } else if (currentState == VERIFYING_FALL) {
    snprintf(line1, sizeof(line1), "VERIFYING...");
    snprintf(line2, sizeof(line2), "T:%4d I:%3d", (int)total,
             (int)impactSignal);
  } else {
    snprintf(line1, sizeof(line1), "FALL DETECTED");
    if (front > back * 1.2f) {
      snprintf(line2, sizeof(line2), "Front T:%4d", (int)total);
    } else if (back > front * 1.2f) {
      snprintf(line2, sizeof(line2), "Back  T:%4d", (int)total);
    } else {
      snprintf(line2, sizeof(line2), "Center T:%3d", (int)total);
    }
  }

  writeLine(0, line1);
  writeLine(1, line2);
}

void writeLine(uint8_t row, const char* text) {
  char buf[17];
  snprintf(buf, sizeof(buf), "%-16s", text);
  lcd.setCursor(0, row);
  lcd.print(buf);
}

// ============================================================
// DEBUG PRINTING
// ============================================================
void printDebug() {
  static unsigned long lastPrintMs = 0;

  if (millis() - lastPrintMs < SERIAL_PRINT_MS) {
    return;
  }
  lastPrintMs = millis();

  Serial.print("rawF:");
  Serial.print(rawFront, 1);
  Serial.print(" rawB:");
  Serial.print(rawBack, 1);

  Serial.print(" front:");
  Serial.print(front, 1);
  Serial.print(" back:");
  Serial.print(back, 1);

  Serial.print(" total:");
  Serial.print(total, 1);

  Serial.print(" fast:");
  Serial.print(fastEMA, 1);
  Serial.print(" slow:");
  Serial.print(slowEMA, 1);
  Serial.print(" impact:");
  Serial.print(impactSignal, 1);

  Serial.print(" state:");
  Serial.println(stateName(currentState));
}

void printLibraryStats() {
  Serial.println("=== HX711 stats ===");

  Serial.print("LC1 samples: ");
  Serial.println(LoadCell_1.getSamplesInUse());
  Serial.print("LC1 SPS: ");
  Serial.println(LoadCell_1.getSPS());
  Serial.print("LC1 convTime(ms): ");
  Serial.println(LoadCell_1.getConversionTime());
  Serial.print("LC1 settling(ms): ");
  Serial.println(LoadCell_1.getSettlingTime());

  Serial.print("LC2 samples: ");
  Serial.println(LoadCell_2.getSamplesInUse());
  Serial.print("LC2 SPS: ");
  Serial.println(LoadCell_2.getSPS());
  Serial.print("LC2 convTime(ms): ");
  Serial.println(LoadCell_2.getConversionTime());
  Serial.print("LC2 settling(ms): ");
  Serial.println(LoadCell_2.getSettlingTime());

  Serial.println("===================");
}

// ============================================================
// HELPER
// ============================================================
const char* stateName(State s) {
  switch (s) {
    case NORMAL:
      return "NORMAL";
    case VERIFYING_FALL:
      return "VERIFYING_FALL";
    case ALERT_TRIGGERED:
      return "ALERT_TRIGGERED";
    default:
      return "UNKNOWN";
  }
}