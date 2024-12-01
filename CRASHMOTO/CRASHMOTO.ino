#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <math.h>

#define GPS_RX_PIN 8
#define GPS_TX_PIN 9
#define BUZZER_PIN 10
#define LED_PIN 13
#define ACCEL_X_PIN A0
#define ACCEL_Y_PIN A1
#define ACCEL_Z_PIN A2
#define GSM_TX_PIN 6
#define GSM_RX_PIN 7
#define BUTTON_WEATHER_PIN 2
#define BUTTON_ROAD_PIN 3
#define TRIG_PIN 4        // Trigger pin for HC-SR04
#define ECHO_PIN 5        // Echo pin for HC-SR04

TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
SoftwareSerial gsmSerial(GSM_TX_PIN, GSM_RX_PIN);
LiquidCrystal_I2C lcd(0x27, 16, 2);

int weatherCondition = 0; // 0: Sunny, 1: Rainy
int roadCondition = 0;    // 0: Asphalt, 1: Rough
float bumpThreshold = 1000;
float speed = 0.0;
float ultrasonicDistance = 0.0;  // Distance from HC-SR04

bool weatherButtonPressed = false;
bool roadButtonPressed = false;
bool isSettingMode = false; // Toggle for display mode

unsigned long lastDisplayUpdate = 0;  // Variable to store the time of the last display update
const unsigned long displayUpdateInterval = 1000; // Interval for display update in milliseconds (1 second)

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_WEATHER_PIN, INPUT_PULLUP);
  pinMode(BUTTON_ROAD_PIN, INPUT_PULLUP);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.begin(9600);
  gpsSerial.begin(9600);
  gsmSerial.begin(9600);

  lcd.init();
  lcd.backlight();
  updateDisplays();

  delay(10000);
  Serial.println("Initializing GSM module...");
  gsmSerial.println("AT");
  delay(1000);
  while (gsmSerial.available()) {
    Serial.write(gsmSerial.read());
  }

  updateBumpThreshold();
}

void loop() {
  // Handle button presses for settings
  if (digitalRead(BUTTON_WEATHER_PIN) == LOW && !weatherButtonPressed) {
    isSettingMode = true;
    weatherCondition = (weatherCondition + 1) % 2;
    updateDisplays();
    updateBumpThreshold();
    delay(2000);  // Delay for 2 seconds before returning to main display
    isSettingMode = false; // Return to main display
    updateDisplays();
    weatherButtonPressed = true;
  }
  if (digitalRead(BUTTON_WEATHER_PIN) == HIGH) {
    weatherButtonPressed = false;
  }

  if (digitalRead(BUTTON_ROAD_PIN) == LOW && !roadButtonPressed) {
    isSettingMode = true;
    roadCondition = (roadCondition + 1) % 2;
    updateDisplays();
    updateBumpThreshold();
    delay(2000);  // Delay for 2 seconds before returning to main display
    isSettingMode = false; // Return to main display
    updateDisplays();
    roadButtonPressed = true;
  }
  if (digitalRead(BUTTON_ROAD_PIN) == HIGH) {
    roadButtonPressed = false;
  }

  if (isBumped()) {
    triggerAlarm();
    sendLocationSMS();
    Serial.println("Alarm triggered!");
    delay(500);
  }

  // Measure distance continuously
  ultrasonicDistance = measureUltrasonicDistance();

  // Trigger continuous high-pitch sound if distance is less than 100 cm
  if (ultrasonicDistance < 100.0) {
    tone(BUZZER_PIN, 2000);  // 2000 Hz frequency for high-pitch sound
  } else {
    noTone(BUZZER_PIN);  // Turn off buzzer if distance is 100 cm or more
  }

  // Update display every 1 second
  if (millis() - lastDisplayUpdate >= displayUpdateInterval) {
    lastDisplayUpdate = millis();  // Update the last time the display was updated
    updateDisplays();  // Update the LCD with the most recent distance
  }
}

float measureUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 2; // Calculate distance in cm
  return distance;
}

void updateDisplays() {
  lcd.clear();
  if (isSettingMode) {
    updateWeatherDisplay();
    updateRoadDisplay();
  } else {
    updateSpeedAndDistanceDisplay();
  }
}

void updateSpeedAndDistanceDisplay() {
  lcd.setCursor(0, 0);
  lcd.print("Speed: ");
  lcd.print(speed);
  lcd.print(" km/h");

  lcd.setCursor(0, 1);
  lcd.print("Distance: ");
  lcd.print(ultrasonicDistance, 1);
  lcd.print(" cm");
}

void updateWeatherDisplay() {
  lcd.setCursor(0, 0);
  if (weatherCondition == 0) {
    lcd.print("Weather: Sunny ");
  } else {
    lcd.print("Weather: Rainy ");
  }
}

void updateRoadDisplay() {
  lcd.setCursor(0, 1);
  if (roadCondition == 0) {
    lcd.print("Road: Asphalt ");
  } else {
    lcd.print("Road: Rough   ");
  }
}

void updateBumpThreshold() {
  if (weatherCondition == 0 && roadCondition == 0) {
    bumpThreshold = 1000;
  } else if (weatherCondition == 0 && roadCondition == 1) {
    bumpThreshold = 1300;
  } else if (weatherCondition == 1 && roadCondition == 0) {
    bumpThreshold = 900;
  } else if (weatherCondition == 1 && roadCondition == 1) {
    bumpThreshold = 800;
  }

  Serial.print("Updated bump threshold: ");
  Serial.println(bumpThreshold);
}

bool isBumped() {
  int x = analogRead(ACCEL_X_PIN);
  int y = analogRead(ACCEL_Y_PIN);
  int z = analogRead(ACCEL_Z_PIN);
  float magnitude = sqrt(sq(x - 512) + sq(y - 512) + sq(z - 512));

  return magnitude > bumpThreshold;
}

void triggerAlarm() {
  digitalWrite(BUZZER_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
}

void sendLocationSMS() {
  bool locationFound = false;
  unsigned long start = millis();
  while (millis() - start < 5000) {
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
    if (gps.location.isUpdated()) {
      locationFound = true;
      break;
    }
  }

  char message[160];
  if (locationFound) {
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();
    sprintf(message, "CRASH DETECTED! Location: https://maps.google.com/?q=%f,%f", latitude, longitude);
  } else {
    sprintf(message, "CRASH DETECTED! Location: Unable to determine GPS location.");
  }

  gsmSerial.print("AT+CMGF=1\r");
  delay(100);
  gsmSerial.print("AT+CMGS=\"+639091165627\"\r");
  delay(100);
  gsmSerial.print(message);
  delay(100);
  gsmSerial.write(26);
  delay(1000);

  Serial.println("SMS sending...");
  while (gsmSerial.available()) {
    Serial.write(gsmSerial.read());
  }
}
