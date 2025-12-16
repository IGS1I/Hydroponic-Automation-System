/*************************************************
 * ESP32 + Blynk + HC-SR04 + pH + EC + DS18B20
 * + Circulation Pump + pH Dosing Pumps + Nutrient Pump
 *
 * - Water Level (HC-SR04) read:
 *      • when manual button (V1) is pressed
 *      • at 7:00 AM and 7:00 PM
 * - Temp, pH, EC probes always in nutrient solution
 * - Temperature:
 *      • every 2 hours
 *      • plus at each full measurement
 * - Circulation pump:
 *      • 5 seconds ON, 5 minutes OFF
 *      • controlled by emergency enable/disable button on V2
 *      • when enabled, pump starts immediately
 * - pH dosing:
 *      • pH UP pump on GPIO 25
 *      • pH DOWN pump on GPIO 27
 *      • several pH readings averaged, then dose based on average
 *      • runs on manual measure and scheduled 7:00 AM / 7:00 PM
 * - Nutrient dosing:
 *      • Nutrient pump on GPIO 32
 *      • doses when EC is below EC_MIN_mScm
 *      • one dose per measurement event
 * - EC calibration:
 *      • Single-point calibration with known EC solution
 *      • Serial command 'e' to calibrate, 'E' to print
 *
 * Pins:
 *   DS18B20: GPIO 22  (with 4.7k pull-up to 3.3V)
 *   pH:      GPIO 34
 *   EC:      GPIO 35
 *   HC-SR04: TRIG 18, ECHO 5 (with 10k / 20k divider!)
 *   Pump relay (circulation): GPIO 26
 *   pH UP relay:   GPIO 25
 *   pH DOWN relay: GPIO 27
 *   Nutrient relay: GPIO 32
 *
 * Blynk Virtual Pins:
 *   V1  - Manual Measure button
 *   V2  - Circulation enable (emergency shutoff)
 *   V5  - Status text
 *   V6  - Temperature (°C)
 *   V7  - EC (mS/cm @25°C, calibrated)
 *   V8  - TDS (ppm)
 *   V9  - pH
 *   V10 - Reservoir level (% full)
 *************************************************/

// ===== Blynk Template =====
#define BLYNK_TEMPLATE_ID   "TMPL2MY17Ut_-"
#define BLYNK_TEMPLATE_NAME "Water Level"
#define BLYNK_AUTH_TOKEN    "9Bg7YlfmHiVTPydFHQ_06LZLN4IZ19Oq"

// ===== Includes =====
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>
#include <time.h>

// ===== WiFi =====
#define WIFI_SSID  "******************" // Your WiFi SSID
#define WIFI_PASS  "**************" // Your router's password

// ===== Time / Scheduling =====
#define TZ_STRING "EST5EDT,M3.2.0/2,M11.1.0/2"  // America/New_York
const int SCHED_HOUR_AM = 7;
const int SCHED_HOUR_PM = 19;
const int SCHED_MINUTE   = 0;

// Track last scheduled run
int lastRunY = -1, lastRunM = -1, lastRunD = -1;
bool lastRunWasAM = false;

// ===== HC-SR04 Water Level =====
#define ULTRASONIC_TRIG_PIN 18
#define ULTRASONIC_ECHO_PIN 5

// Total height from sensor tip to bottom of reservoir (cm)
const float RESERVOIR_HEIGHT_CM = 18.0f;

// Alert when ≤ 25% full
const float LOW_THRESHOLD_PCT   = 25.0f;

// ===== DS18B20 Temperature =====
#define ONE_WIRE_BUS 22
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);
float temperatureC = NAN;

// ===== pH Sensor =====
#define PH_PIN 34
Preferences prefs;
float slope_m = NAN, intercept_b = NAN;  // pH = m*VmV + b
float V4mV = NAN, V9mV = NAN;

// ===== EC Sensor =====
#define EC_PIN 35
#define ADC_SAMPLES 15
#define EC_SLOPE  1.00f      // base slope in volts→EC domain (pre-cal)
#define EC_OFFSET 0.00f      // base offset in volts
#define EC_TEMP_COEFF 0.02f
#define REF_TEMP 25.0f
#define TDS_FACTOR 500

// EC calibration (final scaling)
float ecCalK = 1.0f;        // multiplicative calibration factor
float ecCalB = 0.0f;        // additive offset (kept at 0 in single-point)
const float EC_CAL_REF_mScm = 1.413f; // calibration solution EC (change if needed)

float ec_mScm = NAN;
float tds_ppm = NAN;
float pH_value = NAN;

// ===== Circulation Pump (Relay) =====
#define RELAY_PUMP_CIRC 26

const uint32_t CIRC_ON_MS  = 5000UL;               // 5 s ON
const uint32_t CIRC_OFF_MS = 5UL * 60UL * 1000UL;  // 5 min OFF

bool circEnabled = true;     // controlled by V2 (emergency)
bool circOn      = false;
uint32_t circT0  = 0;

// ===== pH Dosing Pumps =====
#define RELAY_PH_UP   25
#define RELAY_PH_DOWN 27

// pH range and dosing parameters
const float PH_LOW  = 5.60f;
const float PH_HIGH = 6.40f;
const uint32_t PH_DOSE_MS = 2000UL;   // 2-second dose pulse
const int PH_AVG_SAMPLES  = 5;        // number of pH samples to average

// ===== Nutrient Pump =====
#define RELAY_NUTRIENT 32
const float EC_MIN_mScm     = 1.20f;      // minimum desired EC
const uint32_t NUTRIENT_DOSE_MS = 3000UL; // 3-second nutrient pump pulse

// ===== Blynk Virtual Pins =====
#define VPIN_MEASURE    V1   // Manual full measurement
#define VPIN_CIRC_EN    V2   // Circulation enable / emergency shutoff
#define VPIN_STATUS     V5
#define VPIN_TEMP       V6
#define VPIN_EC         V7
#define VPIN_TDS        V8
#define VPIN_PH         V9
#define VPIN_RESERVOIR  V10

// ===== Globals =====
BlynkTimer timer;
bool lowLatched = false;

// ===== Helper Functions =====
void blynkStatus(const String& s) {
  Serial.println(s);
  Blynk.virtualWrite(VPIN_STATUS, s);
}

void sendNotify(const String& msg) {
  // Blynk IoT event ID: low_reservoir (configured in template)
  Blynk.logEvent("low_reservoir", msg);
  Serial.print("NOTIFY: ");
  Serial.println(msg);
}

// ===== Time Helpers =====
bool syncTime() {
  configTzTime(TZ_STRING, "pool.ntp.org", "time.nist.gov");
  Serial.print("Syncing time");
  for (int i=0; i<40; i++) {
    time_t now = time(nullptr);
    if (now > 8*3600*2) {  // arbitrary "time is valid" check
      Serial.println("\nTime synced.");
      return true;
    }
    delay(250);
    Serial.print('.');
  }
  Serial.println("\nTime sync failed; will keep running anyway.");
  return false;
}

bool getLocalTM(struct tm& out) {
  return getLocalTime(&out, 2000);
}

// ===== Ultrasonic =====
bool ultrasonicReadCM(float &cmOut) {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 30000UL);
  if (duration <= 0) return false;

  cmOut = (duration / 2.0f) * 0.0343f;
  return true;
}

float computePercent(float distance) {
  float pct = (RESERVOIR_HEIGHT_CM - distance) / RESERVOIR_HEIGHT_CM * 100.0f;
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return pct;
}

void checkWaterLevel() {
  float dcm;
  if (!ultrasonicReadCM(dcm)) {
    blynkStatus("Ultrasonic read error");
    return;
  }

  float pct = computePercent(dcm);
  Serial.printf("Water: %.2f cm  |  %.1f%%\n", dcm, pct);
  Blynk.virtualWrite(VPIN_RESERVOIR, pct);

  if (!lowLatched && pct <= LOW_THRESHOLD_PCT) {
    lowLatched = true;
    sendNotify("Reservoir low: " + String(pct,1) + "%");
    blynkStatus("Reservoir LOW alert");
  } else if (lowLatched && pct > LOW_THRESHOLD_PCT) {
    lowLatched = false;
    blynkStatus("Reservoir OK (latch reset)");
  }
}

// ===== pH Calibration Helpers =====
uint16_t readMedianMilliVolts(uint8_t n=15) {
  if (n > 25) n = 25;
  uint16_t buf[25];
  for (uint8_t i=0; i<n; i++) {
    buf[i] = analogReadMilliVolts(PH_PIN);
    delay(12);
  }
  // insertion sort
  for (uint8_t i=1; i<n; i++) {
    uint16_t key = buf[i];
    int j = i - 1;
    while (j >= 0 && buf[j] > key) {
      buf[j+1] = buf[j];
      j--;
    }
    buf[j+1] = key;
  }
  return buf[n/2];
}

void computeAndStorePH() {
  if (isnan(V4mV) || isnan(V9mV) || V4mV == V9mV) return;
  const float pH4 = 4.01f, pH9 = 9.18f;
  slope_m     = (pH9 - pH4) / (V9mV - V4mV);
  intercept_b = pH4 - slope_m * V4mV;

  prefs.begin("phcal", false);
  prefs.putFloat("m", slope_m);
  prefs.putFloat("b", intercept_b);
  prefs.putFloat("v4", V4mV);
  prefs.putFloat("v9", V9mV);
  prefs.end();

  Serial.println(F("pH calibration saved."));
}

void loadPHCalibration() {
  prefs.begin("phcal", true);
  if (prefs.isKey("m") && prefs.isKey("b")) {
    slope_m     = prefs.getFloat("m", NAN);
    intercept_b = prefs.getFloat("b", NAN);
    V4mV        = prefs.getFloat("v4", NAN);
    V9mV        = prefs.getFloat("v9", NAN);
    Serial.println(F("Loaded pH calibration from NVS."));
  } else {
    Serial.println(F("No pH calibration found. Use '4' and '9' over Serial."));
  }
  prefs.end();
}

// ===== EC Calibration Helpers =====
void loadECCalibration() {
  prefs.begin("eccal", true);
  ecCalK = prefs.getFloat("k", 1.0f);
  ecCalB = prefs.getFloat("b", 0.0f);
  prefs.end();
  Serial.print(F("EC cal loaded: K="));
  Serial.print(ecCalK, 6);
  Serial.print(F(", B="));
  Serial.println(ecCalB, 6);
}

void saveECCalibration() {
  prefs.begin("eccal", false);
  prefs.putFloat("k", ecCalK);
  prefs.putFloat("b", ecCalB);
  prefs.end();
  Serial.println(F("EC calibration saved."));
}

// Single-point EC calibration: assume probe is in EC_CAL_REF_mScm solution
void calibrateEC() {
  Serial.println(F("EC calibration: place probe in EC calibration solution..."));
  Serial.print(F("Assuming reference EC = "));
  Serial.print(EC_CAL_REF_mScm, 3);
  Serial.println(F(" mS/cm"));

  // Take fresh temperature + EC reading (uncalibrated)
  ds18b20.requestTemperatures();
  float tC = ds18b20.getTempCByIndex(0);

  // Raw EC from voltage (uncalibrated)
  uint32_t sum = 0;
  for (int i=0; i<ADC_SAMPLES; i++) {
    sum += analogRead(EC_PIN);
    delay(2);
  }
  float rawADC = (float)sum / ADC_SAMPLES;
  float ecVolt = (rawADC / 4095.0f) * 3.3f;

  float ec_raw  = EC_SLOPE * max(0.0f, (ecVolt - EC_OFFSET));
  float comp    = (!isnan(tC) ? (1.0f + EC_TEMP_COEFF * (tC - REF_TEMP)) : 1.0f);
  float ec_unscaled = ec_raw / comp;   // this is EC before calibration scaling

  if (isnan(ec_unscaled) || ec_unscaled <= 0.0f) {
    Serial.println(F("EC calibration failed: invalid raw EC reading."));
    return;
  }

  // Compute calibration factor so that:
  // ecCalK * ec_unscaled + ecCalB = EC_CAL_REF_mScm
  ecCalK = EC_CAL_REF_mScm / ec_unscaled;
  ecCalB = 0.0f;  // single-point, no offset

  saveECCalibration();

  Serial.print(F("EC raw (uncalibrated) = "));
  Serial.print(ec_unscaled, 4);
  Serial.println(F(" mS/cm"));
  Serial.print(F("New EC K = "));
  Serial.println(ecCalK, 6);
}

// ===== pH Reading (averaged) =====
float readPHAveraged(int N) {
  if (isnan(slope_m) || isnan(intercept_b)) {
    return NAN;
  }

  double sum = 0.0;
  int count = 0;

  for (int i = 0; i < N; i++) {
    uint16_t VmV = readMedianMilliVolts();
    float p = slope_m * (float)VmV + intercept_b;
    if (!isnan(p)) {
      sum += p;
      count++;
    }
    delay(200);
  }

  if (count == 0) return NAN;
  return (float)(sum / count);
}

// ===== EC Reading =====
float readECVoltage() {
  uint32_t sum = 0;
  for (int i=0; i<ADC_SAMPLES; i++) {
    sum += analogRead(EC_PIN);
    delay(2);
  }
  float raw = (float)sum / ADC_SAMPLES;
  return (raw / 4095.0f) * 3.3f;
}

// ===== pH Dosing Helpers =====
void dosePHUp(uint32_t ms) {
  blynkStatus(String("Dosing pH UP for ") + ms + " ms");
  digitalWrite(RELAY_PH_UP, HIGH);
  delay(ms);
  digitalWrite(RELAY_PH_UP, LOW);
  blynkStatus("pH UP dose complete");
}

void dosePHDown(uint32_t ms) {
  blynkStatus(String("Dosing pH DOWN for ") + ms + " ms");
  digitalWrite(RELAY_PH_DOWN, HIGH);
  delay(ms);
  digitalWrite(RELAY_PH_DOWN, LOW);
  blynkStatus("pH DOWN dose complete");
}

// ===== Nutrient Dosing Helper =====
void doseNutrient(uint32_t ms) {
  blynkStatus(String("Dosing nutrients for ") + ms + " ms");
  digitalWrite(RELAY_NUTRIENT, HIGH);
  delay(ms);
  digitalWrite(RELAY_NUTRIENT, LOW);
  blynkStatus("Nutrient dose complete");
}

// ===== Full Measurement + Dosing Logic =====
void measureSensors() {
  blynkStatus("Measuring Water Level, Temp, EC, TDS, pH...");

  // 1) Water level
  checkWaterLevel();

  // 2) Temperature (needed for EC)
  ds18b20.requestTemperatures();
  temperatureC = ds18b20.getTempCByIndex(0);
  Blynk.virtualWrite(VPIN_TEMP, temperatureC);

  // 3) EC + TDS (with calibration)
  float ecVolt = readECVoltage();
  float ec_raw  = EC_SLOPE * max(0.0f, (ecVolt - EC_OFFSET));
  float comp    = (!isnan(temperatureC) ? (1.0f + EC_TEMP_COEFF * (temperatureC - REF_TEMP)) : 1.0f);
  float ec_unscaled = ec_raw / comp;               // before calibration
  ec_mScm = ec_unscaled * ecCalK + ecCalB;         // calibrated EC
  tds_ppm = ec_mScm * TDS_FACTOR;

  Blynk.virtualWrite(VPIN_EC, ec_mScm);
  Blynk.virtualWrite(VPIN_TDS, tds_ppm);

  // 4) pH averaged
  pH_value = readPHAveraged(PH_AVG_SAMPLES);
  Blynk.virtualWrite(VPIN_PH, pH_value);

  Serial.print("Temp: "); Serial.print(temperatureC);
  Serial.print(" °C | EC(raw): "); Serial.print(ec_unscaled);
  Serial.print(" mS/cm | EC(cal): "); Serial.print(ec_mScm);
  Serial.print(" mS/cm | TDS: "); Serial.print(tds_ppm);
  Serial.print(" ppm | pH(avg): "); Serial.println(pH_value);

  // 5) pH-based dosing (one dose per event max)
  if (!isnan(pH_value)) {
    if (pH_value < PH_LOW) {
      blynkStatus("pH below range: dosing pH UP");
      dosePHUp(PH_DOSE_MS);
    } else if (pH_value > PH_HIGH) {
      blynkStatus("pH above range: dosing pH DOWN");
      dosePHDown(PH_DOSE_MS);
    } else {
      blynkStatus("pH within range: no pH dosing");
    }
  } else {
    blynkStatus("pH invalid (no calibration?) - skipping pH dosing");
  }

  // 6) EC-based nutrient dosing (one dose per event max)
  if (!isnan(ec_mScm)) {
    if (ec_mScm < EC_MIN_mScm) {
      blynkStatus("EC below target: dosing nutrients");
      doseNutrient(NUTRIENT_DOSE_MS);
    } else {
      blynkStatus("EC within range: no nutrient dosing");
    }
  } else {
    blynkStatus("EC invalid - skipping nutrient dosing");
  }

  blynkStatus("Measurement complete");
}

// ===== Scheduled Measurement (7 AM / 7 PM) =====
void checkSchedule() {
  struct tm lt;
  if (!getLocalTM(lt)) return;

  bool isAM = (lt.tm_hour == SCHED_HOUR_AM && lt.tm_min == SCHED_MINUTE);
  bool isPM = (lt.tm_hour == SCHED_HOUR_PM && lt.tm_min == SCHED_MINUTE);
  if (!(isAM || isPM)) return;

  int year = lt.tm_year + 1900;
  int mon  = lt.tm_mon + 1;
  int day  = lt.tm_mday;

  // Same day and same window? Skip
  if (lastRunY == year && lastRunM == mon && lastRunD == day &&
      ((isAM && lastRunWasAM) || (isPM && !lastRunWasAM))) {
    return;
  }

  blynkStatus(isAM ? "Scheduled measurement: 7 AM" : "Scheduled measurement: 7 PM");
  measureSensors();

  lastRunY = year;
  lastRunM = mon;
  lastRunD = day;
  lastRunWasAM = isAM;
}

// ===== Temperature-only periodic update (every 2 hours) =====
void updateTemperatureOnly() {
  ds18b20.requestTemperatures();
  temperatureC = ds18b20.getTempCByIndex(0);
  Blynk.virtualWrite(VPIN_TEMP, temperatureC);
  Serial.print("Temp-only update: ");
  Serial.print(temperatureC);
  Serial.println(" °C");
}

// ===== Circulation Pump Scheduler =====
void runCirculationCycle() {
  if (!circEnabled) {
    // Emergency OFF
    digitalWrite(RELAY_PUMP_CIRC, LOW);
    if (circOn) {
      circOn = false;
      blynkStatus("Circulation: EMERGENCY OFF");
    }
    return;
  }

  uint32_t now = millis();

  if (circOn) {
    // Pump currently ON
    if (now - circT0 >= CIRC_ON_MS) {
      digitalWrite(RELAY_PUMP_CIRC, LOW);
      circOn = false;
      circT0 = now;
      blynkStatus("Circulation: OFF");
    }
  } else {
    // Pump currently OFF
    if (now - circT0 >= CIRC_OFF_MS) {
      digitalWrite(RELAY_PUMP_CIRC, HIGH);
      circOn = true;
      circT0 = now;
      blynkStatus("Circulation: ON");
    }
  }
}

// ===== Blynk Handlers =====
BLYNK_CONNECTED() {
  blynkStatus("Blynk connected");
  Blynk.syncVirtual(VPIN_CIRC_EN);
}

// Manual measurement button on V1
BLYNK_WRITE(VPIN_MEASURE) {
  int state = param.asInt();
  if (state == 1) {       // assume push button or ON edge
    measureSensors();
    // Optional auto-reset:
    // Blynk.virtualWrite(VPIN_MEASURE, 0);
  }
}

// Circulation enable / emergency shutoff on V2
BLYNK_WRITE(VPIN_CIRC_EN) {
  circEnabled = (param.asInt() == 1);
  if (!circEnabled) {
    digitalWrite(RELAY_PUMP_CIRC, LOW);
    circOn = false;
    blynkStatus("Circulation DISABLED (emergency)");
  } else {
    digitalWrite(RELAY_PUMP_CIRC, HIGH);
    circOn = true;
    circT0 = millis();   // start ON duration now
    blynkStatus("Circulation ENABLED (pump ON)");
  }
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println(F("pH calibration commands:"));
  Serial.println(F("  '4' in pH 4.01 buffer"));
  Serial.println(F("  '9' in pH 9.18 buffer"));
  Serial.println(F("  'p' to print current pH calibration"));
  Serial.println(F("EC calibration commands:"));
  Serial.println(F("  'e' in EC cal solution (e.g. 1.413 mS/cm)"));
  Serial.println(F("  'E' to print current EC calibration"));

  // Water level pins
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  // DS18B20
  ds18b20.begin();

  // ADC config
  analogReadResolution(12);
  analogSetPinAttenuation(EC_PIN, ADC_11db);
  analogSetPinAttenuation(PH_PIN, ADC_11db);

  // pH calibration
  loadPHCalibration();

  // EC calibration
  loadECCalibration();

  // Circulation pump relay
  pinMode(RELAY_PUMP_CIRC, OUTPUT);

  // pH dosing relays
  pinMode(RELAY_PH_UP, OUTPUT);
  pinMode(RELAY_PH_DOWN, OUTPUT);
  digitalWrite(RELAY_PH_UP, LOW);
  digitalWrite(RELAY_PH_DOWN, LOW);

  // Nutrient pump relay
  pinMode(RELAY_NUTRIENT, OUTPUT);
  digitalWrite(RELAY_NUTRIENT, LOW);

  // Start circulation pump ON immediately if enabled
  if (circEnabled) {
    digitalWrite(RELAY_PUMP_CIRC, HIGH);
    circOn = true;
    circT0 = millis();
    blynkStatus("Circulation: START ON (boot)");
  } else {
    digitalWrite(RELAY_PUMP_CIRC, LOW);
    circOn = false;
    circT0 = millis();
  }

  // WiFi + Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASS);
  syncTime();

  // Timers
  timer.setInterval(30000L, checkSchedule);         // check schedule every 30s
  timer.setInterval(2UL * 60UL * 60UL * 1000UL, updateTemperatureOnly); // temp every 2h
  timer.setInterval(1000L, runCirculationCycle);    // circulation pump scheduler

  // Initial readings
  updateTemperatureOnly();
}

// ===== Loop =====
void loop() {
  Blynk.run();
  timer.run();

  // Calibration via Serial Monitor
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '4') {
      Serial.println(F("Capturing pH 4.01 mV..."));
      V4mV = readMedianMilliVolts();
      Serial.print(F("V4.01 = "));
      Serial.print(V4mV);
      Serial.println(F(" mV"));
      if (!isnan(V9mV)) computeAndStorePH();
    } else if (c == '9') {
      Serial.println(F("Capturing pH 9.18 mV..."));
      V9mV = readMedianMilliVolts();
      Serial.print(F("V9.18 = "));
      Serial.print(V9mV);
      Serial.println(F(" mV"));
      if (!isnan(V4mV)) computeAndStorePH();
    } else if (c == 'p') {
      Serial.print(F("pH m = ")); Serial.println(slope_m, 8);
      Serial.print(F("pH b = ")); Serial.println(intercept_b, 6);
      Serial.print(F("V4 = ")); Serial.print(V4mV); Serial.println(F(" mV"));
      Serial.print(F("V9 = ")); Serial.print(V9mV); Serial.println(F(" mV"));
    } else if (c == 'e') {
      calibrateEC();
    } else if (c == 'E') {
      Serial.print(F("EC K = ")); Serial.println(ecCalK, 6);
      Serial.print(F("EC B = ")); Serial.println(ecCalB, 6);
      Serial.print(F("EC ref = ")); Serial.print(EC_CAL_REF_mScm, 3);
      Serial.println(F(" mS/cm"));
    }
  }

  delay(5);
}
