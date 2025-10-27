Here is the final code:
/*
  ESP32 Hydroponics + Blynk (Switch buttons for dosing)
  - Scheduled measure+dose at 07:00 and 19:00 (America/New_York via NTP)
  - Servos lower/raise pH & EC probes only during measurement
  - EC temperature-compensated to 25°C for decisions
  - Circulation pump scheduler: 1 min ON / 5 min OFF (when enabled)
  - Blynk controls:
      V0: System ON/OFF (Switch)  [master]
      V1: Circulation scheduler ON/OFF (Switch)
      V2: pH UP manual override (Switch)      <-- switched from Push to Switch
      V3: pH DOWN manual override (Switch)    <-- switched from Push to Switch
      V4: Nutrients manual override (Switch)  <-- switched from Push to Switch
    Telemetry (read-only):
      V10: Temp (°C)
      V11: EC @25°C (mS/cm)
      V12: TDS (ppm)
      V13: pH

  Author: Boris Navarro and William Coleman
*/

#if !defined(ESP32)
  #error "This sketch targets ESP32. In Arduino IDE: Tools → Board → select an ESP32 board."
#endif

// -------- Blynk credentials (YOURS) --------
#define BLYNK_TEMPLATE_ID   "TMPL2DOkYpGvj"
#define BLYNK_TEMPLATE_NAME "Hydroponic"
#define BLYNK_AUTH_TOKEN    "vB6wS9UJe9BIOsmO0t9UBJ1fqb5VY-i7"

#define BLYNK_PRINT Serial
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>
#include <math.h>              // fmaxf()
#include <ESP32Servo.h>        // Library: ESP32Servo
#include <time.h>              // NTP / local time

// ---------- WiFi ----------
const char* ssid     = "wifi name";
const char* password = "password";

// ---------- Time / Scheduling ----------
#define TZ_STRING "EST5EDT,M3.2.0/2,M11.1.0/2"  // America/New_York
const int SCHED_HOUR_AM = 7;    // 07:00
const int SCHED_HOUR_PM = 19;   // 19:00
const int SCHED_MINUTE   = 0;

BlynkTimer timer;

// ---------- DS18B20 ----------
#define ONE_WIRE_BUS 22
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensorsDS(&oneWire);

// ---------- EC Sensor ----------
#define EC_PIN 36                  // ADC1 (GPIO32-39)
#define ADC_SAMPLES 15
#define EC_SLOPE  1.00f            // volts->mS/cm factor (adjust if needed)
#define EC_OFFSET 0.00f            // V offset
#define EC_TEMP_COEFF 0.02f        // ~2%/°C
#define REF_TEMP 25.0f
#define TDS_FACTOR 500             // ppm per mS/cm (500 or 640)

// ---------- pH Sensor ----------
const int PH_PIN = 34;             // ADC1
Preferences prefs;
float slope_m = NAN, intercept_b = NAN; // pH = m*VmV + b
float V4mV = NAN, V9mV = NAN;

// ---------- Servos ----------
const int SERVO_PH_PIN = 14;
const int SERVO_EC_PIN = 27;
Servo servoPH, servoEC;
const int SERVO_UP_ANGLE   = 10;     // adjust to your mechanics
const int SERVO_DOWN_ANGLE = 110;    // adjust to your mechanics
const uint32_t PROBE_SETTLE_MS = 15000; // wait after lowering

// ---------- Relays (assume HIGH = ON; invert if needed) ----------
const int RELAY_PUMP_CIRC   = 26;  // 120V pump via relay
const int RELAY_PH_UP       = 25;
const int RELAY_PH_DOWN     = 33;
const int RELAY_NUTRIENTS   = 32;

// ---------- Targets / Dosing Limits ----------
const float PH_LOW  = 5.60f;
const float PH_HIGH = 6.40f;
const float EC_MIN_mScm = 1.00f;

const uint32_t PH_DOSE_MS               = 3000; // auto single pulse
const uint32_t PH_MAX_MS_PER_WINDOW     = 5000; // cap per window
const uint32_t NUTRIENT_DOSE_MS         = 5000; // auto single pulse
const uint32_t NUTRIENT_MAX_MS_PER_WINDOW = 8000;

// ---------- Sampling ----------
const int NUM_SAMPLES = 10; // per measurement window

// ---------- Circulation Pump Scheduler (1 min ON / 5 min OFF) ----------
const uint32_t CIRC_ON_MS  = 60UL * 1000UL;
const uint32_t CIRC_OFF_MS = 5UL * 60UL * 1000UL;
bool circOn = false;
uint32_t circT0 = 0;

// ---------- Telemetry ----------
float temperatureC = NAN;
float ec_mScm_25   = NAN;   // compensated to 25°C
float tds_ppm      = NAN;
float pH_value     = NAN;

// ---------- State / Flags ----------
bool systemEnabled = false;        // V0
bool circSchedulerEnabled = false; // V1

// Manual override (Switches) for dosing pumps:
bool manualPHUpOn     = false; // V2
bool manualPHDownOn   = false; // V3
bool manualNutrOn     = false; // V4

uint32_t usedPhMsThisWindow = 0;
uint32_t usedNutrientMsThisWindow = 0;

int lastRunY = -1, lastRunM = -1, lastRunD = -1;
bool lastRunWasAM = false;

// ---------- Helpers ----------
inline void relayOn(int pin)  { digitalWrite(pin, HIGH); }
inline void relayOff(int pin) { digitalWrite(pin, LOW);  }

void allRelaysOff() {
  relayOff(RELAY_PUMP_CIRC);
  relayOff(RELAY_PH_UP);
  relayOff(RELAY_PH_DOWN);
  relayOff(RELAY_NUTRIENTS);
}

void probesDown() { servoPH.write(SERVO_DOWN_ANGLE); servoEC.write(SERVO_DOWN_ANGLE); }
void probesUp()   { servoPH.write(SERVO_UP_ANGLE);   servoEC.write(SERVO_UP_ANGLE);   }

// Keep Blynk responsive during longer waits
void waitWithYield(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    Blynk.run();
    timer.run();
    delay(5);
  }
}

uint16_t readMedianMilliVolts(uint8_t n=15) {
  n = constrain(n, 5, 25);
  uint16_t buf[25];
  for (uint8_t i=0;i<n;i++){ buf[i] = analogReadMilliVolts(PH_PIN); delay(12); }
  // insertion sort
  for (uint8_t i=1;i<n;i++){ uint16_t key=buf[i]; int j=i-1; while(j>=0 && buf[j]>key){buf[j+1]=buf[j]; j--;} buf[j+1]=key; }
  return buf[n/2];
}

void computeAndStorePH() {
  if (isnan(V4mV) || isnan(V9mV) || V4mV==V9mV) return;
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
  Serial.print(F("m = ")); Serial.println(slope_m, 8);
  Serial.print(F("b = ")); Serial.println(intercept_b, 6);
}
void loadPHCalibration() {
  prefs.begin("phcal", true);
  if (prefs.isKey("m") && prefs.isKey("b")) {
    slope_m = prefs.getFloat("m", NAN);
    intercept_b = prefs.getFloat("b", NAN);
    V4mV = prefs.getFloat("v4", NAN);
    V9mV = prefs.getFloat("v9", NAN);
  }
  prefs.end();
}

float readECVoltage() {
  uint32_t sum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    sum += analogRead(EC_PIN);
    delay(2);
  }
  float raw = (float)sum / ADC_SAMPLES;
  return (raw / 4095.0f) * 3.3f; // assuming 3.3V ADC reference
}

void readSensorsOnce(float& tC, float& ec25, float& tds_ppm_out, float& pH) {
  // Temperature
  sensorsDS.requestTemperatures();
  tC = sensorsDS.getTempCByIndex(0);
  if (tC == DEVICE_DISCONNECTED_C) tC = NAN;

  // EC (compensated to 25°C) + TDS
  float ecVoltage = readECVoltage();
  float ec_raw = EC_SLOPE * fmaxf(0.0f, (ecVoltage - EC_OFFSET)); // mS/cm at measured T
  float compFactor = (!isnan(tC) ? (1.0f + EC_TEMP_COEFF * (tC - REF_TEMP)) : NAN);
  ec25 = (!isnan(compFactor) && compFactor > 0.0f) ? (ec_raw / compFactor) : NAN;
  tds_ppm_out = (!isnan(ec25)) ? (ec25 * TDS_FACTOR) : NAN;

  // pH
  float VmV = readMedianMilliVolts();
  pH = (!isnan(slope_m) && !isnan(intercept_b)) ? (slope_m * VmV + intercept_b) : NAN;
}

void readSensorsAveraged(int N, float& tC, float& ec25, float& tds_ppm_out, float& pH) {
  double sumT = 0, sumEC = 0, sumTDS = 0, sumPH = 0;
  int cT = 0, cEC = 0, cTDS = 0, cPH = 0;

  for (int i=0;i<N;i++) {
    float a,b,c,d;
    readSensorsOnce(a,b,c,d);
    if (!isnan(a)) { sumT += a; cT++; }
    if (!isnan(b)) { sumEC += b; cEC++; }
    if (!isnan(c)) { sumTDS += c; cTDS++; }
    if (!isnan(d)) { sumPH += d; cPH++; }
    waitWithYield(200);
  }
  tC  = (cT   ? sumT  / cT   : NAN);
  ec25= (cEC  ? sumEC / cEC  : NAN);
  tds_ppm_out = (cTDS ? sumTDS/ cTDS : NAN);
  pH  = (cPH  ? sumPH / cPH  : NAN);
}

void pushReadingsToBlynk() {
  if (!isnan(temperatureC)) Blynk.virtualWrite(V10, temperatureC);
  if (!isnan(ec_mScm_25))   Blynk.virtualWrite(V11, ec_mScm_25);
  if (!isnan(tds_ppm))      Blynk.virtualWrite(V12, tds_ppm);
  if (!isnan(pH_value))     Blynk.virtualWrite(V13, pH_value);
}

// Apply manual overrides to relays every loop tick
void applyManualRelayStates() {
  if (!systemEnabled) {
    // Master OFF: everything forced OFF
    allRelaysOff();
    return;
  }

  // Manual dosing overrides (Switches): hold relays ON while switch is ON
  if (manualPHUpOn)   relayOn(RELAY_PH_UP);     else relayOff(RELAY_PH_UP);
  if (manualPHDownOn) relayOn(RELAY_PH_DOWN);   else relayOff(RELAY_PH_DOWN);
  if (manualNutrOn)   relayOn(RELAY_NUTRIENTS); else relayOff(RELAY_NUTRIENTS);
}

// Dosing helpers with per-window caps; they SKIP if manual override is ON
void dosePHUp(uint32_t ms) {
  if (!systemEnabled || manualPHUpOn) return; // skip if manual ON
  uint32_t allow = (usedPhMsThisWindow + ms > PH_MAX_MS_PER_WINDOW)
                    ? (PH_MAX_MS_PER_WINDOW - usedPhMsThisWindow) : ms;
  if ((int32_t)allow <= 0) return;
  Serial.printf("AUTO: pH UP for %lu ms\n", (unsigned long)allow);
  relayOn(RELAY_PH_UP);
  waitWithYield(allow);
  relayOff(RELAY_PH_UP);
  usedPhMsThisWindow += allow;
}

void dosePHDown(uint32_t ms) {
  if (!systemEnabled || manualPHDownOn) return; // skip if manual ON
  uint32_t allow = (usedPhMsThisWindow + ms > PH_MAX_MS_PER_WINDOW)
                    ? (PH_MAX_MS_PER_WINDOW - usedPhMsThisWindow) : ms;
  if ((int32_t)allow <= 0) return;
  Serial.printf("AUTO: pH DOWN for %lu ms\n", (unsigned long)allow);
  relayOn(RELAY_PH_DOWN);
  waitWithYield(allow);
  relayOff(RELAY_PH_DOWN);
  usedPhMsThisWindow += allow;
}

void doseNutrients(uint32_t ms) {
  if (!systemEnabled || manualNutrOn) return; // skip if manual ON
  uint32_t allow = (usedNutrientMsThisWindow + ms > NUTRIENT_MAX_MS_PER_WINDOW)
                    ? (NUTRIENT_MAX_MS_PER_WINDOW - usedNutrientMsThisWindow) : ms;
  if ((int32_t)allow <= 0) return;
  Serial.printf("AUTO: Nutrients for %lu ms\n", (unsigned long)allow);
  relayOn(RELAY_NUTRIENTS);
  waitWithYield(allow);
  relayOff(RELAY_NUTRIENTS);
  usedNutrientMsThisWindow += allow;
}

// ----- Scheduled measurement pass -----
void performScheduledMeasurement(bool isAMwindow) {
  if (!systemEnabled) {
    Serial.println("[Skip] System disabled.");
    return;
  }
  Serial.println("\n=== Scheduled Measurement START ===");
  usedPhMsThisWindow = 0;
  usedNutrientMsThisWindow = 0;

  probesDown();
  waitWithYield(PROBE_SETTLE_MS);

  float tC, ec25, tds, ph;
  readSensorsAveraged(NUM_SAMPLES, tC, ec25, tds, ph);

  temperatureC = tC;
  ec_mScm_25   = ec25;
  tds_ppm      = tds;
  pH_value     = ph;
  pushReadingsToBlynk();

  // Auto-dosing decisions (use EC @25°C)
  if (!isnan(pH_value)) {
    if (pH_value < PH_LOW)       dosePHUp(PH_DOSE_MS);
    else if (pH_value > PH_HIGH) dosePHDown(PH_DOSE_MS);
    else Serial.println("pH within range—no pH dosing.");
  } else {
    Serial.println("pH invalid—skipping pH dosing.");
  }

  if (!isnan(ec_mScm_25)) {
    if (ec_mScm_25 < EC_MIN_mScm) doseNutrients(NUTRIENT_DOSE_MS);
    else Serial.println("EC above minimum—no nutrients.");
  } else {
    Serial.println("EC invalid—skipping nutrient dosing.");
  }

  probesUp();

  // Mark last run to prevent duplicates
  struct tm lt;
  if (getLocalTime(&lt, 2000)) {
    lastRunY = lt.tm_year + 1900;
    lastRunM = lt.tm_mon + 1;
    lastRunD = lt.tm_mday;
    lastRunWasAM = isAMwindow;
  }
  Serial.println("=== Scheduled Measurement END ===\n");
}

bool checkAndRunSchedule() {
  struct tm lt;
  if (!getLocalTime(&lt, 500)) return false;

  bool isAM = (lt.tm_hour == SCHED_HOUR_AM && lt.tm_min == SCHED_MINUTE);
  bool isPM = (lt.tm_hour == SCHED_HOUR_PM && lt.tm_min == SCHED_MINUTE);
  if (!(isAM || isPM)) return false;

  if (lastRunY == (lt.tm_year + 1900) &&
      lastRunM == (lt.tm_mon + 1) &&
      lastRunD == lt.tm_mday &&
      ((isAM && lastRunWasAM) || (isPM && !lastRunWasAM))) {
    return false; // already ran this window today
  }

  performScheduledMeasurement(isAM);
  return true;
}

// ---------- Blynk Handlers ----------
BLYNK_CONNECTED() {
  // Sync button states on connect
  Blynk.syncVirtual(V0, V1, V2, V3, V4);
}

BLYNK_WRITE(V0) { // System ON/OFF (master, Switch)
  systemEnabled = (param.asInt() == 1);
  Serial.printf("System %s\n", systemEnabled ? "ENABLED" : "DISABLED");
  if (!systemEnabled) {
    // force everything off
    manualPHUpOn = manualPHDownOn = manualNutrOn = false;
    allRelaysOff();
    probesUp();
  }
}

BLYNK_WRITE(V1) { // Circulation scheduler ON/OFF (Switch)
  circSchedulerEnabled = (param.asInt() == 1);
  Serial.printf("Circulation scheduler %s\n", circSchedulerEnabled ? "ENABLED" : "DISABLED");
  if (!circSchedulerEnabled) {
    relayOff(RELAY_PUMP_CIRC);
    circOn = false;
    circT0 = millis();
  }
}

BLYNK_WRITE(V2) { // pH UP manual override (Switch)
  manualPHUpOn = (param.asInt() == 1);
  Serial.printf("Manual pH UP: %s\n", manualPHUpOn ? "ON" : "OFF");
}

BLYNK_WRITE(V3) { // pH DOWN manual override (Switch)
  manualPHDownOn = (param.asInt() == 1);
  Serial.printf("Manual pH DOWN: %s\n", manualPHDownOn ? "ON" : "OFF");
}

BLYNK_WRITE(V4) { // Nutrients manual override (Switch)
  manualNutrOn = (param.asInt() == 1);
  Serial.printf("Manual Nutrients: %s\n", manualNutrOn ? "ON" : "OFF");
}

// ---------- Setup / Loop ----------
void setup() {
  Serial.begin(115200);
  delay(800);

  // ADC config
  #if defined(analogReadResolution)
    analogReadResolution(12);
  #endif
  #if defined(analogSetPinAttenuation) && defined(ADC_11db)
    analogSetPinAttenuation((gpio_num_t)EC_PIN, ADC_11db);
    analogSetPinAttenuation((gpio_num_t)PH_PIN, ADC_11db);
  #elif defined(analogSetPinAttenuation) && defined(ADC_ATTEN_DB_11)
    analogSetPinAttenuation(EC_PIN, ADC_ATTEN_DB_11);
    analogSetPinAttenuation(PH_PIN, ADC_ATTEN_DB_11);
  #endif

  // Temperature sensor
  sensorsDS.begin();

  // pH calibration helpers
  Serial.println(F("\nESP32 pH: press '4' in pH4.01, '9' in pH9.18, 'p' to print."));
  loadPHCalibration();
  if (!isnan(slope_m) && !isnan(intercept_b)) {
    Serial.print(F("Loaded pH calibration. m=")); Serial.print(slope_m,8);
    Serial.print(F(", b=")); Serial.println(intercept_b,6);
  } else {
    Serial.println(F("No pH calibration yet."));
  }

  // WiFi + Blynk
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect(15000); // try for 15s; will keep trying in loop if needed

  // NTP
  configTzTime(TZ_STRING, "pool.ntp.org", "time.nist.gov");

  // Servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servoPH.setPeriodHertz(50);
  servoEC.setPeriodHertz(50);
  servoPH.attach(SERVO_PH_PIN, 500, 2400);
  servoEC.attach(SERVO_EC_PIN, 500, 2400);
  probesUp(); // start lifted

  // Relays
  pinMode(RELAY_PUMP_CIRC, OUTPUT);
  pinMode(RELAY_PH_UP, OUTPUT);
  pinMode(RELAY_PH_DOWN, OUTPUT);
  pinMode(RELAY_NUTRIENTS, OUTPUT);
  allRelaysOff();

  // Push telemetry to Blynk every 5s
  timer.setInterval(5000, pushReadingsToBlynk);
}

void loop() {
  Blynk.run();
  timer.run();

  // Serial keys for pH calibration (optional)
  if (Serial.available()) {
    char c = Serial.read();
    if (c=='4') {
      V4mV = readMedianMilliVolts();
      Serial.print(F("Captured V4.01 = ")); Serial.print(V4mV); Serial.println(F(" mV"));
      if (!isnan(V9mV)) computeAndStorePH();
    } else if (c=='9') {
      V9mV = readMedianMilliVolts();
      Serial.print(F("Captured V9.18 = ")); Serial.print(V9mV); Serial.println(F(" mV"));
      if (!isnan(V4mV)) computeAndStorePH();
    } else if (c=='p') {
      Serial.print(F("m=")); Serial.print(slope_m,8);
      Serial.print(F(", b=")); Serial.println(intercept_b,6);
      Serial.print(F("V4=")); Serial.print(V4mV);
      Serial.print(F(" mV, V9=")); Serial.print(V9mV); Serial.println(F(" mV"));
    }
  }

  // Apply manual dosing switch states continuously
  applyManualRelayStates();

  // Circulation scheduler (only if system & scheduler enabled)
  if (systemEnabled && circSchedulerEnabled) {
    uint32_t now = millis();
    if (circOn) {
      if (now - circT0 >= CIRC_ON_MS) {
        relayOff(RELAY_PUMP_CIRC);
        circOn = false;
        circT0 = now;
        Serial.println("Circulation: OFF (completed 1 min)");
      }
    } else {
      if (now - circT0 >= CIRC_OFF_MS) {
        relayOn(RELAY_PUMP_CIRC);
        circOn = true;
        circT0 = now;
        Serial.println("Circulation: ON (starting 1 min)");
      }
    }
  } else {
    // If disabled, ensure pump is OFF
    if (circOn) { relayOff(RELAY_PUMP_CIRC); circOn = false; }
  }

  // Scheduled measurement windows
  if (systemEnabled) {
    checkAndRunSchedule();
  }
}