/*
 * SMD Reflow - PID regulator temperature za grejac pegle (800 W) preko SSR-a
 * Hardver: ESP32, MAX6675 + termopar tip K, SH1106 OLED 128x64, rotacioni enkoder, buzzer, SSR
 *
 * Verzija: 1.2
 *  - Signal (3 kratka pistanja) i tajmer na ekranu kada se dostigne zadata temperatura
 *  - Brzi izbor temperature: dug pritisak na SET -> 150 / 180 / 220 / 240 C
 *  - Taster: kratak pritisak na pustanje, dug pritisak 0,8 s; drzanje vise ne ponavlja akciju
 *
 * Verzija 1.1:
 *  - Bezbednost: otkaz senzora (NAN / nemoguca vrednost) odmah gasi grejac i zaustavlja proces
 *  - Bezbednost: apsolutna granica temperature (MAX_SAFE_TEMP), proverava se i kada proces ne radi
 *  - Bezbednost: provera rasta temperature (grejac pun, a temperatura ne raste -> FAULT)
 *  - START je blokiran dok senzor ne radi ili dok greska nije potvrdjena tasterom
 *  - PID se cisto resetuje na svaki START (nema zaostalog integrala), PWM prozor krece od nule
 *  - PID racuna tacno jednom po novom ocitavanju temperature (SampleTime = period citanja)
 *  - Serijski ispis u CSV formatu jednom u sekundi, osvezavanje ekrana 5x u sekundi
 *
 * Verzija 1.0 se nalazi u folderu v1.0/
 */

#include <Wire.h>
#include <U8g2lib.h>
#include <max6675.h>
#include <RotaryEncoder.h>
#include <PID_v1.h>
#include "mera_logo.h"

#define FW_VERSION "1.2"

// ---------------------------------------------------------------------------
// OLED
// ---------------------------------------------------------------------------
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#define SCREEN_WIDTH 128

// ---------------------------------------------------------------------------
// MAX6675
// ---------------------------------------------------------------------------
const int MAX6675_SCK = 18;
const int MAX6675_CS  = 5;
const int MAX6675_SO  = 19;
MAX6675 thermocouple(MAX6675_SCK, MAX6675_CS, MAX6675_SO);

// ---------------------------------------------------------------------------
// Rotacioni enkoder
// ---------------------------------------------------------------------------
#define ENCODER_CLK_PIN 25
#define ENCODER_DT_PIN  26
#define ENCODER_SW_PIN  27   // eksterni pull-up otpornik na ploci
RotaryEncoder encoder(ENCODER_CLK_PIN, ENCODER_DT_PIN, RotaryEncoder::LatchMode::FOUR3);
volatile bool encoderChanged = false;

// ---------------------------------------------------------------------------
// Izlazi
// ---------------------------------------------------------------------------
const int BUZZER_PIN  = 2;   // GPIO 2
const int SSR_PIN     = 23;  // GPIO 23 -> SSR
const int PID_LED_PIN = 32;  // LED: aktivno zagrevanje
const float hysteresisTemp = 2.0; // Prag (C) ispod Setpointa gde se LED gasi

// ---------------------------------------------------------------------------
// BEZBEDNOSNI PARAMETRI
// ---------------------------------------------------------------------------
const float MAX_SAFE_TEMP        = 270.0; // Iznad ove temperature: trenutno gasenje i alarm (uvek aktivno)
const float MIN_VALID_TEMP       = 1.0;   // Ocitavanje ispod ovoga smatra se neispravnim (MAX6675 vraca 0 kad ne radi)
const float MAX_VALID_TEMP       = 500.0; // Ocitavanje iznad ovoga smatra se neispravnim
const int   SENSOR_FAULT_COUNT   = 3;     // Broj uzastopnih losih ocitavanja (3 x 250 ms) pre FAULT-a

// Provera rasta temperature (termopar ispao sa ploce, pregoreo grejac, SSR ne ukljucuje...)
// Ako je PID izlaz >= NO_RISE_OUTPUT_PCT, a temperatura je bar NO_RISE_MIN_ERROR ispod zadate,
// temperatura mora da poraste bar NO_RISE_MIN_DELTA za NO_RISE_TIME_MS.
const float         NO_RISE_OUTPUT_PCT = 90.0;
const float         NO_RISE_MIN_ERROR  = 10.0;
const float         NO_RISE_MIN_DELTA  = 3.0;
const unsigned long NO_RISE_TIME_MS    = 90000UL;

enum FaultCode {
  FAULT_NONE = 0,
  FAULT_SENSOR,    // termopar otkacen / MAX6675 ne odgovara
  FAULT_OVERTEMP,  // temperatura iznad MAX_SAFE_TEMP
  FAULT_NO_RISE    // grejac radi punom snagom, a temperatura ne raste
};
FaultCode activeFault = FAULT_NONE;

// ---------------------------------------------------------------------------
// PID (Gain Scheduling po zadatoj temperaturi)
// ---------------------------------------------------------------------------
const double KI_LOW_TEMP  = 0.025; // Ki za zadatu temperaturu <= 160C
const double KI_MID_TEMP  = 0.04;  // Ki za 161C - 180C
const double KI_HIGH_TEMP = 0.06;  // Ki za > 180C

// Kp i Kd su konstantni (vrednosti dobijene tjuningom)
const double FIXED_KP = 18.0;
const double FIXED_KD = 1.0;

double Kp_current = FIXED_KP;
double Ki_current = KI_LOW_TEMP;
double Kd_current = FIXED_KD;

double Input;    // Stvarna temperatura
double Output;   // Izlaz PID-a = vreme ukljucenja u ms unutar WindowSize
double Setpoint; // Zadata temperatura

PID myPID(&Input, &Output, &Setpoint, Kp_current, Ki_current, Kd_current, DIRECT);

// Time-proportional kontrola SSR-a
const unsigned long WindowSize = 2000; // ms
unsigned long windowStartTime;

// ---------------------------------------------------------------------------
// Meni
// ---------------------------------------------------------------------------
int menuIndex = 0;
const int totalMenuItems = 3; // SET, BEEP, START/STOP
int desiredTemperature = 160;
const int MIN_SET_TEMP = 50;
const int MAX_SET_TEMP = 250; // mora biti dovoljno ispod MAX_SAFE_TEMP
bool editingTemperature = false;
bool editingBeep = true;
bool isReflowRunning = false;

// Taster enkodera: kratak pritisak (na pustanje) i dug pritisak
const unsigned long debounceDelay = 50;    // ms
const unsigned long longPressTime = 800;   // ms

// Brzi izbor temperature: dug pritisak na SET prelazi na sledecu vrednost
const int PRESET_TEMPS[] = {150, 180, 220, 240};
const int PRESET_COUNT = sizeof(PRESET_TEMPS) / sizeof(PRESET_TEMPS[0]);

// Signal "temperatura dostignuta" + tajmer
const float READY_BAND = 2.0;   // C - temperatura je "dostignuta" kada je u opsegu SET +/- READY_BAND
bool  setpointReached = false;
unsigned long reachedTime = 0;
int   readySetpoint = -1;       // za koju zadatu temperaturu vazi setpointReached

// ---------------------------------------------------------------------------
// Temperatura
// ---------------------------------------------------------------------------
unsigned long lastTempReadTime = 0;
const unsigned long tempReadInterval = 250; // MAX6675 konverzija traje do 220 ms
float currentTempC = NAN;
bool  sensorOk = false;
int   badReadingCount = 0;

// Provera rasta
unsigned long noRiseCheckStart = 0;
float noRiseCheckTemp = 0.0;
bool  noRiseTracking = false;

// Ispis
unsigned long lastDisplayTime = 0;
const unsigned long displayInterval = 200;  // 5 Hz
unsigned long lastLogTime = 0;
const unsigned long logInterval = 1000;     // 1 Hz CSV
unsigned long processStartTime = 0;

// ===========================================================================
// Pomocne funkcije
// ===========================================================================

void IRAM_ATTR readEncoderISR() {
  encoder.tick();
  encoderChanged = true;
}

bool isValidTemp(float t) {
  return !isnan(t) && t >= MIN_VALID_TEMP && t <= MAX_VALID_TEMP;
}

void beep(int durationMs) {
  if (editingBeep) {
    tone(BUZZER_PIN, 4000, durationMs);
    noTone(BUZZER_PIN);
  }
}

// Alarm se cuje uvek, bez obzira na BEEP podesavanje
void alarmBeep() {
  for (int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, 3000, 300);
    delay(400);
  }
  noTone(BUZZER_PIN);
}

// Signal da je zadata temperatura dostignuta - cuje se uvek, bez obzira na BEEP podesavanje
void readyBeep() {
  for (int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, 4500, 80);
    delay(150);
  }
  noTone(BUZZER_PIN);
}

// Prati da li je temperatura dostignuta; kod prvog ulaska u opseg pusta signal i pokrece tajmer.
// Promena zadate temperature resetuje i signal i tajmer.
void checkSetpointReached() {
  if (!isReflowRunning) {
    setpointReached = false;
    readySetpoint = -1;
    return;
  }
  if (readySetpoint != desiredTemperature) {
    readySetpoint = desiredTemperature;
    setpointReached = false;
  }
  if (!setpointReached && fabs(currentTempC - desiredTemperature) <= READY_BAND) {
    setpointReached = true;
    reachedTime = millis();
    Serial.print("# Setpoint reached: ");
    Serial.println(desiredTemperature);
    readyBeep();
  }
}

// Jedino mesto gde se SSR ukljucuje. Grejac moze da radi SAMO ako proces radi,
// nema aktivne greske i poslednje ocitavanje je ispravno.
void setHeater(bool on) {
  if (on && isReflowRunning && activeFault == FAULT_NONE && sensorOk) {
    digitalWrite(SSR_PIN, HIGH);
  } else {
    digitalWrite(SSR_PIN, LOW);
  }
}

const char* faultText(FaultCode f) {
  switch (f) {
    case FAULT_SENSOR:   return "SENSOR ERROR";
    case FAULT_OVERTEMP: return "OVER TEMP!";
    case FAULT_NO_RISE:  return "NO HEAT RISE";
    default:             return "";
  }
}

void updatePIDTunings() {
  double newKi;
  if (desiredTemperature <= 160) {
    newKi = KI_LOW_TEMP;
  } else if (desiredTemperature <= 180) {
    newKi = KI_MID_TEMP;
  } else {
    newKi = KI_HIGH_TEMP;
  }

  if (newKi != Ki_current) {
    Ki_current = newKi;
    myPID.SetTunings(Kp_current, Ki_current, Kd_current);
    Serial.print("# PID tunings updated: Kp="); Serial.print(Kp_current, 3);
    Serial.print(", Ki="); Serial.print(Ki_current, 3);
    Serial.print(", Kd="); Serial.println(Kd_current, 3);
  }
}

void stopProcess() {
  isReflowRunning = false;
  myPID.SetMode(MANUAL); // PID prestaje da racuna, Output ostaje ono sto upisemo
  Output = 0;
  noRiseTracking = false;
  digitalWrite(SSR_PIN, LOW);
  digitalWrite(PID_LED_PIN, LOW);
}

bool startProcess() {
  if (activeFault != FAULT_NONE || !sensorOk) {
    Serial.println("# START blocked: sensor not OK or fault active");
    return false;
  }
  updatePIDTunings();
  Input = currentTempC;
  Setpoint = desiredTemperature;
  Output = 0;
  myPID.SetMode(AUTOMATIC);   // prelaz MANUAL -> AUTOMATIC radi Initialize(): ITerm = 0, lastInput = Input
  windowStartTime = millis(); // PWM prozor krece od sada
  processStartTime = millis();
  noRiseTracking = false;
  setpointReached = false;
  readySetpoint = -1;
  isReflowRunning = true;
  Serial.println("# time_s,temp_c,setpoint_c,output_pct");
  return true;
}

void raiseFault(FaultCode f) {
  if (activeFault != FAULT_NONE) return; // vec postoji greska
  activeFault = f;
  stopProcess();
  editingTemperature = false;
  Serial.print("# FAULT: ");
  Serial.print(faultText(f));
  Serial.print("  temp=");
  Serial.println(currentTempC);
  alarmBeep();
}

// Provera: grejac radi skoro punom snagom, a temperatura ne raste
void checkHeatRise() {
  float outputPct = (Output / WindowSize) * 100.0;
  bool heatingHard = outputPct >= NO_RISE_OUTPUT_PCT && (Setpoint - Input) >= NO_RISE_MIN_ERROR;

  if (!heatingHard) {
    noRiseTracking = false;
    return;
  }
  if (!noRiseTracking) {
    noRiseTracking = true;
    noRiseCheckStart = millis();
    noRiseCheckTemp = currentTempC;
    return;
  }
  if (millis() - noRiseCheckStart >= NO_RISE_TIME_MS) {
    if (currentTempC - noRiseCheckTemp < NO_RISE_MIN_DELTA) {
      raiseFault(FAULT_NO_RISE);
    } else {
      // temperatura raste - pocni novi interval provere
      noRiseCheckStart = millis();
      noRiseCheckTemp = currentTempC;
    }
  }
}

// Ocitavanje temperature + sve bezbednosne provere vezane za senzor
void readTemperature() {
  float t = thermocouple.readCelsius();

  if (isValidTemp(t)) {
    currentTempC = t;
    sensorOk = true;
    badReadingCount = 0;
  } else {
    sensorOk = false; // setHeater() odmah odbija ukljucenje
    if (badReadingCount < SENSOR_FAULT_COUNT) badReadingCount++;
    if (badReadingCount >= SENSOR_FAULT_COUNT) {
      currentTempC = NAN;
      if (isReflowRunning) raiseFault(FAULT_SENSOR);
    }
    return;
  }

  // Apsolutna granica - proverava se uvek, i kada proces ne radi
  if (currentTempC > MAX_SAFE_TEMP) {
    raiseFault(FAULT_OVERTEMP);
    return;
  }

  if (isReflowRunning) {
    Input = currentTempC;
    Setpoint = desiredTemperature;
    myPID.Compute();
    if (isnan(Output)) {           // dodatna zastita, ne bi smelo da se desi
      raiseFault(FAULT_SENSOR);
      return;
    }
    checkHeatRise();
    if (activeFault == FAULT_NONE) checkSetpointReached();
  }
}

// ===========================================================================
// Setup
// ===========================================================================
void setup() {
  // Grejac iskljucen sto je pre moguce
  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW);
  pinMode(PID_LED_PIN, OUTPUT);
  digitalWrite(PID_LED_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);

  Serial.begin(115200);
  Serial.println("# SMD Reflow PID v" FW_VERSION);

  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.clearBuffer();
  u8g2.drawBitmap(0, 0, 128 / 8, 64, myLogo);
  u8g2.sendBuffer();

  delay(500); // MAX6675 treba ~250 ms posle ukljucenja za prvu konverziju
  float tempTest = thermocouple.readCelsius();
  sensorOk = isValidTemp(tempTest);
  if (sensorOk) currentTempC = tempTest;

  u8g2.setFont(u8g2_font_7x13_tf);
  if (!sensorOk) {
    Serial.println("# MAX6675 ERROR or thermocouple not connected!");
    u8g2.clearBuffer();
    u8g2.setCursor(0, 12);
    u8g2.print("MAX6675 ERROR!");
    u8g2.setCursor(0, 28);
    u8g2.print("Check thermocouple");
    u8g2.setCursor(0, 44);
    u8g2.print("START blocked");
  } else {
    Serial.println("# MAX6675 initialized and working.");
    u8g2.setCursor(0, 12);
    u8g2.print("MAX6675 OK  v" FW_VERSION);
  }
  u8g2.sendBuffer();
  delay(2500);

  tone(BUZZER_PIN, 4000, 50); noTone(BUZZER_PIN);
  delay(100);
  tone(BUZZER_PIN, 4000, 50); noTone(BUZZER_PIN);

  // PID: ostaje u MANUAL dok se ne pritisne START
  myPID.SetOutputLimits(0, WindowSize);
  // SampleTime malo kraci od perioda citanja, da Compute() sigurno racuna pri svakom novom
  // ocitavanju (razlika 4% u efektivnom Ki/Kd je zanemarljiva)
  myPID.SetSampleTime(tempReadInterval - 10);
  myPID.SetMode(MANUAL);
  Output = 0;
  updatePIDTunings();

  Serial.print("# Initial PID tunings: Kp="); Serial.print(Kp_current, 3);
  Serial.print(", Ki="); Serial.print(Ki_current, 3);
  Serial.print(", Kd="); Serial.println(Kd_current, 3);

  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK_PIN), readEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_DT_PIN), readEncoderISR, CHANGE);
}

// ===========================================================================
// Ulaz (enkoder i taster)
// ===========================================================================
void handleEncoder() {
  if (!encoderChanged) return;
  encoderChanged = false;

  static int oldPosition = 0;
  int newPosition = encoder.getPosition();
  if (newPosition == oldPosition) return;

  int direction = newPosition - oldPosition;
  oldPosition = newPosition;

  if (activeFault != FAULT_NONE) return; // meni je zakljucan dok se greska ne potvrdi

  beep(50);

  if (editingTemperature) {
    desiredTemperature += direction;
    bool limitReached = false;
    if (desiredTemperature < MIN_SET_TEMP) { desiredTemperature = MIN_SET_TEMP; limitReached = true; }
    if (desiredTemperature > MAX_SET_TEMP) { desiredTemperature = MAX_SET_TEMP; limitReached = true; }
    if (editingBeep && limitReached) {
      tone(BUZZER_PIN, 4000, 200);
      delay(100);
      tone(BUZZER_PIN, 4000, 200);
      noTone(BUZZER_PIN);
    }
  } else {
    menuIndex = (menuIndex + direction + totalMenuItems) % totalMenuItems;
    editingTemperature = false;
  }
}

// Dug pritisak na SET: sledeca brza vrednost temperature (vece od trenutne, ili prva u listi)
void selectNextPreset() {
  int next = PRESET_TEMPS[0];
  for (int i = 0; i < PRESET_COUNT; i++) {
    if (PRESET_TEMPS[i] > desiredTemperature) { next = PRESET_TEMPS[i]; break; }
  }
  desiredTemperature = next;
  editingTemperature = false;
  updatePIDTunings();
  Serial.print("# Preset selected: ");
  Serial.println(desiredTemperature);
  if (editingBeep) {
    tone(BUZZER_PIN, 4000, 50); delay(80);
    tone(BUZZER_PIN, 4000, 50);
    noTone(BUZZER_PIN);
  }
}

void onShortPress();

void onLongPress() {
  if (activeFault != FAULT_NONE) { onShortPress(); return; }
  if (menuIndex == 0) {
    selectNextPreset();
  } else {
    onShortPress(); // na ostalim stavkama dug pritisak radi isto sto i kratak
  }
}

// Taster: kratak pritisak se izvrsava na pustanje, dug pritisak cim se drzi longPressTime.
// Drzanje tastera vise ne ponavlja akciju (u v1.0 se akcija ponavljala na svakih 250 ms).
void handleButton() {
  static bool isDown = false;
  static bool longFired = false;
  static unsigned long pressStart = 0;
  static unsigned long lastEdge = 0;

  bool pressed = (digitalRead(ENCODER_SW_PIN) == LOW);
  unsigned long now = millis();

  if (pressed != isDown) {
    if (now - lastEdge < debounceDelay) return;
    lastEdge = now;
    isDown = pressed;
    if (pressed) {
      pressStart = now;
      longFired = false;
    } else if (!longFired) {
      onShortPress();
    }
    return;
  }

  if (isDown && !longFired && (now - pressStart >= longPressTime)) {
    longFired = true;
    onLongPress();
  }
}

void onShortPress() {
  // Potvrda greske: brise se samo ako je senzor ponovo ispravan i temperatura ispod granice
  if (activeFault != FAULT_NONE) {
    if (sensorOk && currentTempC < MAX_SAFE_TEMP) {
      Serial.println("# Fault acknowledged");
      activeFault = FAULT_NONE;
      beep(50);
    } else {
      tone(BUZZER_PIN, 2000, 300); noTone(BUZZER_PIN);
    }
    return;
  }

  if (menuIndex == 0) { // SET
    editingTemperature = !editingTemperature;
    if (!editingTemperature) updatePIDTunings();
    beep(50);

  } else if (menuIndex == 1) { // BEEP
    editingBeep = !editingBeep;
    beep(75);
    editingTemperature = false;

  } else if (menuIndex == 2) { // START/STOP
    editingTemperature = false;
    if (!isReflowRunning) {
      if (startProcess()) {
        Serial.println("# Reflow process STARTED");
        if (editingBeep) {
          tone(BUZZER_PIN, 4000, 70);  delay(100);
          tone(BUZZER_PIN, 4000, 200); delay(100);
          noTone(BUZZER_PIN);
        }
      } else {
        tone(BUZZER_PIN, 2000, 300); noTone(BUZZER_PIN); // START odbijen
      }
    } else {
      stopProcess();
      Serial.println("# Reflow process STOPPED");
      if (editingBeep) {
        tone(BUZZER_PIN, 4000, 200); delay(100);
        tone(BUZZER_PIN, 4000, 70);  delay(100);
        noTone(BUZZER_PIN);
      }
    }
  }
}

// ===========================================================================
// Grejac (time-proportional PWM) i LED
// ===========================================================================
void driveHeater() {
  if (!isReflowRunning) {
    setHeater(false);
    digitalWrite(PID_LED_PIN, LOW);
    return;
  }

  unsigned long now = millis();
  while (now - windowStartTime >= WindowSize) {
    windowStartTime += WindowSize;
  }
  setHeater((now - windowStartTime) < Output);

  digitalWrite(PID_LED_PIN, (currentTempC < (Setpoint - hysteresisTemp)) ? HIGH : LOW);
}

// ===========================================================================
// Serijski log (CSV) - pogodan za Serial Plotter ili Excel
// ===========================================================================
void logSerial() {
  if (millis() - lastLogTime < logInterval) return;
  lastLogTime = millis();

  if (isReflowRunning) {
    Serial.print((millis() - processStartTime) / 1000.0, 1); Serial.print(',');
    Serial.print(currentTempC, 2);                             Serial.print(',');
    Serial.print(Setpoint, 0);                                 Serial.print(',');
    Serial.println((Output / WindowSize) * 100.0, 1);
  } else if (!sensorOk) {
    Serial.println("# TEMP ERROR");
  } else {
    Serial.print("# idle temp: ");
    Serial.println(currentTempC, 2);
  }
}

// ===========================================================================
// Ekran
// ===========================================================================
void drawFaultScreen() {
  u8g2.clearBuffer();
  u8g2.setDrawColor(1);
  u8g2.drawBox(0, 0, SCREEN_WIDTH, 15);
  u8g2.setDrawColor(0);
  u8g2.setCursor(4, 12);
  u8g2.print("!! FAULT !!");
  u8g2.setDrawColor(1);
  u8g2.setCursor(0, 30);
  u8g2.print(faultText(activeFault));
  u8g2.setCursor(0, 45);
  if (sensorOk) {
    u8g2.print("Temp: ");
    u8g2.print(currentTempC, 1);
    u8g2.print("°C");
  } else {
    u8g2.print("Temp: ---");
  }
  u8g2.setCursor(0, 61);
  u8g2.print("Press to reset");
  u8g2.sendBuffer();
}

void drawMainScreen() {
  u8g2.clearBuffer();
  u8g2.setDrawColor(1);
  u8g2.setCursor(0, 9);

  if (!sensorOk) {
    u8g2.print("TEMP ERROR!");
  } else if (isReflowRunning && setpointReached) {
    // Temperatura dostignuta: prikaz temperature + vreme od dostizanja (m:ss)
    u8g2.print(currentTempC, 1);
    u8g2.print("°C");
    unsigned long s = (millis() - reachedTime) / 1000;
    char buf[12];
    snprintf(buf, sizeof(buf), "T %lu:%02lu", s / 60, s % 60);
    u8g2.setCursor(SCREEN_WIDTH - u8g2.getStrWidth(buf), 9);
    u8g2.print(buf);
  } else {
    u8g2.print("Temp : ");
    u8g2.print(currentTempC);
    u8g2.print("°C");
  }
  u8g2.drawLine(0, 12, SCREEN_WIDTH - 1, 12);

  int y_pos_start = 16;
  int lineHeight = 14;
  int textYOffset = u8g2.getFontAscent() + 2;

  // SET
  u8g2.setDrawColor(1);
  if (menuIndex == 0) {
    u8g2.drawBox(0, y_pos_start, SCREEN_WIDTH, lineHeight);
    u8g2.setDrawColor(0);
  }
  u8g2.setCursor(0, y_pos_start + textYOffset);
  u8g2.print(" SET : ");
  u8g2.print(desiredTemperature);
  u8g2.print("°C");
  if (menuIndex == 0 && editingTemperature && (millis() % 1000 < 500)) {
    u8g2.print(" <<<");
  }

  // BEEP
  u8g2.setDrawColor(1);
  int y_pos_beep = y_pos_start + lineHeight + 2;
  if (menuIndex == 1) {
    u8g2.drawBox(0, y_pos_beep, SCREEN_WIDTH, lineHeight);
    u8g2.setDrawColor(0);
  }
  u8g2.setCursor(0, y_pos_beep + textYOffset);
  u8g2.print(" BEEP: ");
  u8g2.print(editingBeep ? "ON" : "OFF");

  // START/STOP
  u8g2.setDrawColor(1);
  int y_pos_start_stop = y_pos_start + (lineHeight + 2) * 2;
  if (menuIndex == 2) {
    u8g2.drawBox(0, y_pos_start_stop, SCREEN_WIDTH, lineHeight);
    u8g2.setDrawColor(0);
  }
  u8g2.setCursor(0, y_pos_start_stop + textYOffset);
  if (isReflowRunning) {
    u8g2.print(" STOP PROCESS");
    if (millis() % 1000 < 500) {
      u8g2.drawDisc(0, y_pos_start_stop + textYOffset - 4, 3);
    }
    float outputPercentage = (Output / WindowSize) * 100.0;
    u8g2.setCursor(u8g2.getCursorX() + 8, y_pos_start_stop + textYOffset);
    u8g2.print((int)outputPercentage);
    u8g2.print("%");
  } else {
    u8g2.print(sensorOk ? " START PROCESS" : " START BLOCKED");
  }
  u8g2.sendBuffer();
}

void updateDisplay() {
  if (millis() - lastDisplayTime < displayInterval) return;
  lastDisplayTime = millis();
  if (activeFault != FAULT_NONE) {
    drawFaultScreen();
  } else {
    drawMainScreen();
  }
}

// ===========================================================================
// Loop
// ===========================================================================
void loop() {
  if (millis() - lastTempReadTime >= tempReadInterval) {
    lastTempReadTime = millis();
    readTemperature();
  }

  handleEncoder();
  handleButton();
  driveHeater();
  logSerial();
  updateDisplay();
}
