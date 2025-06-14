#include <Wire.h>
#include <U8g2lib.h>
#include <max6675.h>
#include <RotaryEncoder.h>
#include <mera_logo.h>
#include <PID_v1.h>

// OLED definicije
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#define SCREEN_WIDTH 128

// MAX6675 definicije
const int MAX6675_SCK = 18;
const int MAX6675_CS = 5;
const int MAX6675_SO = 19;
MAX6675 thermocouple(MAX6675_SCK, MAX6675_CS, MAX6675_SO);

// Rotary Encoder definicije
#define ENCODER_CLK_PIN 25
#define ENCODER_DT_PIN 26
#define ENCODER_SW_PIN 27
RotaryEncoder encoder(ENCODER_CLK_PIN, ENCODER_DT_PIN, RotaryEncoder::LatchMode::FOUR3);

// Fleg za signalizaciju promene enkodera
volatile bool encoderChanged = false; // 'volatile' je vazno za promenljive koje menja ISR

// Definicija PIN-a za buzzer
const int BUZZER_PIN = 2; // GPIO 2

// Definicija PIN-a za SSR
const int SSR_PIN = 23; // GPIO 23 pin na ESP32

// Definicija PIN-a za LED indikator PID Outputa
const int PID_LED_PIN = 32; // GPIO 32 (ili neki drugi slobodan pin na ESP32)
const float hysteresisTemp = 2.0; // Prag (u C) ispod Setpointa gde se LED gasi

// PID Gain Scheduling - zadane PID varijable 
const double KI_LOW_TEMP  = 0.025; // Ki za 130C - 150C
const double KI_MID_TEMP  = 0.04;  // Ki za 150C - 180C 
const double KI_HIGH_TEMP = 0.06;  // Ki za 200C i vise

// Odrzavamo Kp i Kd konstantnim. Tjuningom ustanovljenje vrednosti
const double FIXED_KP = 18.0;
const double FIXED_KD = 1.0;

// Deklaracija _current varijabli za PID
double Kp_current = FIXED_KP;
double Ki_current = KI_LOW_TEMP;
double Kd_current = FIXED_KD;

double Input;    // Stvarna temperatura (trenutnaTempC)
double Output;   // Izlaz PID-a (snaga grejača, 0-255 ili 0-100%)
double Setpoint; // Željena temperatura (desiredTemperature)

// Objekat PID kontrolera
PID myPID(&Input, &Output, &Setpoint, Kp_current, Ki_current, Kd_current, DIRECT);

// Period za PWM kontrolu grejača (npr. 2 sekunde)
const int WindowSize = 2000;
unsigned long windowStartTime;

// Meni promenljive
int menuIndex = 0;
const int totalMenuItems = 3; // SET, SOUND, START/STOP
int desiredTemperature = 160;
bool editingTemperature = false;
bool editingBeep = true; // Default value (false for OFF, true for ON)
bool isReflowRunning = false; // prati da li je proces u toku
unsigned long lastButtonPressTime = 0;
const unsigned long debounceDelay = 250;

// Promenljive za kontrolu učestalosti čitanja temperature
unsigned long lastTempReadTime = 0;
const unsigned long tempReadInterval = 250; // Minimalno 200-250 ms između čitanja
float currentTempC = 0.0; // Inicijalizovana globalna promenljiva za temperaturu

// Funkcija za novi update za PID Ki element
void updatePIDTunings() {
  double newKi = 0.0;

  if (desiredTemperature <= 160) {
    newKi = KI_LOW_TEMP;
  } else if (desiredTemperature > 160 && desiredTemperature <= 180) {
    newKi = KI_MID_TEMP;
  } else {
    newKi = KI_HIGH_TEMP;
  }

  // Proveravamo da li se Ki promenio pre nego sto pozovem SetTunings
  if (newKi != Ki_current) {
    Ki_current = newKi; // Ažuriraj globalnu promenljivu
    myPID.SetTunings(Kp_current, Ki_current, Kd_current);
    Serial.print("PID tunings updated: Kp="); Serial.print(Kp_current, 3);
    Serial.print(", Ki="); Serial.print(Ki_current, 3);
    Serial.print(", Kd="); Serial.println(Kd_current, 3);
  }
}

// Funkcija za interrupt
void IRAM_ATTR readEncoderISR() {
  encoder.tick();
  encoderChanged = true; // Postavi fleg
}

void setup() {
  Serial.begin(115200);
  // Inicijalizacija OLED displeja
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.clearBuffer();
  u8g2.drawBitmap( 0, 0, 128/8, 64, myLogo);
  u8g2.sendBuffer();

  // Inicijalizacija MAX6675
  double tempTest = thermocouple.readCelsius();

  u8g2.setFont(u8g2_font_7x13_tf);
  if (isnan(tempTest) || tempTest == -1000.0) {
    Serial.println("Greska pri inicijalizaciji MAX6675 ili termopar nije povezan!");
    u8g2.clearBuffer();
    u8g2.setCursor(0, 10);
    u8g2.println("MAX6675 ERROR!");
    u8g2.println("Check Thermocouple!");
    // Ovde mozda staviti neprekidni uslov if(;;)
  } else {
    Serial.println("MAX6675 initialized and working.");
    u8g2.println("MAX6675 OK!");
  }
  u8g2.sendBuffer();
  delay(3000);

  // Inicijalizacija pina za buzzer
  pinMode(BUZZER_PIN, OUTPUT);

  // Zvučni signal pri pokretanju uređaja
  tone(BUZZER_PIN, 4000, 50); noTone(BUZZER_PIN);
  delay(100);
  tone(BUZZER_PIN, 4000, 50); noTone(BUZZER_PIN);

  // Inicijalizacija pina za SSR
  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW); // Osiguravamo da je grejač isključen na početku

  // Inicijalizacija PID-a
  windowStartTime = millis(); // Postavi početak prvog PWM prozora
  myPID.SetMode(AUTOMATIC);   // Postavi PID na automatski rad
  myPID.SetOutputLimits(0, WindowSize); // Izlaz PID-a će biti vreme uključenja unutar WindowSize
  // (npr. 0 do 2000 ms, što znači 0-100% snage)

  updatePIDTunings(); // Postavi pocetni Ki baziran na desiredTemperature

  // Inicijalni ispis PID parametara ---
  Serial.print("Initial PID tunings: Kp="); Serial.print(Kp_current, 3);
  Serial.print(", Ki="); Serial.print(Ki_current, 3);
  Serial.print(", Kd="); Serial.println(Kd_current, 3);

  // Inicijalizacija pina za LED indikator PID Outputa
  pinMode(PID_LED_PIN, OUTPUT);
  digitalWrite(PID_LED_PIN, LOW); // LED isključena na početku

  // Inicijalizacija Rotary Encodera
  // pinMode(ENCODER_SW_PIN, INPUT_PULLUP); // Zakomentarisano zbog eksternih pull-up otpornika na ploči
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK_PIN), readEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_DT_PIN), readEncoderISR, CHANGE);
}

void loop() {

  // Kontrolisano očitavanje temperature sa ne-blokirajućim kašnjenjem
  if (millis() - lastTempReadTime > tempReadInterval) {
    currentTempC = thermocouple.readCelsius();
    lastTempReadTime = millis(); // Ažuriraj vreme poslednjeg očitavanja

    // Serijski ispis temperature samo kada je nova vrednost očitana
    if (isnan(currentTempC) || currentTempC == -1000.0) {
        Serial.println("Greska: TEMP ERROR!");
    } else {
        Serial.print("Temperatura: ");
        Serial.print(currentTempC);
        Serial.println(" °C");
    }
  }

  if (encoderChanged) { // Proveri da li se enkoder pomerao
    encoderChanged = false; // Resetuj fleg odmah
    int newPosition = encoder.getPosition();
    static int oldPosition = 0; // Mora biti static da bi zadrzala vrednost izmedju poziva
    
    if (newPosition != oldPosition) {
      int direction = newPosition - oldPosition;

      // Zvučni signal pri promeni temperature (ako je BEEP ON)
      if (editingBeep) {
        tone(BUZZER_PIN, 4000, 50); // 4000Hz kratak i tiši beep za svaku promenu
        noTone(BUZZER_PIN);
      }
      
      if (editingTemperature) {
        desiredTemperature += direction;
        bool limitReached = false; // Flega za limit
        if (desiredTemperature < 50.0) {
          desiredTemperature = 50.0;
          limitReached = true;
        }
        if (desiredTemperature > 250.0) {
          desiredTemperature = 250.0;
          limitReached = true;
        }
        // Zvučni signal za dostizanje limita (ako je BEEP ON)
        if (editingBeep && limitReached) {
          tone(BUZZER_PIN, 4000, 200); // Dva beepa
          delay(100);
          tone(BUZZER_PIN, 4000, 200);
          noTone(BUZZER_PIN);
        }
      }
      else { // Navigacija menijem
        menuIndex = (menuIndex + direction + totalMenuItems) % totalMenuItems;
        // Resetuje se samo editingTemperature kada se menja stavka menija
        editingTemperature = false;
      }
      oldPosition = newPosition;
    }
  }
  // Citanje button-a enkodera
  bool buttonPressed = (digitalRead(ENCODER_SW_PIN) == LOW);

  if (buttonPressed && (millis() - lastButtonPressTime > debounceDelay)) {
    lastButtonPressTime = millis();

    if (menuIndex == 0) { // SET (Target Temperature)
      editingTemperature = !editingTemperature;
      Serial.print("Editing temperature: ");
      Serial.println(editingTemperature ? "ON" : "OFF");
      
      // Ako smo upravo izašli iz moda za uređivanje temperature
      if (!editingTemperature) { 
        updatePIDTunings(); // POZOVI FUNKCIJU ZA AŽURIRANJE PID KOEFICIJENATA
      }

      // Zvučna potvrda za SET aktivno
      if (editingBeep) { // Ako je BEEP sada ON, pusti zvuk
        tone(BUZZER_PIN, 4000, 50); // Ton za potvrdu ON
        noTone(BUZZER_PIN);
      }// Kada je BEEP setovan na OFF, ne puštamo zvuk

    } else if (menuIndex == 1) { // BEEP (Sound)
      editingBeep = !editingBeep; // Prebacuje ON/OFF
      Serial.print("BEEP status: ");
      Serial.println(editingBeep ? "ON" : "OFF");
      // Zvučna potvrda za promenu BEEP stanja
      if (editingBeep) { // Ako je BEEP sada ON, pusti zvuk
        tone(BUZZER_PIN, 4000, 75); // Ton za potvrdu ON
        noTone(BUZZER_PIN);
      }// Kada je BEEP setovan na OFF, ne puštamo zvuk po tvojoj želji
      editingTemperature = false;

    } else if (menuIndex == 2) { // START/STOP Process
      isReflowRunning = !isReflowRunning; // Prebaci stanje: ako radi, zaustavi; ako ne radi, pokreni
      if (isReflowRunning) {
        Serial.println("Reflow process STARTED!");
        
        // Zvuk za START
        if (editingBeep) {
          tone(BUZZER_PIN, 4000, 70);
          delay(100);
          tone(BUZZER_PIN, 4000, 200);
          delay(100);
          noTone(BUZZER_PIN);
        }

      } else {
        Serial.println("Reflow process STOPPED!");
        
        // Zvuk za STOP
        if (editingBeep) {
          tone(BUZZER_PIN, 4000, 200); // Viši ton
          delay(100);
          tone(BUZZER_PIN, 4000, 70); // Niži ton
          delay(100);
          noTone(BUZZER_PIN);
        }
  
      }
      // Resetuje se samo editingTemperature
      editingTemperature = false;
    }
  }

  //Poveži Input i Setpoint sa globalnim varijablama
  Input = currentTempC;
  Setpoint = desiredTemperature;

  // PID kontrola samo ako je reflow proces u toku
  if (isReflowRunning) {
    myPID.Compute(); // Izračunaj PID izlaz

    // Time Proportional Control za SSR
    if (millis() - windowStartTime > WindowSize) { // Ako je prošao jedan PWM prozor
      windowStartTime += WindowSize; // Pomeri početak prozora za sledeći ciklus
    }
    if (Output < (millis() - windowStartTime)) { // Ako je vreme uključenja prošlo unutar prozora
      digitalWrite(SSR_PIN, LOW); // ISKLJUČI grejač
    } else {
      digitalWrite(SSR_PIN, HIGH); // UKLJUČI grejač
    }
    // Serijski ispis PID Outputa
    Serial.print("PID Output: ");
    Serial.println(Output);

    // LED svetli kada je proces aktivan I temperatura je značajno ispod Setpointa.
    // Gasi se kada je temperatura blizu ili iznad Setpointa.
    if (currentTempC < (Setpoint - hysteresisTemp)) {
      // Temperatura je značajno ispod Setpointa, aktivno se zagreva
      digitalWrite(PID_LED_PIN, HIGH); // Uključi LED
    } else {
      // Temperatura je blizu ili iznad Setpointa, ili se održava
      digitalWrite(PID_LED_PIN, LOW);  // Isključi LED
    }

  } else {
    // Proces nije aktivan, osiguraj da je grejač isključen
    digitalWrite(SSR_PIN, LOW);
    digitalWrite(PID_LED_PIN, LOW); // LED je ugašena kada proces ne radi
  }
  u8g2.clearBuffer();
  
  u8g2.setDrawColor(1);
  u8g2.setCursor(0, 9);

  if (isnan(currentTempC) || currentTempC == -1000.0) {
    u8g2.println("TEMP ERROR!");
  } else {
    u8g2.print("Temp : ");
    u8g2.print(currentTempC);
    u8g2.println("°C");
    Serial.print("PID Output: ");
    Serial.println(Output);
  }
  u8g2.drawLine(0, 12, SCREEN_WIDTH - 1, 12);

  // Prikaz menija
  int y_pos_start = 16;
  int lineHeight = 14; // Visina box-a za selekciju
  int textYOffset = u8g2.getFontAscent() + 2; // Pozicija teksta unutar box-a

  // Stavka 1: SET (Target Temperature)
  u8g2.setDrawColor(1);
  if (menuIndex == 0) {
    u8g2.drawBox(0, y_pos_start, SCREEN_WIDTH, lineHeight);
    u8g2.setDrawColor(0);
  }
  u8g2.setCursor(0, y_pos_start + textYOffset);
  u8g2.print(" SET : ");
  u8g2.print(desiredTemperature);
  u8g2.print("°C");
  if (menuIndex == 0 && editingTemperature) {
    if (millis() % 1000 < 500) {
      u8g2.print(" <<<");
    } else{
      u8g2.print("    ");
    }
  }
  u8g2.println("");

  // Stavka 2: BEEP (Sound)
  u8g2.setDrawColor(1);
  int y_pos_beep = y_pos_start + lineHeight + 2; // +2 za mali razmak
  if (menuIndex == 1) {
    u8g2.drawBox(0, y_pos_beep, SCREEN_WIDTH, lineHeight);
    u8g2.setDrawColor(0);
  }
  u8g2.setCursor(0, y_pos_beep + textYOffset);
  u8g2.print(" BEEP: ");
  u8g2.print(editingBeep ? "ON" : "OFF");
  u8g2.println("");

  // Stavka 3: START/STOP
  u8g2.setDrawColor(1);
  int y_pos_start_stop = y_pos_start + (lineHeight + 2) * 2;
  if (menuIndex == 2) {
    u8g2.drawBox(0, y_pos_start_stop, SCREEN_WIDTH, lineHeight);
    u8g2.setDrawColor(0);
  }
  u8g2.setCursor(0, y_pos_start_stop + textYOffset);
  // Prikaz "START" ili "STOP" u zavisnosti od stanja isReflowRunning
  if (isReflowRunning) {
    u8g2.print(" STOP PROCESS"); // Kada proces radi, dugme ga stopira
    
    // Dodavanje trepereće indikacije
    if (millis() % 1000 < 500) {
      u8g2.drawDisc(0, y_pos_start_stop + textYOffset - 4, 3); // Mala tačka
    }
    
    // Prikaz PID Outputa u procentima
    // Računamo procenat Outputa
    float outputPercentage = (Output / WindowSize) * 100.0;
    
    // Prebacujemo boju nazad na belu za procenat ako je linija menija selektovana
    if (menuIndex == 2) {
        u8g2.setDrawColor(0); // Ako je selektovano, ispiši crno na beloj pozadini
    } else {
        u8g2.setDrawColor(1); // Inače, ispiši belo na crnoj pozadini
    }
    
    // Pomeramo kursor desno da bi bilo mesta za "STOP PROCESS" i indikator
    u8g2.setCursor(u8g2.getCursorX() + 8, y_pos_start_stop + textYOffset); // Podesi X poziciju
    u8g2.print((int)outputPercentage); // Prikazi kao int (bez decimala)
    u8g2.print("%"); 
    u8g2.println(""); // Pređi u novi red
  } else {
    u8g2.println(" START PROCESS");
  }
  u8g2.sendBuffer();
}