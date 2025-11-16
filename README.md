// -- IDENTITY (🚨 don't commit real tokens/SSIDs) --
#define BLYNK_TEMPLATE_ID "TMPL3T0JOfLs_"
#define BLYNK_TEMPLATE_NAME "SmartGuard Home Automation"
#define BLYNK_AUTH_TOKEN "Q9-qayLQVk2Sg7PDE_0MSGAckDawX00H"

// -- LIBS --
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// -- LCD CONFIG --
#define LCD_ADDR 0x27
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

// -- PIN DEFINITIONS --
#define ACS_PIN     34
#define LED1_PIN    26
#define LED2_PIN    25
#define FAN_PIN     27
#define SW_LED1     13
#define SW_LED2     14
#define SW_FAN      12
#define BUZZER_PIN  33   // PWM-capable for LEDC tone

// -- WIFI + BLYNK --
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "rabbuni";
char pass[] = "44444444";

#define VPIN_FAN      V1
#define VPIN_LED1     V2
#define VPIN_VOLTAGE  V3
#define VPIN_LED2     V4

// -- CONSTANTS --
const float VREF = 3.3f;
const int   ADC_RES = 4095;
const float SCALE = 5.3391812865f;  // Your voodoo factor; left as-is
const float SAFE_VOLTAGE = 7.0f;

// -- STATES --
bool fanState  = false;
bool led1State = false;
bool led2State = false;

bool lastSwFan  = HIGH;
bool lastSwLed1 = HIGH;
bool lastSwLed2 = HIGH;

// Debounce
const uint16_t DEBOUNCE_MS = 40;
unsigned long lastDebounceFan  = 0;
unsigned long lastDebounceLed1 = 0;
unsigned long lastDebounceLed2 = 0;

// Smoothing
const uint8_t ADC_SAMPLES = 32;

// Blynk timer
BlynkTimer timer;

// -- BUZZER (LEDC tone wrapper) --
const int BUZZER_CHANNEL = 4;   // 0..15 valid
void buzzerTone(uint32_t freq) {
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
  ledcWriteTone(BUZZER_CHANNEL, freq);
}
void buzzerOff() {
  ledcWrite(BUZZER_CHANNEL, 0);
  ledcDetachPin(BUZZER_PIN);
}

//-- UTILS --
int readAdcAveraged(uint8_t pin, uint8_t samples = ADC_SAMPLES) {
  uint32_t acc = 0;
  for (uint8_t i = 0; i < samples; i++) {
    acc += analogRead(pin);
  }
  return (int)(acc / samples);
}

float computeBatteryVoltage(int raw) {
  float sensorV = (raw * VREF) / ADC_RES;
  float batteryV = sensorV * SCALE; // ⚠️ You’re treating current-sensor output as voltage; left intact
  if (batteryV < 0) batteryV = 0.0f;
  return batteryV;
}

void setRelaysFromState() {
  // Active-LOW relays: LOW = ON, HIGH = OFF
  digitalWrite(LED1_PIN, led1State ? LOW : HIGH);
  digitalWrite(LED2_PIN, led2State ? LOW : HIGH);
  digitalWrite(FAN_PIN,  fanState  ? LOW : HIGH);
}

void syncAppFromState() {
  // Your UI expects inverted writes
  Blynk.virtualWrite(VPIN_LED1, !led1State);
  Blynk.virtualWrite(VPIN_LED2, !led2State);
  Blynk.virtualWrite(VPIN_FAN,  !fanState);
}

// -- BLYNK HOOKS --
BLYNK_CONNECTED() {
  // Push hardware state to app on connect
  syncAppFromState();
}

BLYNK_WRITE(VPIN_LED1) {
  bool buttonState = param.asInt();      // 0 = ON, 1 = OFF (you inverted)
  led1State = !buttonState;
  digitalWrite(LED1_PIN, led1State ? LOW : HIGH);
  Blynk.virtualWrite(VPIN_LED1, !led1State);
  Serial.printf("[APP] LED1 %s\n", led1State ? "ON" : "OFF");
}

BLYNK_WRITE(VPIN_LED2) {
  bool buttonState = param.asInt();
  led2State = !buttonState;
  digitalWrite(LED2_PIN, led2State ? LOW : HIGH);
  Blynk.virtualWrite(VPIN_LED2, !led2State);
  Serial.printf("[APP] LED2 %s\n", led2State ? "ON" : "OFF");
}

BLYNK_WRITE(VPIN_FAN) {
  bool buttonState = param.asInt();
  fanState = !buttonState;
  digitalWrite(FAN_PIN, fanState ? LOW : HIGH);
  Blynk.virtualWrite(VPIN_FAN, !fanState);
  Serial.printf("[APP] Fan %s\n", fanState ? "ON" : "OFF");
}

// -- TASKS --
void taskVoltage() {
  int raw = readAdcAveraged(ACS_PIN);
  float batt = computeBatteryVoltage(raw);

  // LCD (rate-limited: this task runs every 300 ms)
  lcd.setCursor(0, 0);
  lcd.print("Voltage:        ");
  lcd.setCursor(0, 1);
  lcd.print(batt, 2);
  lcd.print(" V      ");

  Serial.printf("RAW: %d | BatteryV: %.3f\n", raw, batt);
  Blynk.virtualWrite(VPIN_VOLTAGE, batt);

  // High voltage action
  if (batt >= SAFE_VOLTAGE && (fanState || led1State || led2State)) {
    fanState = false;
    led1State = false;
    led2State = false;
    setRelaysFromState();
    syncAppFromState();

    // Non-blocking beep sequence (short)
    buzzerTone(1500);
    timer.setTimeout(300, [] { buzzerOff(); });

    Serial.println("[ALERT] Voltage Too High! Devices turned OFF!");
    Blynk.logEvent("voltage_alert", "⚠️ Voltage too high! Devices turned OFF!");
  }
}

void taskSwitches() {
  unsigned long now = millis();

  // LED1
  bool swLed1 = digitalRead(SW_LED1);
  if (swLed1 != lastSwLed1 && (now - lastDebounceLed1) > DEBOUNCE_MS) {
    lastDebounceLed1 = now;
    lastSwLed1 = swLed1;
    if (swLed1 == LOW) { // active on press
      led1State = !led1State;
      digitalWrite(LED1_PIN, led1State ? LOW : HIGH);
      Blynk.virtualWrite(VPIN_LED1, !led1State);
      Serial.printf("[MANUAL] LED1 %s\n", led1State ? "ON" : "OFF");
    }
  }

  // LED2
  bool swLed2 = digitalRead(SW_LED2);
  if (swLed2 != lastSwLed2 && (now - lastDebounceLed2) > DEBOUNCE_MS) {
    lastDebounceLed2 = now;
    lastSwLed2 = swLed2;
    if (swLed2 == LOW) {
      led2State = !led2State;
      digitalWrite(LED2_PIN, led2State ? LOW : HIGH);
      Blynk.virtualWrite(VPIN_LED2, !led2State);
      Serial.printf("[MANUAL] LED2 %s\n", led2State ? "ON" : "OFF");
    }
  }

  // FAN
  bool swFan = digitalRead(SW_FAN);
  if (swFan != lastSwFan && (now - lastDebounceFan) > DEBOUNCE_MS) {
    lastDebounceFan = now;
    lastSwFan = swFan;
    if (swFan == LOW) {
      fanState = !fanState;
      digitalWrite(FAN_PIN, fanState ? LOW : HIGH);
      Blynk.virtualWrite(VPIN_FAN, !fanState);
      Serial.printf("[MANUAL] Fan %s\n", fanState ? "ON" : "OFF");
    }
  }
}

// -- SETUP --
void setup() {
  Serial.begin(115200);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SmartGuard Init");

  // IO
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(SW_LED1, INPUT_PULLUP);
  pinMode(SW_LED2, INPUT_PULLUP);
  pinMode(SW_FAN,  INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);

  // Set initial relay OFF (Active-LOW → HIGH)
  fanState = led1State = led2State = false;
  setRelaysFromState();
  buzzerOff();

  // Boot beep (non-blocking)
  buzzerTone(1000);
  timer.setTimeout(200, [] { buzzerOff(); });

  // ADC config
  analogSetPinAttenuation(ACS_PIN, ADC_11db); // OK for up to ~3.3V input at pin

  // WiFi + Blynk
  WiFi.begin(ssid, pass);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(250);
  }
  Serial.println("\nWiFi Connected");
  Blynk.begin(auth, ssid, pass); // Will block until connected to Blynk cloud

  // Timers
  timer.setInterval(300L, taskVoltage);  // ~3 Hz updates; stable LCD/Blynk
  timer.setInterval(20L,  taskSwitches); // fast poll with debounce
}

// -- LOOP --
void loop() {
  Blynk.run();
  timer.run();
}
