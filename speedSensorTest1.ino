#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <Adafruit_BNO08x.h>

// ---------- OLED (I2C) ----------
constexpr uint8_t SDA_PIN   = 5;          // OLED SDA (wired on this board)
constexpr uint8_t SCL_PIN   = 6;          // OLED SCL (wired on this board)
constexpr uint8_t OLED_ADDR = 0x3C;

// Visible 72x40 window centered in SSD1306 128x64 buffer
constexpr int PANEL_W  = 72;
constexpr int PANEL_H  = 40;
constexpr int X_OFFSET = 30;
constexpr int Y_OFFSET = 25;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(
  U8G2_R0, U8X8_PIN_NONE, SCL_PIN, SDA_PIN
);

// ---------- BNO085 (SPI) ----------
constexpr uint8_t BNO_SCK = 3;
constexpr uint8_t BNO_MOSI = 2;
constexpr uint8_t BNO_MISO = 4;
constexpr uint8_t BNO_CS   = 7;
constexpr uint8_t BNO_INT  = 10;   // DRDY/INT
constexpr uint8_t BNO_RST  = 1;    // optional reset

Adafruit_BNO08x bno08x(BNO_RST);
sh2_SensorValue_t sensorValue;

// Reports we want
sh2_SensorId_t reports[] = {
  SH2_LINEAR_ACCELERATION  // m/s^2 (gravity-compensated)
};

// ---------- Speed / gating (Weightlifting optimized) ----------
float v_mag = 0.0f;
float v_last = 0.0f;
float amax   = 0.0f;
uint32_t t_prev_ms = 0;

// Best values for barbell speed & acceleration tracking:
const float A_GATE  = 0.30f;        // movement threshold (m/s^2)
const uint32_t STILL_HOLD_MS = 120; // time needed to confirm bar is still
const float DAMPING = 0.03f;        // drift control while integrating
const float V_CLAMP = 6.0f;         // highest realistic bar speed (m/s)


enum RunState { MOVING, STILL };
RunState state = STILL;
uint32_t gate_timer_ms = 0;

// ---------- helpers ----------
void enableReports() {
  for (uint8_t i = 0; i < sizeof(reports)/sizeof(reports[0]); i++) {
    // try 200 Hz; if not supported, library will choose lower
    if (!bno08x.enableReport(reports[i], 5000)) {
      bno08x.enableReport(reports[i], 10000); // 100 Hz fallback
    }
  }
}

void oledPrint3(const String& l1, const String& l2, const String& l3) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);

  // Tweaked layout that matches your board's actual OLED mapping
  const int ascent     = u8g2.getAscent();         // ~8 px
  const int lineHeight = u8g2.getMaxCharHeight();  // ~10 px

  // Adjusted top-of-text inside the 72x40 window
  // (Your display begins ~3 px lower than standard)
  const int topY = Y_OFFSET + 3 + ascent;

  const int y1 = topY;                 // Speed
  const int y2 = topY + lineHeight;    // amax
  const int y3 = topY + 2*lineHeight;  // STILL/MOVING

  // Center horizontally
  int x1 = X_OFFSET + (PANEL_W - u8g2.getStrWidth(l1.c_str()))/2;
  int x2 = X_OFFSET + (PANEL_W - u8g2.getStrWidth(l2.c_str()))/2;
  int x3 = X_OFFSET + (PANEL_W - u8g2.getStrWidth(l3.c_str()))/2;

  u8g2.setCursor(x1, y1); u8g2.print(l1);
  u8g2.setCursor(x2, y2); u8g2.print(l2);
  u8g2.setCursor(x3, y3); u8g2.print(l3);

  u8g2.sendBuffer();
}



void softResetBNO() {
  // optional gentle reset pulse
  pinMode(BNO_RST, OUTPUT);
  digitalWrite(BNO_RST, LOW); delay(2);
  digitalWrite(BNO_RST, HIGH); delay(10);
}

void setup() {
  // --- Serial ---
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}
  Serial.println(F("\nBNO08x (SPI) + speed-any-direction + OLED gate"));

  // --- OLED I2C ---
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // safer on this board
  u8g2.setI2CAddress(OLED_ADDR * 2);
  u8g2.begin();
  u8g2.setContrast(255);
  u8g2.setFont(u8g2_font_6x10_tf);
  oledPrint3("OLED OK", "Init IMU...", "");

  // --- SPI + BNO ---
  SPI.begin(BNO_SCK, BNO_MISO, BNO_MOSI, BNO_CS);

  // Let library own INT; no pullups here
  pinMode(BNO_CS, OUTPUT);
  digitalWrite(BNO_CS, HIGH);
  softResetBNO();

  if (!bno08x.begin_SPI(BNO_CS, BNO_INT, &SPI)) {
    Serial.println(F("BNO08x: init FAILED (SPI)"));
    oledPrint3("IMU FAIL", "SPI init", "");
    while (true) delay(1000);
  }
  Serial.println(F("BNO08x: init OK (SPI)"));
  oledPrint3("IMU OK", "Config reports", "");

  enableReports();
  t_prev_ms = millis();

  // Initial screen
  oledPrint3("0.000 m/s", "STILL", "amax: 0.000 m/s^2");
}

void loop() {
  // Check IMU reset
  if (bno08x.wasReset()) {
    Serial.println(F("BNO08x: wasReset, re-enabling reports"));
    enableReports();
  }

  // Non-blocking: read up to N events per loop
  const int MAX_EVENTS = 8;
  int ev_count = 0;
  bool got_accel = false;
  float ax=0, ay=0, az=0;

  while (ev_count < MAX_EVENTS && bno08x.getSensorEvent(&sensorValue)) {
    ev_count++;
    if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
      ax = sensorValue.un.linearAcceleration.x;
      ay = sensorValue.un.linearAcceleration.y;
      az = sensorValue.un.linearAcceleration.z;
      got_accel = true;
    }
  }

  uint32_t now = millis();
  float dt = (now - t_prev_ms) / 1000.0f;
  if (dt <= 0) dt = 1e-3;
  t_prev_ms = now;

  static float a_peak = 0.0f;

  if (got_accel) {
    float amag = sqrtf(ax*ax + ay*ay + az*az); // m/s^2
    if (amag > a_peak) a_peak = amag;

    // Stationary gate logic
    if (amag < A_GATE) {
      // candidate STILL: run a short hold timer
      if (state == MOVING) {
        if (gate_timer_ms == 0) gate_timer_ms = now;
        if (now - gate_timer_ms >= STILL_HOLD_MS) {
          // transition to STILL: freeze last v and keep amax shown
          state = STILL;
          v_last = v_mag;
          gate_timer_ms = 0;
        }
      }
      // when STILL, decay speed to exactly the frozen value
      if (state == STILL) {
        v_mag = v_last;
      }
    } else {
      // MOVING: clear gate, integrate
      gate_timer_ms = 0;
      state = MOVING;

      // integrate speed magnitude crudely using |a| as scalar increment
      // (note: true speed vector integration needs orientation; this is a quick demo)
      v_mag += amag * dt;
      v_mag *= (1.0f - DAMPING * dt);
      if (v_mag > V_CLAMP) v_mag = V_CLAMP;
      if (v_mag < 0) v_mag = 0;
    }

    // Serial debug
    Serial.print(F("a=("));
    Serial.print(ax,3); Serial.print(',');
    Serial.print(ay,3); Serial.print(',');
    Serial.print(az,3); Serial.print(") |a|=");
    Serial.print(sqrtf(ax*ax+ay*ay+az*az),3);
    Serial.print("  v|="); Serial.print(v_mag,3);
    Serial.print("  state="); Serial.print(state==STILL?"STILL":"MOVING");
    Serial.print("  amax="); Serial.println(a_peak,3);
  }

  // Update OLED at ~10 Hz
  static uint32_t last_oled = 0;
  if (now - last_oled >= 100) {
    char line1[20], line2[34], line3[16];
    snprintf(line1, sizeof(line1), "%.3fm/s", (state==STILL? v_last : v_mag));
    snprintf(line2, sizeof(line2), "a:%.3fm/s^2", a_peak);
    snprintf(line3, sizeof(line3), state==STILL ? "STILL" : "MOVING");
    oledPrint3(line1, line2, line3);
    last_oled = now;
  }
}
