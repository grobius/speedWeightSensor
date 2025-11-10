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
constexpr int X_OFFSET = 30;              // ~center for this board
constexpr int Y_OFFSET = 22;              // 12 is safer than 25 (avoids clipping)

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(
  U8G2_R0, U8X8_PIN_NONE, SCL_PIN, SDA_PIN
);

// ---------- BNO085 (SPI) ----------
constexpr uint8_t BNO_SCK  = 3;
constexpr uint8_t BNO_MOSI = 2;
constexpr uint8_t BNO_MISO = 4;
constexpr uint8_t BNO_CS   = 7;
constexpr uint8_t BNO_INT  = 10;  // DRDY/INT
constexpr uint8_t BNO_RST  = 1;   // optional reset

Adafruit_BNO08x bno08x(BNO_RST);
sh2_SensorValue_t sensorValue;

// Reports we want
sh2_SensorId_t reports[] = {
  SH2_LINEAR_ACCELERATION,  // m/s^2 (gravity-compensated, body frame)
  SH2_GRAVITY               // m/s^2 (points toward down, world frame)
};

// ---------- Speed / gating (Weightlifting: lift-only integration) ----------
float v_up   = 0.0f;     // integrated upward speed (m/s)
float v_last = 0.0f;     // last non-zero speed before STILL
float a_peak = 0.0f;     // peak |a| of current lift (m/s^2)
uint32_t t_prev_ms = 0;

// Tuning (good starting points for barbell work)
const float A_GATE       = 0.20f;     // noise floor for |a| when deciding STILL
const float A_LIFT       = 0.25f;     // only integrate when a_up > this (lift-only)
const uint32_t STILL_HOLD_MS = 120;   // time under A_GATE to confirm STILL
const float DAMPING      = 0.02f;     // mild decay against drift
const float V_CLAMP      = 6.0f;      // sanity cap for bar speed (m/s)

enum RunState { MOVING, STILL };
RunState state = STILL;
uint32_t gate_timer_ms = 0;

// latest samples
bool have_la = false, have_g = false;
float la_x=0, la_y=0, la_z=0;          // linear accel (body), m/s^2
float g_x=0,  g_y=0,  g_z=0;           // gravity (world), m/s^2

// ---------- helpers ----------
void enableReports() {
  for (uint8_t i = 0; i < sizeof(reports)/sizeof(reports[0]); i++) {
    // Try 200 Hz (5000 us). If not supported, library picks lower.
    if (!bno08x.enableReport(reports[i], 5000)) {
      bno08x.enableReport(reports[i], 10000); // 100 Hz fallback
    }
  }
}

void oledPrint3(const char* l1, const char* l2, const char* l3) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);

  // Three lines inside the 72x40 pane; baseline rows that fit well
  const int ascent     = u8g2.getAscent();        // ~8 px
  const int lineHeight = u8g2.getMaxCharHeight(); // ~10 px
  const int topY       = Y_OFFSET + 2 + ascent;   // small +2 nudges text down

  const int y1 = topY;                    // Speed
  const int y2 = topY + lineHeight;       // amax
  const int y3 = topY + 2 * lineHeight;   // STILL/MOVING

  int x1 = X_OFFSET + (PANEL_W - u8g2.getStrWidth(l1)) / 2;
  int x2 = X_OFFSET + (PANEL_W - u8g2.getStrWidth(l2)) / 2;
  int x3 = X_OFFSET + (PANEL_W - u8g2.getStrWidth(l3)) / 2;

  u8g2.setCursor(x1, y1); u8g2.print(l1);
  u8g2.setCursor(x2, y2); u8g2.print(l2);
  u8g2.setCursor(x3, y3); u8g2.print(l3);
  u8g2.sendBuffer();
}

void softResetBNO() {
  pinMode(BNO_RST, OUTPUT);
  digitalWrite(BNO_RST, LOW);  delay(2);
  digitalWrite(BNO_RST, HIGH); delay(10);
}

static inline float dot3(float ax, float ay, float az,
                         float bx, float by, float bz) {
  return ax*bx + ay*by + az*bz;
}

static inline float mag3(float x, float y, float z) {
  return sqrtf(x*x + y*y + z*z);
}

void setup() {
  // Serial
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}
  Serial.println(F("\nBNO08x (SPI) + Lift-only speed + OLED"));

  // OLED
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);                 // safer on this board
  u8g2.setI2CAddress(OLED_ADDR * 2);
  u8g2.begin();
  u8g2.setContrast(255);
  u8g2.setFont(u8g2_font_6x10_tf);
  oledPrint3("OLED OK", "Init IMU...", "");

  // SPI + BNO
  SPI.begin(BNO_SCK, BNO_MISO, BNO_MOSI, BNO_CS);
  pinMode(BNO_CS, OUTPUT); digitalWrite(BNO_CS, HIGH);
  softResetBNO();

  if (!bno08x.begin_SPI(BNO_CS, BNO_INT, &SPI)) {
    Serial.println(F("BNO08x: init FAILED (SPI)"));
    oledPrint3("IMU FAIL", "SPI init", "");
    while (true) delay(1000);
  }
  Serial.println(F("BNO08x: init OK (SPI)"));

  enableReports();
  t_prev_ms = millis();

  oledPrint3("0.000 m/s", "amax: 0.000", "STILL");
}

void loop() {
  if (bno08x.wasReset()) {
    Serial.println(F("BNO08x: wasReset, re-enabling reports"));
    enableReports();
  }

  // Gather a few events per loop
  have_la = false; have_g = false;
  const int MAX_EVENTS = 10;
  int ev = 0;

  while (ev < MAX_EVENTS && bno08x.getSensorEvent(&sensorValue)) {
    ev++;
    switch (sensorValue.sensorId) {
      case SH2_LINEAR_ACCELERATION:
        la_x = sensorValue.un.linearAcceleration.x;
        la_y = sensorValue.un.linearAcceleration.y;
        la_z = sensorValue.un.linearAcceleration.z;
        have_la = true;
        break;
      case SH2_GRAVITY:
        g_x = sensorValue.un.gravity.x;
        g_y = sensorValue.un.gravity.y;
        g_z = sensorValue.un.gravity.z;
        have_g = true;
        break;
      default: break;
    }
  }

  uint32_t now = millis();
  float dt = (now - t_prev_ms) / 1000.0f;
  if (dt <= 0) dt = 1e-3f;
  t_prev_ms = now;

  // Only proceed when we have both linear accel and gravity
  if (have_la && have_g) {
    // up_hat = -normalize(gravity)  (gravity points down)
    float gmag = mag3(g_x, g_y, g_z);
    float ux=0, uy=0, uz=0;
    if (gmag > 1e-3f) {
      ux = -g_x / gmag;  // unit vector UP (world)
      uy = -g_y / gmag;
      uz = -g_z / gmag;
    }

    // Project linear acceleration (body) into world-up direction.
    // NOTE: BNO’s linear accel is reported in the sensor’s *world* frame
    // when fusion is active with GRAVITY/rotation, so the projection with up_hat works.
    float a_up = dot3(la_x, la_y, la_z, ux, uy, uz);
    float a_mag = mag3(la_x, la_y, la_z);

    // Track peak acceleration during a lift run
    // Reset when a new MOVING phase begins
    static bool prev_moving = false;

    // Stationary detection based on |a| (noise floor)
    if (a_mag < A_GATE) {
      if (state == MOVING) {
        if (gate_timer_ms == 0) gate_timer_ms = now;
        if (now - gate_timer_ms >= STILL_HOLD_MS) {
          state = STILL;
          v_last = v_up;          // freeze last speed for display
          gate_timer_ms = 0;
        }
      } else {
        // keep v_up at v_last while STILL
        v_up = v_last;
      }
    } else {
      // We are moving
      if (state == STILL) {
        // transition: new lift starting
        state = MOVING;
        a_peak = 0.0f;           // reset peak accel for a new run
      }
      gate_timer_ms = 0;

      // Integrate ONLY if moving against gravity (lifting)
      if (a_up > A_LIFT) {
        v_up += a_up * dt;
        // light damping to reduce drift
        v_up *= (1.0f - DAMPING * dt);
        if (v_up < 0) v_up = 0;
        if (v_up > V_CLAMP) v_up = V_CLAMP;
      }

      // peak accel for the run
      if (a_mag > a_peak) a_peak = a_mag;
    }

    // Serial debug (compact)
    Serial.print(F("a=("));
    Serial.print(la_x,3); Serial.print(',');
    Serial.print(la_y,3); Serial.print(',');
    Serial.print(la_z,3); Serial.print(") ");
    Serial.print("|a|="); Serial.print(a_mag,3);
    Serial.print("  a_up="); Serial.print(a_up,3);
    Serial.print("  v_up="); Serial.print(v_up,3);
    Serial.print("  state="); Serial.print(state==STILL?"STILL":"MOVING");
    Serial.print("  amax="); Serial.println(a_peak,3);

    prev_moving = (state == MOVING);
  }

  // OLED ~10 Hz
  static uint32_t last_oled = 0;
  if (now - last_oled >= 100) {
    char line1[20], line2[24], line3[16];
    // During STILL, show frozen v_last; during MOVING show v_up
    snprintf(line1, sizeof(line1), "%.3f m/s", (state==STILL ? v_last : v_up));
    snprintf(line2, sizeof(line2), "amax: %.3f", a_peak);
    snprintf(line3, sizeof(line3), state==STILL ? "STILL" : "MOVING");
    oledPrint3(line1, line2, line3);
    last_oled = now;
  }
}
